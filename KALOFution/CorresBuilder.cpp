#include <limits>
#include <numeric>
#include <cfloat>
#include <cmath>
#include <fstream>

#include "CorresBuilder.h"
#include "DataProvider.h"

#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

using namespace std;

float CorresBuilder::volumeOverlapRatio(const Eigen::Affine3f& trans)
{
    const int res = 20;
    double ul = 1. / res;
    int s = 0;
    for (int i = 0; i < res; i++) {
        for (int j = 0; j < res; j++) {
            for (int k = 0; k < res; k++) {
                Eigen::Vector3f pos((i + 0.5) * ul, (j + 0.5) * ul, (k + 0.5) * ul);
                Eigen::Vector3f ppos = trans * pos;
                if (ppos(0) >= 0 && ppos(0) <= 1. &&
                    ppos(1) >= 0 && ppos(1) <= 1. &&
                    ppos(2) >= 0 && ppos(2) <= 1.) {
                    s++;
                }
            }
        }
    }

    return (double)s / res / res / res;
}

void CorresBuilder::operator()(DataProvider& provider)
{
    assert(provider.size() >= 2);
    m_cloudPairs.clear();
    m_initCloudTransform.clear();
    m_pointClouds.clear();

    PCL_INFO("Start build correspondence...\n");

    initCloudAndTransform(provider);
    initCloudPair();
    if (m_params.cloudPairNeedAlignment)
        alignPairs();
    findCorresPoints();
}

void CorresBuilder::initCloudAndTransform(DataProvider& provider)
{
    m_pointClouds.resize(provider.size());

    for (uint32_t i = 0; i < provider.size(); ++i)
    {
        CloudTypePtr rawCloud = provider[i];
        CloudTypePtr cloud(new CloudType);
        for (int j = 0; j < rawCloud->points.size(); ++j)
        {
            if (!_isnan(rawCloud->points[j].normal_x)) {
                cloud->push_back(rawCloud->points[j]);
            }
        }
        m_pointClouds[i] = cloud;
    }

    m_initCloudTransform.resize(provider.size());

    //CloudTransform base = provider.initTransformOfCloudAtIndex(0).inverse();
    for (uint32_t i = 0; i < provider.size(); ++i)
    {
        CloudTransform trans = provider.initTransformOfCloudAtIndex(i);
        m_initCloudTransform[i] = trans;
    }
}

void CorresBuilder::initCloudPair()
{
    uint32_t size = m_pointClouds.size();
    for (uint32_t i = 0; i < size - 1; ++i)
    {
        m_cloudPairs.push_back(CloudPair(i, i + 1, m_initCloudTransform[i].inverse() * m_initCloudTransform[i + 1]));
        for (uint32_t j = i + 2; j < size; ++j)
        {
            CloudTransform trans = m_initCloudTransform[i].inverse() * m_initCloudTransform[j];
            if (volumeOverlapRatio(trans) > 0.3f) {
                m_cloudPairs.push_back(CloudPair(i, j, trans));
            }
        }
    }
    PCL_WARN("%d cloud pairs\n", m_cloudPairs.size());
}

void CorresBuilder::alignPairs()
{
    for (size_t i = 0; i < m_cloudPairs.size(); ++i)
    {
        const int K = 1;
        pcl::KdTreeFLANN<PointType> kdtree;
        std::vector<int> pointIndices(K);
        std::vector<float> pointDistances(K);

        PCL_INFO("Align pair <%d, %d>\n", m_cloudPairs[i].corresIdx.first, m_cloudPairs[i].corresIdx.second);

        CloudTypePtr p0 = m_pointClouds[m_cloudPairs[i].corresIdx.first];
        CloudTypePtr p1 = m_pointClouds[m_cloudPairs[i].corresIdx.second];

        CloudTypePtr transformed(new CloudType);
        pcl::transformPointCloudWithNormals(*p1, *transformed, m_cloudPairs[i].relativeTrans);

        kdtree.setInputCloud(p0);
        size_t count = 0;
        for (size_t n = 0; n < transformed->size(); ++n) {
            if (kdtree.nearestKSearch(transformed->points[n], K, pointIndices, pointDistances) > 0) {
                if (pointDistances[0] < m_params.acceptableCorresPointDistThres * m_params.acceptableCorresPointDistThres) {
                    ++count;
                }
            }
        }

        bool accept = count >= m_params.acceptableCorresPointNum || (1. * count / p0->size() && 1. * count / p1->size());
        if (!accept) {
            PCL_INFO("reject\n");
            m_cloudPairs[i].corresIdx = std::make_pair(-1, -1);
            continue;
        }

        PCL_INFO("accept\n");
        m_cloudPairs[i].validCorresPointNumber = count;

        pcl::IterativeClosestPoint<PointType, PointType> icp;
        typedef pcl::registration::TransformationEstimationPointToPlaneLLS<PointType, PointType> P2PlaneEstimation;
        boost::shared_ptr<P2PlaneEstimation> point_to_plane(new P2PlaneEstimation);

        icp.setInputCloud(p1);
        icp.setInputTarget(p0);
        icp.setMaxCorrespondenceDistance(m_params.acceptableCorresPointDistThres);
        icp.setMaximumIterations(m_params.maxICPIteration);
        icp.setTransformationEpsilon(1e-6);
        icp.setTransformationEstimation(point_to_plane);
        icp.align(*transformed, m_cloudPairs[i].relativeTrans.matrix());

        PCL_INFO("\t<%d, %d> : ICP fitness score is %.6f\n", m_cloudPairs[i].corresIdx.first, m_cloudPairs[i].corresIdx.second, icp.getFitnessScore());
        PCL_INFO("\tTransform matrix from :\n");
        cout << m_cloudPairs[i].relativeTrans.matrix() << endl;
        PCL_INFO("\tto:\n");
        m_cloudPairs[i].relativeTrans = icp.getFinalTransformation();
        cout << m_cloudPairs[i].relativeTrans.matrix() << endl;
    }

}

void CorresBuilder::findCorresPoints()
{
    boost::thread_group group;
    for (size_t i = 0; i < m_cloudPairs.size(); ++i) {
        group.create_thread(boost::bind(&CorresBuilder::findCorres, this, m_cloudPairs[i]));
    }
    group.join_all();
}

void CorresBuilder::findCorres(CloudPair& pair)
{
    std::vector<PointPair> pointCorrespondence;

    const int K = 1;

    if (pair.corresIdx == std::make_pair(-1, -1))
    {
        return;
    }

    pcl::KdTreeFLANN<PointType> kdtree;
    std::vector<int> pointIndices(K);
    std::vector<float> pointDistances(K);

    PCL_INFO("Processing pair <%d, %d>\n", pair.corresIdx.first, pair.corresIdx.second);

    CloudTypePtr p0 = m_pointClouds[pair.corresIdx.first];
    CloudTypePtr p1 = m_pointClouds[pair.corresIdx.second];

    CloudTypePtr transformed(new CloudType);
    pcl::transformPointCloudWithNormals(*p1, *transformed, pair.relativeTrans);

    kdtree.setInputCloud(p0);
    const float normThres = cosf(m_params.corresPointNormThres * M_PI / 180);
    for (size_t n = 0; n < transformed->size(); ++n) {
        if (kdtree.nearestKSearch(transformed->points[n], K, pointIndices, pointDistances) > 0) {
            PointType point0 = p0->points[pointIndices[0]];
            PointType point1 = transformed->points[n];
            float norm = Eigen::Map<Eigen::Vector3f>(point0.normal).dot(Eigen::Map<Eigen::Vector3f>(point1.normal));
            if (pointDistances[0] < m_params.corresPointDistThres * m_params.corresPointDistThres &&
                norm > normThres) {
                pointCorrespondence.push_back(PointPair(pointIndices[0], n));
            }
        }
    }

    PCL_INFO("\t<%d, %d> : Corresponce point number %d", pair.corresIdx.first, pair.corresIdx.second, pointCorrespondence.size());
    if (pair.validCorresPointNumber > 0 && pointCorrespondence.size() < pair.validCorresPointNumber * 0.5) {
        PCL_INFO("\tReduced too much!\n");
        pair.corresIdx = std::make_pair(-1, -1);
    }
    else {
        pair.validCorresPointNumber = pointCorrespondence.size();
    }

    if (m_params.saveCorresPointIndices) {
        char file[1024] = { 0 };
        sprintf(file, "%s/corres_%d_%d.txt", m_params.corresPointSavePath.c_str(), pair.corresIdx.first, pair.corresIdx.second);
        ofstream fs;
        fs.open(file, ofstream::out);
        if (fs.is_open()) {
            for (size_t i = 0; i < pointCorrespondence.size(); ++i) {
                fs << pointCorrespondence[i].first << " " << pointCorrespondence[i].second << endl;
            }
            fs.close();
        }
        else {
            PCL_ERROR("fail to save %s !", file);
        }
    }
}
