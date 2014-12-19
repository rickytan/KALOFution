#include <limits>
#include <numeric>
#include <cfloat>
#include <cmath>
#include <fstream>
#include <cstdio>
#include <cstdlib>

#include "CorresBuilder.h"
#include "DataProvider.h"

#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/math/special_functions.hpp>
#include <boost/random.hpp>

using namespace std;

float CorresBuilder::volumeOverlapRatio(const Eigen::Affine3f& trans)
{
    const int res = 20;
    double ul = 3. / res;
    int s = 0;
    for (int i = 0; i < res; i++) {
        for (int j = 0; j < res; j++) {
            for (int k = 0; k < res; k++) {
                Eigen::Vector3f pos((i + 0.5) * ul, (j + 0.5) * ul, (k + 0.5) * ul);
                Eigen::Vector3f ppos = trans * pos;
                if (ppos(0) >= 0 && ppos(0) <= 3. &&
                    ppos(1) >= 0 && ppos(1) <= 3. &&
                    ppos(2) >= 0 && ppos(2) <= 3.) {
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
    using namespace boost::math;
    m_pointClouds.resize(provider.size());

    for (uint32_t i = 0; i < provider.size(); ++i)
    {
        CloudTypePtr rawCloud = provider[i];
        CloudTypePtr cloud(new CloudType);
        for (int j = 0; j < rawCloud->points.size(); ++j)
        {
            if (!isnan(rawCloud->points[j].normal_x)) {
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
    int cpu_cores = std::max(boost::thread::hardware_concurrency(), (unsigned)2);
    int count = 0;
    boost::thread_group group;
    BOOST_FOREACH(CloudPair &pair, m_cloudPairs) {
        if (count == cpu_cores) {
            count = 0;
            group.join_all();
        }
        group.create_thread(boost::bind(&CorresBuilder::alignEachPair, this, pair));
        count++;
    }
    group.join_all();
}

void CorresBuilder::findCorresPoints()
{
    boost::thread_group group;
    int cores = boost::thread::hardware_concurrency();
    for (size_t i = 0; i < m_cloudPairs.size(); ++i) {
        if (i % cores == cores - 1) {
            group.join_all();
        }
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
                fabsf(norm) > normThres) {
                pointCorrespondence.push_back(PointPair(pointIndices[0], n));
            }
        }
    }

    PCL_INFO("\t<%d, %d> : Corresponce point number %d\n", pair.corresIdx.first, pair.corresIdx.second, pointCorrespondence.size());
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

CloudTypePtr CorresBuilder::downsampledCloudWithNumberOfPoints(CloudTypePtr cloud, int points)
{
    if (cloud->size() <= points)
        return cloud;

    size_t size = cloud->size();
    CloudTypePtr downsampledCloud(new CloudType);

    boost::random::mt19937 rng;
    boost::random::uniform_int_distribution<> six(0, size - 1);
    for (int i = 0; i < points; ++i)
    {
        downsampledCloud->points.push_back(cloud->points[six(rng)]);
    }
    downsampledCloud->width = downsampledCloud->points.size();
    downsampledCloud->height = 1;
    downsampledCloud->is_dense = true;
    return downsampledCloud;
}

void CorresBuilder::alignEachPair(CloudPair &pair)
{
    const int K = 1;
    pcl::KdTreeFLANN<PointType> kdtree;
    std::vector<int> pointIndices(K);
    std::vector<float> pointDistances(K);

    PCL_INFO("Align pair <%d, %d>\n", pair.corresIdx.first, pair.corresIdx.second);

    CloudTypePtr p0 = m_pointClouds[pair.corresIdx.first];
    CloudTypePtr p1 = m_pointClouds[pair.corresIdx.second];

    CloudTypePtr transformed(new CloudType);
    pcl::transformPointCloudWithNormals(*p1, *transformed, pair.relativeTrans);

    kdtree.setInputCloud(p0);
    size_t count = 0;
    for (size_t n = 0; n < transformed->size(); ++n) {
        if (kdtree.nearestKSearch(transformed->points[n], K, pointIndices, pointDistances) > 0) {
            if (pointDistances[0] < m_params.acceptableICPPointDistThres * m_params.acceptableICPPointDistThres) {
                ++count;
            }
        }
    }

    double r1 = (double)count / (double)p0->size();
    double r2 = (double)count / (double)p1->size();
    PCL_INFO("    <%d, %d> : %d inliers with ratio %.2f(%d) and %.2f(%d) ...\n", pair.corresIdx.first, pair.corresIdx.second, count, r1, p0->size(), r2, p1->size());

    bool accept = count >= m_params.acceptableCorresPointNum ||
        (r1 > m_params.acceptableCorresPointRatio && r2 > m_params.acceptableCorresPointRatio);
    if (!accept) {
        PCL_INFO("reject\n");
        pair.corresIdx = std::make_pair(-1, -1);
        return;
    }

    PCL_INFO("accept\n");
    pair.validCorresPointNumber = count;

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    typedef pcl::registration::TransformationEstimationPointToPlaneLLS<PointType, PointType> P2PlaneEstimation;
    boost::shared_ptr<P2PlaneEstimation> point_to_plane(new P2PlaneEstimation);

    icp.setInputCloud(downsampledCloudWithNumberOfPoints(p1, 15000));
    icp.setInputTarget(downsampledCloudWithNumberOfPoints(p0, 15000));
    icp.setMaxCorrespondenceDistance(m_params.acceptableICPPointDistThres);
    icp.setMaximumIterations(m_params.maxICPIteration);
    icp.setTransformationEpsilon(1e-6);
    icp.setTransformationEstimation(point_to_plane);
    icp.align(*transformed, pair.relativeTrans.matrix());

    if (icp.hasConverged()) {
        PCL_INFO("\t<%d, %d> : ICP fitness score is %.6f\n", pair.corresIdx.first, pair.corresIdx.second, icp.getFitnessScore());
        PCL_INFO("\tTransform matrix from :\n");
        cout << pair.relativeTrans.matrix() << endl;
        PCL_INFO("\tto:\n");
        pair.relativeTrans = icp.getFinalTransformation();
        cout << pair.relativeTrans.matrix() << endl;
    }
    else {
        PCL_WARN("\tCan't align pair <%d, %d>\n", pair.corresIdx.first, pair.corresIdx.second);
    }
}
