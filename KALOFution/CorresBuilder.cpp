#include <limits>
#include <numeric>
#include <algorithm>

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
    m_pointClouds.resize(provider.size());

    for (uint32_t i = 0; i < provider.size(); ++i)
    {
        CloudTypePtr rawCloud = provider[i];
        CloudTypePtr cloud(new CloudType);
        for (int j = 0; j < rawCloud->points.size(); ++j)
        {
            if (!pcl_isnan(rawCloud->points[j].normal_x)) {
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
            if (volumeOverlapRatio(trans) > 0.3f && centroidDistance(i, j) < 1.5) {
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
        group.create_thread(boost::bind(&CorresBuilder::alignEachPair, this, boost::ref(pair)));
        count++;
    }
    group.join_all();
    
    /*
    BOOST_FOREACH(CloudPair &pair, m_cloudPairs) {
        alignEachPair(pair);
    }
     */
}

void CorresBuilder::findCorresPoints()
{
    boost::thread_group group;
    int cores = std::max(boost::thread::hardware_concurrency(), (unsigned)2);
    int count = 0;
    for (size_t i = 0; i < m_cloudPairs.size(); ++i) {
        if (count == cores) {
            count = 0;
            group.join_all();
        }
        if (m_cloudPairs[i].corresIdx != std::make_pair(-1, -1)) {
            group.create_thread(boost::bind(&CorresBuilder::findCorres, this, boost::ref(m_cloudPairs[i])));
            count++;
        }
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
        if (!m_params.useBetterCorrespondence) {
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
        else {
            const float lamda = 10.f;
            if (kdtree.radiusSearch(transformed->points[n], m_params.corresPointDistThres, pointIndices, pointDistances, 50) > 0) {
                for (int i = 0; i < pointDistances.size(); ++i)
                {
                    Eigen::Vector3f Np = transformed->points[n].getNormalVector3fMap();
                    Eigen::Vector3f Nq = p0->points[pointIndices[i]].getNormalVector3fMap();
                    pointDistances[i] = 1.f - sqrtf(pointDistances[i]) / m_params.corresPointDistThres + lamda * Np.dot(Nq);
                }
                int index = std::max_element(pointDistances.begin(), pointDistances.end()) - pointDistances.begin();
                float norm = transformed->points[n].getNormalVector3fMap().dot(p0->points[pointIndices[index]].getNormalVector3fMap());
                if ( norm > normThres) {
                    pointCorrespondence.push_back(PointPair(pointIndices[index], n));
                }
            }
        }
    }

    PCL_INFO("\t<%d, %d> : Corresponce point number %d / %d\n", pair.corresIdx.first, pair.corresIdx.second, pointCorrespondence.size(), pair.validCorresPointNumber);
    if (pair.validCorresPointNumber > 0 && (pointCorrespondence.size() < pair.validCorresPointNumber / 2)) {
        PCL_WARN("\tReduced too much!\n");
        pair.corresIdx = std::make_pair(-1, -1);
    }
    else {
        pair.validCorresPointNumber = pointCorrespondence.size();
    }

    if (m_params.saveCorresPointIndices && pair.corresIdx != std::make_pair(-1, -1)) {
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

    int sample_count = m_params.randomSamplingLimitUsedToAlignment;
    CloudTypePtr p0 = downsampledCloudWithNumberOfPoints(m_pointClouds[pair.corresIdx.first], sample_count);
    CloudTypePtr p1 = downsampledCloudWithNumberOfPoints(m_pointClouds[pair.corresIdx.second], sample_count);

    CloudTypePtr transformed(new CloudType);
    pcl::transformPointCloudWithNormals(*p1, *transformed, pair.relativeTrans);

    kdtree.setInputCloud(p0);
    size_t count = 0;
#pragma unroll 8
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

    const int min_count = m_params.acceptableCorresPointNum;
    const float min_ratio = m_params.acceptableCorresPointRatio;
    bool accept = count >= min_count || (r1 > min_ratio && r2 > min_ratio);
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

    icp.setInputCloud(downsampledCloudWithNumberOfPoints(p1, 20000));
    icp.setInputTarget(downsampledCloudWithNumberOfPoints(p0, 20000));
    icp.setMaxCorrespondenceDistance(m_params.acceptableICPPointDistThres);
    icp.setMaximumIterations(m_params.maxICPIteration);
    icp.setRANSACOutlierRejectionThreshold(0.05);
    icp.setRANSACIterations(20);
    icp.setTransformationEpsilon(1e-6);
    icp.setTransformationEstimation(point_to_plane);
    icp.align(*transformed, pair.relativeTrans.matrix());

    if (icp.hasConverged()) {
        boost::mutex::scoped_lock lock(m_outputMutex);

        PCL_INFO("\t<%d, %d> : ICP fitness score is %.6f\n", pair.corresIdx.first, pair.corresIdx.second, icp.getFitnessScore());
        PCL_INFO("\tTransform matrix from :\n");
        cout << pair.relativeTrans.matrix() << endl;
        PCL_INFO("\tto:\n");
        cout << icp.getFinalTransformation() << endl;

        Eigen::Affine3f changed;
        changed = pair.relativeTrans.matrix().inverse() * icp.getFinalTransformation();
        if (changed.translation().maxCoeff() > m_params.translationThres || changed.translation().minCoeff() < -m_params.translationThres) {
            PCL_WARN("Pair align translate too much!! Reject\n");
            pair.corresIdx = std::make_pair(-1, -1);
            return;
        }

        Eigen::AngleAxisf angle;
        angle.fromRotationMatrix(changed.linear());
        if (fabsf(angle.angle()) > m_params.rotationThres * M_PI / 180) {
            PCL_WARN("Pair align rotate too much!! Reject\n");
            pair.corresIdx = std::make_pair(-1, -1);
            return;
        }

        pair.relativeTrans = icp.getFinalTransformation();
    }
    else {
        PCL_WARN("\tCan't align pair <%d, %d>\n", pair.corresIdx.first, pair.corresIdx.second);
        pair.corresIdx = std::make_pair(-1, -1);
    }
}

float CorresBuilder::centroidDistance(int cloud0, int cloud1)
{
    Eigen::Vector4f c0, c1;
    pcl::compute3DCentroid(*m_pointClouds[cloud0], c0);
    pcl::compute3DCentroid(*m_pointClouds[cloud1], c1);

    return (c0 - c1).norm();
}
