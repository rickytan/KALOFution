#pragma once
#ifndef _CORRESBUILDER_H_
#define _CORRESBUILDER_H_

#include "define.h"

#include <vector>
#include <map>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <boost/thread.hpp>

#include "BasicParameter.h"

class DataProvider;


class CorresBuilder
{
public:
    CorresBuilder(BasicParameter &params): m_params(params) {}
    ~CorresBuilder() {}

    void operator ()(DataProvider& provider);

private:
    void initCloudAndTransform(DataProvider& provider);
    void initCloudPair();
    float volumeOverlapRatio(const Eigen::Affine3f& trans);
    float centroidDistance(int cloud0, int cloud1);
    void findCorresPoints();
    void findCorres(CloudPair& pair);
    void alignPairs();
    void alignEachPair(CloudPair &pair);
    CloudTypePtr downsampledCloudWithNumberOfPoints(CloudTypePtr cloud, int points);
    void gridFilterPointPair(CloudPair& pair, float gridSize, int Nc = 5);
private:
    std::vector<CloudTypePtr> m_pointClouds;
    std::vector<CloudPair, Eigen::aligned_allocator<CloudPair> > m_cloudPairs;
    std::vector<CloudTransform, Eigen::aligned_allocator<CloudTransform> > m_initCloudTransform;

    BasicParameter &m_params;

    boost::mutex m_outputMutex;
};

#endif  // _CORRESBUILDER_H_