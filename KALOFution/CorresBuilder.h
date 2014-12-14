#ifndef _CORRESBUILDER_H_
#define _CORRESBUILDER_H_

#include <vector>
#include <map>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "define.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class DataProvider;
class DefaultDataProvider;


class CorresBuilder
{
public:
    CorresBuilder() {}
    ~CorresBuilder() {}

    void operator ()(DataProvider& provider);

private:
    float volumeOverlapRatio(const Eigen::Affine3f& trans);
private:
    std::vector<CloudTypePtr> m_pointClouds;
    std::vector<CloudPair, Eigen::aligned_allocator<CloudPair> > m_cloudPairs;
    std::vector<CloudTransform, Eigen::aligned_allocator<CloudTransform> > m_initCloudTransform;
};

#endif  // _CORRESBUILDER_H_