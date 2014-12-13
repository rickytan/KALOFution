#ifndef _CORRESBUILDER_H_
#define _CORRESBUILDER_H_

#include <vector>
#include <map>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template <typename PointType>
class DataProvider;

typedef struct CloudPair {
    std::pair<int, int> corresIdx;
    Eigen::Affine3f relativeTrans;
    struct CloudPair(int p, int q, const Eigen::Affine3f& incTrans) {
        corresIdx = std::make_pair(p, q);
        relativeTrans = incTrans;
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} CloudPair;

template <typename PointType>
class CorresBuilder
{
    typedef pcl::PointCloud<PointType> CloudType;
    typedef typename CloudType::Ptr CloudTypePtr;
    typedef std::pair<int, int> PointPair;
    typedef Eigen::Affine3f CloudTransform;

public:
    CorresBuilder() {}
    ~CorresBuilder() {}

    void operator ()(const DataProvider<PointType>& provider);

private:
    float volumeOverlapRatio(const Eigen::Affine3f& trans);
private:
    std::vector<CloudTypePtr> m_pointClouds;
    std::vector<CloudPair, Eigen::aligned_allocator<CloudPair> > m_cloudPairs;
    std::vector<CloudTransform, Eigen::aligned_allocator<CloudTransform> > m_initCloudTransform;
};

#endif  // _CORRESBUILDER_H_