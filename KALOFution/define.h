#ifndef _DEFINE_H_
#define _DEFINE_H_

#include <cstdio>
#include <cstdlib>

#ifdef WIN32

#endif


namespace Eigen {
    namespace internal {
        static inline double sqrt(double a) { return sqrt(a); }
        static inline double cos(double a) { return cos(a); }
        static inline double sin(double a) { return sin(a); }
    }
}


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/math/special_functions.hpp>

using boost::math::isnan;

typedef pcl::PointXYZRGBNormal PointType;
typedef pcl::PointCloud<PointType> CloudType;
typedef pcl::PointCloud<PointType>::Ptr CloudTypePtr;
typedef std::pair<int, int> PointPair;
typedef Eigen::Affine3f CloudTransform;


typedef struct CloudPair {
    std::pair<int, int> corresIdx;
    Eigen::Affine3f relativeTrans;
    int validCorresPointNumber;
    std::vector<PointPair> corresPointIdx;
    CloudPair(int p, int q, const Eigen::Affine3f& incTrans) {
        corresIdx = std::make_pair(p, q);
        relativeTrans = incTrans;
        validCorresPointNumber = 0;
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} CloudPair;


//#endif  // cplusplus
#endif  // _DEFINE_H_