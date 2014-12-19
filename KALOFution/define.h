#ifndef _DEFINE_H_
#define _DEFINE_H_

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>

#ifdef WIN32

#endif

/*
namespace Eigen {
    namespace internal {
        static inline double sqrt(double a) { return sqrt(a); }
        static inline double cos(double a) { return cos(a); }
        static inline double sin(double a) { return sin(a); }
    }
}
*/

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/math/special_functions.hpp>

using boost::math::isnan;

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> CloudType;
typedef pcl::PointCloud<PointType>::Ptr CloudTypePtr;
typedef std::pair<int, int> PointPair;
typedef Eigen::Affine3f CloudTransform;


typedef struct CloudPair {
    std::pair<int, int> corresIdx;
    Eigen::Affine3f relativeTrans;
    int validCorresPointNumber;
    std::vector<PointPair> corresPointIdx;
    CloudPair(int p, int q, const Eigen::Affine3f& incTrans = Eigen::Affine3f::Identity()) {
        corresIdx = std::make_pair(p, q);
        relativeTrans = incTrans;
        validCorresPointNumber = 0;
    }
    void loadCorresPoints(const std::string &file) {
        PCL_INFO("Loading correspondences : %s\n", file.c_str());
        std::ifstream infile(file, std::ios::in);
        if (infile.is_open()) {
            int p0, p1;
            while (infile >> p0 >> p1) {
                corresPointIdx.push_back(std::make_pair(p0, p1));
            }
            infile.close();
        }
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} CloudPair;


//#endif  // cplusplus
#endif  // _DEFINE_H_