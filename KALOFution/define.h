#ifndef _DEFINE_H_
#define _DEFINE_H_

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>

#ifdef _MSC_VER
/*
namespace Eigen {
	namespace internal {
		static inline double sqrt(double a) { return sqrt(a); }
		static inline double cos(double a) { return cos(a); }
		static inline double sin(double a) { return sin(a); }
	}
}
*/
#else

namespace Eigen {
    namespace internal {
        static inline double sqrt(double a) { return sqrt(a); }
        static inline double cos(double a) { return cos(a); }
        static inline double sin(double a) { return sin(a); }
    }
}

#endif

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/math/special_functions.hpp>

//using boost::math::isnan;

typedef pcl::PointNormal PointType;
typedef pcl::PointCloud<PointType> CloudType;
typedef pcl::PointCloud<PointType>::Ptr CloudTypePtr;

typedef Eigen::Affine3f CloudTransform;

struct PointPair: public std::pair<int, int> {
    typedef struct {
        bool operator()(const PointPair& _Left, const PointPair& _Right) const {
            return _Left.quality > _Right.quality;
        }
    } PointPairComparer;

    typedef struct {
        bool operator()(const PointPair& _Left, const PointPair& _Right) const {
            return _Left.first < _Right.first;
        }
    } PointPairIndexComparer;

    PointPair(): std::pair<int, int>(), quality(0.f) {}
    PointPair(int first, int second, float q = 0.f) {
        this->first = first;
        this->second = second;
        this->quality = q;
    }
    float quality;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef struct CloudPair {
    std::pair<int, int> corresIdx;
    Eigen::Affine3f relativeTrans;
    int validCorresPointNumber;
    std::vector<PointPair, Eigen::aligned_allocator<PointPair> > corresPointIdx;
    CloudPair(int p, int q, const Eigen::Affine3f& incTrans = Eigen::Affine3f::Identity()): corresPointIdx() {
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
                corresPointIdx.push_back(PointPair(p0, p1));
            }
            infile.close();
        }
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} CloudPair;


//#endif  // cplusplus
#endif  // _DEFINE_H_