#pragma once
#ifndef _OPTIMIZER_H_
#define _OPTIMIZER_H_

#include "define.h"

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <Eigen/StdVector>

class OptimizerParameter;
class boost::mutex;

typedef Eigen::VectorXd                                     Vec;
typedef Eigen::SparseMatrix<double>                         Mat;
typedef Eigen::Triplet<double>                              Tri;
typedef std::vector<Tri, Eigen::aligned_allocator<Tri> >    TriContainer;

class Optimizer
{
public:
    Optimizer(OptimizerParameter &params);
    ~Optimizer();

    void operator ()(std::vector<CloudTypePtr> &pointClouds, std::vector<CloudPair> &cloudPaire);

private:
    void optimizeRigid();
    void eachCloudPair(CloudPair &pair);
    void mergeClouds();
    void optimizeUseG2O();
private:
    OptimizerParameter &m_params;

    std::vector<CloudTypePtr> m_pointClouds;
    std::vector<CloudPair> m_cloudPairs;

    boost::mutex m_cloudPairMutex;

    Mat ATA;
    Vec ATb;
    double align_error;
};

#endif  // _OPTIMIZER_H_
