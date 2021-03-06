#include <fstream>
#include <iostream>

#include "define.h"

#include <Eigen/SparseCholesky>
#include <Eigen/CholmodSupport>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/filesystem.hpp>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/slam3d/types_slam3d.h>

#include "Optimizer.h"
#include "OptimizerParameter.h"


class SparseMatFiller {
public:
    SparseMatFiller(TriContainer &tri) : m_tripletToFill(tri) {}

    void fill(int row, int col, const Eigen::Matrix<double, 6, 1> &vec0, const Eigen::Matrix<double, 6, 1> &vec1) {
        fillBlock(row, row, vec0 * vec0.transpose());
        fillBlock(row, col, vec0 * vec1.transpose());
        fillBlock(col, row, vec1 * vec0.transpose());
        fillBlock(col, col, vec1 * vec1.transpose());
    }
    inline void add(int r, int c, double v) {
        m_tripletToFill.push_back(Tri(r, c, v));
        /*
        if (!m_indexMap.count(r)) {
            m_indexMap[r] = std::map<int, size_t>();
        }
        if (!m_indexMap[r].count(c)) {
            m_indexMap[r][c] = m_tripletToFill.size();
            m_tripletToFill.push_back(Tri(r, c, v));
        }
        else {
            m_tripletToFill[m_indexMap[r][c]].m_value += v;
        }
         */
    }
private:
    inline void fillBlock(int block_r, int block_c, const Eigen::Matrix<double, 6, 6> &mat) {
#pragma unroll 6
        for (int i = 0; i < 6; ++i) {
#pragma unroll 6
            for (int j = 0; j < 6; ++j) {
                add(block_r * 6 + i, block_c * 6 + j, mat(i, j));
            }
        }
    }

private:
    TriContainer &m_tripletToFill;
    std::map<int, std::map<int, size_t> > m_indexMap;
};

Optimizer::Optimizer(OptimizerParameter &params)
: m_params(params)
{
}


Optimizer::~Optimizer()
{
}

using namespace g2o;

G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(slam3d);

void Optimizer::optimizeUseG2O()
{


    // create the linear solver
    BlockSolverX::LinearSolverType * linearSolver = new LinearSolverCSparse<BlockSolverX::PoseMatrixType>();

    // create the block solver on top of the linear solver
    BlockSolverX* blockSolver = new BlockSolverX(linearSolver);

    // create the algorithm to carry out the optimization
    //OptimizationAlgorithmGaussNewton* optimizationAlgorithm = new OptimizationAlgorithmGaussNewton(blockSolver);
    OptimizationAlgorithmLevenberg* optimizationAlgorithm = new OptimizationAlgorithmLevenberg(blockSolver);

    // NOTE: We skip to fix a variable here, either this is stored in the file
    // itself or Levenberg will handle it.

    // create the optimizer to load the data and carry out the optimization
    SparseOptimizer optimizer;
    SparseOptimizer::initMultiThreading();
    optimizer.setVerbose(true);
    optimizer.setAlgorithm(optimizationAlgorithm);

    {
        pcl::ScopeTime time("G2O setup Graph vertices");
        for (size_t cloud_count = 0; cloud_count < m_pointClouds.size(); ++cloud_count)
        {
            VertexSE3 *vertex = new VertexSE3;
            vertex->setId(cloud_count);
            Isometry3D affine = Isometry3D::Identity();
            affine.linear() = m_pointClouds[cloud_count]->sensor_orientation_.toRotationMatrix().cast<Isometry3D::Scalar>();
            affine.translation() = m_pointClouds[cloud_count]->sensor_origin_.block<3, 1>(0, 0).cast<Isometry3D::Scalar>();
            vertex->setEstimate(affine);
            optimizer.addVertex(vertex);
        }
        optimizer.vertex(0)->setFixed(true);
    }

    {
        pcl::ScopeTime time("G2O setup Graph edges");
        double trans_noise = 0.5, rot_noise = 0.5235;
        EdgeSE3::InformationType infomation = EdgeSE3::InformationType::Zero();
        infomation.block<3, 3>(0, 0) << trans_noise * trans_noise, 0, 0,
                                        0, trans_noise * trans_noise, 0,
                                        0, 0, trans_noise * trans_noise;
        infomation.block<3, 3>(3, 3) << rot_noise * rot_noise, 0, 0,
                                        0, rot_noise * rot_noise, 0,
                                        0, 0, rot_noise * rot_noise;
        for (size_t pair_count = 0; pair_count < m_cloudPairs.size(); ++pair_count)
        {
            CloudPair pair = m_cloudPairs[pair_count];
		    int from = pair.corresIdx.first;
		    int to = pair.corresIdx.second;
            EdgeSE3 *edge = new EdgeSE3;
		    edge->vertices()[0] = optimizer.vertex(from);
		    edge->vertices()[1] = optimizer.vertex(to);

            Eigen::Matrix<double, 6, 6> ATA = Eigen::Matrix<double, 6, 6>::Zero();
            Eigen::Matrix<double, 6, 1> ATb = Eigen::Matrix<double, 6, 1>::Zero();
#pragma unroll 8
            for (size_t point_count = 0; point_count < pair.corresPointIdx.size(); ++point_count) {
                int point_p = pair.corresPointIdx[point_count].first;
                int point_q = pair.corresPointIdx[point_count].second;
                PointType P = m_pointClouds[from]->points[point_p];
                PointType Q = m_pointClouds[to]->points[point_q];

                Eigen::Vector3d p = P.getVector3fMap().cast<double>();
                Eigen::Vector3d q = Q.getVector3fMap().cast<double>();
                Eigen::Vector3d Np = P.getNormalVector3fMap().cast<double>();

                double b = (p - q).dot(Np);

                Eigen::Matrix<double, 6, 1> A_p;
                A_p.block<3, 1>(0, 0) = p.cross(Np);
                A_p.block<3, 1>(3, 0) = Np;

                ATA += A_p * A_p.transpose();
                ATb += A_p * b;
            }

            Eigen::Matrix<double, 6, 1> X = ATA.ldlt().solve(ATb);
            Isometry3D measure = Isometry3D::Identity();
            float beta = X[0];
            float gammar = X[1];
            float alpha = X[2];
            measure.linear() = (Eigen::Matrix3d)Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(gammar, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(beta, Eigen::Vector3d::UnitX());
            measure.translation() = X.block<3, 1>(3, 0);

            edge->setMeasurement(measure);

		    edge->setInformation(infomation);
            
            optimizer.addEdge(edge);
        }
    }

    optimizer.save("debug_preOpt.g2o");
    {
        pcl::ScopeTime time("g2o optimizing");
        optimizer.initializeOptimization();
        optimizer.optimize(30);
    }
    optimizer.save("debug_postOpt.g2o");

    for (size_t cloud_count = 0; cloud_count < m_pointClouds.size(); ++cloud_count)
    {
        CloudTypePtr tmp(new CloudType);
        Isometry3D trans = ((VertexSE3 *)optimizer.vertices()[cloud_count])->estimate();
        Eigen::Affine3d affine;
        affine.linear() = trans.rotation();
        affine.translation() = trans.translation();
        pcl::transformPointCloudWithNormals(*m_pointClouds[cloud_count], *tmp, affine.cast<float>());
        pcl::copyPointCloud(*tmp, *m_pointClouds[cloud_count]);
    }

    PCL_WARN("Opitimization DONE!!!!\n");

    if (m_params.saveDirectory.length()) {
        if (boost::filesystem::exists(m_params.saveDirectory) && !boost::filesystem::is_directory(m_params.saveDirectory)) {
            boost::filesystem::remove(m_params.saveDirectory);
        }
        boost::filesystem::create_directories(m_params.saveDirectory);

        char filename[1024] = { 0 };
        for (size_t i = 0; i < m_pointClouds.size(); ++i) {
            sprintf(filename, "%s/cloud_%d.ply", m_params.saveDirectory.c_str(), i);
            pcl::io::savePLYFileBinary(filename, *m_pointClouds[i]);
        }
    }
}

void Optimizer::optimizeRigid()
{
    size_t num_clouds = m_pointClouds.size();
    size_t matrix_size = num_clouds * 6;

    ATb.resize(matrix_size);
    ATA.resize(matrix_size, matrix_size);

    for (int iter_count = 0; iter_count < m_params.maxIteration; ++iter_count)
    {
        PCL_INFO("Iteration : %d\n", iter_count);
        pcl::ScopeTime time("optimizing");

        ATb.setZero();
        ATA.setZero();

        align_error = 0.0;

        PCL_INFO("\tProcessing %d cloud pairs\n", m_cloudPairs.size());
        
        int cpu_cores = std::max(boost::thread::hardware_concurrency(), (unsigned)2);
        int count = 0;
        boost::thread_group group;
        for (size_t pair_count = 0; pair_count < m_cloudPairs.size(); ++pair_count) {
            if (count == cpu_cores) {
                count = 0;
                group.join_all();
            }
            group.create_thread(boost::bind(&Optimizer::eachCloudPair, this, boost::ref(m_cloudPairs[pair_count])));
            count++;
        }
        group.join_all();
        
        /*
        for (size_t pair_count = 0; pair_count < m_cloudPairs.size(); ++pair_count) {
            eachCloudPair(m_cloudPairs[pair_count]);
        }*/

        PCL_WARN("\tDone!\n");
        PCL_WARN("\tAlign error : %.6f\n", align_error);

        Eigen::IOFormat format(Eigen::FullPrecision, 0);
        std::ofstream("debug_matrix.txt", std::ios::out) << ATA;
        std::ofstream("debug_b.txt", std::ios::out) << ATb.format(format);

        Vec X;
        if (m_params.useCholmod) {
            pcl::ScopeTime mat_solve("\t\tMatrix solve");
            Eigen::CholmodSupernodalLLT<Mat> solver;
            solver.compute(ATA);
            X = solver.solve(ATb);
        }
        else {
            pcl::ScopeTime mat_solve("\t\tMatrix solve");
            Eigen::SimplicialCholesky<Mat> solver(ATA);
            X = solver.solve(ATb);
        }

        std::ofstream("debug_result.txt", std::ios::out) << X.format(format);
        //std::cout << "Result :\n\n" << X << std::endl << std::endl;

#pragma unroll 8
        for (size_t i = 0; i < num_clouds; ++i) {
            CloudTransform trans;
            float beta = X.block<6, 1>(i * 6, 0)[0];
            float gammar = X.block<6, 1>(i * 6, 0)[1];
            float alpha = X.block<6, 1>(i * 6, 0)[2];
            trans.linear() = (Eigen::Matrix3f)Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitZ()) *
                Eigen::AngleAxisf(gammar, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(beta, Eigen::Vector3f::UnitX());
            trans.translation() = X.block<6, 1>(i * 6, 0).block<3, 1>(3, 0).cast<float>();
            CloudTypePtr transformed(new CloudType);
            pcl::transformPointCloudWithNormals(*m_pointClouds[i], *transformed, trans);
            m_pointClouds[i] = transformed;
        }
    }

    PCL_WARN("Opitimization DONE!!!!\n");

    if (m_params.saveDirectory.length()) {
        if (boost::filesystem::exists(m_params.saveDirectory) && !boost::filesystem::is_directory(m_params.saveDirectory)) {
            boost::filesystem::remove(m_params.saveDirectory);
        }
        boost::filesystem::create_directories(m_params.saveDirectory);

        char filename[1024] = { 0 };
        for (size_t i = 0; i < m_pointClouds.size(); ++i) {
            sprintf(filename, "%s/cloud_%d.ply", m_params.saveDirectory.c_str(), i);
            pcl::io::savePLYFileBinary(filename, *m_pointClouds[i]);
        }
    }
}

void Optimizer::eachCloudPair(CloudPair &pair)
{
    int cloud0 = pair.corresIdx.first;
    int cloud1 = pair.corresIdx.second;

    size_t matrix_size = m_pointClouds.size() * 6;

    TriContainer mat_elem;
    mat_elem.reserve(matrix_size * matrix_size / 5);
    SparseMatFiller filler(mat_elem);

    for (int i = 0; i < 6; ++i) {
        filler.add(i, i, 1.0);
    }

    Vec atb(matrix_size);
    Mat ata(matrix_size, matrix_size);
    atb.setZero(), ata.setZero();

    double score = 0.0;
    {
        //pcl::ScopeTime time("calculate LSE matrix");
    
#pragma unroll 8
        for (size_t point_count = 0; point_count < pair.corresPointIdx.size(); ++point_count) {
            int point_p = pair.corresPointIdx[point_count].first;
            int point_q = pair.corresPointIdx[point_count].second;
            PointType P = m_pointClouds[cloud0]->points[point_p];
            PointType Q = m_pointClouds[cloud1]->points[point_q];

            Eigen::Vector3d p = P.getVector3fMap().cast<double>();
            Eigen::Vector3d q = Q.getVector3fMap().cast<double>();
            Eigen::Vector3d Np = P.getNormalVector3fMap().cast<double>();

            double b = -(p - q).dot(Np);
            score += b * b;
            Eigen::Matrix<double, 6, 1> A_p, A_q;
            A_p.block<3, 1>(0, 0) = p.cross(Np);
            A_p.block<3, 1>(3, 0) = Np;
            A_q.block<3, 1>(0, 0) = -q.cross(Np);
            A_q.block<3, 1>(3, 0) = -Np;
        
            filler.fill(cloud0, cloud1, A_p, A_q);
            atb.block<6, 1>(cloud0 * 6, 0) += A_p * b;
            atb.block<6, 1>(cloud1 * 6, 0) += A_q * b;
        }
        ata.setFromTriplets(mat_elem.begin(), mat_elem.end());
    }

    {
        //pcl::ScopeTime time("Fill sparse matrix");
        boost::mutex::scoped_lock lock(m_cloudPairMutex);
        //std::cout << "\tcurrent thread : " << boost::this_thread::get_id() << std::endl;
        //PCL_INFO("\tPair <%d, %d> alignment Score : %.6f\n", cloud0, cloud1, score);
        ATA += ata;
        ATb += atb;
        align_error += score;
    }
}

void Optimizer::operator()(std::vector<CloudTypePtr> &pointClouds, std::vector<CloudPair> &cloudPaire)
{
    m_pointClouds = pointClouds;
    m_cloudPairs = cloudPaire;
    
    if (m_params.g2oOptimize) {
        optimizeUseG2O();
    }
    else {
        optimizeRigid();
    }
}

void Optimizer::mergeClouds()
{
    CloudTypePtr all_cloud(new CloudType);
    for (size_t i = 0; i < m_pointClouds.size(); ++i) {
        //all_cloud += m_pointClouds[i];
    }
}
