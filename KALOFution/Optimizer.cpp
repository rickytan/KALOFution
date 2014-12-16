#include <Eigen/SparseCholesky>
#include <Eigen/CholmodSupport>
#include <Eigen/Geometry>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "Optimizer.h"
#include "OptimizerParameter.h"

class SparseMatFiller {
public:
    SparseMatFiller(TriContainer &tri) : m_tripletToFill(tri) {}

    void fill(int row, int col, Eigen::Matrix<double, 6, 1> &vec0, Eigen::Matrix<double, 6, 1> &vec1) {
        fillBlock(row, row, vec0 * vec0.transpose());
        fillBlock(row, col, vec0 * vec1.transpose());
        fillBlock(col, row, vec1 * vec0.transpose());
        fillBlock(col, col, vec1 * vec1.transpose());
    }

private:
    void fillBlock(int block_r, int block_c, Eigen::Matrix<double, 6, 6> mat) {
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                add(block_r * 6 + i, block_c * 6 + j, mat(i, j));
            }
        }
    }
    void add(int r, int c, double v) {
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

void Optimizer::optimizeRigid()
{
    size_t num_clouds = m_pointClouds.size();
    size_t matrix_size = num_clouds * 6;

    for (int iter_count = 0; iter_count < m_params.maxIteration; ++iter_count)
    {
        PCL_INFO("Iteration : %d\n", iter_count);

        Vec ATb(matrix_size);
        Mat ATA(matrix_size, matrix_size);

        ATb.setZero();
        ATA.setZero();

        double align_error = 0.0;

        PCL_INFO("\tProcessing %d cloud pairs\n", m_cloudPairs.size());

        boost::thread_group group;
        for (size_t pair_count = 0; pair_count < m_cloudPairs.size(); ++pair_count) {
            group.create_thread(boost::bind(&Optimizer::eachCloudPair, this, m_cloudPairs[pair_count], ATA, ATb, align_error));
        }
        group.join_all();

        PCL_INFO("\tDone!\n");
        PCL_INFO("\t\nAlign error : %.6f\n", align_error);

        Eigen::CholmodSupernodalLLT<Mat, Eigen::Upper> solver;
        solver.analyzePattern(ATA);
        solver.factorize(ATA);

        Vec X = solver.solve(ATb);

        for (size_t i = 0; i < num_clouds; ++i) {
            CloudTransform trans;
            float beta = X.block<6, 1>(i, 0)[0];
            float gammar = X.block<6, 1>(i, 0)[1];
            float alpha = X.block<6, 1>(i, 0)[2];
            trans.linear() = (Eigen::Matrix3f)Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitZ()) *
                Eigen::AngleAxisf(gammar, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(beta, Eigen::Vector3f::UnitX());
            trans.translation() = X.block<6, 1>(i, 0).block<3, 1>(1, 0).cast<float>();
        }
    }
}

void Optimizer::eachCloudPair(CloudPair &pair, Mat &ATA, Vec &ATb, double &align_error)
{
    int cloud0 = pair.corresIdx.first;
    int cloud1 = pair.corresIdx.second;

    size_t matrix_size = m_pointClouds.size() * 6;

    TriContainer mat_elem;
    SparseMatFiller filler(mat_elem);

    Vec atb(matrix_size);
    Mat ata(matrix_size, matrix_size);
    atb.setZero(), ata.setZero();

    double score = 0.0;

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
        atb.block<6, 1>(cloud0, 0) = A_p * b;
        atb.block<6, 1>(cloud1, 0) = A_q * b;
    }
    ata.setFromTriplets(mat_elem.begin(), mat_elem.end());

    std::cout << "\t\tcurrent thread : " << boost::this_thread::get_id() << std::endl;
    {
        boost::mutex::scoped_lock lock(m_cloudPairMutex);
        ATA += ata;
        ATb += atb;
        align_error += score;
    }
}
