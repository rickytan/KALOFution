#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/features/feature.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/3dsc.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/smoothed_surfaces_keypoint.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/search/flann_search.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_io.h>

#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>

#include <boost/filesystem.hpp>

typedef pcl::PointNormal                    PointType;
typedef pcl::PointCloud<PointType>          CloudType;
typedef pcl::PointCloud<PointType>::Ptr     CloudTypePtr;
typedef pcl::FPFHSignature33                FeatureType;
typedef pcl::PointCloud<FeatureType>        FeatureCloudType;
typedef pcl::PointCloud<FeatureType>::Ptr   FeatureCloudTypePtr;

typedef pcl::PointWithScale                     ScalePointType;
typedef pcl::PointCloud<ScalePointType>         ScaleCloudType;
typedef pcl::PointCloud<ScalePointType>::Ptr    ScaleCloudTypePtr;


using namespace std;

CloudTypePtr loadCloud(const string &filename) {
    using namespace boost::filesystem;
    if (!exists(filename)) {
        PCL_ERROR("File not exists : %s\n", filename.c_str());
        exit(1);
    }
    CloudTypePtr cloud(new CloudType);

    path path(filename);
    if (path.extension() == ".ply") {
        pcl::io::loadPLYFile(filename, *cloud);
    }
    else if (path.extension() == ".pcd") {
        pcl::io::loadPCDFile(filename, *cloud);
    }
    else {
        PCL_ERROR("Unsupported file format : %s\n", filename.c_str());
        exit(1);
    }

    return cloud;
}

void saveCloud(CloudTypePtr cloud, const string &filename) {
    using namespace boost::filesystem;
    path path(filename);
    if (path.extension() == ".ply") {
        pcl::io::savePLYFileBinary(filename, *cloud);
    }
    else if (path.extension() == ".pcd") {
        pcl::io::savePCDFileBinary(filename, *cloud);
    }
    else {
        PCL_ERROR("Unsupported file format : %s\n", filename.c_str());
        exit(1);
    }
}

CloudTypePtr downsampledCloudWithNumberOfPoints(CloudTypePtr cloud, int points)
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

void detect_keypoints(CloudTypePtr cloud, float min_scale, int nr_octaves, int nr_scale_per_octave, float min_contrast, ScaleCloudTypePtr &out)
{
    pcl::SIFTKeypoint<PointType, ScalePointType> sift;
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, nr_octaves, nr_scale_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud);
    sift.setSearchSurface(cloud);
    sift.setKSearch(150);
    sift.compute(*out);
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        printf("Usage: %s <target_cloud> <input_cloud>\n", argv[0]);
        return 0;
    }
    string filename(argv[2]);
    string ext = filename.substr(filename.rfind('.'));
    string outfile = filename.substr(0, filename.rfind('.')) + "-aligned" + ext;
    
    CloudTypePtr cloud0 = loadCloud(argv[1]);
    CloudTypePtr cloud1 = loadCloud(argv[2]);

    CloudTypePtr transformed(new CloudType);
    CloudTypePtr aligned_cloud(new CloudType);
    ScaleCloudTypePtr out(new ScaleCloudType);

    float p1 = 0.01f;
    int p2 = 3, p3 = 1;
    float p4 = 0.f;
    sscanf(argv[3], "%f", &p1);
    sscanf(argv[4], "%d", &p2);
    sscanf(argv[5], "%d", &p3);
    sscanf(argv[6], "%f", &p4);

    detect_keypoints(cloud0, p1, p2, p3, p4, out);
    if (transformed->size() == 0) {
        PCL_ERROR("No keypoints!");
    }
    else {
        saveCloud(transformed, outfile);
    }
    return 0;

    FeatureCloudType feature_cloud0;
    FeatureCloudType feature_cloud1;

    

    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    pcl::FPFHEstimation<PointType, PointType, FeatureType> fpfh;
    fpfh.setSearchMethod(tree);
    fpfh.setKSearch(120);
    CloudTypePtr down0 = downsampledCloudWithNumberOfPoints(cloud0, 20000);
    fpfh.setInputCloud(down0);
    fpfh.setInputNormals(cloud0);
    fpfh.setSearchSurface(cloud0);
    fpfh.compute(feature_cloud0);

    CloudTypePtr down1 = downsampledCloudWithNumberOfPoints(cloud1, 20000);
    fpfh.setInputCloud(down1);
    fpfh.setInputNormals(cloud1);
    fpfh.setSearchSurface(cloud1);
    fpfh.compute(feature_cloud1);

    pcl::SampleConsensusInitialAlignment<PointType, PointType, FeatureType> sac;
    sac.setMinSampleDistance(0.15);
    sac.setMaxCorrespondenceDistance(0.15);
    sac.setMaximumIterations(1000);
//    Provide a pointer to the input point cloud and features
    sac.setInputCloud(down1);
    sac.setSourceFeatures(feature_cloud1.makeShared());
//    Provide a pointer to the target point cloud and features
    sac.setInputTarget(down0);
    sac.setTargetFeatures(feature_cloud0.makeShared());
//    Align input to target to obtain
    sac.align(*aligned_cloud);
    if (sac.hasConverged()) {
        PCL_INFO("\tSAC fitness score is %.6f\n", sac.getFitnessScore());
        PCL_INFO("\tTransform matrix :\n");
        cout << sac.getFinalTransformation() << endl;
        saveCloud(aligned_cloud, outfile);
    }
    else {
        PCL_ERROR("Can't align cloud pair!\n");
    }
    return 0;

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    typedef pcl::registration::TransformationEstimationPointToPlaneLLS<PointType, PointType> P2PlaneEstimation;
    boost::shared_ptr<P2PlaneEstimation> point_to_plane(new P2PlaneEstimation);

    printf("Align %s( %d points ) and %s( %d points )\n", argv[1], cloud0->size(), argv[2], cloud1->size());

    icp.setInputCloud(downsampledCloudWithNumberOfPoints(cloud1, 50000));
    icp.setInputTarget(downsampledCloudWithNumberOfPoints(cloud0, 50000));
    icp.setMaxCorrespondenceDistance(0.25);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-6);
    icp.setTransformationEstimation(point_to_plane);
    icp.align(*transformed);

    if (icp.hasConverged()) {

        PCL_INFO("\tICP fitness score is %.6f\n", icp.getFitnessScore());
        PCL_INFO("\tTransform matrix :\n");
        cout << icp.getFinalTransformation() << endl;
        saveCloud(transformed, outfile);
        /*
        Eigen::Affine3f changed;
        changed = icp.getFinalTransformation();
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
        */
    }
    else {
        //PCL_WARN("\tCan't align pair <%d, %d>\n", pair.corresIdx.first, pair.corresIdx.second);
        //pair.corresIdx = std::make_pair(-1, -1);
    }

    return 0;
}