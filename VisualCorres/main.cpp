#include <iostream>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

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

#include <pcl/console/parse.h>

#include <boost/filesystem.hpp>

#include "define.h"

typedef pcl::PointNormal                    PointType;
typedef pcl::PointCloud<PointType>          CloudType;
typedef pcl::PointCloud<PointType>::Ptr     CloudTypePtr;
typedef pcl::FPFHSignature33                FeatureType;
typedef pcl::PointCloud<FeatureType>        FeatureCloudType;
typedef pcl::PointCloud<FeatureType>::Ptr   FeatureCloudTypePtr;

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

int main(int argc, char *argv[]) {
    PCL_INFO("Load cloud 0\n");
    CloudTypePtr cloud0 = loadCloud(argv[1]);
    PCL_INFO("Load cloud 1\n");
    CloudTypePtr cloud1 = loadCloud(argv[2]);

    CloudPair pair(-1, -1);
    pair.loadCorresPoints(argv[3]);
    PCL_INFO("Loaded!\n");

    pcl::visualization::PCLVisualizer viewer;
    viewer.addCoordinateSystem(1.0);

    viewer.addPointCloud<PointType>(cloud0, "Cloud 0");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "Cloud 0");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud 0");
    viewer.addPointCloud<PointType>(cloud1, "Cloud 1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "Cloud 1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "Cloud 1");
    
    PCL_INFO("Adding correspondence lines...\n");
    int lines = 1200;
    pcl::console::parse_argument(argc, argv, "--lines", lines);


    const int N = pair.corresPointIdx.size() / lines;
    size_t start = pair.corresPointIdx.size() * (N - 1) / (2 * N);
    size_t size = start + pair.corresPointIdx.size() / N;
    for (size_t i = start; i < size; ++i)
    {
        char line_id[255] = { 0 }; 
        sprintf(line_id, "line_%d", i);
        viewer.addLine(cloud0->points[pair.corresPointIdx[i].first], cloud1->points[pair.corresPointIdx[i].second], 1., 1., 0., line_id);
        PCL_INFO("Add %d/%d\n", i, pair.corresPointIdx.size());
    }
    PCL_INFO("Done!\n");
    viewer.spin();
    /*
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    */
    return 0;
}