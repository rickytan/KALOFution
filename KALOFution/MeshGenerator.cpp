#include "MeshGenerator.h"

#include <pcl/surface/gp3.h>
#include <pcl/kdtree/kdtree_flann.h>

MeshGenerator::MeshGenerator()
{
}


MeshGenerator::~MeshGenerator()
{
}

pcl::PolygonMeshPtr MeshGenerator::generateMesh()
{
    CloudTypePtr merged_cloud = mergeAllClouds();
    m_cloudList.clear();

    // Create search tree*
    pcl::search::KdTree<PointType>::Ptr tree2(new pcl::search::KdTree<PointType>);
    tree2->setInputCloud(merged_cloud);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<PointType> gp3;
    pcl::PolygonMeshPtr triangles(new pcl::PolygonMesh);

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(0.5);

    // Set typical values for the parameters
    gp3.setMu(4);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(45. * M_PI / 180); // 45 degrees
    gp3.setMinimumAngle(10. * M_PI / 180); // 10 degrees
    gp3.setMaximumAngle(120. * M_PI / 180); // 120 degrees
    gp3.setNormalConsistency(true);

    // Get result
    gp3.setInputCloud(merged_cloud);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(*triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    return triangles;
}

CloudTypePtr MeshGenerator::mergeAllClouds()
{
    CloudTypePtr merged_cloud(new CloudType);
    merged_cloud->height = 1;
    merged_cloud->is_dense = true;

    size_t total_size = 0;
    for (std::list<CloudTypePtr>::iterator it = m_cloudList.begin(); it != m_cloudList.end(); ++it) {
        total_size += (*it)->size();
    }
    merged_cloud->points.resize(total_size);

    size_t count = 0;
    for (std::list<CloudTypePtr>::iterator it = m_cloudList.begin(); it != m_cloudList.end(); ++it) {
        std::copy((*it)->points.begin(), (*it)->points.end(), merged_cloud->begin() + count);
        count += (*it)->size();
    }
    merged_cloud->width = merged_cloud->points.size();

    return merged_cloud;
}
