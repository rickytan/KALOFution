#include "MapDumper.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <pcl/console/print.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

void MapDumper::dumpTo(const string &dump_dir)
{
    //initPoses();
    m_dumpDir = dump_dir;

    if (boost::filesystem::exists(m_dumpDir)) {
        if (!boost::filesystem::is_directory(m_dumpDir)) {
            boost::filesystem::remove(m_dumpDir);
            boost::filesystem::create_directory(m_dumpDir);
        }
    }
    else {
        boost::filesystem::create_directory(m_dumpDir);
    }

    int count = 0;
    int cloud_index = 0;
    size_t total = m_cameraPoses.size();
    boost::thread_group group;

    for (; count + m_dumpStep < total; count += m_dumpStep) {
        group.create_thread(boost::bind(&MapDumper::forEachMap, this, count));
    }
    group.join_all();
}

CloudTypePtr MapDumper::mapToCloud(std::vector<float> &vmap, std::vector<float> &nmap)
{
    assert(vmap.size() == nmap.size());

    CloudTypePtr cloud(new CloudType);
    cloud->height = 1;
    cloud->is_dense = true;
    cloud->points.reserve(vmap.size());

    for (int row = 0; row < 480; ++row) {
        for (int col = 0; col < 640; ++col) {
            Eigen::Vector3f norm;
            PointType point;
            point.normal_x = nmap[640 * (0 * 480 + row) + col];
            if (isnan(point.normal_x)) {
                continue;
            }
            point.normal_y = nmap[640 * (1 * 480 + row) + col];
            point.normal_z = nmap[640 * (2 * 480 + row) + col];

            point.x  = vmap[640 * (0 * 480 + row) + col];

            if (isnan(point.x)) {
                continue;
            }
            point.y = vmap[640 * (1 * 480 + row) + col];
            point.z = vmap[640 * (2 * 480 + row) + col];
            if (point.z < m_minDepth || point.z > m_maxDepth) {
                continue;
            }
            float thre_cos = cosf(m_normAngleThres);
            if (fabsf(point.normal_z) < thre_cos) {
                continue;
            }
            if (point.normal_z > 0) {
                point.normal_x = -point.normal_x;
                point.normal_y = -point.normal_y;
                point.normal_z = -point.normal_z;
            }
            cloud->points.push_back(point);
        }
    }

    cloud->width = cloud->points.size();
    return cloud;
}

void MapDumper::forEachMap(int index)
{
    char filename[1024] = { 0 };
    std::ifstream vertex_file;
    sprintf(filename, "%s/cloud_%d.vmap", m_mapDir.c_str(), index);
    vertex_file.open(filename, ios::in | ios::binary);
    std::vector<float> point_data;
    point_data.resize(640 * 480 * 3);
    vertex_file.read((char *)&point_data[0], point_data.size() * sizeof(float));
    vertex_file.close();

    std::ifstream norm_file;
    sprintf(filename, "%s/cloud_%d.nmap", m_mapDir.c_str(), index);
    norm_file.open(filename, ios::in | ios::binary);
    std::vector<float> norm_data;
    norm_data.resize(640 * 480 * 3);
    norm_file.read((char *)&norm_data[0], norm_data.size() * sizeof(float));
    norm_file.close();

    CloudTypePtr cloud = mapToCloud(point_data, norm_data);
    //CloudType transformed;
    //pcl::transformPointCloudWithNormals(*cloud, transformed, m_cameraPoses[index]);
    sprintf(filename, "%s/cloud_%d.%s", m_dumpDir.c_str(), index++, m_dumpFormat.c_str());
    if (m_dumpFormat == "pcd") {
        pcl::io::savePCDFileBinary(filename, cloud);
    }
    else if (m_dumpFormat == "ply") {
        pcl::io::savePLYFileBinary(filename, cloud);
    }
}

void MapDumper::initPoses()
{
    m_cameraPoses.clear();

    FILE *fp = fopen(m_posFile.c_str(), "r");
    if (!fp) {
        PCL_ERROR("Can't open file : %s\n", m_posFile.c_str());
        exit(-1);
    }
    while (!feof(fp)) {
        float tx, ty, tz, w, x, y, z;
        fscanf(fp, "%f, %f, %f, %f, %f, %f, %f", &tx, &ty, &tz, &w, &x, &y, &z);
        Eigen::Vector3f trans(tx, ty, tz);
        Eigen::Quaternionf rot(w, x, y, z);
        Eigen::Affine3f affine;
        affine.linear() = rot.toRotationMatrix();
        affine.translation() = trans;
        m_cameraPoses.push_back(affine);
    }
    fclose(fp);
}
