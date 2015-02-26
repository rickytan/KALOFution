#include "MapDumper.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <pcl/console/print.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

void MapDumper::dumpTo(const string &dump_dir)
{
    if (m_cameraPosFile.length())
        initPoses();

    m_dumpDir = dump_dir;

    if (boost::filesystem::exists(m_dumpDir) && !boost::filesystem::is_directory(m_dumpDir)) {
            boost::filesystem::remove(m_dumpDir);
    }
    boost::filesystem::create_directories(m_dumpDir);

    using namespace boost::filesystem;

    int map_index = m_dumpStart;
    int cloud_index = 0;
    size_t total = m_cameraPoses.size() ? m_cameraPoses.size() : std::count_if(
        directory_iterator(m_mapDir),
        directory_iterator(),
        boost::bind(static_cast<bool(*)(const path&)>(is_regular_file), boost::bind(&directory_entry::path, _1))) / 2;

    boost::thread_group group;

    int cpu_cores = std::max(boost::thread::hardware_concurrency(), (unsigned)2);
    int count = 0;
    for (; map_index + m_dumpStep < total; map_index += m_dumpStep) {
        if (count == cpu_cores) {
            count = 0;
            group.join_all();
        }
        group.create_thread(boost::bind(&MapDumper::forEachMap, this, map_index));
        ++count;
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

    int nan_count = 0;
    int invalid_norm_count = 0;
    int invalid_depth_count = 0;
    for (int row = 0; row < 480; ++row) {
        for (int col = 0; col < 640; ++col) {
            Eigen::Vector3f norm;
            PointType point;
            point.normal_x = nmap[640 * (0 * 480 + row) + col];
            if (pcl_isnan(point.normal_x)) {
                ++nan_count;
                continue;
            }
            point.normal_y = nmap[640 * (1 * 480 + row) + col];
            point.normal_z = nmap[640 * (2 * 480 + row) + col];

            point.x  = vmap[640 * (0 * 480 + row) + col];

            if (pcl_isnan(point.x)) {
                ++nan_count;
                continue;
            }
            point.y = vmap[640 * (1 * 480 + row) + col];
            point.z = vmap[640 * (2 * 480 + row) + col];
            if (point.z < m_minDepth || point.z > m_maxDepth) {
                ++invalid_depth_count;
                continue;
            }
            float thre_cos = cosf(m_normAngleThres * M_PI / 180);
            if (fabsf(point.normal_z) < thre_cos) {
                ++invalid_norm_count;
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

    {
        boost::mutex::scoped_lock lock(m_outputMutex);
        PCL_INFO("\n\tCloud made with %d NaN, %d invalid depth, %d invalid norm points skipped!\n", nan_count, invalid_depth_count, invalid_norm_count);
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
    if (!vertex_file.is_open()) {
        PCL_WARN("Can't open vertex map file : %s, skipped!", filename);
        return;
    }
    std::vector<float> point_data;
    point_data.resize(640 * 480 * 3);
    vertex_file.read((char *)&point_data[0], point_data.size() * sizeof(float));
    vertex_file.close();

    std::ifstream norm_file;
    sprintf(filename, "%s/cloud_%d.nmap", m_mapDir.c_str(), index);
    norm_file.open(filename, ios::in | ios::binary);
    if (!norm_file.is_open()) {
        PCL_WARN("Can't open normal map file : %s, skipped!", filename);
        return;
    }
    std::vector<float> norm_data;
    norm_data.resize(640 * 480 * 3);
    norm_file.read((char *)&norm_data[0], norm_data.size() * sizeof(float));
    norm_file.close();

    CloudTypePtr cloud = mapToCloud(point_data, norm_data);
    if (m_shouldFilter) {
        cloud = filterByRemoveOutlier(cloud);
    }
    CloudTypePtr transformed = cloud;
    if (m_cameraPoses.size() > index) {
        transformed.reset(new CloudType);
        pcl::transformPointCloudWithNormals(*cloud, *transformed, m_cameraPoses[index]);
    }
    sprintf(filename, "%s/cloud_%d.%s", m_dumpDir.c_str(), index++, m_dumpFormat.c_str());
    if (m_dumpFormat == "pcd") {
        pcl::io::savePCDFileBinary(filename, *transformed);
    }
    else if (m_dumpFormat == "ply") {
        pcl::io::savePLYFileBinary(filename, *transformed);
    }
}

void MapDumper::initPoses()
{
    m_cameraPoses.clear();

    FILE *fp = fopen(m_cameraPosFile.c_str(), "r");
    if (!fp) {
        PCL_ERROR("Can't open file : %s\n", m_cameraPosFile.c_str());
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

CloudTypePtr MapDumper::filterByRemoveOutlier(CloudTypePtr &incloud)
{
    // point cloud instance for the result
    CloudTypePtr cleaned(new CloudType);
    // create the radius outlier removal filter
    pcl::RadiusOutlierRemoval<PointType> radius_outlier_removal;
    // set input cloud
    radius_outlier_removal.setInputCloud(incloud);
    // set radius for neighbor search
    radius_outlier_removal.setRadiusSearch(0.05);
    // set threshold for minimum required neighbors neighbors
    radius_outlier_removal.setMinNeighborsInRadius(200);
    // do filtering
    radius_outlier_removal.filter(*cleaned);
    return cleaned;
}
