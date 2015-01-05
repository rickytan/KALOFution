#pragma once
#ifndef _MESHGENERATOR_H
#define _MESHGENERATOR_H

#include <list>
#include <string>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/file_io.h>
#include <pcl/console/print.h>

#include <boost/filesystem.hpp>


#include "define.h"

using namespace std;

class MeshGenerator
{
public:
    MeshGenerator();
    ~MeshGenerator();

    void addPointCloud(CloudTypePtr cloud) { m_cloudList.push_back(cloud); }
    void addPointCloudFromPCD(const std::string &filepath) {
        CloudTypePtr cloud(new CloudType);
        pcl::io::loadPCDFile(filepath, *cloud);
        addPointCloud(cloud);
    }
    void addPointCloudFromPLY(const std::string &filepath) {
        CloudTypePtr cloud(new CloudType);
        pcl::io::loadPLYFile(filepath, *cloud);
        addPointCloud(cloud);
    }

    void addPointCloudsFromDirectory(const std::string &directory, const std::string &fileext) {
        namespace fs = boost::filesystem;
        if (!fs::exists(directory) || !fs::is_directory(directory)) {
            PCL_ERROR("Directory not exist!");
            return;
        }

        fs::directory_iterator it(directory);
        fs::directory_iterator end;

        PCL_INFO("Listing directory : %s\n", directory.c_str());
        while (it != end) {
            if (fs::is_regular_file(*it) && it->path().extension() == "." + fileext) {
                PCL_INFO("\tadd %s\n", it->path().string().c_str());
                if (string("pcd") == fileext) {
                    addPointCloudFromPCD(it->path().string());
                }
                else if (string("ply") == fileext) {
                    addPointCloudFromPLY(it->path().string());
                }
            }
            ++it;
        }
    }

    pcl::PolygonMeshPtr generateMesh();
private:
    CloudTypePtr mergeAllClouds();

private:
    std::list<CloudTypePtr> m_cloudList;
};

#endif
