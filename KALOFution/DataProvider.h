#ifndef _DATAPROVIDER_H_
#define _DATAPROVIDER_H_

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <Eigen/Dense>

#include "define.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>

using namespace std;

class DataProvider
{
public:
    DataProvider() {}
    virtual ~DataProvider() {}

    const uint32_t size();
    CloudTypePtr operator [](uint32_t index) { return getCloudAtIndex(index); }

    virtual const uint32_t numberOfClouds() = 0;
    virtual CloudTypePtr getCloudAtIndex(uint32_t index) = 0;
    virtual const Eigen::Affine3f initTransformOfCloudAtIndex(uint32_t index) = 0;
};

class DefaultDataProvider : public DataProvider
{
public:

    DefaultDataProvider(uint32_t num, const std::string& data_dir)
    : m_numberOfClouds(num)
    , m_dataDirectory(data_dir)
    {}
    virtual ~DefaultDataProvider() {}
    
    virtual const uint32_t numberOfClouds();
    virtual CloudTypePtr getCloudAtIndex(uint32_t index)
    {
        CloudTypePtr cloud(new CloudType);
        const std::string filename = filenameOfCloudAtIndex(index);
        PCL_INFO("Loading file : %s\n", filename.c_str());
        int size = pcl::io::loadPLYFile(filename, *cloud);
        if (size < 0) {
            PCL_ERROR("Fail to load file : %s\n", filename.c_str());
        }
        return cloud;
    }
    virtual const Eigen::Affine3f initTransformOfCloudAtIndex(uint32_t index);

private:
    const std::string filenameOfCloudAtIndex(uint32_t index);


private:
    uint32_t m_numberOfClouds;
    std::string m_dataDirectory;
};

#endif  //  _DATAPROVIDER_H_