#ifndef _DATAPROVIDER_H_
#define _DATAPROVIDER_H_

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>

using namespace std;

template <typename PointType>
class DataProvider
{
public:
    typedef pcl::PointCloud<PointType> CloudType;
    typedef typename CloudType::Ptr CloudTypePtr;

    const uint32_t size();
    CloudTypePtr operator [](uint32_t index) const { return getCloudAtIndex(index); }
    CloudTypePtr& operator[](uint32_t index);

    virtual const uint32_t numberOfClouds() = 0;
    virtual CloudTypePtr getCloudAtIndex(uint32_t index) = 0;
    virtual const Eigen::Affine3f initTransformOfCloudAtIndex(uint32_t index) = 0;
};

template <typename PointType>
class DefaultDataProvider : public DataProvider<PointType>
{
public:

    DefaultDataProvider(uint32_t num, const std::string& data_dir)
    : m_numberOfClouds(num)
    , m_dataDirectory(data_dir)
    {}
    virtual ~DefaultDataProvider() {}
    
    const uint32_t numberOfClouds();
    CloudTypePtr getCloudAtIndex(uint32_t index)
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
    const Eigen::Affine3f initTransformOfCloudAtIndex(uint32_t index);

private:
    const std::string filenameOfCloudAtIndex(uint32_t index);


private:
    uint32_t m_numberOfClouds;
    std::string m_dataDirectory;
};

#endif  //  _DATAPROVIDER_H_