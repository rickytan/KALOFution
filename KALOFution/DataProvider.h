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
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/console/print.h>

using namespace std;

class DataProvider
{
public:
    DataProvider() {}
    virtual ~DataProvider() {}

    const uint32_t size();
    CloudTypePtr operator [](uint32_t index) { return getCloudAtIndex(index); }

    virtual void prepareData() { m_dataInitailized = true; }   // override me!!!

    virtual const uint32_t numberOfClouds() = 0;
    virtual CloudTypePtr getCloudAtIndex(uint32_t index) = 0;
    virtual const Eigen::Affine3f initTransformOfCloudAtIndex(uint32_t index) = 0;

protected:
    bool m_dataInitailized;
};

class DefaultDataProvider : public DataProvider
{
public:

    DefaultDataProvider(uint32_t num, const std::string& data_dir, uint32_t step = 1)
        : m_numberOfClouds(num)
        , m_dataDirectory(data_dir)
        , m_step(step)
        , m_format("pcd")
        , m_posFile("camera_pos.txt")
    {}
    virtual ~DefaultDataProvider() {}

    virtual void prepareData();
    
    virtual const uint32_t numberOfClouds();
    virtual CloudTypePtr getCloudAtIndex(uint32_t index)
    {
        CloudTypePtr cloud(new CloudType);
        const std::string filename = filenameOfCloudAtIndex(index);
        PCL_INFO("Loading file : %s\n", filename.c_str());
        int size = -1;
        if (m_format == "ply") {
            size = pcl::io::loadPLYFile<PointType>(filename, *cloud);
        }
        else if (m_format == "pcd") {
            size = pcl::io::loadPCDFile<PointType>(filename, *cloud);
        }
        if (size < 0) {
            PCL_ERROR("Fail to load file : %s\n", filename.c_str());
        }
        return cloud;
    }
    virtual const Eigen::Affine3f initTransformOfCloudAtIndex(uint32_t index);

    void setPoseFile(const std::string &file) { m_posFile = file; }
private:
    const std::string filenameOfCloudAtIndex(uint32_t index);
    void initPoses();

private:
    string m_format;
    string m_posFile;
    uint32_t m_numberOfClouds;
    uint32_t m_step;
    std::string m_dataDirectory;
    std::vector<Eigen::Affine3f> m_cameraPoses;
};



#endif  //  _DATAPROVIDER_H_