#include "DataProvider.h"

const uint32_t DataProvider::size()
{
    return this->numberOfClouds();
}

const uint32_t DefaultDataProvider::numberOfClouds()
{
    return this->m_numberOfClouds;
}

const Eigen::Affine3f DefaultDataProvider::initTransformOfCloudAtIndex(uint32_t index)
{
    return Eigen::Affine3f::Identity();
}

const std::string DefaultDataProvider::filenameOfCloudAtIndex(uint32_t index)
{
    char filepath[1024] = { 0 };
    sprintf(filepath, "%s/cloud_%d.ply", m_dataDirectory.c_str(), index * m_step);
    return string(filepath);
}
