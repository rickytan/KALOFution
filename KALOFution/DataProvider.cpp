#include "DataProvider.h"

template <typename PointType>
const uint32_t DataProvider<PointType>::size()
{
    return this->numberOfClouds();
}

template <typename PointType>
const uint32_t DefaultDataProvider<PointType>::numberOfClouds()
{
    return this->m_numberOfClouds;
}

template <typename PointType>
const Eigen::Affine3f DefaultDataProvider<PointType>::initTransformOfCloudAtIndex(uint32_t index)
{
    return Eigen::Affine3f::Identity();
}

template <typename PointType>
const std::string DefaultDataProvider<PointType>::filenameOfCloudAtIndex(uint32_t index)
{
    char filepath[1024] = { 0 };
    sprintf_s(filepath, "%s/cloud_%d.ply", m_dataDirectory.c_str(), index);
    return string(filepath);
}
