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
    if (!m_dataInitailized) {
        prepareData();
    }
    return m_cameraPoses[index * m_step];
}

const std::string DefaultDataProvider::filenameOfCloudAtIndex(uint32_t index)
{
    char filepath[1024] = { 0 };
    sprintf(filepath, "%s/cloud_%d.%s", m_dataDirectory.c_str(), index * m_step, m_format.c_str());
    return string(filepath);
}

void DefaultDataProvider::prepareData()
{
    initPoses();
    DataProvider::prepareData();
}

void DefaultDataProvider::initPoses()
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