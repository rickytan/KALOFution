#include "Depth2PLY.h"

#include <limits>
#include <numeric>

#include <boost/math/special_functions.hpp>

#define NAN (std::numeric_limits<float>::quiet_NaN())

using namespace boost::math;

CloudTypePtr Depth2PLY::operator()(int width, int height, unsigned short *depth)
{
    m_width = width;
    m_height = height;
    setCameraParams(m_fx, m_fy);

    CloudTypePtr cloudPtr(new CloudType);
    cloudPtr->height = 1;
    cloudPtr->is_dense = true;

    for (int row = 0; row < height - 1; ++row) {
        for (int col = 0; col < width - 1; ++col) {
            unsigned short d = depth[row * width + col];
            Point p00 = uvd2point(col, row, d);
            if (isnan(p00.x))
                continue;
            Point p01 = uvd2point(col + 1, row, d);
            if (isnan(p01))
                continue;
            Point p10 = uvd2point(col, row + 1, d);
            if (isnan(p10))
                continue;
            PointType point;
            point.x = p00.x;
            point.y = p00.y;
            point.z = p00.z;
            Eigen::Vector3f norm = (p01.getVector3fMap() - p00.getVector3fMap()).cross(p10.getVector3fMap() - p00.getVector3fMap());
            point.normal_x = norm[0];
            point.normal_y = norm[1];
            point.normal_z = norm[2];
            cloudPtr->points.push_back(point);
        }
    }
    cloudPtr->width = cloudPtr->points.size();
    return cloudPtr;
}

CloudTypePtr Depth2PLY::operator()(int width, int height, float *depth)
{
    m_width = width;
    m_height = height;
    setCameraParams(m_fx, m_fy);

    CloudTypePtr cloudPtr(new CloudType);
    cloudPtr->height = 1;
    cloudPtr->is_dense = true;

    for (int row = 0; row < height - 1; ++row) {
        for (int col = 0; col < width - 1; ++col) {
            float d = depth[row * width + col];
            Point p00 = uvd2point(col, row, d);
            if (isnan(p00.x))
                continue;
            Point p01 = uvd2point(col + 1, row, d);
            if (isnan(p01))
                continue;
            Point p10 = uvd2point(col, row + 1, d);
            if (isnan(p10))
                continue;
            PointType point;
            point.x = p00.x;
            point.y = p00.y;
            point.z = p00.z;
            Eigen::Vector3f norm = (p01.getVector3fMap() - p00.getVector3fMap()).cross(p10.getVector3fMap() - p00.getVector3fMap());
            point.normal_x = norm[0];
            point.normal_y = norm[1];
            point.normal_z = norm[2];
            cloudPtr->points.push_back(point);
        }
    }
    cloudPtr->width = cloudPtr->points.size();
    return cloudPtr;
}

CloudTypePtr Depth2PLY::operator()(int width, int height, std::vector<unsigned short> &depth)
{
    return Depth2PLY::operator ()(width, height, &depth[0]);
}

CloudTypePtr Depth2PLY::operator()(int width, int height, std::vector<float> &depth)
{
    return Depth2PLY::operator ()(width, height, &depth[0]);
}

void Depth2PLY::setCameraParams(float fx, float fy)
{
    m_fx = fx;
    m_fy = fy;
    m_cx = m_width / 2 - 0.5f;
    m_cy = m_height / 2 - 0.5f;
}

Point Depth2PLY::uvd2point(int u, int v, float depth)
{
    if (depth < std::numeric_limits<float>::epsilon())
        return Point(NAN, NAN, NAN);

    float x = (u - m_cx) / m_fx;
    float y = (v - m_cy) / m_fy;
    //float length = sqrtf(x * x + y * y + 1);
    float z = depth;// / length;
    return Point(z * x, z * y, z);
}

Point Depth2PLY::uvd2point(int u, int v, unsigned short depth)
{
    if (depth == 0)
        return Point(NAN, NAN, NAN);
    return uvd2point(u, v, 0.001f * depth);
}
