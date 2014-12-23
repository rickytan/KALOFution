#pragma once
#ifndef _DEPTH2PLY_H_
#define _DEPTH2PLY_H_

#include "define.h"

#include <iostream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define KINECT_DEFAULT_DEPTH_FOCAL_X    (575.816f)
#define KINECT_DEFAULT_DEPTH_FOCAL_Y    (575.816f)

typedef pcl::PointXYZ Point;

class Depth2PLY
{
public:
    Depth2PLY() { setCameraParams(KINECT_DEFAULT_DEPTH_FOCAL_X, KINECT_DEFAULT_DEPTH_FOCAL_Y); }
    Depth2PLY(float fx, float fy) : m_width(640), m_height(480) { setCameraParams(fx, fy); }
    ~Depth2PLY() {}

    template<typename DepthType>
    inline CloudTypePtr operator ()(int width, int height, DepthType *depth) {
        return operator() (width, height, depth, width);
    }
    template<typename DepthType>
    CloudTypePtr operator ()(int width, int height, DepthType *depth, int stride) {

        m_width = width;
        m_height = height;
        setCameraParams(m_fx, m_fy);

        CloudTypePtr cloudPtr(new CloudType);
        cloudPtr->height = 1;
        cloudPtr->is_dense = true;

        for (int row = 0; row < height - 1; ++row) {
            for (int col = 0; col < width - 1; ++col) {
                DepthType d = depth[row * stride + col];
                Point p00 = uvd2point(col, row, d);
                if (pcl_isnan(p00.x))
                    continue;
                Point p01 = uvd2point(col + 1, row, d);
                if (pcl_isnan(p01.x))
                    continue;
                Point p10 = uvd2point(col, row + 1, d);
                if (pcl_isnan(p10.x))
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

    template<typename DepthType>
    CloudTypePtr operator ()(int width, int height, std::vector<DepthType> &depth) {
        return operator() (width, height, &depth[0]);
    }

private:
    void setCameraParams(float fx, float fy);
    template<typename DepthType>
    Point uvd2point(int u, int v, DepthType depth);
    bool estimateNorm(int u, int v, Eigen::Vector3f &norm);
private:
    float m_fx, m_fy, m_cx, m_cy;
    int m_width, m_height;
    float * m_floatDepth;
    unsigned short * m_rawDepth;
};


#endif  // _DEPTH2PLY_H_
