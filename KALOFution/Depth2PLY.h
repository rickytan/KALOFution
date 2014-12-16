#pragma once
#ifndef _DEPTH2PLY_H_
#define _DEPTH2PLY_H_

#include "define.h"

#include <iostream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define KINECT_DEFAULT_DEPTH_FOCAL_X    (585.f)
#define KINECT_DEFAULT_DEPTH_FOCAL_Y    (585.f)

typedef pcl::PointXYZ Point;


class Depth2PLY
{
public:
    Depth2PLY() { setCameraParams(KINECT_DEFAULT_DEPTH_FOCAL_X, KINECT_DEFAULT_DEPTH_FOCAL_Y); }
    Depth2PLY(float fx, float fy) : m_width(640), m_height(480) { setCameraParams(fx, fy); }
    ~Depth2PLY() {}

    CloudTypePtr operator ()(int width, int height, unsigned short *depth);
    CloudTypePtr operator ()(int width, int height, float *depth);
    CloudTypePtr operator ()(int width, int height, std::vector<unsigned short> &depth);
    CloudTypePtr operator ()(int width, int height, std::vector<float> &depth);

private:
    void setCameraParams(float fx, float fy);
    Point uvd2point(int u, int v, float depth);
    Point uvd2point(int u, int v, unsigned short depth);
private:
    float m_fx, m_fy, m_cx, m_cy;
    int m_width, m_height;
    float * m_floatDepth;
    unsigned short * m_rawDepth;
};

#endif  // _DEPTH2PLY_H_
