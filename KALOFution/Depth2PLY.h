#pragma once
#ifndef _DEPTH2PLY_H_
#define _DEPTH2PLY_H_

#include "define.h"

#include <iostream>
#include <string>

#include <pcl/point_cloud.h>

class Depth2PLY
{
public:
    Depth2PLY(const std::string &filepath);
    Depth2PLY(const int width, const int height, unsigned short *depth);
    Depth2PLY(const int width, const int height, std::vector<unsigned short> &depth);
    Depth2PLY(const int width, const int height, float *depth);
    Depth2PLY(const int width, const int height, std::vector<float> &depth);
    ~Depth2PLY();

    operator CloudTypePtr ();

private:

private:
    int m_width, m_height;
    float * m_floatDepth;
    unsigned short * m_rawDepth;
};

#endif  // _DEPTH2PLY_H_
