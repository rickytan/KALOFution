#include "Depth2PLY.h"

#include <limits>
#include <numeric>


#define NAN (std::numeric_limits<float>::quiet_NaN())

using namespace boost::math;

void Depth2PLY::setCameraParams(float fx, float fy)
{
    m_fx = fx;
    m_fy = fy;
    m_cx = m_width / 2 - 0.5f;
    m_cy = m_height / 2 - 0.5f;
}

template<typename DepthType>
Point Depth2PLY::uvd2point(int u, int v, DepthType depth)
{
    if (depth < std::numeric_limits<DepthType>::epsilon())
        return Point(NAN, NAN, NAN);
    DepthType x = (u - m_cx) / m_fx;
    DepthType y = (v - m_cy) / m_fy;
    DepthType z = depth;
    return Point(z * x, z * y, z);
}

template<>
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

template<>
Point Depth2PLY::uvd2point(int u, int v, unsigned short depth)
{
    if (depth == 0)
        return Point(NAN, NAN, NAN);
    return uvd2point(u, v, 0.001f * depth);
}

template<>
Point Depth2PLY::uvd2point(int u, int v, char depth)
{
    if (depth == 0)
        return Point(NAN, NAN, NAN);
    return uvd2point(u, v, 1.f * (unsigned char)depth / 255 * 5 + 0.3f);
}