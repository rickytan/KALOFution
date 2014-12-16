#include "Depth2PLY.h"


Depth2PLY::Depth2PLY(const std::string& filepath)
{
}

Depth2PLY::Depth2PLY(const int width, const int height, unsigned short *depth)
: m_width(width)
, m_height(height)
, m_floatDepth(NULL)
, m_rawDepth(depth)
{
}

Depth2PLY::Depth2PLY(const int width, const int height, std::vector<unsigned short> &depth)
: m_width(width)
, m_height(height)
, m_floatDepth(NULL)
, m_rawDepth(&depth[0])
{
}

Depth2PLY::Depth2PLY(const int width, const int height, float *depth)
: m_width(width)
, m_height(height)
, m_floatDepth(depth)
, m_rawDepth(NULL)
{
}

Depth2PLY::Depth2PLY(const int width, const int height, std::vector<float> &depth)
: m_width(width)
, m_height(height)
, m_floatDepth(&depth[0])
, m_rawDepth(NULL)
{
}


Depth2PLY::~Depth2PLY()
{
}

Depth2PLY::operator CloudTypePtr()
{
    CloudTypePtr cloudPtr(new CloudType);

    return cloudPtr;
}
