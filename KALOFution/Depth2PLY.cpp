#include "Depth2PLY.h"


Depth2PLY::Depth2PLY(const std::string& filepath)
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
