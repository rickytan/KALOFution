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
    ~Depth2PLY();

    operator CloudTypePtr ();

};

#endif  // _DEPTH2PLY_H_
