#pragma once
#ifndef _PARAMETER_H_
#define _PARAMETER_H_

#include <iostream>

class Parameter
{
public:
    Parameter() {}
    virtual ~Parameter() {}

    virtual void help() { std::cout << "No help info." << std::endl; exit(0); }  // override me!!
    virtual void parse(int argc, char *argv[]) = 0;
};

#endif  // _PARAMETER_H_
