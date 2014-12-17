#pragma once
#ifndef _PARAMETER_H_
#define _PARAMETER_H_

class Parameter
{
public:
    Parameter() {}
    virtual ~Parameter() {}

    virtual void parse(int argc, char *argv[]) = 0;
};

#endif  // _PARAMETER_H_
