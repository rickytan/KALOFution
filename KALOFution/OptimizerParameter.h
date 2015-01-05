#pragma once
#ifndef _OPTIMIZERPARAMETER_H_
#define _OPTIMIZERPARAMETER_H_

#include "Parameter.h"

#include <string>

using namespace std;

class OptimizerParameter : public Parameter
{
public:
    OptimizerParameter()
        : maxIteration(8)
        , saveDirectory("./optimized")
        , useCholmod(false)
    {}
    virtual ~OptimizerParameter() {}

    virtual void help();
    virtual void parse(int argc, char *argv[]);

    int maxIteration;
    string saveDirectory;
    bool useCholmod;
};

#endif  // _OPTIMIZERPARAMETER_H_
