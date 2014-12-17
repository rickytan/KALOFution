#pragma once
#ifndef _OPTIMIZERPARAMETER_H_
#define _OPTIMIZERPARAMETER_H_

#include "Parameter.h"

class OptimizerParameter : public Parameter
{
public:
    OptimizerParameter()
        : maxIteration(50)
    {}
    virtual ~OptimizerParameter() {}

    virtual void parse(int argc, char *argv[]);

    int maxIteration;
};

#endif  // _OPTIMIZERPARAMETER_H_
