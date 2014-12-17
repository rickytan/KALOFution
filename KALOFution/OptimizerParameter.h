#pragma once
#ifndef _OPTIMIZERPARAMETER_H_
#define _OPTIMIZERPARAMETER_H_

class Parameter;

class OptimizerParameter : public Parameter
{
public:
    OptimizerParameter()
        : maxIteration(50)
    {}
    virtual ~OptimizerParameter() {}

    virtual parse(int argc, char *argv[]);

    int maxIteration;
};

#endif  // _OPTIMIZERPARAMETER_H_
