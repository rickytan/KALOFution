#pragma once
#ifndef _OPTIMIZERPARAMETER_H_
#define _OPTIMIZERPARAMETER_H_

class OptimizerParameter
{
public:
    OptimizerParameter()
        : maxIteration(50)
    {}

    int maxIteration;
};

#endif  // _OPTIMIZERPARAMETER_H_
