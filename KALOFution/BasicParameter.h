#pragma once
#ifndef _BASICPARAMETER_H_
#define _BASICPARAMETER_H_

#include <iostream>
#include <string>

#include "Parameter.h"

class BasicParameter : public Parameter
{
public:
    BasicParameter()
        : acceptableICPPointDistThres(0.05)
        , corresPointDistThres(0.015)
        , corresPointNormThres(30.f)
        , saveCorresPointIndices(true)
        , corresPointSavePath("./corres")
        , cloudPairNeedAlignment(true)
        , acceptableCorresPointNum(40000)
        , acceptableCorresPointRatio(0.3)
        , translationThres(0.24)
        , rotationThres(15)
        , maxICPIteration(50)
    {}
    virtual ~BasicParameter() {}

    virtual void help();
    virtual void parse(int argc, char *argv[]);

    float translationThres;
    float rotationThres;

    float acceptableICPPointDistThres;
    float corresPointDistThres;
    float corresPointNormThres; // 对应点法向角度差
    bool saveCorresPointIndices;
    std::string corresPointSavePath;
    int acceptableCorresPointNum;
    int maxICPIteration;
    float acceptableCorresPointRatio;
    bool cloudPairNeedAlignment;
};

#endif  // _BASICPARAMETER_H_
