#pragma once
#ifndef _BASICPARAMETER_H_
#define _BASICPARAMETER_H_

#include <iostream>
#include <string>

class Parameter

class BasicParameter : public Parameter
{
public:
    BasicParameter()
        : acceptableCorresPointDistThres(0.05)
        , corresPointDistThres(0.015)
        , corresPointNormThres(30.f)
        , saveCorresPointIndices(true)
        , corresPointSavePath("./")
        , cloudPairNeedAlignment(false)
        , acceptableCorresPointNum(40000)
        , acceptableCorresPointRatio(0.3)
        , maxICPIteration(20)
    {}
    virtual ~BasicParameter() {}

    virtual parse(int argc, char *argv[]);

    float acceptableCorresPointDistThres;
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
