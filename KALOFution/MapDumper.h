#pragma once
#ifndef _MAPDUMPER_H_
#define _MAPDUMPER_H_

#include <string>
#include <fstream>
#include <iostream>

#include "define.h"

#include <boost/thread.hpp>

class boost::mutex;

using namespace std;

class MapDumper
{
public:
    MapDumper(const string &map_dir)
        : m_mapDir(map_dir)
        , m_dumpStep(1)
        , m_minDepth(0.3)
        , m_maxDepth(3)
        , m_dumpFormat("pcd")
        , m_normAngleThres(80)
    {}
    ~MapDumper() {}

    void dumpTo(const string &dump_dir);
    void setStep(int step) { m_dumpStep = step; }
    void setMaxClipDepth(float depth) { m_maxDepth = depth; }
    void setMinClipDepth(float depth) { m_minDepth = depth; }
    void setDumpFormat(const string &fmt) { m_dumpFormat = fmt; }

private:
    CloudTypePtr mapToCloud(std::vector<float> &vmap, std::vector<float> &nmap);
    void forEachMap(int map_file_index);

private:
    string m_mapDir;
    string m_dumpDir;
    string m_dumpFormat;
    int m_dumpStep;
    std::vector<Eigen::Affine3f> m_cameraPoses;
    float m_minDepth, m_maxDepth;
    float m_normAngleThres;     // the angle between norm and view direction

    boost::mutex m_outputMutex;
};

#endif  // _MAPDUMPER_H_
