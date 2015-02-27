#pragma once
#ifndef _MAPDUMPER_H_
#define _MAPDUMPER_H_

#include <string>
#include <fstream>
#include <iostream>

#include "define.h"

#include <boost/thread.hpp>

using namespace std;

class MapDumper
{
public:
    MapDumper(const string &map_dir)
        : m_mapDir(map_dir)
        , m_dumpStep(1)
        , m_dumpStart(0)
        , m_minDepth(0.8)
        , m_maxDepth(2.4)
        , m_dumpFormat("pcd")
        , m_normAngleThres(80)
    {}
    ~MapDumper() {}

    void dumpTo(const string &dump_dir);
    void setStep(int step) { m_dumpStep = step; }
    void setStart(int start) { m_dumpStart = start; }
    void setMaxClipDepth(float depth) { m_maxDepth = depth; }
    void setMinClipDepth(float depth) { m_minDepth = depth; }
    void setDumpFormat(const string &fmt) { m_dumpFormat = fmt; }
    void setCameraPosFile(const string &file) { m_cameraPosFile = file; }
    void setShouldFilter(const bool filter) { m_shouldFilter = filter; }
private:
    CloudTypePtr mapToCloud(std::vector<float> &vmap, std::vector<float> &nmap);
    CloudTypePtr filterByRemoveOutlier(CloudTypePtr &incloud);
    void forEachMap(int map_file_index);
    void initPoses();
private:
    string m_mapDir;
    string m_dumpDir;
    string m_dumpFormat;
    string m_cameraPosFile;
    int m_dumpStep;
    int m_dumpStart;
    std::vector<Eigen::Affine3f> m_cameraPoses;
    float m_minDepth, m_maxDepth;
    float m_normAngleThres;     // the angle between norm and view direction
    bool m_shouldFilter;
    boost::mutex m_outputMutex;
};

#endif  // _MAPDUMPER_H_
