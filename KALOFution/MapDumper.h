#pragma once
#ifndef _MAPDUMPER_H_
#define _MAPDUMPER_H_

#include <string>
#include <fstream>
#include <iostream>

#include "define.h"

using namespace std;

class MapDumper
{
public:
    MapDumper(const string &map_dir, const string &posfile)
        : m_mapDir(map_dir)
        , m_posFile(posfile)
        , m_dumpStep(1)
        , m_minDepth(0.3)
        , m_maxDepth(4)
    {}
    ~MapDumper() {}

    void dumpTo(const string &dump_dir);
    void setStep(int step) { m_dumpStep = step; }
    void setMaxClipDepth(float depth) { m_maxDepth = depth; }
    void setMinClipDepth(float depth) { m_minDepth = depth; }
private:
    CloudTypePtr mapToCloud(std::vector<float> &vmap, std::vector<float> &nmap);
    void forEachMap(int map_file_index);
    void initPoses();
private:
    string m_mapDir;
    string m_posFile;
    string m_dumpDir;
    int m_dumpStep;
    std::vector<Eigen::Affine3f> m_cameraPoses;
    float m_minDepth, m_maxDepth;
};

#endif  // _MAPDUMPER_H_
