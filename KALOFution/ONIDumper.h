#pragma once
#ifndef _ONIDUMPER_H_
#define _ONIDUMPER_H_

#include <string>

#include <pcl/io/oni_grabber.h>

class ONIDumper
{
public:
    ONIDumper(const std::string& onifile);
    ~ONIDumper();

    void dumpTo(const std::string& dir);

private:
    void depthImageCallback(const boost::shared_ptr<openni_wrapper::DepthImage>&);
    void execute(bool hasdata);

private:
    pcl::ONIGrabber m_grabber;
    boost::condition_variable m_condDataReady;
    boost::mutex m_mutexDataReady;
    std::vector<unsigned short> m_rawDepthData;
    int m_width, m_height;
};

#endif  // _ONIDUMPER_H_
