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

private:
    pcl::ONIGrabber grabber;
};

#endif  // _ONIDUMPER_H_
