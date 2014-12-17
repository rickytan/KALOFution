#pragma once
#ifndef _MP4DUMPER_H_
#define _MP4DUMPER_H_

#include <string>

class CvCapture;

class MP4Dumper
{
public:
    MP4Dumper(const std::string& mp4file);
    ~MP4Dumper();

    void dumpTo(const std::string& dir);

private:
    CvCapture *m_videoCapture;
};

#endif  // _MP4DUMPER_H_
