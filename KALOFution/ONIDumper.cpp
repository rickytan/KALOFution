#include "ONIDumper.h"
#include "Depth2PLY.h"

#include <pcl/io/ply_io.h>

ONIDumper::ONIDumper(const std::string& onifile)
: m_grabber(onifile, false, true)
, m_condDataReady()
, m_mutexDataReady()
, m_rawDepthData()
, m_width(0)
, m_height(0)
{
    boost::function<void(const boost::shared_ptr<openni_wrapper::DepthImage>&)> depthCb;
    depthCb = boost::bind(&ONIDumper::depthImageCallback, this, _1);
    m_grabber.registerCallback(depthCb);
}


ONIDumper::~ONIDumper()
{
    m_grabber.stop();
}

void ONIDumper::dumpTo(const std::string& dir)
{
    m_dumpDirectory = dir;

    boost::unique_lock<boost::mutex> lock(m_mutexDataReady);

    m_grabber.start();
    while (m_grabber.isRunning()) {
        bool has_data = m_condDataReady.timed_wait(lock, boost::posix_time::microsec(300));
        try {
            execute(has_data);
        }
        catch (std::bad_alloc) {
            PCL_ERROR("Out of memory!\n");
        }
        catch (std::exception e) {
            PCL_ERROR("Exception : %s\n", e.what());
        }
        catch (...) {
            PCL_ERROR("Unkowne Error\n");
        }
    }
}

void ONIDumper::depthImageCallback(const boost::shared_ptr<openni_wrapper::DepthImage>& depth_wrapper)
{
    {
        boost::mutex::scoped_try_lock lock(m_mutexDataReady);
        if (!lock)
            return;

        if (m_width != depth_wrapper->getWidth() || m_height != depth_wrapper->getHeight()) {
            m_width = depth_wrapper->getWidth();
            m_height = depth_wrapper->getHeight();
            m_rawDepthData.resize(m_width * m_height);
        }

        depth_wrapper->fillDepthImageRaw(m_width, m_height, &m_rawDepthData[0]);
    }
    m_condDataReady.notify_one();
}

void ONIDumper::execute(bool hasdata)
{
    if (!hasdata)
        return;

    static int frameCount = 0;
    CloudTypePtr cloud = Depth2PLY(m_width, m_height, m_rawDepthData);
    char filepath[1024] = { 0 };
    sprintf(filepath, "%s/cloud_%d.ply", m_dumpDirectory.c_str(), frameCount++);
    pcl::io::savePLYFileBinary(filepath, *cloud);
}
