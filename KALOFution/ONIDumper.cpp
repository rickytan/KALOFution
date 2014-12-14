#include "ONIDumper.h"
#include "Depth2PLY.h"

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

}

void ONIDumper::execute(bool hasdata)
{
    if (!hasdata)
        return;


}
