#include "ONIDumper.h"
#include "Depth2PLY.h"

ONIDumper::ONIDumper(const std::string& onifile)
: grabber(onifile, false, true)
{
    boost::function<void(const boost::shared_ptr<openni_wrapper::DepthImage>&)> depthCb;

    depthCb = boost::bind(&ONIDumper::depthImageCallback, this, _1);
    grabber.registerCallback(depthCb);
}


ONIDumper::~ONIDumper()
{
    grabber.stop();
}

void ONIDumper::dumpTo(const std::string& dir)
{
    grabber.start();
}

void ONIDumper::depthImageCallback(const boost::shared_ptr<openni_wrapper::DepthImage>&)
{

}
