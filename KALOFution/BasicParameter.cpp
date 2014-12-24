#include "Parameter.h"
#include "BasicParameter.h"

#include <pcl/console/parse.h>
#include <pcl/console/print.h>

#include <boost/filesystem.hpp>

using namespace pcl::console;

void BasicParameter::parse(int argc, char *argv[])
{
    if (find_switch(argc, argv, "-h") || find_switch(argc, argv, "--help")) {
        help();
    }
    parse_argument(argc, argv, "--dist-thres", corresPointDistThres);
    parse_argument(argc, argv, "--angle-thres", corresPointNormThres);
    parse_argument(argc, argv, "--icp-itera", maxICPIteration);
    parse_argument(argc, argv, "--valid-pair-dist-thres", acceptableICPPointDistThres);
    parse_argument(argc, argv, "--save-to", corresPointSavePath);

    if (boost::filesystem::exists(corresPointSavePath) && !boost::filesystem::is_directory(corresPointSavePath)) {
        boost::filesystem::remove(corresPointSavePath);
    }
    boost::filesystem::create_directories(corresPointSavePath);
}

void BasicParameter::help()
{
    printf("\
Options:\n\
    --dist-thres <num> :            default 0.15\n\
    --angle-thres <string> :        default 30 deg\n\
    --icp-itera <num> :             default 20\n\
    --valid-pair-dist-thres <num> : default 0.05\n\
    --save-path <string> :          default ./corres\n");
    exit(0);
}
