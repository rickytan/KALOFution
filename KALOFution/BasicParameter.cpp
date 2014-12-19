#include "Parameter.h"
#include "BasicParameter.h"

#include <pcl/console/parse.h>
#include <pcl/console/print.h>

#include <boost/filesystem.hpp>

using namespace pcl::console;

void BasicParameter::parse(int argc, char *argv[])
{
    parse_argument(argc, argv, "--dist-thres", corresPointDistThres);
    parse_argument(argc, argv, "--angle-thres", corresPointNormThres);
    parse_argument(argc, argv, "--icp-itera", maxICPIteration);
    parse_argument(argc, argv, "--valid-pair-dist-thres", acceptableCorresPointDistThres);
    parse_argument(argc, argv, "--save-path", corresPointSavePath);

    if (boost::filesystem::exists(corresPointSavePath)) {
        if (!boost::filesystem::is_directory(corresPointSavePath)) {
            boost::filesystem::remove(corresPointSavePath);
            boost::filesystem::create_directory(corresPointSavePath);
        }
    }
    else {
        boost::filesystem::create_directory(corresPointSavePath);
    }
}
