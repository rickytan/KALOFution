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
    parse_argument(argc, argv, "--align-sampling-limit", randomSamplingLimitUsedToAlignment);
    parse_argument(argc, argv, "--min-corres-num", acceptableCorresPointNum);
    parse_argument(argc, argv, "--min-corres-ratio", acceptableCorresPointRatio);
    parse_argument(argc, argv, "--align-translation-thres", translationThres);
    parse_argument(argc, argv, "--align-rotation-thres", rotationThres);
    parse_argument(argc, argv, "--need-align", cloudPairNeedAlignment);
    parse_argument(argc, argv, "--better-corres", useBetterCorrespondence);

    if (boost::filesystem::exists(corresPointSavePath) && !boost::filesystem::is_directory(corresPointSavePath)) {
        boost::filesystem::remove(corresPointSavePath);
    }
    boost::filesystem::create_directories(corresPointSavePath);
}

void BasicParameter::help()
{
    printf("\
Options:\n\
    --dist-thres <num> :                       default 0.15\n\
    --angle-thres <string> :                   default 30 deg\n\
    --icp-itera <num> :                        default 20\n\
    --valid-pair-dist-thres <num> :            default 0.05\n\
    --save-to <string> :                       default ./corres\n\
    --align-sampling-limit <num> :             Alignment downsampling limit, default 50000\n\
    --align-translation-thres <num> :          Threshold for alignment translation, default 0.3 m\n\
    --align-rotation-thres <num> :             Threshold for alignemnt rotaion, default 15 deg\n\
    --need-align 0|1 :              align before find correspondence points. Default 1\n");
    exit(0);
}
