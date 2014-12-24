
#include "define.h"
#include "DataProvider.h"
#include "CorresBuilder.h"
#include "ONIDumper.h"
#include "MP4Dumper.h"
#include "MapDumper.h"
#include "OptimizerParameter.h"
#include "Optimizer.h"

#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

#include <boost/filesystem.hpp>

void help()
{
    printf("\
\nSupported command :\n\
\tbuildcorres : \n\
\toptimize :    \n\
\tdumpmap :     \n\
\n\n\
use '-run command -h' for details.");
}

using namespace pcl::console;

int main(int argc, char *argv[])
{
    std::string program(argv[0]);
    std::string sub = program.substr(program.rfind('\\') + 1);
    strcpy(argv[0], sub.c_str());

    std::string sub_prog;
    if (pcl::console::parse(argc, argv, "-run", sub_prog) == -1) {
        printf("Usage:\n\n\t%s -run <sub program>\n", argv[0]);
        help();
        return 0;
    }

    if (sub_prog == "buildcorres") {
        BasicParameter params;
        params.parse(argc, argv);

        std::string pos_file = "";
        std::string data_dir = "./data";
        int size = 48;
        int step = 25;
        parse_argument(argc, argv, "--pos-file", pos_file);
        parse_argument(argc, argv, "--cloud-num", size);
        parse_argument(argc, argv, "--step", step);
        parse_argument(argc, argv, "--data-dir", data_dir);

        DefaultDataProvider provider(size, data_dir, step);
        provider.setPoseFile(pos_file);
        provider.prepareData();
        CorresBuilder builder(params);

        builder(provider);
    }
    else if (sub_prog == "optimize") {
        OptimizerParameter oparam;
        oparam.parse(argc, argv);

        std::string pos_file = "";
        std::string data_dir = "./data";
        int size = 48;
        int step = 25;
        parse_argument(argc, argv, "--pos-file", pos_file);
        parse_argument(argc, argv, "--cloud-num", size);
        parse_argument(argc, argv, "--step", step);
        parse_argument(argc, argv, "--data-dir", data_dir);

        DefaultDataProvider provider(size, data_dir, step);
        provider.setPoseFile(pos_file);
        provider.prepareData();
        std::vector<CloudTypePtr> clouds;
        for (size_t i = 0; i < provider.size(); ++i) {
            CloudTypePtr trans(new CloudType);
            pcl::transformPointCloudWithNormals(*provider[i], *trans, provider.initTransformOfCloudAtIndex(i));
            clouds.push_back(trans);
        }

        string corres_dir = "./corres";
        parse_argument(argc, argv, "--corres-dir", corres_dir);

        std::vector<CloudPair> pairs;
        char filename[1024] = { 0 };
        for (size_t i = 0; i < clouds.size(); ++i) {
            for (size_t j = i + 1; j < clouds.size(); ++j) {
                sprintf(filename, "%s/corres_%d_%d.txt", corres_dir.c_str(), i, j);
                if (boost::filesystem::exists(filename)) {
                    CloudPair pair(i, j);
                    pair.loadCorresPoints(filename);
                    if (pair.corresPointIdx.size())
                        pairs.push_back(pair);
                }
            }
        }

        Optimizer optimizer(oparam);
        optimizer(clouds, pairs);
    }
    else if (sub_prog == "dumponi") {
        if (argc < 5) {
            printf("Usage:\n\n\t%s <file.oni> <dump dir>\n", argv[0]);
            return 0;
        }
        ONIDumper dumper(argv[3]);
        dumper.dumpTo(argv[4]);
    }
    else if (sub_prog == "dumpvideo") {
        if (argc < 5) {
            printf("Usage:\n\n\t%s <file.mp4> <dump dir>\n", argv[0]);
            return 0;
        }
        MP4Dumper dumper(argv[3]);
        dumper.dumpTo(argv[4]);
    }
    else if (sub_prog == "dumpmap") {
        using namespace pcl::console;

        std::string dump_dir = "./data";
        int step = 25;
        std::string data_dir = "./datadump";
        std::string data_format = "pcd";
        std::string pos_file = "";
        float min_clip = 0.3;
        float max_clip = 2.4;
        parse_argument(argc, argv, "--dump-dir", dump_dir);
        parse_argument(argc, argv, "--step", step);
        parse_argument(argc, argv, "--data-dir", data_dir);
        parse_argument(argc, argv, "--format", data_format);

        MapDumper dumper(data_dir);
        dumper.setStep(step);
        dumper.setDumpFormat(data_format);
        
        if (parse_argument(argc, argv, "--pos-file", pos_file) >= 0) {
            dumper.setCameraPosFile(pos_file);
        }
        if (parse_argument(argc, argv, "--min-clip", min_clip) >= 0) {
            dumper.setMinClipDepth(min_clip);
        }
        if (parse_argument(argc, argv, "--max-clip", max_clip) >= 0) {
            dumper.setMaxClipDepth(max_clip);
        }
        dumper.dumpTo(dump_dir);
    }
    else {
        PCL_ERROR("Unknown command : %s\n", sub_prog.c_str());
    }

    PCL_WARN("DONE!!! Press any key to exit.");
    getchar();
    return 0;
}