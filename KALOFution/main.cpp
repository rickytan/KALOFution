
#include "define.h"
#include "DataProvider.h"
#include "CorresBuilder.h"
#include "ONIDumper.h"
#include "MP4Dumper.h"
#include "MapDumper.h"
#include "OptimizerParameter.h"
#include "Optimizer.h"

#include <pcl/console/parse.h>

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

        std::string pos_file = "camera_pos.txt";
        parse_argument(argc, argv, "--pos-file", pos_file);

        DefaultDataProvider provider(48, "./data", 25);
        provider.setPoseFile(pos_file);
        provider.prepareData();
        CorresBuilder builder(params);

        builder(provider);
    }
    else if (sub_prog == "optimize") {
        OptimizerParameter oparam;
        oparam.parse(argc, argv);

        DefaultDataProvider provider(48, "./data", 25);
        std::vector<CloudTypePtr> clouds;
        for (size_t i = 0; i < provider.size(); ++i) {
            clouds.push_back(provider[i]);
        }

        string corres_dir = "./corres";

        std::vector<CloudPair> pairs;
        char filename[1024] = { 0 };
        for (size_t i = 0; i < clouds.size(); ++i) {
            for (size_t j = i + 1; j < clouds.size(); ++j) {
                sprintf(filename, "%s/corres_%d_%d.txt", corres_dir.c_str(), i, j);
                if (boost::filesystem::exists(filename)) {
                    CloudPair pair(i, j);
                    pair.loadCorresPoints(filename);
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

        std::string dump_dir = "data";
        int step = 25;
        std::string pos_file = "camera_pos.txt";
        std::string data_dir = "datadump";
        std::string data_format = "pcd";
        parse_argument(argc, argv, "--dump-dir", dump_dir);
        parse_argument(argc, argv, "--step", step);
        parse_argument(argc, argv, "--pos-file", pos_file);
        parse_argument(argc, argv, "--data-dir", data_dir);
        parse_argument(argc, argv, "--format", data_format);

        MapDumper dumper(data_dir, pos_file);
        dumper.setStep(step);
        dumper.setDumpFormat(data_format);
        dumper.dumpTo(dump_dir);
    }
    else {
        PCL_ERROR("Unknown command : %s\n", sub_prog.c_str());
    }
    getchar();
    return 0;
}