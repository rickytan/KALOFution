
#include "define.h"
#include "DataProvider.h"
#include "CorresBuilder.h"
#include "ONIDumper.h"
#include "OptimizerParameter.h"
#include "Optimizer.h"

#include <pcl/console/parse.h>

void parse_params(BasicParameter& params, int argc, char *argv[])
{
    using namespace pcl::console;

    parse_argument(argc, argv, "--max_icp_iter", params.maxICPIteration);
}

int main(int argc, char *argv[])
{


    std::string sub_prog;
    if (pcl::console::parse(argc, argv, "-run", sub_prog) == -1) {
        PCL_INFO("Usage:\n\n\t%s -run <sub program>\n", argv[0]);
        return 0;
    }

    if (sub_prog == "buildcorres") {
        BasicParameter params;
        params.parse(argc, argv);

        DefaultDataProvider provider(10, "./data");
        CorresBuilder builder(params);

        builder(provider);
    }
    else if (sub_prog == "optimise") {
        OptimizerParameter oparam;
        oparam.parse(argc, argv);

        Optimizer optimizer(oparam);
        optimizer();
    }

    if (argc < 3) {
        printf("Usage:\n\n\t%s <file.oni> <dump dir>\n", argv[0]);
        return 0;
    }
    ONIDumper dumper(argv[1]);
    dumper.dumpTo(argv[2]);


    return 0;
}