
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
    /*
    BasicParameter params;
    parse_params(params, argc, argv);

    DefaultDataProvider provider(10, "./data");
    CorresBuilder builder(params);

    builder(provider);
    */
    ONIDumper dumper("ml.oni");
    dumper.dumpTo("./data");

    OptimizerParameter oparam;
    Optimizer optimizer(oparam);

    optimizer();

    return 0;
}