#include "Parameter.h"
#include "OptimizerParameter.h"

#include <pcl/console/parse.h>
#include <pcl/console/print.h>

using namespace pcl::console;

void OptimizerParameter::parse(int argc, char *argv[])
{
    if (find_switch(argc, argv, "-h") || find_switch(argc, argv, "--help")) {
        help();
    }
    parse_argument(argc, argv, "--max-iteration", maxIteration);
    parse_argument(argc, argv, "--save-to", saveDirectory);
    useCholmod = find_switch(argc, argv, "--use-cholmod");
    g2oOptimize = find_switch(argc, argv, "--use-g2o");
}

void OptimizerParameter::help()
{
    printf("\
Options:\n\
    --max-iteration <num> :     default 8\n\
    --save-to <string> :        save optimized clouds to the <string> directory\n\
\
");
    exit(0);
}
