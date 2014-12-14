
#include "define.h"
#include "DataProvider.h"
#include "CorresBuilder.h"
#include "ONIDumper.h"

void parse_params(BasicParameter& params, int argc, char *argv[])
{

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
    ONIDumper dumper("lounge.oni");
    dumper.dumpTo("./data");

    return 0;
}