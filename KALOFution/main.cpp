
#include "define.h"
#include "DataProvider.h"
#include "CorresBuilder.h"

int main(int argc, char *agrv[])
{
    DefaultDataProvider provider(10, "./data");
    CorresBuilder builder;

    builder(provider);

    return 0;
}