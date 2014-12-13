#include "DataProvider.h"
#include "CorresBuilder.h"

typedef pcl::PointXYZRGBNormal PointType;

int main(int argc, char *agrv)
{
    DefaultDataProvider<PointType> provider(10, "./data");
    CorresBuilder<PointType> builder;

    builder(provider);

    return 0;
}