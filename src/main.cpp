
#include "LiDAR_SCV.h"
#include "cudafunc.h"

int main(int argc, char **argv)
{
    LiDAR_SCV lidar;

    lidar.init(argc, argv, "lidar_scv");
    lidar.end();

    return 0;
}

