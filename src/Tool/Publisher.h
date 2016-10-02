
#pragma once

#include <ros/publisher.h>
#include "lidar_msgs.h"

namespace SCV
{

class Publisher
{

public:
    Publisher()
    {

    }
    ~Publisher() {}

private:
    ros::Publisher pub;


};

}
