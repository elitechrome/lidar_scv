
#pragma once

/// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

/// USER
#include "SharedStorage.h"
#include "DebugWindow.h"
#include "clothoid_CAN.h"
#include "ublox_msgs.h"
#include "novatel_msgs.h"
#include "lidar_msgs.h"

namespace SCV
{

class SensorCallback
{

public:
    SensorCallback(SharedStorage *newStorage) { storage = newStorage; }
    ~SensorCallback() {}

    // callback functions
public:
    void func_callbackLIDAR_rear(const sensor_msgs::LaserScan::ConstPtr &scan) { storage->setRangesRear(scan->ranges); }
    void func_callbackLIDAR_front(const sensor_msgs::LaserScan::ConstPtr &scan) { storage->setRangesFront(scan->ranges); }
    void func_callbackLIDAR_left(const sensor_msgs::LaserScan::ConstPtr &scan) { storage->setRangesLeft(scan->ranges); }
    void func_callbackLIDAR_right(const sensor_msgs::LaserScan::ConstPtr &scan) { storage->setRangesRight(scan->ranges); storage->setIntensRight(scan->intensities); }//scan.get()->header.stamp = ros::Time::now() }
    void func_callbackUblox(const clothoid_msgs::ublox_msgs::ConstPtr &ublox) { UbloxIMU imu = {ublox->gyro_x, ublox->gyro_y, ublox->gyro_z, ublox->acc_x, ublox->acc_y, ublox->acc_z}; storage->setUbloxIMU(imu); }
    void func_callbackVehicle(const clothoid_msgs::clothoid_CAN::ConstPtr &vehicle) {
        VehicleInfo info = {vehicle->Gway_Longitudinal_Accel_Speed, vehicle->Gway_Lateral_Accel_Speed, vehicle->Gway_Yaw_Rate_Sensor, vehicle->Gway_Wheel_Velocity_FL, vehicle->Gway_Wheel_Velocity_FR, vehicle->Gway_Wheel_Velocity_RL, vehicle->Gway_Wheel_Velocity_RR}; storage->setVehicleInfo(info); }
    void func_callbackNova(const clothoid_msgs::novatel_msgs::ConstPtr &nova) {  }

private:
    SharedStorage *storage;

public:
    void start()
    {
        ros::NodeHandle n;
        ros::Subscriber lidarSubs[4], ubloxSub, vehicleSub, novaSub;
        Parameter param;

        storage->getParameter(param);

        lidarSubs[0] = n.subscribe (param.topic_LiDAR_rear, 4, &SensorCallback::func_callbackLIDAR_rear, this);
        lidarSubs[1] = n.subscribe (param.topic_LiDAR_front, 4, &SensorCallback::func_callbackLIDAR_front, this);
        //lidarSubs[2] = n.subscribe (param.topic_LiDAR_left, 4, &SensorCallback::func_callbackLIDAR_left, this);
        lidarSubs[3] = n.subscribe (param.topic_LiDAR_right, 4, &SensorCallback::func_callbackLIDAR_right, this);
        //ubloxSub = n.subscribe (param.topic_ublox, 4, &SensorCallback::func_callbackUblox, this);
        //vehicleSub = n.subscribe (param.topic_vehicle, 4, &SensorCallback::func_callbackVehicle, this);
        //novaSub = n.subscribe (param.topic_nova, 4, &SensorCallback::func_callbackNova, this);

        while(!storage->exit())
        {
            ros::spinOnce();
            usleep(10);
        }
    }



};




}
