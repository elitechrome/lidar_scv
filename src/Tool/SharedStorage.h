
#pragma once

/// ROS
#include <ros/node_handle.h>

/// STD
#include <pthread.h>
#include <vector>

/// USER
#include "DataType.h"

namespace SCV
{

class SharedStorage
{

public:
    SharedStorage() {}
    ~SharedStorage()
    {
        pthread_mutex_destroy(&mtxRangesFront);
        pthread_mutex_destroy(&mtxRangesRear);
        pthread_mutex_destroy(&mtxRangesLeft);
        pthread_mutex_destroy(&mtxRangesRight);
        pthread_mutex_destroy(&mtxIntensFront);
        pthread_mutex_destroy(&mtxIntensRear);
        pthread_mutex_destroy(&mtxIntensLeft);
        pthread_mutex_destroy(&mtxIntensRight);
        pthread_mutex_destroy(&mtxIMU);
        pthread_mutex_destroy(&mtxINS);
        pthread_mutex_destroy(&mtxChangedPixels);
    }

    /* Main */
private:
    ros::Publisher * pub_lidar_msgs;
    Parameter param;
    bool flagExit;

    /* Sensor */
private:
    /// scan ranges
    Ranges rangesFront, rangesRear, rangesLeft, rangesRight;
    Intens intensFront, intensRear, intensLeft, intensRight;
    pthread_mutex_t mtxRangesFront = PTHREAD_MUTEX_INITIALIZER, mtxRangesRear = PTHREAD_MUTEX_INITIALIZER, mtxRangesLeft = PTHREAD_MUTEX_INITIALIZER, mtxRangesRight = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_t mtxIntensFront = PTHREAD_MUTEX_INITIALIZER, mtxIntensRear = PTHREAD_MUTEX_INITIALIZER, mtxIntensLeft = PTHREAD_MUTEX_INITIALIZER, mtxIntensRight = PTHREAD_MUTEX_INITIALIZER;
    /// imu (ublox)
    UbloxIMU imu;
    pthread_mutex_t mtxIMU = PTHREAD_MUTEX_INITIALIZER;
    /// ins (vehicle)
    VehicleInfo vehicleInfo;
    pthread_mutex_t mtxINS = PTHREAD_MUTEX_INITIALIZER;


    /* Mid */
private:
    /// changed pixels
    ChangedPixels changedPixels;
    bool flagFleshChangedPixels;
    pthread_mutex_t mtxChangedPixels = PTHREAD_MUTEX_INITIALIZER;
    /// vehicle pose
    Pose vehiclePose;

    /* result */
private:
    /// map
    Map map;

public:
    void init(int argc, char **argv, const std::string name, ros::NodeHandle *nh)
    {
        /// start
        flagExit = false;
        flagFleshChangedPixels = false;

        param.rosArgc                       = argc;
        param.rosArgv                       = argv;
        param.rosName                       = name;
        nh->param <std::string> ("topic_LiDAR_rear", param.topic_LiDAR_rear, "/lidar1/scan");
        nh->param <std::string> ("topic_LiDAR_front", param.topic_LiDAR_front, "/lidar2/scan");
        nh->param <std::string> ("topic_LiDAR_left", param.topic_LiDAR_left, "/lidar3/scan");
        nh->param <std::string> ("topic_LiDAR_right", param.topic_LiDAR_right, "/lidar4/scan");
        nh->param <std::string> ("topic_ublox", param.topic_ublox, "/ublox_msgs");
        nh->param <std::string> ("topic_vehicle", param.topic_vehicle, "/vehicle_status");
        nh->param <std::string> ("topic_nova", param.topic_nova, "/nova");

        nh->param <bool> ("bag_use", param.bag_use, true);
        nh->param <std::string> ("bag_path", param.bag_path, "/home/scv/clothoid_ws/dataset/TestCampus.bag");
        nh->param <float> ("bag_start_time", param.bag_start_time, 0);
        nh->param <float> ("bag_period", param.bag_period, 0.0);

        nh->param <int> ("debug_mode_road", param.debug_mode_road, 3);

        nh->param <float> ("vehicle_width", param.vehicle_width, 3.0);
        nh->param <float> ("vehicle_length", param.vehicle_length, 4.0);
        nh->param <float> ("vehicle_pos_x", param.vehicle_pos_x, 0.0);
        nh->param <float> ("vehicle_pos_y", param.vehicle_pos_y, 0.0);
        nh->param <float> ("vehicle_pos_head", param.vehicle_pos_head, 0.0);

        nh->param <float> ("scan_range_min", param.scan_range_min, 1.0);
        nh->param <float> ("scan_range_max", param.scan_range_max, 60.0);
        nh->param <bool> ("scan_dir_invert", param.scan_dir_invert, true);
        nh->param <float> ("scan_angle_min", param.scan_angle_min, -1.658063);
        nh->param <float> ("scan_angle_max", param.scan_angle_max, 1.658063);
        nh->param <float> ("scan_angle_increment_LiDAR_rear", param.scan_angle_increment_rear, 0.011636);
        nh->param <float> ("scan_angle_increment_LiDAR_front", param.scan_angle_increment_front, 0.011636); //0.002909
        nh->param <float> ("scan_angle_increment_LiDAR_left", param.scan_angle_increment_left, 0.011636);
        nh->param <float> ("scan_angle_increment_LiDAR_right", param.scan_angle_increment_right, 0.002909);

        nh->param <bool> ("map_load_map", param.map_load_map, true);
        nh->param <bool> ("map_save_map", param.map_save_map, false);
        nh->param <std::string> ("map_filepath_read", param.map_filepath_read, "/home/scv/clothoid_ws/src/lidar/map/TestCampus/map_lastcreated.bmp");
        nh->param <std::string> ("map_filepath_write", param.map_filepath_write, "/home/scv/clothoid_ws/src/lidar/map/Test2/map_lastcreated.bmp");
        nh->param <int> ("map_size_x", param.map_size_x, 2048);
        nh->param <int> ("map_size_y", param.map_size_y, 2048);
        nh->param <float> ("map_resolution", param.map_resolution, 0.1);
        nh->param <float> ("map_mapping_dist_thres", param.map_mapping_dist_thres, 0.3);
        nh->param <float> ("map_mapping_rot_thres", param.map_mapping_rot_thres, 0.06);
        nh->param <float> ("map_occ_prob", param.map_occ_prob, 0.90);
        nh->param <float> ("map_free_prob", param.map_free_prob, 0.40);

        nh->param <bool> ("match_without_mapping", param.match_without_mapping, true);
        nh->param <float> ("match_period", param.match_period, 0.01);
        nh->param <int> ("match_maxiter", param.match_maxiter, 10);

        nh->param <float> ("roadext_bp_dis_thres", param.roadext_bp_dis_thres, 1.0);
        nh->param <float> ("roadext_dis_thres", param.roadext_dis_thres, 0.12);
        nh->param <float> ("roadext_dis_break_thres", param.roadext_dis_break_thres, 0.02);
        nh->param <float> ("roadext_angle_thres", param.roadext_angle_thres, 8 * M_PI / 180);
        nh->param <int> ("roadext_diff_index_thres", param.roadext_diff_index_thres, 3);
        nh->param <float> ("roadext_length_thres", param.roadext_length_thres, 2.0);
        nh->param <float> ("roadext_safety_width", param.roadext_safety_width, 5.0);
        nh->param <float> ("roadext_center_width", param.roadext_center_width, 3.0);


        nh->param <float> ("tf_center_x", param.tf_center_x, 0);
        nh->param <float> ("tf_center_y", param.tf_center_y, 0);
        nh->param <float> ("tf_center_z", param.tf_center_z, 1.48);
        nh->param <float> ("tf_center_ax", param.tf_center_ax, 0);
        nh->param <float> ("tf_center_ay", param.tf_center_ay, 0);
        nh->param <float> ("tf_center_az", param.tf_center_az, 0);
        nh->param <float> ("tf_lidar1_x", param.tf_lidar1_x, -1.595);
        nh->param <float> ("tf_lidar1_y", param.tf_lidar1_y, 0);
        nh->param <float> ("tf_lidar1_z", param.tf_lidar1_z, 0.050);
        nh->param <float> ("tf_lidar1_ax", param.tf_lidar1_ax, 0);
        nh->param <float> ("tf_lidar1_ay", param.tf_lidar1_ay, 0.28798);
        nh->param <float> ("tf_lidar1_az", param.tf_lidar1_az, 3.14159265359);
        nh->param <float> ("tf_lidar2_x", param.tf_lidar2_x, 1.910);
        nh->param <float> ("tf_lidar2_y", param.tf_lidar2_y, 0);
        nh->param <float> ("tf_lidar2_z", param.tf_lidar2_z, -0.880);
        nh->param <float> ("tf_lidar2_ax", param.tf_lidar2_ax, 0);
        nh->param <float> ("tf_lidar2_ay", param.tf_lidar2_ay, 0);
        nh->param <float> ("tf_lidar2_az", param.tf_lidar2_az, 0);
        nh->param <float> ("tf_lidar3_x", param.tf_lidar3_x, 0.116);
        nh->param <float> ("tf_lidar3_y", param.tf_lidar3_y, 0.420);
        nh->param <float> ("tf_lidar3_z", param.tf_lidar3_z, 0.048);
        nh->param <float> ("tf_lidar3_ax", param.tf_lidar3_ax, 0);
        nh->param <float> ("tf_lidar3_ay", param.tf_lidar3_ay, 0.10472);
        nh->param <float> ("tf_lidar3_az", param.tf_lidar3_az, 0);
        nh->param <float> ("tf_lidar4_x", param.tf_lidar4_x, 0.120);
        nh->param <float> ("tf_lidar4_y", param.tf_lidar4_y, -0.420);
        nh->param <float> ("tf_lidar4_z", param.tf_lidar4_z, 0.050);
        nh->param <float> ("tf_lidar4_ax", param.tf_lidar4_ax, 0);
        nh->param <float> ("tf_lidar4_ay", param.tf_lidar4_ay, 0.28798);
        nh->param <float> ("tf_lidar4_az", param.tf_lidar4_az, 0);

        /// allocate map
        map.allocate(param.map_size_x, param.map_size_y, param.map_resolution);

    }

    inline bool exit() { return flagExit; }
    inline void exit(bool flag) { flagExit = flag; }
    inline void setPub(ros::Publisher * newPub) { pub_lidar_msgs = newPub; }
    inline ros::Publisher * getPub() { return pub_lidar_msgs; }

    void setRangesRear(const Ranges &newRanges)
    {
        pthread_mutex_lock(&mtxRangesRear);
        rangesRear = newRanges;
        pthread_mutex_unlock(&mtxRangesRear);
    }
    void getRangesRear(Ranges &newRanges)
    {
        pthread_mutex_lock(&mtxRangesRear);
        newRanges = rangesRear;
        pthread_mutex_unlock(&mtxRangesRear);
    }

    void setRangesFront(const Ranges &newRanges)
    {
        pthread_mutex_lock(&mtxRangesFront);
        rangesFront = newRanges;
        pthread_mutex_unlock(&mtxRangesFront);
    }
    void getRangesFront(Ranges &newRanges)
    {
        pthread_mutex_lock(&mtxRangesFront);
        newRanges = rangesFront;
        rangesFront.clear();
        pthread_mutex_unlock(&mtxRangesFront);
    }

    void setRangesLeft(const Ranges &newRanges)
    {
        pthread_mutex_lock(&mtxRangesLeft);
        rangesLeft = newRanges;
        pthread_mutex_unlock(&mtxRangesLeft);
    }
    void getRangesLeft(Ranges &newRanges)
    {
        pthread_mutex_lock(&mtxRangesLeft);
        newRanges = rangesLeft;
        pthread_mutex_unlock(&mtxRangesLeft);
    }

    void setRangesRight(const Ranges &newRanges)
    {
        pthread_mutex_lock(&mtxRangesRight);
        rangesRight = newRanges;
        pthread_mutex_unlock(&mtxRangesRight);
    }
    void getRangesRight(Ranges &newRanges)
    {
        pthread_mutex_lock(&mtxRangesRight);
        newRanges = rangesRight;
        rangesRight.clear();
        pthread_mutex_unlock(&mtxRangesRight);
    }
    void setIntensRight(const Intens &newIntens)
    {
        pthread_mutex_lock(&mtxIntensRight);
        intensRight = newIntens;
        pthread_mutex_unlock(&mtxIntensRight);
    }
    void getIntensRight(Intens &newIntens)
    {
        pthread_mutex_lock(&mtxIntensRight);
        newIntens = intensRight;
        intensRight.clear();
        pthread_mutex_unlock(&mtxIntensRight);
    }


    void setUbloxIMU(const UbloxIMU &newIMU)
    {
        pthread_mutex_lock(&mtxIMU);
        imu = newIMU;
        pthread_mutex_unlock(&mtxIMU);
    }
    void getUbloxIMU(UbloxIMU &newIMU)
    {
        pthread_mutex_lock(&mtxIMU);
        newIMU = imu;
        pthread_mutex_unlock(&mtxIMU);
    }

    void setVehicleInfo(const VehicleInfo &newInfo)
    {
        pthread_mutex_lock(&mtxIMU);
        vehicleInfo = newInfo;
        pthread_mutex_unlock(&mtxIMU);
    }
    void getVehicleInfo(VehicleInfo &newInfo)
    {
        pthread_mutex_lock(&mtxINS);
        newInfo = vehicleInfo;
        pthread_mutex_unlock(&mtxINS);
    }

    void getParameter(Parameter &newParam) { newParam = param; }

    void setChangedPixels(const ChangedPixels &newChangedPixels)
    {
        pthread_mutex_lock(&mtxChangedPixels);
        flagFleshChangedPixels = false;
        changedPixels.copy(newChangedPixels);
        pthread_mutex_unlock(&mtxChangedPixels);
    }
    void fleshChangedPixels() { flagFleshChangedPixels = true; }
    bool isFleshChangedPixels() { return flagFleshChangedPixels; }

    Map * getMapPtr() { return &map; }

    bool updateMap()
    {
        if (changedPixels.empty()) return false;

        int i, size;
        unsigned char *m = map.getPixels();
        Pixels *px;

        pthread_mutex_lock(&mtxChangedPixels);

        /// occ
        px = &changedPixels.changedOccs;
        size = px->size();
        int j;
        for (i = 0; i < size; i++)
        {
            j = 3 * (*px)[i];
            m[j] = 0;
            m[j + 1] = 0;
            m[j + 2] = 0;
        }

        if (param.debug_mode_road == 0)
        {
            /// free
            px = &changedPixels.changedFrees;
            size = px->size();
            for (i = 0; i < size; i++)
            {
                j = 3 * (*px)[i];
                m[j] = 255;
                m[j + 1] = 255;
                m[j + 2] = 255;
            }
        }
        else if (param.debug_mode_road > 0)
        {
            /// road
            px = &changedPixels.changedRoads;
            size = px->size();
            for (i = 0; i < size; i++)
            {
                j = 3 * (*px)[i];
                m[j] = 30;
                m[j + 1] = 255;
                m[j + 2] = 30;
            }
        }

        /// trace
//        px = &changedPixels.changedTraces;
//        size = px->size();
//        for (i = 0; i < size; i++)
//        {
//            j = 3 * (*px)[i];
//            m[j] = 255;
//            m[j + 1] = 0;
//            m[j + 2] = 0;
//        }

        changedPixels.clear();
        fleshChangedPixels();

        pthread_mutex_unlock(&mtxChangedPixels);

        return true;
    }

    void setVehiclePose(const Pose &newPose)
    {
        vehiclePose = newPose;
    }
    void getVehiclePose(Pose &newPose)
    {
        newPose = vehiclePose;
    }

};






}
