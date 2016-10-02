
#pragma once

/// STD
#include <fstream>

/// ROS
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>

/// BOOST
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#define foreach BOOST_FOREACH

/// USER
#include "DataType.h"
#include "SharedStorage.h"
#include "ProcessSlam.h"
#include "Bitmap.h"
#include "lidar_msgs.h"


namespace SCV
{


class Process
{

public:
    Process(SharedStorage *newStorage) { storage = newStorage; }
    ~Process() {}

private:
    SharedStorage *storage;
    Vector3f prevPose2d;

public:
    void start()
    {
        /// copy const parameter
        Parameter param;
        storage->getParameter(param);

        /// SLAM
        ProcessSlam procSlam(param);
        Ranges rangesFront, rangesRight;
        Intens intensRight;
        VehicleInfo vehicleInfo;
        UbloxIMU ubloxIMU;
        Vector3f pose2d;
        Pose pose3d;
        Matrix3f cov;
        Vector2f curbLeft, curbRight;

        /// load map
        loadMap(procSlam);

        /// updatedMap
        ChangedPixels updatedPixels;

        /// publisher
        ros::Publisher * pub = storage->getPub();
        clothoid_msgs::lidar_msgs msgLidar;

        /// file
        //ofstream fout_ins("/home/scv/0805_02_VehicleInfo.txt");

        while (!storage->exit())
        {
            /// get Ranges
            storage->getRangesFront(rangesFront);
            storage->getRangesRight(rangesRight);
            storage->getIntensRight(intensRight);

            /// debug
            if (storage->isFleshChangedPixels())
            {
                updatedPixels.clear();
                procSlam.clearUpdatedPixels();
            }

            /// get vehicle info, ubloxIMU
            storage->getVehicleInfo(vehicleInfo);
            storage->getUbloxIMU(ubloxIMU);

            /// SLAM
            if (!param.match_without_mapping)
            {
                if (procSlam.update(rangesFront, rangesRight, intensRight, vehicleInfo, ubloxIMU, updatedPixels))
                {
                    storage->setChangedPixels(updatedPixels);
                }
            }
            else
            {
                procSlam.update(rangesFront, rangesRight, intensRight, vehicleInfo, ubloxIMU, updatedPixels);
                storage->setChangedPixels(updatedPixels);
            }

            pose2d = procSlam.getPoseForMap();
            pose3d.x = pose2d(0);
            pose3d.y = pose2d(1);
            pose3d.z = 0;
            pose3d.ax = 0;
            pose3d.ay = 0;
            pose3d.az = pose2d(2);
            storage->setVehiclePose(pose3d);

                /// ins acc, lidar acc
        //            pose3d.az = refineFullAngleDegree(pose3d.az * 180 / M_PI);
        //            if (pose3d.az > 300) pose3d.az = pose3d.az - 360;
//                fout_ins << (vehicleInfo.velRR + vehicleInfo.velFL + vehicleInfo.velFR + vehicleInfo.velRL) / 4.0 * (1000.0 / 3600.0) << "\t"
//                         << vehicleInfo.yawrate << "\t" << vehicleInfo.accX << "\t" << vehicleInfo.accY << "\t"
//                         << ubloxIMU.accX << "\t" << ubloxIMU.accY << "\t" << ubloxIMU.accZ << "\t" << ubloxIMU.gyroX << "\t" << ubloxIMU.gyroY << "\t" << ubloxIMU.gyroZ << endl;

//                prevPose = pose3d;
//            }

            /// publish
            pose2d = procSlam.getPose();
            cov = procSlam.getCovariance();
            procSlam.getCurb(curbLeft, curbRight);

            msgLidar.header.stamp = ros::Time::now();
            msgLidar.pos_x = pose2d(0);
            msgLidar.pos_y = pose2d(1);
            msgLidar.pos_head = pose2d(2);
            for (int j = 0; j < 3; j++) for (int i = 0; i < 3; i++) msgLidar.pos_cov[j * 3 + i] = cov(i ,j);
            msgLidar.road_boundary_1_x = curbLeft(0);
            msgLidar.road_boundary_1_y = curbLeft(1);
            msgLidar.road_boundary_2_x = curbRight(0);
            msgLidar.road_boundary_2_y = curbRight(1);
            pub->publish(msgLidar);

            boost::this_thread::sleep(boost::posix_time::milliseconds(param.match_period * 1000));
        }

        //fout_ins.close();

        /// save map
        saveMap(procSlam);

    }

    void startFromBag()
    {
        Parameter param;
        storage->getParameter(param);

        /// SLAM
        ProcessSlam procSlam(param);
        Ranges rangesFront, rangesRight;
        Intens intensRight;
        VehicleInfo vehicleInfo;
        UbloxIMU ubloxIMU;
        Vector3f pose2d;
        Pose pose3d;

        /// load map
        loadMap(procSlam);

        /// updatedMap
        ChangedPixels updatedPixels;

        /// read bagfile
        rosbag::Bag bag;
        bag.open(param.bag_path, rosbag::bagmode::Read);

        vector <string> topics;
        topics.push_back(param.topic_LiDAR_front);
        if (param.match_without_mapping) topics.push_back(param.topic_LiDAR_right);

        rosbag::View view(bag, rosbag::TopicQuery(topics));
        double begin = view.getBeginTime().toSec();
        double curTime;

        foreach(rosbag::MessageInstance const m, view)
        {
            curTime = m.getTime().toSec() - begin;

            if (curTime < param.bag_start_time) continue;

            if (storage->exit()) break;

            sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan> ();

            string curTopic = m.getTopic();

            if (curTopic == param.topic_LiDAR_front) rangesFront = scan->ranges;
            if (curTopic == param.topic_LiDAR_right) { rangesRight = scan->ranges; intensRight = scan->intensities; }

            /// debug
            if (storage->isFleshChangedPixels())
            {
                updatedPixels.clear();
                procSlam.clearUpdatedPixels();
            }

            /// SLAM
            if (!param.match_without_mapping)
            {
                if (procSlam.update(rangesFront, rangesRight, intensRight, vehicleInfo, ubloxIMU, updatedPixels))
                {
                    storage->setChangedPixels(updatedPixels);
                }
            }
            else
            {
                procSlam.update(rangesFront, rangesRight, intensRight, vehicleInfo, ubloxIMU, updatedPixels);
                storage->setChangedPixels(updatedPixels);
            }

            pose2d = procSlam.getPoseForMap();
            pose3d.x = pose2d(0);
            pose3d.y = pose2d(1);
            pose3d.z = 0;
            pose3d.ax = 0;
            pose3d.ay = 0;
            pose3d.az = pose2d(2);
            storage->setVehiclePose(pose3d);

            if (param.bag_period > 0.001) boost::this_thread::sleep(boost::posix_time::milliseconds(param.bag_period * 1000));
        }

        bag.close();

        while (!storage->exit());

        /// save map
        saveMap(procSlam);
    }

private:
    void loadMap(ProcessSlam &procSlam)
    {
        Parameter param;
        storage->getParameter(param);

        if (!param.map_load_map) return;

        vector <unsigned char> newMap, newMapSub[3];

        int i, sizeX = param.map_size_x, sizeY = param.map_size_y, size = sizeX * sizeY;
        newMap.resize(size);
        for (i = 0; i < 3; i++)
        {
            size = size / 4;
            newMapSub[i].resize(size);
        }

        Bitmap bitmap, bitmapSub[3];
        bitmap.initForSLAM(newMap, sizeX, sizeY);
        bitmap.load(param.map_filepath_read, newMap);
        for (i = 0; i < 3; i++)
        {
            sizeX = sizeX / 2;
            sizeY = sizeY / 2;
            bitmapSub[i].initForSLAM(newMapSub[i], sizeX, sizeY);
            string sstr = boost::str(boost::format("_sub%d.bmp") % i);
            boost::filesystem::path path(param.map_filepath_read);
            bitmapSub[i].load(path.parent_path().string() + '/' + path.stem().string() + sstr, newMapSub[i]);
        }

        procSlam.loadMap(newMap, newMapSub);
        Map *map = storage->getMapPtr();
        map->load(newMap);
    }

    void saveMap(ProcessSlam &procSlam)
    {
        Parameter param;
        storage->getParameter(param);

        if (!param.map_save_map) return;

        vector <unsigned char> newMap, newMapSub[3];
        int i, sizeX = param.map_size_x, sizeY = param.map_size_y, size = sizeX * sizeY;
        newMap.resize(size);
        for (i = 0; i < 3; i++)
        {
            size = size / 4;
            newMapSub[i].resize(size);
        }

        procSlam.saveMap(newMap, newMapSub);

        Bitmap bitmap, bitmapSub[3];
        bitmap.initForSLAM(newMap, sizeX, sizeY);
        bitmap.save(param.map_filepath_write);
        for (i = 0; i < 3; i++)
        {
            sizeX = sizeX / 2;
            sizeY = sizeY / 2;
            bitmapSub[i].initForSLAM(newMapSub[i], sizeX, sizeY);
            string sstr = boost::str(boost::format("_sub%d.bmp") % i);
            boost::filesystem::path path(param.map_filepath_write);
            bitmapSub[i].save(path.parent_path().string() + '/' + path.stem().string() + sstr);
        }
    }

};







}
