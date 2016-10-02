
#pragma once

/// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


/// STD
#include <pthread.h>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <unistd.h>

/// BOOST

/// USER
#include "Tool/DataType.h"
#include "Tool/SharedStorage.h"
#include "Tool/SensorCallback.h"
#include "Tool/Process.h"
using namespace SCV;


class LiDAR_SCV
{

public:
    LiDAR_SCV() {}
    ~LiDAR_SCV() {}


    // thread
private:
    pthread_t thread_callback, thread_process, thread_debugWindow;
private:
    static void * tfunc_callback(void * t) { ((LiDAR_SCV*)t)->main_callback(); return NULL; }
    static void * tfunc_process(void * t) { ((LiDAR_SCV*)t)->main_process(); return NULL; }
    static void * tfunc_debugWindow(void * t) { ((LiDAR_SCV*)t)->main_debugWindow(); return NULL; }

    // data
public:
    SharedStorage mainStorage;


public:

    void init(int argc, char **argv, const std::string name)
    {
        ros::init(argc, argv, name);
        ros::NodeHandle nh("~");
        ros::Publisher pub = nh.advertise <clothoid_msgs::lidar_msgs> ("lidar_msgs", 10);
	
        /// param init
        mainStorage.init(argc, argv, name, &nh);
        mainStorage.setPub(&pub);

        /// thread init
        pthread_create(&thread_callback, NULL, tfunc_callback, this);
        pthread_create(&thread_process, NULL, tfunc_process, this);
        pthread_create(&thread_debugWindow, NULL, tfunc_debugWindow, this);

        /// thread main
        main_command();
    }

    static LiDAR_SCV * tempObject;
    static void my_handler(int s)
    {
        LiDAR_SCV::tempObject->mainStorage.exit(true);
        LiDAR_SCV::tempObject->end();

        exit(1);
    }

    void main_command()
    {
        struct sigaction sigIntHandler;

        sigIntHandler.sa_handler = &my_handler;
        sigemptyset(&sigIntHandler.sa_mask);
        sigIntHandler.sa_flags = 0;

        sigaction(SIGINT, &sigIntHandler, NULL);
        LiDAR_SCV::tempObject = this;

        //while(!mainStorage.exit())
        while(ros::ok())
        {
            sleep(10000);
//            std::cin >> strCmd;
//            if (strCmd == "exit")
//            {
//                std::cout << "save map (y / n) \n\n";
//                std::cin >> strCmd;
//                if (strCmd == "y" || strCmd == "Y") mainStorage.saveMap();

//                mainStorage.exit(true);
//                break;
//            }
        }
    }

    void main_callback()
    {
        unsigned long cid = 2;
        pthread_setaffinity_np(pthread_self(), sizeof(cid), (cpu_set_t *)&cid);

        SensorCallback cb(&mainStorage);
        cb.start();

        std::cout << "thread_callback exit" << std::endl;
        pthread_exit((void *) 0);
    }

    void main_process()
    {
        unsigned long cid = 3;
        pthread_setaffinity_np(pthread_self(), sizeof(cid), (cpu_set_t *)&cid);

        Process p(&mainStorage);
        Parameter param;
        mainStorage.getParameter(param);
        if (!param.bag_use) p.start();
        else p.startFromBag();

        std::cout << "thread_process exit" << std::endl;
        pthread_exit((void *) 0);
    }

    void main_debugWindow()
    {
        unsigned long cid = 4;
        pthread_setaffinity_np(pthread_self(), sizeof(cid), (cpu_set_t *)&cid);

        MapViewer v(&mainStorage);
        Parameter param;
        mainStorage.getParameter(param);
        v.start(param);

        std::cout << "thread_debugWindow exit" << std::endl;
        pthread_exit((void *) 0);
    }

    void end()
    {
        int status;
        int j1 = pthread_join(thread_callback, (void **)&status);
        if (j1 == 0) std::cout << "completed join with thread_callback, status : " << status << std::endl;
        int j2 = pthread_join(thread_process, (void **)&status);
        if (j2 == 0) std::cout << "completed join with thread_process, status : " << status << std::endl;
        int j3 = pthread_join(thread_debugWindow, (void **)&status);
        if (j3 == 0) std::cout << "completed join with thread_debugWindow, status : " << status << std::endl;
    }


};

LiDAR_SCV * LiDAR_SCV::tempObject = 0;
