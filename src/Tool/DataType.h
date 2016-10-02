
#pragma once

/// Eigen
#include <Eigen/Dense>

/// STD
#include <vector>
#include <cstring>

const float M_2PI = M_PI * 2;

namespace SCV
{

//struct timeval timeStart;
//struct timeval timeEnd;
//double timeSample;
//gettimeofday(&timeStart, NULL);
//gettimeofday(&timeEnd, NULL);
//timeSample = (double)(timeEnd.tv_sec)+(double)(timeEnd.tv_usec)/1000000.0-(double)(timeStart.tv_sec)-(double)(timeStart.tv_usec)/1000000.0;
//std::cout << timeSample << "\n";

typedef std::vector <float> Ranges;

typedef std::vector <float> Intens;

typedef std::vector <Eigen::Vector2f> Points;

struct UbloxIMU { long long gyroX, gyroY, gyroZ, accX, accY, accZ; };

struct VehicleInfo { double accX, accY, yawrate; double velFL, velFR, velRL, velRR; };

struct Pose { double x, y, z, ax, ay, az; };

struct Parameter
{
    /// ros topic
    std::string topic_LiDAR_rear, topic_LiDAR_front, topic_LiDAR_left, topic_LiDAR_right, topic_ublox, topic_vehicle, topic_nova;

    /// ros init
    int rosArgc;
    char **rosArgv;
    std::string rosName;

    /// bag
    bool bag_use;
    std::string bag_path;
    float bag_start_time;
    float bag_period;

    /// debug
    int debug_mode_road;

    /// Vehicle
    float vehicle_width;
    float vehicle_length;
    float vehicle_pos_x;
    float vehicle_pos_y;
    float vehicle_pos_head;

    /// LiDAR scan
    float scan_range_min;
    float scan_range_max;
    float scan_angle_min;
    float scan_angle_max;
    float scan_angle_increment_rear;
    float scan_angle_increment_front;
    float scan_angle_increment_left;
    float scan_angle_increment_right;
    bool scan_dir_invert;

    /// SLAM map
    int map_size_x;
    int map_size_y;
    float map_resolution;
    float map_occ_prob;
    float map_free_prob;
    float map_mapping_dist_thres;
    float map_mapping_rot_thres;
    std::string map_filepath_write, map_filepath_read;
    bool map_load_map;
    bool map_save_map;

    /// SLAM matching
    bool match_without_mapping;
    float match_period;
    int match_maxiter;

    // TF
    /// Vehicle Center to LIDAR Center
    float tf_center_x;
    float tf_center_y;
    float tf_center_z;
    float tf_center_ax;
    float tf_center_ay;
    float tf_center_az;

    /// LIDAR Center to LIDAR 1
    float tf_lidar1_x;
    float tf_lidar1_y;
    float tf_lidar1_z;
    float tf_lidar1_ax;
    float tf_lidar1_ay;
    float tf_lidar1_az;

    /// LIDAR Center to LIDAR 2
    float tf_lidar2_x;
    float tf_lidar2_y;
    float tf_lidar2_z;
    float tf_lidar2_ax;
    float tf_lidar2_ay;
    float tf_lidar2_az;

    /// LIDAR Center to LIDAR 3
    float tf_lidar3_x;
    float tf_lidar3_y;
    float tf_lidar3_z;
    float tf_lidar3_ax;
    float tf_lidar3_ay;
    float tf_lidar3_az;

    /// LIDAR Center to LIDAR 4
    float tf_lidar4_x;
    float tf_lidar4_y;
    float tf_lidar4_z;
    float tf_lidar4_ax;
    float tf_lidar4_ay;
    float tf_lidar4_az;

    // road extraction
    /// Line Extraction
    float line_sensor_range_offset;
    float line_break_angle_thres;
    float line_linerange_thres;
    float line_bp_length_thres;
    float line_scan_cut_angle;

    /// road pos optimization
    float road_dls_cut_y;
    float road_dls_damp_min;
    float road_dls_damp_init;
    float road_dls_damp_diff;
    float road_angle_y_offset;

    /// road ext
    float roadext_bp_dis_thres;
    float roadext_dis_thres;
    float roadext_dis_break_thres;
    float roadext_angle_thres;
    int roadext_diff_index_thres;
    float roadext_length_thres;
    float roadext_safety_width;
    float roadext_center_width;

};

typedef int Pixel;

typedef std::vector <Pixel> Pixels;

class ChangedPixels
{

public:
    ChangedPixels() {}
    ~ChangedPixels() {}

public:
    Pixels changedOccs, changedFrees, changedRoads, changedTraces;

public:
    bool empty() { return changedOccs.empty() && changedFrees.empty() && changedRoads.empty() && changedTraces.empty(); }
    void clear() { changedOccs.clear(); changedFrees.clear(); changedRoads.clear(); changedTraces.clear(); }

    void copy(const ChangedPixels &src)
    {
        changedOccs = src.changedOccs;
        changedFrees = src.changedFrees;
        changedRoads = src.changedRoads;
        changedTraces = src.changedTraces;
    }

    void pushbackOccs(Pixels &newOccs)
    {
        changedOccs.reserve(changedOccs.size() + newOccs.size());
        changedOccs.insert(changedOccs.end(), newOccs.begin(), newOccs.end());
    }

    void pushbackFrees(Pixels &newFrees)
    {
        changedFrees.reserve(changedFrees.size() + newFrees.size());
        changedFrees.insert(changedFrees.end(), newFrees.begin(), newFrees.end());
    }

    void pushbackRoads(Pixels &newRoads)
    {
        changedRoads.reserve(changedRoads.size() + newRoads.size());
        changedRoads.insert(changedRoads.end(), newRoads.begin(), newRoads.end());
    }

    void pushbackTraces(Pixels &newTraces)
    {
        changedTraces.reserve(changedTraces.size() + newTraces.size());
        changedTraces.insert(changedTraces.end(), newTraces.begin(), newTraces.end());
    }

    void pushbackTraces(Pixel newTrace)
    {
        changedTraces.push_back(newTrace);
    }

};

class Map
{

public:
    Map() { pixels = 0; }
    ~Map() { destroy(); }

private:
    unsigned char * pixels;
    int maxX, maxY;
    float gridSize;

public:
    unsigned char * getPixels() { return pixels; }
    int getMaxX() { return maxX; }
    int getMaxY() { return maxY; }
    int getGridSize() { return gridSize; }

    void allocate(int newMaxX, int newMaxY, float newGridSize)
    {
        if (pixels != 0) { std::cout << "map : already allocated"; return; }

        if (newMaxX <= 0 || newMaxY <= 0 || newGridSize <= 0) { std::cout << "invalid map size"; return; }

        maxX = newMaxX;
        maxY = newMaxY;
        gridSize = newGridSize;
        pixels = new unsigned char [3 * maxX * maxY];
        std::memset(pixels, 255 * 0.2, 3 * maxX * maxY * sizeof(unsigned char));
    }

    void destroy()
    {
        if (pixels == 0) { std::cout << "map :already destroyed"; return; }

        delete pixels;
    }

    void load(std::vector <unsigned char> &newMap)
    {
        int i, j, size = newMap.size();

        for (i = 0; i < size; i++)
        {
            if (newMap[i] == 0)
            {
                j = 3 * i;
                pixels[j] = 0;
                pixels[j + 1] = 0;
                pixels[j + 2] = 0;
            }
            else if (newMap[i] == 255)
            {
                j = 3 * i;
                pixels[j] = 255;
                pixels[j + 1] = 255;
                pixels[j + 2] = 255;
            }
        }
    }

};

inline double refineHalfAngleDegree(double angle)
{
    double refinedAngle;

    refinedAngle = fmod(fmod(angle, 360) + 360, 360);
    if (refinedAngle > 180) refinedAngle = refinedAngle - 360;

    return refinedAngle;
}

inline double refineFullAngleDegree(double angle)
{
    double refinedAngle;

    refinedAngle = fmod(fmod(angle, 360) + 360, 360);

    return refinedAngle;
}




}
