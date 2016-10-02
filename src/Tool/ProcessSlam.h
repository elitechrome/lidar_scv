
#pragma once

/// EIGEN
using namespace Eigen;

/// STD
#include <iostream>
using namespace std;

/// USER
#include "DataType.h"
#include "SharedStorage.h"
#include "SlamMap.h"
#include "Filter.h"

namespace SCV
{

class ProcessSlam
{

public:
    ProcessSlam(Parameter &newParam)
    {
        param = newParam;

        poseSensor << param.vehicle_pos_x, param.vehicle_pos_y, param.vehicle_pos_head;
        prevPoseSensor << 0, 0, 1000;
        for (int i = 2; i >= 0; i--)
        {
            float scale = pow(2, i + 1);
            slamMapSub[i].init(newParam, param.map_resolution * scale, param.map_size_x / scale, param.map_size_y / scale);
        }
        slamMapFinal.init(newParam, param.map_resolution, param.map_size_x, param.map_size_y);

        affLiDAR4.setIdentity();
        affLiDAR4.translation() << param.tf_lidar4_x, param.tf_lidar4_y, param.tf_lidar4_z + param.tf_center_z;
        affLiDAR4.rotate(AngleAxisf(param.tf_lidar4_az,  Vector3f::UnitZ()) * AngleAxisf(param.tf_lidar4_ay,  Vector3f::UnitY()) * AngleAxisf(param.tf_lidar4_ax,  Vector3f::UnitX()));
    }
    ~ProcessSlam() { }

private:
    Parameter param;
    SlamMap slamMapFinal, slamMapSub[3];
    Vector3f poseSensor, prevPoseSensor;
    Matrix3f hessian;
    Vector2f curbLeft, curbRight;
    Affine3f affLiDAR4;
    KF_CI kfposition;

public:
    bool update(Ranges &rangesFront, Ranges &rangesRight, Intens &intensRight, VehicleInfo &vehicleInfo, UbloxIMU &ubloxIMU, ChangedPixels &updatedPixels)
    {
        bool flagMapping;

        /// SLAM (front)
        if (!rangesFront.empty())
        {
            Points pointsFrontFinal, pointsFrontSub[3];

            /// scaled (x, y) points
            scanToPointsFront(rangesFront, param.scan_angle_increment_front, pointsFrontFinal, pointsFrontSub);

            /// vehicle info
//            double Vhead = ((vehicleInfo.velFL + vehicleInfo.velFR + vehicleInfo.velRL + vehicleInfo.velRR) / 4.0) * (1000.0 / 3600.0);
//            double Vt = vehicleInfo.yawrate * (M_PI / 180);
//            float dt = param.match_period;
//            double Ahead = vehicleInfo.accX + 0.11188;
//            double Aleft = vehicleInfo.accY + 0.063143;

//            Vector3f poseSLAM = getPose(poseSensor);
//            Vector3f posePred = poseSLAM;
//            posePred(2) = posePred(2) + dt * Vt;
//            posePred(0) = posePred(0) + dt * Vhead * cos(posePred(2));
//            posePred(1) = posePred(1) + dt * Vhead * sin(posePred(2));

            /// matching
            float errMatching;
//            poseSensor = getPoseInv(posePred);
            for (int i = 2; i >= 0; i--) slamMapSub[i].matching(pointsFrontSub[i], poseSensor, hessian, 3);
            errMatching = slamMapFinal.matching(pointsFrontFinal, poseSensor, hessian, param.match_maxiter);



            /// error
//            poseSLAM = getPose(poseSensor);
//            Vector3f errE = poseSLAM - posePred;
//            errE << fabs(errE(0)), fabs(errE(1)), fabs(errE(2));
//            hessian = 1000 * hessian.inverse().eval();
//            printf("%.6f\t%.6f\t%.6f\n", hessian(0, 0), hessian(1, 1), hessian(2, 2));


            /// filter
//            KF_CI::Vec Z, X, Err;
//            KF_CI::Mat R;
//            float scaleFactor = 0.1*0.1;
//            Z = poseSensor;
//            if (!kfposition.initX(Z))
//            {
//                R = scaleFactor * hessian.inverse();
//                kfposition.updateR(R);
//                X = kfposition.update(Z, Err);
//            }
//            printf("%.3f\t%.3f\t%.3f\n", Err(0), Err(1), Err(2));

            /// mapping
            flagMapping = timeToMapping(param.map_mapping_dist_thres);
//            bool flagFailMatching = false;
//            if ((errE(0) > 0.03 && errE(1) > 0.03) || errE(2) > 0.005)
//            {
//                flagFailMatching = true;
//                poseSensor = getPoseInv(posePred);
//            }
            if (flagMapping)
            {
                if (!param.match_without_mapping)
                {
                    for (int i = 2; i >= 0; i--) slamMapSub[i].mapping(pointsFrontSub[i], poseSensor);
                    slamMapFinal.mapping(pointsFrontFinal, poseSensor, updatedPixels);
                }
                Vector3f poseForTrace = getPoseForMap(poseSensor);
                updatedPixels.pushbackTraces(poseForTrace(0) + poseForTrace(1) * param.map_size_x);
                prevPoseSensor = poseSensor;
            }

            rangesFront.clear();
        }

        /// Road Extraction (right)
        if (!rangesRight.empty())
        {
            Points pointsRight;

            scanToPointsRight(rangesRight, intensRight, param.scan_angle_increment_right, pointsRight);

            /// mapping (road)
            vector <Vector4f> roads;
            slamMapFinal.roadExtraction(pointsRight, roads);

            /// trans
            int i, j, size = pointsRight.size();
            for (i = 0; i < size; i++) transPoint(pointsRight[i]);

            /// merge
            vector <Vector2f> tL, tR, L, R;
            vector <int> label;
            size = roads.size();
            tL.resize(size);
            tR.resize(size);
            label.resize(size);
            label.assign(size, 0);
            for (i = 0; i < size; i++) transCurb(roads[i], tL[i], tR[i]);
            curbLeft.setZero(2);
            curbRight.setZero(2);
            if (size == 1)
            {
                if (fabs(((tL[0](1) + tR[0](1)) / 2)) < param.roadext_center_width)
                {
                    curbLeft = tL[0];
                    curbRight = tR[0];
                }
            }
            if (size >= 2)
            {
                j = 0;
                label[0] = 0;
                for (i = 1; i < size; i++)
                {
                    if ((tL[i] - tR[i - 1]).norm() > 0.4) j++;
                    label[i] = j;
                }
                int labelsize = label[size - 1] + 1;
                L.resize(labelsize);
                R.resize(labelsize);

                j = 0;
                L[0] = tL[0];
                R[0] = tR[0];
                for (i = 1; i < size; i++)
                {
                    if (label[i] == j)
                    {
                        R[j] = tR[i];
                    }
                    else
                    {
                        j++;
                        L[j] = tL[i];
                        R[j] = tR[i];
                    }
                }

                if (labelsize == 1)
                {
                    curbLeft = L[0];
                    curbRight = R[0];
                }
                else
                {
                    for (i = 0; i < labelsize; i++)
                    {
                        if (fabs(((L[i] + R[i]) / 2)(1)) < param.roadext_center_width)
                        {
                            curbLeft = L[i];
                            curbRight = R[i];
                            break;
                        }
                    }
                }
            }

            vector <int> roadmarkIndex;
            size = intensRight.size();
            roadmarkIndex.resize(size);
            int newSize = 0;
            for (i = 0; i < size; i++)
            {
                if (intensRight[i] > 150)
                {
                    roadmarkIndex[newSize] = i;
                    newSize++;
                }
            }
            if (newSize == 0)
            {
                cout << "a";
            }

            roadmarkIndex.resize(newSize);
            Points roadmarkPoints;
            roadmarkPoints.resize(newSize);
            for (i = 0; i < newSize; i++) roadmarkPoints[i] = pointsRight[roadmarkIndex[i]];

            if (param.debug_mode_road == 1) slamMapFinal.roadMapping(curbLeft, curbRight, poseSensor, updatedPixels);
            if (param.debug_mode_road == 2) slamMapFinal.roadsMapping(roads, poseSensor, updatedPixels, affLiDAR4);
            if (param.debug_mode_road == 3) slamMapFinal.roadmarkMapping(roadmarkPoints, poseSensor, updatedPixels);

            rangesRight.clear();
            intensRight.clear();
        }

        return flagMapping;
    }

    inline void getCurb(Vector2f &left, Vector2f &right) { left = curbLeft; right = curbRight;}

    inline Matrix3f getCovariance() { return hessian.inverse(); }

    inline Vector3f getPose() { return Vector3f((poseSensor(0) - param.tf_lidar2_x * cos(poseSensor(2))), (poseSensor(1) - param.tf_lidar2_x * sin(poseSensor(2))), poseSensor(2)); }

    inline Vector3f getPose(Vector3f pose) { return Vector3f((pose(0) - param.tf_lidar2_x * cos(pose(2))), (pose(1) - param.tf_lidar2_x * sin(pose(2))), pose(2)); }

    inline Vector3f getPoseInv(Vector3f pose) { return Vector3f((pose(0) + param.tf_lidar2_x * cos(pose(2))), (pose(1) + param.tf_lidar2_x * sin(pose(2))), pose(2)); }

    inline Vector3f getPoseForMap()
    {
        Vector3f pose = poseSensor;
        float scale = 1 / param.map_resolution;
        return Vector3f((scale * (pose(0) - param.tf_lidar2_x * cos(pose(2))) + param.map_size_x / 2), (scale * (pose(1) - param.tf_lidar2_x * sin(pose(2))) + param.map_size_y / 2), pose(2));
    }

    inline Vector3f getPoseForMap(Vector3f pose)
    {
        float scale = 1 / param.map_resolution;
        return Vector3f((int)(scale * (pose(0) - param.tf_lidar2_x * cos(pose(2))) + param.map_size_x / 2 + 0.5), (int)(scale * (pose(1) - param.tf_lidar2_x * sin(pose(2))) + param.map_size_y / 2 + 0.5), pose(2));
    }

    inline void loadMap(vector <unsigned char> &newMap, vector <unsigned char> *newMapSub)
    {
        slamMapFinal.loadMap(newMap);
        for (int i = 0; i < 3; i++) slamMapSub[i].loadMap(newMapSub[i]);
    }

    inline void saveMap(vector <unsigned char> &newMap, vector <unsigned char> *newMapSub)
    {
        slamMapFinal.saveMap(newMap);
        for (int i = 0; i < 3; i++) slamMapSub[i].saveMap(newMapSub[i]);
    }

    inline void clearUpdatedPixels() { }

private:
    void scanToPointsFront(Ranges &ranges, float angleIncrement, Points &points, Points *pointsSub)
    {
        float angle;
        int size = ranges.size();
        int newSize = 0;

        if (size == 0) return;
        if (param.scan_dir_invert)
        {
            angle = param.scan_angle_max;
            angleIncrement = -angleIncrement;
        }
        else
        {
            angle = param.scan_angle_min;
        }
        float scale = 1 / param.map_resolution;
        float range;

        points.resize(size);
        int i, j;
        float x, y;
        for (i = 2; i >= 0; i--) pointsSub[i].resize(size);
        for (i = 0; i < size; i++)
        {
            if (ranges[i] > param.scan_range_min && ranges[i] < param.scan_range_max)
            {
                range = scale * ranges[i];
                x = range * cos(angle);
                y = range * sin(angle);
                points[newSize][0] = x;
                points[newSize][1] = y;
                for (j = 0; j < 3; j++)
                {
                    x = x / 2;
                    y = y / 2;
                    pointsSub[j][newSize][0] = x;
                    pointsSub[j][newSize][1] = y;
                }
                newSize++;
            }
            angle = angle + angleIncrement;
        }
        points.resize(newSize);
        for (int i = 2; i >= 0; i--) pointsSub[i].resize(newSize);
    }

    void scanToPointsRight(Ranges &ranges, Intens &intens, float angleIncrement, Points &points)
    {
        Intens tempIntens;

        float angle;
        int size = ranges.size();
        int newSize = 0;

        if (size == 0) return;
        if (param.scan_dir_invert)
        {
            angle = param.scan_angle_max;
            angleIncrement = -angleIncrement;
        }
        else
        {
            angle = param.scan_angle_min;
        }
        float range;

        if (intens.size() == ranges.size())
        {
            points.resize(size);
            tempIntens.resize(size);
            for (int i = 0; i < size; i++)
            {
                range = ranges[i];
                if (ranges[i] > param.scan_range_min && ranges[i] < param.scan_range_max)
                {
                    points[newSize][0] = range * cos(angle);
                    points[newSize][1] = range * sin(angle);
                    tempIntens[newSize] = intens[i];
                    newSize++;
                }
                angle = angle + angleIncrement;
            }
            points.resize(newSize);
            tempIntens.resize(newSize);
            intens = tempIntens;
        }
        else
        {
            points.resize(size);
            for (int i = 0; i < size; i++)
            {
                range = ranges[i];
                if (ranges[i] > param.scan_range_min && ranges[i] < param.scan_range_max)
                {
                    points[newSize][0] = range * cos(angle);
                    points[newSize][1] = range * sin(angle);
                    newSize++;
                }
                angle = angle + angleIncrement;
            }
            points.resize(newSize);
        }
    }

    bool timeToMapping(float distThres)
    {
        float dx = poseSensor(0) - prevPoseSensor(0);
        float dy = poseSensor(1) - prevPoseSensor(1);
        float dt = poseSensor(2) - prevPoseSensor(2);

        if (dt > M_PI) {
            dt = dt - M_2PI;
        }
        else if (dt < -M_PI)
        {
            dt = dt + M_2PI;
        }

        return (sqrt(dx * dx + dy * dy) > distThres || abs(dt) > param.map_mapping_rot_thres);
    }

    inline void transCurb(Vector4f road, Vector2f &left, Vector2f &right)
    {
        Vector3f tL, tR;
        tL << road(0), road(1), 0;
        tR << road(2), road(3), 0;
        tL = affLiDAR4 * tL;
        tR = affLiDAR4 * tR;
        left << tL(0), tL(1);
        right << tR(0), tR(1);
    }

    inline void transPoint(Vector2f &p)
    {
        Vector3f pTemp;
        pTemp << p(0), p(1), 0;
        pTemp = affLiDAR4 * pTemp;
        p << pTemp(0), pTemp(1);
    }



};




}
