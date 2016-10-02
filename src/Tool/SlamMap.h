
#pragma once

/// Eigen
#include <Eigen/Dense>
using namespace Eigen;

/// STD
#include <vector>
#include <fstream>
using namespace std;

/// USER
#include "DataType.h"


namespace SCV
{


const float occFactor = log(0.9 / (1 - 0.9));
const float freeFactor = log(0.4 / (1 - 0.4));


class Cell
{


private:
    float occ; // logOdd
    int updateCounter;


public:
    Cell() { occ = 0; updateCounter = -1; }

    ~Cell() {}


public:
    inline void setValue(float newVal) { occ = newVal; }

    inline float getValue() { return occ; }

    inline int isOcc()
    {
        if (occ > 0) return 1;
        if (occ < 0) return -1;
        return 0;
    }

    inline bool setOcc(int updateOccCounter, int updateFreeCounter)
    {
        if (updateCounter < updateOccCounter)
        {
            if (updateCounter == updateFreeCounter)
            {
                if (occ < 30) occ = occ - freeFactor + occFactor;
                updateCounter = updateOccCounter;
                return false;
            }
            if (occ < 30) occ = occ + occFactor;
            updateCounter = updateOccCounter;
            return true;
        }

        return false;
    }

    inline bool setFree(int updateFreeCounter)
    {
        if (updateCounter < updateFreeCounter)
        {
            occ = occ + freeFactor;
            updateCounter = updateFreeCounter;
            return true;
        }

        return false;
    }

    inline float getProb()
    {
        float odds = exp(occ);
        return odds / (odds + 1.0f);
    }

};



class SlamMap
{


private:
    Parameter param;
    int sizeX, sizeY;
    int sizeMap;
    float resolution;
    Cell *cells;
    int updateCounter, updateOccCounter, updateFreeCounter;

    ///set of last updated cell
    vector <Pixel> updatedMap, updatedOccMap, updatedFreeMap;

public:
    SlamMap() { }
    ~SlamMap() { }


public:
    void init(Parameter &newParam, float res, int maxX, int maxY)
    {
        /// allocate
        resolution = res;
        sizeX = maxX;
        sizeY = maxY;
        sizeMap = sizeX * sizeY;
        cells = new Cell[sizeMap];

        /// updateCounter
        updateCounter = 0;

        /// param
        param = newParam;
    }

    float matching(Points &points, Vector3f &pose, Matrix3f &hessian, int maxIter)
    {
        Vector3f estPose;
        estPose = worldToMap(pose);

        float err;

        for (int i = 0; i < maxIter; i++)
        {
            err = findOptimizedPose(points, estPose, hessian);
        }

        estPose(2) = fmod(fmod(estPose(2), M_2PI) + M_2PI, M_2PI);
        if (estPose(2) > M_PI) estPose(2) = estPose(2) - M_2PI;

        pose = mapToWorld(estPose);

        return err;
    }

    void mapping(Points &points, Vector3f pose, ChangedPixels &updatedPixels)
    {
        Vector3f estPose;
        estPose = worldToMap(pose);

        int i, j, size = points.size();
        float startX = estPose(0);
        float startY = estPose(1);
        float endX, endY;
        float sint = sin(estPose(2)), cost = cos(estPose(2));

        updatedMap.clear();
        updatedOccMap.clear();
        updatedFreeMap.clear();
        int sx, sy, ex, ey, prevEx = -1, prevEy = -1, dx, dy, offset_dx, offset_dy, error;
        unsigned int abs_dx, abs_dy, offset, end;
        updateFreeCounter = updateCounter + 1;
        updateOccCounter = updateCounter + 2;

        /// update line
        sx = (int)(startX + 0.5f);
        sy = (int)(startY + 0.5f);
        for (i = 0; i < size; i++)
        {
            endX = estPose(0) + points[i](0) * cost - points[i](1) * sint;
            endY = estPose(1) + points[i](0) * sint + points[i](1) * cost;

            ex = (int)(endX + 0.5f);
            ey = (int)(endY + 0.5f);
            if (prevEx == ex && prevEy == ey) continue;
            if ((ex < 0) || (ex >= sizeX) || (ey < 0) || (ey >= sizeY)) return;
            prevEx = ex;
            prevEy = ey;

            dx = ex - sx;
            dy = ey - sy;
            abs_dx = abs(dx);
            abs_dy = abs(dy);
            offset_dx = dx > 0 ? 1 : -1;
            offset_dy = dy > 0 ? sizeX : -sizeX;
            offset = sy * sizeX + sx;

            if (abs_dx >= abs_dy)
            {
                error = abs_dx / 2;
                setFree(offset);
                end = abs_dx - 1;
                for (j = 0; j < end; j++)
                {
                    offset += offset_dx;
                    error += abs_dy;
                    if ((unsigned int)error >= abs_dx)
                    {
                        offset += offset_dy;
                        error -= abs_dx;
                    }
                    setFree(offset);
                }
            }
            else
            {
                error = abs_dy / 2;
                setFree(offset);
                end = abs_dy - 1;
                for (j = 0; j < end; j++)
                {
                    offset += offset_dy;
                    error += abs_dx;
                    if ((unsigned int)error >= abs_dy)
                    {
                        offset += offset_dx;
                        error -= abs_dy;
                    }
                    setFree(offset);
                }
            }
            setOcc(ey * sizeX + ex);
        }

        updateCounter = updateCounter + 3;
        classifyUpdatedMap();

        updatedPixels.pushbackOccs(updatedOccMap);
        updatedPixels.pushbackFrees(updatedFreeMap);
    }

    void mapping(Points &points, Vector3f pose)
    {
        Vector3f estPose;
        estPose = worldToMap(pose);

        int i, j, size = points.size();
        float startX = estPose(0);
        float startY = estPose(1);
        float endX, endY;
        float sint = sin(estPose(2)), cost = cos(estPose(2));

        updatedMap.clear();
        updatedOccMap.clear();
        updatedFreeMap.clear();
        int sx, sy, ex, ey, prevEx = -1, prevEy = -1, dx, dy, offset_dx, offset_dy, error;
        unsigned int abs_dx, abs_dy, offset, end;
        updateFreeCounter = updateCounter + 1;
        updateOccCounter = updateCounter + 2;

        /// update line
        sx = (int)(startX + 0.5f);
        sy = (int)(startY + 0.5f);
        for (i = 0; i < size; i++)
        {
            endX = estPose(0) + points[i](0) * cost - points[i](1) * sint;
            endY = estPose(1) + points[i](0) * sint + points[i](1) * cost;

            ex = (int)(endX + 0.5f);
            ey = (int)(endY + 0.5f);
            if (prevEx == ex && prevEy == ey) continue;
            if ((ex < 0) || (ex >= sizeX) || (ey < 0) || (ey >= sizeY)) return;
            prevEx = ex;
            prevEy = ey;

            dx = ex - sx;
            dy = ey - sy;
            abs_dx = abs(dx);
            abs_dy = abs(dy);
            offset_dx = dx > 0 ? 1 : -1;
            offset_dy = dy > 0 ? sizeX : -sizeX;
            offset = sy * sizeX + sx;

            if (abs_dx >= abs_dy)
            {
                error = abs_dx / 2;
                setFree(offset);
                end = abs_dx - 1;
                for (j = 0; j < end; j++)
                {
                    offset += offset_dx;
                    error += abs_dy;
                    if ((unsigned int)error >= abs_dx)
                    {
                        offset += offset_dy;
                        error -= abs_dx;
                    }
                    setFree(offset);
                }
            }
            else
            {
                error = abs_dy / 2;
                setFree(offset);
                end = abs_dy - 1;
                for (j = 0; j < end; j++)
                {
                    offset += offset_dy;
                    error += abs_dx;
                    if ((unsigned int)error >= abs_dy)
                    {
                        offset += offset_dx;
                        error -= abs_dy;
                    }
                    setFree(offset);
                }
            }
            setOcc(ey * sizeX + ex);
        }

        updateCounter = updateCounter + 3;
    }

    void roadExtraction(Points &points, vector <Vector4f> &roads)
    {
        if (points.empty()) return;

        Points roadPoints;
        int i, j, newSize = 0, size = points.size();
        roadPoints.resize(size);

        /// filter
        float tempY, upY = param.roadext_safety_width + param.tf_lidar4_y, downY = -param.roadext_safety_width + param.tf_lidar4_y;
        for (i = 0; i < size; i++)
        {
            tempY = points[i](1);
            if (tempY < upY && tempY > downY)
            {
                roadPoints[newSize] = points[i];
                newSize++;
            }
        }
        roadPoints.resize(newSize);

        size = roadPoints.size();
        vector <bool> flagBP;

        roads.clear();
        if (size == 0) return;

        // break point
        float dis;
        flagBP.assign(size, false);
        for (i = 1; i < size; i++)
        {
            dis = (roadPoints[i - 1] - roadPoints[i]).norm();
            if (dis > param.roadext_bp_dis_thres)
            {
                flagBP[i] = true;
                flagBP[i - 1] = true;
            }
        }

        // line
        int iStart, iEnd = 0, indexDMax;
        int lineStart, lineEnd;
        float dMax = -1;
        int size2 = size - 1;
        Vector2f alpha, aT, sV, eV, pose;
        float sX, sY, eX, eY, angle, length;

        while (iEnd < size)
        {
            iStart = iEnd;
            iEnd++;
            if (iEnd >= size) break;
            while (!flagBP[iEnd])
            {
                iEnd++;
                if (iEnd >= size2) break;
            }
            if (iEnd >= size) break;

            while (iEnd - iStart > 1)
            {
                sX = roadPoints[iStart](0);
                sY = roadPoints[iStart](1);
                eX = roadPoints[iEnd](0);
                eY = roadPoints[iEnd](1);
                alpha << eX - sX, eY - sY;

                lineStart = iStart + 1;
                lineEnd = iEnd - 1;
                dMax = -1;

                for (j = lineStart; j < lineEnd; j++)
                {
                    aT << roadPoints[j](0) - eX, roadPoints[j](1) - eY;
                    dis = (aT - (alpha.dot(aT) / (alpha(0)*alpha(0) + alpha(1)*alpha(1))) * alpha).norm();

                    if (dis > dMax)
                    {
                        indexDMax = j;
                        dMax = dis;
                    }
                }
                angle = fabs(atan2(eY - sY, eX - sX) + M_PI_2);

                if (angle < param.roadext_angle_thres)
                {
                    if (dMax > param.roadext_dis_thres)
                    {
                        iEnd = indexDMax;
                    }
                    else
                    {
                        sV << sX, sY;
                        eV << eX, eY;
                        length = (eV - sV).norm();
                        if (length > param.roadext_length_thres)
                        {
                            if (abs(iStart - indexDMax) < param.roadext_diff_index_thres)
                            {
                                sX = roadPoints[indexDMax](0);
                                sY = roadPoints[indexDMax](1);
                                sV << sX, sY;
                            }
                            getLinePose(sV, eV, pose);
                            if (fabs(pose(1) + param.tf_lidar4_ay) < 0.04)
                            {
                                roads.push_back(Vector4f(sX, sY, eX, eY));
                            }
                        }
                        break;
                    }
                }
                else
                {
                    if (dMax > param.roadext_dis_break_thres)
                    {
                        iEnd = indexDMax;
                    }
                    else
                    {
                        break;
                    }
                }

            }
        }

    }

    void roadsMapping(vector <Vector4f> &roadBoundary, Vector3f pose, ChangedPixels &updatedPixels, Affine3f &aff)
    {
        if (roadBoundary.empty()) return;

//        struct timeval timeStart;
//        struct timeval timeEnd;
//        double timeSample;

//        gettimeofday(&timeStart, NULL);

        Vector4f road;
        Pixels updatedRoad;

        int i, j, size;
        float startX, startY, endX, endY;
        int sx, sy, ex, ey, dx, dy, offset_dx, offset_dy, error;
        unsigned int abs_dx, abs_dy, offset, end;
        float cost = cos(pose(2)), sint = sin(pose(2));

        pose = getVehiclePose(pose);
        size = roadBoundary.size();
        for (i = 0; i < size; i++)
        {
            road = roadBoundary[i];

            Vector3f tR, tL;
            tR << road(0), road(1), 0;
            tL << road(2), road(3), 0;
            tR = aff * tR;
            tL = aff * tL;
            road << tR(0), tR(1), tL(0), tL(1);

            road = road / resolution;

            /// update line
            startX = road(0) * cost - road(1) * sint + pose(0);
            startY = road(0) * sint + road(1) * cost + pose(1);
            sx = (int)(startX + 0.5f);
            sy = (int)(startY + 0.5f);

            endX = road(2) * cost - road(3) * sint + pose(0);
            endY = road(2) * sint + road(3) * cost + pose(1);
            ex = (int)(endX + 0.5f);
            ey = (int)(endY + 0.5f);

            dx = ex - sx;
            dy = ey - sy;
            if (dx == 0 && dy == 0) continue;
            abs_dx = abs(dx);
            abs_dy = abs(dy);
            offset_dx = dx > 0 ? 1 : -1;
            offset_dy = dy > 0 ? sizeX : -sizeX;
            offset = sy * sizeX + sx;

            if (abs_dx >= abs_dy)
            {
                error = abs_dx / 2;
                updatedRoad.push_back(offset);
                end = abs_dx - 1;
                for (j = 0; j < end; j++)
                {
                    offset += offset_dx;
                    error += abs_dy;
                    if ((unsigned int)error >= abs_dx)
                    {
                        offset += offset_dy;
                        error -= abs_dx;
                    }
                    updatedRoad.push_back(offset);
                }
            }
            else
            {
                error = abs_dy / 2;
                updatedRoad.push_back(offset);
                end = abs_dy - 1;
                for (j = 0; j < end; j++)
                {
                    offset += offset_dy;
                    error += abs_dx;
                    if ((unsigned int)error >= abs_dy)
                    {
                        offset += offset_dx;
                        error -= abs_dy;
                    }
                    updatedRoad.push_back(offset);
                }
            }
            updatedRoad.push_back(ey * sizeX + ex);

        }
        updatedPixels.pushbackRoads(updatedRoad);

//        gettimeofday(&timeEnd, NULL);
//        timeSample = (double)(timeEnd.tv_sec)+(double)(timeEnd.tv_usec)/1000000.0-(double)(timeStart.tv_sec)-(double)(timeStart.tv_usec)/1000000.0;
//        std::cout << timeSample << "\n";
    }

    void roadMapping(Vector2f left, Vector2f right, Vector3f pose, ChangedPixels &updatedPixels)
    {
        Vector4f road;
        Pixels updatedRoad;

        int j;
        float startX, startY, endX, endY;
        int sx, sy, ex, ey, dx, dy, offset_dx, offset_dy, error;
        unsigned int abs_dx, abs_dy, offset, end;
        float cost = cos(pose(2)), sint = sin(pose(2));

        pose = getVehiclePose(pose);
        road << left(0), left(1), right(0), right(1);
        road = road / resolution;

        /// update line
        startX = road(0) * cost - road(1) * sint + pose(0);
        startY = road(0) * sint + road(1) * cost + pose(1);
        sx = (int)(startX + 0.5f);
        sy = (int)(startY + 0.5f);

        endX = road(2) * cost - road(3) * sint + pose(0);
        endY = road(2) * sint + road(3) * cost + pose(1);
        ex = (int)(endX + 0.5f);
        ey = (int)(endY + 0.5f);

        dx = ex - sx;
        dy = ey - sy;
        if (dx == 0 && dy == 0) return;
        abs_dx = abs(dx);
        abs_dy = abs(dy);
        offset_dx = dx > 0 ? 1 : -1;
        offset_dy = dy > 0 ? sizeX : -sizeX;
        offset = sy * sizeX + sx;

        if (abs_dx >= abs_dy)
        {
            error = abs_dx / 2;
            updatedRoad.push_back(offset);
            end = abs_dx - 1;
            for (j = 0; j < end; j++)
            {
                offset += offset_dx;
                error += abs_dy;
                if ((unsigned int)error >= abs_dx)
                {
                    offset += offset_dy;
                    error -= abs_dx;
                }
                updatedRoad.push_back(offset);
            }
        }
        else
        {
            error = abs_dy / 2;
            updatedRoad.push_back(offset);
            end = abs_dy - 1;
            for (j = 0; j < end; j++)
            {
                offset += offset_dy;
                error += abs_dx;
                if ((unsigned int)error >= abs_dy)
                {
                    offset += offset_dx;
                    error -= abs_dy;
                }
                updatedRoad.push_back(offset);
            }
        }
        updatedRoad.push_back(ey * sizeX + ex);
        updatedPixels.pushbackRoads(updatedRoad);
    }

    void roadmarkMapping(Points &roadmarkPoints, Vector3f pose, ChangedPixels &updatedPixels)
    {
        Pixels updatedRoad;
        Vector2f p;

        float cost = cos(pose(2)), sint = sin(pose(2));
        pose = getVehiclePose(pose);

        int i, ex, ey, size = roadmarkPoints.size();
        float x, y;
        for (i = 0; i < size; i++)
        {
            p = roadmarkPoints[i] / resolution;
            x = p(0) * cost - p(1) * sint + pose(0);
            y = p(0) * sint + p(1) * cost + pose(1);
            ex = (int)(x + 0.5f);
            ey = (int)(y + 0.5f);
            if (ex < 0 || ey < 0 || ex >= sizeX || ey >= sizeY) continue;
            updatedRoad.push_back(ey * sizeX + ex);
        }
        updatedPixels.pushbackRoads(updatedRoad);
    }

    inline int getSizeX() { return sizeX; }

    inline int getSizeY() { return sizeY; }

    inline float getResolution() { return resolution; }

    inline vector <Pixel> * getUpdatedOccMap() { return &updatedOccMap; }

    inline vector <Pixel> * getUpdatedFreeMap() { return &updatedFreeMap; }

    inline void clearUpdatedMap() { updatedOccMap.clear(); updatedFreeMap.clear(); }

    void loadMap(vector <unsigned char> &newMap)
    {
        int i, size = newMap.size();
        float val;

        for (i = 0; i < size; i++)
        {
            val = 0.0;
            if (newMap[i] == 0) val = 30.0f;
            else if (newMap[i] == 255) val = -1.0f;
            cells[i].setValue(val);
        }
    }

    void saveMap(vector <unsigned char> &newMap)
    {
        int i, size = newMap.size();
        unsigned char pix;
        float val;

        for (i = 0; i < size; i++)
        {
            pix = 128;
            val = cells[i].getValue();
            if (val > 0) pix = 0;
            else if (val < 0) pix = 255;
            newMap[i] = pix;
        }
    }

private:
    inline Vector3f worldToMap(Vector3f pose) { return Vector3f(pose(0) / resolution + sizeX / 2, pose(1) / resolution + sizeY / 2, pose(2)); }

    inline Vector4f worldToMap(Vector4f road) { return Vector4f(road(0) / resolution + sizeX / 2, road(1) / resolution + sizeY / 2, road(2) / resolution + sizeX / 2, road(3) / resolution + sizeY / 2); }

    inline Vector3f mapToWorld(Vector3f pose) { return Vector3f((pose(0) - sizeX / 2) * resolution, (pose(1) - sizeY / 2) * resolution, pose(2)); }

    inline void setOcc(int off)
    {
        if (off >= sizeMap) return;
        if (cells[off].setOcc(updateOccCounter, updateFreeCounter)) updatedMap.push_back(off);
    }

    inline void setFree(int off)
    {
        if (off >= sizeMap) return;
        if (cells[off].setFree(updateFreeCounter)) updatedMap.push_back(off);
    }

    inline void classifyUpdatedMap()
    {
        int f, i, size = updatedMap.size();
        for (i = 0; i < size; i++)
        {
            f = cells[updatedMap[i]].isOcc();
            if (f == 1)
            {
                updatedOccMap.push_back(updatedMap[i]);
            }
            else if (f == -1)
            {
                updatedFreeMap.push_back(updatedMap[i]);
            }
        }
    }

    void interpolation(float x, float y, float &occ, float &gx, float &gy)
    {
        float x0, y0, x1, y1;
        float yy0, y1y0, y1y, xx0, x1x0, x1x;
        float dyF, dyB, dxF, dxB;
        float occ11, occ01, occ10, occ00;

        if (x < 0 || y < 0 || x >= sizeX - 1 || y >= sizeY - 1) return;

        x0 = (float)((int)x);
        x1 = x0 + 1;
        y0 = (float)((int)y);
        y1 = y0 + 1;

        yy0 = y - y0;
        y1y0 = y1 - y0;
        y1y = y1 - y;
        xx0 = x - x0;
        x1x0 = x1 - x0;
        x1x = x1 - x;

        dyF = yy0 / y1y0;
        dyB = y1y / y1y0;
        dxF = xx0 / x1x0;
        dxB = x1x / x1x0;

        int offset = y0 * sizeX + x0;
        if (offset < 0 || offset >= sizeMap) return;
        occ00 = cells[offset].getProb();
        occ01 = cells[offset + sizeX].getProb();
        occ10 = cells[offset + 1].getProb();
        occ11 = cells[offset + sizeX + 1].getProb();

        occ = (dyF * (dxF * occ11 +  dxB * occ01) + dyB * (dxF * occ10 + dxB * occ00));
        gx = (dyF * (occ11 - occ01) + dyB * (occ10 - occ00));
        gy = (dxF * (occ11 - occ10) + dxB * (occ01 - occ00));
    }

    float findOptimizedPose(Points &points, Vector3f &estPose, Matrix3f &hessian)
    {
        Vector3f dEstPose;
        Matrix3f H;
        float x, y, tx, ty, occ, gx, gy, dRot;
        float sint;
        float cost;
        int i, size = points.size();
        float err = 0;

        float h11 = 0, h22 = 0, h33 = 0;
        float h12 = 0, h13 = 0, h23 = 0;
        float e1 = estPose(0), e2 = estPose(1), e3 = estPose(2);
        float de1 = 0, de2 = 0, de3 = 0;

        H.setZero(3, 3);
        dEstPose.setZero(3);
        sint = sin(e3);
        cost = cos(e3);
        for (i = 0; i < size; i++)
        {
            x = points[i](0);
            y = points[i](1);
            tx = e1 + x * cost - y * sint;
            ty = e2 + x * sint + y * cost;
            if (tx >= sizeX || ty >= sizeY || tx < 0 || ty < 0) continue;
            interpolation(tx, ty, occ, gx, gy);
            occ = 1.0f - occ;
            err = err + occ;
            dRot = (-sint * x - cost * y) * gx + (cost * x - sint * y) * gy;


            h11 = h11 + gx * gx;
            h22 = h22 + gy * gy;
            h33 = h33 + dRot * dRot;
            h12 = h12 + gx * gy;
            h13 = h13 + gx * dRot;
            h23 = h23 + gy * dRot;

            de1 = de1 + gx * occ;
            de2 = de2 + gy * occ;
            de3 = de3 + dRot * occ;
        }

        if (h11 != 0.0f && h22 != 0.0f)
        {
            H << h11, h12, h13,
                 h12, h22, h23,
                 h13, h23, h33;
            hessian = H;
            dEstPose << de1, de2, de3;

            dEstPose = H.inverse() * dEstPose;
            if (dEstPose(2) > 0.2f)
            {
                dEstPose(2) = 0.2f;
            }
            else if (dEstPose(2) < -0.2f)
            {
                dEstPose(2) = -0.2f;
            }
            estPose = estPose + dEstPose;
        }

        return err;
    }

    inline void getLinePose(Vector2f pStart, Vector2f pEnd, Vector2f &pose)
    {
        float rangeS = pStart.norm();
        float rangeE = pEnd.norm();
        float angS = atan2(pStart(1), pStart(0)) + M_PI;
        float angE = atan2(pEnd(1), pEnd(0)) + M_PI;
        float tanS = tanf(angS);
        float tanE = tanf(angE);
        float hpCS = (param.tf_center_z) / (rangeS * cosf(angS));
        float hpCE = (param.tf_center_z) / (rangeE * cosf(angE));
        float tanSmE = tanS - tanE;

        pose(0) = asin((hpCS - hpCE) / tanSmE);
        pose(1) = asin(((-hpCS * tanE + hpCE * tanS) / tanSmE) / cos(pose(0)));
    }

    inline Vector3f getVehiclePose(Vector3f pose)
    {
        float scale = 1 / param.map_resolution;
        return Vector3f((int)(scale * (pose(0) - param.tf_lidar2_x * cos(pose(2))) + param.map_size_x / 2 + 0.5), (int)(scale * (pose(1) - param.tf_lidar2_x * sin(pose(2))) + param.map_size_y / 2 + 0.5), pose(2));
    }

};



} // end namespace
