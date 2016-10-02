
#pragma once

/// Eigen
#include <Eigen/Dense>
using namespace Eigen;

namespace SCV
{

template <unsigned int dim, bool flagExtend>
class KalmanFilter
{

public:
    KalmanFilter() {}
    ~KalmanFilter() {}
    typedef Matrix <float, dim, dim> Mat;
    typedef Matrix <float, dim, 1> Vec;

protected:
    Mat A, H, P, Q, R;
    Vec X;

public:
    Vec update(Vec &Z, Vec &Err)
    {
        if (flagExtend) return updateEKF(Z, Err);
        else return updateKF(Z, Err);
    }

private:
    inline Vec updateKF(Vec &Z, Vec &Err)
    {
        Mat Pp, K, Ht;
        Vec Xp;

        // prediction
        Xp = A * X;
        Pp = A * P * A.transpose() + Q;

        // get kalman gain
        Ht = H.transpose();
        K = Pp * Ht * (H * Pp * Ht + R).inverse();

        // estimation
        Err = Z - H * Xp;
        X = Xp + K * Err;

        // update cov
        P = Pp - K * H * Pp;

        return X;
    }

    inline Vec updateEKF(Vec &Z, Vec &Err)
    {
        Mat Pp, K, Ht;
        Vec Xp;

        // prediction
        Xp = fX();
        Ajacob();
        Pp = A * P * A.transpose() + Q;

        // get kalman gain
        Hjacob();
        Ht = H.transpose();
        K = Pp * Ht * (H * Pp * Ht + R).inverse();

        // estimation
        Err = Z - hXp(Xp);
        X = Xp + K * Err;

        // update cov
        P = Pp - K * H * Pp;

        return X;
    }

public:
    virtual Vec fX() {}
    virtual Vec hXp(Vec &Xp) {}
    virtual void Ajacob() {}
    virtual void Hjacob() {}

};

class EKF_position : public KalmanFilter <5, true>
{

public:
    EKF_position() { init(); }
    ~EKF_position() {}

private:
    float dt;

public:
    /// ==============================================
    /// desc
    /// ==============================================
    ///
    /// X(0) : x position from SLAM
    /// X(1) : y position from SLAM
    /// X(2) : head angular position from SLAM
    /// X(3) : head velocity from wheels
    /// X(4) : head angular velocity from 2-axis INS
    ///
    /// ==============================================


    void init()
    {
        // init X
        X(0) = 0;
        X(1) = 0;
        X(2) = 0;
        X(3) = 0;
        X(4) = 0;

        // init P
        P.setIdentity();

        // constant
        dt = 0.04;

        // jacob constant
        A.setIdentity();
        A(2, 4) = dt;
        H.setIdentity();

        // process noise
        Q.setIdentity();
        Q = Q * 0.01;

        // measurement noise
        R.setIdentity();
        R(0, 0) = 0.5;
        R(1, 1) = 0.5;
        R(2, 2) = 0.01;
        R(3, 3) = 2.0 * 10e-5;
        R(4, 4) = 0.04;

    }

    virtual Vec fX()
    {
        Vec Xp;

        Xp(0) = X(0) + dt * X(3) * cos(X(2) + dt * X(4));
        Xp(1) = X(1) + dt * X(3) * sin(X(2) + dt * X(4));
        Xp(2) = X(2) + dt * X(4);
        Xp(3) = X(3);
        Xp(4) = X(4);

        return Xp;
    }

    virtual Vec hXp(Vec &Xp) { return Xp; }

    virtual void Ajacob()
    {
        A(0, 2) = - dt * X(3) * sin(X(2) + dt * X(4));
        A(0, 3) = dt * cos(X(2) + dt * X(4));
        A(0, 4) = - dt * dt * X(3) * sin(X(2) + dt * X(4));

        A(1, 2) = dt * X(3) * cos(X(2) + dt * X(4));
        A(1, 3) = dt * sin(X(2) + dt * X(4));
        A(1, 4) = dt * dt * X(3) * cos(X(2) + dt * X(4));
    }

    virtual void Hjacob() {}

};

class KF_CI : public KalmanFilter <3, false>
{

public:
    KF_CI() { init(); }
    ~KF_CI() {}

private:
    bool firstScan;

public:
    /// ==============================================
    /// desc
    /// ==============================================
    ///
    /// X(0) : x position from SLAM
    /// X(1) : y position from SLAM
    /// X(2) : head angular position from SLAM
    /// R : update from hessian
    /// ==============================================

    void init()
    {
        firstScan = true;

        // init X
        X(0) = 0;
        X(1) = 0;
        X(2) = 0;

        // init P
        P.setIdentity();

        // proc ,obs
        A.setIdentity();
        H.setIdentity();

        // process noise
        Q.setZero(3, 3);

        // measurement noise
        R.setIdentity();
    }

    void updateR(Mat newR) { R = newR; }

    inline bool initX(Vec &newX)
    {
        if (firstScan)
        {
            firstScan = false;
            X = newX;
            return true;
        }

        return false;
    }

};


}
