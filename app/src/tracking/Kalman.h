/* Copyright (C) 2013 iRobot Corporation. All rights reserved.
 */

#ifndef KALMAN_H
#define KALMAN_H

#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>

#define M_LEN 2 // length of measurement vector
#define X_LEN 4 // length of state vector

class Kalman
{
public:

    typedef cv::Vec<double,  X_LEN> SVec;
    typedef cv::Vec<double,  M_LEN> MVec;
    typedef cv::Matx<double, X_LEN, X_LEN> SCov;

    Kalman();
    ~Kalman();

    void init ( Kalman::MVec measurement );
    void deInit ();
    void setPosition ( Kalman::MVec pos );

    /**
     * Uses process model to predict the robot's next position.
     *
     * Process model is constant speed.
     *
     */
    bool predict();

    /**
     * Uses one of three measurement models (right, left or center) to
     * correct the predicted position.
     *
     * @param measurement New measurement.
     * @param model The measurement model to be used.
     * @param val The value for the given model.
     */
    bool update( const Kalman::MVec measurement );

    bool         isInit          () const;
    SVec         getCurrentState () const;
    SCov         getCurrentCov   () const;
    float        getError        () const;
    CvPoint2D32f getPosition     () const;
    void         setPosition     ( CvPoint2D32f pos );
    void         setCurrentState ( Kalman::MVec pos, double linearSpeed, double angularSpeed );

private:
    /*
      Position is [x,y,theta]:
        x       [pixels ]
        y       [pixels ]
     */
    cv::Point2f m_pos;

    float m_error;

    bool m_init;

    cv::KalmanFilter KF;

    /*
      State vector is:
        x             [pixels   ]
        y             [pixels   ]
        linear speed  [pixels/s ]
        angular speed [degrees/s]
     */
    Kalman::SVec m_current_state;

    Kalman::SCov m_current_cov;
    double m_kappa; // Used in the unscented transform

    bool cholesky (const cv::Mat & inputmat, cv::Mat & output);

};

#endif // KALMAN_H
