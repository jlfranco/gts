/* Copyright (C) 2013 iRobot Corporation. All rights reserved.
 */

#ifndef KALMAN_H
#define KALMAN_H

#include <opencv2/core/core.hpp>

#define X_LEN 5

class Kalman
{
public:

    typedef cv::Vec<double,  X_LEN> SVec;
    typedef cv::Vec<double,  3> MVec;
    typedef cv::Vec<double,  2*X_LEN> SVecExt;
    typedef cv::Matx<double, X_LEN, X_LEN> SCov;
    typedef cv::Matx<double, 3, 3> MCov3x3;
    typedef cv::Matx<double, 2*X_LEN, 3> CCov;
    typedef cv::Matx<double, 2*X_LEN, 2*X_LEN> CovExt;

    Kalman();
    ~Kalman();

    void initialize ( Kalman::MVec measurement );

    /**
     * Uses process model to predict the robot's next position.
     *
     * Process model is constante speed.
     *
     * @param delta_t Time since previous sample [ms].
     */
    void predict( double delta_t );

    /**
     * Uses one of three measurement models (right, left or center) to
     * correct the predicted position.
     *
     * @param measurement New measurement.
     * @param model The measurement model to be used.
     * @param val The value for the given model.
     */
    void update( const Kalman::MVec measurement );

    bool         isInitialized   () const;
    SVec         getCurrentState () const;
    bool         getError        () const;
    float        getHeading      () const;
    CvPoint3D32f getPosition     () const;
    void         setPosition     ( CvPoint3D32f pos );
    void         setCurrentState ( Kalman::MVec pos, double linearSpeed, double angularSpeed );

private:
    /*
      Position is [x,y,theta]:
        x       [pixels ]
        y       [pixels ]
        heading [degrees]
     */
    cv::Point3f m_pos;

    float m_error;

    bool m_initialized;

    /*
      State vector is:
        x             [pixels   ]
        y             [pixels   ]
        theta         [degrees  ]
        linear speed  [pixels/s ]
        angular speed [degrees/s]
     */
    Kalman::SVec m_current_state;

    Kalman::SCov m_current_cov;
    Kalman::SCov m_proc_noise_cov;
    Kalman::MCov3x3 m_meas_noise_cov;
    double m_kappa; // Used in the unscented transform

    bool cholesky (const cv::Mat & inputmat, cv::Mat & output);

};

#endif // KALMAN_H
