/* Copyright (C) 2013 iRobot Corporation. All rights reserved.
 */

#ifndef KALMAN_H
#define KALMAN_H

#include <opencv2/core/core.hpp>

#define M_LEN 3 // length of measurement vector
#define X_LEN 5 // length of state vector

class Kalman
{
public:

    typedef cv::Vec<double,  X_LEN> SVec;
    typedef cv::Vec<double,  M_LEN> MVec;
    typedef cv::Vec<double,  X_LEN+M_LEN> SVecExtUpdate;
    typedef cv::Vec<double,  2*X_LEN> SVecExtPrediction;
    typedef cv::Matx<double, X_LEN, X_LEN> SCov;
    typedef cv::Matx<double, M_LEN, M_LEN> MCov;
    typedef cv::Matx<double, X_LEN+M_LEN, M_LEN> CCov;
    typedef cv::Matx<double, X_LEN+M_LEN, X_LEN+M_LEN> CovExtUpdate;
    typedef cv::Matx<double, 2*X_LEN, 2*X_LEN> CovExtPrediction;

    Kalman();
    ~Kalman();

    void init ( Kalman::MVec measurement );
    void init ( Kalman::MVec measurement, double timestampMs );
    void deInit ();
    void setPosition ( Kalman::MVec pos );

    /**
     * Uses process model to predict the robot's next position.
     *
     * Process model is constante speed.
     *
     * @param delta_t Time since previous sample [ms].
     */
    bool predict( double delta_t );

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
    float        getHeading      () const;
    CvPoint3D32f getPosition     () const;
    void         setPosition     ( CvPoint2D32f pos );
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

    bool m_init;

    /*
      State vector is:
        x             [pixels   ]
        y             [pixels   ]
        theta         [degrees  ]
        linear speed  [pixels/s ]
        angular speed [degrees/s]
     */
    Kalman::SVec m_current_state;

    double m_timestampMs;

    Kalman::SCov m_current_cov;
    Kalman::SCov m_proc_noise_cov;
    Kalman::MCov m_meas_noise_cov;
    double m_kappa; // Used in the unscented transform

    bool cholesky (const cv::Mat & inputmat, cv::Mat & output);

};

#endif // KALMAN_H
