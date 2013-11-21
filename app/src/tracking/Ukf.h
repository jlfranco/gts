/* Copyright (C) 2013 iRobot Corporation. All rights reserved.
 */

#ifndef UKF_H
#define UKF_H

#include <opencv2/core/core.hpp>

typedef cv::Vec<double, 5> SVec;
typedef cv::Vec<double, 2> MVec;
typedef cv::Vec<double, 10> XVec10;
typedef cv::Vec<double, 7> XVec7;
typedef cv::Matx<double, 5, 5> SCov;
typedef cv::Matx<double, 2, 2> MCov;
typedef cv::Matx<double, 7, 2> CCov;
typedef cv::Matx<double, 10, 10> XCov10;
typedef cv::Matx<double, 7, 7> XCov7;

class Ukf
{
public:
    Ukf();
    ~Ukf();

    void initialize ( MVec pos );
    void initialize ( MVec l_blob, MVec r_blob, double distLeft, double distRight );

    /**
     * Uses process model to predict the robot's next position
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
    void update( const MVec measurement, const int model, const double modelVal );

    bool         isInitialized   () const;
    SVec         getCurrentState () const;
    bool         getError        () const;
    float        getHeading      () const;
    CvPoint2D32f getPosition     () const;
    void         setPosition     ( CvPoint2D32f pos );
    void         setCurrentState ( MVec pos, float angle, double curr3, double curr4 );

private:
    cv::Point2f m_pos;
    float m_angle;
    float m_error;

    bool m_initialized;

    SVec m_current_state;
    SCov m_current_cov;
    SCov m_proc_noise_cov;
    MCov m_meas_noise_cov;
    double m_kappa; // Used in the unscented transform

    bool cholesky (const cv::Mat & inputmat, cv::Mat & output);

};

#endif // UKF_H
