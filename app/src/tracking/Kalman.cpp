/* Copyright (C) 2013 iRobot Corporation. All rights reserved.
 */

#include "Kalman.h"
#include "Angles.h"

#include "Logging.h"

#include <iostream>

Kalman::Kalman() :
    m_pos        ( cv::Point2f( 0, 0 ) ),
    m_error      ( 0 ),
    m_init       ( false ),
    m_kappa      ( 1 )
{
    KF = cv::KalmanFilter( X_LEN, M_LEN, 0 );
    KF.transitionMatrix = *(cv::Mat_<float>(X_LEN, X_LEN) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
}

Kalman::~Kalman()
{
}

void Kalman::init( MVec measurement )
{

    KF.statePre.at<float>(0) = measurement[0];
    KF.statePre.at<float>(1) = measurement[1];
    KF.statePre.at<float>(2) = 0;
    KF.statePre.at<float>(3) = 0;

    setIdentity(KF.measurementMatrix);

    //    setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-4));
    KF.processNoiseCov.at<float>(0, 0) = 1.5f; // pixels
    KF.processNoiseCov.at<float>(1, 1) = 1.5f; // pixels
    KF.processNoiseCov.at<float>(2, 2) = 4.f;
    KF.processNoiseCov.at<float>(3, 3) = 4.f;

    //    setIdentity(KF.measurementNoiseCov, cv::Scalar::all(0.5));
    KF.measurementNoiseCov.at<float>(0,0) = 0.5f;
    KF.measurementNoiseCov.at<float>(1,1) = 0.5f;

    //    setIdentity(KF.errorCovPost, cv::Scalar::all(.1));
    KF.errorCovPost.at<float>(0, 0) = 1.5f;
    KF.errorCovPost.at<float>(1, 1) = 1.5f;
    KF.errorCovPost.at<float>(2, 2) = 20.f;
    KF.errorCovPost.at<float>(3, 3) = 20.f;

    fprintf( stderr, "Kalman::init\n");
    setPosition( measurement );

    m_init = true;
}

void Kalman::deInit()
{
    fprintf( stderr, "Kalman::deInit\n");
    m_init = false;
}

void Kalman::setPosition( Kalman::MVec pos )
{
    m_pos.x = pos[0];
    m_pos.y = pos[1];

    KF.statePre.at<float>(0) = pos[0];
    KF.statePre.at<float>(1) = pos[1];
    KF.statePre.at<float>(2) = 0;
    KF.statePre.at<float>(3) = 0;
}

bool Kalman::predict()
{
  cv::Mat kfPrediction = KF.predict();
  m_pos.x = kfPrediction.at<float>(0);
  m_pos.y = kfPrediction.at<float>(1);

  m_error = cv::sum(cv::trace(kfPrediction))[0];
  return true;
}

bool Kalman::update( const MVec measurement )
{
  cv::Mat_<float> meas(2,1); meas.setTo(cv::Scalar(0));
  meas(0) = measurement[0];
  meas(1) = measurement[1];

  cv::Mat kfCorrected = KF.correct( meas );

  m_pos.x = kfCorrected.at<float>(0);
  m_pos.y = kfCorrected.at<float>(1);
  m_error = cv::sum(cv::trace(kfCorrected))[0];

  return true;
}

Kalman::SVec Kalman::getCurrentState() const
{
    return KF.statePost;
}

Kalman::SCov Kalman::getCurrentCov() const
{
    return KF.errorCovPost;
}

float Kalman::getError() const
{
    return m_error;
}

CvPoint2D32f Kalman::getPosition() const
{
    return (CvPoint2D32f) m_pos;
}

void Kalman::setPosition( CvPoint2D32f pos )
{
    m_pos = pos;
}

bool Kalman::isInit() const
{
    return m_init;
}
