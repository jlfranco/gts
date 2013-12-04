/* Copyright (C) 2013 iRobot Corporation. All rights reserved.
 */

#include "Kalman.h"

#include "Logging.h"

#include <iostream>

Kalman::Kalman() :
    m_pos        ( cv::Point3f(0, 0, 0) ),
    m_error      ( 0 ),
    m_init       ( false ),
    m_kappa      ( 1 )
{
    m_proc_noise_cov = SCov::eye();
    m_proc_noise_cov(0, 0) = 4.;
    m_proc_noise_cov(1, 1) = 4.;
    m_proc_noise_cov(2, 2) = M_PI/40.;
    m_proc_noise_cov(3, 3) = 20.;
    m_proc_noise_cov(4, 4) = M_PI/80.;
    m_meas_noise_cov       = 2. * MCov::eye();
    m_meas_noise_cov(2,2)  = 20.;
}

Kalman::~Kalman()
{
}

void Kalman::init( MVec measurement )
{
    fprintf( stderr, "Kalman::init\n");
    m_pos.x = measurement[0];
    m_pos.y = measurement[1];
    m_pos.z = measurement[2]; // angle
    m_current_state[0] = m_pos.x;
    m_current_state[1] = m_pos.y;
    m_current_state[2] = m_pos.z;
    m_current_state[3] = 0;
    m_current_state[4] = 0;

    m_current_cov = SCov::zeros();
    m_current_cov(0, 0) = 30;
    m_current_cov(1, 1) = 30;
    m_current_cov(2, 2) = M_PI/4;
    m_current_cov(3, 3) = 200;
    m_current_cov(4, 4) = M_PI/4; //TODO verify

    m_init = true;
}

void Kalman::init( MVec measurement, double timestampMs )
{
    m_timestampMs = timestampMs;

    init( measurement );
}

void Kalman::deInit()
{
    fprintf( stderr, "Kalman::deInit\n");
    m_init = false;
}

bool Kalman::cholesky (const cv::Mat & inputmat, cv::Mat & output)
{
    double sum;
    cv::Mat mat;
    inputmat.assignTo(mat, CV_64FC1);
    output = cv::Mat::zeros(mat.rows, mat.cols, CV_64FC1);
    for (int i = 0; i < mat.rows; ++i)
    {
        for (int j = 0; j <= i; ++j)
        {
            sum = mat.at<double>(i, j);
            for (int k = 0; k < j ; ++k)
            {
                sum -= output.at<double>(i, k) * output.at<double>(j, k);
            }
            if (i == j)
            {
                if (sum <= 0)
                {
                    return false; // Matrix is not positive definite
                }
                output.at<double>(i, j) = sqrt(sum);
            }
            else
            {
                output.at<double>(i, j) = sum/output.at<double>(j, j);
            }
        }
    }
    return true; // Matrix is positive definite
}

bool Kalman::predict( double timestampMs )
{
  fprintf( stderr, "Kalman::predict\n");

  double delta_t = timestampMs - m_timestampMs;
  m_timestampMs = timestampMs;

  // First, construct the extended state vector and covariance matrix
  // to extract the sigma-points for prediction
  SVecExtPrediction extended_state;
  for (int i = 0; i < X_LEN; ++i)
  {
      extended_state[i] = m_current_state(i);
      extended_state[i+X_LEN] = 0;
  }
  CovExtPrediction extended_covariance = CovExtPrediction::zeros();
  for (int i = 0; i < X_LEN; ++i)
  {
      for (int j = 0; j < X_LEN; ++j)
      {
          extended_covariance(i, j) = m_current_cov(i, j);
          extended_covariance(i+X_LEN, j+X_LEN) = m_proc_noise_cov(i, j);
      }
  }
  // Obtain vector of sigma points and corresponding weights
  cv::Mat cov_as_mat = (10 + m_kappa) * cv::Mat(extended_covariance);
  cv::Mat decomposed_cov;
  if ( !cholesky(cov_as_mat, decomposed_cov) )
  {
      LOG_ERROR("You're not giving me a proper covariance matrix");
      m_init = false;
      return false;
  }
  std::vector<double> sigma_weights;
  std::vector<SVecExtPrediction> sigma_points;
  sigma_points.push_back(extended_state);
  sigma_weights.push_back(m_kappa/(m_kappa + 10));
  SVecExtPrediction current_col;
  for (int i = 0; i < 2*X_LEN; ++i)
  {
      decomposed_cov.col(i).copyTo(current_col);
      sigma_points.push_back(
                  extended_state + current_col);
      sigma_points.push_back(
                  extended_state - current_col);
      sigma_weights.push_back(0.5/(m_kappa + 10));
      sigma_weights.push_back(0.5/(m_kappa + 10));
  }
  // Transform sigma points according to the process model
  std::vector<SVec> transformed_points;
  SVec new_point;
  std::vector<SVecExtPrediction>::iterator it;
  std::vector<SVec>::iterator jt;
  for (it = sigma_points.begin(); it != sigma_points.end(); ++it)
  {
      new_point[0] = (*it)[0] + delta_t * (*it)[3] * cos((*it)[2]) + (*it)[5];
      new_point[1] = (*it)[1] + delta_t * (*it)[3] * sin((*it)[2]) + (*it)[6];
      new_point[2] = (*it)[2] + delta_t * (*it)[4] + (*it)[7];
      new_point[3] = (*it)[3] + (*it)[8];
      new_point[4] = (*it)[4] + (*it)[9];
      transformed_points.push_back(new_point);
  }
  // Recover new mean and covariance;
  SVec weighted_mean;
  weighted_mean *= 0; // Make sure elements are initd to 0
  std::vector<double>::iterator wt;
  for (jt = transformed_points.begin(), wt = sigma_weights.begin();
       jt != transformed_points.end(); ++jt, ++wt)
  {
      weighted_mean += (*wt) * (*jt);
  }
  SCov weighted_cov = SCov::zeros();
  for (jt = transformed_points.begin(), wt = sigma_weights.begin();
       jt != transformed_points.end(); ++jt, ++wt)
  {
      weighted_cov += (*wt) *
              (*jt - weighted_mean) * (*jt - weighted_mean).t();
  }
  m_current_state = weighted_mean;
  m_current_cov = weighted_cov;
  m_pos.x = m_current_state[0];
  m_pos.y = m_current_state[1];
  m_pos.z = m_current_state[2];
  m_error = cv::trace(m_current_cov);
  return true;
}

bool Kalman::update( const MVec measurement )
{
  fprintf( stderr, "Kalman::update\n");
  // First, construct the extended state vector and covariance matrix
  // to extract the sigma-points for prediction
  SVecExtUpdate extended_state;
  for (int i = 0; i < X_LEN; ++i)
  {
      extended_state[i] = m_current_state(i);
  }
  for (int i = X_LEN; i < X_LEN+M_LEN; i++)
      extended_state[i] = 0;
  CovExtUpdate extended_covariance = CovExtUpdate::zeros();
  for (int i = 0; i < X_LEN; ++i)
  {
      for (int j = 0; j < X_LEN; ++j)
      {
          extended_covariance(i, j) = m_current_cov(i, j);
      }
  }
  for (int i = X_LEN; i < X_LEN+M_LEN; ++i)
  {
    for (int j = X_LEN; j < X_LEN+M_LEN; ++j)
    {
      extended_covariance(i, j) = m_meas_noise_cov(i-X_LEN, j-X_LEN);
    }
  }
  // Obtain vector of sigma points and corresponding weights
  cv::Mat cov_as_mat = (X_LEN + M_LEN + m_kappa) * cv::Mat(extended_covariance);
  cv::Mat decomposed_cov;
  if ( !cholesky(cov_as_mat, decomposed_cov) )
  {
      LOG_ERROR("You're not giving me a proper covariance matrix");
      m_init = false;
      return false;
  }

  // Obtain predicted measurement by transforming sigma points
  // with measurement model
  std::vector<double> sigma_weights;
  std::vector<SVecExtUpdate> sigma_points;
  sigma_points.push_back(extended_state);
  sigma_weights.push_back(m_kappa/(m_kappa + X_LEN + M_LEN ));
  SVecExtUpdate current_col;
  for (int i = 0; i < X_LEN + M_LEN; ++i)
  {
      decomposed_cov.col(i).copyTo(current_col);
      sigma_points.push_back(
                  extended_state + current_col);
      sigma_points.push_back(
                  extended_state - current_col);
      sigma_weights.push_back(0.5/(m_kappa + X_LEN + M_LEN));
      sigma_weights.push_back(0.5/(m_kappa + X_LEN + M_LEN));
  }
  // Find weighted average of sigma points
  SVecExtUpdate avg_sigma_points;
  for (unsigned int i = 0; i < sigma_points.size(); ++i)
  {
    avg_sigma_points += sigma_weights[i] * sigma_points[i];
  }
  // Transform sigma points according to the measurement model
  std::vector<MVec> transformed_points;
  MVec new_point;
  std::vector<SVecExtUpdate>::iterator it;
  std::vector<MVec>::iterator jt;
  for (it = sigma_points.begin(); it != sigma_points.end(); ++it)
  {
    new_point[0] = (*it)[0] + (*it)[5];
    new_point[1] = (*it)[1] + (*it)[6];
    new_point[2] = (*it)[2] + (*it)[7];
    transformed_points.push_back(new_point);
  }
  // Recover new mean and covariance;
  MVec predicted_meas;
  predicted_meas *= 0; // Make sure elements are initd to 0
  std::vector<double>::iterator wt;
  for (jt = transformed_points.begin(), wt = sigma_weights.begin();
       jt != transformed_points.end(); ++jt, ++wt)
  {
      predicted_meas += (*wt) * (*jt);
  }
  MCov weighted_cov = MCov::zeros();
  for (jt = transformed_points.begin(), wt = sigma_weights.begin();
       jt != transformed_points.end(); ++jt, ++wt)
  {
      weighted_cov += (*wt) *
              (*jt - predicted_meas) * (*jt - predicted_meas).t();
  }
  // For the Kalman update, the cross covariance between the state and the
  // measurement is required - compute this
  CCov cross_cov = CCov::zeros();
  for (unsigned int i = 0; i < sigma_points.size(); ++i)
  {
    cross_cov += sigma_weights[i] *
        (sigma_points[i] - avg_sigma_points) *
        (transformed_points[i] - predicted_meas).t();
  }
  CCov kalman_gain = cross_cov * weighted_cov.inv();
  SVecExtUpdate corrected_state = extended_state +
      kalman_gain * (measurement - predicted_meas);
  CovExtUpdate corrected_cov = extended_covariance -
      kalman_gain * weighted_cov * kalman_gain.t();
  for (int i = 0; i < X_LEN; ++i)
  {
    m_current_state[i] = corrected_state[i];
    for (int j = 0; j < X_LEN; ++j)
    {
      m_current_cov(i, j) = corrected_cov(i, j);
    }
  }
  m_pos.x = m_current_state[0];
  m_pos.y = m_current_state[1];
  m_pos.z = m_current_state[2];
  m_error = cv::trace(m_current_cov);
  return true;
}

Kalman::SVec Kalman::getCurrentState() const
{
    return m_current_state;
}

bool Kalman::getError() const
{
    return m_error;
}

float Kalman::getHeading() const
{
    return m_pos.z;
}

CvPoint3D32f Kalman::getPosition() const
{
    return (CvPoint3D32f) m_pos;
}

void Kalman::setPosition( CvPoint3D32f pos )
{
    m_pos = pos;
}

void Kalman::setPosition( CvPoint2D32f pos )
{
    m_pos.x = pos.x;
    m_pos.y = pos.y;
}

void Kalman::setCurrentState( MVec pos, double linearSpeed, double angularSpeed )
{
    m_pos.x = pos[0];
    m_pos.y = pos[1];
    m_pos.z = pos[2];

    m_current_state[0] = m_pos.x;
    m_current_state[1] = m_pos.y;
    m_current_state[2] = m_pos.z;
    m_current_state[3] = linearSpeed;
    m_current_state[4] = angularSpeed;
}

bool Kalman::isInit() const
{
    return m_init;
}
