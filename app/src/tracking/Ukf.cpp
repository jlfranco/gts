/* Copyright (C) 2013 iRobot Corporation. All rights reserved.
 */

#include "Ukf.h"

Ukf::Ukf() :
    m_pos               ( cv::Point2f(0, 0) ),
    m_angle             ( 0 ),
    m_error             ( 0 ),
    m_kappa             ( 1 ), // This is to try out n + kappa = 3
    m_initialized       ( false )

{
    m_proc_noise_cov = 2 * SCov::eye();
    m_proc_noise_cov(0, 0) = 4.;
    m_proc_noise_cov(1, 1) = 4.;
    m_proc_noise_cov(2, 2) = M_PI/40.;
    m_proc_noise_cov(3, 3) = 20.;
    m_proc_noise_cov(4, 4) = M_PI/80.;
    m_meas_noise_cov = 2. * MCov::eye();
}

Ukf::~Ukf()
{
}

void Ukf::initialize( MVec l_blob, MVec r_blob, double distLeft, double distRight )
{
  MVec pos = 0.5 * ( (1 - distRight + distLeft) * r_blob +
                     (1 + distRight - distLeft) * l_blob );
  MVec dif = l_blob - r_blob;
  m_angle = atan2(-dif[0], dif[1]);
  m_angle = m_angle>M_PI?m_angle-2*M_PI:m_angle;
  m_current_state[2] = m_angle;
  initialize( pos );
}

void Ukf::initialize( MVec pos )
{
  m_pos.x = pos[0];
  m_pos.y = pos[1];
  m_current_state[0] = m_pos.x;
  m_current_state[1] = m_pos.y;
  m_current_state[3] = 0;
  m_current_state[4] = 0;

  m_current_cov = SCov::zeros();
  m_current_cov(0, 0) = 30;
  m_current_cov(1, 1) = 30;
  m_current_cov(2, 2) = M_PI/4;
  m_current_cov(3, 3) = 200;
  m_current_cov(4, 4) = M_PI/8;

  m_initialized = true;
}

bool Ukf::cholesky (const cv::Mat & inputmat, cv::Mat & output)
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

void Ukf::predict(double delta_t)
{
  // First, construct the extended state vector and covariance matrix
  // to extract the sigma-points for prediction
  XVec10 extended_state;
  for (int i = 0; i < 5; ++i)
  {
      extended_state[i] = m_current_state(i);
      extended_state[i+5] = 0;
  }
  XCov10 extended_covariance = XCov10::zeros();
  for (int i = 0; i < 5; ++i)
  {
      for (int j = 0; j < 5; ++j)
      {
          extended_covariance(i, j) = m_current_cov(i, j);
          extended_covariance(i+5, j+5) = m_proc_noise_cov(i, j);
      }
  }
  // Obtain vector of sigma points and corresponding weights
  cv::Mat cov_as_mat = (10 + m_kappa) * cv::Mat(extended_covariance);
  cv::Mat decomposed_cov;
  assert( cholesky(cov_as_mat, decomposed_cov) &&
          "Error - you're not giving me a proper covariance matrix");
  std::vector<double> sigma_weights;
  std::vector<XVec10> sigma_points;
  sigma_points.push_back(extended_state);
  sigma_weights.push_back(m_kappa/(m_kappa + 10));
  XVec10 current_col;
  for (int i = 0; i < 10; ++i)
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
  std::vector<XVec10>::iterator it;
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
  weighted_mean *= 0; // Make sure elements are initialized to 0
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
  m_angle = m_current_state[2];
  m_error = cv::trace(m_current_cov);
}

void Ukf::update( const MVec measurement, const int model, const double modelVal )
{
  // Direction can be -1 for left or 1 for right
  assert(model == 1 || model == 0 || model == -1);
  // First, construct the extended state vector and covariance matrix
  // to extract the sigma-points for prediction
  XVec7 extended_state;
  for (int i = 0; i < 5; ++i)
  {
      extended_state[i] = m_current_state(i);
  }
  extended_state[5] = 0;
  extended_state[6] = 0;
  XCov7 extended_covariance = XCov7::zeros();
  for (int i = 0; i < 5; ++i)
  {
      for (int j = 0; j < 5; ++j)
      {
          extended_covariance(i, j) = m_current_cov(i, j);
      }
  }
  for (int i = 5; i < 7; ++i)
  {
    for (int j = 5; j < 7; ++j)
    {
      extended_covariance(i, j) = m_meas_noise_cov(i-5, j-5);
    }
  }
  // Obtain vector of sigma points and corresponding weights
  cv::Mat cov_as_mat = (7 + m_kappa) * cv::Mat(extended_covariance);
  cv::Mat decomposed_cov;
  assert( cholesky(cov_as_mat, decomposed_cov) &&
          "Error - you're not giving me a proper covariance matrix");

  // Obtain predicted measurement by transforming sigma points
  // with measurement model
  std::vector<double> sigma_weights;
  std::vector<XVec7> sigma_points;
  sigma_points.push_back(extended_state);
  sigma_weights.push_back(m_kappa/(m_kappa + 7));
  XVec7 current_col;
  for (int i = 0; i < 7; ++i)
  {
      decomposed_cov.col(i).copyTo(current_col);
      sigma_points.push_back(
                  extended_state + current_col);
      sigma_points.push_back(
                  extended_state - current_col);
      sigma_weights.push_back(0.5/(m_kappa + 7));
      sigma_weights.push_back(0.5/(m_kappa + 7));
  }
  // Find weighted average of sigma points
  XVec7 avg_sigma_points;
  for (unsigned int i = 0; i < sigma_points.size(); ++i)
  {
    avg_sigma_points += sigma_weights[i] * sigma_points[i];
  }
  // Transform sigma points according to the measurement model
  std::vector<MVec> transformed_points;
  MVec new_point;
  std::vector<XVec7>::iterator it;
  std::vector<MVec>::iterator jt;
  for (it = sigma_points.begin(); it != sigma_points.end(); ++it)
  {
    new_point[0] = (*it)[0] + model*modelVal * sin((*it)[2]) + (*it)[5];
    new_point[1] = (*it)[1] - model*modelVal * cos((*it)[2]) + (*it)[6];
    transformed_points.push_back(new_point);
  }
  // Recover new mean and covariance;
  MVec predicted_meas;
  predicted_meas *= 0; // Make sure elements are initialized to 0
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
  XVec7 corrected_state = extended_state +
      kalman_gain * (measurement - predicted_meas);
  XCov7 corrected_cov = extended_covariance -
      kalman_gain * weighted_cov * kalman_gain.t();
  for (int i = 0; i < 5; ++i)
  {
    m_current_state[i] = corrected_state[i];
    for (int j = 0; j < 5; ++j)
    {
      m_current_cov(i, j) = corrected_cov(i, j);
    }
  }
  m_pos.x = m_current_state[0];
  m_pos.y = m_current_state[1];
  m_angle = m_current_state[2];
  m_error = cv::trace(m_current_cov);
}

SVec Ukf::getCurrentState() const
{
    return m_current_state;
}

bool Ukf::getError() const
{
    return m_error;
}

float Ukf::getHeading() const
{
    return m_angle;
}

CvPoint2D32f Ukf::getPosition() const
{
    return (CvPoint2D32f) m_pos;
}

void Ukf::setPosition( CvPoint2D32f pos )
{
    m_pos = pos;
}

void Ukf::setCurrentState( MVec pos, float angle, double curr3, double curr4 )
{
    m_pos.x = pos[0];
    m_pos.y = pos[1];
    m_angle = angle;

    m_current_state[0] = m_pos.x;
    m_current_state[1] = m_pos.y;
    m_current_state[2] = m_angle;
    m_current_state[3] = curr3;
    m_current_state[4] = curr4;
}

bool Ukf::isInitialized() const
{
    return m_initialized;
}
