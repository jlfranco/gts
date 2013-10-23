#include "colortracker.h"

#include "ColorCalibration.h"
#include "CameraCalibration.h"
#include "RobotMetrics.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cmath>
#include <string>

bool cholesky (cv::Mat & mat, cv::Mat & output)
{
    double sum;
    assert(mat.depth() == CV_32F || mat.depth() == CV_64F);
    output = cv::Mat::zeros(mat.rows, mat.cols, CV_64FC1);
    for (int i = 0; i < mat.rows; ++i)
    {
        for (int j = 0; j <= i; ++j)
        {
            if (mat.depth() == CV_32F)
            {
                sum = double(mat.at<float>(i, j));
            }
            else
            {
                sum = mat.at<double>(i, j);
            }
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

ColorTracker::ColorTracker(const CameraCalibration * cam_calib,
                           const RobotMetrics * metrics,
                           const ColorCalibration * col_calib,
                           const IplImage * currentImage) :
    m_pos      ( cv::Point2f(0, 0) ),
    m_angle    ( 0 ),
    m_error    ( 0 ),
    m_cal      ( cam_calib ),
    m_metrics  ( metrics ),
    m_colorCal ( col_calib ),
    m_currImg  ( cv::Mat(currentImage) )
{
}

ColorTracker::~ColorTracker()
{
}

/**
 Returns the position of the left edge of the brushbar
 (given that robot has the specified position and heading).
 **/
CvPoint2D32f ColorTracker::GetBrushBarLeft( CvPoint2D32f position, float heading ) const
{
    heading += M_PI / 2.f;

    float w = GetMetrics()->GetBrushBarWidthPx(); // brush bar width (pixels)

    float px = -w / 2.f; // half bbar width
    float py = m_metrics->GetBrushBarOffsetPx(); // offset of brush bar in direction of travel (from centre of robot)

    float cosa = cos( -heading );
    float sina = sin( -heading );

    float x = px * cosa - py * sina;
    float y = px * sina + py * cosa;

    return cvPoint2D32f( position.x + x, position.y + y );
}

/**
 Returns the position of the right edge of the brushbar
 (given that robot has the specified position and heading).
 **/
CvPoint2D32f ColorTracker::GetBrushBarRight( CvPoint2D32f position, float heading ) const
{
    heading += M_PI / 2.f;

    float w = GetMetrics()->GetBrushBarWidthPx(); // brush bar width (pixels)

    float px = w / 2.f; // half bbar width
    float py = m_metrics->GetBrushBarOffsetPx(); // offset of brush bar in direction of travel (from centre of robot)

    float cosa = cos( -heading );
    float sina = sin( -heading );

    float x = px * cosa - py * sina;
    float y = px * sina + py * cosa;

    return cvPoint2D32f( position.x + x, position.y + y );
}

void ColorTracker::Activate()
{
    /* Placeholder function */
    m_status = TRACKER_ACTIVE;
}

bool ColorTracker::Track(double timeStamp)
{
    /* Placeholder function */
    return false;
}

void ColorTracker::DoInactiveProcessing(double timeStamp)
{
    /* Placeholder function */
}

void ColorTracker::Rewind(double timeStamp)
{
    /* Placeholder function */
}

void ColorTracker::SetParam(paramType param, float value)
{
    /* Placeholder function */
}

int ColorTracker::largest_polygon(std::vector<std::vector<cv::Point2i> > & polygons)
{
  int best_index = -1;
  double best_area = 0;
  int current_index = 0;
  double current_area;
  std::vector<std::vector<cv::Point2i> >::iterator it;
  for (it = polygons.begin(); it != polygons.end(); ++it, ++current_index)
  {
    current_area = cv::contourArea(*it);
    if (current_area >= best_area)
    {
      best_index = current_index;
      best_area = current_area;
    }
  }
  return best_index;
}

void ColorTracker::segment_blobs(const cv::Mat & input_image,
    std::vector<std::vector<cv::Point2i> > * contours, double hue_ref,
    double hue_thr, double sat_thr)
{
  contours->clear();
  cv::Mat segmented_image = cv::Mat::zeros(
      input_image.rows, input_image.cols, CV_8UC1);
  double distance;
  double min_luminance = 0.2*255;
  double max_luminance = 0.9*255;
  double hue, saturation, luminance;
  cv::Vec3f current_pixel;
  cv::Mat small_strel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
      cv::Size(3, 3));
  cv::Mat big_strel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
      cv::Size(5, 5));
  for (int i = 0; i < input_image.rows; ++i)
  {
    for (int j = 0; j < input_image.cols; ++j)
    {
      current_pixel = input_image.at<cv::Vec3f>(i, j);
      hue = current_pixel.val[0];
      saturation = current_pixel.val[1];
      luminance = current_pixel.val[2];
      distance = abs(hue - hue_ref);
      distance = distance > 180 ? (360 - distance) : distance;
      if ( (min_luminance <= luminance && luminance <= max_luminance) &&
           (saturation >= sat_thr) && distance < hue_thr)
      {
        segmented_image.at<unsigned char>(i, j) = 255;
      }
    }
  }
  // Morphological closing to remove noise and obtain more regular shapes
  cv::erode(segmented_image, segmented_image, small_strel, cv::Point(-1, -1), 2);
  cv::dilate(segmented_image, segmented_image, small_strel, cv::Point(-1, -1), 2);
  cv::dilate(segmented_image, segmented_image, small_strel, cv::Point(-1, -1), 1);
  cv::erode(segmented_image, segmented_image, small_strel, cv::Point(-1, -1), 1);
  cv::findContours(segmented_image, *contours, CV_RETR_EXTERNAL,
      CV_CHAIN_APPROX_SIMPLE);
}

cv::Point2f ColorTracker::find_blob(const cv::Mat & input_image, double hue_ref,
    double hue_thr, double sat_thr)
{
  std::vector<std::vector<cv::Point2i> > contours;
  segment_blobs(input_image, &contours, hue_ref, hue_thr, sat_thr);
  int best_index = largest_polygon(contours);
  cv::Point2f centroid;
  double area;
  if (!contours.empty())
  {
    std::vector<cv::Point2i> best_polygon = contours[best_index];
    std::vector<cv::Point2i>::iterator it;
    area = 0;
    cv::Point2i p0, p1;
    for (it = best_polygon.begin(); it != best_polygon.end(); ++it)
    {
      p0 = *it;
      if ( (it + 1) == best_polygon.end() )
      {
        p1 = best_polygon[0];
      }
      else
      {
        p1 = *(it + 1);
      }
      area += 0.5 * double(p0.x * p1.y - p0.y * p1.x);
      centroid.x = centroid.x +
          double((p0.x + p1.x) * (p0.x * p1.y - p0.y * p1.x));
      centroid.y = centroid.y +
          double((p0.y + p1.y) * (p0.x * p1.y - p0.y * p1.x));
    }
    centroid.x = centroid.x / (6*area);
    centroid.y = centroid.y / (6*area);
  }
  else
  {
    centroid = cv::Point2f(-1., -1.);
  }
  return centroid;
}

void ColorTracker::predict(double delta_t)
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
  cholesky(cov_as_mat, decomposed_cov);
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
      new_point[0] = (*it)[0] + delta_t * (*it)[3] * cos((*it)[2]);
      new_point[1] = (*it)[1] + delta_t * (*it)[3] * sin((*it)[2]);
      new_point[2] = (*it)[2] + delta_t * (*it)[4];
      new_point[3] = (*it)[3];
      new_point[4] = (*it)[4];
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
}

void ColorTracker::update(MVec measurement, int direction)
{
  assert(direction == 1 || direction == -1);
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
      extended_covariance(i, j) = m_meas_noise_cov(i, j);
    }
  }
  // Obtain vector of sigma points and corresponding weights
  cv::Mat cov_as_mat = (7 + m_kappa) * cv::Mat(extended_covariance);
  cv::Mat decomposed_cov;
  cholesky(cov_as_mat, decomposed_cov);
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
  for (int i = 0; i < sigma_points.size(); ++i)
  {
    avg_sigma_points += sigma_weights[i] * sigma_points[i];
  }
  // Transform sigma points according to the process model
  std::vector<MVec> transformed_points;
  MVec new_point;
  std::vector<XVec7>::iterator it;
  std::vector<MVec>::iterator jt;
  for (it = sigma_points.begin(); it != sigma_points.end(); ++it)
  {
    if (direction == -1)
    {
      new_point[0] = (*it)[0] - m_dist_left * sin((*it)[2]);
      new_point[1] = (*it)[1] + m_dist_left * cos((*it)[2]);
    }
    else
    {
      new_point[0] = (*it)[0] + m_dist_right * sin((*it)[2]);
      new_point[1] = (*it)[1] - m_dist_right * cos((*it)[2]);
    }
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
  CCov cross_cov = CCov::zeros();
  // Construct cross covariance
  // ...
  for (int i = 0; i < sigma_points.size(); ++i)
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
}
