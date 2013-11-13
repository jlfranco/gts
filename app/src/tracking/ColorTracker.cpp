#include "ColorTracker.h"

#include "ColorCalibration.h"
#include "CameraCalibration.h"
#include "RobotMetrics.h"

#include "Logging.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cmath>
#include <fstream>
#include <string>
#include <iostream>

bool cholesky (cv::Mat & inputmat, cv::Mat & output)
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

ColorTracker::ColorTracker(const CameraCalibration * cam_calib,
                           const RobotMetrics * metrics,
                           const ColorCalibration * col_calib,
                           const IplImage * currentImage) :
    m_pos               ( cv::Point2f(0, 0) ),
    m_angle             ( 0 ),
    m_error             ( 0 ),
    m_cal               ( cam_calib ),
    m_metrics           ( metrics ),
    m_colorCal          ( col_calib ),
    m_currImg           ( cv::Mat(currentImage) ),
    m_kappa             ( 1 ), // This is to try out n + kappa = 3
    m_initialized       ( false ),
    m_current_timestamp ( 0 )
/* What's missing:
   - timestamp initialization (is it 0 or is it somehow offset?)
*/
{
    // convert from cm to px
    m_dist_left  = col_calib->getLeftDist() *metrics->GetScaleFactor();
    m_dist_right = col_calib->getRightDist()*metrics->GetScaleFactor();

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

void ColorTracker::SetCurrentImage(const IplImage *const pImg)
{
    //TODO prev not required??
    m_legacy_img = pImg;
    m_currImg = cv::Mat(m_legacy_img);
    m_currImg.convertTo(m_currImg, CV_32F, 1/255.);

    bool ok = m_colorCal->CorrectColorBalance( &m_currImg );
    if ( !ok  )
    {
        LOG_ERROR("CC - Failed to set new image!");
    }
    cv::cvtColor(m_currImg, m_hsvImg, CV_BGR2HSV);
}

bool ColorTracker::Track(double timeStamp)
{
    double delta_t = timeStamp - m_current_timestamp;
    if (delta_t >= 0)
    {
      m_current_timestamp = timeStamp;
      MVec l_blob, r_blob;
      bool found_l = find_blob(m_hsvImg, m_colorCal->getHueLeft(),
          m_colorCal->getHueThr(), m_colorCal->getMinSat(), &l_blob);
      bool found_r = find_blob(m_hsvImg, m_colorCal->getHueRight(),
          m_colorCal->getHueThr(), m_colorCal->getMinSat(), &r_blob);
      if (m_initialized)
      {
        predict(delta_t);
        if (found_l)
        {
          update(l_blob, -1);
        }
        if (found_r)
        {
          update(r_blob, 1);
        }
      }
      else
      {
        if ( found_l && found_r )
        {
          initialize(l_blob, r_blob);
        }
      }
      if (m_initialized)
      {
        // Log everything
        std::stringstream swriter;
        swriter.precision(20); // Is this precision overkill?
        swriter << m_current_state[3] << " " << m_current_state[4];
        std::string sref = swriter.str();
        m_history.emplace_back( TrackEntry(
          GetPosition(), GetHeading(), GetError(), timeStamp, sref ) );
        return true;
      }
      else
      {
        return false;
      }
    }
    return true;
}

void ColorTracker::initialize (MVec l_blob, MVec r_blob)
{
    MVec pos = 0.5 * ( (1 - m_dist_right + m_dist_left) * r_blob +
                       (1 + m_dist_right - m_dist_left) * l_blob );
    MVec dif = l_blob - r_blob;
    m_pos.x = pos[0];
    m_pos.y = pos[1];
    m_angle = atan2(-dif[0], dif[1]);
    m_angle = m_angle>M_PI?m_angle-2*M_PI:m_angle;
    m_current_state[0] = m_pos.x;
    m_current_state[1] = m_pos.y;
    m_current_state[2] = m_angle;
    m_current_state[3] = 0;
    m_current_state[4] = 0;
    m_current_cov = SCov::zeros();
    m_current_cov(0, 0) = 5;
    m_current_cov(1, 1) = 5;
    m_current_cov(2, 2) = M_PI/10;
    m_current_cov(3, 3) = 5;
    m_current_cov(4, 4) = M_PI/10;
    // Set flag to initialized
    m_initialized = true;
}

void ColorTracker::DoInactiveProcessing(double timeStamp)
{
    // If it is necessary (very low framerates causing very
    // sporadic updates) this can be implemented as a simple
    // call to predict(delta_t) after computing the elated
    // time since the last change to the timestamp
    /* Placeholder function */
    Q_UNUSED(timeStamp);
}

void ColorTracker::Rewind(double timeStamp)
{
    while (!m_history.empty())
    {
        TrackEntry entry = m_history.back();

        if (entry.GetTimeStamp() > timeStamp)
        {
            m_pos = entry.GetPosition();
            m_angle = entry.GetOrientation();
            std::stringstream sreader(*(entry.GetString()));
            sreader.precision(20);
            m_current_state[0] = m_pos.x;
            m_current_state[1] = m_pos.y;
            m_current_state[2] = m_angle;
            sreader >> m_current_state[3];
            sreader >> m_current_state[4];
            m_history.pop_back();
        }
        else
        {
            break;
        }
    }
}

void ColorTracker::SetParam(paramType param, float value)
{
    /* Placeholder function */
    Q_UNUSED(param);
    Q_UNUSED(value);
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
  CV_Assert(input_image.type() == CV_32FC3);

  contours->clear();

  cv::Mat segmented_image = cv::Mat::zeros(
      input_image.rows, input_image.cols, CV_8UC1);
  double distance;
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
      if ( ( m_colorCal->getMinLum() <= luminance &&
             luminance <= m_colorCal->getMaxLum() ) &&
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

/* The input image must be a HSV image */
bool ColorTracker::find_blob(const cv::Mat & input_image, double hue_ref,
    double hue_thr, double sat_thr, MVec * blob)
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
    (*blob)[0] = centroid.x;
    (*blob)[1] = centroid.y;
    return true;
  }
  else
  {
    blob = 0;
    return false;
  }
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
  if (!cholesky(cov_as_mat, decomposed_cov))
  {
    std::cout << "Error - you're not giving me a proper covariance matrix"
                 << std::endl;
    exit(1);
  }
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

void ColorTracker::update(MVec measurement, int direction)
{
  // Direction can be -1 for left or 1 for right
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
      extended_covariance(i, j) = m_meas_noise_cov(i-5, j-5);
    }
  }
  // Obtain vector of sigma points and corresponding weights
  cv::Mat cov_as_mat = (7 + m_kappa) * cv::Mat(extended_covariance);
  cv::Mat decomposed_cov;
  if (!cholesky(cov_as_mat, decomposed_cov))
  {
    std::cout << "Error - you're not giving me a proper covariance matrix"
                 << std::endl;
    exit(1);
  }

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
    if (direction == -1)
    {
      new_point[0] = (*it)[0] - m_dist_left * sin((*it)[2]) + (*it)[5];
      new_point[1] = (*it)[1] + m_dist_left * cos((*it)[2]) + (*it)[6];
    }
    else
    {
      new_point[0] = (*it)[0] + m_dist_right * sin((*it)[2]) + (*it)[5];
      new_point[1] = (*it)[1] - m_dist_right * cos((*it)[2]) + (*it)[6];
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

void testColorTracker(std::string infn, std::string outfn)
{
  std::ifstream inputFile;
  std::ofstream outputFile;
  std::stringstream inLine;
  std::string readLine;
  std::stringstream outLine;
  ColorTracker tracker(NULL, NULL, NULL, NULL);
  // Hack in measurement/process covariances - To calibrate later!
  tracker.m_proc_noise_cov = 2 * SCov::eye();
  tracker.m_proc_noise_cov(0, 0) = 4.;
  tracker.m_proc_noise_cov(1, 1) = 4.;
  tracker.m_proc_noise_cov(2, 2) = M_PI/40.;
  tracker.m_proc_noise_cov(3, 3) = 20.;
  tracker.m_proc_noise_cov(4, 4) = M_PI/80.;
  tracker.m_meas_noise_cov = 2. * MCov::eye();
  tracker.m_dist_left = 10;
  tracker.m_dist_right = 10;
  inLine.precision(15);
  outLine.precision(15);
  inputFile.open(infn);
  outputFile.open(outfn);
  double dt, x_l, y_l, x_r, y_r;
  bool found_l, found_r;
  MVec l_blob, r_blob;
  if (!inputFile.is_open())
  {
    std::cout << "Couldn't open input file" << std::endl;
    exit(1);
  }
  if (!outputFile.is_open())
  {
    std::cout << "Couldn't open output file" << std::endl;
    exit(1);
  }
  while (std::getline(inputFile, readLine))
  {
    inLine.clear();
    inLine.str(readLine);
    inLine >> dt >> found_l >> x_l >> y_l >> found_r >> x_r >> y_r;
    l_blob[0] = x_l; l_blob[1] = y_l;
    r_blob[0] = x_r; r_blob[1] = y_r;
    if (!initialized)
    {
      if (found_l && found_r)
      {
        tracker.initialize(l_blob, r_blob);
        tracker.m_current_cov(0, 0) = 30.;
        tracker.m_current_cov(1, 1) = 30.;
        tracker.m_current_cov(2, 2) = M_PI/4;
        tracker.m_current_cov(3, 3) = 200.;
        tracker.m_current_cov(4, 4) = M_PI/8;
        initialized = true;
      }
    }
    else
    {
      tracker.predict(dt);
      if (found_l)
      {
        tracker.update(l_blob, -1);
      }
      if (found_r)
      {
        tracker.update(r_blob, 1);
      }
    }
    outLine.str("");
    for (int i = 0; i < 5; ++i)
    {
      outLine << tracker.m_current_state[i] << " ";
    }
    for (int i = 0; i < 5; ++i)
    {
      for (int j = 0; j <= i; ++j)
      {
        outLine << tracker.m_current_cov(i, j) << " ";
      }
    }
    outputFile << outLine.str() << std::endl;
  }
  inputFile.close();
  outputFile.close();
}
