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
    m_cal               ( cam_calib ),
    m_metrics           ( metrics ),
    m_colorCal          ( col_calib ),
    m_currImg           ( cv::Mat(currentImage) ),
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

    m_colorCal->CorrectColorBalance( &m_currImg );
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
      if ( ukf.isInitialized() )
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
          ukf.initialize( l_blob, r_blob, m_dist_left, m_dist_right );
        }
      }
      if ( ukf.isInitialized() )
      {
        // Log everything
        SVec currentState = ukf.getCurrentState();
        std::stringstream swriter;
        swriter.precision(20); // Is this precision overkill?
        swriter << currentState[3] << " " << currentState[4];
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
    ukf.initialize( l_blob, r_blob, m_dist_left, m_dist_right );
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
            std::stringstream sreader(*(entry.GetString()));
            sreader.precision(20);

            double currState3;
            sreader >> currState3;
            double currState4;
            sreader >> currState4;

            MVec pos;
            CvPoint2D32f posCv = entry.GetPosition();
            pos[0] = posCv.x;
            pos[1] = posCv.y;

            ukf.setCurrentState( pos,
                                 entry.GetOrientation(),
                                 currState3,
                                 currState4 );
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

void ColorTracker::predict( double delta_t )
{
  ukf.predict( delta_t );
}

void ColorTracker::update( MVec measurement, int model )
{
  ukf.update( measurement, model, model==-1?m_dist_left:m_dist_right );
}

// void testColorTracker(std::string infn, std::string outfn)
// {
//   std::ifstream inputFile;
//   std::ofstream outputFile;
//   std::stringstream inLine;
//   std::string readLine;
//   std::stringstream outLine;
//   ColorTracker tracker(NULL, NULL, NULL, NULL);
//   // Hack in measurement/process covariances - To calibrate later!
//   tracker.m_proc_noise_cov = 2 * SCov::eye();
//   tracker.m_proc_noise_cov(0, 0) = 4.;
//   tracker.m_proc_noise_cov(1, 1) = 4.;
//   tracker.m_proc_noise_cov(2, 2) = M_PI/40.;
//   tracker.m_proc_noise_cov(3, 3) = 20.;
//   tracker.m_proc_noise_cov(4, 4) = M_PI/80.;
//   tracker.m_meas_noise_cov = 2. * MCov::eye();
//   tracker.m_dist_left = 10;
//   tracker.m_dist_right = 10;
//   inLine.precision(15);
//   outLine.precision(15);
//   inputFile.open(infn);
//   outputFile.open(outfn);
//   double dt, x_l, y_l, x_r, y_r;
//   bool found_l, found_r, initialized;
//   MVec l_blob, r_blob;
//   if (!inputFile.is_open())
//   {
//     std::cout << "Couldn't open input file" << std::endl;
//     exit(1);
//   }
//   if (!outputFile.is_open())
//   {
//     std::cout << "Couldn't open output file" << std::endl;
//     exit(1);
//   }
//   while (std::getline(inputFile, readLine))
//   {
//     inLine.clear();
//     inLine.str(readLine);
//     inLine >> dt >> found_l >> x_l >> y_l >> found_r >> x_r >> y_r;
//     l_blob[0] = x_l; l_blob[1] = y_l;
//     r_blob[0] = x_r; r_blob[1] = y_r;
//     if (!initialized)
//     {
//       if (found_l && found_r)
//       {
//         tracker.initialize(l_blob, r_blob);
//         tracker.m_current_cov(0, 0) = 30.;
//         tracker.m_current_cov(1, 1) = 30.;
//         tracker.m_current_cov(2, 2) = M_PI/4;
//         tracker.m_current_cov(3, 3) = 200.;
//         tracker.m_current_cov(4, 4) = M_PI/8;
//         initialized = true;
//       }
//     }
//     else
//     {
//       tracker.predict(dt);
//       if (found_l)
//       {
//         tracker.update(l_blob, -1);
//       }
//       if (found_r)
//       {
//         tracker.update(r_blob, 1);
//       }
//     }
//     outLine.str("");
//     for (int i = 0; i < 5; ++i)
//     {
//       outLine << tracker.m_current_state[i] << " ";
//     }
//     for (int i = 0; i < 5; ++i)
//     {
//       for (int j = 0; j <= i; ++j)
//       {
//         outLine << tracker.m_current_cov(i, j) << " ";
//       }
//     }
//     outputFile << outLine.str() << std::endl;
//   }
//   inputFile.close();
//   outputFile.close();
// }
