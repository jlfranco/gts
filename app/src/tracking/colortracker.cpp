#include "colortracker.h"

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
    m_currImg  ( currentImage )
{
}

/**
 Returns the position of the left edge of the brushbar
 (given that robot has the specified position and heading).
 **/
CvPoint2D32f ColorTracker::GetBrushBarLeft( CvPoint2D32f position, float heading ) const
{
    heading += MathsConstants::F_PI / 2.f;

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
    heading += 3.14159265359f / 2.f;

    float w = GetMetrics()->GetBrushBarWidthPx(); // brush bar width (pixels)

    float px = w / 2.f; // half bbar width
    float py = m_metrics->GetBrushBarOffsetPx(); // offset of brush bar in direction of travel (from centre of robot)

    float cosa = cos( -heading );
    float sina = sin( -heading );

    float x = px * cosa - py * sina;
    float y = px * sina + py * cosa;

    return cvPoint2D32f( position.x + x, position.y + y );
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

void segment_blobs(const cv::Mat & input_image,
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

cv::Point2f find_blob(const cv::Mat & input_image, double hue_ref,
    double hue_thr, double sat_thr)
{
  std::vector<std::vector<cv::Point2i> > contours;
  segment_blobs(input_image, &contours, hue_ref, hue_thr, sat_thr);
  int best_index = largest_polygon(contours);
  cv::Point2f centroid;
  if (!contours.empty())
  {
    std::vector<cv::Point2i> best_polygon = contours[best_index];
    std::vector<cv::Point2i>::iterator it;
    double area;
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
      centroid.x = centroid.x + double((p0.x + p1.x) * (p0.x * p1.y - p0.y * p1.x));
      centroid.y = centroid.y + double((p0.y + p1.y) * (p0.x * p1.y - p0.y * p1.x));
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
