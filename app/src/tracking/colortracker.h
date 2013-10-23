/* Copyright (C) 2013 iRobot Corporation. All rights reserved.
 */

#ifndef COLORTRACKER_H
#define COLORTRACKER_H

#include "RobotTracker.h"

typedef cv::Vec<double, 5> SVec;
typedef cv::Vec<double, 2> MVec;
typedef cv::Vec<double, 10> XVec10;
typedef cv::Vec<double, 7> XVec7;
typedef cv::Matx<double, 5, 5> SCov;
typedef cv::Matx<double, 2, 2> MCov;
typedef cv::Matx<double, 7, 2> CCov;
typedef cv::Matx<double, 10, 10> XCov10;
typedef cv::Matx<double, 7, 7> XCov7;

class CameraCalibration;
class ColorCalibration;
class RobotMetrics;

/**
  Alternative tracker to the KLT Optical Flow tracker
  provided in GTS. Works by segmenting two different
  colored blobs in a colored pattern and performing
  filtering on these measurements
  **/

/* Utility function that decomposes a positive definite matrix A
   into its so called matrix square root L so that L L' = A.
   Returns false if matrix is not positive definite*/

bool cholesky (cv::Mat & mat, cv::Mat & output);

class ColorTracker : public RobotTracker
{
public:
    ColorTracker( const CameraCalibration * cam_calib,
                  const RobotMetrics * metrics,
                  const ColorCalibration * col_calib,
                  const IplImage * currentImage );
    ~ColorTracker();

    CvPoint2D32f GetPosition() const
    {
        return (CvPoint2D32f) m_pos;
    }

    CvPoint2D32f GetGroundPlanePos() const
    {
        return ConvertTrackToCm(
                   AdjustTrackForRobotHeight( GetPosition(), GetHeading() ) );
    }

    float GetHeading() const
    {
        return m_angle;
    }

    double GetCurrentTime() const
    {
        return m_current_timestamp;
    }

    CvPoint2D32f GetBrushBarLeft(CvPoint2D32f position, float heading) const;
    CvPoint2D32f GetBrushBarRight(CvPoint2D32f position, float heading) const;

    float GetError() const
    {
        return m_error;
    }

    const IplImage * GetCurrentImage() const
    {
        return m_legacy_img;
    }

    // Any parameters that need to be set here?
    void SetParam( paramType param, float value );

    const TrackHistory::TrackLog& GetHistory() const
    {
        return m_history;
    }

    void SetPosition( CvPoint2D32f robotPosition )
    {
        m_pos = robotPosition;
    }

    void SetCurrentImage( const IplImage *const pImg )
    {
        m_legacy_img = pImg;
        m_currImg = cv::Mat(m_legacy_img);
    }

    void Activate();

    bool Track( double timeStamp );

    void DoInactiveProcessing( double timeStamp );

    // LossRecovery???

    void Rewind( double timeStamp );

    // LoadTargetImage???

    const CameraCalibration * GetCalibration() const
    {
        return m_cal;
    }

    const RobotMetrics * GetMetrics() const
    {
        return m_metrics;
    }

    const ColorCalibration * GetColorCalib() const
    {
        return m_colorCal;
    }

private:

    int largest_polygon(std::vector<std::vector<cv::Point2i> > & polygons);

    void segment_blobs(const cv::Mat & input_image,
        std::vector<std::vector<cv::Point2i> > * contours, double hue_ref,
        double hue_thr, double sat_thr);

    cv::Point2f find_blob(const cv::Mat & input_image, double hue_ref,
        double hue_thr, double sat_thr);

    // Uses process model to predict the robot's next position
    void predict(double delta_t);
    // Uses one of two measurement models (one for left measurement,
    // one for right measurement) to correct the predicted position.
    void update(MVec measurement, int direction);

    cv::Point2f m_pos;
    float m_angle;

    float m_error; // Keep this?

    double m_current_timestamp;

    double m_dist_left;
    double m_dist_right;

    SVec m_current_state;
    SCov m_current_cov;
    SCov m_proc_noise_cov;
    MCov m_meas_noise_cov;
    double m_kappa; // Used in the unscented transform

    cv::Mat m_currImg;
    const IplImage * m_legacy_img;

    TrackHistory::TrackLog m_history;

    const CameraCalibration * m_cal;
    const RobotMetrics * m_metrics;
    const ColorCalibration * m_colorCal;

};

#endif // COLORTRACKER_H
