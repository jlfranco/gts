/* Copyright (C) 2013 iRobot Corporation. All rights reserved.
 */

#ifndef COLORTRACKER_H
#define COLORTRACKER_H

#include "RobotTracker.h"

class CameraCalibration;
class ColorCalibration;
class RobotMetrics;

/**
  Alternative tracker to the KLT Optical Flow tracker
  provided in GTS. Works by segmenting two different
  colored blobs in a colored pattern and performing
  filtering on these measurements
  **/

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

    CvPoint2D32f GetBrushBarLeft(CvPoint2D32f position, float heading) const;
    CvPoint2D32f GetBrushBarRight(CvPoint2D32f position, float heading) const;

    float GetError() const
    {
        return m_error;
    }

    const IplImage * GetCurrentImage() const
    {
        return m_currImg;
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
        m_currImg = pImg;
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

    cv::Point2f m_pos;
    float m_angle;

    float m_error; // Keep this?

    IplImage * m_currImg;

    TrackHistory::TrackLog m_history;

    const CameraCalibration * m_cal;
    const RobotMetrics * m_metrics;
    const ColorCalibration * m_colorCal;

};

#endif // COLORTRACKER_H
