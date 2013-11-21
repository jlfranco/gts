/* Copyright (C) 2013 iRobot Corporation. All rights reserved.
 */

#ifndef COLORTRACKER_H
#define COLORTRACKER_H

#include "RobotTracker.h"
#include "Ukf.h"

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
void testColorTracker(std::string inputFilename, std::string outputFilename);

class ColorTracker : public RobotTracker
{
  friend void testColorTracker(std::string, std::string);
public:
    ColorTracker( const CameraCalibration * cam_calib,
                  const RobotMetrics * metrics,
                  const ColorCalibration * col_calib,
                  const IplImage * currentImage );
    ~ColorTracker();

    CvPoint2D32f GetPosition() const
    {
        return ukf.getPosition();
    }

    CvPoint2D32f GetGroundPlanePos() const
    {
        return ConvertTrackToCm(
                   AdjustTrackForRobotHeight( GetPosition(), GetHeading() ) );
    }

    float GetHeading() const
    {
        return ukf.getHeading();
    }

    double GetCurrentTime() const
    {
        return m_current_timestamp;
    }

    CvPoint2D32f GetBrushBarLeft(CvPoint2D32f position, float heading) const;
    CvPoint2D32f GetBrushBarRight(CvPoint2D32f position, float heading) const;

    float GetError() const
    {
        return ukf.getError();
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
        ukf.setPosition( robotPosition );
    }

    bool UsesColorImages()
    {
        return true;
    }

    void SetCurrentImage( const IplImage *const pImg );

    bool Track( double timeStamp );

    void initialize(MVec l_blob, MVec r_blob);

    void DoInactiveProcessing( double timeStamp );

    // LossRecovery???

    void Rewind( double timeStamp );

    // LoadTargetImage is not required since the tracking method
    // is target free
    bool LoadTargetImage(const char * targetFilename) {(void)targetFilename;return true;}

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

    bool find_blob(const cv::Mat & input_image, double hue_ref,
        double hue_thr, double sat_thr, MVec * blob);

    // Uses process model to predict the robot's next position
    void predict(double delta_t);
    // Uses one of two measurement models (one for left measurement,
    // one for right measurement) to correct the predicted position.
    void update(MVec measurement, int model);

    bool m_initialized;

    double m_current_timestamp;

    double m_dist_left;  // [px]
    double m_dist_right; // [px]

    Ukf ukf;

    cv::Mat m_currImg;
    cv::Mat m_hsvImg;

    const IplImage * m_legacy_img;

    TrackHistory::TrackLog m_history;

    const CameraCalibration * m_cal;
    const RobotMetrics * m_metrics;
    const ColorCalibration * m_colorCal;

};

#endif // COLORTRACKER_H
