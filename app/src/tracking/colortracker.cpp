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
CvPoint2D32f KltTracker::GetBrushBarLeft( CvPoint2D32f position, float heading ) const
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
CvPoint2D32f KltTracker::GetBrushBarRight( CvPoint2D32f position, float heading ) const
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
