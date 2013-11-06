#ifndef COLORCALIBRATION_H
#define COLORCALIBRATION_H

#include "WbConfig.h"

#include <opencv2/core/core.hpp>


class ColorCalibration
{
    public:
    ColorCalibration();
    ~ColorCalibration();
    /* TODO: Implement functions to load/save from/to WbConfig files
       (c.f. CameraCalibration, RobotMetrics) */
    // Methods
    // Corrects color balance in place
    void CorrectColorBalance(cv::Mat * inputImage); /*TODO*/
    // Automatically determines gray levels using gray world assumption
    void AutoCalibrate(cv::Mat * sampleImage); /*TODO*/
    // Accessors
    float  getHueLeft()   const { return m_hueLeft; }
    float  getHueRight()  const { return m_hueRight; }
    float  getHueThr()    const { return m_hueThr; }
    float  getMinLum()    const { return m_minLum; }
    float  getMaxLum()    const { return m_maxLum; }
    float  getMinSat()    const { return m_minSat; }
    float  getGrayR()     const { return m_gray_r; }
    float  getGrayG()     const { return m_gray_g; }
    float  getGrayB()     const { return m_gray_b; }
    double getLeftDist()  const { return m_dist_l; }
    double getRightDist() const { return m_dist_r; }
    double getMethod()    const { return m_method; }
    void   setHueLeft   ( const float val )  { m_hueLeft = val; }
    void   setHueRight  ( const float val )  { m_hueRight = val; }
    void   setHueThr    ( const float val )  { m_hueThr = val; }
    void   setMinLum    ( const float val )  { m_minLum = val; }
    void   setMaxLum    ( const float val )  { m_maxLum = val; }
    void   setMinSat    ( const float val )  { m_minSat = val; }
    void   setGrayR     ( const float val )  { m_gray_r = val; }
    void   setGrayG     ( const float val )  { m_gray_g = val; }
    void   setGrayB     ( const float val )  { m_gray_b = val; }
    void   setGrayL     ( const float val )  { m_gray_l = val; }
    void   setLeftDist  ( const double val ) { m_dist_l = val; }
    void   setRightDist ( const double val ) { m_dist_r = val; }
    void   setMethod    ( const bool val  )  { m_method = val; }

    bool   Run( const WbConfig& config);

    protected:
    /* Selected hues for left and right markers */
    float m_hueLeft;
    float m_hueRight;
    /* Threshold for hue segmentation */
    float m_hueThr;
    /* Thresholds for saturation and luminance to accept a given color */
    float m_minLum;
    float m_maxLum;
    float m_minSat;
    /* Measured gray values measured from the observed scene and true gray
       level */
    float m_gray_r;
    float m_gray_g;
    float m_gray_b;
    float m_gray_l;
    /* Distance from robot center to the centers of the left and right markers.
       These three points are assumed to lay in the same line  */
    double m_dist_l;
    double m_dist_r;
    /* If true them use user provided gray levels, otherwise will determine
       gray levels automatically. */
    bool m_method;

    QImage im;

    bool   HexStrRgbToHsv( const QString& hexRgbStr, float* h, float* s, float* v );
    bool   HexStrToRgbScaled( const QString& hexRgbStr, float* r, float* g, float* b );

    bool   Load( const WbConfig& config);
};

#endif // COLORCALIBRATION_H
