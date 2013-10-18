#ifndef COLORCALIBRATION_H
#define COLORCALIBRATION_H

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
    float getHueLeft() const { return m_hueLeft; }
    float getHueRight() const { return m_hueRight; }
    float getMinLum() const { return m_minLum; }
    float getMaxLum() const { return m_maxLum; }
    float getMinSat() const { return m_minSat; }
    float getGrayR() const { return m_gray_r; }
    float getGrayG() const { return m_gray_g; }
    float getGrayB() const { return m_gray_b; }
    void setHueLeft ( const float val ) { m_hueLeft = val; }
    void setHueRight ( const float val ) { m_hueRight = val; }
    void setMinLum ( const float val ) { m_minLum = val; }
    void setMaxLum ( const float val ) { m_maxLum = val; }
    void setMinSat ( const float val ) { m_minSat = val; }
    void setGrayR ( const float val ) { m_gray_r = val; }
    void setGrayG ( const float val ) { m_gray_g = val; }
    void setGrayB ( const float val ) { m_gray_b = val; }

    protected:
    float m_hueLeft;
    float m_hueRight;
    float m_minLum;
    float m_maxLum;
    float m_minSat;
    float m_gray_r;
    float m_gray_g;
    float m_gray_b;
};

#endif // COLORCALIBRATION_H
