#include "ColorCalibration.h"

ColorCalibration::ColorCalibration()
{
}

ColorCalibration::~ColorCalibration()
{
}

void ColorCalibration::CorrectColorBalance(cv::Mat * inputImage)
{
  switch (inputImage->depth())
  {
    case CV_8U:
    {
      cv::MatIterator_<cv::Vec3b> it;
      for (it = inputImage -> begin<cv::Vec3b>();
           it != inputImage -> end<cv::Vec3b>(); ++it)
      {
        int b = (0.5 + int( (*it)[0] ) * 255 * m_gray_l) / m_gray_b;
        int g = (0.5 + int( (*it)[1] ) * 255 * m_gray_l) / m_gray_g;
        int r = (0.5 + int( (*it)[2] ) * 255 * m_gray_l) / m_gray_r;
        (*it)[0] = uchar( b );
        (*it)[1] = uchar( g );
        (*it)[2] = uchar( r );
      }
      break;
    }
    case CV_32F:
    {
      cv::MatIterator_<cv::Vec3f> it;
      for (it = inputImage -> begin<cv::Vec3f>();
           it != inputImage -> end<cv::Vec3f>(); ++it)
      {
        (*it)[0] *= m_gray_l / m_gray_b;
        (*it)[1] *= m_gray_l / m_gray_g;
        (*it)[2] *= m_gray_l / m_gray_r;
      }
      break;
    }
    case CV_64F:
    {
      cv::MatIterator_<cv::Vec3d> it;
      for (it = inputImage -> begin<cv::Vec3d>();
           it != inputImage -> end<cv::Vec3d>(); ++it)
      {
        (*it)[0] *= m_gray_l / m_gray_b;
        (*it)[1] *= m_gray_l / m_gray_g;
        (*it)[2] *= m_gray_l / m_gray_r;
      }
      break;
    }
  }
}

void ColorCalibration::AutoCalibrate(cv::Mat *sampleImage)
{
  float avg_r = 0;
  float avg_g = 0;
  float avg_b = 0;
  float factor = 1.f/( (sampleImage->rows) * (sampleImage->cols) );
  switch (sampleImage->depth())
  {
    case CV_8U:
    {
      cv::MatIterator_<cv::Vec3b> it;
      for (it = sampleImage -> begin<cv::Vec3b>();
           it != sampleImage -> end<cv::Vec3b>(); ++it)
      {
        avg_b += factor * (*it)[0];
        avg_g += factor * (*it)[1];
        avg_r += factor * (*it)[2];
      }
      break;
    }
    case CV_32F:
    {
      cv::MatIterator_<cv::Vec3f> it;
      for (it = sampleImage -> begin<cv::Vec3f>();
           it != sampleImage -> end<cv::Vec3f>(); ++it)
      {
        avg_b += factor * (*it)[0];
        avg_g += factor * (*it)[1];
        avg_r += factor * (*it)[2];
      }
      break;
    }
    case CV_64F:
    {
      cv::MatIterator_<cv::Vec3d> it;
      for (it = sampleImage -> begin<cv::Vec3d>();
           it != sampleImage -> end<cv::Vec3d>(); ++it)
      {
        avg_b += factor * (*it)[0];
        avg_g += factor * (*it)[1];
        avg_r += factor * (*it)[2];
      }
      break;
    }
  }
  setGrayR(avg_r);
  setGrayG(avg_g);
  setGrayB(avg_b);
}
