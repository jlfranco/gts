#include "ColorCalibration.h"
#include "CalibrationSchema.h"
#include <algorithm> // std::max

#include "Logging.h"

#include <opencv2/highgui/highgui.hpp>

ColorCalibration::ColorCalibration()
{
}

ColorCalibration::~ColorCalibration()
{
}

bool ColorCalibration::HexStrToRgbScaled ( const QString& hexRgbStr, float* r, float* g, float* b )
{
    if ( hexRgbStr.length() != 6 )
    {
        /* Expects "RRGGBB" string */
        return false;
    }
    QString red   = hexRgbStr.mid(0,2); /* RR */
    QString green = hexRgbStr.mid(2,2); /* GG */
    QString blue  = hexRgbStr.mid(4,2); /* BB */

    /* convert from hex str to int */
    bool convOk;
    int rInt, gInt, bInt;
    rInt = red.toInt  ( &convOk, 16 ); if ( !convOk ) return false;
    gInt = green.toInt( &convOk, 16 ); if ( !convOk ) return false;
    bInt = blue.toInt ( &convOk, 16 ); if ( !convOk ) return false;

    /* scale */
    *r = ((float)rInt)/255.0;
    *g = ((float)gInt)/255.0;
    *b = ((float)bInt)/255.0;
    return true;
}

bool ColorCalibration::HexStrRgbToHsv( const QString& hexRgbStr, float* h, float* s, float* v)
{
    float rF, gF, bF;
    LOG_TRACE(QObject::tr("CC: Converting HEX string [%1] to RGB").arg(hexRgbStr));
    if ( ! HexStrToRgbScaled( hexRgbStr, &rF, &gF, &bF ) ) return false;

    /*
     * use opencv RGB to HSV
     * http://docs.opencv.org/modules/imgproc/doc/miscellaneous_transformations.html?highlight=cvtcolor#cv.CvtColor
     */
    LOG_TRACE("CC: Converting RGB string to HSV");
    float hTmp, sTmp, vTmp;
    vTmp = std::max(std::max(rF,gF),bF);
    sTmp = ( vTmp==0 )?0:( vTmp - std::min( std::min( rF,gF ),bF ) )/vTmp;
    if ( vTmp == rF )
    {
        hTmp = 60.0*(gF - bF)/( vTmp - std::min( std::min( rF,gF),bF ) );
    }
    else
    {
        if ( vTmp == gF )
        {
            hTmp = 120.0 + 60.0*(bF - rF)/( vTmp - std::min( std::min( rF,gF ),bF ) );
        }
        else
        {
            if ( vTmp == bF )
            {
                hTmp = 240.0 + 60.0*( rF - gF )/( vTmp - std::min( std::min( rF,gF ),bF ) );
            }
            else
            {
                /* this should not happen */
                LOG_ERROR("CC: HSV fail.");
                return false;
            }
        }
    }

    /* 8 bit images */
    *h = hTmp;
    *s = sTmp;
    *v = vTmp;

    LOG_TRACE(QObject::tr("CC: Validating RGB->HSV conversion results"));
    if ( *h > 360 || *h < 0 )
    {
        LOG_ERROR(QObject::tr("CC: Invalid HSV H value: %1").arg(*h));;
        return false;
    }
    if ( *s > 1   || * s < 0 )
    {
        LOG_ERROR(QObject::tr("CC: Invalid HSV S value. %1").arg(*s));
        return false;
    }
    if ( *v > 1   || * v < 0 )
    {
        LOG_ERROR(QObject::tr("CC: Invalid HSV V value. %1").arg(*v));
        return false;
    }

    return true;
}

bool ColorCalibration::Load( const WbConfig& config)
{
    QString colorStr;
    float h, s,v;

    /* Left */
    colorStr = config.GetKeyValue( CalibrationSchema::hueLeftKey ).ToQString();
    if ( !HexStrRgbToHsv( colorStr, &h, &s, &v ) ) return false;
    setHueLeft(h);
    setLeftDist ( config.GetKeyValue( CalibrationSchema::distLeftKey  ).ToDouble() );

    /* Right */
    colorStr = config.GetKeyValue( CalibrationSchema::hueRightKey ).ToQString();
    if ( !HexStrRgbToHsv( colorStr, &h, &s, &v ) ) return false;
    setHueRight(h);
    setRightDist( config.GetKeyValue( CalibrationSchema::distRightKey ).ToDouble() );

    /* Gray levels */
    float gr, gg, gb;
    colorStr = config.GetKeyValue( CalibrationSchema::hueGrayKey ).ToQString();
    if ( !HexStrToRgbScaled( colorStr, &gr, &gg, &gb ) ) return false;
    setGrayR(gr); setGrayG(gg); setGrayB(gb);
    setGrayL( (float) config.GetKeyValue( CalibrationSchema::grayPercentageKey ).ToDouble() );

    /* Luminance */
    setMinLum( (float) config.GetKeyValue( CalibrationSchema::luminanceMinKey ).ToDouble() );
    setMaxLum( (float) config.GetKeyValue( CalibrationSchema::luminanceMaxKey ).ToDouble() );

    /* Thresholds */
    setHueThr( (float) config.GetKeyValue( CalibrationSchema::hueThresholdKey ).ToDouble() );
    setMinSat( (float) config.GetKeyValue( CalibrationSchema::saturationMinKey ).ToDouble() );

    /* method */
    setMethod( config.GetKeyValue( CalibrationSchema::methodKey ).ToBool() );

    return true;
}

cv::Mat ColorCalibration::QtToCv( const QImage& im ) const
{
    // http://stackoverflow.com/questions/17127762/cvmat-to-qimage-and-back#answer-17137998
    cv::Mat tmp( im.height(), im.width(), CV_8UC3, (uchar*)im.bits(), im.bytesPerLine() );
    cv::Mat imMat;
    cvtColor( tmp, imMat, CV_BGR2RGB );
    return imMat;
}

QImage ColorCalibration::CvToQt( const cv::Mat& mat ) const
{
    cv::Mat matData;
    cv::cvtColor( mat, matData, CV_BGR2RGB );
    assert( matData.isContinuous() );
    QImage im = QImage( matData.data, matData.cols, matData.rows,
                        3*matData.cols, QImage::Format_RGB888);
    return im;
}

bool ColorCalibration::Test( const WbConfig& config, const QImage& input, QImage* output )
{
    if ( !Load( config ) )
    {
        LOG_ERROR(QObject::tr("Failed to load color calibration config!"));
        return false;
    }

    using namespace cv;
    cv::Mat imMat = QtToCv(input);
    if ( !imMat.data )
    {
        LOG_ERROR( QObject::tr( "Failed to convert image!" ) );
        return false;
    }

    if ( m_method )
    {
        AutoCalibrate( &imMat );
    }
    CorrectColorBalance( &imMat );

    *output = CvToQt( imMat );
    return true;
}

void ColorCalibration::CorrectColorBalance(cv::Mat * inputImage) const
{
  double cc_r, cc_g, cc_b;
  if (m_gray_l == m_gray_r)
  {
    cc_r = 1.;
  }
  else
  {
    cc_r = (m_gray_l - pow(m_gray_r, 2)) /
           (m_gray_r - pow(m_gray_r, 2)) ;
  }
  if (m_gray_l == m_gray_g)
  {
    cc_g = 1.;
  }
  else
  {
    cc_g = (m_gray_l - pow(m_gray_g, 2)) /
           (m_gray_g - pow(m_gray_g, 2)) ;
  }
  if (m_gray_l == m_gray_b)
  {
    cc_b = 1.;
  }
  else
  {
    cc_b = (m_gray_l - pow(m_gray_b, 2)) /
           (m_gray_b - pow(m_gray_b, 2)) ;
  }
  switch (inputImage->depth())
  {
    case CV_8U:
    {
      cv::MatIterator_<cv::Vec3b> it;
      double b, g, r;
      for (it = inputImage -> begin<cv::Vec3b>();
           it != inputImage -> end<cv::Vec3b>(); ++it)
      {

        b = cc_b * double((*it)[0]) / 255. +
            (1 - cc_b) * pow( double((*it)[0]) / 255., 2 );
        g = cc_g * double((*it)[1]) / 255. +
            (1 - cc_g) * pow( double((*it)[1]) / 255., 2 );
        r = cc_r * double((*it)[2]) / 255. +
            (1 - cc_r) * pow( double((*it)[2]) / 255., 2 );
        (*it)[0] = uchar( 255 * b );
        (*it)[1] = uchar( 255 * g );
        (*it)[2] = uchar( 255 * r );
      }
      break;
    }
    case CV_32F:
    {
      cv::MatIterator_<cv::Vec3f> it;
      for (it = inputImage -> begin<cv::Vec3f>();
           it != inputImage -> end<cv::Vec3f>(); ++it)
      {
        (*it)[0] = cc_b * (*it)[0] + (1 - cc_b) * pow((*it)[0], 2);
        (*it)[1] = cc_g * (*it)[1] + (1 - cc_g) * pow((*it)[1], 2);
        (*it)[2] = cc_r * (*it)[2] + (1 - cc_r) * pow((*it)[2], 2);
      }
      break;
    }
    case CV_64F:
    {
      cv::MatIterator_<cv::Vec3d> it;
      for (it = inputImage -> begin<cv::Vec3d>();
           it != inputImage -> end<cv::Vec3d>(); ++it)
      {
        (*it)[0] = cc_b * (*it)[0] + (1 - cc_b) * pow((*it)[0], 2);
        (*it)[1] = cc_g * (*it)[1] + (1 - cc_g) * pow((*it)[1], 2);
        (*it)[2] = cc_r * (*it)[2] + (1 - cc_r) * pow((*it)[2], 2);
      }
      break;
    }
  }
}

void ColorCalibration::AutoCalibrate(QImage& im)
{
    cv::Mat imMat = QtToCv( im );
    AutoCalibrate( &imMat );
    im = CvToQt( imMat );
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
      avg_b /= 255;
      avg_g /= 255;
      avg_r /= 255;
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
