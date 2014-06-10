/*
 * Copyright (C) 2007-2013 Dyson Technology Ltd, all rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "GtsView.h"

#include "CoverageSystem.h"
#include "RobotMetrics.h"
#include "CameraCalibration.h"
#include "KltTracker.h"
#include "GroundTruthUI.h"
#include "MathsConstants.h"

#include "ImageView.h"
#include "ImageGrid.h"

#include "VideoCaptureCv.h"
#include "FileCapture.h"

#include "CalibrationSchema.h"
#include "ExtrinsicCalibrationSchema.h"

#include "TrackRobotWidget.h"

#include "Logging.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <QTextStream>
#include <QtCore/QTime>
#include <QtCore/QThread>
#include <QtCore/QMutex>
#include <QtGui/QApplication>
#include <QtGui/QPainter>
#include <QtGui/QPen>
#include <QtGlobal>
#include <QObject>

#include <sstream>

#include <math.h>
#include <stdio.h>

#ifdef _WIN32
#include "windows.h"
#include <direct.h>
#else
#include <sys/stat.h>
#endif

GtsView::GtsView() :
    m_id          ( -1 ),
    m_fps         ( 0.0 ),
    m_calScaled   ( 0 ),
    m_calNormal   ( 0 ),
    m_tracker     ( 0 ),
    m_sequencer   ( 0 ),
    m_imgFrame    ( 0 ),
    m_imgGrey     ( 0 ),
    m_thumbnail   ( 0 ),
    m_metrics     ( 0 ),
    m_imgIndex    ( 0 )
{
    m_imgWarp[0] = 0;
    m_imgWarp[1] = 0;
}

/**
    Releases resources
**/
GtsView::~GtsView()
{
}

void GtsView::Reset()
{
    cvReleaseImage( &m_imgWarp[0] );
    cvReleaseImage( &m_imgWarp[1] );
    cvReleaseImage( &m_imgFrame );
    cvReleaseImage( &m_thumbnail );

    delete m_tracker;
    delete m_calScaled;
    delete m_calNormal;
    delete m_sequencer;
    delete m_metrics;

    m_id = -1;
}

/**
    Give a view its own id.
    Also constructs an internal name string which is based on the id.
**/
void GtsView::SetId( int id )
{
    m_id = id;

    char num[8];
    sprintf( num, "%d", id );
    m_name = std::string("camera_") + num;
}

/**
    Save a scaled down version of the tracked image
    (includes frame number and camera id in the file name).
**/
void GtsView::SaveThumbnail()
{
    if ( m_sequencer )
    {
        unsigned int index = (unsigned int)m_sequencer->GetFrameIndex();

        if (index%15==1)
        {
            if ( m_thumbnail == 0 )
            {
                m_thumbnail = cvCreateImage( cvSize( m_imgFrame->width/3,
                                                     m_imgFrame->height/3 ),
                                             m_imgFrame->depth,
                                             m_imgFrame->nChannels );
#ifdef _WIN32
                if ( _mkdir( "thumbnails") )
#else
                if ( mkdir( "thumbnails", 0755) )
#endif
                {
                    LOG_WARN("Could not create a directory for thumbnails!");
                }
            }

            cvResize( m_imgFrame, m_thumbnail, CV_INTER_NN );
            char filename[] = "#######################################thumbnail.png";
            sprintf( filename, "thumbnails/%.6d_%s_thumbnail.png", index, m_name.c_str() );
            cvSaveImage( filename, m_thumbnail );
        }
    }
}

/**
 Allocate a RobotMetrics object and read data into it.
 **/
bool GtsView::LoadMetrics( const WbConfig& metricsConfig,
                           const WbConfig& camPosCalConfig,
                           float trackingResolution )
{
    m_metrics = new RobotMetrics();

    return m_metrics->LoadMetrics( metricsConfig, camPosCalConfig, trackingResolution );
}

/**
    Pass in the names for the calibration-image and calibration-data files.
    GtsView will then load/compute the calibration data.
**/
bool GtsView::SetupCalibration( const KeyId     camPosId,
                                const WbConfig& cameraConfig,
                                const WbConfig& camPosConfig,
                                const WbConfig& floorPlanConfig,
                                const WbConfig& camPosCalConfig,
                                RobotMetrics&   metrics )
{
    // first load calibration config
    const WbConfig cameraIntrisicConfig( cameraConfig.GetSubConfig( CalibrationSchema::schemaName ) );
    const WbConfig cameraExtrisicConfig( camPosConfig.GetSubConfig( ExtrinsicCalibrationSchema::schemaName ));

    const KeyValue fileNameKeyValue( cameraExtrisicConfig.GetKeyValue( ExtrinsicCalibrationSchema::calibrationImageKey ) );
    const QString fileName( fileNameKeyValue.ToQString() );
    QString calibImageFileName = cameraExtrisicConfig.GetAbsoluteFileNameFor( fileName );

    m_calScaled = new CameraCalibration();
    m_calNormal = new CameraCalibration();

    if ( !m_calScaled->LoadIntrinsicCalibration( cameraIntrisicConfig ) ||
         !m_calNormal->LoadIntrinsicCalibration( cameraIntrisicConfig ) )
    {
        LOG_ERROR("Load intrinsic calibration failed!");

        m_id = -1;
        return false;
    }

    CvSize m_boardsize = cvSize( camPosCalConfig.GetKeyValue(ExtrinsicCalibrationSchema::gridColumnsKey).ToInt(),
                                 camPosCalConfig.GetKeyValue(ExtrinsicCalibrationSchema::gridRowsKey).ToInt() );

    LOG_INFO(QObject::tr("Board size: %1,%2.").arg(m_boardsize.width)
                                              .arg(m_boardsize.height));

    if ( !m_calScaled->PerformExtrinsicCalibration( m_boardsize,
                                                    metrics,
                                                    &m_imgWarp[m_imgIndex],
                                                    true,
                                                    calibImageFileName.toAscii().data() ) ||
         !m_calNormal->PerformExtrinsicCalibration( m_boardsize,
                                                    metrics,
                                                    &m_imgWarp_[m_imgIndex],
                                                    false,
                                                    calibImageFileName.toAscii().data() ) )
    {
        LOG_ERROR("Extrinsic calibration failed!");

        m_id = -1;
        return false;
    }

    if ( !m_calScaled->LoadCameraTransform( camPosId, floorPlanConfig ) )
    {
        LOG_ERROR("Load camera transform failed!");

        m_id = -1;
        return false;
    }

    m_imgGrey = cvCreateImage( m_calScaled->GetImageSize(), IPL_DEPTH_8U, 1 );

    m_imgWarp[1-m_imgIndex] = cvCloneImage( m_imgWarp[m_imgIndex] );

    return true;
}

/**
    Initialise the tracking algorithm.
**/
bool GtsView::SetupTracker( RobotTracker::trackerType type,
                            const RobotMetrics&       metrics,
                            const char*               targetFile,
                            int                       biLevelThreshold )
{
    bool success = false;

    switch ( type )
    {
        case RobotTracker::KLT_TRACKER:
            LOG_TRACE("Creating target tracker");
            m_tracker = new KltTracker( m_calScaled,
                                        &metrics,
                                        m_imgWarp[m_imgIndex],
                                        biLevelThreshold );
            success = m_tracker->LoadTargetImage( targetFile );
            break;

        default:
            assert( 0 );
            break;
    }

    return success;
}

/**
    Initialise the video source so that it is ready to supply frames.
**/
bool GtsView::SetupVideo( const char* const videoFile,
                          const char* const timestampFile,
			  int id_cam,
                          float shutter,
                          float gain )
{
    Q_UNUSED(shutter);
    Q_UNUSED(gain);

    std::string f( videoFile );
    const std::string openCvCameraPrefix("opencv-camera:");

    if (f == openCvCameraPrefix)
    {
	id_cam = 1;
	std::cout<<"Openning camera with id =" << id_cam << std::endl;
	m_sequencer = new VideoCaptureCv(id_cam);

        if (!m_sequencer->IsSetup())
        {
            LOG_ERROR("Could not locate OpenCV camera!");

            m_id = -1;
            return false;
        }
    }
    else if ( f.find(".txt") != std::string::npos )
    {
        LOG_INFO(QObject::tr("Loading image list from: %1.")
                    .arg(videoFile));

        m_sequencer = new FileCapture( videoFile );

        if ( !m_sequencer->IsSetup() )
        {
            LOG_ERROR("Could not load from file!");

            m_id = -1;
            return false;
        }
    }
    else
    {
        LOG_INFO(QObject::tr("Loading video from file: %1.")
                    .arg(videoFile));

        m_sequencer = new VideoCaptureCv( videoFile );

        if ( !( m_sequencer && m_sequencer->IsSetup() ) )
        {
            LOG_ERROR("Could not load from file!");

            m_id = -1;
            return false;
        }
    }

    m_fps = m_sequencer->GetFrameRate();

    LoadTimestampFile( timestampFile );

    return true;
}

void GtsView::LoadTimestampFile( const char* const fileName )
{
    m_timestamps.clear();
    QFile file( fileName );

    if ( file.open( QFile::ReadOnly ) )
    {
        QTextStream stream( &file );

        while ( !stream.atEnd() )
        {
            QString line = stream.readLine();
            QStringList fields = line.split( ' ' );

            timespec t;
            t.tv_sec  = fields.takeFirst().toInt();
            t.tv_nsec = fields.takeFirst().toInt();

            m_timestamps.push_back(t);
        }
    }
}

/**
    @return The current seek posision in the video file @note This is not the same as the frame timestamp (which comes from system time of capture)!.
    This is the time that should be used to in seek commands.
*/
double GtsView::GetSeekPositionInMilliseconds() const
{
    assert( m_sequencer );
    if ( m_sequencer )
    {
       return m_sequencer->GetTimeStamp();
    }

    return -1.0;
}

/**
    Ready the next frame based upon the timestamp - i.e. seek to frame with specifed timestamp.
    @return true if frame is ready, false if there was an error.
*/
bool GtsView::ReadySeekFrame( double msec )
{
    assert( m_sequencer );
    if ( !m_sequencer )
    {
        return false;
    }

    return m_sequencer->ReadyNextFrame( msec );
}

/**
    Simply ready the next consequetive frame in the video sequence - this is much more
    efficient than seeking (especially on video-files which have a broken/missing seek index).

    @return true if frame is ready, false if there was an error.
*/
bool GtsView::ReadyNextFrame()
{
    assert( m_sequencer );
    if ( !m_sequencer )
    {
        return false;
    }

    const double idx = m_sequencer->GetFrameIndex();
    const double final = m_sequencer->GetNumFrames();

    if ( idx < final )
    {
        return m_sequencer->ReadyNextFrame();
    }

    return false;
}

const IplImage* GtsView::GetNextFrame()
{
    bool bad = true;

    if ( m_sequencer )
    {
        // New frame is available
        const IplImage* img = m_sequencer->RetrieveNextFrame();

        if ( m_imgFrame )
        {
            // An image is already allocated so copy over it.
            // (Assumes all images in sequence are same size!)

            if ( img->nChannels == 3 )
            {
                cvCopyImage( img, m_imgFrame );
            }
            else
            {
                cvConvertImage( img, m_imgFrame, 0 );
            }
        }
        else
        {
            // Need to allocate space for image
            m_imgFrame = cvCreateImage( cvSize(img->width,
                                               img->height), IPL_DEPTH_8U, 3 );

            if ( img->nChannels == 3 )
            {
                cvCopyImage( img, m_imgFrame );
            }
            else
            {
                cvConvertImage( img, m_imgFrame, 0 );
            }
        }

        m_imgFrame->origin = img->origin;

        bad = false;

        if ( m_sequencer->IsLive() )
        {
            SaveThumbnail();
        }
    }
    else
    {
        bad = true;
    }

    if ( bad )
    {
        // If there is no frame
        // clean up and return 0

        if ( m_imgFrame )
        {
            cvReleaseImage( &m_imgFrame );
        }

        m_imgFrame = 0;
    }

    return m_imgFrame;
}

/**
    Perform one iteration of tracking algorithm.

    If the tracker is inactive then we just get the next image
    from the video sequence but do no processing on it.
**/
void GtsView::StepTracker( bool forward, CoverageSystem* coverage )
{
    Q_UNUSED(coverage);

    QImage qimage;

    bool tracking = false;

    if ( m_sequencer->TakeFrame() )
    {
        assert( "video size & calibration size do not match" &&
                m_imgFrame->width == m_imgGrey->width &&
                m_imgFrame->height == m_imgGrey->height );

        // Convert to grey-scale and apply the calibration
        cvConvertImage( m_imgFrame, m_imgGrey, m_sequencer->Flip() );
        m_calScaled->UnwarpGroundPlane( m_imgGrey, m_imgWarp[m_imgIndex] );

        double videoTimeStampInMillisecs = -1.0;
        if ( m_timestamps.size() > 0 )
        {
            unsigned int frame = m_sequencer->GetFrameIndex() - 1;
            assert( frame < m_timestamps.size() );
            timespec t = m_timestamps[frame];
            videoTimeStampInMillisecs = t.tv_sec * 1000.0;
            videoTimeStampInMillisecs += t.tv_nsec * 0.000001;
        }
        else
        {
            videoTimeStampInMillisecs = m_sequencer->GetTimeStamp();
        }

        m_tracker->SetCurrentImage( m_imgWarp[m_imgIndex] );

        // Perform motion detection so
        // that recovery is possible.

        if ( m_tracker->IsActive() )
        {
            tracking = m_tracker->Track( videoTimeStampInMillisecs );
	    // If active but cannot find track then go to lossRecovery
	    if(!tracking)
	    {
		  m_tracker->DoInactiveProcessing( videoTimeStampInMillisecs );
                  m_tracker->LossRecovery();
	    }
        }
        else
        {
            if (forward)
            {
                if ( m_tracker->IsLost() )
                {
                    m_tracker->DoInactiveProcessing( videoTimeStampInMillisecs );
                    m_tracker->LossRecovery();
                }
            }
            else
            {
                m_tracker->Rewind( videoTimeStampInMillisecs );
            }
        }

        qimage = GroundTruthUI::showRobotTrack( m_tracker, tracking );

        m_tool->ImageUpdate( m_id, qimage.rgbSwapped(), m_fps );

        m_imgIndex = 1 - m_imgIndex;
    }
}

void GtsView::ShowRobotTrack()
{
    QImage qimage = GroundTruthUI::showRobotTrack( m_tracker, true );

    m_tool->ImageSet( m_id, qimage.rgbSwapped(), m_fps );
}

void GtsView::HideRobotTrack()
{
    QImage qimage = GroundTruthUI::showRobotTrack( m_tracker, false );

    m_tool->ImageSet( m_id, qimage.rgbSwapped(), m_fps );
}

void GtsView::SetTrackerParam( RobotTracker::paramType param, float value )
{
    if ( m_tracker )
    {
        m_tracker->SetParam( param, value );
    }
}

void GtsView::SetupView( TrackRobotWidget* tool, ImageGrid* imageGrid )
{
    m_tool = tool;

    m_viewer = imageGrid->AddBlankImage( QSize(m_calScaled->GetImageSize().width,
                                               m_calScaled->GetImageSize().height), m_id );

    QObject::connect( (QObject*)m_viewer,
                      SIGNAL( onLeftClick( int, int, int ) ),
                      (QObject*)tool,
                      SLOT( SelectTrack( int, int, int ) ),
                      Qt::AutoConnection );

    QObject::connect( (QObject*)m_viewer,
                      SIGNAL( onRightClick( int ) ),
                      (QObject*)tool,
                      SLOT( ClearTrack( int ) ),
                      Qt::AutoConnection );

    QObject::connect( (QObject*)tool,
                      SIGNAL( UpdateImage( int, const QImage&, double ) ),
                      (QObject*)imageGrid,
                      SLOT( updateImage( int, const QImage&, double ) ),
                      Qt::BlockingQueuedConnection );

    QObject::connect( (QObject*)tool,
                      SIGNAL( SetImage( int, const QImage&, double ) ),
                      (QObject*)imageGrid,
                      SLOT( updateImage( int, const QImage&, double ) ),
                      Qt::AutoConnection );
}
