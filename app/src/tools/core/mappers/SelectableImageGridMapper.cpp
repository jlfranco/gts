/* Copyright (C) 2013 iRobot Corporation. All rights reserved.
 *
 * based on CalibrationImageGridMapper.cpp
 */

#include "SelectableImageGridMapper.h"

#include <opencv/cv.h>

SelectableImageGridMapper::SelectableImageGridMapper( SelectableImageGrid& imageGrid ) :
    ConfigKeyMapper( CalibrationSchema::colorCalibImageFileKey ),
    m_grid( imageGrid ),
    m_image(),
    m_notFoundImage(":/image_not_found.png")
{
    m_selectionModeEnabled = false;
}

void SelectableImageGridMapper::CommitData( WbConfig& config )
{
    Q_UNUSED(config);
}

void SelectableImageGridMapper::SetConfig( const WbConfig& config )
{
    if ( !m_currentImageId.isEmpty() )
    {
        UpdateImage( config );
        OverlayCornersIfPossible( config );
        UpdateGrid();
    }
}

bool SelectableImageGridMapper::ImageIsFound() const
{
    return !m_image.isNull();
}

void SelectableImageGridMapper::OverlayCornersIfPossible( const WbConfig& config )
{
    if (ImageIsFound())
    {
        using namespace CalibrationSchema;
        cv::Mat cameraMtx(3, 3, CV_64F);
        cv::Mat distortionCoeffs(5, 1, CV_64F);
        bool successful = !config.GetKeyValue(rowsUsedForCalibrationKey).IsNull() &&
                          !config.GetKeyValue(columnsUsedForCalibrationKey).IsNull();

        const cv::Size gridSize( config.GetKeyValue( columnsUsedForCalibrationKey ).ToInt(),
                                 config.GetKeyValue( rowsUsedForCalibrationKey ).ToInt() );

        if (successful)
        {
            successful = config.GetKeyValue(distortionCoefficientsKey).TocvMat(distortionCoeffs);
        }

        if (successful)
        {
            successful = config.GetKeyValue(cameraMatrixKey).TocvMat(cameraMtx);
        }

        if (successful)
        {
            std::vector<cv::Point2f> reprojectedPoints;
            reprojectedPoints.reserve(gridSize.area());
            const bool canGetPoints = config.GetKeyValue(imageReprojectedPointsKey, m_currentImageId)
                                      .ToStdVectorOfCvPoint2f(reprojectedPoints);
            if (canGetPoints)
            {
                cv::Mat imageMat(m_image.height(), m_image.width(), CV_8UC3, m_image.bits());
                cv::drawChessboardCorners(imageMat, gridSize, reprojectedPoints, true);
            }
        }
    }
}

void SelectableImageGridMapper::UpdateImage( const WbConfig& config )
{
    const KeyValue cameraImageFile = config.GetKeyValue(CalibrationSchema::colorCalibImageFileKey, m_currentImageId);
    const QString fileName(config.GetAbsoluteFileNameFor(cameraImageFile.ToQString()));

    m_image = QImage(fileName).convertToFormat(QImage::Format_RGB888);
}

void SelectableImageGridMapper::SetCurrentImage( const KeyId & imageId )
{
    m_currentImageId = imageId;
}

KeyId SelectableImageGridMapper::GetCurrentImageKey()
{
    return m_currentImageId;
}

/** @brief Toggle (or Enable/disable) selection mode on image.
 *
 *  Will toggle selection mode except if param force is supplied.
 *
 *  @param int force If negative will toggle, if 0 will disable, else enable.
 *  @return bool resulting status of selection mode.
 */
bool SelectableImageGridMapper::selectionMode( const int force )
{
    if ( force >= 0 )
    {
        m_selectionModeEnabled = force > 0;
    }
    else
    {
        m_selectionModeEnabled = !m_selectionModeEnabled;
    }

    // enable selection on current image
    if ( m_currentImageView )
    {
        m_currentImageView->selectionMode( m_selectionModeEnabled? 1:0 );
    }

    return m_selectionModeEnabled;
}

QImage SelectableImageGridMapper::GetRealOrNotFoundImage() const
{
    if (ImageIsFound())
    {
        return m_image;
    }

    return m_notFoundImage;
}

void SelectableImageGridMapper::UpdateGrid()
{
    m_grid.Clear();
    m_currentImageView = m_grid.AddImage(GetRealOrNotFoundImage());
}
