/* Copyright (C) 2013 iRobot Corporation. All rights reserved.
 *
 * based on CalibrationImageGridMapper.cpp
 */

#ifndef SELECTABLEIMAGEGRIDMAPPER_H
#define SELECTABLEIMAGEGRIDMAPPER_H

#include "ConfigKeyMapper.h"
#include "KeyName.h"
#include "SelectableImageGrid.h"
#include "CalibrationSchema.h"

#include <vector>

#include <QtGui/QImage>

class SelectableImageGridMapper: public ConfigKeyMapper
{
    Q_OBJECT

public:
    SelectableImageGridMapper( SelectableImageGrid& imageGrid );

    virtual void CommitData( WbConfig& config );
    virtual void SetConfig( const WbConfig& config );

    void SetCurrentImage( const KeyId& imageId );
    KeyId GetCurrentImageKey();

    bool selectionMode( const int force = -1 );

private:
    void UpdateImage( const WbConfig& config );
    void OverlayCornersIfPossible( const WbConfig& config );
    void UpdateGrid();
    bool ImageIsFound() const;
    QImage GetRealOrNotFoundImage() const;

    SelectableImageGrid& m_grid;
    QImage m_image;
    KeyId  m_currentImageId;
    const QImage m_notFoundImage;

    bool m_selectionModeEnabled;
    SelectableImageView* m_currentImageView;
};

#endif // SELECTABLEIMAGEGRIDMAPPER_H
