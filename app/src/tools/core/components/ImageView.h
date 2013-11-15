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

#ifndef IMAGEVIEW_H
#define IMAGEVIEW_H

#include <memory>
#include <string>

#include <QtGui/QImage>
#include <QtGui/QPixmap>
#include <QtGui/QResizeEvent>
#include <QtGui/QPaintEvent>
#include <QtGui/QFrame>
#include <QtGui/QLabel>

/** @brief A class to display images.
 *
 *  Can retain aspect ratio and add a caption.
 *
 */
class ImageView : public QFrame
{
    Q_OBJECT

public:
    enum ConversionMethod
    {
        FastConversion,
        SmoothConversion
    };

    explicit ImageView( QWidget* parent = 0, int id = -1 );

    void Clear();

    void SetImage( const QString& imageName );
    void SetImage( const QImage&  image );
    void SetPreserveAspectRatio( const bool preserveAspectRatio );
    void SetCaption( const QString& caption );

    const QSize GetImageSize  () const { return m_image.size(); }
    double GetImageAspectRatio() const { return ( double )m_image.width () /
                                                ( double )m_image.height(); }
    const QImage GetCurrentImage() const;
    void SetConversionMethod( const ConversionMethod& method );

    void setZoom( double zoom ) { m_zoom = zoom; };

    bool selectionMode( const int force = -1 );

signals:
    void onLeftClick( int id, int x, int y );
    void onRightClick( int id );

    void hueChanged( QRgb val );

protected:
    virtual void resizeEvent( QResizeEvent* );
    virtual void paintEvent ( QPaintEvent* );

    virtual void mousePressEvent( QMouseEvent* event );
    virtual void mouseMoveEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);

private:
    void UpdateScaledPixmap();
    const Qt::TransformationMode GetImageTransformationMode() const;

    QImage                  m_image;
    QImage                  m_scaledImage;
    QPixmap                 m_scaledPixmap;
    Qt::AspectRatioMode     m_aspectRatioMode;
    std::auto_ptr< QLabel > m_captionLabel;
    double                  m_framesPerSec;
    ConversionMethod        m_conversionMethod;

    double                  m_zoom;

    int                     m_id;

    bool m_selectionModeEnabled;
    bool m_selectionStarted;
    bool m_drawSelection;
    QRect m_selectionRect;
};

#endif // IMAGEVIEW_H
