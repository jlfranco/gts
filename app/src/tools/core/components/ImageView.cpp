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

#include "ImageView.h"

#include "RobotTracker.h"

#include <QtGui/QPainter>
#include <QMouseEvent>

#include "Debugging.h"

namespace
{
    const QPalette BlackOnWhitePalette()
    {
        QPalette captionPalette;
        captionPalette.setColor( QPalette::Window,     Qt::white );
        captionPalette.setColor( QPalette::WindowText, Qt::black );
        return captionPalette;
    }
}

ImageView::ImageView( QWidget* parent, int id ) :
    QFrame( parent ),
    m_image(),
    m_aspectRatioMode( Qt::KeepAspectRatio ),
    m_captionLabel( new QLabel( this ) ),
    m_framesPerSec( 0.0 ),
    m_conversionMethod( FastConversion ),
    m_zoom( 1.0 ),
    m_id( id )
{
    m_selectionStarted = false;
    m_drawSelection    = false;
    m_selectionModeEnabled = false;

    const QPoint captionOffset( 20, 20 );
    m_captionLabel->move( captionOffset );
    m_captionLabel->setPalette( BlackOnWhitePalette() );
    m_captionLabel->setAutoFillBackground( true );
    SetCaption( "" );
}

/** @brief Remove currently-set image
 *
 *  Image is cleared internally, but not updated on screen until update()
 *  is called.  This is for reasons of speed (avoid multiple calls to update).
 *
 */
void ImageView::Clear()
{
    SetImage( QImage() );
}

/** @brief Set the image from file
 *
 *  Image is stored internally, but not updated on screen until update()
 *  is called.  This is for reasons of speed (avoid multiple calls to update).
 *
 *  @param imageName The name of the image file to display.
 */
void ImageView::SetImage( const QString& imageName )
{
    SetImage( QImage( imageName ) );
}

/** @brief Set the image to @a image.
 *
 *  Image is stored internally, but not updated on screen until update()
 *  is called.  This is for reasons of speed (avoid multiple calls to update).
 *
 *  @param image The image to display.
 */
void ImageView::SetImage( const QImage& image )
{
    m_image = image;

    UpdateScaledPixmap();
}

/** @brief Set the image caption.
 *
 *  @param caption The string to use for the caption.
 */
void ImageView::SetCaption( const QString& caption )
{
    if ( caption.isEmpty() )
    {
        m_captionLabel->hide();
    }
    else
    {
        m_captionLabel->setText( caption );
        m_captionLabel->adjustSize();
        m_captionLabel->show();
    }
}

const QImage ImageView::GetCurrentImage() const
{
    return m_image;
}

void ImageView::SetConversionMethod( const ConversionMethod& method )
{
    m_conversionMethod = method;
}

/** @brief Toggle (or Enable/disable) selection mode on image.
 *
 *  Will toggle selection mode except if param force is supplied.
 *
 *  @param force If negative will toggle, if 0 will disable, else enable.
 *
 */
bool ImageView::selectionMode( const int force )
{
    if ( force >= 0 )
    {
        m_selectionModeEnabled = force > 0;
    }
    else
    {
        m_selectionModeEnabled = !m_selectionModeEnabled;
    }
    return m_selectionModeEnabled;
}

/** Scale #m_image to fit the available space and store it as a pixmap at the correct size.
 *
 *  The reduces recalculation if the image doen't change.  We only calculate the image scaling and
 *  the colour space transformation when the image changes, or the image resizes.
 */
void ImageView::UpdateScaledPixmap()
{
    if ( m_image.isNull() )
    {
        m_scaledPixmap = QPixmap();
    }
    else
    {
        m_scaledImage = QImage ( m_image.scaled( rect().size(),
                                                 m_aspectRatioMode,
                                                 GetImageTransformationMode() ) );

        const Qt::ImageConversionFlags FAST_CONVERSION =
                    Qt::AutoColor |
                    Qt::ThresholdDither |
                    Qt::ThresholdAlphaDither;

        m_scaledPixmap.convertFromImage( m_scaledImage, FAST_CONVERSION );
    }
}

const Qt::TransformationMode ImageView::GetImageTransformationMode() const
{
    switch ( m_conversionMethod )
    {
        case FastConversion:
            return Qt::FastTransformation;

        case SmoothConversion:
            return Qt::SmoothTransformation;

        default:
            ASSERT( !"Unknown conversion method" );
            return Qt::FastTransformation;
    }
}

/** @brief Set whether the view preserves the images aspect ratio.
 *
 *  @param preserveAspectRatio If @a true, the image is as large as possible @em inside the available
 *  space, but retaining aspect ratio. If @a false, the image is stretched to fill all the available space.
 */
void ImageView::SetPreserveAspectRatio( const bool preserveAspectRatio )
{
    if ( preserveAspectRatio )
    {
        m_aspectRatioMode = Qt::KeepAspectRatio;
    }
    else
    {
        m_aspectRatioMode = Qt::IgnoreAspectRatio;
    }
}

/** @brief Handle the Qt GUI resize event.
 *
 * QResizeEvent parameter unused.
 */
void ImageView::resizeEvent( QResizeEvent* )
{
    UpdateScaledPixmap();
}

/** @brief Handle the Qt GUI paint event.
 *
 *  This occurs when update() is called or an area of the widget needs to be repainted.
 *  QPaintEvent parameter unused.
 */
void ImageView::paintEvent( QPaintEvent* )
{
    QPainter painter( this );

    if ( !m_scaledPixmap.isNull() )
    {
        /// @todo offset image so its centred in its space
        painter.drawPixmap( m_scaledPixmap.rect(), m_scaledPixmap );

        if ( m_drawSelection )
        {
            painter.setPen(QPen(QBrush(QColor(0,0,0,180)),4,Qt::DashLine));
            painter.setBrush(QBrush(QColor(255,255,255,120)));

            painter.drawRect(m_selectionRect);
            m_drawSelection = false;
            QColor color, colorAvg;
            float r = 0.0f, g = 0.0f, b = 0.0f, a = 0.0f;
            int pxCount = 0;

            int x1, x2, y1, y2, iTmp;
            m_selectionRect.getCoords(&x1, &y1, &x2, &y2);
            if ( x2 < x1 )
            {
                iTmp = x1;
                x1 = x2;
                x2 = iTmp;
            }
            if ( y2 < y1 )
            {
                iTmp = y1;
                y1 = y2;
                y2 = iTmp;
            }

            for ( int x = x1; x < x2; x++)
            {
                for ( int y = y1; y < y2; y++ )
                {
                    color = m_scaledImage.pixel(x,y);
                    r += color.redF();
                    g += color.greenF();
                    b += color.blueF();
                    a += color.alphaF();
                    pxCount++;
                }
            }
            float scale = 1.0f / float(pxCount);
            colorAvg = QColor::fromRgbF(r*scale, g*scale, b*scale, a*scale);
            QRgb colorAvgRGB = colorAvg.rgba();

            r = r*scale; g = g*scale; b = b*scale; a = a*scale;
            emit hueChanged(colorAvgRGB);
        }
    }
}

void ImageView::mousePressEvent( QMouseEvent* event )
{
  	if ( event->button() == Qt::LeftButton )
    {
        double scale_x = (double)m_image.size().width() /
                         (double)m_scaledPixmap.rect().size().width();
        double scale_y = (double)m_image.size().height() /
                         (double)m_scaledPixmap.rect().size().height();

        double x = ((double)event->x() * scale_x); // * m_zoom;
        double y = ((double)event->y() * scale_y); // * m_zoom;

        if (( x <= m_image.size().width() ) &&
            ( y <= m_image.size().height() ))
        {
            emit onLeftClick( m_id, x, y );
        }

        if ( m_selectionModeEnabled )
        {
            m_selectionStarted=true;
            m_selectionRect.setTopLeft(event->pos());
            m_selectionRect.setBottomRight(event->pos());
        }

    }

    if ( event->button() == Qt::RightButton )
    {
        emit onRightClick( m_id );
    }

    QFrame::mousePressEvent( event );
}

void ImageView::mouseMoveEvent(QMouseEvent *event)
{
    if (m_selectionStarted)
    {
        m_selectionRect.setBottomRight(event->pos());
        repaint();
    }
}

void ImageView::mouseReleaseEvent(QMouseEvent* event)
{
    Q_UNUSED(event);
    if ( m_selectionStarted )
    {
        m_selectionStarted = false;
        m_selectionModeEnabled = false;
        m_drawSelection = true;
        repaint();
    }
}
