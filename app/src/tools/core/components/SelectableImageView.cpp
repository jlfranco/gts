/* Copyright (C) 2013 iRobot Corporation. All rights reserved.
 *
 * based on ImageView.cpp
 */

#include "SelectableImageView.h"

#include "RobotTracker.h"

#include <QtGui/QPainter>
#include <QMouseEvent>

#include <iostream>
using namespace std;

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

SelectableImageView::SelectableImageView( QWidget* parent, int id ) :
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
void SelectableImageView::Clear()
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
void SelectableImageView::SetImage( const QString& imageName )
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
void SelectableImageView::SetImage( const QImage& image )
{
    m_image = image;

    UpdateScaledPixmap();
}

/** @brief Set the image caption.
 *
 *  @param caption The string to use for the caption.
 */
void SelectableImageView::SetCaption( const QString& caption )
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

const QImage SelectableImageView::GetCurrentImage() const
{
    return m_image;
}

void SelectableImageView::SetConversionMethod( const ConversionMethod& method )
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
bool SelectableImageView::selectionMode( const int force )
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
void SelectableImageView::UpdateScaledPixmap()
{
    if ( m_image.isNull() )
    {
        m_scaledPixmap = QPixmap();
    }
    else
    {
        QImage scaledImage( m_image.scaled( rect().size(),
                                            m_aspectRatioMode,
                                            GetImageTransformationMode() ) );

        const Qt::ImageConversionFlags FAST_CONVERSION =
                    Qt::AutoColor |
                    Qt::ThresholdDither |
                    Qt::ThresholdAlphaDither;

        m_scaledPixmap.convertFromImage( scaledImage, FAST_CONVERSION );
    }
}

const Qt::TransformationMode SelectableImageView::GetImageTransformationMode() const
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
void SelectableImageView::SetPreserveAspectRatio( const bool preserveAspectRatio )
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
void SelectableImageView::resizeEvent( QResizeEvent* )
{
    UpdateScaledPixmap();
}

/** @brief Handle the Qt GUI paint event.
 *
 *  This occurs when update() is called or an area of the widget needs to be repainted.
 *  QPaintEvent parameter unused.
 */
void SelectableImageView::paintEvent( QPaintEvent* )
{
    QPainter painter( this );

    if ( !m_scaledPixmap.isNull() )
    {
        /// @todo offset image so its centred in its space
        painter.drawPixmap( m_scaledPixmap.rect(), m_scaledPixmap );
    }

    if ( m_drawSelection )
    {
        painter.setPen(QPen(QBrush(QColor(0,0,0,180)),4,Qt::DashLine));
        painter.setBrush(QBrush(QColor(255,255,255,120)));
 
        painter.drawRect(m_selectionRect);
        //        m_ui->m_view->update();
        m_drawSelection = false;
    }
}

void SelectableImageView::mousePressEvent( QMouseEvent* event )
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

            cout << "PRESS:";
            cout << event->x();
            cout << " -- Y:";
            cout << event->y();
            cout << endl;
        }

    }

    if ( event->button() == Qt::RightButton )
    {
        emit onRightClick( m_id );
    }

    QFrame::mousePressEvent( event );
}

void SelectableImageView::mouseMoveEvent(QMouseEvent *event)
{
    if (m_selectionStarted)
    {
        m_selectionRect.setBottomRight(event->pos());
        repaint();

        cout << "X:";
        cout << event->x();
        cout << " -- Y:";
        cout << event->y();
        cout << endl;
    }
}

void SelectableImageView::mouseReleaseEvent(QMouseEvent* event)
{
    if ( m_selectionStarted )
    {
        m_selectionStarted = false;
        m_selectionModeEnabled = false;
        m_drawSelection = true;
        repaint();

        cout << "RELEASE:";
        cout << event->x();
        cout << " -- Y:";
        cout << event->y();
        cout << endl;
    }
}
