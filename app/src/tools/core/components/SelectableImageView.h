/* Copyright (C) 2013 iRobot Corporation. All rights reserved.
 *
 * based on ImageView.h
 */

#ifndef SELECTABLEIMAGEVIEW_H
#define SELECTABLEIMAGEVIEW_H

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
class SelectableImageView : public QFrame
{
    Q_OBJECT

public:
    enum ConversionMethod
    {
        FastConversion,
        SmoothConversion
    };

    explicit SelectableImageView( QWidget* parent = 0, int id = -1 );

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

#endif // SELECTABLEIMAGEVIEW_H
