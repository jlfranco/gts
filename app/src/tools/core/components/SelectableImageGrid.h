
#ifndef SELECTABLEIMAGEGRID_H
#define SELECTABLEIMAGEGRID_H

#include <QtGui/QFrame>
#include <QtGui/QScrollArea>
#include <QtCore/QString>
#include <QtGui/QImage>
#include <QtGui/QGridLayout>

#include "SelectableImageView.h"

#include <vector>
#include <memory>

namespace Ui
{
    class SelectableImageGrid;
}

/**
 *  @brief A class to show a grid of images, laid out in order to make each
 *  image as large as possible.
 *
 */
class SelectableImageGrid : public QScrollArea
{
    Q_OBJECT

public:
    explicit SelectableImageGrid( QWidget* parent = 0 );
    ~SelectableImageGrid();

    typedef std::unique_ptr<SelectableImageView> SelectableImageViewPtr;
    SelectableImageView* const AddImage( const QString& imageFileName, const QString& caption = QString(), int id = 0 );
    SelectableImageView* const AddImage( const QImage& image,          const QString& caption = QString(), int id = 0 );
    SelectableImageView *const AddBlankImage( const QSize& imageSize, int id = 0 );

    void Clear();

public slots:
     void updateImage( int id, const QImage& image, double fps );

protected:
    virtual void resizeEvent( QResizeEvent* );
    virtual void wheelEvent( QWheelEvent* event );

private:
    void InsertImageInGrid( const int imageIndex );
    void ReflowImages();
    void CalculateNumRowsColumns();

    double GetImageAreaWith( const size_t rows, const size_t columns ) const;

    Ui::SelectableImageGrid*             m_ui;         ///< The UI class created by Qt designer.
    std::vector<SelectableImageViewPtr>  m_imageViews; ///< The image views to display.
    size_t                     m_rows;       ///< The current number of rows.
    size_t                     m_columns;    ///< The current number of columns.
    QFrame*                    m_frame;
    QGridLayout*               m_gridLayout;
    double                     m_zoom;
};

#endif // SELECTABLEIMAGEGRID_H
