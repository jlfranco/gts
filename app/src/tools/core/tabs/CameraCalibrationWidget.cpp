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

#include "CameraCalibrationWidget.h"

#include "ui_CameraCalibrationWidget.h"

#include "CalibrationImageGridMapper.h"
#include "SelectableImageGridMapper.h"
#include "CalibrationImageTableMapper.h"
#include "ColorCalibrationImageTableMapper.h"
#include "VideoSource.h"
#include "CameraHardware.h"
#include "Message.h"
#include "CalibrationSchema.h"
#include "CalibrationAlgorithm.h"
#include "CaptureLiveBtnController.h"
#include "WbKeyValues.h"
#include "ImagePrintPreviewDlg.h"
#include "ChessboardImage.h"
#include "ButtonLabelMapper.h"
#include "PrintCalibrationGrid.h"
#include "UnknownLengthProgressDlg.h"
#include "CalibrateCameraResultsMapper.h"

#include <QtGui/QCheckBox>
#include <QtGui/QFileDialog>
#include <QtGui/QRegExpValidator>

#include "FileUtilities.h"
#include "FileDialogs.h"

#include "WbConfigTools.h"

#if defined(__MINGW32__) || defined(__GNUC__)
    #include <Callback.h>
#else
    #include <functional>
#endif

/** @todo add newly-captured files with relative file paths
 */
CameraCalibrationWidget::CameraCalibrationWidget( CameraHardware& cameraHardware,
                                                      QWidget* const parent ) :
    Tool          ( parent, CreateSchema() ),
    m_cameraHardware( cameraHardware ),
    m_captureLiveBtnController(),
    m_ui          ( new Ui::CameraCalibrationWidget ),
    m_imageGridMapper( 0 ),
    m_imageTableMapper( 0 ),
    m_imageGridMapperColor( 0 ),
    m_imageTableMapperColor( 0 )
{
    m_ui->setupUi( this );

    m_captureLiveBtnController.reset(
        new CaptureLiveBtnController( *m_ui->m_captureLiveBtn,
                                      *m_ui->m_captureCancelBtn, *this, m_cameraHardware ) );

    QHeaderView* horizHeader = m_ui->m_imagesTableWidget->horizontalHeader();
    horizHeader->setResizeMode( 0, QHeaderView::Stretch );
    horizHeader->setResizeMode( 1, QHeaderView::ResizeToContents );

    horizHeader = m_ui->m_imagesTableWidgetColor->horizontalHeader();
    horizHeader->setResizeMode( 0, QHeaderView::Stretch );
    horizHeader->setResizeMode( 1, QHeaderView::ResizeToContents );

    AddMapper( CalibrationSchema::gridSquareSizeInCmKey, m_ui->m_gridSquareSizeSpinBox );
    AddMapper( CalibrationSchema::gridRowsKey,       m_ui->m_gridRowsSpinBox );
    AddMapper( CalibrationSchema::gridColumnsKey,    m_ui->m_gridColumnsSpinBox );
    AddMapper( CalibrationSchema::noTangentialDistortionKey,
               m_ui->m_zeroTangentialCheckBox );
    AddMapper( CalibrationSchema::fixPrincipalPointKey,    m_ui->m_fixPrincipalPtCheckBox );
    AddMapper( CalibrationSchema::flipImagesKey,           m_ui->m_flipCheckBox );
    AddMapper( CalibrationSchema::fixedAspectRatioKey,     m_ui->m_aspectRatioLineEdit );
    AddMapper( CalibrationSchema::shouldFixAspectRatioKey, m_ui->m_fixAspectRatioGroup );

    AddMapper( new ButtonLabelMapper( *m_ui->m_printGridBtn,
                                      tr( "Print Grid" ),
                                      tr( " (%1x%2)" ),
                                      CalibrationSchema::gridRowsKey,
                                      CalibrationSchema::gridColumnsKey ) );

    AddMapper( CalibrationSchema::hueLeftKey, m_ui->m_hueLeft );
    AddMapper( CalibrationSchema::hueRightKey, m_ui->m_hueRight );
    AddMapper( CalibrationSchema::luminanceMaxKey, m_ui->m_luminanceMax );
    AddMapper( CalibrationSchema::luminanceMinKey, m_ui->m_luminanceMin );
    AddMapper( CalibrationSchema::saturationMinKey, m_ui->m_saturationMin );
    AddMapper( CalibrationSchema::grayRedKey, m_ui->m_grayRed );
    AddMapper( CalibrationSchema::grayGreenKey, m_ui->m_grayGreen );
    AddMapper( CalibrationSchema::grayBlueKey, m_ui->m_grayBlue );
    AddMapper( CalibrationSchema::grayPercentageKey, m_ui->m_grayPercentage );
    AddMapper( CalibrationSchema::methodKey, m_ui->m_method );

    m_imageTableMapper = new CalibrationImageTableMapper( *m_ui->m_imagesTableWidget );
    AddMapper( m_imageTableMapper );
    m_imageGridMapper = new CalibrationImageGridMapper( *m_ui->m_imageGrid );
    AddMapper( m_imageGridMapper );

    m_imageTableMapperColor = new ColorCalibrationImageTableMapper( *m_ui->m_imagesTableWidgetColor );
    AddMapper( m_imageTableMapperColor );
    m_imageGridMapperColor = new SelectableImageGridMapper( *m_ui->m_imageGridColor );
    AddMapper( m_imageGridMapperColor );
    selectionMode = NONE;

    AddMapper( new CalibrateCameraResultsMapper( *m_ui->m_resultsTextBrowser ) );

    QDoubleValidator* validator = new QDoubleValidator;
    m_ui->m_aspectRatioLineEdit->setValidator( validator );

    m_ui->m_calibrationType->setCurrentIndex( 0 );
    m_ui->m_optionsTabs->setCurrentIndex( 0 );

    QObject::connect( m_ui->m_fromFileBtn,
                      SIGNAL( clicked() ),
                      this,
                      SLOT( FromFileClicked() ) );
    QObject::connect( m_ui->m_captureLiveBtn,
                      SIGNAL( clicked() ),
                      this,
                      SLOT( CaptureLiveBtnClicked() ) );
    QObject::connect( m_ui->m_captureCancelBtn,
                      SIGNAL( clicked() ),
                      this,
                      SLOT( CaptureCancelBtnClicked() ) );
    QObject::connect( m_ui->m_calibrateBtn,
                      SIGNAL( clicked() ),
                      this,
                      SLOT( CalibrateBtnClicked() ) );
    QObject::connect( m_ui->m_printGridBtn,
                      SIGNAL( clicked() ),
                      this,
                      SLOT( PrintGridBtnClicked() ) );
    QObject::connect( m_ui->m_imagesTableWidget,
                      SIGNAL( currentItemChanged ( QTableWidgetItem*,
                                                   QTableWidgetItem* ) ),
                      this,
                      SLOT( ImageTableItemChanged( QTableWidgetItem*,
                                                   QTableWidgetItem* ) ) );

    QObject::connect( m_ui->m_imagesTableWidgetColor,
                      SIGNAL( currentItemChanged ( QTableWidgetItem*,
                                                   QTableWidgetItem* ) ),
                      this,
                      SLOT( ImageTableItemChangedColor( QTableWidgetItem*,
                                                        QTableWidgetItem* ) ) );


    QObject::connect( m_ui->m_colorGetImageFromFileBtn,
                      SIGNAL( clicked() ),
                      this,
                      SLOT( ColorFromFileClicked() ) );
    QObject::connect( m_ui->m_hueLeftLabel,
                      SIGNAL( clicked() ),
                      this,
                      SLOT( HueLeftBtnClicked() ) );
    QObject::connect( m_ui->m_hueRightLabel,
                      SIGNAL( clicked() ),
                      this,
                      SLOT( HueRightBtnClicked() ) );
    QObject::connect( m_ui->m_colorCalibrateBtn,
                      SIGNAL( clicked() ),
                      this,
                      SLOT( ColorCalibrateBtnClicked() ) );

    QObject::connect( m_ui->m_imageGridColor,
                      SIGNAL( hueSet(QRgb) ),
                      this,
                      SLOT( HueChanged(QRgb) ) );

}

CameraCalibrationWidget::~CameraCalibrationWidget()
{
    delete m_ui;
}

const QString CameraCalibrationWidget::GetSubSchemaDefaultFileName() const
{
    return "calibration.xml";
}

void CameraCalibrationWidget::ImageTableItemChanged(QTableWidgetItem* current,
                                                      QTableWidgetItem* previous)
{
    Q_UNUSED(previous);

    assert( m_imageGridMapper );
    if ( m_imageGridMapper && current )
    {
        QTableWidgetItem* currentRowNameItem =
            m_ui->m_imagesTableWidget->item(current->row(),
                                            CalibrationImageTableMapper::nameColumn );
        m_imageGridMapper->SetCurrentImage(
            currentRowNameItem->data(CalibrationImageTableMapper::idRoleOnName)
                                     .toString(),
                                     false);
        ReloadCurrentConfig( m_imageTableMapper ); //must exclude table here to ensure it still has a "current row" for delete
    }
}

void CameraCalibrationWidget::ImageTableItemChangedColor(QTableWidgetItem* current,
                                                         QTableWidgetItem* previous)
{
    Q_UNUSED(previous);

    assert( m_imageGridMapperColor );
    if ( m_imageGridMapperColor && current )
    {
        QTableWidgetItem* currentRowNameItem =
            m_ui->m_imagesTableWidgetColor->item(current->row(),
                                            ColorCalibrationImageTableMapper::nameColumn );
        m_imageGridMapperColor->SetCurrentImage(
            currentRowNameItem->data(ColorCalibrationImageTableMapper::idRoleOnName)
                                     .toString());
        ReloadCurrentConfig( m_imageTableMapperColor ); //must exclude table here to ensure it still has a "current row" for delete
    }
}

void CameraCalibrationWidget::FromFile(const bool colorCalibFile)
{
    const QString relativePath = (colorCalibFile)?"colorCalibrationImages/":"calibrationImages/";
    // Make sure folder is there before adding file
    const QString fileDirPath( GetCurrentConfig().GetAbsoluteFileNameFor( relativePath ) );
    const bool mkPathSuccessful = QDir().mkpath( fileDirPath );

    if (!mkPathSuccessful)
    {
        Message::Show( this,
                       tr( "Camera Calibration Tool" ),
                       tr( "Folder is missing!"),
                       Message::Severity_Critical );
        return;
    }

    // Display file selection dialog
    FileDialogs::ExtendedFileDialog fileDialog( this,
                                                tr( "Select Image(s) to Add" ),
                                                GetCurrentConfig().GetAbsoluteFileInfo().absolutePath(),
                                                "Images( *.png *.jpg *.bmp *.ppm );;All Files( * )",
                                                false );
    const int result = fileDialog.exec();
    if ( result == QFileDialog::Accepted )
    {
        QStringList filesToOpen( fileDialog.selectedFiles() );

        foreach (QString imageName, filesToOpen)
        {
            WbConfigTools::FileNameMode mode = WbConfigTools::FileNameMode_RelativeInsideWorkbench;

            if ( FileUtilities::FileIsExternal( imageName, GetCurrentConfig() ) )
            {
                if ( fileDialog.CopyFileSelected() )
                {
                    const QString dstFile = QFileInfo( imageName ).fileName();

                    const QString newImageName(
                        GetCurrentConfig().GetAbsoluteFileNameFor( relativePath + dstFile ) );

                    QFile::copy( imageName, newImageName );

                    imageName = newImageName;
                }
                else
                {
                    if ( fileDialog.RelativeSelected() )
                    {
                        mode = WbConfigTools::FileNameMode_Relative;
                    }
                    else
                    {
                        mode = WbConfigTools::FileNameMode_Absolute;
                    }
                }
            }
            AddImageIfValid( imageName, mode, colorCalibFile );

        }
    }
}

void CameraCalibrationWidget::FromFileClicked()
{
    FromFile(false);
}

void CameraCalibrationWidget::ColorFromFileClicked()
{
    FromFile(true);
}

void CameraCalibrationWidget::CalibrateBtnClicked()
{
    CalibrationAlgorithm alg;
    UnknownLengthProgressDlg* const progressDialog = new UnknownLengthProgressDlg( this );
    progressDialog->Start( tr( "Calibrating" ), tr( "" ) );
    const bool calibrationSuccessful = alg.Run( GetCurrentConfig() );
    ReloadCurrentConfig();

    if ( calibrationSuccessful )
    {
        progressDialog->Complete( tr( "Camera Calibration Successful" ),
                                  tr( "The camera has been calibrated.\n"
                                      "Average reprojection error is: %1." )
                                      .arg( GetCurrentConfig()
                                            .GetKeyValue( CalibrationSchema::avgReprojectionErrorKey )
                                            .ToDouble() ) );
    }
    else
    {
        progressDialog->ForceClose();

        Message::Show( 0,
                       tr( "Camera Calibration Tool" ),
                       tr( "See the log for details!" ),
                       Message::Severity_Critical );
    }
}

void CameraCalibrationWidget::PrintGridBtnClicked()
{
    PrintCalibrationGrid( m_ui->m_gridRowsSpinBox->value()+1,
                          m_ui->m_gridColumnsSpinBox->value()+1 );
}

ImageView* const CameraCalibrationWidget::CreateStreamingView( const QSize& imageSize )
{
    m_ui->m_imageGrid->Clear();

    return m_ui->m_imageGrid->AddBlankImage( imageSize );
}

void CameraCalibrationWidget::ReloadCurrentConfigToolSpecific()
{
    const WbKeyValues::ValueIdPairList calibImages(
                GetCurrentConfig().GetKeyValues( CalibrationSchema::imageFileKey ) );

    m_ui->m_calibrateBtn->setEnabled( calibImages.size() >= 2 );

    const WbKeyValues::ValueIdPairList colorCalibImages(
                GetCurrentConfig().GetKeyValues( CalibrationSchema::colorCalibImageFileKey ) );

    m_ui->m_colorCalibrateBtn->setEnabled( colorCalibImages.size() >= 1 );
}

void CameraCalibrationWidget::AddImageIfValid( const QString& imageFileName,
                                               const WbConfigTools::FileNameMode& mode,
                                               const bool colorCalibImage )
{
    if ( !imageFileName.isEmpty() )
    {
        if ( !colorCalibImage )
        {
            WbConfigTools::AddFileName( GetCurrentConfig(),
                                        imageFileName,
                                        CalibrationSchema::imageFileKey,
                                        mode );
        }
        else
        {
            WbConfigTools::AddFileName( GetCurrentConfig(),
                                        imageFileName,
                                        CalibrationSchema::colorCalibImageFileKey,
                                        mode );
        }
        ReloadCurrentConfig();
    }
}

void CameraCalibrationWidget::CaptureLiveBtnClicked()
{
    WbConfig config( GetCurrentConfig() );
    const WbConfig cameraConfig( config.FindAncestor( KeyName( "camera" ) ) );

    const QString newFileNameFormat( config.GetAbsoluteFileNameFor( "calibrationImages/Calib%1.png" ) );

    const QString capturedImageFileName =
            m_captureLiveBtnController->CaptureLiveBtnClicked(cameraConfig,
                                                              newFileNameFormat,
#if defined(__MINGW32__) || defined(__GNUC__)
                                                              MakeCallback( this,
                                                                            &CameraCalibrationWidget::CreateStreamingView ) );
#else
															  [this](const QSize& imageSize) -> ImageView*
                                                              {
                                                                  return CreateStreamingView(imageSize);
                                                              } );
#endif

    AddImageIfValid( capturedImageFileName, WbConfigTools::FileNameMode_RelativeInsideWorkbench, false );
}

void CameraCalibrationWidget::CaptureCancelBtnClicked()
{
    m_ui->m_captureCancelBtn->setEnabled(false);
    m_ui->m_captureLiveBtn->setEnabled(true);
    m_captureLiveBtnController->CaptureCancelBtnClicked();
}

void CameraCalibrationWidget::HueLeftBtnClicked()
{
    m_imageGridMapperColor->selectionMode( 1 );
    selectionMode = LEFT;
}

void CameraCalibrationWidget::HueRightBtnClicked()
{
    m_imageGridMapperColor->selectionMode( 1 );
    selectionMode = RIGHT;
}

using namespace std;
void CameraCalibrationWidget::HueChanged( QRgb val )
{
    // remove the alpha crap
    QString valNum = QString::number(val,16);
    valNum.remove(0,2);

    // build hex only
    QString hex = QString("#%1").arg(valNum);

    // build format string
    QString str = QString("background-color:%1;").arg(hex);


    // set color and save info
    if ( selectionMode == LEFT )
    {
        m_ui->m_hueLeft->setStyleSheet(str);
        //        emit hueLeftSet(hex);

        cout << "Hue Left: ";
    }
    else
    {
        m_ui->m_hueRight->setStyleSheet(str);
        //        emit hueRightSet(hex);
        cout << "Hue Right: ";
    }
    selectionMode = NONE;
    cout << hex.toStdString() << endl;
}

void CameraCalibrationWidget::ColorCalibrateBtnClicked()
{
    /*TODO implement me! */
    CalibrationAlgorithm alg;
    UnknownLengthProgressDlg* const progressDialog = new UnknownLengthProgressDlg( this );
    progressDialog->Start( tr( "Performing color calibration" ), tr( "" ) );

    //    const bool calibrationSuccessful = alg.Run( GetCurrentConfig() );
    //    ReloadCurrentConfig();
    const bool calibrationSuccessful = false;

    if ( calibrationSuccessful )
    {
        progressDialog->Complete( tr( "Camera Color Calibration Successful" ),
                                  tr( "The camera has been color calibrated.\n"
                                      "Average reprojection error is: %1." )
                                      .arg( GetCurrentConfig()
                                            .GetKeyValue( CalibrationSchema::avgReprojectionErrorKey )
                                            .ToDouble() ) );
    }
    else
    {
        progressDialog->ForceClose();

        Message::Show( 0,
                       tr( "Camera Color Calibration Tool" ),
                       tr( "NOT IMPLEMENTED!" ),
                       Message::Severity_Critical );
    }
}

bool CameraCalibrationWidget::IsDataValid() const
{
    if (GetCurrentConfig().IsNull()) return true;

    bool valid = true;

    if ( m_ui->m_gridSquareSizeSpinBox->value() == 0.0 )
    {
        valid = valid && false;
        Tool::HighlightLabel(m_ui->m_gridSquareSizeLabel, true);
    }
    else {
        Tool::HighlightLabel(m_ui->m_gridSquareSizeLabel, false);
    }
    if ( m_ui->m_gridRowsSpinBox->value() == 0 )
    {
        valid = valid && false;
        Tool::HighlightLabel(m_ui->m_gridRowsLabel, true);
    }
    else {
        Tool::HighlightLabel(m_ui->m_gridRowsLabel, false);
    }
    if ( m_ui->m_gridColumnsSpinBox->value() == 0 )
    {
        valid = valid && false;
        Tool::HighlightLabel(m_ui->m_gridColumnsLabel, true);
    }
    else {
        Tool::HighlightLabel(m_ui->m_gridColumnsLabel, false);
    }

    return valid;
}

bool CameraCalibrationWidget::CanClose() const
{
    return IsDataValid() && !m_captureLiveBtnController->CurrentlyStreamingLiveSource();
}

const QString CameraCalibrationWidget::CannotCloseReason() const
{
    return tr("Please complete all highlighted boxes before leaving this tab.");
}

const WbSchema CameraCalibrationWidget::CreateSchema()
{
    using namespace CalibrationSchema;
    WbSchema schema( CreateWorkbenchSubSchema( schemaName,
                                               tr( "Calibration" ) ) );

    schema.AddKeyGroup( gridGroup,
                        WbSchemaElement::Multiplicity::One,
                        KeyNameList() << gridSquareSizeInCmKey
                                      << gridRowsKey
                                      << gridColumnsKey,
                        DefaultValueMap().WithDefault( gridSquareSizeInCmKey,
                                                       KeyValue::from( 0.0 ) )
                                         .WithDefault( gridRowsKey,
                                                       KeyValue::from( 0 ) )
                                         .WithDefault( gridColumnsKey,
                                                       KeyValue::from( 0 ) ) );

    schema.AddKeyGroup( imageGroup,
                        WbSchemaElement::Multiplicity::One,
                        KeyNameList() << imageFileKey
                                      << imageErrorKey
                                      << imageReprojectedPointsKey );

    schema.AddKeyGroup( advancedGroup,
                        WbSchemaElement::Multiplicity::One,
                        KeyNameList() << noTangentialDistortionKey
                                      << fixPrincipalPointKey
                                      << flipImagesKey
                                      << shouldFixAspectRatioKey
                                      << fixedAspectRatioKey,
                        DefaultValueMap().WithDefault( noTangentialDistortionKey,
                                                       KeyValue::from( true ) )
                                         .WithDefault( fixPrincipalPointKey,
                                                       KeyValue::from( false ) )
                                         .WithDefault( flipImagesKey,
                                                       KeyValue::from( false ) )
                                         .WithDefault( shouldFixAspectRatioKey,
                                                       KeyValue::from( false ) )
                                         .WithDefault( fixedAspectRatioKey,
                                                       KeyValue::from( 1.0 ) ));

    schema.AddKeyGroup( resultsGroup,
                        WbSchemaElement::Multiplicity::One,
                        KeyNameList() << calibrationSuccessfulKey
                                      << calibrationDateKey
                                      << calibrationTimeKey
                                      << rowsUsedForCalibrationKey
                                      << columnsUsedForCalibrationKey
                                      << imageHeightKey
                                      << imageWidthKey
                                      << cameraMatrixKey
                                      << distortionCoefficientsKey
                                      << invDistortionCoefficientsKey
                                      << avgReprojectionErrorKey );

    schema.AddKeyGroup( colorCalibrationGroup,
                        WbSchemaElement::Multiplicity::One,
                        KeyNameList() << hueLeftKey
                                      << hueRightKey
                                      << luminanceMaxKey
                                      << luminanceMinKey
                                      << saturationMinKey
                                      << grayRedKey
                                      << grayGreenKey
                                      << grayBlueKey
                                      << grayPercentageKey
                                      << methodKey,
                        DefaultValueMap().WithDefault( hueLeftKey,
                                                       KeyValue::from( "#FFFFFFFF" ) )
                                         .WithDefault( hueRightKey,
                                                       KeyValue::from( "#FFFFFFFF" ) )
                                         .WithDefault( luminanceMaxKey,
                                                       KeyValue::from( 1 ) )
                                         .WithDefault( luminanceMinKey,
                                                       KeyValue::from( 0 ) )
                                         .WithDefault( saturationMinKey,
                                                       KeyValue::from( 0.5 ) ) /*TODO define default*/
                                         .WithDefault( grayRedKey,
                                                       KeyValue::from( 0.5 ) )
                                         .WithDefault( grayGreenKey,
                                                       KeyValue::from( 0.5 ) )
                                         .WithDefault( grayBlueKey,
                                                       KeyValue::from( 0.5 ) )
                                         .WithDefault( grayPercentageKey,
                                                       KeyValue::from( 0.5 ) )
                                         .WithDefault( methodKey,
                                                       KeyValue::from( 0 ) ));

    schema.AddKeyGroup( colorCalibImageGroup,
                        WbSchemaElement::Multiplicity::One,
                        KeyNameList() << colorCalibImageFileKey
                                      << colorCalibImageErrorKey );

    return schema;
}
