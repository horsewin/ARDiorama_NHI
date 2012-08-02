#include "FingertipPoseEstimation.h"
#include <iostream>

extern int img_width;
extern int img_height;

FingertipPoseEstimation::FingertipPoseEstimation(void)
{
    // Images
    _pImage = cvCreateImage(cvSize(img_width,img_height),IPL_DEPTH_8U,3);
    _pGray = cvCreateImage(cvSize(img_width,img_height),IPL_DEPTH_8U,1);
    _pSkinColorImage = cvCreateImage(cvSize(img_width,img_height),IPL_DEPTH_8U,1);

    _HandRegion.LoadSkinColorProbTable();
}

FingertipPoseEstimation::~FingertipPoseEstimation(void)
{
    Terminate();
}

bool FingertipPoseEstimation::Initialize( IplImage * srcImage, char * calibFilename )
{
    bool fResult = false;

    //
    // Font
    //
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN, 1, 1, 0, 1, 8 );

    //
    // Images
    //
    if ( !_pImage )
    {
        _pImage = cvCreateImage( cvGetSize( srcImage ), 8, 3 );
        _pImage->origin = srcImage->origin;

        _pGray = cvCreateImage( cvGetSize( _pImage ), 8, 1 );
        _pGray->origin = _pImage->origin;
    }

    // Hand Region
    if ( _HandRegion.LoadSkinColorProbTable() == false )
    {
      return fResult;      
    }

    //
    // done
    //
    fResult = true;

    return fResult;
}

void FingertipPoseEstimation::Terminate()
{
    if ( _pImage )
    {
        cvReleaseImage( &_pImage );
        _pImage = 0;
    }
    if ( _pGray )
    {
        cvReleaseImage( &_pGray );
        _pGray = 0;
    }
    if ( _pSkinColorImage )
    {
        cvReleaseImage( &_pSkinColorImage );
        _pSkinColorImage = 0;
    }
}

void FingertipPoseEstimation::OnCapture( IplImage * frame, int64 nTickCount, IplImage * gray )
{
    //
    // uhoh, let me do a little better way of this.
    // maybe we just don't need to copy when we have already another
    // copy of the images. anyway, i am gonna do this for now.
    //
    cvCopy( frame, _pImage );
    cvConvertImage( _pImage, _pGray, CV_BGR2GRAY );
}

void FingertipPoseEstimation::OnProcess()
{
    //
    // Segment Hand Region
    //
    cvCopyImage( _HandRegion.GetHandRegion( _pImage, _pGray), _pSkinColorImage );
    IplImage * handRegion = _pSkinColorImage;

}

IplImage * FingertipPoseEstimation::OnDisplay( IplImage * image )
{
    //if ( !image )
    //{
    //    image = _pImage;
    //}

    ////
    //// Fingertips ( Candidates )
    ////
    //static CvScalar color[] = {
    //    {{64,64,255}},   // red
    //    {{0,128,255}},  // orange
    //    {{0,255,255}},  // yellow
    //    {{0,192,0}},  // green
    //    {{255,0,0}}     // blue
    //};

    //for ( int i = 0 ; i < NUM_FINGERTIP; i ++ )
    //{
    //    CvPoint2D32f * pt = _FingertipTracker.QueryFingertip( i );
    //    if ( pt )
    //    {
    //        cvCircle(
    //            image,
    //            cvPointFrom32f( *pt ),
    //            _FingertipTracker.QueryFingertipDist( i ),
    //            _fFingertipDetected ? color[i] : CV_RGB(255,0,0),
    //            -1,
    //            8,
    //            0 );
    //    }
    //}/**/

    //for ( int i = 0 ; i < _FingerTip._numEndPoint ; i ++ )
    //{
    //    /*
    //    cvCircle(
    //        image,
    //        _FingerTip.QueryEndPoint( i ),
    //        _FingerTip.QueryEndPointDist( i ),
    //        CV_RGB(0,255,0),
    //        1,8,0 );/**/
    //    
    //    CvPoint center = cvPointFrom32f( _FingerTip._Ellipse[i].center );
    //    CvSize size = cvSize( _FingerTip._Ellipse[i].size.width * 0.5, _FingerTip._Ellipse[i].size.height * 0.5 );

    //    if ( size.width > 0 && size.height > 0 &&
    //         size.width < image->width && size.height < image->height )
    //    {
    //        cvEllipse(
    //            image,
    //            center,
    //            size,
    //            - _FingerTip._Ellipse[i].angle,
    //            0,
    //            360,
    //            CV_RGB(0,255,0),
    //            1,
    //            CV_AA, 0 );
    //    }/**/
    //}

    ////
    //// Max Distance Point + Region of Interest
    ////
    //
    //cvCircle( image, _FingerTip._maxDistPoint, _FingerTip._maxDistValue * 0.7, CV_RGB(255,255,0), 1, 8, 0 );
    //int i1 = _FingerTip._maxDistPoint.y - 4 * _FingerTip._maxDistValue;
    //int i2 = _FingerTip._maxDistPoint.y + 4 * _FingerTip._maxDistValue;
    //int j1 = _FingerTip._maxDistPoint.x - 4 * _FingerTip._maxDistValue;
    //int j2 = _FingerTip._maxDistPoint.x + 4 * _FingerTip._maxDistValue;
    ////cvRectangle( image, cvPoint(j1, i1), cvPoint(j2, i2), CV_RGB(0,255,0), 1 );

    return image;
}

IplImage * FingertipPoseEstimation::QueryHandImage()
{
	return _HandRegion.QueryHandRegion();
}
