#ifndef _FINGERTIP_POSE_ESTIMATION_H_
#define _FINGERTIP_POSE_ESTIMATION_H_

#include "HandRegion.h"
#ifndef OPENCV
#define OPENCV
	#include <opencv/cv.h>
	#include <opencv/highgui.h>
	#include <opencv2/opencv.hpp>
#endif

class FingertipPoseEstimation
{
public:
    FingertipPoseEstimation(void);
    ~FingertipPoseEstimation(void);

    bool Initialize( IplImage * srcImage, char * calibFilename );
    void Terminate();

    void OnCapture( IplImage * frame, int64 nTickCount, IplImage * gray = 0 );
    void OnProcess();
    IplImage * OnDisplay( IplImage * image = 0 );

    IplImage * QueryHandImage();

private:

    //
    // Images
    //
    IplImage *  _pImage;
    IplImage *  _pGray;
    IplImage *  _pSkinColorImage;

    CvFont      font;

    //
    // Process Modules
    //
    HandRegion          _HandRegion;

    //
    // Hand Center
    //
    CvPoint _PrevCentroid;
    CvPoint _CurrCentroid;
};

#endif // _FINGERTIP_POSE_ESTIMATION_H_
