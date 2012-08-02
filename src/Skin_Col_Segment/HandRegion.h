#ifndef _HAND_REGION_H_
#define _HAND_REGION_H_

#ifndef OPENCV
#define OPENCV
	#include <opencv/cv.h>
	#include <opencv/highgui.h>
	//#include <opencv2/opencv.hpp>
#endif
#include "GMM.h"

#define ALPHA   0.9

class HandRegion
{
public:
    HandRegion(void);
    ~HandRegion(void);

    bool LoadSkinColorProbTable();

    IplImage * GetHandRegion( IplImage * srcImage, int *cont_num, std::vector <CvRect> & cont_boundbox,  std::vector <CvBox2D> & cont_boundbox2D, std::vector <CvPoint> & cont_center);

    IplImage * QueryHandRegion() { return _pImage; }

private:

    GMM     _SkinColor;
    GMM     _NonSkinColor;

    IplImage *      _pImage;            // Hand Region Image ( binary )
};

#endif // _HAND_REGION_H_
