#ifndef MAIN_H_
#define MAIN_H_

#include "HandRegion.h"
#include <time.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#define KINECT_CONFIG_FILENAME "../KinectConfig.xml"

int64   _PrevTickCount;

// Camera capture
CvCapture * gCapture = 0;

//
// Images
//
IplImage * image = 0;
IplImage * gray = 0;
IplImage * hand_region;

HandRegion _HandRegion;

#endif