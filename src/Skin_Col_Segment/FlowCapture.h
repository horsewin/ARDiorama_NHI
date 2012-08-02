/*
 * FlowCapture.h
 *
 * Created on : 11/14/2011
 * Author : Atsushi UMAKATSU
 *
 */

#ifndef FLOWCAPTURE_H_
#define FLOWCAPTURE_H_

#include "HandRegion.h"
//#include "FingertipPoseEstimation.h"
#include <time.h>

#ifndef OPENCV
#define OPENCV
	#include <opencv/cv.h>
	#include <opencv/highgui.h>
	#include <opencv2/opencv.hpp>
#endif

#ifndef XN_WRAPPER_H_
#define XN_WRAPPER_H_
	#include <XnCppWrapper.h>
#endif

class VarFlow;
class FlowCapture{
public:
	FlowCapture();
	~FlowCapture();

	void Init();
	void Run(IplImage* prev_img, IplImage* curr_img, IplImage* col_img );

private:
  IplImage* imgA;
  IplImage* imgB;

	IplImage* imgU;
	IplImage* imgV;
  
	IplImage* imgColor;
  IplImage* imgMotion;
  
  //We will start at level 0 (full size image) and go down to level 4 (coarse image 16 times smaller than original)
  //Experiment with these values to see how they affect the flow field as well as calculation time
  int max_level;
  int start_level;
  
  //Two pre and post smoothing steps, should be greater than zero
  int n1;
  int n2;
  
  //Smoothing and regularization parameters, experiment but keep them above zero
  float rho;
  float alpha;
  float sigma;

	VarFlow * optical_flow;

	HandRegion _HandRegion;
//	FingertipPoseEstimation g_fingertip_pose_estimation;

  // create font for displaying fps
  CvFont font;
};

#endif