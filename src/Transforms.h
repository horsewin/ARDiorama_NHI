#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include <XnOS.h>
#include <XnCppWrapper.h>

using namespace xn;

CvMat *kinectParams, *kinectDistort;
CvMat *kinectTransform =0;
CvMat *kinectHomo;
Registration *kinectReg;

CvSize markerSize;

DepthGenerator g_depth;
ImageGenerator g_image;

CvPoint2D32f marker_origin;
float WORLD_SCALE = 1;
float WORLD_ANGLE = 0;
float MARKER_DEPTH = 0;
float WORLD_ORIGIN_X = 0;
float WORLD_ORIGIN_Y = 0;
bool UseAverageDepth = false;
//Function Prototypes
float FindMarkerAffineRotation(CvPoint3D32f *markCorn);

bool calcKinectOpenGLTransform(IplImage *colourIm, IplImage* depthIm, CvMat** transform) {
	bool found =false;
	vector<MarkerTransform> mt = kinectReg->performRegistration(colourIm, kinectParams, kinectDistort);
	if (mt.size()>0) {
		//Find the position of the corners on the image
		CvPoint2D32f *markerCorners = (CvPoint2D32f *)malloc(4*sizeof(CvPoint2D32f));
		markerCorners[0] = cvPoint2D32f(0,0); markerCorners[1] = cvPoint2D32f(mt.at(0).marker.size.width,0); 
		markerCorners[2] = cvPoint2D32f(mt.at(0).marker.size.width,mt.at(0).marker.size.height); markerCorners[3] = cvPoint2D32f(0,mt.at(0).marker.size.height);

		CvMat mCorners = cvMat(4,1,CV_32FC2, markerCorners);
		cvPerspectiveTransform(&mCorners, &mCorners, mt.at(0).homography);

		for (int i=0; i<4; i++) {
			if (markerCorners[i].x<0 || markerCorners[i].x>depthIm->width || markerCorners[i].y<0 || markerCorners[i].y>depthIm->height) {
				for (unsigned int i=0; i<mt.size(); i++) mt.at(i).clear(); mt.clear();
				free(markerCorners);
				return false;
			}
		}

		//Find the position of the corners in the real world wrt kinect
		XnPoint3D xnCorner[4], xnNewCorner[4];
		CvPoint3D32f markCorn[4];

		for (int i=0; i<4; i++) {
			markCorn[i] = cvPoint3D32f(markerCorners[i].x, markerCorners[i].y, CV_IMAGE_ELEM(depthIm, unsigned short, (int)markerCorners[i].y, (int)markerCorners[i].x));
			xnCorner[i].X = markCorn[i].x; xnCorner[i].Y = markCorn[i].y; xnCorner[i].Z = markCorn[i].z;
		}
		g_depth.ConvertProjectiveToRealWorld(4, xnCorner, xnNewCorner);

		//Calculate width and height of marker in real world
		float width1 = sqrt((xnNewCorner[0].X - xnNewCorner[1].X)*(xnNewCorner[0].X - xnNewCorner[1].X) + (xnNewCorner[0].Y - xnNewCorner[1].Y)*(xnNewCorner[0].Y - xnNewCorner[1].Y) + (xnNewCorner[0].Z - xnNewCorner[1].Z)*(xnNewCorner[0].Z - xnNewCorner[1].Z));
		float width2 = sqrt((xnNewCorner[3].X - xnNewCorner[2].X)*(xnNewCorner[3].X - xnNewCorner[2].X) + (xnNewCorner[3].Y - xnNewCorner[2].Y)*(xnNewCorner[3].Y - xnNewCorner[2].Y) + (xnNewCorner[3].Z - xnNewCorner[2].Z)*(xnNewCorner[3].Z - xnNewCorner[2].Z));
		float height1 = sqrt((xnNewCorner[3].X - xnNewCorner[0].X)*(xnNewCorner[3].X - xnNewCorner[0].X) + (xnNewCorner[3].Y - xnNewCorner[0].Y)*(xnNewCorner[3].Y - xnNewCorner[0].Y) + (xnNewCorner[3].Z - xnNewCorner[0].Z)*(xnNewCorner[3].Z - xnNewCorner[0].Z));
		float height2 = sqrt((xnNewCorner[2].X - xnNewCorner[1].X)*(xnNewCorner[2].X - xnNewCorner[1].X) + (xnNewCorner[2].Y - xnNewCorner[1].Y)*(xnNewCorner[2].Y - xnNewCorner[1].Y) + (xnNewCorner[2].Z - xnNewCorner[1].Z)*(xnNewCorner[2].Z - xnNewCorner[1].Z));
		//markerSize.width = (width1+width2)/2.0; markerSize.height = markerSize.width * ((float)mt.at(0).marker.size.height/(float)mt.at(0).marker.size.width);
		markerSize.height = (height1+height2)/2.0; markerSize.width = markerSize.height * ((float)mt.at(0).marker.size.width/(float)mt.at(0).marker.size.height);
		printf("Marker Size %dx%d\n", markerSize.width, markerSize.height);
		//if(USE_TRIMESH_GROUND) {
			//WORLD_SCALE = (xnCorner[1].X - xnCorner[0].X)*1.6/640;// (x*2/640=x/320)
			//WORLD_SCALE = 4.2;
		//}
		//printf("scale = %f \n", WORLD_SCALE);
		//WORLD_SCALE = Image_Width/ ((width1+width2)/2.0);
		//float height = WORLD_SCALE*(height1+height2)/2.0;
		//markerSize.width = (int) Image_Width;
		//markerSize.height = (int) height;
		//markerSize.width = (int) (width1+width2)/2.0;
		//markerSize.height = (int) (height1+height2)/2.0;
		//printf("Marker Size %dx%d and scale = %.2f \n", markerSize.width, markerSize.height, WORLD_SCALE);

		//Render the marker corners
		IplImage *kinectMarker = cvCloneImage(colourIm);
		for (int i=0; i<4; i++) cvCircle(kinectMarker, cvPoint(markerCorners[i].x, markerCorners[i].y), 3, cvScalar(0,0,255), -1);
		cvShowImage("Kinect Found Marker", kinectMarker);
		cvReleaseImage(&kinectMarker);

		free(markerCorners);
		//Calculate Kinect to OpenGL Transform
		{
			vector <CvPoint3D32f> srcPoints3D, dstPoints3D; vector <CvPoint2D32f> srcPoints2D, dstPoints2D;
			srcPoints2D.resize(50); srcPoints3D.resize(50); dstPoints2D.resize(50); dstPoints3D.resize(50);
			float xStep = float(markerSize.width)/9.0; float yStep = float(markerSize.height)/4.0;
			float xStep1 = float(mt.at(0).marker.size.width)/9.0; float yStep1 = float(mt.at(0).marker.size.height)/4.0;
			for (int y=0; y<5; y++) {
				for (int x=0; x<10; x++) {
					int index = x+(y*10);
					srcPoints3D.at(index) = cvPoint3D32f(x*xStep, y*yStep, 0);
					srcPoints2D.at(index) = cvPoint2D32f(x*xStep1, y*yStep1);
				}
			}
			
			CvMat mSrcCorners = cvMat(50,1,CV_32FC2, &srcPoints2D[0]); CvMat mDstCorners = cvMat(50,1,CV_32FC2, &dstPoints2D[0]);
			cvPerspectiveTransform(&mSrcCorners, &mDstCorners, mt.at(0).homography);

			XnPoint3D _xnCorner[50], _xnNewCorner[50]; 
			for (int i=0; i<50; i++) {_xnCorner[i].X = dstPoints2D[i].x; _xnCorner[i].Y = dstPoints2D[i].y; _xnCorner[i].Z = CV_IMAGE_ELEM(depthIm, unsigned short, (int)_xnCorner[i].Y, (int)_xnCorner[i].X);}
			g_depth.ConvertProjectiveToRealWorld(50, _xnCorner, _xnNewCorner);
			for (int i=0; i<50; i++) {dstPoints3D[i] = cvPoint3D32f(_xnNewCorner[i].X, _xnNewCorner[i].Y, _xnNewCorner[i].Z);}

			*transform = findTransform(dstPoints3D, srcPoints3D);
			kinectHomo = cvCloneMat(mt.at(0).homography);

			for (int y=0; y<4; y++) {
				for (int x=0; x<4; x++) {
					printf("%.2f\t", CV_MAT_ELEM((**transform), float, y,x));
				}
				printf("\n");
			}

			marker_origin = cvPoint2D32f(xnCorner[0].X, xnCorner[0].Y);
			printf("Marker origin %.2f , %.2f\n", marker_origin.x, marker_origin.y);
			
		}
		WORLD_ANGLE = FindMarkerAffineRotation(markCorn);
		if(UseAverageDepth) {
			float sum=0;
			for(int i = 0; i < 4; i++) sum += markCorn[i].z;
			MARKER_DEPTH = sum/4;
		} else {
			float min = markCorn[0].z;
			for(int i = 1; i < 4; i++) {
				if(min > markCorn[i].z)
					min =  markCorn[i].z;
			}
			MARKER_DEPTH = min;
		}
		float pixel_width = (xnCorner[1].X- xnCorner[0].X)/4;
		//WORLD_SCALE = (floorf(markerSize.width*10/pixel_width) /100)-0.01;
		WORLD_SCALE = floorf(markerSize.width*10/pixel_width) /100;//cm_per_pixel
		printf("width = %.4f,  WORLD_SCALE= %.4f \n", pixel_width, WORLD_SCALE);

		found = true;
	}
	
	for (unsigned int i=0; i<mt.size(); i++) mt.at(i).clear(); mt.clear();
	return found;
}


float FindMarkerAffineRotation(CvPoint3D32f *markCorn) {	
	//markCorn
	float x,y,magnitude,val,angle;
	float cornerX[4], cornerY[4];
	cornerX[0] = markCorn[0].x; cornerY[0] = markCorn[0].y;
	cornerX[1] = markCorn[1].x; cornerY[1] = markCorn[1].y;
	x = cornerX[1]-cornerX[0];
	y = cornerY[1]-cornerY[0];
	magnitude = sqrt((float)x*x + y*y);
	val = x/magnitude;
	angle = acos(val)*180/CV_PI;
	if(y < 0)
		angle = -angle;
	return angle;
}

#endif