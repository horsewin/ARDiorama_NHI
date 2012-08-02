//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <XnOpenNI.h>
#include <XnCodecIDs.h>

#ifndef OPENCV
#define OPENCV
	#include <opencv/cv.h>
	#include <opencv/highgui.h>
	#include <opencv2/opencv.hpp>
#endif

#include <map>
#include <vector>
#include <iostream>
#include <stdexcept>

#include "System.h"

using namespace std;

//---------------------------------------------------------------------------
// Constant
//---------------------------------------------------------------------------
int width  = 640;
int height = 480;

//---------------------------------------------------------------------------
// Method
//---------------------------------------------------------------------------
void CleanupExit( void );

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
xn::Context g_Context;
xn::ScriptNode g_scriptNode;
xn::DepthGenerator g_DepthGenerator;
xn::ImageGenerator g_ImageGenerator;
xn::UserGenerator g_UserGenerator;
xn::SceneMetaData g_SceneMD;
xn::DepthMetaData g_DepthMD;
xn::ImageMetaData g_ImageMD;
//xn::GestureGenerator g_GestureGenerator;
xn::Player g_Player;

XnBool g_bNeedPose = FALSE;
XnChar g_strPose[20] = "";
XnBool g_bDrawBackground = TRUE;
XnBool g_bDrawPixels = TRUE;
XnBool g_bDrawSkeleton = TRUE;
XnBool g_bPrintID = TRUE;
XnBool g_bPrintState = TRUE;

XnBool g_bPause = false;
XnBool g_bRecord = false;
XnBool g_bQuit = false;

std::map<XnUInt32, std::pair<XnCalibrationStatus, XnPoseDetectionStatus> > m_Errors;

System g_System;			// Main part

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
int main(int argc, char **argv)
{
	try{
		if( g_System.Init( argc, argv) != 0){
			exit(1);
		}
		
		g_System.Run();

	}catch (std::exception& ex) {
		std::cout << "!! Failed to run system; got exception." << std::endl;
		std::cout << "   Exception was :" << ex.what() << std::endl;			
	}
	return 0;
}


void CleanupExit()
{
	g_scriptNode.Release();
	g_DepthGenerator.Release();
	g_UserGenerator.Release();
	g_Player.Release();
	g_Context.Release();

	exit (1);
}
