/*
 * System.cpp
 *
 * Created on : 10/7/2011
 * Author : Atsushi UMAKATSU
 *
 */

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "System.h"

#include "FlowCapture.h"

#ifndef OPENCV
#define OPENCV
	#include <opencv/cv.h>
	#include <opencv/highgui.h>
	#include <opencv2/opencv.hpp>
#endif

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}

//---------------------------------------------------------------------------
// Constant
//---------------------------------------------------------------------------
extern const int width;
extern const int height;
static const char* SAMPLE_XML_PATH = "xml/SamplesConfig.xml";
static const char* XN_CALIBRATION_FILE_NAME = "UserCalibration.bin";
static const char* CV_CASCADE_FACE = "xml/haarcascade_frontalface_default.xml";

//---------------------------------------------------------------------------
// Global variable
//---------------------------------------------------------------------------
extern xn::Context g_Context;
extern xn::ScriptNode g_scriptNode;
extern xn::DepthGenerator g_DepthGenerator;
extern xn::ImageGenerator g_ImageGenerator;
extern xn::UserGenerator g_UserGenerator;

extern XnBool g_bNeedPose;
extern XnChar g_strPose[20];
extern XnBool g_bDrawBackground;
extern XnBool g_bDrawPixels;
extern XnBool g_bDrawSkeleton;
extern XnBool g_bPrintID;
extern XnBool g_bPrintState;
extern XnBool g_bTrackFace;

extern XnBool g_bPause;
extern XnBool g_bRecord;
extern XnBool g_bQuit;

extern xn::Player g_Player;

extern std::map<XnUInt32, std::pair<XnCalibrationStatus, XnPoseDetectionStatus> > m_Errors;

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

using namespace std;

System::System( void )
{
	m_opt_flow = new FlowCapture();
}

System::~System( void )
{
	delete m_opt_flow;
}

//void System::Run( const xn::DepthMetaData& dmd, const xn::SceneMetaData& smd ){
void System::Run( )
{
	m_opt_flow->Init();
	while(1){
		// if you want to detect user by kinect, have to valid this
		//g_Context.WaitOneUpdateAll(g_DepthGenerator);

		// Acquisition of Depth and Image data
		xn::SceneMetaData g_SceneMD;
		xn::DepthMetaData g_DepthMD;
		xn::ImageMetaData g_imageMD;
		g_DepthGenerator.GetMetaData(g_DepthMD);
		g_UserGenerator.GetUserPixels(0, g_SceneMD);	
		g_ImageGenerator.GetMetaData(g_imageMD);

		// fitting depth image to colour image coordinate 
		//g_DepthGenerator.GetAlternativeViewPointCap().SetViewPoint(g_ImageGenerator);

		// calculate variational optical flow
		m_opt_flow->Run(g_DepthMD, g_SceneMD, g_imageMD);
	}
}


int System::Init(int argc, char **argv)
{
	XnStatus nRetVal = XN_STATUS_OK;
	if (argc > 1)
	{
		nRetVal = g_Context.Init();
		CHECK_RC(nRetVal, "Init");
		nRetVal = g_Context.OpenFileRecording(argv[1], g_Player);
		if (nRetVal != XN_STATUS_OK)
		{
			printf("Can't open recording %s: %s\n", argv[1], xnGetStatusString(nRetVal));
			return 1;
		}
	}
	else
	{
		xn::EnumerationErrors errors;
		nRetVal = g_Context.InitFromXmlFile(SAMPLE_XML_PATH, g_scriptNode, &errors);
		if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
		{
			XnChar strError[1024];
			errors.ToString(strError, 1024);
			printf("%s\n", strError);
			return (nRetVal);
		}
		else if (nRetVal != XN_STATUS_OK)
		{
			printf("Open failed: %s\n", xnGetStatusString(nRetVal));
			return (nRetVal);
		}
	}

	// Create Depth generator
	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	CHECK_RC(nRetVal, "Find depth generator");

	// Create Image generator
	g_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_ImageGenerator);
	CHECK_RC(nRetVal, "Find image generator");

	// Create User generator
	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK)
	{
		nRetVal = g_UserGenerator.Create(g_Context);
		CHECK_RC(nRetVal, "Find user generator");
	}
	//nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_GESTURE, g_GestureGenerator);

	// Fitting depth image to color image
	g_DepthGenerator.GetAlternativeViewPointCap().SetViewPoint(g_ImageGenerator);

	//// create callback handles
	//XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete, hPoseDetected, hCalibrationInProgress, hPoseInProgress;
	//				 //hGestureCallbacks, hGestureChange;
	//if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
	//{
	//	printf("Supplied user generator doesn't support skeleton\n");
	//	return 1;
	//}

	//// register callbacks
	//nRetVal = g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
	//CHECK_RC(nRetVal, "Register to user callbacks");
	//nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationStart(UserCalibration_CalibrationStart, NULL, hCalibrationStart);
	//CHECK_RC(nRetVal, "Register to calibration start");
	//nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, NULL, hCalibrationComplete);
	//CHECK_RC(nRetVal, "Register to calibration complete");

	//if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
	//{
	//	g_bNeedPose = TRUE;
	//	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
	//	{
	//		printf("Pose required, but not supported\n");
	//		return 1;
	//	}
	//	nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected, NULL, hPoseDetected);
	//	CHECK_RC(nRetVal, "Register to Pose Detected");
	//	g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	//}

	//g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	//nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationInProgress(MyCalibrationInProgress, NULL, hCalibrationInProgress);
	//CHECK_RC(nRetVal, "Register to calibration in progress");

	//nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseInProgress(MyPoseInProgress, NULL, hPoseInProgress);
	//CHECK_RC(nRetVal, "Register to pose in progress");

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	// mirroring 1
	g_Context.SetGlobalMirror(!g_Context.GetGlobalMirror());

	std::cout << "Finished Initializing : Kinect " << std::endl;
	return 0;
}
	
// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	printf("New User %d\n", nId);
	// New user found
	if (g_bNeedPose)
	{
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	}
	else
	{
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}
// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	printf("Lost user %d\n", nId);
}
// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
	printf("Pose %s detected for user %d\n", strPose, nId);
	g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
	g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}
// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
	printf("Calibration started for user %d\n", nId);
}
// Callback: Finished calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie)
{
	if (bSuccess)
	{
		// Calibration succeeded
		printf("Calibration complete, start tracking user %d\n", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else
	{
		// Calibration failed
		printf("Calibration failed for user %d\n", nId);
		if (g_bNeedPose)
		{
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		}
		else
		{
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
		}
	}
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(xn::SkeletonCapability& capability, XnUserID nId, XnCalibrationStatus eStatus, void* pCookie)
{
	if (eStatus == XN_CALIBRATION_STATUS_OK)
	{
		// Calibration succeeded
		printf("Calibration complete, start tracking user %d\n", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else
	{
		// Calibration failed
		printf("Calibration failed for user %d\n", nId);
		if (g_bNeedPose)
		{
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		}
		else
		{
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
		}
	}
}
void XN_CALLBACK_TYPE MyCalibrationInProgress(xn::SkeletonCapability& capability, XnUserID id, XnCalibrationStatus calibrationError, void* pCookie)
{
	m_Errors[id].first = calibrationError;
}
void XN_CALLBACK_TYPE MyPoseInProgress(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID id, XnPoseDetectionStatus poseError, void* pCookie)
{
	m_Errors[id].second = poseError;
}

//bool System::IsGazing(int userId)
//{
//	return g_FaceDetector->IsGazing(userId);
//}