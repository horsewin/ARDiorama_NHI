/*
 * System.h
 *
 * Created on : 10/7/2011
 * Author : Atsushi UMAKATSU
 *
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#ifndef XN_WRAPPER_H_
#define XN_WRAPPER_H_
	#include <XnCppWrapper.h>
#endif
#include <time.h>

class FlowCapture;

//---------------------------------------------------------------------------
// Class Definition
//---------------------------------------------------------------------------
class System{
public:	
	System( void );
	~System( void );

	void Run();
	int  Init(int argc, char **argv);

private:
	FlowCapture * m_opt_flow;

};

//void XN_CALLBACK_TYPE GestureProgress(xn::GestureGenerator& gesture, const XnChar* strGesture, const XnPoint3D* pPosition, XnFloat fProgress, void* pCookie);
//void XN_CALLBACK_TYPE GestureRecognized(xn::GestureGenerator& gesture, const XnChar* strGesture, const XnPoint3D* pIDPosition, const XnPoint3D* pEndPosition, void* pCookie);
//void XN_CALLBACK_TYPE GestureChange(xn::ProductionNode& node, void* pCookie);
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie);
void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(xn::SkeletonCapability& capability, XnUserID nId, XnCalibrationStatus eStatus, void* pCookie);
void XN_CALLBACK_TYPE MyCalibrationInProgress(xn::SkeletonCapability& capability, XnUserID id, XnCalibrationStatus calibrationError, void* pCookie);
void XN_CALLBACK_TYPE MyPoseInProgress(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID id, XnPoseDetectionStatus poseError, void* pCookie);

#endif