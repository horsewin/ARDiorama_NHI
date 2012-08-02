#ifndef KINECTCALIB

//#define USE_ARMM_VRPN 1
#define USE_SKIN_SEGMENTATION 1
#define USE_OPTICAL_FLOW 1 
//#define USE_PARTICLES 1

#include "main.h"

#include <XnOS.h>
#include <XnCppWrapper.h>
//OpenCV
#include "opencv\cv.h"
#include "opencv\highgui.h"
//OPIRA
#include "CaptureLibrary.h"
#include "OPIRALibrary.h"
#include "OPIRALibraryMT.h"
#include "RegistrationAlgorithms/OCVSurf.h"
//Bullet
#include "KCRPhysicsWorld.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"
//Graphics calls
#include "osg.h"
#include "leastsquaresquat.h"

//Transforms
#include "transforms.h"

//Controller Input
#include "Controls/KeyboardControls.h"
#include "Controls/XBoxControls.h"

#ifdef USE_ARMM_VRPN
#include "ARMM_vrpn.h"
#endif

#ifdef USE_SKIN_SEGMENTATION
#include "HandRegion.h"
#include "FlowCapture.h"
#endif

using namespace std; 
using namespace xn;

class KCRPhysicsWorld *m_world;
KeyboardController *kc;
XboxController *xc;
// OpenNI Global
Context niContext;
DepthMetaData niDepthMD;
ImageMetaData niImageMD;

bool running = true;
bool loadKinectParams(char *filename, CvMat **params, CvMat **distortion);
void loadKinectTransform(char *filename);
osg::Node* osgNodeFromBtCollisionShape( const btConvexHullShape* hull, const btTransform& trans );
void RenderScene(IplImage *arImage, Capture *capture);
osg::Node* createVArrayFromHField(const btTransform& trans);
void GenerateTrimeshGroundFromDepth(IplImage* depthIm, float markerDepth);
osg::Node* createVArrayFromDepth();
void inpaintDepth(DepthMetaData *niDepthMD, bool halfSize);
float FindMarkerAffineRotation();
void setWorldOrigin();
void SceneOpticalFlowLK(IplImage* prevImg, IplImage* currImg);
void SegmentHandRegion(IplImage* color_src);
void TransformImage(IplImage* depthIm, IplImage* ResDepth, float markerDepth, CvSize img_size, bool isDepth);
void UpdateHandRegions();
void removeNoise( IplImage* src, int size );

Capture *capture;
CvMat *RegistrationParams;

int counter = 5;
float *ground_grid, *voxel_grid;
float MaxHeight, MinHeight;

IplImage *colourIm, *depthIm, *prev_gray, *curr_gray ,*transDepth160, *transDepth320, *transColor320;

osg::Quat CarsOrientation[NUM_CAR];
osg::Quat WheelsOrientaion[NUM_CAR][NUM_WHEEL];
osg::Vec3d CarsPosition[NUM_CAR];
osg::Vec3d WheelsPosition[NUM_CAR][NUM_WHEEL];

#ifdef USE_ARMM_VRPN
vrpn_Connection_IP* m_Connection;
ARMM_Communicator* ARMM_server;
#endif

#ifdef USE_SKIN_SEGMENTATION
HandRegion _HandRegion;
IplImage *gray, *hand_region;
#endif

//Optical Flow
#ifdef USE_OPTICAL_FLOW
int OPT_X_STEP = 20;
int OPT_Y_STEP = 20;
int LK_SIZE = 15;
bool RunOnce = false;
#endif

inline CvMat* scaleParams(CvMat *cParams, double scaleFactor) {
	CvMat *sParams = cvCloneMat(cParams);
	sParams->data.db[0]*= scaleFactor;	sParams->data.db[4]*= scaleFactor;
	sParams->data.db[2]*= scaleFactor;	sParams->data.db[5]*= scaleFactor;
	return sParams;
}

int main(int argc, char* argv[]) {

	markerSize.width = -1; markerSize.height = -1;
	EnumerationErrors errors;
	switch (XnStatus rc = niContext.InitFromXmlFile(KINECT_CONFIG_FILENAME, &errors)) {
		case XN_STATUS_OK:
			break;
		case XN_STATUS_NO_NODE_PRESENT:
			XnChar strError[1024];	errors.ToString(strError, 1024);
			printf("%s\n", strError);
			return rc; break;
		default:
			printf("Open failed: %s\n", xnGetStatusString(rc));
			return rc;
	}

	capture = new Camera(CAPTURE_SIZE, CAMERA_PARAMS_FILENAME);

	RegistrationParams = scaleParams(capture->getParameters(), double(REGISTRATION_SIZE.width)/double(CAPTURE_SIZE.width));
	osg_init(calcProjection(RegistrationParams, capture->getDistortion(), REGISTRATION_SIZE));

	loadKinectParams(KINECT_PARAMS_FILENAME, &kinectParams, &kinectDistort);
	kinectDistort =0;
	kinectParams->data.db[2]=320.0; kinectParams->data.db[5]=240.0;

	niContext.FindExistingNode(XN_NODE_TYPE_DEPTH, g_depth);
	niContext.FindExistingNode(XN_NODE_TYPE_IMAGE, g_image);

	g_depth.GetMirrorCap().SetMirror(false);
	g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);

	kinectReg = new RegistrationOPIRA(new OCVSurf());
	kinectReg->addResizedMarker(MARKER_FILENAME, 400);

	//physics
	m_world = new KCRPhysicsWorld();
	ground_grid = new float[19200];
	for (int i =0;i < 19200; i++) {
		ground_grid[i] = 0; 
	}
#ifdef SIM_PARTICLES
	voxel_grid = new float[1200];
	for (int i =0;i < 1200; i++) {
		voxel_grid[i] = 0;
	}
#endif

	//controls
	KeyboardController *kc = new KeyboardController(m_world);
	XboxController *xc = new XboxController(m_world);

	loadKinectTransform(KINECT_TRANSFORM_FILENAME);

#ifdef USE_ARMM_VRPN
	m_Connection = new vrpn_Connection_IP();
	ARMM_server = new ARMM_Communicator(m_Connection );
    cout << "Created VRPN server." << endl;
#endif

#ifdef USE_SKIN_SEGMENTATION	//Skin color look up
	_HandRegion.LoadSkinColorProbTable();
#endif

#ifdef USE_OPTICAL_FLOW
	prev_gray = cvCreateImage(cvSize(OPFLOW_SIZE.width, OPFLOW_SIZE.height), IPL_DEPTH_8U, 1);
	curr_gray = cvCreateImage(cvSize(OPFLOW_SIZE.width, OPFLOW_SIZE.height), IPL_DEPTH_8U, 1);
#endif
/////////////////////////////////////////////Main Loop////////////////////////////////////////////////
	FlowCapture * flow_capture = new FlowCapture();
	 flow_capture->Init();
	while (running) {
		if (XnStatus rc = niContext.WaitAnyUpdateAll() != XN_STATUS_OK) {
			printf("Read failed: %s\n", xnGetStatusString(rc));
			return rc;
		}
		g_depth.GetMetaData(niDepthMD);
		g_image.GetMetaData(niImageMD);

		colourIm = cvCreateImage(cvSize(niImageMD.XRes(), niImageMD.YRes()), IPL_DEPTH_8U, 3);
		memcpy(colourIm->imageData, niImageMD.Data(), colourIm->imageSize); cvCvtColor(colourIm, colourIm, CV_RGB2BGR);
		cvFlip(colourIm, colourIm, 1);

		depthIm = cvCreateImage(cvSize(niDepthMD.XRes(), niDepthMD.YRes()), IPL_DEPTH_16U, 1);
		transDepth160 = cvCreateImage(cvSize(MESH_SIZE.width, MESH_SIZE.height), IPL_DEPTH_32F, 1);
		transDepth320 = cvCreateImage(cvSize(CV_OP_SIZE.width, CV_OP_SIZE.height), IPL_DEPTH_32F, 1);
		transColor320 = cvCreateImage(cvSize(CV_OP_SIZE.width, CV_OP_SIZE.height), IPL_DEPTH_8U, 3);
		memcpy(depthIm->imageData, niDepthMD.Data(), depthIm->imageSize);	
		cvShowImage("Kinect View", colourIm);

		IplImage *arImage = capture->getFrame();
		cvWaitKey(1); 
		kc->check_input(); xc->check_input();

#ifdef USE_OPTICAL_FLOW
//		if(RunOnce) SceneOpticalFlowLK(prev_img, colourIm);
#endif

		if(kinectTransform) { // kinect transform as cvmat* for use
			if( counter >= 4) {
				inpaintDepth(&niDepthMD, true); 
				memcpy(depthIm->imageData, niDepthMD.Data(), depthIm->imageSize);				
				TransformImage(depthIm, transDepth160, MARKER_DEPTH, MESH_SIZE, true);
				GenerateTrimeshGroundFromDepth(transDepth160, MARKER_DEPTH); /*Trimesh generation*/
				m_world->updateTrimeshRefitTree(ground_grid);//opencl?
				osg_UpdateHeightfieldTrimesh(ground_grid);//opencl?
#ifdef SIM_PARTICLES
/*World spheres simulation*/
//				GenerateVoxelFromDepth(depthIm, MARKER_DEPTH);
//				m_world->updateWorldSphereTransform(voxel_grid);
//				osgUpdateWorldSphereTransform(voxel_grid);
#endif
				counter = 0;
			} else {
#ifdef USE_SKIN_SEGMENTATION /*Skin color segmentation*/ // may be reduce resolution first as well as cut off depth make processing faster
//				inpaintDepth(&niDepthMD, true); 
//				memcpy(depthIm->imageData, niDepthMD.Data(), depthIm->imageSize);

				//Transform both depth and color
				TransformImage(depthIm, transDepth320, MARKER_DEPTH, CV_OP_SIZE, true);
				TransformImage(colourIm, transColor320, MARKER_DEPTH, CV_OP_SIZE, false);

//				IplImage* depth32_640 = cvCreateImage(cvSize(depthIm->width, depthIm->height), IPL_DEPTH_32F, 1);
//				IplImage* depthTmp640 = cvCreateImage(cvSize(depthIm->width, depthIm->height), IPL_DEPTH_8U, 1);

				IplImage* depthTmp = cvCreateImage(cvSize(CV_OP_SIZE.width, CV_OP_SIZE.height), IPL_DEPTH_8U, 1);
				IplImage* colourImResized = cvCreateImage(cvSize(CV_OP_SIZE.width, CV_OP_SIZE.height), IPL_DEPTH_8U, 3);
				gray = cvCreateImage(cvSize(colourImResized->width, colourImResized->height),IPL_DEPTH_8U,1);
				hand_region = cvCreateImage(cvSize(colourImResized->width, colourImResized->height),IPL_DEPTH_8U,1);
				IplImage* colourIm640 = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);

//				cvConvertScale(depthIm,depth32_640,1);
//				cvThreshold(depth32_640, depthTmp640, MARKER_DEPTH+5, 255, CV_THRESH_BINARY_INV); //thres at 1cm above marker
//				cvShowImage("xxx",depthTmp640);
//				cvCmpS(transDepth, 0, depthTmp, CV_CMP_LT);//dst must b 8U
				cvThreshold(transDepth320, depthTmp, 1, 255, CV_THRESH_BINARY_INV); //thres at 1cm above marker
//				cvResize(colourIm, colourImResized, CV_INTER_NN);//use nearest neighbor interpolation
//				removeNoise( depthTmp, 100 );

				cvResize(transColor320, colourImResized, CV_INTER_NN);
				cvSet(colourImResized, cvScalar(0), depthTmp);
//				cvShowImage ("Marker Thresh", colourImResized);
				cvResize(colourImResized, colourIm640,CV_INTER_NN);
				cvShowImage ("Marker Thresh 640", colourIm640);

				cvCopyImage( _HandRegion.GetHandRegion( colourImResized, gray), hand_region );
//				removeNoise( hand_region, 20 );
				cvThreshold(hand_region, depthTmp, 0, 255, CV_THRESH_BINARY_INV);
//				removeNoise( depthTmp, 100 );
//				cvShowImage ("depthTmp", depthTmp);
//				cvShowImage ("hand_region", hand_region);

				cvSet(colourImResized, cvScalar(0), depthTmp);

//				cvShowImage ("Skin Color", colourImResized);

//				cvDilate(colourImResized,colourImResized,CV_SHAPE_RECT,1);
//				cvErode(colourImResized,colourImResized,CV_SHAPE_RECT,1);
//				cvMorphologyEx(colourImResized,colourImResized,NULL,CV_SHAPE_RECT,CV_MOP_OPEN,1);
//				cvResize(colourImResized, colourIm640,CV_INTER_NN);
//				cvShowImage ("Color Skin Color 640", colourIm640);


				IplImage* colourIm160 = cvCreateImage(cvSize(OPFLOW_SIZE.width, OPFLOW_SIZE.height), IPL_DEPTH_8U, 3);
//				cvResize(colourIm, colourIm160, 1);
				cvCopyImage(colourImResized,colourIm160);

				cvCvtColor(colourIm160, curr_gray, CV_BGR2GRAY);
				if(RunOnce)
					flow_capture->Run(prev_gray, curr_gray, colourIm160);
				IplImage* col_640 = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
				cvResize(colourIm160, col_640, 1);
				cvShowImage("Op_Flow_640",col_640);
				cvReleaseImage(&colourIm160);
				cvReleaseImage(&col_640);

				cvReleaseImage(&depthTmp);
				cvReleaseImage(&colourImResized);
				cvReleaseImage(&colourIm640);

#endif

#ifdef USE_PARTICLES
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#endif
				counter++;
			}
			//do hand pose recognition
			m_world->Update();
			RenderScene(arImage, capture);
		}
#ifdef USE_ARMM_VRPN
		ARMM_server->mainloop();
		m_Connection->mainloop();
#endif

#ifdef USE_OPTICAL_FLOW
		if(!RunOnce) RunOnce = true;
		cvCopyImage(curr_gray, prev_gray);
#endif

		cvReleaseImage(&arImage);
		cvReleaseImage(&depthIm); cvReleaseImage(&colourIm);
		cvReleaseImage(&transDepth320);cvReleaseImage(&transDepth160);
		cvReleaseImage(&transColor320);
#ifdef USE_SKIN_SEGMENTATION
		cvReleaseImage(&gray); cvReleaseImage(&hand_region);
#endif
	}

	cvReleaseImage(&prev_gray); cvReleaseImage(&curr_gray);
	osg_uninit();
	delete m_world;
	delete kinectReg;

	cvReleaseMat(&RegistrationParams);

	delete kc;

	return 0;
}

void RenderScene(IplImage *arImage, Capture *capture) {
		float scale = 10;
#ifdef SIM_MICROMACHINE
		for(int i = 0; i < NUM_CAR; i++) {
			btTransform trans = m_world->getCarPose(i);
			btQuaternion quat = trans.getRotation();
			CarsOrientation[i] = osg::Quat(quat.getX(), quat.getY(), quat.getZ(), quat.getW()); 
			CarsPosition[i] = osg::Vec3d(trans.getOrigin().getX()*scale, trans.getOrigin().getY()*scale,trans.getOrigin().getZ()*scale);
			for(int j = 0; j < 4; j++) {
				btTransform trans_tmp = m_world->getWheelTransform(i,j);
				btQuaternion quattmp = trans_tmp.getRotation();
				WheelsOrientaion[i][j] = osg::Quat(quattmp.getX(), quattmp.getY(), quattmp.getZ(), quattmp.getW()); 	
				WheelsPosition[i][j] = osg::Vec3d(trans_tmp.getOrigin().getX()*scale, trans_tmp.getOrigin().getY()*scale,trans_tmp.getOrigin().getZ()*scale);
			}
		}
		if(Virtual_Objects_Count > 0) {
			std::vector <osg::Quat> quat_obj_array;
			std::vector <osg::Vec3d> vect_obj_array;
			for(int i = 0; i < Virtual_Objects_Count; i++) {
				btTransform trans2 = m_world->get_Object_Transform(i);
				btQuaternion quat2 = trans2.getRotation();
				quat_obj_array.push_back(osg::Quat(quat2.getX(), quat2.getY(), quat2.getZ(), quat2.getW())); 
				vect_obj_array.push_back(osg::Vec3d(trans2.getOrigin().getX()*scale, trans2.getOrigin().getY()*scale,trans2.getOrigin().getZ()*scale));		
			}
			osg_render(arImage, CarsOrientation, CarsPosition, WheelsOrientaion, WheelsPosition, RegistrationParams, capture->getDistortion(), quat_obj_array, vect_obj_array);
		} else {
			osg_render(arImage, CarsOrientation, CarsPosition, WheelsOrientaion, WheelsPosition, RegistrationParams, capture->getDistortion());
		}
#else
		osg_render(arImage, CarsOrientation, CarsPosition, WheelsOrientaion, WheelsPosition, RegistrationParams, capture->getDistortion());
#endif /*SIM_MICROMACHINE*/

}


bool loadKinectParams(char *filename, CvMat **params, CvMat **distortion) {
	CvFileStorage* fs = cvOpenFileStorage( filename, 0, CV_STORAGE_READ );
	if (fs==0) return false; 

	CvFileNode* fileparams;
	//Read the Camera Parameters
	fileparams = cvGetFileNodeByName( fs, NULL, "camera_matrix" );
	*params = (CvMat*)cvRead( fs, fileparams );

	//Read the Camera Distortion 
	fileparams = cvGetFileNodeByName( fs, NULL, "distortion_coefficients" );
	*distortion = (CvMat*)cvRead( fs, fileparams );
	cvReleaseFileStorage( &fs );

	return true;
}

void loadKinectTransform(char *filename) {
	CvFileStorage* fs = cvOpenFileStorage( filename, 0, CV_STORAGE_READ );
	if (fs!=0) {
		CvSeq *s = cvGetFileNodeByName(fs, 0, "MarkerSize")->data.seq;
		markerSize.width = cvReadInt((CvFileNode*)cvGetSeqElem(s, 0));
		markerSize.height = cvReadInt((CvFileNode*)cvGetSeqElem(s, 1));

		s = cvGetFileNodeByName(fs, 0, "MarkerOrigin")->data.seq;
		marker_origin.x = cvReadInt((CvFileNode*)cvGetSeqElem(s, 0));
		marker_origin.y = cvReadInt((CvFileNode*)cvGetSeqElem(s, 1));
		setWorldOrigin();
		WORLD_SCALE = cvReadRealByName(fs, 0, "WorldScale", 1);
		WORLD_ANGLE = cvReadRealByName(fs, 0, "WorldAngle", 0);
		MARKER_DEPTH = cvReadRealByName(fs, 0, "MARKER_DEPTH", 0);

		CvFileNode* fileparams = cvGetFileNodeByName( fs, NULL, "KinectTransform" );
		kinectTransform = (CvMat*)cvRead( fs, fileparams );
		cvReleaseFileStorage( &fs );

		if (niContext.WaitAnyUpdateAll() == XN_STATUS_OK) {
			//Load in the marker for registration
			osg_inittracker(MARKER_FILENAME, 400, markerSize.width);

			m_world->setWorldDepth(MARKER_DEPTH);
			m_world->setWorldScale(WORLD_SCALE);
			setOSGTrimeshScale(WORLD_SCALE);

			g_depth.GetMetaData(niDepthMD);
			inpaintDepth(&niDepthMD, true);
			depthIm = cvCreateImage(cvSize(niDepthMD.XRes(), niDepthMD.YRes()), IPL_DEPTH_16U, 1);
			transDepth160 = cvCreateImage(cvSize(MESH_SIZE.width, MESH_SIZE.height), IPL_DEPTH_32F, 1);
			memcpy(depthIm->imageData, niDepthMD.Data(), depthIm->imageSize);	

			TransformImage(depthIm, transDepth160, MARKER_DEPTH, MESH_SIZE, true);
			GenerateTrimeshGroundFromDepth(transDepth160, MARKER_DEPTH);
			m_world->updateTrimesh(ground_grid);
			m_world->setMinHeight(MinHeight);
			m_world->setMaxHeight(MaxHeight);
			m_world->initPhysics();
#ifdef SIM_PARTICLES
			CreateOSGSphereProxy();//osg spheres representation
#endif
#ifdef SIM_MICROMACHINE
			m_world->resetCarScene(0);
			m_world->resetCarScene(1);
#endif /*SIM_MICROMACHINE*/
		}
	}
}

void TransformImage(IplImage* src_img, IplImage* dst_img, float markerDepth, CvSize img_size, bool isDepth) {

	float ground_depth = (float)markerDepth/10;
	int Depth, Channels;
	if(isDepth) {
		Depth = IPL_DEPTH_32F;
		Channels = 1;
	} else {
		Depth = IPL_DEPTH_8U;
		Channels = 3;
	}
	IplImage *tmp_img = cvCreateImage(cvGetSize(src_img), Depth, Channels);//problem using cvthres with 64F
	IplImage *lowerResImg1 = cvCreateImage(cvSize(img_size.width,img_size.height), Depth, Channels);
	IplImage *lowerResImg2 = cvCreateImage(cvSize(img_size.width,img_size.height), Depth, Channels);
	if(isDepth) 
		cvConvertScale(src_img, tmp_img, .1);//convert img from 16U to 32F and mm to cm
	else
		cvCopyImage(src_img, tmp_img);
	cvResize(tmp_img, lowerResImg1, CV_INTER_NN);//use nearest neighbor interpolation
	if(isDepth) {
		cvSubRS(lowerResImg1, cvScalar(ground_depth), lowerResImg1);
		for (int i = 0; i < img_size.height; i++) {
			for (int j = 0; j < img_size.width; j++) {
				if (i < 5 || j < 3)
					CV_IMAGE_ELEM(lowerResImg1, float, i, j) = 0;
			}
		}
	}
	int scale =  (int) img_size.width/MESH_SIZE.width;
	cvSetZero(lowerResImg2);
	CvMat* rot_mat = cvCreateMat(2,3,CV_32FC1);
	cv2DRotationMatrix( cvPoint2D32f(WORLD_ORIGIN_X*scale,WORLD_ORIGIN_Y*scale), WORLD_ANGLE, 1, rot_mat );//correct back
	cvWarpAffine( lowerResImg1, lowerResImg2, rot_mat );
	if(isDepth && img_size.width == MESH_SIZE.width) cvShowImage("Depth Transformed", lowerResImg2); else cvShowImage("Color Transformed", lowerResImg2);
	cvCopyImage(lowerResImg2, dst_img);
	cvReleaseImage(&tmp_img);
	cvReleaseImage(&lowerResImg1);
	cvReleaseImage(&lowerResImg2);		

}


void GenerateTrimeshGroundFromDepth(IplImage* depthIm, float markerDepth) {

	float ground_depth = (float)markerDepth/10;
	MaxHeight = ground_depth-40;
	MinHeight = -MaxHeight;
	float val = 0;
	for (int i = 0; i < MESH_SIZE.width; i++) {
		for (int j = 0; j < MESH_SIZE.height; j++) {
			int index = j*MESH_SIZE.width+i;
			//val = CV_IMAGE_ELEM(lowerResDepth2, float, 119-j, i);
			val = CV_IMAGE_ELEM(depthIm, float, (MESH_SIZE.height-1)-j, i);
			if(val > MaxHeight)
				val = MaxHeight;
			else if(val < MinHeight)
				val = MinHeight;
				
			ground_grid[index] =  val;
		}
	}
}

void inpaintDepth(DepthMetaData *niDepthMD, bool halfSize) {
	IplImage *depthIm, *depthImFull;
	
	if (halfSize) {
		depthImFull = cvCreateImage(cvSize(niDepthMD->XRes(), niDepthMD->YRes()), IPL_DEPTH_16U, 1);
		depthImFull->imageData = (char*)niDepthMD->WritableData();
		depthIm = cvCreateImage(cvSize(depthImFull->width/4.0, depthImFull->height/4.0), IPL_DEPTH_16U, 1);
		cvResize(depthImFull, depthIm, 0);
	} else {
		depthIm = cvCreateImage(cvSize(niDepthMD->XRes(), niDepthMD->YRes()), IPL_DEPTH_16U, 1);
		depthIm->imageData = (char*)niDepthMD->WritableData();
	}
	
	IplImage *depthImMask = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_8U, 1);
	for (int y=0; y<depthIm->height; y++) {
		for (int x=0; x<depthIm->width; x++) {
			CV_IMAGE_ELEM(depthImMask, char, y, x)=CV_IMAGE_ELEM(depthIm, unsigned short,y,x)==0?255:0;
		}
	}

	IplImage *depthImMaskInv = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_8U, 1);
	cvNot(depthImMask, depthImMaskInv);

	double min, max; cvMinMaxLoc(depthIm, &min, &max, 0, 0, depthImMaskInv);
	
	IplImage *depthIm8 = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_8U, 1);
	float scale = 255.0/(max-min);
	cvConvertScale(depthIm, depthIm8, scale, -(min*scale));

	IplImage *depthPaint = cvCreateImage(cvGetSize(depthIm8), IPL_DEPTH_8U, 1);
	cvInpaint(depthIm8, depthImMask, depthPaint, 3, CV_INPAINT_NS);
	
	IplImage *depthIm16 = cvCreateImage(cvGetSize(depthIm), IPL_DEPTH_16U, 1);
	cvConvertScale(depthPaint, depthIm16, 1/scale, min);

	if (halfSize) {
		IplImage *depthPaintedFull = cvCreateImage(cvGetSize(depthImFull), IPL_DEPTH_16U, 1);
		cvResize(depthIm16, depthPaintedFull,0);
		IplImage *depthImMaskFull = cvCreateImage(cvGetSize(depthImFull), IPL_DEPTH_8U, 1);
		for (int y=0; y<depthImFull->height; y++) for (int x=0; x<depthImFull->width; x++)
			CV_IMAGE_ELEM(depthImMaskFull, char, y, x)=CV_IMAGE_ELEM(depthImFull, unsigned short,y,x)==0?255:0;
		cvCopy(depthPaintedFull, depthImFull, depthImMaskFull);
		cvReleaseImage(&depthPaintedFull); cvReleaseImage(&depthImMaskFull);
		cvReleaseImage(&depthImFull);
	} else {
		cvCopy(depthIm16, depthIm, depthImMask);
	}

	cvReleaseImage(&depthIm8); cvReleaseImage(&depthIm16);
	cvReleaseImage(&depthPaint);
	cvReleaseImage(&depthImMask); cvReleaseImage(&depthImMaskInv);
	cvReleaseImage(&depthIm);
}

void setWorldOrigin() {
	WORLD_ORIGIN_X = marker_origin.x/4; WORLD_ORIGIN_Y = marker_origin.y /4; 
	center_trimesh = cvPoint2D32f(WORLD_ORIGIN_X, WORLD_ORIGIN_Y);
	m_world->set_center_trimesh(WORLD_ORIGIN_X,WORLD_ORIGIN_Y);
}

void registerMarker() {
	if (calcKinectOpenGLTransform(colourIm, depthIm, &kinectTransform)) {
		//Load in the marker for registration
		osg_inittracker(MARKER_FILENAME, 400, markerSize.width);
		
		//Recreat world and controls
		delete kc;
		delete xc;
		delete m_world;
		m_world = new KCRPhysicsWorld();
		kc = new KeyboardController(m_world);
		xc = new XboxController(m_world);

		m_world->setWorldDepth(MARKER_DEPTH);
		m_world->setWorldScale(WORLD_SCALE);
		setOSGTrimeshScale(WORLD_SCALE);
		setWorldOrigin();
		transDepth160 = cvCreateImage(cvSize(MESH_SIZE.width, MESH_SIZE.height), IPL_DEPTH_32F, 1);
		TransformImage(depthIm, transDepth160, MARKER_DEPTH, MESH_SIZE, true);
		GenerateTrimeshGroundFromDepth(transDepth160, MARKER_DEPTH);
		m_world->updateTrimesh(ground_grid);
		m_world->setMinHeight(MinHeight);
		m_world->setMaxHeight(MaxHeight);
		m_world->initPhysics();
#ifdef SIM_PARTICLES
		CreateOSGSphereProxy();//osg spheres representation
#endif
#ifdef SIM_MICROMACHINE
		m_world->resetCarScene(0);
		m_world->resetCarScene(1);
#endif /*SIM_MICROMACHINE*/
	} else {
		printf("Couldn't find marker, please try again!\n");
	}
}

/*
#ifdef USE_SKIN_SEGMENTATION
void SegmentHandRegion(IplImage* color_src) {
		IplImage* colourImResized = cvCreateImage(cvSize(TRACKING_SIZE.width, TRACKING_SIZE.height), IPL_DEPTH_8U, 3);
		cvResize(color_src, colourImResized, CV_INTER_NN);//use nearest neighbor interpolation
		IplImage* gray = cvCreateImage(cvSize(colourImResized->width, colourImResized->height),IPL_DEPTH_8U,1);
		hand_region = cvCreateImage(cvSize(colourImResized->width, colourImResized->height),IPL_DEPTH_8U,1);
		cvCopyImage( _HandRegion.GetHandRegion( colourImResized, gray, false ), hand_region );
		cvShowImage ("Hand Segmentation", hand_region);
//		gray        = cvCreateImage(cvSize(niImageMD.XRes(), niImageMD.YRes()),IPL_DEPTH_8U,1);
//		hand_region = cvCreateImage(cvSize(niImageMD.XRes(), niImageMD.YRes()),IPL_DEPTH_8U,1);
//		cvCopyImage( _HandRegion.GetHandRegion( colourIm, gray, false ), hand_region );
//		cvShowImage ("Hand Segmentation", hand_region);
}
#endif
*/

#ifdef SIM_PARTICLES
void CreateOSGSphereProxy() {
	for(int i = 0; i < 30; i++) {
		for(int j = 0; j < 40; j++) {
			int index = i*40 + j;
			osgAddWorldSphereProxyNode(osgNodeFromBtSphere(m_world->getWorldSphereTransform(index)));
		}
	}
	printf("Spheres proxy created! \n");
}
#endif

#ifdef USE_OPTICAL_FLOW
void SceneOpticalFlowLK(IplImage* prevImg, IplImage* currImg) {
	IplImage* PrevGray = cvCreateImage(cvGetSize(prevImg), prevImg->depth, 1);
	IplImage* CurrGray = cvCreateImage(cvGetSize(currImg), currImg->depth, 1);
	IplImage* opImg = cvCreateImage(cvGetSize(currImg), currImg->depth, 3);
	CvMat* vx = cvCreateMat(currImg->height, currImg->width, CV_32FC1);
    CvMat* vy = cvCreateMat(currImg->height, currImg->width, CV_32FC1);
	cvCopyImage(currImg, opImg);
	cvCvtColor(prevImg,PrevGray, CV_RGB2GRAY);
	cvCvtColor(currImg,CurrGray, CV_RGB2GRAY);
	cvShowImage("CurrGray",CurrGray);
	cvCalcOpticalFlowLK(PrevGray,CurrGray, cvSize(LK_SIZE,LK_SIZE), vx, vy);
	for (int y = 0; y< CurrGray->height; y+=OPT_Y_STEP){
		for (int x = 0; x< CurrGray->width; x+=OPT_X_STEP){
			float dx = cvGetReal2D(vx,y,x);
			float dy = cvGetReal2D(vy,y,x);
			if( dx < 5 && dy < 5);
			else cvLine(opImg, cvPoint(x,y), cvPoint(x + dx, y + dy),CV_RGB(0,255,0));
		}
	}
	cvShowImage("opImg",opImg);
	cvReleaseImage(&PrevGray);
	cvReleaseImage(&CurrGray);
	cvReleaseImage(&opImg);
	cvReleaseMat(&vx);
	cvReleaseMat(&vy);

}
#endif

#ifdef USE_SKIN_SEGMENTATION
// Remove certain contour with area less than threshold
void removeNoise( IplImage* src, int size ){    
	IplImage* tmp = cvCreateImage(cvSize(src->width, src->height),IPL_DEPTH_8U,1);
	cvCopyImage(src, tmp);
	CvMemStorage* storage   = cvCreateMemStorage( 0 );     
	CvSeq* contours         = NULL;    
	CvScalar black          = CV_RGB( 0, 0, 0 ); 
	CvScalar white          = CV_RGB( 255, 255, 255 ); 
    double area;   
	cvFindContours( tmp, storage, &contours, sizeof( CvContour ), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );  

	while( contours )   {  
		area = cvContourArea( contours, CV_WHOLE_SEQ );       
		if( fabs( area ) <= size )  {// if less than threshold then remove by paint over white by black           
			cvDrawContours( src, contours, black, black, -1, CV_FILLED, 8 );      
		}     
		contours = contours->h_next;   
	}   
	cvReleaseMemStorage( &storage );    
	cvReleaseImage(&tmp);
}
#endif

#endif