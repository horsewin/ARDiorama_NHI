#ifndef ARMM_VRPN_H
#define ARMM_VRPN_H

#define SIM_MICROMACHINE 1

#include <stdio.h>
#include <tchar.h>
#include <math.h>

//#include "vrpn_Text.h"
#include "ARMM_Communicator.h"
#include <iostream>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Texture2D>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PolygonMode>
#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osgShadow/ShadowedScene>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/io_utils>

#include "btBulletDynamicsCommon.h"

//#include "opencv\cv.h"
//#include "opencv\highgui.h"

using namespace std;
const int HAND_SIZE = 442;

/////////////////////// ARMM_COMMUNICATOR /////////////////////////////
class bt_ARMM_hand;
class ARMM_Communicator : public vrpn_Tracker
{
public:
	ARMM_Communicator( vrpn_Connection_IP *c = 0);
	virtual ~ARMM_Communicator();
	virtual void mainloop();

	void SetObjectsData(std::vector<btRigidBody*> * obj);
	void SetHandsData(std::vector<bt_ARMM_hand*> * hand);

protected:
	virtual int encode_hand_to(char *buf);

protected:
    struct timeval _timestamp;

		std::vector<btRigidBody*> * m_objects_body;
		std::vector<bt_ARMM_hand*>* m_hands_body;

		//Atsushi
		vrpn_int32 hand_m_id;	// ID of tracker hand message					

		//Atsushi
		vrpn_float32 hand[HAND_SIZE][3];

		//	float* HeightfieldData;
//	osg::Vec3d *CarsArrayPos;
//	osg::Quat *CarsArrayQuat;
//	osg::Vec3d **WheelsArrayPos;
//	osg::Quat **WheelsArrayQuat;
};

#endif