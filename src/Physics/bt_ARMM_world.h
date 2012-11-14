
#ifndef BT_ARMM_WORLD_H
#define BT_ARMM_WORLD_H

class btBroadphaseInterface;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
class btVAehicleTuning;
struct btVehicleRaycaster;
class btCollisionShape;
class bt_ARMM_hand;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

//Bullet Physics Library
#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftBody.h>

//OpenSceneGraph
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osg/Geode>
#include <osg/LightModel>
#include <osg/Texture2D>
#include <osgUtil/SmoothingVisitor>

//Standard 
#include <vector>
#include <string>

//Boost Library
#include <boost\shared_ptr.hpp>

#include "../UserConstant.h"
#include "ARMM\Rendering\osg_Menu.h"

#define FRICTION 1.0f
#define RESTITUTION 0.0
#define REP(i,n) for(int i=0;i<(int)n;++i)

const int NUM_VERTS_X = 160;
const int NUM_VERTS_Y = 120;
const int NUM_CAR = 2;

enum panelinput{NOTHING, ADDARMODEL, SPHERE, CAR1, CAR2, REGENERATE};

struct Car{
	float	chassis_width;
	float	chassis_height;
	float	chassis_length;
	float	car_mass;
	float	gEngineForce;
	float	gBreakingForce;
	float	maxEngineForce;
	float	maxBreakingForce;
	float	defaultBreakingForce;
	float	gVehicleSteering;
	float	steeringIncrement;
	float	steeringClamp;
	float	wheelRadius;
	float	wheelWidth;
	float	wheelFriction;
	float	suspensionStiffness;
	float	suspensionDamping;
	float	suspensionCompression;
	float	rollInfluence;
	float	chassisDistFromGround;
	float	connectionHeight;
	float	wheelLengthOffsetFront;
	float	wheelLengthOffsetBack;
	float	wheelWidthOffsetFront;
	float	wheelWidthOffsetBack;
};

class bt_ARMM_world 
{
public:
		class btThreadSupportInterface*		m_threadSupportCollision;
		class btThreadSupportInterface*		m_threadSupportSolver;

		btAlignedObjectArray<btCollisionShape*>		m_collisionShapes;
		btBroadphaseInterface*						m_overlappingPairCache;
		btCollisionDispatcher*						m_dispatcher;
		btConstraintSolver*							m_constraintSolver;
		btSoftBodyRigidBodyCollisionConfiguration*	m_SoftCollisionConfiguration; //added by Atsushi.U 2012/07/28
		btDefaultCollisionConfiguration*			m_collisionConfiguration;
		btTriangleIndexVertexArray*					m_indexVertexArrays;
		btSoftBodyWorldInfo							m_worldInfo;

	    btDiscreteDynamicsWorld* m_dynamicsWorld;

		btDefaultMotionState*				m_groundMotionState;
		std::vector<btDefaultMotionState*>	m_WorldSphereProxyMotionState;
		std::vector<btDefaultMotionState*>	ObjectsMotionState;
		btRigidBody* groundRigidBody;

		btBoxShape* Box_Shape;
		btSphereShape* Sphere_Shape;
		std::vector<btConvexHullShape*> Convex_Shape;

		//virtual objects with physics
		std::vector< boost::shared_ptr<btRigidBody> >	m_objectsBody;
		std::vector< boost::shared_ptr<bt_ARMM_hand>>	HandObjectsArray;
		std::vector<bool>			HandFingersArray;
		std::vector< boost::shared_ptr<btRigidBody> >	mMenuBody;
		int mMenuIndexOffset;
		std::vector< boost::shared_ptr<btRigidBody> >	mModelButton;
		int mModelButtonIndexOffset;
		
		//virtual terrain with physics
		btCollisionShape* groundShape;
		btBvhTriangleMeshShape* trimeshShape;

		btVector3*	m_vertices;

		std::vector<btDefaultMotionState*>	chassisMotionState;
		std::vector<btRigidBody*>			m_carChassis;
		btRaycastVehicle::btVehicleTuning	m_tuning[NUM_CAR];
		std::vector<btVehicleRaycaster*>	m_vehicleRayCaster;
		std::vector<btRaycastVehicle*>		m_vehicle;
		std::vector<btCollisionShape*>		m_wheelShape;

		btClock m_clock;

		btSoftBody* textureSoftBody;

	public:
		bt_ARMM_world(void);
		virtual ~bt_ARMM_world(void);
		virtual void Update();

		void initPhysics();

		void setCarMovement();
		btTransform getCarPose(int index);
		btTransform getWheelTransform(int carIndex, int wheelInt);
		float* Quaternion2RotMat3x3(btScalar x, btScalar y, btScalar z, btScalar w);
		btScalar virtual getDeltaTimeMicroseconds();
		void resetEngineForce(int index);
		void accelerateEngine(int index);
		void decelerateEngine(int index);
		void turnReset(int index);
		void turnEngineLeft(int index);
		void turnEngineRight(int index);
		void resetCarScene(int car_index);
		void setWorldDepth(float w);
		//
		float getGridHeight(float* grid, int i, int j);
		float* getTrimeshGround();
		int getGridSize();
		btTransform getTrimeshGroundTransform();
		void clearTrimesh();
		void updateTrimesh(float* hf);
		void updateTrimeshRefitTree(float* hf);
		void setMinHeight(float min);
		void setMaxHeight(float max);
		void setWorldScale(float scale);
		//
		int create_Sphere();
		int create_Box();
		int create_3dsmodel(std::string modelname, osg::ref_ptr<osg::Node> sample = NULL);
		void CreateMenu(ARMM::osg_Menu* osgMenu);
		void CreateModelButton(ARMM::osg_Menu* osgMenu);
		osg::Node* CreateSoftTexture(std::string texturename);

		double get3dsScale();
		std::string GetModelName(void) const;

		btConvexHullShape* get_Convex_Shape(int index);
		btTransform get_Object_Transform(int index);
		int Num_Object();
		void set_center_trimesh(float x, float y);
		void createWorldSphereProxy();
		//btTransform getWorldSphereTransform(int index);
		//void updateWorldSphereTransform(float *voxel);
		
		//Function for hand creation
		void createHand(int hand_x, int hand_y, int sphere_resolution, float ratio);
		void updateHandDepth(int index, int hand_x, int hand_y, float* depth_grid);
		void updateHandDepth(int index, int hand_x, int hand_y, float curr_hands_ratio, float* depth_grid);
		btTransform getHandTransform(int index);
		int getTotalNumberHand();
		float* debugHandX(int index);
		float* debugHandY(int index);
		float* debugHandZ(int index);	
		bool ToggleSphereRep( void );
		bool GetSphereRep( void );

		//Function for kinematic box
		void ChangeAttribute(int x=10, int y=-5, int z=2, int index = 0);
		void ChangeRotation(double x, double y, double z);
		virtual const btSoftRigidDynamicsWorld*	getSoftDynamicsWorld() const
		{
			///just make it a btSoftRigidDynamicsWorld please
			///or we will add type checking
			return (btSoftRigidDynamicsWorld*) m_dynamicsWorld;
		}

		virtual btSoftRigidDynamicsWorld*	getSoftDynamicsWorld()
		{
			///just make it a btSoftRigidDynamicsWorld please
			///or we will add type checking
			return (btSoftRigidDynamicsWorld*) m_dynamicsWorld;
		}

		void ResetARButtonInput( void );

	private:
		void CalcGlobalValue(float * global_x, float * global_y, const int & hand_x, const int & hand_y);
	    void DecideCollisionCondition();
	    void DecideCollisionPanel();
		void DecideCollisionModelButton();
		void SoftTextureUpdate( void );

	private:
		bool hasInit;
		int count;
		int rightIndex;
		int upIndex;
		int forwardIndex;
		//
		btVector3 wheelDirectionCS0[NUM_CAR];
		btVector3 wheelAxleCS[NUM_CAR];
		btScalar suspensionRestLength[NUM_CAR];
		Car Car_Array[NUM_CAR];
		float	worldDepth;
		float	world_scale;

		float*	m_rawHeightfieldData;
		btScalar	m_minHeight;
		btScalar	m_maxHeight;
		int HF_Size;
		btVector3	center_trimesh;

		bool m_sphere_rep;

		//for 3ds model
		double scale_3ds;
		std::string modelname;
};

#endif //BT_ARMM_WORLD_H