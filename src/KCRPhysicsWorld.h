
#ifndef KCR_PHYS_WORLD_H
#define KCR_PHYS_WORLD_H

class btVehicleTuning;
struct btVehicleRaycaster;
class btCollisionShape;

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"
#include <vector>

const int NUM_VERTS_X = 160;
const int NUM_VERTS_Y = 120;
const int NUM_CAR = 2;

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

class KCRPhysicsWorld {
	public:

		class btThreadSupportInterface*		m_threadSupportCollision;
		class btThreadSupportInterface*		m_threadSupportSolver;

		btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
		btBroadphaseInterface*	m_overlappingPairCache;
		btCollisionDispatcher*	m_dispatcher;
		btConstraintSolver*	m_constraintSolver;
		btDefaultCollisionConfiguration* m_collisionConfiguration;
		btTriangleIndexVertexArray*	m_indexVertexArrays;
		btDiscreteDynamicsWorld* m_dynamicsWorld;

		btDefaultMotionState* groundMotionState;
		std::vector<btDefaultMotionState*> WorldSphereProxyMotionState;
		std::vector<btDefaultMotionState*> ObjectsMotionState;
		btRigidBody* groundRigidBody;

		btBoxShape* Box_Shape;
		btSphereShape* Sphere_Shape;
		std::vector<btConvexHullShape*> Convex_Shape;
		std::vector<btRigidBody*> Objects_Body;
		std::vector<btRigidBody*> WorldSphereProxyRigidBody;

		btCollisionShape* groundShape;
		btBvhTriangleMeshShape* trimeshShape;

		btVector3*	m_vertices;

		std::vector<btDefaultMotionState*> chassisMotionState;
		std::vector<btRigidBody*> m_carChassis;
		btRaycastVehicle::btVehicleTuning	m_tuning[NUM_CAR];
		std::vector<btVehicleRaycaster*>	m_vehicleRayCaster;
		std::vector<btRaycastVehicle*>	m_vehicle;
		std::vector<btCollisionShape*>	m_wheelShape;

		btClock m_clock;

		KCRPhysicsWorld(void);
		virtual ~KCRPhysicsWorld(void);
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
		int create_Box();
		int create_OneSided_Trapezoid_Ramp();
		int create_Triangular_Ramp();
		btConvexHullShape* get_Convex_Shape(int index);
		btTransform get_Object_Transform(int index);
		int Num_Object();
		void set_center_trimesh(float x, float y);
		void createWorldSphereProxy();
		btTransform getWorldSphereTransform(int index);
		void updateWorldSphereTransform(float *voxel);
		//
	private:
		bool hasInit;
		int count;
		int rightIndex;
		int upIndex;
		int forwardIndex;
		float	m_forwardAxis;
		//
		btVector3 wheelDirectionCS0[NUM_CAR];
		btVector3 wheelAxleCS[NUM_CAR];
		btScalar suspensionRestLength[NUM_CAR];
		Car Car_Array[NUM_CAR];
		float	worldDepth;
		float	wscale;

		float*	m_rawHeightfieldData;
		btScalar	m_minHeight;
		btScalar	m_maxHeight;
		int HF_Size;
		btVector3	center_trimesh;

};

#endif //KCR_PHYS_WORLD_H