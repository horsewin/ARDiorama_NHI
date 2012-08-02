
//#define USE_PARALLEL_SOLVER 1 //experimental parallel solver
//#define USE_PARALLEL_DISPATCHER 1
#define CUBE_HALF_EXTENTS 1
#define SIM_MICROMACHINE 1
//#define SIM_PARTICLES 1

#include "KCRPhysicsWorld.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
//#include "BulletCollision/CollisionShapes/btShapeHull.h"
//#include <stdio.h>

#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"

#ifdef USE_PARALLEL_DISPATCHER
#include "BulletMultiThreaded/SpuGatheringCollisionDispatcher.h"
#include "BulletMultiThreaded/PlatformDefinitions.h"

#ifdef USE_LIBSPE2
#include "BulletMultiThreaded/SpuLibspe2Support.h"
#elif defined (_WIN32)
#include "BulletMultiThreaded/Win32ThreadSupport.h"
#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#endif 

#ifdef USE_PARALLEL_SOLVER
#include "BulletMultiThreaded/btParallelConstraintSolver.h"
#include "BulletMultiThreaded/SequentialThreadSupport.h"

btThreadSupportInterface* createSolverThreadSupport(int maxNumThreads) {
#ifdef _WIN32
	Win32ThreadSupport::Win32ThreadConstructionInfo threadConstructionInfo("solverThreads",SolverThreadFunc,SolverlsMemoryFunc,maxNumThreads);
	Win32ThreadSupport* threadSupport = new Win32ThreadSupport(threadConstructionInfo);
	threadSupport->startSPU();
#endif
	return threadSupport;
}
#endif //USE_PARALLEL_SOLVER

#endif//USE_PARALLEL_DISPATCHER

const int maxProxies = 32766;
const int maxOverlap = 65535;

btVector3 worldMin(-1000,-1000,-1000);
btVector3 worldMax(1000,1000,1000);

KCRPhysicsWorld::KCRPhysicsWorld(void):m_carChassis(0),m_indexVertexArrays(0),m_vertices(0)
{

	hasInit = false;
	//m_vehicle.push_back(0);
	//m_wheelShape.push_back(0);
	count = 0;
	rightIndex = 0; 
	upIndex = 2; 
	forwardIndex = 1;

#ifdef SIM_MICROMACHINE

	wheelDirectionCS0[0] = btVector3(0,0,-1);
	wheelAxleCS[0] = btVector3(1,0,0);
	suspensionRestLength[0] = btScalar(0.6);

	wheelDirectionCS0[1] = btVector3(0,0,-1);
	wheelAxleCS[1] = btVector3(1,0,0);
	suspensionRestLength[1] = btScalar(0.6);

	Car_Array[0].chassis_width = 1.f;
	Car_Array[0].chassis_length = 2.8f;
	Car_Array[0].chassis_height = 0.7f;
	Car_Array[0].car_mass = 250;
	Car_Array[0].gEngineForce = 0.f;
	Car_Array[0].gBreakingForce = 0.f;
	Car_Array[0].maxEngineForce = 2500.f;//this should be engine/velocity dependent //1000
	Car_Array[0].maxBreakingForce = 200.f;//100
	Car_Array[0].defaultBreakingForce = 50.f;//10
	Car_Array[0].gVehicleSteering = 0.f;
	Car_Array[0].steeringIncrement = 0.2f;//0.04f
	Car_Array[0].steeringClamp = 0.5f;//0.3f
	Car_Array[0].wheelRadius = 0.5f;
	Car_Array[0].wheelWidth = 0.4f;
	Car_Array[0].wheelFriction = 1000;//BT_LARGE_FLOAT;
	Car_Array[0].suspensionStiffness = 20.f;
	Car_Array[0].suspensionDamping = 2.3f;
	Car_Array[0].suspensionCompression = 4.4f;
	Car_Array[0].rollInfluence = 1.0f;//0.1f;
	Car_Array[0].chassisDistFromGround = 1.3f;
	Car_Array[0].connectionHeight = 1.3f;
	Car_Array[0].wheelLengthOffsetFront = 1.1f;
	Car_Array[0].wheelLengthOffsetBack = 1.2f;
	Car_Array[0].wheelWidthOffsetFront = 1.1f;
	Car_Array[0].wheelWidthOffsetBack = 1.1f;

	Car_Array[1].chassis_width = 1.3f;
	Car_Array[1].chassis_length = 2.8f;
	Car_Array[1].chassis_height = 0.5f;
	Car_Array[1].car_mass = 250;
	Car_Array[1].gEngineForce = 0.f;
	Car_Array[1].gBreakingForce = 0.f;
	Car_Array[1].maxEngineForce = 2500.f;//this should be engine/velocity dependent //1000
	Car_Array[1].maxBreakingForce = 200.f;//100
	Car_Array[1].defaultBreakingForce = 50.f;//10
	Car_Array[1].gVehicleSteering = 0.f;
	Car_Array[1].steeringIncrement = 0.2f;//0.04f
	Car_Array[1].steeringClamp = 0.5f;//0.3f
	Car_Array[1].wheelRadius = 0.5f;
	Car_Array[1].wheelWidth = 0.4f;
	Car_Array[1].wheelFriction = 1000;//BT_LARGE_FLOAT;
	Car_Array[1].suspensionStiffness = 20.f;
	Car_Array[1].suspensionDamping = 2.3f;
	Car_Array[1].suspensionCompression = 4.4f;
	Car_Array[1].rollInfluence = 1.0f;//0.1f;
	Car_Array[1].chassisDistFromGround = 1.3f;
	Car_Array[1].connectionHeight = 1.2f;
	Car_Array[1].wheelLengthOffsetFront = 1.1f;
	Car_Array[1].wheelLengthOffsetBack = 1.2f;
	Car_Array[1].wheelWidthOffsetFront = 1.3f;
	Car_Array[1].wheelWidthOffsetBack = 1.2f;

#endif /*SIM_MICROMACHINE*/

	worldDepth = 0;
	m_rawHeightfieldData = new float[19200];
	HF_Size = 19200;
}

KCRPhysicsWorld::~KCRPhysicsWorld(void) { //cleanup in the reverse order of creation/initialization
	//remove the rigidbodies from the dynamics world and delete them
	int i;

	if (hasInit) {
		for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--) {
			btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState()) {
				delete body->getMotionState();
			}
			m_dynamicsWorld->removeCollisionObject( obj );
			delete obj;
		}
#ifdef SIM_MICROMACHINE
		chassisMotionState.clear();
#endif /*SIM_MICROMACHINE*/
		//delete collision shapes
		for (int j=0;j<m_collisionShapes.size();j++) delete m_collisionShapes[j];
		m_collisionShapes.clear();

//		for (int j=0;j<Convex_Shape.size();j++) delete Convex_Shape[j];
		Convex_Shape.clear();
//		for (int j=0;j<Objects_Body.size();j++)	delete Objects_Body[j];
		Objects_Body.clear();

		delete m_indexVertexArrays;
		delete m_vertices;
		delete m_dynamicsWorld;

#ifdef SIM_MICROMACHINE
		for (int i = 0; i < 2; i++) {
			delete m_vehicleRayCaster.at(i);
			delete m_vehicle.at(i);
			delete m_wheelShape.at(i);
		}
		m_wheelShape.clear();
#endif /*SIM_MICROMACHINE*/
		delete m_constraintSolver;
#ifdef USE_PARALLEL_DISPATCHER
		if (m_threadSupportSolver) {
			delete m_threadSupportSolver;
		}
#endif
		delete m_overlappingPairCache; //delete broadphase
		delete m_dispatcher; //delete dispatcher
#ifdef USE_PARALLEL_DISPATCHER
		if (m_threadSupportCollision) {
			delete m_threadSupportCollision;
		}
#endif
		delete m_collisionConfiguration;
	}
	hasInit =false;

}

void KCRPhysicsWorld::initPhysics() {

#ifdef USE_PARALLEL_DISPATCHER
	m_threadSupportSolver = 0;
	m_threadSupportCollision = 0;
#endif

	m_forwardAxis = 1;
	groundShape = new btBoxShape(btVector3(50,3,50));
//	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	btDefaultCollisionConstructionInfo cci;
	cci.m_defaultMaxPersistentManifoldPoolSize = maxProxies;
	m_collisionConfiguration = new btDefaultCollisionConfiguration(cci);


#ifdef USE_PARALLEL_DISPATCHER
	int maxNumOutstandingTasks = 2;

#ifdef USE_WIN32_THREADING

m_threadSupportCollision = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
								"collision",
								processCollisionTask,
								createCollisionLocalStoreMemory,
								maxNumOutstandingTasks));
#endif
	m_dispatcher = new	SpuGatheringCollisionDispatcher(m_threadSupportCollision,maxNumOutstandingTasks,m_collisionConfiguration);
#else
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#endif //USE_PARALLEL_DISPATCHER

//	btVector3 worldMin(-1000,-1000,-1000);
//	btVector3 worldMax(1000,1000,1000);
	m_overlappingPairCache = new btAxisSweep3(worldMin,worldMax);
	
#ifdef USE_PARALLEL_SOLVER
	m_threadSupportSolver = createSolverThreadSupport(maxNumOutstandingTasks);
	m_constraintSolver = new btParallelConstraintSolver(m_threadSupportSolver);
	//this solver requires the contacts to be in a contiguous pool, so avoid dynamic allocation
	m_dispatcher->setDispatcherFlags(btCollisionDispatcher::CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION);
#else
	m_constraintSolver = new btSequentialImpulseConstraintSolver();
	//btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
	//m_constraintSolver = solver;
#endif //USE_PARALLEL_SOLVER

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_constraintSolver,m_collisionConfiguration);

#ifdef USE_PARALLEL_SOLVER
	m_dynamicsWorld->getSimulationIslandManager()->setSplitIslands(false);
	m_dynamicsWorld->getSolverInfo().m_numIterations = 4;
	m_dynamicsWorld->getSolverInfo().m_solverMode = SOLVER_SIMD+SOLVER_USE_WARMSTARTING;//+SOLVER_RANDMIZE_ORDER;
	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
#endif

	m_dynamicsWorld->setGravity(btVector3(0,0,-10));

	btTransform tr;
	tr.setIdentity();

	/*Trimesh ground creation*/
	const float TRIANGLE_SIZE=wscale;
	int vertStride = sizeof(btVector3);
	int indexStride = 3*sizeof(int);
	const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
	const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);

	m_vertices = new btVector3[totalVerts];
	int* gIndices = new int[totalTriangles*3];

//	printf("tri_size = %f , x = %d, y = %d\n", TRIANGLE_SIZE, NUM_VERTS_X, NUM_VERTS_Y);

	for (int i=0;i<NUM_VERTS_X;i++) {
		for (int j=0;j<NUM_VERTS_Y;j++) {
			float x = (i-center_trimesh.getX())*TRIANGLE_SIZE;
			float y = (j-(120-center_trimesh.getY()))*TRIANGLE_SIZE;
			float height = m_rawHeightfieldData[j*160+i];
			m_vertices[i+j*NUM_VERTS_X].setValue(x, y, height);
		}
	}
	int index=0;
	for (int i=0;i<NUM_VERTS_X-1;i++) {
		for (int j=0;j<NUM_VERTS_Y-1;j++) {
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = j*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i;
		}
	}
	
	m_indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles, gIndices,indexStride,totalVerts,(btScalar*) &m_vertices[0].x(),vertStride);
	bool useQuantizedAabbCompression = true;

	trimeshShape = new btBvhTriangleMeshShape(m_indexVertexArrays,useQuantizedAabbCompression,worldMin,worldMax);
	groundShape = trimeshShape;
	//btVector3 localScaling(wscale,wscale,1);
	btVector3 localScaling(1,1,1);
	localScaling[upIndex]=1.f;
	groundShape->setLocalScaling(localScaling);

	tr.setOrigin(btVector3(0,0,0));
	m_collisionShapes.push_back(groundShape);
	groundMotionState = new btDefaultMotionState(tr);
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
    groundRigidBody = new btRigidBody(groundRigidBodyCI);
	groundRigidBody->setFriction(btScalar(0.8));
    m_dynamicsWorld->addRigidBody(groundRigidBody);
	/*End trimesh ground*/

#ifdef SIM_MICROMACHINE
	/*Car Creation - Chassis*/
	for (int i = 0; i < NUM_CAR; i++ ) {	
		btCollisionShape* chassisShape = new btBoxShape(btVector3(Car_Array[i].chassis_width,Car_Array[i].chassis_length, Car_Array[i].chassis_height));
		m_collisionShapes.push_back(chassisShape);
		btCompoundShape* compound = new btCompoundShape();
		m_collisionShapes.push_back(compound);

		btTransform localTrans;
		localTrans.setIdentity();
		localTrans.setOrigin(btVector3(0,0,Car_Array[i].chassisDistFromGround));//localTrans effectively shifts the center of mass with respect to the chassis

		compound->addChildShape(localTrans,chassisShape);
	
		chassisMotionState.push_back(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0))));
		btVector3 localInertia(0, 0, 0);
		compound->calculateLocalInertia(Car_Array[i].car_mass,localInertia);
		btRigidBody::btRigidBodyConstructionInfo carRigidBody1CI( Car_Array[i].car_mass,chassisMotionState.at(i),compound,localInertia);
		m_carChassis.push_back(new btRigidBody(carRigidBody1CI));

		m_dynamicsWorld->addRigidBody(m_carChassis.at(i));

		m_wheelShape.push_back(new btCylinderShapeX(btVector3(Car_Array[i].wheelWidth,Car_Array[i].wheelRadius,Car_Array[i].wheelRadius)));

		{
			m_vehicleRayCaster.push_back(new btDefaultVehicleRaycaster(m_dynamicsWorld));
			m_vehicle.push_back(new btRaycastVehicle(m_tuning[i],m_carChassis.at(i),m_vehicleRayCaster.at(i)));
			///never deactivate the vehicle
			m_carChassis.at(i)->setActivationState(DISABLE_DEACTIVATION);
			m_dynamicsWorld->addVehicle(m_vehicle.at(i));
			bool isFrontWheel=true;

			m_vehicle.at(i)->setCoordinateSystem(rightIndex,upIndex,forwardIndex);
			btVector3 connectionPointCS0(Car_Array[i].wheelWidthOffsetFront*CUBE_HALF_EXTENTS-(0.3*Car_Array[i].wheelWidth),Car_Array[i].wheelLengthOffsetFront*2*CUBE_HALF_EXTENTS-Car_Array[i].wheelRadius, Car_Array[i].connectionHeight);
			m_vehicle.at(i)->addWheel(connectionPointCS0,wheelDirectionCS0[i],wheelAxleCS[i],suspensionRestLength[i],Car_Array[i].wheelRadius,m_tuning[i],isFrontWheel);
			connectionPointCS0 = btVector3(Car_Array[i].wheelWidthOffsetFront*-CUBE_HALF_EXTENTS+(0.3*Car_Array[i].wheelWidth),Car_Array[i].wheelLengthOffsetFront*2*CUBE_HALF_EXTENTS-Car_Array[i].wheelRadius, Car_Array[i].connectionHeight);
			m_vehicle.at(i)->addWheel(connectionPointCS0,wheelDirectionCS0[i],wheelAxleCS[i],suspensionRestLength[i],Car_Array[i].wheelRadius,m_tuning[i],isFrontWheel);
			connectionPointCS0 = btVector3(Car_Array[i].wheelWidthOffsetBack*-CUBE_HALF_EXTENTS+(0.3*Car_Array[i].wheelWidth),Car_Array[i].wheelLengthOffsetBack*(-2)*CUBE_HALF_EXTENTS+Car_Array[i].wheelRadius, Car_Array[i].connectionHeight);
			isFrontWheel = false;
			m_vehicle.at(i)->addWheel(connectionPointCS0,wheelDirectionCS0[i],wheelAxleCS[i],suspensionRestLength[i],Car_Array[i].wheelRadius,m_tuning[i],isFrontWheel);
			connectionPointCS0 = btVector3(Car_Array[i].wheelWidthOffsetBack*CUBE_HALF_EXTENTS-(0.3*Car_Array[i].wheelWidth),Car_Array[i].wheelLengthOffsetBack*(-2)*CUBE_HALF_EXTENTS+Car_Array[i].wheelRadius, Car_Array[i].connectionHeight);
			m_vehicle.at(i)->addWheel(connectionPointCS0,wheelDirectionCS0[i],wheelAxleCS[i],suspensionRestLength[i],Car_Array[i].wheelRadius,m_tuning[i],isFrontWheel);
			for (int k=0;k<m_vehicle.at(i)->getNumWheels();k++) {
				btWheelInfo& wheel = m_vehicle.at(i)->getWheelInfo(k);
				wheel.m_suspensionStiffness = Car_Array[i].suspensionStiffness;
				wheel.m_wheelsDampingRelaxation = Car_Array[i].suspensionDamping;
				wheel.m_wheelsDampingCompression = Car_Array[i].suspensionCompression;
				wheel.m_frictionSlip = Car_Array[i].wheelFriction;
				wheel.m_rollInfluence = Car_Array[i].rollInfluence;
			}
		}
	}

	resetCarScene(0);
	resetCarScene(1);
#endif /*SIM_MICROMACHINE*/

#ifdef SIM_PARTICLES
	createWorldSphereProxy();
#endif

	m_clock.reset();

	hasInit = true;
}


void KCRPhysicsWorld::Update() {

#ifdef SIM_MICROMACHINE
	setCarMovement();
	for (int i = 0; i < NUM_CAR; i++) {
		for (int j=0; j < m_vehicle.at(i)->getNumWheels(); j++) {
			m_vehicle.at(i)->updateWheelTransform(j,true); 
		}
	}
#endif /*SIM_MICROMACHINE*/
	m_dynamicsWorld->stepSimulation(1/20.f,5); //60s
//	m_dynamicsWorld->stepSimulation(1/15.f,3);
	//float ms = getDeltaTimeMicroseconds();
	//m_dynamicsWorld->stepSimulation(ms / 1000000.f);

}

void KCRPhysicsWorld::setCarMovement() {
	//Drive front wheels
	for (int i = 0; i < NUM_CAR; i++) {
		m_vehicle.at(i)->setSteeringValue(Car_Array[i].gVehicleSteering,0);
		m_vehicle.at(i)->setBrake(Car_Array[i].gBreakingForce,0);
		m_vehicle.at(i)->setSteeringValue(Car_Array[i].gVehicleSteering,1);
		m_vehicle.at(i)->setBrake(Car_Array[i].gBreakingForce,1);

		//Drive rear wheels
		m_vehicle.at(i)->applyEngineForce(Car_Array[i].gEngineForce,2);
		m_vehicle.at(i)->setBrake(Car_Array[i].gBreakingForce,2);
		m_vehicle.at(i)->applyEngineForce(Car_Array[i].gEngineForce,3);
		m_vehicle.at(i)->setBrake(Car_Array[i].gBreakingForce,3);
	}
}

btTransform KCRPhysicsWorld::getCarPose(int index) {
	btTransform trans;
	m_carChassis.at(index)->getMotionState()->getWorldTransform(trans);
	return trans;
}

btTransform KCRPhysicsWorld::getWheelTransform(int carIndex, int wheelInt) {
	btTransform trans = m_vehicle.at(carIndex)->getWheelTransformWS(wheelInt);
	//btQuaternion rot = m_vehicle.at(carIndex)->getWheelInfo(wheelInt).m_worldTransform.getRotation();
	//m_vehicle.at(carIndex)->getWheelInfo(wheelInt).m_worldTransform.getOpenGLMatrix(m);
	//m_carChassis.at(index)->getMotionState()->getWorldTransform(trans);
	return trans;
}


btScalar KCRPhysicsWorld::getDeltaTimeMicroseconds() {
	btScalar dt = (btScalar)m_clock.getTimeMicroseconds();
	m_clock.reset();
	return dt;
}

void KCRPhysicsWorld::resetEngineForce(int index) {
	Car_Array[index].gEngineForce = 0.f;
	Car_Array[index].gBreakingForce = Car_Array[index].defaultBreakingForce; 
	//printf("reset engine force\n");
}

void KCRPhysicsWorld::accelerateEngine(int index) {
	Car_Array[index].gEngineForce = Car_Array[index].maxEngineForce;
	Car_Array[index].gBreakingForce = 0.f;
	//printf("accelerate \n");
}

void KCRPhysicsWorld::decelerateEngine(int index) {
	Car_Array[index].gEngineForce = -Car_Array[index].maxEngineForce;
	Car_Array[index].gBreakingForce = 0.f;
	//printf("decelerate \n");
}

void KCRPhysicsWorld::turnReset(int index) {
	Car_Array[index].gVehicleSteering = 0;
}

void KCRPhysicsWorld::turnEngineLeft(int index) {
	Car_Array[index].gVehicleSteering += Car_Array[index].steeringIncrement;
	if (Car_Array[index].gVehicleSteering > Car_Array[index].steeringClamp)	
		Car_Array[index].gVehicleSteering = Car_Array[index].steeringClamp;
	//printf("turn left \n");
}

void KCRPhysicsWorld::turnEngineRight(int index) {
	Car_Array[index].gVehicleSteering -= Car_Array[index].steeringIncrement;
	if (Car_Array[index].gVehicleSteering < -Car_Array[index].steeringClamp)
		Car_Array[index].gVehicleSteering = -Car_Array[index].steeringClamp;
	//printf("turn right \n");
}

void KCRPhysicsWorld::resetCarScene(int car_index) {
	//for (int i =0; i < NUM_CAR; i++) {
		Car_Array[car_index].gVehicleSteering = 0.f;
		Car_Array[car_index].gBreakingForce = Car_Array[car_index].defaultBreakingForce;
		Car_Array[car_index].gEngineForce = 0.f;

		//m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
		m_carChassis.at(car_index)->setCenterOfMassTransform(btTransform(btQuaternion(0,0,0,1),btVector3(5*car_index+5,-5,5)));
		m_carChassis.at(car_index)->setLinearVelocity(btVector3(0,0,0));
		m_carChassis.at(car_index)->setAngularVelocity(btVector3(0,0,0));
		m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_carChassis.at(car_index)->getBroadphaseHandle(),m_dynamicsWorld->getDispatcher());
		if (m_vehicle.at(car_index))
		{
			m_vehicle.at(car_index)->resetSuspension();
			for (int j=0;j<m_vehicle.at(car_index)->getNumWheels();j++)
			{
				//synchronize the wheels with the (interpolated) chassis worldtransform
				m_vehicle.at(car_index)->updateWheelTransform(j,true);
			}
		}
	//}
}

void KCRPhysicsWorld::setWorldDepth(float w) {
	worldDepth = w;
}

float* KCRPhysicsWorld::getTrimeshGround() {
	return m_rawHeightfieldData;
}

btTransform KCRPhysicsWorld::getTrimeshGroundTransform() {
	btTransform trans;
	groundRigidBody->getMotionState()->getWorldTransform(trans);
	return trans;
}

void KCRPhysicsWorld::setMinHeight(float min) {
	m_minHeight = btScalar(min);
}

void KCRPhysicsWorld::setMaxHeight(float max) {
	m_maxHeight = btScalar(max);
}

void KCRPhysicsWorld::setWorldScale(float scale) {
	wscale = scale;
}

int KCRPhysicsWorld::create_Box() {
	float boxMass = 50;
//	Convex_Shape.push_back(new btConvexHullShape());//place holder
//	int shape_index = Convex_Shape.size()-1;
	float scale = 2;
	Box_Shape =  new btBoxShape(btVector3(1, 1, 1));
	m_collisionShapes.push_back(Box_Shape);
	ObjectsMotionState.push_back(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0))));
	btVector3 localInertia(0.0f, 0.0f, 0.0f);
	Box_Shape->calculateLocalInertia(boxMass, localInertia);

	int index = ObjectsMotionState.size()-1;
    btRigidBody::btRigidBodyConstructionInfo Box_Body_CI(boxMass,ObjectsMotionState.at(index),Box_Shape,localInertia);
	Objects_Body.push_back(new btRigidBody(Box_Body_CI));
	m_dynamicsWorld->addRigidBody(Objects_Body.at(index));
	Objects_Body.at(index)
		->setCenterOfMassTransform(btTransform(btQuaternion(0,0,0,1),btVector3(10,-5,10)));
	return index;
}

int KCRPhysicsWorld::create_OneSided_Trapezoid_Ramp() {//PointCloud2ConvexHullBody
	Convex_Shape.push_back(new btConvexHullShape());

	int shape_index = Convex_Shape.size()-1;
	float scale = 2;
	Convex_Shape.at(shape_index)->addPoint(btVector3(scale, scale, scale));
	Convex_Shape.at(shape_index)->addPoint(btVector3(scale, -3*scale, -1*scale));
	Convex_Shape.at(shape_index)->addPoint(btVector3(scale, -3*scale, 1*scale));
	Convex_Shape.at(shape_index)->addPoint(btVector3(-0.5*scale, -3*scale, 1*scale));
	Convex_Shape.at(shape_index)->addPoint(btVector3(2*scale, 3*scale, -1*scale));
	Convex_Shape.at(shape_index)->addPoint(btVector3(-2*scale, 3*scale, -1*scale));
	Convex_Shape.at(shape_index)->addPoint(btVector3(-2*scale, 3*scale, 1*scale));
	Convex_Shape.at(shape_index)->addPoint(btVector3(-0.5*scale, 3*scale, 1*scale));
	m_collisionShapes.push_back(Convex_Shape.at(shape_index));

	float convexMass = 500*scale;
	ObjectsMotionState.push_back(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0))));
	btVector3 localInertia(0.0f, 0.0f, 0.0f);
	Convex_Shape.at(shape_index)->calculateLocalInertia(convexMass, localInertia);

	int index = ObjectsMotionState.size()-1;
    btRigidBody::btRigidBodyConstructionInfo Convex_Body_CI(convexMass,ObjectsMotionState.at(index),Convex_Shape.at(shape_index),localInertia);
	Objects_Body.push_back(new btRigidBody(Convex_Body_CI));
	m_dynamicsWorld->addRigidBody(Objects_Body.at(index));
	Objects_Body.at(index)->setCenterOfMassTransform(btTransform(btQuaternion(0,0,-1,1),btVector3(20,-5,10)));
	return index;
}

int KCRPhysicsWorld::create_Triangular_Ramp() {
	Convex_Shape.push_back(new btConvexHullShape());

	int shape_index = Convex_Shape.size()-1;
	float scale = 2;
	Convex_Shape.at(shape_index)->addPoint(btVector3(-1*scale, -1*scale, 0));
	Convex_Shape.at(shape_index)->addPoint(btVector3(1*scale, -1*scale, 0));
	Convex_Shape.at(shape_index)->addPoint(btVector3(1*scale, 1*scale, 0));
	Convex_Shape.at(shape_index)->addPoint(btVector3(-1*scale, 1*scale, 0));
	Convex_Shape.at(shape_index)->addPoint(btVector3(1*scale, 1*scale, 1*scale));
	Convex_Shape.at(shape_index)->addPoint(btVector3(-1*scale, 1*scale, 1*scale));
	m_collisionShapes.push_back(Convex_Shape.at(shape_index));

	float convexMass = 500*scale;
	ObjectsMotionState.push_back(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0))));
	btVector3 localInertia(0.0f, 0.0f, 0.0f);
	Convex_Shape.at(shape_index)->calculateLocalInertia(convexMass, localInertia);

	int index = ObjectsMotionState.size()-1;
    btRigidBody::btRigidBodyConstructionInfo Convex_Body_CI(convexMass,ObjectsMotionState.at(index),Convex_Shape.at(shape_index),btVector3(0,0,0));
	Objects_Body.push_back(new btRigidBody(Convex_Body_CI));
	m_dynamicsWorld->addRigidBody(Objects_Body.at(index));
	Objects_Body.at(index)->setCenterOfMassTransform(btTransform(btQuaternion(0,0,-1,1),btVector3(25,-5,10)));
	return index;
}


btConvexHullShape* KCRPhysicsWorld::get_Convex_Shape(int index) {
	return Convex_Shape.at(index);
}

btTransform KCRPhysicsWorld::get_Object_Transform(int index) {
	btTransform trans;
	Objects_Body.at(index)->getMotionState()->getWorldTransform(trans);
	return trans;
}

int KCRPhysicsWorld::Num_Object() {
	return Objects_Body.size();
}

void KCRPhysicsWorld::set_center_trimesh(float x, float y) {
	center_trimesh.setX(x);
	center_trimesh.setY(y);
}

void KCRPhysicsWorld::clearTrimesh() {
	for (int i = 0; i < HF_Size; i++) {
		m_rawHeightfieldData[i] = 0;
	}
}

void KCRPhysicsWorld::updateTrimesh(float* hf) {
	 memcpy(m_rawHeightfieldData, hf, sizeof(m_rawHeightfieldData));
/*	for (int i = 0; i < HF_Size; i++) {
		if(m_rawHeightfieldData[i] != hf[i])
			m_rawHeightfieldData[i] = hf[i];
	}
	*/
}

void KCRPhysicsWorld::updateTrimeshRefitTree(float* hf) {
	for (int i = 0; i < HF_Size; i++) {
		if(m_rawHeightfieldData[i] != hf[i])
			m_rawHeightfieldData[i] = hf[i];
	}

	//btVector3 aabbMin(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
	//btVector3 aabbMax(-BT_LARGE_FLOAT,-BT_LARGE_FLOAT,-BT_LARGE_FLOAT);
	for (int i=0;i<NUM_VERTS_X;i++) {
		for (int j=0;j<NUM_VERTS_Y;j++) {	
//			worldMax.setMax(m_vertices[i+j*NUM_VERTS_X]); worldMin.setMin(m_vertices[i+j*NUM_VERTS_X]);
			float height = m_rawHeightfieldData[j*160+i];
			m_vertices[i+j*NUM_VERTS_X].setZ(height);					
//			worldMin.setMin(m_vertices[i+j*NUM_VERTS_X]); worldMax.setMax(m_vertices[i+j*NUM_VERTS_X]);
			}
		}
		//trimeshShape->partialRefitTree(aabbMin,aabbMax);
		//trimeshShape->partialRefitTree(worldMin,worldMax);
		trimeshShape->refitTree(worldMin,worldMax);
		m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(groundRigidBody->getBroadphaseHandle(),m_dynamicsWorld->getDispatcher());

}

void KCRPhysicsWorld::createWorldSphereProxy() {

	float scale = wscale;
	float sphereMass = 10;
	Sphere_Shape =  new btSphereShape(1);
	m_collisionShapes.push_back(Sphere_Shape);
	btVector3 localInertia(0.0f, 0.0f, 0.0f);
	Sphere_Shape->calculateLocalInertia(sphereMass, localInertia);

	WorldSphereProxyMotionState.reserve(1200);
	WorldSphereProxyRigidBody.reserve(1200);

	for(int i = 0; i < 30; i++) {
		for(int j = 0; j < 40; j++) {
			int index = i*40 + j;
			float x = (4*j-center_trimesh.getX())*scale;
			float y = (4*i-(120-center_trimesh.getY()))*scale;
			//WorldSphereProxyMotionState.push_back(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0))));
			WorldSphereProxyMotionState.push_back(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(x,y,1))));
			btRigidBody::btRigidBodyConstructionInfo Sphere_Body_CI(sphereMass,WorldSphereProxyMotionState.at(index),Sphere_Shape,localInertia);
			WorldSphereProxyRigidBody.push_back(new btRigidBody(Sphere_Body_CI));
			m_dynamicsWorld->addRigidBody(WorldSphereProxyRigidBody.at(index));
			//WorldSphereProxyRigidBody.at(index)->setCenterOfMassTransform(btTransform(btQuaternion(0,0,0,1),btVector3(x, y, 1)));

			//assign sphere as kinematics
//			WorldSphereProxyRigidBody.at(index)->setCollisionFlags( WorldSphereProxyRigidBody.at(index)->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);//2
//			WorldSphereProxyRigidBody.at(index)->setActivationState(DISABLE_DEACTIVATION);//4	
		}
	}

}

btTransform KCRPhysicsWorld::getWorldSphereTransform(int index) {
	btTransform trans;
	WorldSphereProxyRigidBody.at(index)->getMotionState()->getWorldTransform(trans);
	return trans;
}

void KCRPhysicsWorld::updateWorldSphereTransform(float *voxel) {
	for(int i = 0; i < 30; i++) {
		for(int j = 0; j < 40; j++) {
			int index = i*40 + j;
			if(voxel[index] > 0) {
				btTransform trans;
				WorldSphereProxyRigidBody.at(index)->getMotionState()->getWorldTransform(trans);
				trans.getOrigin().setZ(voxel[index]);
				WorldSphereProxyRigidBody.at(index)->getMotionState()->setWorldTransform(trans);
			}
		}
	}
}