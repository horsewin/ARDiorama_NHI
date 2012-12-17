
#include "bt_ARMM_hand.h"
#include <iostream>

#define CUBE_HALF_EXTENTS 1
#define SIM_MICROMACHINE 1
//#define SIM_PARTICLES 1

using namespace std;
//#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"

//TODO
//btScalar HAND_SPHERE_SIZE = 0.57;

bt_ARMM_hand::bt_ARMM_hand(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*> m_collisionShapes, float world_scale, float global_x, float global_y, int sphere_resolution, float ratio, float centerx, float centery)
	: cx(centerx), cy(centery)
{
	printf("World Scale = %f, Ratio=%f\n",world_scale, ratio);
	num_sphere_x = sphere_resolution;
	num_sphere_y = sphere_resolution;
	num_sphere_total = num_sphere_x*num_sphere_y;
	hand_depth_grid = new float[num_sphere_total];
	world_scale_hand = world_scale;

	float sphereMass = 0;
	//Sphere_Shape =  new btSphereShape(world_scale_hand); //test
	Sphere_Shape =  new btSphereShape(world_scale_hand*ratio);
//	Sphere_Shape =  new btSphereShape(HAND_SPHERE_SIZE);

	m_collisionShapes.push_back(Sphere_Shape);
	btVector3 localInertia(0.0f, 0.0f, 0.0f);
	Sphere_Shape->calculateLocalInertia(sphereMass, localInertia);

	handSphereMotionState.reserve(num_sphere_total);
	handSphereRigidBody.reserve(num_sphere_total);

	for(int i = 0; i < num_sphere_y; i++) {
		for(int j = 0; j < num_sphere_x; j++) {
			int index = i*num_sphere_x + j;
			float x = (global_x+i*world_scale_hand);
			float y = (global_y+j*world_scale_hand);
			//WorldSphereProxyMotionState.push_back(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0))));
			handSphereMotionState.push_back(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(x,y,0))));
			btRigidBody::btRigidBodyConstructionInfo Sphere_Body_CI(sphereMass,handSphereMotionState.at(index),Sphere_Shape,localInertia);
			handSphereRigidBody.push_back(new btRigidBody(Sphere_Body_CI));
			//handSphereRigidBody.at(index)->setFriction(btScalar(0.9));

			m_dynamicsWorld->addRigidBody(handSphereRigidBody.at(index));

			handSphereRigidBody.at(index)->setFriction(1);
			handSphereRigidBody.at(index)->setRestitution(0);

			//handSphereRigidBody.at(index)->setCenterOfMassTransform(btTransform(btQuaternion(0,0,0,1),btVector3(x, y, 1)));
			//assign sphere as kinematics
			if(handSphereRigidBody.at(index)->getCollisionFlags() != 0 ) {
				handSphereRigidBody.at(index)->setCollisionFlags( handSphereRigidBody.at(index)->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);//2
				handSphereRigidBody.at(index)->setActivationState(DISABLE_DEACTIVATION);//4	
			}
		}
	}

	prev_x_corner = global_x;
	prev_y_corner = global_y;
}

bt_ARMM_hand::~bt_ARMM_hand(void) {}

void bt_ARMM_hand::Update(float global_x, float global_y, float curr_hands_ratio, float* depth_grid)
{
	int hand_area = 0;
	for(int i = 0; i < num_sphere_y; i++) {
		for(int j = 0; j < num_sphere_x; j++) {
			int index = i*num_sphere_x + j;
			if(handSphereRigidBody.at(index)->getCollisionFlags() != 0 ) {
				handSphereRigidBody.at(index)->setCollisionFlags( handSphereRigidBody.at(index)->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);//2
				handSphereRigidBody.at(index)->setActivationState(DISABLE_DEACTIVATION);//4	
			
				btTransform trans;
				handSphereRigidBody.at(index)->getMotionState()->getWorldTransform(trans);
				//float x = (global_x+i*curr_hands_ratio * world_scale_hand);
				//float y = (global_y+j*curr_hands_ratio * world_scale_hand);
				//trans.getOrigin().setX(x);
				//trans.getOrigin().setY(y);
				if(depth_grid[index] > 0){
					float x = (float) (global_x+i*curr_hands_ratio - cx)*world_scale_hand; //test
					float y = (float) (global_y+j*curr_hands_ratio - (120-cy))*world_scale_hand; //test
					trans.getOrigin().setX(x);
					trans.getOrigin().setY(y);
					trans.getOrigin().setZ(depth_grid[index]);
					hand_area++;
				}else{
					//float x = (float) (global_x+i*curr_hands_ratio - cx)*world_scale_hand; //test
					//float y = (float) (global_y+j*curr_hands_ratio - (120-cy))*world_scale_hand; //test
					//trans.getOrigin().setX(x);
					//trans.getOrigin().setY(y);
					//trans.getOrigin().setZ(0);
					trans.getOrigin().setX(0);
					trans.getOrigin().setY(0);
					trans.getOrigin().setZ(5000);
				}
				handSphereRigidBody.at(index)->setFriction(1);
				handSphereRigidBody.at(index)->setRestitution(0);
				handSphereRigidBody.at(index)->getMotionState()->setWorldTransform(trans);
				//handSphereRigidBody.at(index)->setLinearVelocity(trans.getOrigin());
				handSphereRigidBody.at(index)->setInterpolationLinearVelocity(btVector3(0,0,0));
				handSphereRigidBody.at(index)->setDamping(1.0f,1.0f);
			} 
			else 
			{
				handSphereRigidBody.at(index)->setCollisionFlags(0);
				handSphereRigidBody.at(index)->setActivationState(1);
				handSphereRigidBody.at(index)->setFriction(1);
				handSphereRigidBody.at(index)->setRestitution(0);
				//handSphereRigidBody.at(index)->setDamping(0.0f,0.0f);
				handSphereRigidBody.at(index)->setInterpolationLinearVelocity(btVector3(0,0,0));
			}
		}
	}
	//cout << hand_area << endl;
//	cout << "Current Ratio = " << curr_hands_ratio << endl;
	prev_x_corner = global_x;
	prev_y_corner = global_y;
}

//Only need to return the lower-left sphere transform and update other sphere accordingly
btTransform bt_ARMM_hand::getHandSphereTransform() {
	btTransform trans;
	handSphereRigidBody.at(0)->getMotionState()->getWorldTransform(trans);
	return trans;
}

int bt_ARMM_hand::get_Num_Sphere_X() {
	return num_sphere_x;
}
int bt_ARMM_hand::get_Num_Sphere_Y() {
	return num_sphere_y;
}
int bt_ARMM_hand::get_Total_Num_Sphere() {
	return num_sphere_total;
}

float* bt_ARMM_hand::debugHandX() {
	float* x_grid =  new float[num_sphere_total];
	for(int i = 0; i < num_sphere_y; i++) {
		for(int j = 0; j < num_sphere_x; j++) {
			int index = i*num_sphere_x + j;
			btTransform trans;
			handSphereRigidBody.at(index)->getMotionState()->getWorldTransform(trans);
			x_grid[index] = trans.getOrigin().getX();
		}
	}
	return x_grid;
}
float* bt_ARMM_hand::debugHandY() {
	float* y_grid =  new float[num_sphere_total];	
	for(int i = 0; i < num_sphere_y; i++) {
		for(int j = 0; j < num_sphere_x; j++) {
			int index = i*num_sphere_x + j;
			btTransform trans;
			handSphereRigidBody.at(index)->getMotionState()->getWorldTransform(trans);
			y_grid[index] = trans.getOrigin().getY();
		}
	}
	return y_grid;
}

float* bt_ARMM_hand::debugHandZ() {
	
	for(int i = 0; i < num_sphere_y; i++) {
		for(int j = 0; j < num_sphere_x; j++) {
			int index = i*num_sphere_x + j;
			btTransform trans;
			handSphereRigidBody.at(index)->getMotionState()->getWorldTransform(trans);
			hand_depth_grid[index] = trans.getOrigin().getZ();
		}
	}
	return hand_depth_grid;
}

// this one is not used already
void bt_ARMM_hand::Update(float global_x, float global_y, float *depth_grid) 
{
	for(int i = 0; i < num_sphere_y; i++) {
		for(int j = 0; j < num_sphere_x; j++) {
			int index = i*num_sphere_x + j;
			if(handSphereRigidBody.at(index)->getCollisionFlags() != 0 ) {
				handSphereRigidBody.at(index)->setCollisionFlags( handSphereRigidBody.at(index)->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);//2
				handSphereRigidBody.at(index)->setActivationState(DISABLE_DEACTIVATION);//4	
			
				btTransform trans;
				handSphereRigidBody.at(index)->getMotionState()->getWorldTransform(trans);
				float x = (global_x+i*world_scale_hand);
				float y = (global_y+j*world_scale_hand);
				trans.getOrigin().setX(x);
				trans.getOrigin().setY(y);
        trans.getOrigin().setZ(depth_grid[index]>0 ? depth_grid[index]-1 : 1000);

        handSphereRigidBody.at(index)->getMotionState()->setWorldTransform(trans);
				handSphereRigidBody.at(index)->setRestitution(0);
				handSphereRigidBody.at(index)->setLinearVelocity(btVector3(0,0,0));
			}
      else{
				handSphereRigidBody.at(index)->setCollisionFlags(0);
				handSphereRigidBody.at(index)->setActivationState(1);
				handSphereRigidBody.at(index)->setFriction(FRICTION_);
				handSphereRigidBody.at(index)->setRestitution(0);
				handSphereRigidBody.at(index)->setLinearVelocity(btVector3(0,0,0));
			}
		}
	}
	prev_x_corner = global_x;
	prev_y_corner = global_y;
}