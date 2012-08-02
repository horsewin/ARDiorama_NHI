#ifndef BT_ARMM_HAND_H
#define BT_ARMM_HAND_H

class btCollisionShape;

#include "btBulletDynamicsCommon.h"
#include <vector>

//#define USE_CCD 1
#define FRICTION_ 500.0f

class bt_ARMM_hand {
	public:

		btSphereShape* Sphere_Shape;
		std::vector<btDefaultMotionState*> handSphereMotionState;//m_WorldSphereProxyMotionState
		std::vector<btRigidBody*> handSphereRigidBody;//WorldSphereProxyRigidBody

		bt_ARMM_hand(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*> m_collisionShapes, 
						float world_scale, float global_x, float global_y, int sphere_resolution, float ratio, float centerx, float centery);
		virtual ~bt_ARMM_hand(void);
		virtual void Update(float global_x, float global_y, float *depth_grid);
		virtual void Update(float global_x, float global_y, float curr_hands_ratio, float* depth_grid);
		btTransform getHandSphereTransform();
		int get_Num_Sphere_X();
		int get_Num_Sphere_Y();
		int get_Total_Num_Sphere();
		float* debugHandX();
		float* debugHandY();
		float* debugHandZ();

private:
		int num_sphere_x;
		int num_sphere_y;
		int num_sphere_total;
		float	worldDepth;
		float	world_scale_hand;
		float	prev_x_corner;
		float	prev_y_corner;
		float	*hand_depth_grid;
		double cx, cy;
};

#endif //BT_ARMM_HAND_H