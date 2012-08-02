#ifndef KEYBOARD_CONTROLS_H
#define KEYBOARD_CONTROLS_H

#define SIM_MICROMACHINE 1

#include "./Controls/Controls.h"

//OpenCV
#include "opencv\cv.h"

extern int collide_counter;
extern double prev_collide_clock;

class KeyboardController: public Controller {
public:
	KeyboardController(bt_ARMM_world *m_world):Controller(m_world) {};
	int check_input() {

#ifdef SIM_MICROMACHINE
		//Car Number 1
		if (getKey(VK_UP)) {
			world->accelerateEngine(0);
		} else if (getKey(VK_DOWN)) {
			world->decelerateEngine(0);
		} else {
			world->resetEngineForce(0);
		}
		if (getKey(VK_LEFT)) {
			world->turnEngineLeft(0);
		} else if (getKey(VK_RIGHT)) {
			world->turnEngineRight(0);
		} else {
			 world->turnReset(0);
		}
		//Car Number 2
		if (getKey(87)) {//W
			world->accelerateEngine(1);
			world->ToggleSphereRep();
		} else if (getKey(83)) {//S
			world->decelerateEngine(1);
		} else {
			world->resetEngineForce(1);
		}
		if (getKey(65)) {//A
			world->turnEngineLeft(1);
		} else if (getKey(68)) {//D
			world->turnEngineRight(1);
		} else {
			 world->turnReset(1);
		}
#endif /*SIM_MICROMACHINE*/

		//A 65 S 83 D 68 W 87 F 70 V 86
		if (getKey(VK_ESCAPE)) running = false;
#ifdef SIM_MICROMACHINE
		if (getKey(82)) world->resetCarScene(0); //R
		if (getKey(84)) world->resetCarScene(1); //T
#endif /*SIM_MICROMACHINE*/

		if (getKey(86)) { //V
			WIREFRAME_MODE = !WIREFRAME_MODE;
			if(WIREFRAME_MODE) {
				HeightFieldGeometry_line->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::OFF);	
				groundQuadColor->pop_back();
				groundQuadColor->push_back(osg::Vec4(1,1,1,0.2));
				groundLineColor->pop_back();
				groundLineColor->push_back(osg::Vec4(1,0.2,0,1.0));
			} else {
				HeightFieldGeometry_line->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);	
				groundQuadColor->pop_back();
				groundQuadColor->push_back(osg::Vec4(1,1,1,0.0));
				groundLineColor->pop_back();
				groundLineColor->push_back(osg::Vec4(1,1,1,0.0));
			}
			printf("Wireframe Mode = %d \n",WIREFRAME_MODE);
			return 86;
		}
#ifdef SIM_MICROMACHINE
		if(Virtual_Objects_Count < MAX_NUM_VIR_OBJ) {
			if (getKey(66)) { //B
				// Add box shape objects
				//int index = world->create_Box();
				//osgAddObjectNode(osgNodeFromBtBoxShape(CUBE_SIZE,world->get_Object_Transform(index)));
				osgAddObjectNode(world->CreateSoftTexture(""));
				cout << "create the soft body object" << endl;
				//Virtual_Objects_Count++;
				return 66;
			}
			if (getKey(77)) { //M
				world->ChangeAttribute();
				return 77;
			}
			if (getKey(79)) { //o
				int index = world->create_Box();
				osgAddObjectNode(osgNodeFromBtBoxShape(CUBE_SIZE,world->get_Object_Transform(index)));
				Virtual_Objects_Count++;
				//world->ChangeAttribute(30);
		        return 79;
			}

			if (getKey(80)) { //p
				string modelname = "Data/Cars/HatuneMiku.3ds";
				int index = world->create_3dsmodel(modelname.c_str());
				osgAddObjectNode(osgNodeFrom3dsModel(world->GetModelName(), world->get3dsScale(), world->get_Object_Transform(index)));
				cout << "Loading..." << endl;

				Virtual_Objects_Count++;
				//world->ChangeAttribute(10);
				return 80;
			}

			if (getKey(78) || (collide_counter%3 == 0 && collide_counter != 0) ) { //N
			//if (getKey(78) ) { //N
				int index = world->create_Sphere();
				osgAddObjectNode(osgNodeFromBtSphere(SPHERE_SIZE, world->get_Object_Transform(index)));
				Virtual_Objects_Count++;

				prev_collide_clock = static_cast<double>(cv::getTickCount());
				collide_counter++;
				return 78;
			}
		}
#endif /*SIM_MICROMACHINE*/
		if (getKey(VK_SPACE)) {
			registerMarker();
		}

		if (getKey(VK_RETURN)) {
			if (kinectTransform) {
				CvFileStorage *fs = cvOpenFileStorage(KINECT_TRANSFORM_FILENAME, 0, CV_STORAGE_WRITE);
				cvStartWriteStruct(fs, "MarkerSize", CV_NODE_MAP); 
					cvWriteInt(fs, "width", markerSize.width);
					cvWriteInt(fs, "height", markerSize.height);
				cvEndWriteStruct(fs);

				cvStartWriteStruct(fs, "MarkerOrigin", CV_NODE_MAP); 
					cvWriteInt(fs, "x", marker_origin.x);
					cvWriteInt(fs, "y", marker_origin.y);
				cvEndWriteStruct(fs);

				cvWriteReal(fs, "WorldScale", WORLD_SCALE);
				cvWriteReal(fs, "WorldAngle", WORLD_ANGLE);
				cvWriteReal(fs, "MARKER_DEPTH", MARKER_DEPTH);

				cvWrite(fs, "KinectTransform", kinectTransform);
				cvReleaseFileStorage( &fs );
				printf("Saved Kinect Transform\n");
			}
		}
		return 0;
	}

	void check_input(int pass) {
			//if (pass == 78) { //N
			//	int index = world->create_Triangular_Ramp();
			//	osgAddObjectNode(osgNodeFromSimpleTriangularRampShape(world->get_Object_Transform(index)));
			//	Virtual_Objects_Count++;
			//}
		}

private:
	inline bool getKey(int key) { return GetAsyncKeyState(key)& 0x8000; }
};

#endif