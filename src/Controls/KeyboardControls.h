#ifndef KEYBOARD_CONTROLS_H
#define KEYBOARD_CONTROLS_H

#include "Controls.h"
#include "src\UserConstant.h"

//OpenCV
#include "opencv\cv.h"

extern int collide_counter;
extern double prev_collide_clock;
extern interaction interact_state;

const char * DATABASEDIR = "../../../Dropbox/Lab/ModelDatabase/";

double x = 0, y=0, z=0;
class KeyboardController: public Controller 
{
public:
	KeyboardController(bt_ARMM_world *m_world):Controller(m_world) {};
	void PressB()
	{
		// Add box shape objects
		osgAddObjectNode(world->CreateSoftTexture("Data/tex.bmp"));
		cout << "create the soft body object" << endl;
	}

	void PressN()
	{
		int index = world->create_Sphere();
		osgAddObjectNode(osgNodeFromBtSphere(SPHERE_SIZE, world->get_Object_Transform(index)));
		Virtual_Objects_Count++;
	}

	int check_input() 
	{

#if CAR_SIMULATION == 1
		//Car Number 1
		if (getKey(VK_UP)) {
			world->accelerateEngine(0);
			world->ChangeRotation(10,0,0);
		} else if (getKey(VK_DOWN)) {
			world->decelerateEngine(0);
			world->ChangeRotation(-10,0,0);	
		} else {
			world->resetEngineForce(0);
		}
		if (getKey(VK_LEFT)) {
			world->turnEngineLeft(0);
			world->ChangeRotation(0,10,0);
		} else if (getKey(VK_RIGHT)) {
			world->turnEngineRight(0);
			world->ChangeRotation(0,-10,0);
		} else {
			 world->turnReset(0);
		}
		//Car Number 2
		if (getKey(87)) {//W
			world->accelerateEngine(1);
			world->ToggleSphereRep();
		} else if (getKey(83)) {//S
			world->decelerateEngine(1);
			//world->ChangeRotation(0,45,0);
		} else {
			world->resetEngineForce(1);
		}
		if (getKey(65)) {//A
			world->turnEngineLeft(1);
			x += 0.1;
			world->ChangeRotation(x,y,z);
		} else if (getKey(68)) {//D
			world->turnEngineRight(1);
			x -= 0.1;
			world->ChangeRotation(x,y,z);
		} else {
			 world->turnReset(1);
		}
#endif /* CAR_SIMULATION == 1 */

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
				PressB();
				return 66;
			}
			if (getKey(77)) { //M
				//interact_state = PINCH;
				return 77;
			}
			if (getKey(79)) { //o
				string modelname(DATABASEDIR);
				modelname+="ItimatsuCow/ItimatsuCow.3ds";
				int index = world->create_3dsmodel(modelname.c_str());
				osgAddObjectNode(osgNodeFrom3dsModel(world->GetModelName(), 0.005, world->get_Object_Transform(index)));
				Virtual_Objects_Count++;
				world->ChangeAttribute(25, -5, 5, index);
								
				return 79;
			}

			if (getKey(80)) { //p
				string modelname(DATABASEDIR);
				modelname+="keyboard/keyboard.3ds";
				//string modelname = "HatuneMiku.3ds";
				int index = world->create_3dsmodel(modelname.c_str());
				osgAddObjectNode(osgNodeFrom3dsModel(world->GetModelName(), world->get3dsScale(), world->get_Object_Transform(index)));
				Virtual_Objects_Count++;
				world->ChangeAttribute(15, -5, 5, index);

				return 80;
			}

			if (getKey(78) ) { //N
				PressN();
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

	int TransmitInput(const int & input);

private:
	inline bool getKey(int key) { return GetAsyncKeyState(key)& 0x8000; }
};

int KeyboardController::TransmitInput(const int & input)
{
	const int transmitOffset = 200;
	return (input>0? (transmitOffset+input) : 0);
}

#endif