#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/StateSet>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/PositionAttitudeTransform>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>

#include "UserConstant.h"

/*
//osgbullet
#include <osgbDynamics/MotionState.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbCollision/Utils.h>
*/
extern const float MIN_HAND_PIX;

osg::Geode *create3dsModel() {
	// vertex array
	osg::Vec3Array *vertexArray = new osg::Vec3Array();
		// face array
	osg::DrawElementsUInt *faceArray = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
		// normal array
	osg::Vec3Array *normalArray = new osg::Vec3Array();

	// normal index
	osg::TemplateIndexArray<unsigned int, osg::Array::UIntArrayType, 24, 4> *normalIndexArray;
	normalIndexArray = new osg::TemplateIndexArray<unsigned int, osg::Array::UIntArrayType, 24, 4>();

	osg::Geometry *geometry = new osg::Geometry();
	geometry->setVertexArray(vertexArray);

	geometry->setNormalArray(normalArray);
	geometry->setNormalIndices(normalIndexArray);
	geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
	//geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
	geometry->addPrimitiveSet(faceArray);

	osg::Vec4Array* color = new osg::Vec4Array();     
	//color->push_back( osg::Vec4( std::rand(), std::rand(), std::rand(), 1. ) );    
	color->push_back( osg::Vec4( 1, 0, 0, 0.5 ) );    
	geometry->setColorArray( color );
	geometry->setColorBinding( osg::Geometry::BIND_OVERALL );

	osg::Geode *cube = new osg::Geode();
	cube->addDrawable(geometry);
	return cube;
}

osg::Geode *createCube() {
	// vertex array
	osg::Vec3Array *vertexArray = new osg::Vec3Array();

	// bottom front left
	vertexArray->push_back(osg::Vec3(-1, -1, -1));
	vertexArray->push_back(osg::Vec3(-1, -1, -1));
	vertexArray->push_back(osg::Vec3(-1, -1, -1));
	// bottom front right
	vertexArray->push_back(osg::Vec3(+1, -1, -1));
	vertexArray->push_back(osg::Vec3(+1, -1, -1));
	vertexArray->push_back(osg::Vec3(+1, -1, -1));
	// bottom back right
	vertexArray->push_back(osg::Vec3(+1, +1, -1));
	vertexArray->push_back(osg::Vec3(+1, +1, -1));
	vertexArray->push_back(osg::Vec3(+1, +1, -1));
	// bottom back left
	vertexArray->push_back(osg::Vec3(-1, +1, -1));
	vertexArray->push_back(osg::Vec3(-1, +1, -1));
	vertexArray->push_back(osg::Vec3(-1, +1, -1));

	// top front left
	vertexArray->push_back(osg::Vec3(-1, -1,  1));
	vertexArray->push_back(osg::Vec3(-1, -1,  1));
	vertexArray->push_back(osg::Vec3(-1, -1,  1));
	// top front right
	vertexArray->push_back(osg::Vec3(+1, -1,  1));
	vertexArray->push_back(osg::Vec3(+1, -1,  1));
	vertexArray->push_back(osg::Vec3(+1, -1,  1));
	// top back right
	vertexArray->push_back(osg::Vec3(+1, +1,  1));
	vertexArray->push_back(osg::Vec3(+1, +1,  1));
	vertexArray->push_back(osg::Vec3(+1, +1,  1));
	// top back left
	vertexArray->push_back(osg::Vec3(-1, +1,  1));
	vertexArray->push_back(osg::Vec3(-1, +1,  1));
	vertexArray->push_back(osg::Vec3(-1, +1,  1));


	// face array
	osg::DrawElementsUInt *faceArray = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);

	// bottom
	faceArray->push_back(0); // face 1
	faceArray->push_back(9);
	faceArray->push_back(3);
	faceArray->push_back(9); // face 2
	faceArray->push_back(6);
	faceArray->push_back(3);
	// top
	faceArray->push_back(21);  //face 3
	faceArray->push_back(12);
	faceArray->push_back(18);
	faceArray->push_back(12);  //face 4
	faceArray->push_back(15);
	faceArray->push_back(18);
	// left
	faceArray->push_back(22);  //face 5
	faceArray->push_back(10);
	faceArray->push_back(13);
	faceArray->push_back(10);  //face 6
	faceArray->push_back(1);
	faceArray->push_back(13);
	// right
	faceArray->push_back(16);  //face 7
	faceArray->push_back(4);
	faceArray->push_back(19);
	faceArray->push_back(4);  //face 8
	faceArray->push_back(7);
	faceArray->push_back(19);
	// front
	faceArray->push_back(14);  //face 9
	faceArray->push_back(2);
	faceArray->push_back(17);
	faceArray->push_back(2);   //face 10
	faceArray->push_back(5);
	faceArray->push_back(17);
	// back
	faceArray->push_back(20);  //face 11
	faceArray->push_back(8);
	faceArray->push_back(23);
	faceArray->push_back(8);   //face 12
	faceArray->push_back(11);
	faceArray->push_back(23);

	// normal array
	osg::Vec3Array *normalArray = new osg::Vec3Array();
	normalArray->push_back(osg::Vec3(+1, 0, 0));
	normalArray->push_back(osg::Vec3(-1, 0, 0));
	normalArray->push_back(osg::Vec3(0, +1, 0));
	normalArray->push_back(osg::Vec3(0, -1, 0));
	normalArray->push_back(osg::Vec3(0, 0, +1));
	normalArray->push_back(osg::Vec3(0, 0, -1));

	// normal index
	osg::TemplateIndexArray<unsigned int, osg::Array::UIntArrayType, 24, 4> *normalIndexArray;
	normalIndexArray = new osg::TemplateIndexArray<unsigned int, osg::Array::UIntArrayType, 24, 4>();

	// bottom front left					
	normalIndexArray->push_back(5);
	normalIndexArray->push_back(3);
	normalIndexArray->push_back(0);
	// bottom front right
	normalIndexArray->push_back(5);
	normalIndexArray->push_back(2);
	normalIndexArray->push_back(0);
	// bottom back right
	normalIndexArray->push_back(5);
	normalIndexArray->push_back(2);
	normalIndexArray->push_back(1);
	// bottom back left
	normalIndexArray->push_back(5);
	normalIndexArray->push_back(3);
	normalIndexArray->push_back(1);

	// top front left					
	normalIndexArray->push_back(4);
	normalIndexArray->push_back(3);
	normalIndexArray->push_back(0);
	// top front right
	normalIndexArray->push_back(4);
	normalIndexArray->push_back(2);
	normalIndexArray->push_back(0);
	// top back right
	normalIndexArray->push_back(4);
	normalIndexArray->push_back(2);
	normalIndexArray->push_back(1);
	// top back left
	normalIndexArray->push_back(4);
	normalIndexArray->push_back(3);
	normalIndexArray->push_back(1);

	osg::Geometry *geometry = new osg::Geometry();
	geometry->setVertexArray(vertexArray);

	geometry->setNormalArray(normalArray);
	geometry->setNormalIndices(normalIndexArray);
	geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
	//geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
	geometry->addPrimitiveSet(faceArray);

	osg::Vec4Array* color = new osg::Vec4Array();     
	//color->push_back( osg::Vec4( std::rand(), std::rand(), std::rand(), 1. ) );    
	color->push_back( osg::Vec4( 1, 0, 0, 0.5 ) );    
	geometry->setColorArray( color );
	geometry->setColorBinding( osg::Geometry::BIND_OVERALL );

	osg::Geode *cube = new osg::Geode();
	cube->addDrawable(geometry);
	return cube;
}

osg::Vec3 asOsgVec3( const btVector3& v )  {
	return osg::Vec3( v.x(), v.y(), v.z() ); 
}  


osg::Node* osgNodeFromBtBoxShape(float cube_size, const btTransform& trans) {
	//osg::ref_ptr< osg::Geode > cube = createCube();
	float scale = 10;
	float x = trans.getOrigin().getX()*scale;
	float y = trans.getOrigin().getY()*scale;
	float z = trans.getOrigin().getZ()*scale;
	btQuaternion quat = trans.getRotation();
	osg::Quat q = osg::Quat(quat.getX(), quat.getY(), quat.getZ(), quat.getW());
	osg::ref_ptr< osg::Box > cube = new osg::Box(osg::Vec3d(0,0,0),cube_size);
	osg::ShapeDrawable * shape = new osg::ShapeDrawable( cube );
    osg::ref_ptr< osg::Geode> geode = new osg::Geode();
    geode->addDrawable( shape );
	osg::ref_ptr< osg::PositionAttitudeTransform > mt = new osg::PositionAttitudeTransform();
	mt->setScale(osg::Vec3d(scale,scale,scale));
	mt->setAttitude(q);
	mt->setPosition(osg::Vec3d(x, y, z));
	mt->addChild( geode.get() );
	return mt.release();
}

//Convert from btConvexHullShape into osg's node
osg::Node* osgNodeFromBtCollisionShape( const btConvexHullShape* hull, const btTransform& trans )  {
	btShapeHull sh( hull );
	sh.buildHull( 0. );
	int nVerts( sh.numVertices () );
	int nIdx( sh.numIndices () ); 
	if( (nVerts <= 0) || (nIdx <= 0) )     
		return( NULL );       
	const btVector3* bVerts( sh.getVertexPointer() );
	const unsigned int* bIdx( sh.getIndexPointer() );
	osg::Vec3Array* v = new osg::Vec3Array();
	v->resize( nVerts );
	unsigned int idx;  
	for( idx = 0; idx < (unsigned int)nVerts; idx++ )        
		( *v )[ idx ] = asOsgVec3( bVerts[ idx ] );

	osg::DrawElementsUInt* deui = new osg::DrawElementsUInt( GL_TRIANGLES );   

	for( idx = 0; idx < (unsigned int)nIdx; idx++ )         
		deui->push_back( bIdx[ idx ] );      

	osg::Vec4Array* color = new osg::Vec4Array();     
	color->push_back( osg::Vec4( 1., 1., 1., 1. ) );    

	osg::Geometry* geom = new osg::Geometry;     
	geom->setVertexArray( v );
	geom->setColorArray( color );
	geom->setColorBinding( osg::Geometry::BIND_OVERALL );      
	geom->addPrimitiveSet( deui );
	osg::ref_ptr< osg::Geode > geode = new osg::Geode();   
	geode->addDrawable( geom ); 
	btQuaternion quat = trans.getRotation();
	osg::Quat q = osg::Quat(quat.getX(), quat.getY(), quat.getZ(), quat.getW());
	float scale = 10;
	float x = trans.getOrigin().getX()*scale;
	float y = trans.getOrigin().getY()*scale;
	float z = trans.getOrigin().getZ()*scale;
	osg::ref_ptr< osg::PositionAttitudeTransform > mt = new osg::PositionAttitudeTransform();
	mt->setScale(osg::Vec3d(scale,scale,scale));
	mt->setAttitude(q);
	mt->setPosition(osg::Vec3d(x, y, z));
	mt->addChild( geode.get() );
	return mt.release();
}  

osg::Node* osgNodeFromBtSphere(float sphere_size, const btTransform& trans) {
	float scale = 10;
	float x = trans.getOrigin().getX()*scale;
	float y = trans.getOrigin().getY()*scale;
	float z = trans.getOrigin().getZ()*scale;
	btQuaternion quat = trans.getRotation();
	osg::Quat q = osg::Quat(quat.getX(), quat.getY(), quat.getZ(), quat.getW());
	osg::ref_ptr< osg::Sphere > sphere = new osg::Sphere(osg::Vec3d(0,0,0), sphere_size);
	osg::ShapeDrawable * shape = new osg::ShapeDrawable( sphere );
    osg::ref_ptr< osg::Geode> geode = new osg::Geode();
    geode->addDrawable( shape );
	osg::ref_ptr< osg::PositionAttitudeTransform > mt = new osg::PositionAttitudeTransform();
	mt->setScale(osg::Vec3d(scale,scale,scale));
	mt->setAttitude(q);
	mt->setPosition(osg::Vec3d(x, y, z));
	mt->addChild( geode.get() );
	return mt.release();
}

osg::Node* osgNodeFrom3dsModel(std::string modelname, const double & scale_3ds, const btTransform& trans) 
{
	//cout << "object scale = " << scale_3ds << endl;
	float scale = 10;
	float x = trans.getOrigin().getX()*scale;
	float y = trans.getOrigin().getY()*scale;
	float z = trans.getOrigin().getZ()*scale;
	btQuaternion quat = trans.getRotation();
	osg::Quat q = osg::Quat(quat.getX(), quat.getY(), quat.getZ(), quat.getW());

	osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(modelname.c_str());

	//osg::ref_ptr< osg::Sphere > sphere = new osg::Sphere(osg::Vec3d(0,0,0), sphere_size);
	//osg::ShapeDrawable * shape = new osg::ShapeDrawable( sphere );
 //   osg::ref_ptr< osg::Geode> geode = new osg::Geode();
 //   geode->addDrawable( shape );
	osg::ref_ptr< osg::PositionAttitudeTransform > mt = new osg::PositionAttitudeTransform();
	//scale = scale_3ds * 10; //this scale set 10 times value against bullet scale(ref:bt_ARMM_world.cpp)
	mt->setScale(osg::Vec3d(scale,scale,scale));
	mt->setAttitude(q);
	mt->setPosition(osg::Vec3d(x, y, z));
	mt->addChild(model);

	return mt.release();
}
