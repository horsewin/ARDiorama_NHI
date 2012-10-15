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

osg::Geode *create3dsModel() 
{
	osg::Vec3Array			*vertexArray	= new osg::Vec3Array();
	osg::DrawElementsUInt	*faceArray		= new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
	osg::Vec3Array			*normalArray	= new osg::Vec3Array();

	// normal index
	osg::TemplateIndexArray<unsigned int, osg::Array::UIntArrayType, 24, 4> *normalIndexArray;
	normalIndexArray = new osg::TemplateIndexArray<unsigned int, osg::Array::UIntArrayType, 24, 4>();

	//setting attributes of geometry
	osg::Geometry *geometry = new osg::Geometry();
	geometry->setVertexArray(vertexArray);
	geometry->setNormalArray(normalArray);
	geometry->setNormalIndices(normalIndexArray);
	geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
	//geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
	geometry->addPrimitiveSet(faceArray);

	osg::Vec4Array* color = new osg::Vec4Array();     
	color->push_back( osg::Vec4( 1, 0, 0, 0.5 ) );    
	geometry->setColorArray( color );
	geometry->setColorBinding( osg::Geometry::BIND_OVERALL );

	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	geode->addDrawable(geometry);
	return geode.release();
}

osg::Vec3 asOsgVec3( const btVector3& v )  
{
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
	model->setName(modelname);

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
