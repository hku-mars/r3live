/****************************************************************************
* MeshLab                                                           o o     *
* An extendible mesh processor                                    o     o   *
*                                                                _   O  _   *
* Copyright(C) 2005, 2009                                          \/)\/    *
* Visual Computing Lab                                            /\/|      *
* ISTI - Italian National Research Council                           |      *
*                                                                    \      *
* All rights reserved.                                                      *
*                                                                           *
* This program is free software; you can redistribute it and/or modify      *
* it under the terms of the GNU General Public License as published by      *
* the Free Software Foundation; either version 2 of the License, or         *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
****************************************************************************/

#ifndef __VCG_OSG_VERTEX_PLUS_COMPONENT
#define __VCG_OSG_VERTEX_PLUS_COMPONENT


#include <vector>
#include <vcg/space/point3.h>
#include <vcg/space/texcoord2.h>
#include <vcg/space/color4.h>

#include <opensg/osggeometry.h>


namespace vcg {
	
namespace vert {

/// Some Naming Rules : All the Components that can be added to a vertex should be defined in the namespace vert:

/// ------------------------- OPENSGINFO -----------------------------------------

template< class T > class EmptyOSGInfo : public T { public : OSG::GeometryPtr Geo() { assert(0); return NULL; } };

template< class T > class OSGInfo : public T 
{
public :
	OSG::GeometryPtr & Geo() { return _geop; }
	int & Index() { return _vertexi; }
private :
	OSG::GeometryPtr _geop;				/// Maybe we can use pointers to buffers directly but now we really don't now if these pointers change in time ... !!!
	int _vertexi;								/// OSG vertex index
};

/// ------------------------- COORD -----------------------------------------

/// Core for the coordinate component templated with the coordinate type of the component
template< class T > class OSGCoordCore
{
public :
	typedef T CoordType;
	typedef typename CoordType::ValueType ScalarType;
	OSGCoordCore( OSG::GeometryPtr p, int i ) { _geopointer = p; _vertexindex = i; }
	~OSGCoordCore() { _vertexindex = -1; }
	CoordType & operator=( CoordType & p2 ) 
	{
		/// Set coordinates
		OSG::GeoPositions3fPtr pos = OSG::GeoPositions3fPtr::dcast( _geopointer->getPositions() );
		OSG::beginEditCP( pos, OSG::GeoPositions3f::GeoPropDataFieldMask );
		pos->setValue( p2, _vertexindex );
		OSG::endEditCP( pos, OSG::GeoPositions3f::GeoPropDataFieldMask );
		return p2;			/// Warning : instead of returning the left side operand we return the right one !!!
	}
	ScalarType X()
	{
		/// Get coordinates
		OSG::GeoPositions3fPtr pos = OSG::GeoPositions3fPtr::dcast( _geopointer->getPositions() );
		OSG::beginEditCP( pos, OSG::GeoPositions3f::GeoPropDataFieldMask );
		OSG::Pnt3f p; pos->getValue( p, _vertexindex );
		OSG::endEditCP( pos, OSG::GeoPositions3f::GeoPropDataFieldMask );
		return p.x();
	}
	ScalarType Y()
	{
		/// Get coordinates
		OSG::GeoPositions3fPtr pos = OSG::GeoPositions3fPtr::dcast( _geopointer->getPositions() );
		OSG::beginEditCP( pos, OSG::GeoPositions3f::GeoPropDataFieldMask );
		OSG::Pnt3f p; pos->getValue( p, _vertexindex );
		OSG::endEditCP( pos, OSG::GeoPositions3f::GeoPropDataFieldMask );
		return p.y();
	}
	ScalarType Z()
	{
		/// Get coordinates
		OSG::GeoPositions3fPtr pos = OSG::GeoPositions3fPtr::dcast( _geopointer->getPositions() );
		OSG::beginEditCP( pos, OSG::GeoPositions3f::GeoPropDataFieldMask );
		OSG::Pnt3f p; pos->getValue( p, _vertexindex );
		OSG::endEditCP( pos, OSG::GeoPositions3f::GeoPropDataFieldMask );
		return p.z();
	}
private :
	OSG::GeometryPtr _geopointer;
	int _vertexindex;
};

template< class T > class EmptyOSGCoord : public T 
{
public :
	typedef OSG::Pnt3f CoordType;
	typedef OSG::Real32 ScalarType;
	CoordType & P() { assert(0); return CoordType(); }
	const CoordType & P() const { assert(0); return CoordType(); }
	const CoordType & cP() const { assert(0); return CoordType(); }
	CoordType & P() { assert(0); return CoordType(); }
	static bool HasCoord() { return false; }
	static void Name( std::vector< std::string > & name ) { T::Name(name); }
};

template< class A, class T > class OSGCoord : public T 
{
public :
	typedef A CoordType;															/// Must be a OSG::Pnt3 type as : OSG::Pnt3s, OSG:Pnt3f, OSG::Pnt3d etc...
	typedef typename CoordType::ValueType ScalarType;				/// Can be a OSG basic type as : OSG::Int16, OSG::Real32, OSG::Real64 etc...
	typedef typename OSGCoordCore< CoordType > CoreType;
	OSGCoord() { _corep = NULL; }
	~OSGCoord() { if( _corep != NULL ) delete _corep; }
	CoreType & P() 
	{ 
		CoreType * tmpcorep = _corep; 
		_corep = new CoreType( Geo(), Index() ); 
		if( tmpcorep != NULL ) delete tmpcorep; 
		return *_corep; 
	}
	static bool HasCoord() { return true; }
	static void Name( std::vector< std::string > & name ) { name.push_back( std::string("OSGCoord") ); T::Name(name); }
private : 
	CoreType * _corep;
};

class OSGCoordCore3f : public OSGCoordCore< OSG::Pnt3f > {};

template< class T > class OSGCoord3f : public OSGCoord< OSG::Pnt3f, T > 
{ public : static void Name( std::vector< std::string > & name ) { name.push_back( std::string( "OSGCoord3f" ) ); T::Name(name); } };


/// -------------------------- NORMAL ----------------------------------------

template< class T > class OSGNormalCore
{
public :
	typedef T NormalType;
	typedef typename NormalType::ValueType ScalarType;
	OSGNormalCore( OSG::GeometryPtr p, int i ) { _geopointer = p; _vertexindex = i; }
	~OSGNormalCore() { _vertexindex = -1; }
	NormalType & operator=( NormalType & n2 ) 
	{
		/// Set coordinates
		OSG::GeoNormals3fPtr norm = OSG::GeoNormals3fPtr::dcast( _geopointer->getNormals() );
		OSG::beginEditCP( norm, OSG::GeoNormals3f::GeoPropDataFieldMask );
		norm->setValue( n2, _vertexindex );
		OSG::endEditCP( norm, OSG::GeoNormals3f::GeoPropDataFieldMask );
		return n2;			/// Warning : instead of returning the left side operand we return the right one !!!
	}
	ScalarType X()
	{
		/// Get coordinates
		OSG::GeoNormals3fPtr norm = OSG::GeoNormals3fPtr::dcast( _geopointer->getNormals() );
		OSG::beginEditCP( norm, OSG::GeoNormals3f::GeoPropDataFieldMask );
		OSG::Vec3f n; norm->getValue( n, _vertexindex );
		OSG::endEditCP( norm, OSG::GeoNormals3f::GeoPropDataFieldMask );
		return n.x();
	}
	ScalarType Y()
	{
		/// Get coordinates
		OSG::GeoNormals3fPtr norm = OSG::GeoNormals3fPtr::dcast( _geopointer->getNormals() );
		OSG::beginEditCP( norm, OSG::GeoNormals3f::GeoPropDataFieldMask );
		OSG::Vec3f n; norm->getValue( n, _vertexindex );
		OSG::endEditCP( norm, OSG::GeoNormals3f::GeoPropDataFieldMask );
		return n.y();
	}
	ScalarType Z()
	{
		/// Get coordinates
		OSG::GeoNormals3fPtr norm = OSG::GeoNormals3fPtr::dcast( _geopointer->getNormals() );
		OSG::beginEditCP( norm, OSG::GeoNormals3f::GeoPropDataFieldMask );
		OSG::Vec3f n; norm->getValue( n, _vertexindex );
		OSG::endEditCP( norm, OSG::GeoNormals3f::GeoPropDataFieldMask );
		return n.z();
	}
private :
	OSG::GeometryPtr _geopointer;
	int _vertexindex;
};

template< class T > class EmptyOSGNormal : public T 
{
public : 
	typedef OSG::Vec3f NormalType;
	typedef OSG::Real32 ScalarType;
	NormalType & N() { assert(0); return NormalType(); }
	const NormalType cN()const { assert(0); return NormalType(); }
	static bool HasNormal() { return false; }
	static bool HasNormalOcc() { return false; }
	static void Name( std::vector< std::string > & name ) { T::Name(name); }
};

template< class A, class T > class OSGNormal : public T 
{
public : 
	typedef A NormalType;														/// Must be a OSG::Vec3 type as : OSG::Vec3s, OSG:Vec3f, OSG::Vec3d etc...
	typedef typename NormalType::ValueType ScalarType;			/// Can be a OSG basic type as : OSG::Int16, OSG::Real32, OSG::Real64 etc...
	typedef typename OSGNormalCore< NormalType > CoreType;
	OSGNormal() { _corep = NULL; }
	~OSGNormal() { if( _corep == NULL ) delete _corep; }
	CoreType & N() 
	{ 
		CoreType * tmpcorep = _corep;
		_corep = new CoreType( Geo(), Index() ); 
		if( tmpcorep == NULL ) delete tmpcorep;
		return *_corep; 
	}
	static bool HasNormal()   { return true; }
	static void Name( std::vector< std::string > & name ) { name.push_back( std::string( "OSGNormal" ) );T::Name(name); }
private : 
	CoreType * _corep;
};

class OSGNormalCore3f : public OSGNormalCore< OSG::Vec3f > {};

template< class T > class OSGNormal3f : public OSGNormal< OSG::Vec3f, T > 
{ public : static void Name( std::vector< std::string > & name ) { name.push_back( std::string( "OSGNormal3f" ) ); T::Name(name); } };


/// -------------------------- COLOR ----------------------------------

template< class T > class OSGColorCore
{
public :
	typedef T ColorType;
	typedef typename ColorType::ValueType ScalarType;
	OSGColorCore( OSG::GeometryPtr p, int i ) { _geopointer = p; _vertexindex = i; }
	~OSGColorCore() { _vertexindex = -1; }
	ColorType & operator=( ColorType & c2 ) 
	{
		/// Set color
		OSG::GeoColors3fPtr colp = OSG::GeoColors3fPtr::dcast( _geopointer->getColors() );
		OSG::beginEditCP( colp, OSG::GeoColors3f::GeoPropDataFieldMask );
		colp->setValue( c2, _vertexindex );
		OSG::endEditCP( colp, OSG::GeoColors3f::GeoPropDataFieldMask );
		return c2;			/// Warning : instead of returning the left side operand we return the right one !!!
	}
	ScalarType R()
	{
		/// Get coordinates
		OSG::GeoColors3fPtr colp = OSG::GeoColors3fPtr::dcast( _geopointer->getColors() );
		OSG::beginEditCP( colp, OSG::GeoColors3f::GeoPropDataFieldMask );
		OSG::Color3f c; colp->getValue( c, _vertexindex );
		OSG::endEditCP( colp, OSG::GeoColors3f::GeoPropDataFieldMask );
		return c.red();
	}
	ScalarType G()
	{
		/// Get coordinates
		OSG::GeoColors3fPtr colp = OSG::GeoColors3fPtr::dcast( _geopointer->getColors() );
		OSG::beginEditCP( colp, OSG::GeoColors3f::GeoPropDataFieldMask );
		OSG::Color3f c; colp->getValue( c, _vertexindex );
		OSG::endEditCP( colp, OSG::GeoColors3f::GeoPropDataFieldMask );
		return c.green();
	}
	ScalarType B()
	{
		/// Get coordinates
		OSG::GeoColors3fPtr colp = OSG::GeoColors3fPtr::dcast( _geopointer->getColors() );
		OSG::beginEditCP( colp, OSG::GeoColors3f::GeoPropDataFieldMask );
		OSG::Color3f c; colp->getValue( c, _vertexindex );
		OSG::endEditCP( colp, OSG::GeoColors3f::GeoPropDataFieldMask );
		return c.blue();
	}
private : 
	OSG::GeometryPtr _geopointer;
	int _vertexindex;
};

template< class T > class EmptyOSGColor : public T 
{
public : 
	typedef OSG::Color3f ColorType;
	typedef OSG::Real32 ScalarType;
	ColorType & C() { assert(0); return ColorType(); }
	static bool HasColor() { return false; }
	static void Name( std::vector< std::string > & name ) { T::Name(name); }
};

template< class A, class T > class OSGColor : public T 
{
public :
	typedef A ColorType;														/// Must be a OSG::Color3 type as : OSG::Color3ub, OSG:Color3f etc...
	typedef typename ColorType::ValueType ScalarType;				/// Can be a OSG basic type as : OSG::UInt8, OSG::Real32 etc...
	typedef typename OSGColorCore< ColorType > CoreType;
	OSGColor() { _corep = NULL; }
	~OSGColor() { if( _corep != NULL ) delete _corep; }
	CoreType & C() 
	{ 
		CoreType * tmpcorep = _corep;
		_corep = new CoreType( Geo(), Index() ); 
		if( tmpcorep != NULL ) delete tmpcorep;
		return *_corep; 
	}
	static bool HasColor() { return true; }
	static void Name( std::vector< std::string > & name ) { name.push_back( std::string( "OSGColor" ) ); T::Name(name); }
private : 
	CoreType * _corep;
};

class OSGColorCore3f : public OSGColorCore< OSG::Color3f > {};

template< class T > class OSGColor3f : public OSGColor< OSG::Color3f, T > 
{ static void Name( std::vector< std::string > & name ) { name.push_back( std::string( "OSGColor3f" ) ); T::Name(name); } };


}			/// end namespace vert

}			/// end namespace vcg


#endif


