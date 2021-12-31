#pragma once
/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2004                                                \/)\/    *
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
/****************************************************************************
  History

$Log: not supported by cvs2svn $
Revision 1.5  2007/11/07 09:37:20  ganovelli
added draft for sphereofsphres enclosing

Revision 1.4  2006/09/14 08:46:00  ganovelli
added inclusion of sphere3

Revision 1.3  2006/07/12 12:13:23  zifnab1974
static keyword only in declaration not in implementation

Revision 1.2  2006/07/10 10:38:16  turini
minor changes in SphereOfTetra()

Revision 1.1  2006/07/06 12:37:18  ganovelli
draft version. For the triangle is not tehe smallest enclosing sphere and for the set of spheres works only for two spheres



****************************************************************************/

#include <vcg/space/triangle3.h>
#include <vcg/space/tetra3.h>
#include <vcg/space/sphere3.h>
#include <assert.h>
namespace vcg{
	/** \addtogroup space */
/*@{*/
/** 
Class for function computing the smallest enclosing bounding volume
  
*/
struct SmallestEnclosing {

	/// computes the smallest enclosing sphere of a triangle
	template <class TriangleType>
		static Sphere3<typename TriangleType::ScalarType>  SphereOfTriangle(const TriangleType & t); 

	/// computes the smallest enclosing sphere of a tetrahedron
	template <class TetraType>
		static Sphere3<typename TetraType::ScalarType>  SphereOfTetra(const TetraType & t); 

	/// computes the smallest enclosing sphere of two spheres
	template <class SphereType>
		static SphereType  SphereOfSpheres( const SphereType & s0,const SphereType & s1); 

	/// computes the smallest enclosing sphere of a container of spheres
	template <class SphereContType>
		static typename SphereContType::value_type  SphereOfSpheres( const SphereContType & t); 
};
/*@}*/

template <class TriangleType>
Sphere3<typename TriangleType::ScalarType> 
SmallestEnclosing::SphereOfTriangle(const TriangleType & t){
	return Sphere3<typename TriangleType::ScalarType>(t.Barycenter(),(t.Barycenter()-t.cP(0)).Norm() );
}

template <class TetraType>
Sphere3<typename TetraType::ScalarType> 
SmallestEnclosing::SphereOfTetra(const TetraType & t){
	return Sphere3<typename TetraType::ScalarType>( t.Barycenter(),( t.Barycenter() - t.cP(0) ).Norm() );
}

template <class SphereType>
  SphereType 
 SmallestEnclosing::
SphereOfSpheres(  const   SphereType  & s0, const  SphereType  & s1)
{
	typename SphereType::ScalarType radius;
	vcg::Point3f center;

	if(s0.Radius()==-1.0) return s1; else if(s1.Radius()==-1.0) return s0;

 	float dst = (s1.Center()-s0.Center()).Norm() ;
	radius = (dst+s1.Radius()+s0.Radius())/2;
	Point3f a=s0.Center();
	Point3f b=s1.Center();
	Point3f dir = (b-a).Normalize();
	a = a - dir*s0.Radius();
	b = b + dir*s1.Radius();
	center = (a+b)/2.0;

	return  SphereType(center,radius);
}

template <typename SphereContType>
typename SphereContType::value_type
 SmallestEnclosing::
SphereOfSpheres(  const SphereContType & spheres)
{
	typename SphereContType::value_type::ScalarType radius;
	typename SphereContType::value_type res;
	typename SphereContType::const_iterator si;

	for(si = spheres.begin(); si != spheres.end(); ++si){
		res  = SphereOfSpheres(res,*si);
	}

	return res;
}
}


