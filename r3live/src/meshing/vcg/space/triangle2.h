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

****************************************************************************/

#ifndef __VCG_TRIANGLE2
#define __VCG_TRIANGLE2
#include <vcg/space/triangle3.h>
#include <vcg/space/point2.h>
#include <vcg/space/segment2.h>
#include <float.h>

namespace vcg {

/** \addtogroup space */
/*@{*/
/** 
		Templated class for storing a generic triangle in a 2D space.
    Note the relation with the Face class of TriMesh complex, both classes provide the P(i) access functions to their points and therefore they share the algorithms on it (e.g. area, normal etc...)
 */
template <class SCALAR_TYPE> class Triangle2
{
public:
  typedef SCALAR_TYPE ScalarType;
  typedef Point2< ScalarType > CoordType;
  typedef Triangle2<ScalarType> TriangleType;

protected:
	/// Vector of vertex pointer incident in the face
	Point2<ScalarType> _v[3];
public:

	Triangle2()
	{}

	Triangle2(const CoordType &p0,const CoordType &p1,const CoordType &p2)
	{
		P(0)=p0;
		P(1)=p1;
		P(2)=p2;
	}

	/// Shortcut per accedere ai punti delle facce
	inline CoordType & P( const int j ) { return _v[j];}
	inline CoordType & P0( const int j ) { return _v[j];}
	inline CoordType & P1( const int j ) { return _v[(j+1)%3];}
	inline CoordType & P2( const int j ) { return _v[(j+2)%3];}
	inline const CoordType &  P( const int j ) const { return _v[j];}
	inline const CoordType &  P0( const int j ) const { return _v[j];}
	inline const CoordType &  P1( const int j ) const { return _v[(j+1)%3];}
	inline const CoordType &  P2( const int j ) const { return _v[(j+2)%3];}
	inline const CoordType & cP0( const int j ) const { return _v[j];}
	inline const CoordType & cP1( const int j ) const { return _v[(j+1)%3];}
	inline const CoordType & cP2( const int j ) const { return _v[(j+2)%3];}

/** evaluate barycentric coordinates
	@param bq Point on the face
	@param L0 barycentric value for V(0)
	@param L1 barycentric value for V(1)
	@param L2 barycentric value for V(2)
	@return true se bq appartain to the face, false otherwise
	from http://en.wikipedia.org/wiki/Barycentric_coordinate_system_(mathematics)
	L1=((y2-y3)(x-x3)+(x3-x2)(y-y3))/((y2-y3)(x1-x3)+(x3-x2)(y1-y3))
	L2=((y3-y1)(x-x3)+(x1-x3)(y-y3))/((y3-y1)(x2-x3)+(x1-x3)(y2-y3))
	L3=1-L1-L2
*/
bool InterpolationParameters(const CoordType & bq, ScalarType &L1, 
							 ScalarType &L2, ScalarType &L3 ) const
{	
	const ScalarType EPSILON = ScalarType(0.0001f);

	ScalarType x1=P(0).X();
	ScalarType x2=P(1).X();
	ScalarType x3=P(2).X();

	ScalarType y1=P(0).Y();
	ScalarType y2=P(1).Y();
	ScalarType y3=P(2).Y();

	ScalarType x=bq.X();
	ScalarType y=bq.Y();
	
	L1=((y2-y3)*(x-x3)+(x3-x2)*(y-y3))/((y2-y3)*(x1-x3)+(x3-x2)*(y1-y3));
	L2=((y3-y1)*(x-x3)+(x1-x3)*(y-y3))/((y3-y1)*(x2-x3)+(x1-x3)*(y2-y3));
	L3=1-L1-L2;
  if(math::IsNAN(L1) || math::IsNAN(L2) || math::IsNAN(L3)) L1=L2=L3=(ScalarType)(1.0/3.0);
	bool inside=true;
	inside&=(L1>=0-EPSILON)&&(L1<=1+EPSILON);
	inside&=(L2>=0-EPSILON)&&(L2<=1+EPSILON);
	inside&=(L3>=0-EPSILON)&&(L3<=1+EPSILON);
	return inside;
}

///return the distance to the point q and neighors point p
void PointDistance(const CoordType & q,
				    ScalarType & dist, 
				    CoordType & p ) const
{
	dist=FLT_MAX;
	///find distance to each segment and take minimum
	for (int i=0;i<3;i++)
	{
		vcg::Segment2<float> s=vcg::Segment2<float>(P(i),P((i+1)%3));
		CoordType clos=ClosestPoint<ScalarType>(s,q);
		ScalarType dis_test=(clos-q).Norm();
		if (dis_test<dist)
		{
			dist=dis_test;
			p=clos;
		}
	}
}

///retutn true if the face is contuerclockwise oriented
bool IsCCW()
{
    ScalarType Area=(P(1)-P(0))^(P(2)-P(0));
    return (Area>0);
}

}; //end Class


}	 // end namespace
#endif

