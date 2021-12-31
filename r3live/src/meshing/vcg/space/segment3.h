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
Revision 1.7  2006/06/06 14:35:31  zifnab1974
Changes for compilation on linux AMD64. Some remarks: Linux filenames are case-sensitive. _fileno and _filelength do not exist on linux

Revision 1.6  2006/03/07 16:39:38  pietroni
compiled  and corrected ClosestPoint function

Revision 1.5  2004/05/13 23:39:47  ponchio
SegmentType -> Segment3 in constructor (g++ complained)

Revision 1.4  2004/05/08 14:07:50  ganovelli
return type of length and squaredlength corrected

Revision 1.3  2004/03/11 11:47:20  tarini
minor updates, corrections, added documentations, etc.


Revision 1.1  2004/03/08 19:46:47  tarini
First Version (tarini)

****************************************************************************/



#ifndef __VCGLIB_SEGMENT3
#define __VCGLIB_SEGMENT3

#include <vcg/space/point3.h>
#include <vcg/space/line3.h>
#include <vcg/space/box3.h>

namespace vcg {

/** \addtogroup space */
/*@{*/
/**
Templated class for 3D segment.
  This is the class for a segment in 3D space. A Segment is stored just as its two extrema (Point3).
    @param SegmentScalarType (template parameter) Specifies the type of scalar used to represent coords.
*/
template <class SegmentScalarType >
class Segment3
{
public:

	/// The scalar type
	typedef SegmentScalarType ScalarType;

	/// The point type
	typedef Point3<SegmentScalarType> PointType;

	/// The point type
	typedef Segment3<SegmentScalarType> SegmentType;

private:

	/// _extrema
	PointType _p0,_p1;

public:

        /// Members to access either extrema
  inline const PointType &P0() const { return _p0; }
  inline const PointType &P1() const { return _p1; }
  inline PointType &P0() { return _p0; }
  inline PointType &P1() { return _p1; }

  inline PointType &operator[] (const int i) {return i==0?_p0:_p1;}
  inline const PointType &operator[] (const int i) const {return i==0?_p0:_p1;}

		/// The empty constructor
	Segment3() {};
		/// The (a,b) constructor
	Segment3(const PointType &a, const PointType &b) { _p0=a; _p1=b; };
		/// Operator to compare segments
	inline bool operator == ( SegmentType const & p ) const
	{	return _p0==p._p0 && _p1==p._p1; }
		/// Operator to dispare segments
	inline bool operator != ( SegmentType const & p ) const
	{	return _p0!=p._p0 || _p1!=p._p1; }
		/// initializes the segment with its extrema
	void Set( const PointType &a, const PointType &b)
	{	_p0=a; _p1=b;}
	  /// calculates the point of parameter t on the segment.
	  /// if t is in [0..1] returned point is inside the segment
	inline PointType Lerp( const ScalarType t ) const
	{ return _p0 + (_p1 - _p0) * t; }
	  /// return the middle point
	inline PointType MidPoint( ) const
	{ return ( _p0 + _p1) / ScalarType(2.0) ; }
	inline PointType Direction( ) const
	{ return ( _p1 - _p0) ; }
	inline PointType NormalizedDirection( ) const
	{ return ( _p1 - _p0).Normalize() ; }
	/// return the bounding box
	inline Box3<ScalarType> BBox( ) const
	{ Box3<ScalarType> t;
	  if (_p0[0]<_p1[0]) { t.min[0]=_p0[0];t.max[0]=_p1[0];} else { t.min[0]=_p1[0];t.max[0]=_p0[0];}
		if (_p0[1]<_p1[1]) { t.min[1]=_p0[1];t.max[1]=_p1[1];} else { t.min[1]=_p1[1];t.max[1]=_p0[1];}
	  if (_p0[2]<_p1[2]) { t.min[2]=_p0[2];t.max[2]=_p1[2];} else { t.min[2]=_p1[2];t.max[2]=_p0[2];}
	  return t; }
		/// returns segment length
	ScalarType Length() const
	{ return (_p0 - _p1).Norm(); }
		/// return segment squared length
	ScalarType SquaredLength() const
	{ return (_p0 - _p1).SquaredNorm(); }
	  /// flips: a-b becomes b-a
	void Flip()
	{ PointType t=_p0; _p0=_p1; _p1=t; }
	  /// importer for different line types
	template <class Q>
	inline void Import( const Segment3<Q> & b )
	{ _p0.Import( b.P0() );	_p1.Import( b.P1() );
	}
	  /// copy constructor (builds a new segment importing an existing one)
	template <class Q>
	static SegmentType Construct( const Segment3<Q> & b )
	{ return SegmentType(PointType::Construct(b.P0()), PointType::Construct(b.P1()));}

//@{
	 /** @name Linearity for 3d segments (operators +, -, *, /) **/
	inline SegmentType operator + ( SegmentType const & p) const
	{return SegmentType( _p0+p.P0(), _p1+p.P1() );}
	inline SegmentType operator - ( SegmentType const & p) const
	{return SegmentType( _p0-p.P0(), _p1-p.P1() );}
	inline SegmentType operator * ( const ScalarType s ) const
	{return SegmentType( _p0*s, _p1*s );}
	inline SegmentType operator / ( const ScalarType s ) const
	{ScalarType s0=((ScalarType)1.0)/s; return SegmentType( _p0*s0, _p1*s0 );}
//@}

}; // end class definition



typedef Segment3<short>  Segment3s;
typedef Segment3<int>	   Segment3i;
typedef Segment3<float>  Segment3f;
typedef Segment3<double> Segment3d;

///*
//* Computes the minimum distance between a segment and a point
//* @param[in] segment	The input segment
//* @param[in] p				The input point
//* @return							The distance between the segment and the point p
//*/
//template < class ScalarType >
//ScalarType SquaredDistance(Segment3< ScalarType > &segment, Point3< ScalarType > &p)
//{
//	typedef typename vcg::Point3< ScalarType > Point3t;
//
//	Point3t	 dir = (segment.P1()-segment.P0()).Normalize();
//	ScalarType h = dir * (p-segment.P0());
//	if			(h<=ScalarType(0.0))	return vcg::SquaredDistance<ScalarType>(p, segment.P0());
//	else if (h>=segment.Length())	return vcg::SquaredDistance<ScalarType>(p, segment.P1());
//	else
//	{
//		dir = segment.P0() + dir*h;
//		return vcg::SquaredDistance<ScalarType>(p, dir);
//	}
//}; //end of Distance method
//
//template <class ScalarType>
//Point3<ScalarType> ClosestPoint( Segment3<ScalarType> s, const Point3<ScalarType> & p)
//{
//	vcg::Line3<ScalarType> l;
//	l.Set(s.P0(),s.P1()-s.P0());
//	l.Normalize();
//	Point3<ScalarType> clos=vcg::ClosestPoint<ScalarType,true>(l,p) ;//attention to call
//	vcg::Box3<ScalarType> b;
//	b.Add(s.P0());
//	b.Add(s.P1());
//	if (b.IsIn(clos))
//		return clos;
//	else
//	{
//		ScalarType d0=(s.P0()-p).Norm();
//		ScalarType d1=(s.P1()-p).Norm();
//		if (d0<d1)
//			return (s.P0());
//		else
//			return (s.P1());
//	}
//	/*ScalarType t = s.Projection(p);
//	if (s<0) return s.P0();
//	if (s>1) return s.P0();
//	return s.P(t);*/
//}

/*@}*/

} // end namespace
#endif
