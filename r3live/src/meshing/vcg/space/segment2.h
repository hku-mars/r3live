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



#ifndef __VCGLIB_SEGMENT2
#define __VCGLIB_SEGMENT2

#include <vcg/space/point2.h>
#include <vcg/space/line2.h>
#include <vcg/space/box2.h>

namespace vcg {

/** \addtogroup space */
/*@{*/
/**
Templated class for 3D segment.
  This is the class for a segment in 3D space. A Segment is stored just as its two extrema (Point3).
    @param SegmentScalarType (template parameter) Specifies the type of scalar used to represent coords.
*/
template <class SegmentScalarType >
class Segment2
{
public:

	/// The scalar type
	typedef SegmentScalarType ScalarType;

	/// The point type
	typedef Point2<SegmentScalarType> PointType;

	/// The point type
	typedef Segment2<SegmentScalarType> SegmentType;

private:

	/// _extrema
	PointType _p0,_p1;

public:

        /// Members to access either extrema
  inline const PointType &P0() const { return _p0; }
  inline const PointType &P1() const { return _p1; }
  inline PointType &P0() { return _p0; }
  inline PointType &P1() { return _p1; }
        /// The empty constructor
    Segment2() {};
        /// The (a,b) constructor
    Segment2(const PointType &a, const PointType &b) { _p0=a; _p1=b; };
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
    { return ( _p0 +  _p1) / ScalarType(2.0) ; }
      /// return the bounding box
    inline Box2<ScalarType> BBox( ) const
    {
      Box2<ScalarType> t;
      t.Add(_p0);
      t.Add(_p1);
      return t;
    }
        /// returns segment length
    ScalarType Length()
    { return (_p0 - _p1).Norm(); }

	/// returns segment length
	ScalarType Length() const
	{ return (_p0 - _p1).Norm(); }

	/// return segment squared length
	ScalarType SquaredLength()
	{ return (_p0 - _p1).SquaredNorm(); }
	  /// flips: a-b becomes b-a
	void Flip()
	{ PointType t=_p0; _p0=_p1; _p1=t; }
	  /// importer for different line types
	template <class Q>
	inline void Import( const Segment2<Q> & b )
	{ _p0.Import( b.P0() );	_p1.Import( b.P1() );
	}
	  /// copy constructor (builds a new segment importing an existing one)
	template <class Q>
	static SegmentType Construct( const Segment2<Q> & b )
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



typedef Segment2<short>  Segment2s;
typedef Segment2<int>	 Segment2i;
typedef Segment2<float>  Segment2f;
typedef Segment2<double> Segment2d;

template <class ScalarType>
Point2<ScalarType> ClosestPoint( Segment2<ScalarType> s, const Point2<ScalarType> & p)
{
	vcg::Line2<ScalarType, true> l;
	l.Set(s.P0(),s.P1()-s.P0());
	ScalarType t = l.Projection(p);
	Point2<ScalarType> clos = l.P(t);
	ScalarType length = s.Length();
	if (t <= 0)
		return s.P0();
	else if (t >= length)
		return s.P1();
	else
		return clos;
}

/*@}*/

} // end namespace
#endif
