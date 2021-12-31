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

#ifndef __VCGLIB_BOX2
#define __VCGLIB_BOX2

#include <assert.h>
#include <algorithm>
#include <vcg/math/base.h>
#include <vcg/space/point2.h>

namespace vcg {


// needed prototype;
template <class SegScalarType> class Segment2;

/** \addtogroup space */
/*@{*/
/**
	Templated class for a 2D bounding box. It is stored just as two Point2	
	@param BoxScalarType (Template Parameter) Specifies the scalar field.
*/
template <class BoxScalarType>
class Box2
{
public:
		/// The scalar type
	typedef BoxScalarType ScalarType;
  typedef Point2<BoxScalarType> PointType ;

		/// min coordinate point
  PointType min;
		/// max coordinate point
  PointType max;
		/// Standard constructor
	inline  Box2() { min.X()= 1; max.X()= -1; min.Y()= 1; max.Y()= -1; }
		/// Copy constructor
	inline  Box2( const Box2 & b ) { min=b.min; max=b.max; }
		/// Min Max constructor
	inline  Box2( const Point2<BoxScalarType> & mi, const Point2<BoxScalarType> & ma ) { min = mi; max = ma; }
		/// Distructor
	inline ~Box2() { }
		/// Operator to compare two bounding box
	inline bool operator == ( Box2 const & p ) const
	{
		return min==p.min && max==p.max;
	}
			/// Initializing the bounding box with a point
	void Set( const PointType & p )
	{
		min = max = p;
	}

	Point2<BoxScalarType> P(const int & i) const 
	{
			return Point2<BoxScalarType>(
				min[0]+ (i%2) * DimX(),
				min[1]+ ((i / 2)%2) * DimY());
	}

		// Initializing with the values
	inline void Set( ScalarType minx, ScalarType miny, ScalarType maxx, ScalarType maxy )
	{
		min[0] = minx;
		min[1] = miny;
		max[0] = maxx;
		max[1] = maxy;
	}
		/// Set the bounding box to a null value
	void SetNull()
	{
		 min.X()= 1; max.X()= -1; min.Y()= 1; max.Y()= -1;
	}
		/** Function to add two bounding box
        the bounding box expand to include the other bounding box (if necessary)
      @param b The bounding box to be added
		*/
	void Add( Box2 const & b )
	{
		if(IsNull())
		{
			min=b.min;
			max=b.max;
		}
		else
		{
			if(min.X() > b.min.X()) min.X() = b.min.X();
			if(min.Y() > b.min.Y()) min.Y() = b.min.Y();

			if(max.X() < b.max.X()) max.X() = b.max.X();
			if(max.Y() < b.max.Y()) max.Y() = b.max.Y();
		}
	}
    /** Adds a point to the bouning box.
      the bounding box expand to include the new point (if necessary)
			@param p The point 2D
		*/
	void Add( const PointType & p )
	{
		if(IsNull()) Set(p);
		else 
		{
			if(min.X() > p.X()) min.X() = p.X();
			if(min.Y() > p.Y()) min.Y() = p.Y();

			if(max.X() < p.X()) max.X() = p.X();
			if(max.Y() < p.Y()) max.Y() = p.Y();
		}
	}

  /** Varies the dimension of the bounding box.
    @param delta The size delta (if positive, box is enlarged)
  */
  void Offset(const ScalarType s)
	{
		Offset(PointType(s, s));
	}

    /** Varies the dimension of the bounding box.
      @param delta The size delta per dimension (if positive, box is enlarged)
		*/
	void Offset(const PointType & delta)
	{
		min -= delta;
		max += delta;
	}

    /** Computes intersection between this and another  bounding box
      @param b The other bounding box
		*/
	void Intersect( const Box2 & b )
	{
		if(min.X() < b.min.X()) min.X() = b.min.X();
		if(min.Y() < b.min.Y()) min.Y() = b.min.Y();

		if(max.X() > b.max.X()) max.X() = b.max.X();
		if(max.Y() > b.max.Y()) max.Y() = b.max.Y();

		if(min.X()>max.X() || min.Y()>max.Y()) SetNull();
	}

    /** Traslate the bounding box by a vectore
      @param p The transolation vector
		*/
	void Translate( const PointType & p )
	{
		min += p;
		max += p;
	}
    /** Checks whether a 2D point p is inside the box
			@param p The point 2D
      @return True iff p inside
		*/
	bool IsIn( PointType const & p ) const
	{
		return (
			min.X() <= p.X() && p.X() <= max.X() &&
			min.Y() <= p.Y() && p.Y() <= max.Y()
		);
	}
    /** Checks whether a 2D point p is inside the box, closed at min but open at max
      @param p The point in 2D
      @return True iff p inside
		*/
	bool IsInEx( PointType const & p ) const
	{
		return  (
			min.X() <= p.X() && p.X() < max.X() &&
			min.Y() <= p.Y() && p.Y() < max.Y() 
		);
	}
  /** Check bbox collision.
      Note: just adjiacent bbox won't collide
			@param b A bounding box
      @return True iff collision
		*/
	bool Collide( Box2 const &b )
	{
		Box2 bb=*this;
		bb.Intersect(b);
		return bb.IsValid();
	}
    /** Check if emptry.
      @return True iff empty
		*/
	inline bool IsNull() const { return min.X()>max.X() || min.Y()>max.Y(); }

    /** Check consistency.
      @return True iff consistent
		*/
	inline bool IsValid() const { return min.X()<max.X() && min.Y()<max.Y(); }

    /** Check if emptry.
      @return True iff empty
		*/
	inline bool IsEmpty() const { return min==max; }
	
    /// Computes length of diagonal
	ScalarType Diag() const
	{
		return Distance(min,max);
	}
    /// Computes box center
	PointType Center() const
	{
		return (min+max)/2;
	}
    /// Computes area
	inline ScalarType Area() const
	{
		return (max[0]-min[0])*(max[1]-min[1]);
	}
    /// computes dimension on X axis.
	inline ScalarType DimX() const { return max.X()-min.X(); }
    /// computes dimension on Y axis.
	inline ScalarType DimY() const { return max.Y()-min.Y(); }

  /// Computes sizes (as a vector)
	inline PointType Dim() const { return max-min; }

  /// Deprecated: use GlobalToLocal
	inline void Normalize( PointType & p )
	{
		p -= min;
		p[0] /= max[0]-min[0];
		p[1] /= max[1]-min[1];
	}
	
      /// Returns global coords of a local point expressed in [0..1]^2
	PointType LocalToGlobal(PointType const & p) const{
		return PointType( 
			min[0] + p[0]*(max[0]-min[0]), 
			min[1] + p[1]*(max[1]-min[1]));
	}
    /// Returns local coords expressed in [0..1]^2 of a point in R^2
	PointType GlobalToLocal(PointType const & p) const{
		return PointType( 
		  (p[0]-min[0])/(max[0]-min[0]), 
		  (p[1]-min[1])/(max[1]-min[1])
			);
	}
	
   /// Turns the bounding box into a square (conservatively)
	void MakeSquare(){
    ScalarType radius = max( DimX(), DimY() ) / 2;
    PointType c = Center();
    max = c + PointType(radius, radius);
    min = c - PointType(radius, radius);
  }
	
}; // end class definition

template <class ScalarType> 
ScalarType DistancePoint2Box2(const Point2<ScalarType> &test,
							  const Box2<ScalarType> &bbox)
{
	///test possible position respect to bounding box
	if (!bbox.IsIn(test)){
		if ((test.X()<=bbox.min.X())&&(test.Y()<=bbox.min.Y()))
			return ((test-bbox.min).Norm());
		else
		if ((test.X()>=bbox.min.X())&&
			(test.X()<=bbox.max.X())&&
			(test.Y()<=bbox.min.Y()))
			return (bbox.min.Y()-test.Y());
		else
		if ((test.X()>=bbox.max.X())&&
			(test.Y()<=bbox.min.Y()))
			return ((test-vcg::Point2<ScalarType>(bbox.max.X(),bbox.min.Y())).Norm());
		else
		if ((test.Y()>=bbox.min.Y())&&
			(test.Y()<=bbox.max.Y())&&
			(test.X()>=bbox.max.X()))
			return (test.X()-bbox.max.X());
		else
		if ((test.X()>=bbox.max.X())&&(test.Y()>=bbox.max.Y()))
			return ((test-bbox.max).Norm());
		else
		if ((test.X()>=bbox.min.X())&&
			(test.X()<=bbox.max.X())&&
			(test.Y()>=bbox.max.Y()))
			return (test.Y()-bbox.max.Y());
		else
		if ((test.X()<=bbox.min.X())&&
			(test.Y()>=bbox.max.Y()))
			return ((test-vcg::Point2<ScalarType>(bbox.min.X(),bbox.max.Y())).Norm());
		else
		if ((test.X()<=bbox.min.X())&&
			(test.Y()<=bbox.max.Y())&&
			(test.Y()>=bbox.min.Y()))
			return (bbox.min.X()-test.X());
	}
	else
	{
		//return minimum distance
		ScalarType dx=std::min<ScalarType>(fabs(test.X()-bbox.min.X()),fabs(bbox.max.X()-test.X()));
		ScalarType dy=std::min<ScalarType>(fabs(test.Y()-bbox.min.Y()),fabs(bbox.max.Y()-test.Y()));
		return(std::min<ScalarType>(dx,dy));
	}
}

template <class ScalarType> 
Point2<ScalarType> ClosestPoint2Box2(const Point2<ScalarType> &test,
									 const Box2<ScalarType> &bbox)
{
	Segment2<ScalarType> Segs[4];
	Segs[0].P0() = bbox.min;
	Segs[0].P1() = vcg::Point2<ScalarType>(bbox.max.X(), bbox.min.Y());

	Segs[1].P0() = Segs[0].P1();
	Segs[1].P1() = bbox.max;

	Segs[2].P0() = Segs[1].P1();
	Segs[2].P1() = vcg::Point2<ScalarType>(bbox.min.X(), bbox.max.Y());

	Segs[3].P0() = Segs[2].P1();
	Segs[3].P1() = bbox.min;
	
	Point2<ScalarType> closest = ClosestPoint(Segs[0], test);
	ScalarType minDist = (closest-test).Norm();
	for (int i = 1; i < 4; i++)
	{
		Point2<ScalarType> point = ClosestPoint(Segs[i], test);
		ScalarType dist = (test - point).Norm();
		if (dist < minDist)
		{
			minDist = dist;
			closest = point;
		}
	}
	return closest;
}

	/// Specificazione di box of short
typedef Box2<short>  Box2s;
	/// Specificazione di box of int
typedef Box2<int>	 Box2i;
	/// Specificazione di box of float
typedef Box2<float>  Box2f;
	/// Specificazione di box of double
typedef Box2<double> Box2d;

/*@}*/
} // end namespace


#endif
