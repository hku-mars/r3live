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
Revision 1.9  2006/10/07 16:51:43  m_di_benedetto
Implemented Scale() method (was only declared).

Revision 1.8  2006/01/19 13:53:19  m_di_benedetto
Fixed product by scalar and SquaredNorm()

Revision 1.7  2005/10/15 19:11:49  m_di_benedetto
Corrected return type in Angle() and protected member access in unary operator -

Revision 1.6  2005/03/18 16:34:42  fiorin
minor changes to comply gcc compiler

Revision 1.5  2004/05/10 13:22:25  cignoni
small syntax error Math -> math in Angle

Revision 1.4  2004/04/05 11:57:32  cignoni
Add V() access function

Revision 1.3  2004/03/10 17:42:40  tarini
Added comments (Dox) !
Added Import(). Costruct(), ScalarType...  Corrected cross prod (sign). Added Angle. Now using Math:: stuff for trigon. etc.

Revision 1.2  2004/03/03 15:07:40  cignoni
renamed protected member v -> _v

Revision 1.1  2004/02/13 00:44:53  cignoni
First commit...


****************************************************************************/

#ifndef __VCGLIB_POINT2
#define __VCGLIB_POINT2

#include <assert.h>
#include <vcg/math/base.h>

namespace vcg {

/** \addtogroup space */
/*@{*/
    /**
        The templated class for representing a point in 2D space.
        The class is templated over the ScalarType class that is used to represent coordinates.
				All the usual operator overloading (* + - ...) is present.
     */
template <class P2ScalarType> class Point2
{
protected:
  /// The only data member. Hidden to user.
	P2ScalarType _v[2];
public:
	  /// the scalar type
	typedef P2ScalarType ScalarType;
	enum {Dimension = 2};

//@{

  /** @name Access to Coords.
   access to coords is done by overloading of [] or explicit naming of coords (X,Y,)
	 ("p[0]" or "p.X()" are equivalent) **/
	inline const ScalarType &X() const {return _v[0];}
	inline const ScalarType &Y() const {return _v[1];}
	inline ScalarType &X() {return _v[0];}
	inline ScalarType &Y() {return _v[1];}
	inline const ScalarType * V() const
	{
		return _v;
	}
	inline ScalarType * V()
	{
		return _v;
	}
	inline ScalarType & V( const int i )
	{
		assert(i>=0 && i<2);
		return _v[i];
	}
	inline const ScalarType & V( const int i ) const
	{
		assert(i>=0 && i<2);
		return _v[i];
	}
	inline const ScalarType & operator [] ( const int i ) const
	{
		assert(i>=0 && i<2);
		return _v[i];
	}
	inline ScalarType & operator [] ( const int i )
	{
		assert(i>=0 && i<2);
		return _v[i];
	}
//@}
	  /// empty constructor (does nothing)
	inline Point2 () { }
	  /// x,y constructor
	inline Point2 ( const ScalarType nx, const ScalarType ny )
	{
			_v[0] = nx; _v[1] = ny;
	}
	  /// copy constructor
	inline Point2 ( Point2 const & p)
	{
			_v[0]= p._v[0];    _v[1]= p._v[1];
	}
	  /// copy
	inline Point2 & operator =( Point2 const & p)
	{
			_v[0]= p._v[0]; _v[1]= p._v[1];
			return *this;
	}
	  /// sets the point to (0,0)
	inline void SetZero()
	{ _v[0] = 0;_v[1] = 0;}
	  /// dot product
	inline ScalarType operator * ( Point2 const & p ) const
	{
			return ( _v[0]*p._v[0] + _v[1]*p._v[1] );
	}
	inline ScalarType dot( const Point2 & p ) const { return (*this) * p; }
  /// cross product
	inline ScalarType operator ^ ( Point2 const & p ) const
	{
			return _v[0]*p._v[1] - _v[1]*p._v[0];
	}
//@{
	 /** @name Linearity for 2d points (operators +, -, *, /, *= ...) **/
	inline Point2 operator + ( Point2 const & p) const
	{
			return Point2<ScalarType>( _v[0]+p._v[0], _v[1]+p._v[1] );
	}
	inline Point2 operator - ( Point2 const & p) const
	{
			return Point2<ScalarType>( _v[0]-p._v[0], _v[1]-p._v[1] );
	}
	inline Point2 operator * ( const ScalarType s ) const
	{
			return Point2<ScalarType>( _v[0] * s, _v[1] * s );
	}
	inline Point2 operator / ( const ScalarType s ) const
	{
			return Point2<ScalarType>( _v[0] / s, _v[1] / s );
	}
	inline Point2 & operator += ( Point2 const & p)
	{
			_v[0] += p._v[0];    _v[1] += p._v[1];
			return *this;
	}
	inline Point2 & operator -= ( Point2 const & p)
	{
			_v[0] -= p._v[0];    _v[1] -= p._v[1];
			return *this;
	}
	inline Point2 & operator *= ( const ScalarType s )
	{
			_v[0] *= s;    _v[1] *= s;
			return *this;
	}
	inline Point2 & operator /= ( const ScalarType s )
	{
			_v[0] /= s;    _v[1] /= s;
			return *this;
	}
 //@}
	  /// returns the norm (Euclidian)
	inline ScalarType Norm( void ) const
	{
		return math::Sqrt( _v[0]*_v[0] + _v[1]*_v[1] );
	}
	  /// returns the squared norm (Euclidian)
	inline ScalarType SquaredNorm( void ) const
	{
			return ( _v[0]*_v[0] + _v[1]*_v[1] );
	}
	inline Point2 & Scale( const ScalarType sx, const ScalarType sy )
	{
		_v[0] *= sx;
		_v[1] *= sy;
		return * this;
	}
	  /// normalizes, and returns itself as result
	inline Point2 & Normalize( void )
	{
			ScalarType n = math::Sqrt(_v[0]*_v[0] + _v[1]*_v[1]);
			if(n>0.0) {	_v[0] /= n;	_v[1] /= n; }
			return *this;
	}
	  /// points equality
	inline bool operator == ( Point2 const & p ) const
	{
			return (_v[0]==p._v[0] && _v[1]==p._v[1]);
	}
	  /// disparity between points
	inline bool operator != ( Point2 const & p ) const
	{
			return ( (_v[0]!=p._v[0]) || (_v[1]!=p._v[1]) );
	}
	  /// lexical ordering
	inline bool operator <  ( Point2 const & p ) const
	{
			return	(_v[1]!=p._v[1])?(_v[1]<p._v[1]):
							(_v[0]<p._v[0]);
	}
	  /// lexical ordering
	inline bool operator >  ( Point2 const & p ) const
	{
			return	(_v[1]!=p._v[1])?(_v[1]>p._v[1]):
							(_v[0]>p._v[0]);
	}
	  /// lexical ordering
	inline bool operator <= ( Point2 const & p ) const
	{
			return	(_v[1]!=p._v[1])?(_v[1]< p._v[1]):
							(_v[0]<=p._v[0]);
	}
	  /// lexical ordering
	inline bool operator >= ( Point2 const & p ) const
	{
			return	(_v[1]!=p._v[1])?(_v[1]> p._v[1]):
							(_v[0]>=p._v[0]);
	}
	  /// returns the distance to another point p
	inline ScalarType Distance( Point2 const & p ) const
	{
			return Norm(*this-p);
	}
	  /// returns the suqared distance to another point p
	inline ScalarType SquaredDistance( Point2 const & p ) const
	{
      return (*this-p).SquaredNorm();
	}
	  /// returns the angle with X axis (radiants, in [-PI, +PI] )
	inline ScalarType Angle() const {
		return math::Atan2(_v[1],_v[0]);
	}
		/// transform the point in cartesian coords into polar coords
	inline Point2 & Cartesian2Polar()
	{
		ScalarType t = Angle();
		_v[0] = Norm();
		_v[1] = t;
		return *this;
	}
		/// transform the point in polar coords into cartesian coords
	inline Point2 & Polar2Cartesian()
	{
		ScalarType l = _v[0];
		_v[0] = (ScalarType)(l*math::Cos(_v[1]));
		_v[1] = (ScalarType)(l*math::Sin(_v[1]));
		return *this;
	}
		/// rotates the point of an angle (radiants, counterclockwise)
	inline Point2 & Rotate( const ScalarType rad )
	{
		ScalarType t = _v[0];
		ScalarType s = math::Sin(rad);
		ScalarType c = math::Cos(rad);

		_v[0] = _v[0]*c - _v[1]*s;
		_v[1] =   t *s + _v[1]*c;

		return *this;
	}

	/// Questa funzione estende il vettore ad un qualsiasi numero di dimensioni
	/// paddando gli elementi estesi con zeri
	inline ScalarType Ext( const int i ) const
	{
		if(i>=0 && i<2) return _v[i];
		else            return 0;
	}
		/// imports from 2D points of different types
	template <class T>
	inline void Import( const Point2<T> & b )
	{
		_v[0] = b.X(); _v[1] = b.Y();
	}
		/// constructs a 2D points from an existing one of different type
	template <class T>
	static Point2 Construct( const Point2<T> & b )
	{
    return Point2(b.X(),b.Y());
	}


}; // end class definition


template <class T>
inline T Angle( Point2<T> const & p0, Point2<T> const & p1 )
{
	return p1.Angle() - p0.Angle();
}

template <class T>
inline Point2<T> operator - ( Point2<T> const & p ){
    return Point2<T>( -p[0], -p[1] );
}

template <class T>
inline Point2<T> operator * ( const T s, Point2<T> const & p ){
    return Point2<T>( p[0] * s, p[1] * s  );
}

template <class T>
inline T Norm( Point2<T> const & p ){
		return p.Norm();
}

template <class T>
inline T SquaredNorm( Point2<T> const & p ){
    return p.SquaredNorm();
}

template <class T>
inline Point2<T> & Normalize( Point2<T> & p ){
    return p.Normalize();
}

template <class T>
inline T Distance( Point2<T> const & p1,Point2<T> const & p2 ){
    return Norm(p1-p2);
}

template <class T>
inline T SquaredDistance( Point2<T> const & p1,Point2<T> const & p2 ){
    return SquaredNorm(p1-p2);
}

template <class SCALARTYPE>
inline Point2<SCALARTYPE> Abs(const Point2<SCALARTYPE> & p) {
  return (Point2<SCALARTYPE>(math::Abs(p[0]), math::Abs(p[1])));
}

typedef Point2<short>  Point2s;
typedef Point2<int>	   Point2i;
typedef Point2<float>  Point2f;
typedef Point2<double> Point2d;

/*@}*/
} // end namespace
#endif
