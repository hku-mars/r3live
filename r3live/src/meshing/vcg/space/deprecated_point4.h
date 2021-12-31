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
Revision 1.12  2006/06/21 11:06:16  ganovelli
changed return type of Zero() (to void)

Revision 1.11  2005/04/13 09:40:30  ponchio
Including math/bash.h

Revision 1.10  2005/03/18 16:34:42  fiorin
minor changes to comply gcc compiler

Revision 1.9  2005/01/21 18:02:11  ponchio
Removed dependence from matrix44 and changed VectProd

Revision 1.8  2005/01/12 11:25:02  ganovelli
added Dimension

Revision 1.7  2004/10/11 17:46:11  ganovelli
added definition of vector product (not implemented)

Revision 1.6  2004/05/10 11:16:19  ganovelli
include assert.h added

Revision 1.5  2004/03/31 10:09:58  cignoni
missing return value in zero()

Revision 1.4  2004/03/11 17:17:49  tarini
added commets (doxy), uniformed with new style, now using math::, ...
added HomoNormalize(), Zero()... remade StableDot() (hand made sort).

Revision 1.1  2004/02/10 01:11:28  cignoni
Edited Comments and GPL license

****************************************************************************/

#ifndef __VCGLIB_POINT4
#define __VCGLIB_POINT4
#include <assert.h>

#include <vcg/math/base.h>

namespace vcg {
/** \addtogroup space */
/*@{*/
    /**
        The templated class for representing a point in 4D space.
        The class is templated over the ScalarType class that is used to represent coordinates.
                All the usual operator (* + - ...) are defined.
     */

template <class T> class Point4
{
public:
  /// The only data member. Hidden to user.
    T _v[4];

public:
	typedef T ScalarType;
	enum {Dimension = 4};

//@{

  /** @name Standard Constructors and Initializers
   No casting operators have been introduced to avoid automatic unattended (and costly) conversion between different point types
   **/

	inline Point4 () { }
	inline Point4 ( const T nx, const T ny, const T nz , const T nw )
	{
		_v[0] = nx; _v[1] = ny; _v[2] = nz; _v[3] = nw;
	}
	inline Point4 ( const T  p[4] )
	{
		_v[0] = p[0]; _v[1]= p[1]; _v[2] = p[2]; _v[3]= p[3];
	}
	inline Point4 ( const Point4 & p )
	{
		_v[0]= p._v[0]; _v[1]= p._v[1]; _v[2]= p._v[2]; _v[3]= p._v[3];
	}
	inline void SetZero()
	{
		_v[0] = _v[1] = _v[2] = _v[3]= 0;
	}
	template <class Q>
	/// importer from different Point4 types
	inline void Import( const Point4<Q> & b )
	{
		_v[0] = T(b[0]);		_v[1] = T(b[1]);
		_v[2] = T(b[2]);
		_v[3] = T(b[3]);
	}
	template <class EigenVector>
	inline void FromEigenVector( const EigenVector & b )
	{
		_v[0] = T(b[0]);
		_v[1] = T(b[1]);
		_v[2] = T(b[2]);
		_v[3] = T(b[3]);
	}
	/// constructor that imports from different Point4 types
  template <class Q>
  static inline Point4 Construct( const Point4<Q> & b )
  {
    return Point4(T(b[0]),T(b[1]),T(b[2]),T(b[3]));
  }

//@}

//@{

  /** @name Data Access.
   access to data is done by overloading of [] or explicit naming of coords (x,y,z,w)
    **/
    inline const T & operator [] ( const int i ) const
    {
        assert(i>=0 && i<4);
        return _v[i];
    }
    inline T & operator [] ( const int i )
    {
        assert(i>=0 && i<4);
        return _v[i];
    }
    inline T &X() {return _v[0];}
    inline T &Y() {return _v[1];}
    inline T &Z() {return _v[2];}
    inline T &W() {return _v[3];}
    inline T const * V() const
    {
        return _v;
    }
    inline T * V()
    {
        return _v;
    }
    inline const T & V ( const int i ) const
    {
        assert(i>=0 && i<4);
        return _v[i];
    }
    inline T & V ( const int i )
    {
        assert(i>=0 && i<4);
        return _v[i];
    }
        /// Padding function: give a default 0 value to all the elements that are not in the [0..2] range.
        /// Useful for managing in a consistent way object that could have point2 / point3 / point4
    inline T Ext( const int i ) const
    {
        if(i>=0 && i<=3) return _v[i];
        else             return 0;
    }
//@}

//@{
  /** @name Linear operators and the likes
  **/
    inline Point4 operator + ( const Point4 & p) const
    {
        return Point4( _v[0]+p._v[0], _v[1]+p._v[1], _v[2]+p._v[2], _v[3]+p._v[3] );
    }
    inline Point4 operator - ( const Point4 & p) const
    {
        return Point4( _v[0]-p._v[0], _v[1]-p._v[1], _v[2]-p._v[2], _v[3]-p._v[3] );
    }
    inline Point4 operator * ( const T s ) const
    {
        return Point4( _v[0]*s, _v[1]*s, _v[2]*s, _v[3]*s );
    }
    inline Point4 operator / ( const T s ) const
    {
        return Point4( _v[0]/s, _v[1]/s, _v[2]/s, _v[3]/s );
    }
    inline Point4 & operator += ( const Point4 & p)
    {
        _v[0] += p._v[0]; _v[1] += p._v[1]; _v[2] += p._v[2]; _v[3] += p._v[3];
        return *this;
    }
    inline Point4 & operator -= ( const Point4 & p )
    {
        _v[0] -= p._v[0]; _v[1] -= p._v[1]; _v[2] -= p._v[2]; _v[3] -= p._v[3];
        return *this;
    }
    inline Point4 & operator *= ( const T s )
    {
        _v[0] *= s; _v[1] *= s; _v[2] *= s; _v[3] *= s;
        return *this;
    }
    inline Point4 & operator /= ( const T s )
    {
        _v[0] /= s; _v[1] /= s; _v[2] /= s; _v[3] /= s;
        return *this;
    }
    inline Point4 operator - () const
    {
        return Point4( -_v[0], -_v[1], -_v[2], -_v[3] );
    }
    inline Point4 VectProd ( const Point4 &x, const Point4 &z ) const
    {
        Point4 res;
        const Point4 &y = *this;

		res[0] = y[1]*x[2]*z[3]-y[1]*x[3]*z[2]-x[1]*y[2]*z[3]+
				 x[1]*y[3]*z[2]+z[1]*y[2]*x[3]-z[1]*y[3]*x[2];
		res[1] = y[0]*x[3]*z[2]-z[0]*y[2]*x[3]-y[0]*x[2]*
				 z[3]+z[0]*y[3]*x[2]+x[0]*y[2]*z[3]-x[0]*y[3]*z[2];
		res[2] = -y[0]*z[1]*x[3]+x[0]*z[1]*y[3]+y[0]*x[1]*
				 z[3]-x[0]*y[1]*z[3]-z[0]*x[1]*y[3]+z[0]*y[1]*x[3];
		res[3] = -z[0]*y[1]*x[2]-y[0]*x[1]*z[2]+x[0]*y[1]*
				 z[2]+y[0]*z[1]*x[2]-x[0]*z[1]*y[2]+z[0]*x[1]*y[2];
		return res;
	}
//@}

//@{
  /** @name Norms and normalizations
  **/
    /// Euclidian normal
    inline T Norm() const
    {
        return math::Sqrt( _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2] + _v[3]*_v[3] );
    }
    /// Squared euclidian normal
    inline T SquaredNorm() const
    {
        return _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2] + _v[3]*_v[3];
    }
    /// Euclidian normalization
  inline Point4 & Normalize()
    {
        T n = sqrt(_v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2] + _v[3]*_v[3] );
        if(n>0.0) {	_v[0] /= n;	_v[1] /= n;	_v[2] /= n; _v[3] /= n; }
        return *this;
    }
    /// Homogeneous normalization (division by W)
    inline Point4 & HomoNormalize(){
        if (_v[3]!=0.0) {	_v[0] /= _v[3];	_v[1] /= _v[3];	_v[2] /= _v[3]; _v[3]=1.0; }
        return *this;
    };

//@}

//@{
  /** @name Comparison operators (lexicographical order)
  **/
    inline bool operator == (  const Point4& p ) const
    {
        return _v[0]==p._v[0] && _v[1]==p._v[1] && _v[2]==p._v[2] && _v[3]==p._v[3];
    }
    inline bool operator != ( const Point4 & p ) const
    {
        return _v[0]!=p._v[0] || _v[1]!=p._v[1] || _v[2]!=p._v[2] || _v[3]!=p._v[3];
    }
    inline bool operator <  ( Point4 const & p ) const
    {
        return	(_v[3]!=p._v[3])?(_v[3]<p._v[3]):
                (_v[2]!=p._v[2])?(_v[2]<p._v[2]):
                (_v[1]!=p._v[1])?(_v[1]<p._v[1]):
                (_v[0]<p._v[0]);
    }
    inline bool operator >  ( const Point4 & p ) const
    {
        return	(_v[3]!=p._v[3])?(_v[3]>p._v[3]):
                (_v[2]!=p._v[2])?(_v[2]>p._v[2]):
                (_v[1]!=p._v[1])?(_v[1]>p._v[1]):
                (_v[0]>p._v[0]);
    }
    inline bool operator <= ( const Point4 & p ) const
    {
        return	(_v[3]!=p._v[3])?(_v[3]< p._v[3]):
                (_v[2]!=p._v[2])?(_v[2]< p._v[2]):
                (_v[1]!=p._v[1])?(_v[1]< p._v[1]):
                (_v[0]<=p._v[0]);
    }
    inline bool operator >= ( const Point4 & p ) const
    {
        return	(_v[3]!=p._v[3])?(_v[3]> p._v[3]):
                (_v[2]!=p._v[2])?(_v[2]> p._v[2]):
                (_v[1]!=p._v[1])?(_v[1]> p._v[1]):
                (_v[0]>=p._v[0]);
    }
//@}

//@{
  /** @name Dot products
  **/

	// dot product
	inline T operator * ( const Point4 & p ) const
	{
		return _v[0]*p._v[0] + _v[1]*p._v[1] + _v[2]*p._v[2] + _v[3]*p._v[3];
	}
	inline T dot( const Point4 & p ) const { return (*this) * p; }
  inline Point4 operator ^ (  const Point4& /*p*/ ) const
    {
        assert(0);// not defined by two vectors (only put for metaprogramming)
        return Point4();
    }

	/// slower version, more stable (double precision only)
	T StableDot ( const Point4<T> & p ) const
	{

		T k0=_v[0]*p._v[0],	k1=_v[1]*p._v[1], k2=_v[2]*p._v[2], k3=_v[3]*p._v[3];
		int exp0,exp1,exp2,exp3;

		frexp( double(k0), &exp0 );frexp( double(k1), &exp1 );
		frexp( double(k2), &exp2 );frexp( double(k3), &exp3 );

		if (exp0>exp1) { std::swap(k0,k1); std::swap(exp0,exp1); }
		if (exp2>exp3) { std::swap(k2,k3); std::swap(exp2,exp3); }
		if (exp0>exp2) { std::swap(k0,k2); std::swap(exp0,exp2); }
		if (exp1>exp3) { std::swap(k1,k3); std::swap(exp1,exp3); }
		if (exp2>exp3) { std::swap(k2,k3); std::swap(exp2,exp3); }

		return ( (k0 + k1) + k2 ) +k3;
	}
//@}


}; // end class definition

template <class T>
T Angle( const Point4<T>& p1, const Point4<T>  & p2 )
{
	T w = p1.Norm()*p2.Norm();
	if(w==0) return -1;
	T t = (p1*p2)/w;
	if(t>1) t=1;
	return T( math::Acos(t) );
}

template <class T>
inline T Norm( const Point4<T> & p )
{
	return p.Norm();
}

template <class T>
inline T SquaredNorm( const Point4<T> & p )
{
	return p.SquaredNorm();
}

template <class T>
inline T Distance( const Point4<T> & p1, const Point4<T> & p2 )
{
	return Norm(p1-p2);
}

template <class T>
inline T SquaredDistance( const Point4<T> & p1, const Point4<T> & p2 )
{
	return SquaredNorm(p1-p2);
}

	/// slower version of dot product, more stable (double precision only)
template<class T>
double StableDot ( Point4<T> const & p0, Point4<T> const & p1 )
{
	return p0.StableDot(p1);
}

typedef Point4<short>  Point4s;
typedef Point4<int>	   Point4i;
typedef Point4<float>  Point4f;
typedef Point4<double> Point4d;

/*@}*/
} // end namespace
#endif
