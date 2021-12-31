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
Revision 1.9  2006/12/20 15:23:52  ganovelli
using of locally defined variable removed

Revision 1.8  2006/04/11 08:10:05  zifnab1974
changes necessary for gcc 3.4.5 on linux 64bit.

Revision 1.7  2005/12/12 11:22:32  ganovelli
compiled with gcc

Revision 1.6  2005/01/12 11:25:52  ganovelli
corrected Point<3

Revision 1.5  2004/10/20 16:45:21  ganovelli
first compiling version (MC,INtel,gcc)

Revision 1.4  2004/04/29 10:47:06  ganovelli
some siyntax error corrected

Revision 1.3  2004/04/05 12:36:43  tarini
unified version: PointBase version, with no guards "(N==3)"



Revision 1.1  2004/03/16 03:07:38  tarini
"dimensionally unified" version: first commit

****************************************************************************/

#ifndef __VCGLIB_POINT
#define __VCGLIB_POINT

#include <assert.h>
#include <vcg/math/base.h>
#include <vcg/space/space.h>

namespace vcg {

    namespace ndim{


//template <int N, class S>
//class Point;

/** \addtogroup space */
/*@{*/
    /**
        The templated class for representing a point in R^N space.
        The class is templated over the ScalarType class that is used to represent coordinates.
                PointBase provides the interface and the common operators for points
                of any dimensionality.
     */

template <int N, class S>
class Point
{
public:
	typedef S          ScalarType;
	typedef VoidType   ParamType;
	typedef Point<N,S> PointType;
	enum {Dimension=N};


protected:
  /// The only data member. Hidden to user.
    S _v[N];

public:

//@{

  /** @name Standard Constructors and Initializers
   No casting operators have been introduced to avoid automatic unattended (and costly) conversion between different PointType types
   **/

  inline Point () { };
//	inline Point ( const S nv[N] );

  /// Padding function: give a default 0 value to all the elements that are not in the [0..2] range.
  /// Useful for managing in a consistent way object that could have point2 / point3 / point4
    inline S Ext( const int i ) const
    {
        if(i>=0 && i<=N) return _v[i];
        else             return 0;
    }

	/// importer for points with different scalar type and-or dimensionality
	template <int N2, class S2>
	inline void Import( const Point<N2,S2> & b )
	{
		_v[0] = ScalarType(b[0]);
		_v[1] = ScalarType(b[1]);
		if (N>2) { if (N2>2) _v[2] = ScalarType(b[2]); else _v[2] = 0;};
		if (N>3) { if (N2>3) _v[3] = ScalarType(b[3]); else _v[3] = 0;};
	}

	/// constructor for points with different scalar type and-or dimensionality
	template <int N2, class S2>
  static inline PointType Construct( const Point<N2,S2> & b )
  {
        PointType p; p.Import(b);
    return p;
  }

	  /// importer for homogeneous points
	template <class S2>
	inline void ImportHomo( const Point<N-1,S2> & b )
	{
		_v[0] = ScalarType(b[0]);
		_v[1] = ScalarType(b[1]);
		if (N>2) { _v[2] = ScalarType(_v[2]); };
		_v[N-1] = 1.0;
	}

		/// constructor for homogeneus point.
	template <int N2, class S2>
  static inline PointType Construct( const Point<N-1,S2> & b )
  {
        PointType p; p.ImportHomo(b);
    return p;
  }

//@}

//@{

  /** @name Data Access.
   access to data is done by overloading of [] or explicit naming of coords (x,y,z)**/

	inline S & operator [] ( const int i )
	{
		assert(i>=0 && i<N);
		return _v[i];
	}
	inline const S & operator [] ( const int i ) const
	{
		assert(i>=0 && i<N);
		return _v[i];
	}
  inline const S &X() const { return _v[0]; }
    inline const S &Y() const { return _v[1]; }
    inline const S &Z() const { static_assert(N>2); return _v[2]; }
     /// W is in any case the last coordinate.
     /// (in a 2D point, W() == Y(). In a 3D point, W()==Z()
     ///  in a 4D point, W() is a separate component)
    inline const S &W() const { return _v[N-1]; }
    inline S &X() { return _v[0]; }
    inline S &Y() { return _v[1]; }
    inline S &Z() { static_assert(N>2); return _v[2]; }
    inline S &W() { return _v[N-1]; }
    inline const S * V() const
    {
        return _v;
    }
    inline S & V( const int i )
    {
        assert(i>=0 && i<N);
        return _v[i];
    }
    inline const S & V( const int i ) const
    {
        assert(i>=0 && i<N);
        return _v[i];
    }
//@}
//@{

  /** @name Linearity for points
  **/

	/// sets a PointType to Zero
	inline void SetZero()
	{
		for(unsigned int ii = 0; ii < Dimension;++ii)
			V(ii) = S();
	}

	inline PointType operator + ( PointType const & p) const
	{
		PointType res;
		for(unsigned int ii = 0; ii < Dimension;++ii)
			res[ii] = V(ii) + p[ii];
		return res;
	}

	inline PointType operator - ( PointType const & p) const
	{
		PointType res;
		for(unsigned int ii = 0; ii < Dimension;++ii)
			res[ii] = V(ii) - p[ii];
		return res;
	}

	inline PointType operator * ( const S s ) const
	{
		PointType res;
		for(unsigned int ii = 0; ii < Dimension;++ii)
			res[ii] = V(ii) * s;
		return res;
	}

	inline PointType operator / ( const S s ) const
	{
		PointType res;
		for(unsigned int ii = 0; ii < Dimension;++ii)
			res[ii] = V(ii) / s;
		return res;
	}

	inline PointType & operator += ( PointType const & p)
	{
		for(unsigned int ii = 0; ii < Dimension;++ii)
			V(ii) += p[ii];
		return *this;
	}

	inline PointType & operator -= ( PointType const & p)
	{
		for(unsigned int ii = 0; ii < Dimension;++ii)
			V(ii) -= p[ii];
		return *this;
	}

	inline PointType & operator *= ( const S s )
	{
		for(unsigned int ii = 0; ii < Dimension;++ii)
			V(ii) *= s;
		return *this;
	}

	inline PointType & operator /= ( const S s )
	{
		for(unsigned int ii = 0; ii < Dimension;++ii)
			V(ii) *= s;
		return *this;
	}

	inline PointType operator - () const
	{
		PointType res;
		for(unsigned int ii = 0; ii < Dimension;++ii)
			res[ii] = - V(ii);
		return res;
	}
//@}
//@{

  /** @name Dot products (cross product "%" is defined olny for 3D points)
  **/
        /// Dot product
    inline S operator * ( PointType const & p ) const;

	/// slower version, more stable (double precision only)
	inline S StableDot ( const PointType & p ) const;

//@}

//@{

  /** @name Norms
  **/

		/// Euclidean norm
	inline S Norm() const;
		/// Euclidean norm, static version
	template <class PT> static S Norm(const PT &p );
		/// Squared Euclidean norm
	inline S SquaredNorm() const;
		/// Squared Euclidean norm, static version
	template <class PT> static S SquaredNorm(const PT &p );
		/// Normalization (division by norm)
	inline PointType & Normalize();
		/// Normalization (division by norm), static version
	template <class PT> static PointType & Normalize(const PT &p);
		/// Homogeneous normalization (division by W)
	inline PointType & HomoNormalize();

	  /// norm infinity: largest absolute value of compoenet
	inline S NormInfinity() const;
		/// norm 1: sum of absolute values of components
	inline S NormOne() const;

//@}
		/// Signed area operator
	  /// a % b returns the signed area of the parallelogram inside a and b
	inline S operator % ( PointType const & p ) const;

	  /// the sum of the components
	inline S Sum() const;
	  /// returns the biggest component
	inline S Max() const;
	  /// returns the smallest component
	inline S Min() const;
	  /// returns the index of the biggest component
	inline int MaxI() const;
	  /// returns the index of the smallest component
	inline int MinI() const;


	 /// Per component scaling
	inline PointType & Scale( const PointType & p );


	 /// Convert to polar coordinates
	void ToPolar( S & ro, S & tetha, S & fi ) const
	{
		ro = Norm();
		tetha = (S)atan2( _v[1], _v[0] );
		fi    = (S)acos( _v[2]/ro );
	}

//@{

  /** @name Comparison Operators.
   Lexicographic order.
   **/

	inline bool operator == ( PointType const & p ) const;
	inline bool operator != ( PointType const & p ) const;
	inline bool operator <  ( PointType const & p ) const;
	inline bool operator >  ( PointType const & p ) const;
	inline bool operator <= ( PointType const & p ) const;
	inline bool operator >= ( PointType const & p ) const;
 //@}

//@{

  /** @name
    Glocal to Local and viceversa
    (provided for uniformity with other spatial classes. trivial for points)
   **/

	inline PointType LocalToGlobal(ParamType p) const{
		return *this; }

  inline ParamType GlobalToLocal(PointType /*p*/) const{
      return ParamType(); }
//@}

}; // end class definition









template <class S>
class Point2 : public Point<2,S> {
public:
	typedef S ScalarType;
	typedef Point2 PointType;
	using Point<2,S>::_v;
	using Point<2,S>::V;
	using Point<2,S>::W;

    //@{
  /** @name Special members for 2D points. **/

	/// default
	inline Point2 (){}

	  /// yx constructor
	inline Point2 ( const S a, const S b){
		_v[0]=a; _v[1]=b;  };

	  /// unary orthogonal operator (2D equivalent of cross product)
	  /// returns orthogonal vector (90 deg left)
	inline Point2 operator ~ () const {
		return Point2 ( -_v[2], _v[1] );
	}

	  /// returns the angle with X axis (radiants, in [-PI, +PI] )
	inline ScalarType &Angle(){
		return math::Atan2(_v[1],_v[0]);}

		/// transform the point in cartesian coords into polar coords
	inline Point2 & ToPolar(){
		ScalarType t = Angle();
		_v[0] = Norm();
		_v[1] = t;
		return *this;}

		/// transform the point in polar coords into cartesian coords
	inline Point2 & ToCartesian() {
		ScalarType l = _v[0];
		_v[0] = (ScalarType)(l*math::Cos(_v[1]));
		_v[1] = (ScalarType)(l*math::Sin(_v[1]));
		return *this;}

			/// rotates the point of an angle (radiants, counterclockwise)
	inline Point2 & Rotate( const ScalarType rad ){
		ScalarType t = _v[0];
		ScalarType s = math::Sin(rad);
		ScalarType c = math::Cos(rad);
		_v[0] = _v[0]*c - _v[1]*s;
		_v[1] =   t *s + _v[1]*c;
		return *this;}

	 //@}

    //@{
  /** @name Implementation of standard functions for 3D points **/

	inline void Zero(){
		_v[0]=0; _v[1]=0;  };

	inline Point2 ( const S nv[2] ){
		_v[0]=nv[0]; _v[1]=nv[1];   };

	inline Point2 operator + ( Point2 const & p) const {
		return Point2( _v[0]+p._v[0], _v[1]+p._v[1]); }

	inline Point2 operator - ( Point2 const & p) const {
		return Point2( _v[0]-p._v[0], _v[1]-p._v[1]); }

	inline Point2 operator * ( const S s ) const {
		return Point2( _v[0]*s, _v[1]*s  ); }

	inline Point2 operator / ( const S s ) const {
		S t=1.0/s;
		return Point2( _v[0]*t, _v[1]*t   ); }

	inline Point2 operator - () const {
		return Point2 ( -_v[0], -_v[1]  ); }

	inline Point2 & operator += ( Point2 const & p ) {
		_v[0] += p._v[0]; _v[1] += p._v[1];   return *this; }

	inline Point2 & operator -= ( Point2 const & p ) {
		_v[0] -= p._v[0]; _v[1] -= p._v[1];   return *this; }

	inline Point2 & operator *= ( const S s ) {
		_v[0] *= s; _v[1] *= s;  return *this; }

	inline Point2 & operator /= ( const S s ) {
		S t=1.0/s; _v[0] *= t; _v[1] *= t;   return *this; }

	inline S Norm() const {
		return math::Sqrt( _v[0]*_v[0] + _v[1]*_v[1]   );}

	template <class PT> static S Norm(const PT &p ) {
		return math::Sqrt( p.V(0)*p.V(0) + p.V(1)*p.V(1)  );}

	inline S SquaredNorm() const {
		return ( _v[0]*_v[0] + _v[1]*_v[1] );}

	template <class PT> static S SquaredNorm(const PT &p ) {
		return ( p.V(0)*p.V(0) + p.V(1)*p.V(1) );}

    inline S operator * ( Point2 const & p ) const {
    return ( _v[0]*p._v[0] + _v[1]*p._v[1]) ; }

	inline bool operator == ( Point2 const & p ) const {
		return _v[0]==p._v[0] && _v[1]==p._v[1] ;}

	inline bool operator != ( Point2 const & p ) const {
		return _v[0]!=p._v[0] || _v[1]!=p._v[1] ;}

	inline bool operator <  ( Point2 const & p ) const{
		return (_v[1]!=p._v[1])?(_v[1]< p._v[1]) : (_v[0]<p._v[0]); }

	inline bool operator >  ( Point2 const & p ) const	{
		return (_v[1]!=p._v[1])?(_v[1]> p._v[1]) : (_v[0]>p._v[0]); }

	inline bool operator <= ( Point2 const & p ) {
		return (_v[1]!=p._v[1])?(_v[1]< p._v[1]) : (_v[0]<=p._v[0]); }

	inline bool operator >= ( Point2 const & p ) const {
		return (_v[1]!=p._v[1])?(_v[1]> p._v[1]) : (_v[0]>=p._v[0]); }

	inline Point2 & Normalize() {
		PointType n = Norm(); if(n!=0.0) { n=1.0/n;	_v[0]*=n;	_v[1]*=n;} return *this;}

	inline Point2 & HomoNormalize(){
		if (_v[2]!=0.0) {	_v[0] /= W(); W()=1.0; } return *this;}

	inline S NormInfinity() const {
		return math::Max( math::Abs(_v[0]), math::Abs(_v[1]) ); }

	inline S NormOne() const {
		return math::Abs(_v[0])+ math::Abs(_v[1]);}

	inline S operator % ( Point2 const & p ) const {
		return _v[0] * p._v[1] - _v[1] * p._v[0]; }

	inline S Sum() const {
		return _v[0]+_v[1];}

	inline S Max() const {
		return math::Max( _v[0], _v[1] ); }

	inline S Min() const {
		return math::Min( _v[0], _v[1] ); }

	inline int MaxI() const {
		return (_v[0] < _v[1]) ? 1:0; };

	inline int MinI() const {
	  return (_v[0] > _v[1]) ? 1:0; };

	inline PointType & Scale( const PointType & p ) {
		_v[0] *= p._v[0]; _v[1] *= p._v[1];  return *this; }

	inline S StableDot ( const PointType & p ) const {
		return _v[0]*p._v[0] +_v[1]*p._v[1]; }
	 //@}

};

template <typename S>
class Point3 : public Point<3,S> {
public:
	typedef S ScalarType;
	typedef Point3<S>  PointType;
	using Point<3,S>::_v;
	using Point<3,S>::V;
	using Point<3,S>::W;


    //@{
  /** @name Special members for 3D points. **/

	/// default
	inline Point3 ():Point<3,S>(){}
	  /// yxz constructor
	inline Point3 ( const S a, const S b,  const S c){
		_v[0]=a; _v[1]=b; _v[2]=c; };

		/// Cross product for 3D points
	inline PointType operator ^ ( PointType const & p ) const {
		return Point3 (
			_v[1]*p._v[2] - _v[2]*p._v[1],
			_v[2]*p._v[0] - _v[0]*p._v[2],
			_v[0]*p._v[1] - _v[1]*p._v[0] );
	}
	 //@}

    //@{
  /** @name Implementation of standard functions for 3D points **/

	inline void Zero(){
		_v[0]=0; _v[1]=0; _v[2]=0; };

	inline Point3 ( const S nv[3] ){
		_v[0]=nv[0]; _v[1]=nv[1]; _v[2]=nv[2];  };

	inline Point3 operator + ( Point3 const & p)  const{
		return Point3( _v[0]+p._v[0], _v[1]+p._v[1], _v[2]+p._v[2]); }

	inline Point3 operator - ( Point3 const & p) const {
		return Point3( _v[0]-p._v[0], _v[1]-p._v[1], _v[2]-p._v[2]); }

	inline Point3 operator * ( const S s ) const {
		return Point3( _v[0]*s, _v[1]*s , _v[2]*s  ); }

	inline Point3 operator / ( const S s ) const {
		S t=1.0/s;
		return Point3( _v[0]*t, _v[1]*t , _v[2]*t  ); }

	inline Point3 operator - () const {
		return Point3 ( -_v[0], -_v[1] , -_v[2]  ); }

	inline Point3 & operator += ( Point3 const & p ) {
		_v[0] += p._v[0]; _v[1] += p._v[1]; _v[2] += p._v[2];  return *this; }

	inline Point3 & operator -= ( Point3 const & p ) {
		_v[0] -= p._v[0]; _v[1] -= p._v[1]; _v[2] -= p._v[2];  return *this; }

	inline Point3 & operator *= ( const S s ) {
		_v[0] *= s; _v[1] *= s; _v[2] *= s; return *this; }

	inline Point3 & operator /= ( const S s ) {
		S t=1.0/s; _v[0] *= t; _v[1] *= t;  _v[2] *= t;   return *this; }

	inline S Norm() const {
		return math::Sqrt( _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2]  );}

	template <class PT> static S Norm(const PT &p ) {
		return math::Sqrt( p.V(0)*p.V(0) + p.V(1)*p.V(1) + p.V(2)*p.V(2)  );}

	inline S SquaredNorm() const {
		return ( _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2]  );}

	template <class PT> static S SquaredNorm(const PT &p ) {
		return ( p.V(0)*p.V(0) + p.V(1)*p.V(1) + p.V(2)*p.V(2)  );}

    inline S operator * ( PointType const & p ) const {
    return ( _v[0]*p._v[0] + _v[1]*p._v[1] + _v[2]*p._v[2]) ; }

	inline bool operator == ( PointType const & p ) const {
		return _v[0]==p._v[0] && _v[1]==p._v[1] && _v[2]==p._v[2] ;}

	inline bool operator != ( PointType const & p ) const {
		return _v[0]!=p._v[0] || _v[1]!=p._v[1] || _v[2]!=p._v[2] ;}

	inline bool operator <  ( PointType const & p ) const{
		return	(_v[2]!=p._v[2])?(_v[2]< p._v[2]):
					(_v[1]!=p._v[1])?(_v[1]< p._v[1]) : (_v[0]<p._v[0]); }

	inline bool operator >  ( PointType const & p ) const	{
		return	(_v[2]!=p._v[2])?(_v[2]> p._v[2]):
					(_v[1]!=p._v[1])?(_v[1]> p._v[1]) : (_v[0]>p._v[0]); }

	inline bool operator <= ( PointType const & p ) {
		return	(_v[2]!=p._v[2])?(_v[2]< p._v[2]):
					(_v[1]!=p._v[1])?(_v[1]< p._v[1]) : (_v[0]<=p._v[0]); }

	inline bool operator >= ( PointType const & p ) const {
		return	(_v[2]!=p._v[2])?(_v[2]> p._v[2]):
					(_v[1]!=p._v[1])?(_v[1]> p._v[1]) : (_v[0]>=p._v[0]); }

	inline PointType & Normalize() {
		S n = Norm();
		if(n!=0.0) {
			n=S(1.0)/n;
		_v[0]*=n;	_v[1]*=n;	_v[2]*=n; }
		return *this;}

	inline PointType & HomoNormalize(){
		if (_v[2]!=0.0) {	_v[0] /= W();	_v[1] /= W(); W()=1.0; }
		return *this;}

	inline S NormInfinity() const {
		return math::Max( math::Max( math::Abs(_v[0]), math::Abs(_v[1]) ),
							math::Abs(_v[3])  ); }

	inline S NormOne() const {
		return math::Abs(_v[0])+ math::Abs(_v[1])+math::Max(math::Abs(_v[2]));}

	inline S operator % ( PointType const & p ) const {
		S t = (*this)*p; /* Area, general formula */
		return math::Sqrt( SquaredNorm() * p.SquaredNorm() - (t*t) );};

	inline S Sum() const {
		return _v[0]+_v[1]+_v[2];}

	inline S Max() const {
		return math::Max( math::Max( _v[0], _v[1] ),  _v[2] ); }

	inline S Min() const {
		return math::Min( math::Min( _v[0], _v[1] ),  _v[2] ); }

	inline int MaxI() const {
		int i= (_v[0] < _v[1]) ? 1:0; if (_v[i] < _v[2]) i=2; return i;};

	inline int MinI() const {
		int i= (_v[0] > _v[1]) ? 1:0; if (_v[i] > _v[2]) i=2; return i;};

	inline PointType & Scale( const PointType & p ) {
		_v[0] *= p._v[0]; _v[1] *= p._v[1]; _v[2] *= p._v[2]; return *this; }

	inline S StableDot ( const PointType & p ) const {
		S k0=_v[0]*p._v[0],	k1=_v[1]*p._v[1], k2=_v[2]*p._v[2];
		int exp0,exp1,exp2;
		frexp( double(k0), &exp0 );
		frexp( double(k1), &exp1 );
		frexp( double(k2), &exp2 );
		if( exp0<exp1 )
			if(exp0<exp2) return (k1+k2)+k0; else return (k0+k1)+k2;
		else
			if(exp1<exp2) return (k0+k2)+k1; else return (k0+k1)+k2;
	}
	 //@}

};

template <typename S>
class Point4 : public Point<4,S> {
public:
	typedef S ScalarType;
	typedef Point4<S> PointType;
	using Point<3,S>::_v;
	using Point<3,S>::V;
	using Point<3,S>::W;

    //@{
  /** @name Special members for 4D points. **/
        /// default
        inline Point4 (){}

	  /// xyzw constructor
	//@}
	inline Point4 ( const S a, const S b,  const S c, const S d){
		_v[0]=a; _v[1]=b; _v[2]=c; _v[3]=d; };
	//@{
  /** @name Implementation of standard functions for 3D points **/

	inline void Zero(){
		_v[0]=0; _v[1]=0; _v[2]=0; _v[3]=0; };

	inline Point4 ( const S nv[4] ){
		_v[0]=nv[0]; _v[1]=nv[1]; _v[2]=nv[2]; _v[3]=nv[3]; };

	inline Point4 operator + ( Point4 const & p) const {
		return Point4( _v[0]+p._v[0], _v[1]+p._v[1], _v[2]+p._v[2], _v[3]+p._v[3] ); }

	inline Point4 operator - ( Point4 const & p) const {
		return Point4( _v[0]-p._v[0], _v[1]-p._v[1], _v[2]-p._v[2], _v[3]-p._v[3] ); }

	inline Point4 operator * ( const S s ) const {
		return Point4( _v[0]*s, _v[1]*s , _v[2]*s , _v[3]*s ); }

	inline PointType operator ^ ( PointType const & p ) const {
		assert(0);
		return *this;
	}

	inline Point4 operator / ( const S s ) const {
		S t=1.0/s;
		return Point4( _v[0]*t, _v[1]*t , _v[2]*t , _v[3]*t ); }

	inline Point4 operator - () const {
		return Point4 ( -_v[0], -_v[1] , -_v[2] , -_v[3] ); }

	inline Point4 & operator += ( Point4 const & p ) {
		_v[0] += p._v[0]; _v[1] += p._v[1]; _v[2] += p._v[2]; _v[3] += p._v[3]; return *this; }

	inline Point4 & operator -= ( Point4 const & p ) {
		_v[0] -= p._v[0]; _v[1] -= p._v[1]; _v[2] -= p._v[2]; _v[3] -= p._v[3]; return *this; }

	inline Point4 & operator *= ( const S s ) {
		_v[0] *= s; _v[1] *= s; _v[2] *= s; _v[3] *= s; return *this; }

	inline Point4 & operator /= ( const S s ) {
		S t=1.0/s; _v[0] *= t; _v[1] *= t;  _v[2] *= t;  _v[3] *= t; return *this; }

	inline S Norm() const {
		return math::Sqrt( _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2] + _v[3]*_v[3] );}

	template <class PT> static S Norm(const PT &p ) {
		return math::Sqrt( p.V(0)*p.V(0) + p.V(1)*p.V(1) + p.V(2)*p.V(2) + p.V(3)*p.V(3) );}

	inline S SquaredNorm() const {
		return ( _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2] + _v[3]*_v[3] );}

	template <class PT> static S SquaredNorm(const PT &p ) {
		return ( p.V(0)*p.V(0) + p.V(1)*p.V(1) + p.V(2)*p.V(2) + p.V(3)*p.V(3) );}

    inline S operator * ( PointType const & p ) const {
    return ( _v[0]*p._v[0] + _v[1]*p._v[1] + _v[2]*p._v[2] + _v[3]*p._v[3] ); }

	inline bool operator == ( PointType const & p ) const {
		return _v[0]==p._v[0] && _v[1]==p._v[1] && _v[2]==p._v[2] && _v[3]==p._v[3];}

	inline bool operator != ( PointType const & p ) const {
		return _v[0]!=p._v[0] || _v[1]!=p._v[1] || _v[2]!=p._v[2] || _v[3]!=p._v[3];}

	inline bool operator <  ( PointType const & p ) const{
		return	(_v[3]!=p._v[3])?(_v[3]< p._v[3]) : (_v[2]!=p._v[2])?(_v[2]< p._v[2]):
					(_v[1]!=p._v[1])?(_v[1]< p._v[1]) : (_v[0]<p._v[0]); }

	inline bool operator >  ( PointType const & p ) const	{
		return	(_v[3]!=p._v[3])?(_v[3]> p._v[3]) : (_v[2]!=p._v[2])?(_v[2]> p._v[2]):
					(_v[1]!=p._v[1])?(_v[1]> p._v[1]) : (_v[0]>p._v[0]); }

	inline bool operator <= ( PointType const & p ) {
		return	(_v[3]!=p._v[3])?(_v[3]< p._v[3]) : (_v[2]!=p._v[2])?(_v[2]< p._v[2]):
					(_v[1]!=p._v[1])?(_v[1]< p._v[1]) : (_v[0]<=p._v[0]); }

	inline bool operator >= ( PointType const & p ) const {
		return	(_v[3]!=p._v[3])?(_v[3]> p._v[3]) : (_v[2]!=p._v[2])?(_v[2]> p._v[2]):
					(_v[1]!=p._v[1])?(_v[1]> p._v[1]) : (_v[0]>=p._v[0]); }

	inline PointType & Normalize() {
		PointType n = Norm(); if(n!=0.0) { n=1.0/n;	_v[0]*=n;	_v[1]*=n;	_v[2]*=n; _v[3]*=n; }
		return *this;};

	template <class PT>  PointType & Normalize(const PT &p){
		PointType n = Norm(); if(n!=0.0) { n=1.0/n;	V(0)*=n;	V(1)*=n;	V(2)*=n; V(3)*=n; }
		return *this;};

	inline PointType & HomoNormalize(){
		if (_v[3]!=0.0) {	_v[0] /= W();	_v[1] /= W();	_v[2] /= W(); W()=1.0; }
		return *this;};

	inline S NormInfinity() const {
		return math::Max( math::Max( math::Abs(_v[0]), math::Abs(_v[1]) ),
							math::Max( math::Abs(_v[2]), math::Abs(_v[3]) ) ); }

	inline S NormOne() const {
		return math::Abs(_v[0])+ math::Abs(_v[1])+math::Max(math::Abs(_v[2]),math::Abs(_v[3]));}

	inline S operator % ( PointType const & p ) const {
		S t = (*this)*p; /* Area, general formula */
		return math::Sqrt( SquaredNorm() * p.SquaredNorm() - (t*t) );};

	inline S Sum() const {
		return _v[0]+_v[1]+_v[2]+_v[3];}

	inline S Max() const {
		return math::Max( math::Max( _v[0], _v[1] ), math::Max( _v[2], _v[3] )); }

	inline S Min() const {
		return math::Min( math::Min( _v[0], _v[1] ), math::Min( _v[2], _v[3] )); }

	inline int MaxI() const {
		int i= (_v[0] < _v[1]) ? 1:0; if (_v[i] < _v[2]) i=2; if (_v[i] < _v[3]) i=3;
		return i;};

	inline int MinI() const {
		int i= (_v[0] > _v[1]) ? 1:0; if (_v[i] > _v[2]) i=2; if (_v[i] > _v[3]) i=3;
		return i;};

	inline PointType & Scale( const PointType & p ) {
		_v[0] *= p._v[0]; _v[1] *= p._v[1]; _v[2] *= p._v[2]; _v[3] *= p._v[3]; return *this; }

	inline S StableDot ( const PointType & p ) const {
		S k0=_v[0]*p._v[0],	k1=_v[1]*p._v[1], k2=_v[2]*p._v[2], k3=_v[3]*p._v[3];
		int exp0,exp1,exp2,exp3;
		frexp( double(k0), &exp0 );frexp( double(k1), &exp1 );
		frexp( double(k2), &exp2 );frexp( double(k3), &exp3 );
		if (exp0>exp1) { std::swap(k0,k1); std::swap(exp0,exp1); }
		if (exp2>exp3) { std::swap(k2,k3); std::swap(exp2,exp3); }
		if (exp0>exp2) { std::swap(k0,k2); std::swap(exp0,exp2); }
		if (exp1>exp3) { std::swap(k1,k3); std::swap(exp1,exp3); }
		if (exp2>exp3) { std::swap(k2,k3); std::swap(exp2,exp3); }
		return ( (k0 + k1) + k2 ) +k3; }

	 //@}
};


template <class S>
inline S Angle( Point3<S>  const & p1, Point3<S>  const & p2 )
{
	S w = p1.Norm()*p2.Norm();
	if(w==0) return -1;
	S t = (p1*p2)/w;
	if(t>1) t = 1;
	else if(t<-1) t = -1;
	return (S) acos(t);
}

// versione uguale alla precedente ma che assume che i due vettori siano unitari
template <class S>
inline S AngleN( Point3<S>  const & p1, Point3<S>  const & p2 )
{
	S w = p1*p2;
	if(w>1)
		w = 1;
	else if(w<-1)
		w=-1;
  return (S) acos(w);
}


template <int N,class S>
inline S Norm( Point<N,S> const & p )
{
    return p.Norm();
}

template <int N,class S>
inline S SquaredNorm( Point<N,S> const & p )
{
  return p.SquaredNorm();
}

template <int N,class S>
inline Point<N,S> & Normalize( Point<N,S> & p )
{
  p.Normalize();
  return p;
}

template <int N, class S>
inline S Distance( Point<N,S> const & p1,Point<N,S> const & p2 )
{
    return (p1-p2).Norm();
}

template <int N, class S>
inline S SquaredDistance( Point<N,S> const & p1,Point<N,S> const & p2 )
{
    return (p1-p2).SquaredNorm();
}


//template <typename S>
//struct Point2:public Point<2,S>{
//	inline Point2(){};
//	inline Point2(Point<2,S> const & p):Point<2,S>(p){} ;
//	inline Point2( const S a, const S b):Point<2,S>(a,b){};
//};
//
//template <typename S>
//struct Point3:public Point3<S> {
//	inline Point3(){};
//	inline Point3(Point3<S>  const & p):Point3<S> (p){}
//	inline Point3( const S a, const S b,  const S c):Point3<S> (a,b,c){};
//};
//
//
//template <typename S>
//struct Point4:public Point4<S>{
//	inline Point4(){};
//	inline Point4(Point4<S> const & p):Point4<S>(p){}
//	inline Point4( const S a, const S b,  const S c, const S d):Point4<S>(a,b,c,d){};
//};

typedef Point2<short>  Point2s;
typedef Point2<int>	  Point2i;
typedef Point2<float>  Point2f;
typedef Point2<double> Point2d;
typedef Point2<short>  Vector2s;
typedef Point2<int>	  Vector2i;
typedef Point2<float>  Vector2f;
typedef Point2<double> Vector2d;

typedef Point3<short>  Point3s;
typedef Point3<int>	  Point3i;
typedef Point3<float>  Point3f;
typedef Point3<double> Point3d;
typedef Point3<short>  Vector3s;
typedef Point3<int>	  Vector3i;
typedef Point3<float>  Vector3f;
typedef Point3<double> Vector3d;


typedef Point4<short>  Point4s;
typedef Point4<int>	  Point4i;
typedef Point4<float>  Point4f;
typedef Point4<double> Point4d;
typedef Point4<short>  Vector4s;
typedef Point4<int>	  Vector4i;
typedef Point4<float>  Vector4f;
typedef Point4<double> Vector4d;

/*@}*/

//added only for backward compatibility
template<unsigned int N,typename S>
struct PointBase : Point<N,S>
{
	PointBase()
		:Point<N,S>()
	{
	}
};

} // end namespace ndim
} // end namespace vcg
#endif

