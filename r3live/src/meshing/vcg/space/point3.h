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

#ifndef VCG_USE_EIGEN
#include "deprecated_point3.h"
#else

#ifndef __VCGLIB_POINT3
#define __VCGLIB_POINT3

#include "../math/eigen.h"

namespace vcg{
template<typename Scalar> class Point3;
}

namespace Eigen{

template<typename Scalar> struct ei_traits<vcg::Point3<Scalar> > : ei_traits<Eigen::Matrix<Scalar,3,1> > {};

template<typename XprType> struct ei_to_vcgtype<XprType,3,1,0,3,1>
{ typedef vcg::Point3<typename XprType::Scalar> type; };

template<typename Scalar>
struct NumTraits<vcg::Point3<Scalar> > : NumTraits<Scalar>
{
  enum {
    ReadCost = 3,
    AddCost = 3,
    MulCost = 3
  };
};

}

namespace vcg {

template<typename Scalar> class Box3;

/** \addtogroup space */
/*@{*/
/**
	The templated class for representing a point in 3D space.
	The class is templated over the ScalarType class that is used to represent coordinates. All the usual
	operator overloading (* + - ...) is present.
*/
template <class _Scalar> class Point3 : public Eigen::Matrix<_Scalar,3,1>
{
//----------------------------------------
// template typedef part
// use it as follow: typename Point3<S>::Type instead of simply Point3<S>
//----------------------------------------
public:
	typedef Eigen::Matrix<_Scalar,3,1> Type;
//----------------------------------------
// inheritence part
//----------------------------------------
private:
	typedef Eigen::Matrix<_Scalar,3,1> _Base;
public:

	using _Base::Construct;
	_EIGEN_GENERIC_PUBLIC_INTERFACE(Point3,_Base);
	VCG_EIGEN_INHERIT_ASSIGNMENT_OPERATORS(Point3)

  /** @name Standard Constructors and Initializers
   No casting operators have been introduced to avoid automatic unattended (and costly) conversion between different point types
   **/

  inline Point3 () {}
	inline Point3 ( const Scalar nx, const Scalar ny, const Scalar nz ) : Base(nx,ny,nz) {}
	inline Point3 ( Point3 const & p ) : Base(p) {}
	inline Point3 ( const Scalar nv[3] ) : Base(nv) {}
	template<typename OtherDerived>
	inline Point3(const Eigen::MatrixBase<OtherDerived>& other) : Base(other) {}


	// this one is very useless
  template <class Q>
  static inline Point3 Construct( const Q & P0, const Q & P1, const Q & P2)
  {
    return Point3(Scalar(P0),Scalar(P1),Scalar(P2));
  }
  vcg::Box3<_Scalar> GetBBox(vcg::Box3<_Scalar> &bb) const;

}; // end class definition (Point3)

	// Dot product preciso numericamente (solo double!!)
	// Implementazione: si sommano i prodotti per ordine di esponente
	// (prima le piu' grandi)
template<class Scalar>
double stable_dot ( Point3<Scalar> const & p0, Point3<Scalar> const & p1 )
{
	Scalar k0 = p0.data()[0]*p1.data()[0];
	Scalar k1 = p0.data()[1]*p1.data()[1];
	Scalar k2 = p0.data()[2]*p1.data()[2];

	int exp0,exp1,exp2;

	frexp( double(k0), &exp0 );
	frexp( double(k1), &exp1 );
	frexp( double(k2), &exp2 );

	if( exp0<exp1 )
	{
		if(exp0<exp2)
			return (k1+k2)+k0;
		else
			return (k0+k1)+k2;
	}
	else
	{
		if(exp1<exp2)
			return(k0+k2)+k1;
		else
			return (k0+k1)+k2;
	}
}

/// Point(p) Edge(v1-v2) dist, q is the point in v1-v2 with min dist
template<class Scalar>
Scalar PSDist( const Point3<Scalar> & p,
			         const Point3<Scalar> & v1,
					 const Point3<Scalar> & v2,
			         Point3<Scalar> & q )
{
    Point3<Scalar> e = v2-v1;
    Scalar  t = ((p-v1).dot(e))/e.SquaredNorm();
    if(t<0)      t = 0;
	else if(t>1) t = 1;
	q = v1+e*t;
    return Distance(p,q);
}

template <class Scalar>
void GetUV( Point3<Scalar> &n,Point3<Scalar> &u, Point3<Scalar> &v, Point3<Scalar> up=(Point3<Scalar>(0,1,0)) )
{
	n.Normalize();
	const double LocEps=double(1e-7);
	u=n^up;
  double len = u.Norm();
 	if(len < LocEps)
	{
		if(fabs(n[0])<fabs(n[1])){
			if(fabs(n[0])<fabs(n[2])) up=Point3<Scalar>(1,0,0); // x is the min
			                         else up=Point3<Scalar>(0,0,1); // z is the min
		}else {
			if(fabs(n[1])<fabs(n[2])) up=Point3<Scalar>(0,1,0); // y is the min
			                         else up=Point3<Scalar>(0,0,1); // z is the min
		}
		u=n^up;
	}
	u.Normalize();
	v=n^u;
	v.Normalize();
	Point3<Scalar> uv=u^v;
}

/*@}*/

typedef Point3<short>  Point3s;
typedef Point3<int>	   Point3i;
typedef Point3<float>  Point3f;
typedef Point3<double> Point3d;

// typedef Eigen::Matrix<short ,3,1> Point3s;
// typedef Eigen::Matrix<int   ,3,1> Point3i;
// typedef Eigen::Matrix<float ,3,1> Point3f;
// typedef Eigen::Matrix<double,3,1> Point3d;
// typedef Eigen::Matrix<short ,3,1> Vector3s;
// typedef Eigen::Matrix<int   ,3,1> Vector3i;
// typedef Eigen::Matrix<float ,3,1> Vector3f;
// typedef Eigen::Matrix<double,3,1> Vector3d;

} // end namespace

#endif

#endif
