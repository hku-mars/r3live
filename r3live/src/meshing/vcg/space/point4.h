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
#include "deprecated_point4.h"
#else

#ifndef __VCGLIB_POINT4
#define __VCGLIB_POINT4

#include "../math/eigen.h"

namespace vcg{
template<typename Scalar> class Point4;
}

namespace Eigen {
template<typename Scalar> struct ei_traits<vcg::Point4<Scalar> > : ei_traits<Eigen::Matrix<Scalar,4,1> > {};
template<typename XprType> struct ei_to_vcgtype<XprType,4,1,0,4,1>
{ typedef vcg::Point4<typename XprType::Scalar> type; };
}

namespace vcg {


/** \addtogroup space */
/*@{*/
/**
		The templated class for representing a point in 4D space.
		The class is templated over the ScalarType class that is used to represent coordinates.
		All the usual operator (* + - ...) are defined.
	*/
template <class T> class Point4 : public Eigen::Matrix<T,4,1>
{
//----------------------------------------
// template typedef part
// use it as follow: typename Point4<S>::Type instead of simply Point4<S>
//----------------------------------------
public:
	typedef Eigen::Matrix<T,4,1> Type;
//----------------------------------------
// inheritence part
//----------------------------------------
private:
	typedef Eigen::Matrix<T,4,1> _Base;
public:
	using _Base::coeff;
	using _Base::coeffRef;
	using _Base::setZero;
	using _Base::data;

	_EIGEN_GENERIC_PUBLIC_INTERFACE(Point4,_Base);
	typedef Scalar ScalarType;

	VCG_EIGEN_INHERIT_ASSIGNMENT_OPERATORS(Point4)

	inline Point4() : Base() {}
	inline Point4( const T nx, const T ny, const T nz , const T nw ) : Base(nx,ny,nz,nw) {}
	inline Point4(const T p[4]) : Base(p) {}
	inline Point4(const Point4& p) : Base(p) {}
	template<typename OtherDerived>
	inline Point4(const Eigen::MatrixBase<OtherDerived>& other) : Base(other) {}


	inline Point4 VectProd ( const Point4 &x, const Point4 &z ) const
	{
		Point4 res;
		const Point4 &y = *this;

		res[0] =  y[1]*x[2]*z[3]-y[1]*x[3]*z[2]-x[1]*y[2]*z[3]+
				  x[1]*y[3]*z[2]+z[1]*y[2]*x[3]-z[1]*y[3]*x[2];
		res[1] =  y[0]*x[3]*z[2]-z[0]*y[2]*x[3]-y[0]*x[2]*
				  z[3]+z[0]*y[3]*x[2]+x[0]*y[2]*z[3]-x[0]*y[3]*z[2];
		res[2] = -y[0]*z[1]*x[3]+x[0]*z[1]*y[3]+y[0]*x[1]*
				z[3]-x[0]*y[1]*z[3]-z[0]*x[1]*y[3]+z[0]*y[1]*x[3];
		res[3] = -z[0]*y[1]*x[2]-y[0]*x[1]*z[2]+x[0]*y[1]*
				  z[2]+y[0]*z[1]*x[2]-x[0]*z[1]*y[2]+z[0]*x[1]*y[2];
		return res;
	}

//@{
  /** @name Dot products
  **/

	inline Point4 operator ^ (  const Point4& p ) const
	{
		assert(0 && "not defined by two vectors (only put for metaprogramming)");
		return Point4();
	}

	/// slower version, more stable (double precision only)
	T StableDot ( const Point4<T> & p ) const
	{
		T k0=data()[0]*p.data()[0],	k1=data()[1]*p.data()[1], k2=data()[2]*p.data()[2], k3=data()[3]*p.data()[3];
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


typedef Point4<short>  Point4s;
typedef Point4<int>	   Point4i;
typedef Point4<float>  Point4f;
typedef Point4<double> Point4d;

// typedef Eigen::Matrix<short ,4,1> Point4s;
// typedef Eigen::Matrix<int   ,4,1> Point4i;
// typedef Eigen::Matrix<float ,4,1> Point4f;
// typedef Eigen::Matrix<double,4,1> Point4d;
// typedef Eigen::Matrix<short ,4,1> Vector4s;
// typedef Eigen::Matrix<int   ,4,1> Vector4i;
// typedef Eigen::Matrix<float ,4,1> Vector4f;
// typedef Eigen::Matrix<double,4,1> Vector4d;

/// slower version of dot product, more stable (double precision only)
template<class T>
double StableDot ( Point4<T> const & p0, Point4<T> const & p1 )
{
	return p0.StableDot(p1);
}

/*@}*/
} // end namespace
#endif

#endif
