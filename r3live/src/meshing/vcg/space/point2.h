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
#include "deprecated_point2.h"
#else

#ifndef __VCGLIB_POINT2
#define __VCGLIB_POINT2

#include "../math/eigen.h"
// #include "point.h"

namespace vcg{
template<typename Scalar> class Point2;
}

namespace Eigen {
template<typename Scalar> struct ei_traits<vcg::Point2<Scalar> > : ei_traits<Eigen::Matrix<Scalar,2,1> > {};
template<typename XprType> struct ei_to_vcgtype<XprType,2,1,0,2,1>
{ typedef vcg::Point2<typename XprType::Scalar> type; };
}

namespace vcg {

/** \addtogroup space */
/*@{*/
/**
		The templated class for representing a point in 2D space.
		The class is templated over the Scalar class that is used to represent coordinates.
		All the usual operator overloading (* + - ...) is present.
	*/
template <class _Scalar> class Point2 : public Eigen::Matrix<_Scalar,2,1>
{
//----------------------------------------
// template typedef part
// use it as follow: typename Point2<S>::Type instead of simply Point2<S>
//----------------------------------------
public:
	typedef Eigen::Matrix<_Scalar,2,1> Type;
//----------------------------------------
// inheritence part
//----------------------------------------
private:
	typedef Eigen::Matrix<_Scalar,2,1> _Base;
public:
	using _Base::coeff;
	using _Base::coeffRef;
	using _Base::setZero;
	using _Base::data;
	using _Base::V;

	_EIGEN_GENERIC_PUBLIC_INTERFACE(Point2,_Base);
	VCG_EIGEN_INHERIT_ASSIGNMENT_OPERATORS(Point2)

	/// empty constructor (does nothing)
	inline Point2 () { }
	/// x,y constructor
	inline Point2 ( const Scalar nx, const Scalar ny ) : Base(nx,ny) {}
	/// copy constructor
	inline Point2(Point2 const & p) : Base(p) {}
	template<typename OtherDerived>
	inline Point2(const Eigen::MatrixBase<OtherDerived>& other) : Base(other) {}

	/// cross product
	// hm.. this is not really a cross product
	inline Scalar operator ^ ( Point2 const & p ) const
	{
		return data()[0]*p.data()[1] - data()[1]*p.data()[0];
	}

	/// returns the angle with X axis (radiants, in [-PI, +PI] )
	inline Scalar Angle() const
	{
		return math::Atan2(data()[1],data()[0]);
	}
	/// transform the point in cartesian coords into polar coords
	inline Point2 & Cartesian2Polar()
	{
		Scalar t = Angle();
		data()[0] = this->norm();
		data()[1] = t;
		return *this;
	}
	/// transform the point in polar coords into cartesian coords
	inline Point2 & Polar2Cartesian()
	{
		Scalar l = data()[0];
		data()[0] = (Scalar)(l*math::Cos(data()[1]));
		data()[1] = (Scalar)(l*math::Sin(data()[1]));
		return *this;
	}
	/// rotates the point of an angle (radiants, counterclockwise)
	inline Point2 & Rotate( const Scalar rad )
	{
		Scalar t = data()[0];
		Scalar s = math::Sin(rad);
		Scalar c = math::Cos(rad);

		data()[0] = data()[0]*c - data()[1]*s;
		data()[1] =   t *s + data()[1]*c;

		return *this;
	}
}; // end class definition

typedef Point2<short>  Point2s;
typedef Point2<int>	   Point2i;
typedef Point2<float>  Point2f;
typedef Point2<double> Point2d;

// typedef Eigen::Matrix<short ,2,1> Point2s;
// typedef Eigen::Matrix<int   ,2,1> Point2i;
// typedef Eigen::Matrix<float ,2,1> Point2f;
// typedef Eigen::Matrix<double,2,1> Point2d;
// typedef Eigen::Matrix<short ,2,1> Vector2s;
// typedef Eigen::Matrix<int   ,2,1> Vector2i;
// typedef Eigen::Matrix<float ,2,1> Vector2f;
// typedef Eigen::Matrix<double,2,1> Vector2d;

/*@}*/
} // end namespace
#endif

#endif
