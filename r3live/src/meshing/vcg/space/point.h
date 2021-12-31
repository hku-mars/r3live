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
#include "deprecated_point.h"
#else

#ifndef __VCGLIB_POINT
#define __VCGLIB_POINT

#include "../math/eigen.h"
#include <vcg/math/base.h>
#include <vcg/space/space.h>

namespace vcg {
template<typename Scalar> class Point2;
template<typename Scalar> class Point3;
template<typename Scalar> class Point4;

namespace ndim{
template <int Size, typename Scalar> class Point;
}
}

namespace vcg {
namespace ndim{

/** \addtogroup space */
/*@{*/
/**
		The templated class for representing a point in R^N space.
		The class is templated over the ScalarType class that is used to represent coordinates.
		PointBase provides the interface and the common operators for points
		of any dimensionality.
	*/
template <int N, class S> class Point : public Eigen::Matrix<S,N,1>
{
//----------------------------------------
// template typedef part
// use it as follow: typename Point<N,S>::Type instead of simply Point<N,S>
//----------------------------------------
public:
	typedef Eigen::Matrix<S,N,1> Type;
//----------------------------------------
// inheritence part
//----------------------------------------
private:
	typedef Eigen::Matrix<S,N,1> _Base;
public:

	using _Base::coeff;
	using _Base::coeffRef;
	using _Base::setZero;
	using _Base::data;
	using _Base::V;

	_EIGEN_GENERIC_PUBLIC_INTERFACE(Point,_Base);
	VCG_EIGEN_INHERIT_ASSIGNMENT_OPERATORS(Point)

	inline Point() : Base() {}
	template<typename OtherDerived>
	inline Point(const Eigen::MatrixBase<OtherDerived>& other) : Base(other) {}

	/// slower version, more stable (double precision only)
	inline S StableDot (const Point& p) const;

		/// Signed area operator
		/// a % b returns the signed area of the parallelogram inside a and b
		// inline S operator % ( PointType const & p ) const;

}; // end class definition


typedef Eigen::Matrix<short ,2,1> Point2s;
typedef Eigen::Matrix<int   ,2,1> Point2i;
typedef Eigen::Matrix<float ,2,1> Point2f;
typedef Eigen::Matrix<double,2,1> Point2d;
typedef Eigen::Matrix<short ,2,1> Vector2s;
typedef Eigen::Matrix<int   ,2,1> Vector2i;
typedef Eigen::Matrix<float ,2,1> Vector2f;
typedef Eigen::Matrix<double,2,1> Vector2d;

typedef Eigen::Matrix<short ,3,1> Point3s;
typedef Eigen::Matrix<int   ,3,1> Point3i;
typedef Eigen::Matrix<float ,3,1> Point3f;
typedef Eigen::Matrix<double,3,1> Point3d;
typedef Eigen::Matrix<short ,3,1> Vector3s;
typedef Eigen::Matrix<int   ,3,1> Vector3i;
typedef Eigen::Matrix<float ,3,1> Vector3f;
typedef Eigen::Matrix<double,3,1> Vector3d;


typedef Eigen::Matrix<short ,4,1> Point4s;
typedef Eigen::Matrix<int   ,4,1> Point4i;
typedef Eigen::Matrix<float ,4,1> Point4f;
typedef Eigen::Matrix<double,4,1> Point4d;
typedef Eigen::Matrix<short ,4,1> Vector4s;
typedef Eigen::Matrix<int   ,4,1> Vector4i;
typedef Eigen::Matrix<float ,4,1> Vector4f;
typedef Eigen::Matrix<double,4,1> Vector4d;


/*@}*/

} // end namespace ndim
} // end namespace vcg
#endif

#endif
