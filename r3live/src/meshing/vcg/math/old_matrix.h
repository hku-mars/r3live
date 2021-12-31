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
#include "deprecated_matrix.h"

#else

#ifndef MATRIX_VCGLIB
#define MATRIX_VCGLIB

#include "eigen.h"
#include <vcg/space/point.h>

namespace vcg{
namespace ndim{
template<class Scalar> class Matrix;
}
}

namespace Eigen{
template<typename Scalar>
struct ei_traits<vcg::ndim::Matrix<Scalar> > : ei_traits<Eigen::Matrix<Scalar,Dynamic,Dynamic> > {};
template<typename XprType> struct ei_to_vcgtype<XprType,Dynamic,Dynamic,RowMajor,Dynamic,Dynamic>
{ typedef vcg::ndim::Matrix<typename XprType::Scalar> type; };
}

namespace vcg{
namespace ndim{

/** \addtogroup math */
/* @{ */

/*!
 * \deprecated use Matrix<Scalar,Rows,Cols> or Matrix<Scalar,Dynamic,Dynamic> or any typedef
 * This class represent a generic <I>m</I>�<I>n</I> matrix. The class is templated over the scalar type field.
 * @param Scalar (Templete Parameter) Specifies the ScalarType field.
 */
template<class _Scalar>
class Matrix : public Eigen::Matrix<_Scalar,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> // FIXME col or row major ?
{
	typedef Eigen::Matrix<_Scalar,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> _Base;

public:

	_EIGEN_GENERIC_PUBLIC_INTERFACE(Matrix,_Base);
	typedef _Scalar ScalarType;
	VCG_EIGEN_INHERIT_ASSIGNMENT_OPERATORS(Matrix)

	/*!
	*	Default constructor
	* All the elements are initialized to zero.
	*	\param m the number of matrix rows
	* \param n the number of matrix columns
	*/
	Matrix(int m, int n)
		: Base(m,n)
	{
		memset(Base::data(), 0, m*n*sizeof(Scalar));
	}

	/*!
	*	Constructor
	* The matrix elements are initialized with the values of the elements in \i values.
	*	\param m the number of matrix rows
	* \param n the number of matrix columns
	*	\param values the values of the matrix elements
	*/
	Matrix(int m, int n, Scalar *values)
		: Base(m,n)
	{
		*this = Eigen::Map<Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >(values, m , n);
	}

	/*!
	*	Empty constructor
	*   Just create the object
	*/
	Matrix() : Base() {}

	/*!
	*	Copy constructor
	*	The matrix elements are initialized with the value of the corresponding element in \i m
	* \param m the matrix to be copied
	*/
	Matrix(const Matrix<Scalar> &m) : Base(m) {}

	template<typename OtherDerived>
	Matrix(const Eigen::MatrixBase<OtherDerived> &m) : Base(m) {}

	/*!
	*	Default destructor
	*/
	~Matrix() {}

	/*!
	* \deprecated use *this.row(i)
	*	Subscript operator:
	* \param i	the index of the row
	*	\return a reference to the <I>i</I>-th matrix row
	*/
	inline typename Base::RowXpr operator[](const unsigned int i)
	{ return Base::row(i); }

	/*!
	* \deprecated use *this.row(i)
	*	Const subscript operator
	* \param i	the index of the row
	*	\return a reference to the <I>i</I>-th matrix row
	*/
	inline const typename Base::RowXpr operator[](const unsigned int i) const
	{ return Base::row(i); }


	/*!
	*	Matrix multiplication: calculates the cross product.
	*	\param	reference to the matrix to multiply by
	*	\return the matrix product
	*/
	// FIXME what the hell is that !
	/*template <int N,int M>
	void DotProduct(Point<N,Scalar> &m,Point<M,Scalar> &result)
	{
		unsigned int i, j,  p,  r;
		for (i=0, p=0, r=0; i<M; i++)
		{ result[i]=0;
			for (j=0; j<N; j++)
				result[i]+=(*this)[i][j]*m[j];
		}
	};*/

	/*!
	* \deprecated use *this.resize(); *this.setZero();
	*	Resize the current matrix.
	*	\param m the number of matrix rows.
	* \param n the number of matrix columns.
	*/
	void Resize(const unsigned int m, const unsigned int n)
	{
		assert(m>=2);
		assert(n>=2);
		Base::resize(m,n);
		memset(Base::data(), 0, m*n*sizeof(Scalar));
	};
};

typedef vcg::ndim::Matrix<double> MatrixMNd;
typedef vcg::ndim::Matrix<float>  MatrixMNf;

/*! @} */

template <class MatrixType>
void Invert(MatrixType & m)
{
	m = m.inverse();
}

}
} // end of namespace

#endif

#endif

