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

#ifdef __GNUC__
#warning You are including deprecated math stuff
#endif

typedef Scalar ScalarType;

/*! \deprecated use cols() */
EIGEN_DEPRECATED inline unsigned int ColumnsNumber() const { return cols(); };


/*! \deprecated use rows() */
EIGEN_DEPRECATED inline unsigned int RowsNumber() const { return rows(); };

/*!
*	\deprecated use *this(i,j) (or *this.coeff(i,j))
* Return the element stored in the <I>i</I>-th rows at the <I>j</I>-th column
*	\param i the row index
*	\param j the column index
*	\return the element
*/
EIGEN_DEPRECATED inline Scalar ElementAt(unsigned int i, unsigned int j) const { return (*this)(i,j); };
EIGEN_DEPRECATED inline Scalar& ElementAt(unsigned int i, unsigned int j) { return (*this)(i,j); };
EIGEN_DEPRECATED inline Scalar V(int i) const { return (*this)[i]; };
EIGEN_DEPRECATED inline Scalar& V(int i) { return (*this)[i]; };

/*!
*	\deprecated use *this.determinant() (or *this.lu().determinant() for large matrices)
*	Calculate and return the matrix determinant (Laplace)
*	\return	the matrix determinant
*/
EIGEN_DEPRECATED Scalar Determinant() const { return determinant(); };

/*!
*	Return the cofactor <I>A<SUB>i,j</SUB></I> of the <I>a<SUB>i,j</SUB></I> element
*	\return	...
*/
EIGEN_DEPRECATED Scalar Cofactor(unsigned int i, unsigned int j) const
{
	assert(rows() == cols());
	assert(rows()>2);
	return (((i+j)%2==0) ? 1. : -1.) * minor(i,j).determinant();
};

/*! \deprecated use *this.col(j) */
EIGEN_DEPRECATED ColXpr GetColumn(const unsigned int j) { return col(j); };

/*! \deprecated use *this.row(i) */
EIGEN_DEPRECATED RowXpr GetRow(const unsigned int i) { return row(i); };

/*! \deprecated use m1.col(i).swap(m1.col(j)); */
EIGEN_DEPRECATED void SwapColumns(const unsigned int i, const unsigned int j)
{
	if (i==j) return;
	col(i).swap(col(j));
};

/*! \deprecated use m1.col(i).swap(m1.col(j)) */
EIGEN_DEPRECATED void SwapRows(const unsigned int i, const unsigned int j)
{
	if (i==j) return;
	row(i).swap(row(j));
};

/*!
*	\deprecated use *this.cwise() += k
*	(Modifier) Add to each element of this matrix the scalar constant <I>k</I>.
* \param k	the scalar constant
*	\return		the modified matrix
*/
EIGEN_DEPRECATED Derived& operator+=(const Scalar k)
{
	cwise() += k;
	return *this;
};

/*!
*	\deprecated use *this.cwise() -= k
*	(Modifier) Subtract from each element of this matrix the scalar constant <I>k</I>.
* \param k	the scalar constant
*	\return		the modified matrix
*/
EIGEN_DEPRECATED Derived& operator-=(const Scalar k)
{
	cwise() -= k;
	return *this;
};

/*!
*	\deprecated use *this.dot
*	Matrix multiplication: calculates the cross product.
*	\param	reference to the matrix to multiply by
*	\return the matrix product
*/
// template <int N,int M>
// EIGEN_DEPRECATED void DotProduct(Point<N,Scalar> &m,Point<M,Scalar> &result)
// {
// 	unsigned int i, j;
// 	for (i=0; i<M; i++)
// 	{ result[i]=0;
// 		for (j=0; j<N; j++)
// 			result[i]+=(*this)[i][j]*m[j];
// 	}
// };


/*! \deprecated use *this = a * b.transpose() (or *this = a * b.adjoint() for complexes) */
template <typename OtherDerived1, typename OtherDerived2>
EIGEN_DEPRECATED void OuterProduct(const MatrixBase<OtherDerived1>& a, const MatrixBase<OtherDerived2>& b)
{ *this = a * b.adjoint(); }

typedef CwiseUnaryOp<ei_scalar_add_op<Scalar>, Derived> ScalarAddReturnType;

/*! \deprecated use *this.cwise() + k */
EIGEN_DEPRECATED const ScalarAddReturnType operator+(const Scalar k) { return cwise() + k; }

/*! \deprecated use *this.cwise() - k */
EIGEN_DEPRECATED const ScalarAddReturnType operator-(const Scalar k) { return cwise() - k; }

/*! \deprecated use *this.setZero() or *this = MatrixType::Zero(rows,cols), etc. */
EIGEN_DEPRECATED void SetZero() { setZero(); };

/*! \deprecated use *this.setIdentity() or *this = MatrixType::Identity(rows,cols), etc. */
EIGEN_DEPRECATED void SetIdentity() { setIdentity(); };

/*! \deprecated use *this.col(j) = expression */
EIGEN_DEPRECATED void SetColumn(unsigned int j, Scalar* v)
{ col(j) = Map<Matrix<Scalar,RowsAtCompileTime,1> >(v,cols(),1); };

/** \deprecated use *this.col(i) = other */
template<typename OtherDerived>
EIGEN_DEPRECATED void SetColumn(unsigned int j, const MatrixBase<OtherDerived>& other)
{ col(j) = other; };

/*! \deprecated use *this.row(i) = expression */
EIGEN_DEPRECATED void SetRow(unsigned int i, Scalar* v)
{ row(i) = Map<Matrix<Scalar,1,ColsAtCompileTime> >(v,1,rows()); };

/** \deprecated use *this.row(i) = other */
template<typename OtherDerived>
EIGEN_DEPRECATED void SetRow(unsigned int j, const MatrixBase<OtherDerived>& other)
{ row(j) = other; };

/*! \deprecated use *this.diagonal() = expression */
EIGEN_DEPRECATED void SetDiagonal(Scalar *v)
{
	assert(rows() == cols());
	diagonal() = Map<Matrix<Scalar,RowsAtCompileTime,1> >(v,cols(),1);
}

/** \deprecated use trace() */
EIGEN_DEPRECATED Scalar Trace() const { return trace(); }

/*! \deprecated use ostream << *this or even ostream << *this.withFormat(...) */
EIGEN_DEPRECATED void Dump()
{
	unsigned int i, j;
	for (i=0; i<rows(); ++i)
	{
		printf("[\t");
		for (j=0; j<cols(); j++)
			printf("%f\t", coeff(i,j));
		printf("]\n");
	}
	printf("\n");
}

/** \deprecated use norm() */
EIGEN_DEPRECATED inline Scalar Norm() const { return norm(); };
/** \deprecated use squaredNorm() */
EIGEN_DEPRECATED inline Scalar SquaredNorm() const { return squaredNorm(); };
/** \deprecated use normalize() or normalized() */
EIGEN_DEPRECATED inline Derived& Normalize() { normalize(); return derived(); };
/** \deprecated use normalized() */
EIGEN_DEPRECATED inline const PlainMatrixType Normalize() const { return normalized(); };

/** \deprecated use transposeInPlace() or transpose() */
EIGEN_DEPRECATED inline Derived& Transpose() { transposeInPlace(); return derived(); };
/** \deprecated use transpose() */
EIGEN_DEPRECATED inline const Eigen::Transpose<Derived> Transpose() const { return transpose(); };

/** \deprecated use .cross(p) */
EIGEN_DEPRECATED inline PlainMatrixType operator ^ (const Derived& p ) const { return this->cross(p); }

/// Homogeneous normalization (division by W)
inline Derived& HomoNormalize()
{
	EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
	enum {
	 SubRows = (int(Flags)&RowMajorBit) ? 1 : (RowsAtCompileTime==Dynamic ? Dynamic : RowsAtCompileTime-1),
	 SubCols = (int(Flags)&RowMajorBit) ? (ColsAtCompileTime==Dynamic ? Dynamic : ColsAtCompileTime-1) : 1,
	};
	Scalar& last = coeffRef(size()-2);
	if (last!=Scalar(0))
	{
		Block<Derived,SubRows,SubCols>(derived(),0,0,
			(int(Flags)&RowMajorBit) ? size()-1 : 1,
			(int(Flags)&RowMajorBit) ? 1 : (size()-1)) / last;
		last = Scalar(1.0);
	}
	return *this;
}

inline const PlainMatrixType HomoNormalize() const
{
	PlainMatrixType res = derived();
	return res.HomoNormalize();
}

/// norm infinity: largest absolute value of compoenet
EIGEN_DEPRECATED inline Scalar NormInfinity() const { return derived().cwise().abs().maxCoeff(); }
/// norm 1: sum of absolute values of components
EIGEN_DEPRECATED inline Scalar NormOne() const { return derived().cwise().abs().sum(); }


/// the sum of the components
EIGEN_DEPRECATED inline Scalar Sum() const { return sum(); }
/// returns the biggest component
EIGEN_DEPRECATED inline Scalar Max() const { return maxCoeff(); }
/// returns the smallest component
EIGEN_DEPRECATED inline Scalar Min() const { return minCoeff(); }
/// returns the index of the biggest component
EIGEN_DEPRECATED inline int MaxI() const { EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived); int i; maxCoeff(&i,0); return i; }
/// returns the index of the smallest component
EIGEN_DEPRECATED inline int MinI() const { EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived); int i; minCoeff(&i,0); return i; }


/// Padding function: give a default 0 value to all the elements that are not in the [0..2] range.
/// Useful for managing in a consistent way object that could have point2 / point3 / point4
inline Scalar Ext( const int i ) const
{
	EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
	EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);

	if(i>=0 && i<SizeAtCompileTime)
		return coeff(i);
	else
		return Scalar(0);
}


/// Per component scaling
template<typename OtherDerived>
EIGEN_DEPRECATED inline Derived& Scale(const MatrixBase<OtherDerived>& other)
{ this->cwise() *= other; return derived; }

template<typename OtherDerived>
EIGEN_DEPRECATED inline
CwiseBinaryOp<ei_scalar_product_op<Scalar>, Derived, OtherDerived>
Scale(const MatrixBase<OtherDerived>& other) const
{ return this->cwise() * other; }


template<typename OtherDerived>
EIGEN_DEPRECATED inline bool operator <  (const MatrixBase<OtherDerived>& other) const {
	EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
	EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(Derived,OtherDerived);
	return ei_lexi_comparison<Derived,OtherDerived,SizeAtCompileTime>::less(derived(),other.derived());
}

template<typename OtherDerived>
EIGEN_DEPRECATED inline bool operator >  (const MatrixBase<OtherDerived>& other) const {
	EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
	EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(Derived,OtherDerived);
	return ei_lexi_comparison<Derived,OtherDerived,SizeAtCompileTime>::geater(derived(),other.derived());
}

template<typename OtherDerived>
EIGEN_DEPRECATED inline bool operator <=  (const MatrixBase<OtherDerived>& other) const {
	EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
	EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(Derived,OtherDerived);
	return ei_lexi_comparison<Derived,OtherDerived,SizeAtCompileTime>::lessEqual(derived(),other.derived());
}

template<typename OtherDerived>
EIGEN_DEPRECATED inline bool operator >=  (const MatrixBase<OtherDerived>& other) const {
	EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
	EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(Derived,OtherDerived);
	return ei_lexi_comparison<Derived,OtherDerived,SizeAtCompileTime>::greaterEqual(derived(),other.derived());
}
