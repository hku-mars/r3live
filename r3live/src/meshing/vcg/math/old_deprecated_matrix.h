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
/***************************************************************************
$Log: not supported by cvs2svn $
Revision 1.9  2006/09/11 16:11:39  marfr960
Added const to declarations of the overloaded (operators *).
Otherwise the  * operator would always attempt to convert any type of data passed as an argument to Point3<TYPE>

Revision 1.8  2006/08/23 15:24:45  marfr960
Copy constructor : faster memcpy instead of slow 'for' cycle
empty constructor

Revision 1.7  2006/04/29 10:26:04  fiorin
Added some utility methods (swapping of columns and rows, matrix-vector multiplication)

Revision 1.6  2006/04/11 08:09:35  zifnab1974
changes necessary for gcc 3.4.5 on linux 64bit. Please take note of case-sensitivity of filenames

Revision 1.5  2005/12/12 11:25:00  ganovelli
added diagonal matrix, outer produce and namespace

***************************************************************************/

#ifndef MATRIX_VCGLIB
#define MATRIX_VCGLIB

#include <stdio.h>
#include <math.h>
#include <memory.h>
#include <assert.h>
#include <algorithm>
#include <vcg/space/point.h>

namespace vcg{
	namespace ndim{

	 /** \addtogroup math */
   /* @{ */

	/*!
 * This class represent a diagonal <I>m</I>�<I>m</I> matrix.
 */
	
	class MatrixDiagBase{public: 
	virtual const int & Dimension()const =0;
        virtual float operator[](const int & i)const = 0;
	};
	template<int N, class S> 
	class MatrixDiag: public Point<N,S>, public MatrixDiagBase{
	public:
		const int & Dimension() const {return N;}
		MatrixDiag(const Point<N,S>&p):Point<N,S>(p){}
	};

/*!
 * This class represent a generic <I>m</I>�<I>n</I> matrix. The class is templated over the scalar type field.
 * @param TYPE (Templete Parameter) Specifies the ScalarType field.
 */
	template<class TYPE> 
	class Matrix
		{
			
		public:
			typedef TYPE ScalarType;

			/*!
			*	Default constructor
			* All the elements are initialized to zero.
			*	\param m the number of matrix rows
			* \param n the number of matrix columns
			*/
			Matrix(unsigned int m, unsigned int n)
			{
				_rows = m;
				_columns = n;
				_data = new ScalarType[m*n];
				memset(_data, 0, m*n*sizeof(ScalarType));
			};

			/*!
			*	Constructor
			* The matrix elements are initialized with the values of the elements in \i values.
			*	\param m the number of matrix rows
			* \param n the number of matrix columns
			*	\param values the values of the matrix elements
			*/
			Matrix(unsigned int m, unsigned int n, TYPE *values)
			{
				_rows = m;
				_columns = n;
				unsigned int dim = m*n;
				_data = new ScalarType[dim];
				memcpy(_data, values, dim*sizeof(ScalarType));
				//unsigned int i;
				//for (i=0; i<_rows*_columns; i++)
				//	_data[i] = values[i];
			};

			/*!
			*	Empty constructor
			*   Just create the object
			*/
			Matrix()
			{
				_rows = 0;
				_columns = 0;
				_data = NULL;
			};

			/*!
			*	Copy constructor
			*	The matrix elements are initialized with the value of the corresponding element in \i m
			* \param m the matrix to be copied
			*/
			Matrix(const Matrix<TYPE> &m)
			{
				_rows = m._rows;
				_columns = m._columns;
				_data = new ScalarType[_rows*_columns];

				unsigned int dim = _rows * _columns;
				memcpy(_data, m._data, dim * sizeof(ScalarType));

//				for (unsigned int i=0; i<_rows*_columns; i++)
//					_data[i] = m._data[i];
			};

			/*!
			*	Default destructor
			*/
			~Matrix()
			{
				delete []_data;
			};

			/*!
			*	Number of columns
			*/
			inline unsigned int ColumnsNumber() const
			{
				return _columns;
			};


			/*!
			*	Number of rows
			*/
			inline unsigned int RowsNumber() const 
			{
				return _rows;
			};

			/*!
			*	Equality operator.
			*	\param m
			*	\return true iff the matrices have same size and its elements have same values.
			*/
			bool operator==(const Matrix<TYPE> &m) const
			{
				if (_rows==m._rows && _columns==m._columns)
				{
					bool result = true;
					for (unsigned int i=0; i<_rows*_columns && result; i++)
						result = (_data[i]==m._data[i]);
					return result;
				}
				return false;
			};

			/*!
			*	Inequality operator
			*	\param m
			*	\return true iff the matrices have different size or if their elements have different values.
			*/
			bool operator!=(const Matrix<TYPE> &m) const
			{
				if (_rows==m._rows && _columns==m._columns)
				{
					bool result = false;
					for (unsigned int i=0; i<_rows*_columns && !result; i++)
						result = (_data[i]!=m._data[i]);
					return result;
				}
				return true;
			};

			/*!
			* Return the element stored in the <I>i</I>-th rows at the <I>j</I>-th column
			*	\param i the row index
			*	\param j the column index
			*	\return the element 
			*/
			inline TYPE ElementAt(unsigned int i, unsigned int j)
			{
				assert(i>=0 && i<_rows);
				assert(j>=0 && j<_columns);
				return _data[i*_columns+j];
			};

			/*!
			*	Calculate and return the matrix determinant (Laplace)
			*	\return	the matrix determinant
			*/
			TYPE Determinant() const
			{
				assert(_rows == _columns);
				switch (_rows)
				{
				case 2:
					{
						return _data[0]*_data[3]-_data[1]*_data[2];
						break;
					};
				case 3:
					{
						return	_data[0]*(_data[4]*_data[8]-_data[5]*_data[7]) - 
										_data[1]*(_data[3]*_data[8]-_data[5]*_data[6]) + 
										_data[2]*(_data[3]*_data[7]-_data[4]*_data[6]) ;
						break;
					};
				default:
					{
						// da migliorare: si puo' cercare la riga/colonna con maggior numero di zeri
						ScalarType det = 0;
						for (unsigned int j=0; j<_columns; j++)
							if (_data[j]!=0)
								det += _data[j]*this->Cofactor(0, j);

						return det;
					}
				};
			};

			/*!
			*	Return the cofactor <I>A<SUB>i,j</SUB></I> of the <I>a<SUB>i,j</SUB></I> element
			*	\return	...
			*/
			TYPE Cofactor(unsigned int i, unsigned int j) const
			{
				assert(_rows == _columns);
				assert(_rows>2);
				TYPE *values = new TYPE[(_rows-1)*(_columns-1)];
				unsigned int u, v, p, q, s, t;
				for (u=0, p=0, s=0, t=0; u<_rows; u++, t+=_rows)
				{
					if (i==u)
						continue;

					for (v=0, q=0; v<_columns; v++)
					{
						if (j==v)
							continue;
						values[s+q] = _data[t+v];
						q++;
					}
					p++;
					s+=(_rows-1);
				}
				Matrix<TYPE> temp(_rows-1, _columns-1, values);
        return (pow(TYPE(-1.0), TYPE(i+j))*temp.Determinant());
			};

			/*!
			*	Subscript operator: 
			* \param i	the index of the row
			*	\return a reference to the <I>i</I>-th matrix row
			*/
			inline TYPE* operator[](const unsigned int i)
			{
        assert(i<_rows);
				return _data + i*_columns;
			};

			/*!
			*	Const subscript operator
			* \param i	the index of the row
			*	\return a reference to the <I>i</I>-th matrix row
			*/
			inline const TYPE* operator[](const unsigned int i) const 
			{
        assert(i<_rows);
				return _data + i*_columns;
			};

						/*!
			*	Get the <I>j</I>-th column on the matrix.
			*	\param j	the column index.
			*	\return		the reference to the column elements. This pointer must be deallocated by the caller.
			*/
			TYPE* GetColumn(const unsigned int j)
			{
				assert(j>=0 && j<_columns);
				ScalarType *v = new ScalarType[_columns];
				unsigned int i, p;
				for (i=0, p=j; i<_rows; i++, p+=_columns)
					v[i] = _data[p];
				return v;
			};

			/*!
			*	Get the <I>i</I>-th row on the matrix.
			*	\param i	the column index.
			*	\return		the reference to the row elements. This pointer must be deallocated by the caller.
			*/
			TYPE* GetRow(const unsigned int i)
			{
				assert(i>=0 && i<_rows);
				ScalarType *v = new ScalarType[_rows];
				unsigned int j, p;
				for (j=0, p=i*_columns; j<_columns; j++, p++)
					v[j] = _data[p];
				return v;
			};

			/*!
			* Swaps the values of the elements between the <I>i</I>-th and the <I>j</I>-th column.
			* \param i the index of the first column
			* \param j the index of the second column
			*/
			void SwapColumns(const unsigned int i, const unsigned int j)
			{
				assert(0<=i && i<_columns);
				assert(0<=j && j<_columns);
				if (i==j)
					return;

				unsigned int r, e0, e1;
				for (r=0, e0=i, e1=j; r<_rows; r++, e0+=_columns, e1+=_columns)
					std::swap(_data[e0], _data[e1]);
			};

			/*!
			* Swaps the values of the elements between the <I>i</I>-th and the <I>j</I>-th row.
			* \param i the index of the first row
			* \param j the index of the second row
			*/
			void SwapRows(const unsigned int i, const unsigned int j)
			{
				assert(0<=i && i<_rows);
				assert(0<=j && j<_rows);
				if (i==j)
					return;

				unsigned int r, e0, e1;
				for (r=0, e0=i*_columns, e1=j*_columns; r<_columns; r++, e0++, e1++)
					std::swap(_data[e0], _data[e1]);
			};

			/*!
			*	Assignment operator
			*	\param m ...
			*/
			Matrix<TYPE>& operator=(const Matrix<TYPE> &m)
			{
				if (this != &m)
				{
					assert(_rows == m._rows);
					assert(_columns == m._columns);
					for (unsigned int i=0; i<_rows*_columns; i++)
						_data[i] = m._data[i];
				}
				return *this;
			};

			/*!
			*	Adds a matrix <I>m</I> to this matrix.
			*	\param m  reference to matrix to add to this  
			*	\return		the matrix sum. 
			*/
			Matrix<TYPE>& operator+=(const Matrix<TYPE> &m)
			{
				assert(_rows == m._rows);
				assert(_columns == m._columns);
				for (unsigned int i=0; i<_rows*_columns; i++)
					_data[i] += m._data[i];
				return *this;
			};

			/*!
			*	Subtracts a matrix <I>m</I> to this matrix. 
			*	\param m  reference to matrix to subtract 
			*	\return		the matrix difference. 
			*/
			Matrix<TYPE>& operator-=(const Matrix<TYPE> &m)
			{
				assert(_rows == m._rows);
				assert(_columns == m._columns);
				for (unsigned int i=0; i<_rows*_columns; i++)
					_data[i] -= m._data[i];
				return *this;
			};

				/*!
			*	(Modifier) Add to each element of this matrix the scalar constant <I>k</I>. 
			* \param k	the scalar constant
			*	\return		the modified matrix
			*/
			Matrix<TYPE>& operator+=(const TYPE k)
			{
				for (unsigned int i=0; i<_rows*_columns; i++)
					_data[i] += k;
				return *this;
			};

			/*!
			*	(Modifier) Subtract from each element of this matrix the scalar constant <I>k</I>. 
			* \param k	the scalar constant
			*	\return		the modified matrix
			*/
			Matrix<TYPE>& operator-=(const TYPE k)
			{
				for (unsigned int i=0; i<_rows*_columns; i++)
					_data[i] -= k;
				return *this;
			};

			/*!
			*	(Modifier) Multiplies each element of this matrix by the scalar constant <I>k</I>. 
			* \param k	the scalar constant
			*	\return		the modified matrix
			*/
			Matrix<TYPE>& operator*=(const TYPE k)
			{
				for (unsigned int i=0; i<_rows*_columns; i++)
					_data[i] *= k;
				return *this;
			};

			/*!
			*	(Modifier) Divides each element of this matrix by the scalar constant <I>k</I>. 
			* \param k	the scalar constant
			*	\return		the modified matrix
			*/
			Matrix<TYPE>& operator/=(const TYPE k)
			{
				assert(k!=0);
				for (unsigned int i=0; i<_rows*_columns; i++)
					_data[i] /= k;
				return *this;
			};

			/*!
			*	Matrix multiplication: calculates the cross product.
			*	\param	m reference to the matrix to multiply by 
			*	\return the matrix product
			*/
			Matrix<TYPE> operator*(const Matrix<TYPE> &m) const
			{
				assert(_columns == m._rows);
				Matrix<TYPE> result(_rows, m._columns);
				unsigned int i, j, k, p, q, r;
				for (i=0, p=0, r=0; i<result._rows; i++, p+=_columns, r+=result._columns)
					for (j=0; j<result._columns; j++)
					{
						ScalarType temp = 0;
						for (k=0, q=0; k<_columns; k++, q+=m._columns)
							temp+=(_data[p+k]*m._data[q+j]);
						result._data[r+j] = temp;
					}
				
				return result;
			};

						/*!
			* Matrix-Vector product. Computes the product of the matrix by the vector v.
			* \param  v reference to the vector to multiply by
			* \return   the matrix-vector product. This pointer must be deallocated by the caller
			*/
			ScalarType* operator*(const ScalarType v[]) const
			{
				ScalarType *result = new ScalarType[_rows];
				memset(result, 0, _rows*sizeof(ScalarType));
				unsigned int r, c, i;
				for (r=0, i=0; r<_rows; r++)
					for (c=0; c<_columns; c++, i++)
						result[r] += _data[i]*v[c];

				return result;
			};

			/*!
			*	Matrix multiplication: calculates the cross product.
			*	\param	reference to the matrix to multiply by 
			*	\return the matrix product
			*/
			template <int N,int M>
			void DotProduct(Point<N,TYPE> &m,Point<M,TYPE> &result)
			{
				unsigned int i, j,  p,  r;
				for (i=0, p=0, r=0; i<M; i++)
				{ result[i]=0;
					for (j=0; j<N; j++)
						result[i]+=(*this)[i][j]*m[j];
				}
			};

			/*!
			*	Matrix multiplication by a diagonal matrix
			*/
			Matrix<TYPE> operator*(const MatrixDiagBase &m) const
			{
				assert(_columns == _rows);
				assert(_columns == m.Dimension());
				int i,j;
				Matrix<TYPE> result(_rows, _columns);

				for (i=0; i<result._rows; i++)
					for (j=0; j<result._columns; j++)
						result[i][j]*= m[j];
				
				return result;
			};
			/*!
			*	Matrix from outer product.
			*/
			template <int N, int M>
			void OuterProduct(const Point<N,TYPE> a, const Point< M,TYPE> b)
			{
				assert(N == _rows);
				assert(M == _columns);
				Matrix<TYPE> result(_rows,_columns);
				unsigned int i, j;

				for (i=0; i<result._rows; i++)
					for (j=0; j<result._columns; j++)
						(*this)[i][j] = a[i] * b[j];
			};


			/*!
			*	Matrix-vector multiplication.
			*	\param	reference to the 3-dimensional vector to multiply by
			*	\return the resulting vector
			*/

			Point3<TYPE> operator*(Point3<TYPE> &p) const
			{
				assert(_columns==3 && _rows==3);
				vcg::Point3<TYPE> result;
				result[0] = _data[0]*p[0]+_data[1]*p[1]+_data[2]*p[2];
				result[1] = _data[3]*p[0]+_data[4]*p[1]+_data[5]*p[2];
				result[2] = _data[6]*p[0]+_data[7]*p[1]+_data[8]*p[2];
				return result;
			};


			/*!
			*	Scalar sum.  
			*	\param k	
			*	\return		the resultant matrix 
			*/
			Matrix<TYPE> operator+(const TYPE k)
			{
				Matrix<TYPE> result(_rows, _columns);
				for (unsigned int i=0; i<_rows*_columns; i++)
					result._data[i] =  _data[i]+k;
				return result;
			};

			/*!
			*	Scalar difference.  
			*	\param k	
			*	\return		the resultant matrix 
			*/
			Matrix<TYPE> operator-(const TYPE k)
			{
				Matrix<TYPE> result(_rows, _columns);
				for (unsigned int i=0; i<_rows*_columns; i++)
					result._data[i] =  _data[i]-k;
				return result;
			};

			/*!
			*	Negate all matrix elements
			*	\return	the modified matrix
			*/
			Matrix<TYPE> operator-() const
			{
				Matrix<TYPE> result(_rows, _columns, _data);
				for (unsigned int i=0; i<_columns*_rows; i++)
					result._data[i] = -1*_data[i];
				return result;
			};

			/*!
			*	Scalar multiplication.  
			*	\param k	value to multiply every member by 
			*	\return		the resultant matrix 
			*/
			Matrix<TYPE> operator*(const TYPE k) const
			{
				Matrix<TYPE> result(_rows, _columns);
				for (unsigned int i=0; i<_rows*_columns; i++)
					result._data[i] =  _data[i]*k;
				return result;
			};

			/*!
			*	Scalar division.  
			*	\param k	value to divide every member by 
			*	\return		the resultant matrix 
			*/
			Matrix<TYPE> operator/(const TYPE k)
			{
				Matrix<TYPE> result(_rows, _columns);
				for (unsigned int i=0; i<_rows*_columns; i++)
					result._data[i] =  _data[i]/k;
				return result;
			};


			/*!
			*	Set all the matrix elements to zero.
			*/
			void SetZero()
			{
				for (unsigned int i=0; i<_rows*_columns; i++)
					_data[i] = ScalarType(0.0);
			};

			/*!
			*	Set the matrix to identity.
			*/
			void SetIdentity()
			{
				assert(_rows==_columns);
				for (unsigned int i=0; i<_rows; i++)
					for (unsigned int j=0; j<_columns; j++)
						_data[i] = (i==j) ? ScalarType(1.0) : ScalarType(0.0f);
			};

			/*!
			*	Set the values of <I>j</I>-th column to v[j]
			*	\param j	the column index
			*	\param v	...
			*/
			void SetColumn(const unsigned int j, TYPE* v)
			{
				assert(j>=0 && j<_columns);
				unsigned int i, p;
				for (i=0, p=j; i<_rows; i++, p+=_columns)
					_data[p] = v[i];
			};

			/*!
			*	Set the elements of the <I>i</I>-th row to v[j] 
			*	\param i	the row index
			*	\param v	...
			*/
			void SetRow(const unsigned int i, TYPE* v)
			{
				assert(i>=0 && i<_rows);
				unsigned int j, p;
				for (j=0, p=i*_rows; j<_columns; j++, p++)
					_data[p] = v[j];
			};

			/*!
			*	Set the diagonal elements <I>v<SUB>i,i</SUB></I> to v[i]
			*	\param v
			*/
			void SetDiagonal(TYPE *v)
			{
				assert(_rows == _columns);
				for (unsigned int i=0, p=0; i<_rows; i++, p+=_rows)
					_data[p+i] = v[i];
			};

			/*!
			*	Resize the current matrix.
			*	\param m the number of matrix rows.
			* \param n the number of matrix columns.
			*/
			void Resize(const unsigned int m, const unsigned int n)
			{
				assert(m>=2);
				assert(n>=2);
				_rows = m;
				_columns = n;
				delete []_data;
				_data = new ScalarType[m*n];
				for (unsigned int i=0; i<m*n; i++)
					_data[i] = 0;
			};


			/*!
			*	Matrix transposition operation: set the current matrix to its transpose
			*/
			void Transpose()
			{
				ScalarType *temp = new ScalarType[_rows*_columns];
				unsigned int i, j, p, q;
				for (i=0, p=0; i<_rows; i++, p+=_columns)
					for (j=0, q=0; j<_columns; j++, q+=_rows)
						temp[q+i] = _data[p+j];
				
				std::swap(_columns, _rows);
				std::swap(_data, temp);
				delete []temp;
			};

			// for the transistion to eigen
			Matrix transpose()
			{
				Matrix res = *this;
				res.Transpose();
				return res;
			}
			void transposeInPlace() { Transpose(); }
			// for the transistion to eigen

			/*!
			*	Print all matrix elements
			*/
			void Dump()
			{
				unsigned int i, j, p;
				for (i=0, p=0; i<_rows; i++, p+=_columns)
				{
					printf("[\t");
					for (j=0; j<_columns; j++)
						printf("%f\t", _data[p+j]);
					
					printf("]\n");
				}
				printf("\n");
			};

		protected:
			///	the number of matrix rows
			unsigned int _rows;

			///	the number of matrix rows
			unsigned int _columns;

			/// the matrix elements 
			ScalarType *_data;
		};

		typedef vcg::ndim::Matrix<double> MatrixMNd;
		typedef vcg::ndim::Matrix<float>  MatrixMNf;

  /*! @} */

//	template <class MatrixType>
//	void Invert(MatrixType & m){
//		typedef typename MatrixType::ScalarType X;
//		X  *diag;
//		diag = new  X [m.ColumnsNumber()];

//		MatrixType res(m.RowsNumber(),m.ColumnsNumber());
//		vcg::SingularValueDecomposition<MatrixType > (m,&diag[0],res,LeaveUnsorted,50 );
//		m.Transpose();
//		// prodotto per la diagonale
//		unsigned  int i,j;
//		for (i=0; i<m.RowsNumber(); i++)
//					for (j=0; j<m.ColumnsNumber(); j++)
//						res[i][j]/= diag[j];
//		m = res *m;
//		}

	}
}; // end of namespace

#endif
