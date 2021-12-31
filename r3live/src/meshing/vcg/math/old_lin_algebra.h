/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2006                                                \/)\/    *
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
Revision 1.16  2007/01/29 00:18:20  pietroni
-added some explicit CASTs in order to avoid warning if one use float instead of double as ScalarType

Revision 1.15  2006/09/29 08:36:10  cignoni
Added missing typedef for gcc compiing

Revision 1.14  2006/09/28 22:49:49  fiorin
Removed some warnings

Revision 1.13  2006/07/28 12:39:05  zifnab1974
added some typename directives

Revision 1.12  2006/07/24 07:26:47  fiorin
Changed the template argument in JacobiRotate and added method for sorting eigenvalues and eigenvectors (SortEigenvaluesAndEigenvectors)

Revision 1.11  2006/05/25 09:35:55  cignoni
added missing internal prototype to Sort function

Revision 1.10  2006/05/17 09:26:35  cignoni
Added initial disclaimer

****************************************************************************/
#ifndef __VCGLIB_LINALGEBRA_H
#define __VCGLIB_LINALGEBRA_H

#include <vcg/math/base.h>
#include <vcg/math/matrix44.h>
#include <algorithm>
#ifndef _YES_I_WANT_TO_USE_DANGEROUS_STUFF
#error "Please do not never user this file. Use EIGEN!!!!"
#endif
namespace vcg
{
	/** \addtogroup math */
	/* @{ */

	/*!
	*
	*/
	template< typename MATRIX_TYPE >
	static void JacobiRotate(MATRIX_TYPE &A, typename MATRIX_TYPE::ScalarType s, typename MATRIX_TYPE::ScalarType tau, int i,int j,int k,int l)
	{
		typename MATRIX_TYPE::ScalarType g=A[i][j];
		typename MATRIX_TYPE::ScalarType h=A[k][l];
		A[i][j]=g-s*(h+g*tau);
		A[k][l]=h+s*(g-h*tau);
	};

	/*!
	*	Computes all eigenvalues and eigenvectors of a real symmetric matrix .
	*	On output, elements of the input matrix above the diagonal are destroyed.
	* \param d  returns the eigenvalues of a.
	* \param v  is a matrix whose columns contain, the normalized eigenvectors
	* \param nrot returns the number of Jacobi rotations that were required.
	*/
	template <typename MATRIX_TYPE, typename POINT_TYPE>
	static void Jacobi(MATRIX_TYPE &w, POINT_TYPE &d, MATRIX_TYPE &v, int &nrot)
	{
       typedef typename MATRIX_TYPE::ScalarType ScalarType;
		assert(w.RowsNumber()==w.ColumnsNumber());
		int dimension = w.RowsNumber();

		int j,iq,ip,i;
		//assert(w.IsSymmetric());
		typename MATRIX_TYPE::ScalarType tresh, theta, tau, t, sm, s, h, g, c;
		POINT_TYPE b, z;

		v.SetIdentity();

		for (ip=0;ip<dimension;++ip)			//Initialize b and d to the diagonal of a.
		{
			b[ip]=d[ip]=w[ip][ip];
			z[ip]=ScalarType(0.0);							//This vector will accumulate terms of the form tapq as in equation (11.1.14).
		}
		nrot=0;
		for (i=0;i<50;i++)
		{
			sm=ScalarType(0.0);
			for (ip=0;ip<dimension-1;++ip)		// Sum off diagonal elements
			{
				for (iq=ip+1;iq<dimension;++iq)
					sm += math::Abs(w[ip][iq]);
			}
			if (sm == ScalarType(0.0))					//The normal return, which relies on quadratic convergence to machine underflow.
			{
				return;
			}
			if (i < 4)
				tresh=ScalarType(0.2)*sm/(dimension*dimension); //...on the first three sweeps.
			else
				tresh=ScalarType(0.0);				//...thereafter.
			for (ip=0;ip<dimension-1;++ip)
			{
				for (iq=ip+1;iq<dimension;iq++)
				{
					g=ScalarType(100.0)*fabs(w[ip][iq]);
					//After four sweeps, skip the rotation if the off-diagonal element is small.
					if(i>4 && (float)(fabs(d[ip])+g) == (float)fabs(d[ip]) && (float)(fabs(d[iq])+g) == (float)fabs(d[iq]))
						w[ip][iq]=ScalarType(0.0);
					else if (fabs(w[ip][iq]) > tresh)
					{
						h=d[iq]-d[ip];
						if ((float)(fabs(h)+g) == (float)fabs(h))
							t=(w[ip][iq])/h; //t =1/(2#)
						else
						{
							theta=ScalarType(0.5)*h/(w[ip][iq]); //Equation (11.1.10).
							t=ScalarType(1.0)/(fabs(theta)+sqrt(ScalarType(1.0)+theta*theta));
							if (theta < ScalarType(0.0)) t = -t;
						}
						c=ScalarType(1.0)/sqrt(ScalarType(1.0)+t*t);
						s=t*c;
						tau=s/(ScalarType(1.0)+c);
						h=t*w[ip][iq];
						z[ip] -= h;
						z[iq] += h;
						d[ip] -= h;
						d[iq] += h;
						w[ip][iq]=ScalarType(0.0);
						for (j=0;j<=ip-1;j++) { //Case of rotations 1 <= j < p.
							JacobiRotate<MATRIX_TYPE>(w,s,tau,j,ip,j,iq) ;
						}
						for (j=ip+1;j<=iq-1;j++) { //Case of rotations p < j < q.
							JacobiRotate<MATRIX_TYPE>(w,s,tau,ip,j,j,iq);
						}
						for (j=iq+1;j<dimension;j++) { //Case of rotations q< j <= n.
							JacobiRotate<MATRIX_TYPE>(w,s,tau,ip,j,iq,j);
						}
						for (j=0;j<dimension;j++) {
							JacobiRotate<MATRIX_TYPE>(v,s,tau,j,ip,j,iq);
						}
						++nrot;
					}
				}
			}
			for (ip=0;ip<dimension;ip++)
			{
				b[ip] += z[ip];
				d[ip]=b[ip]; //Update d with the sum of ta_pq ,
				z[ip]=0.0; //and reinitialize z.
			}
		}
	};


	/*!
	* Given the eigenvectors and the eigenvalues as output from JacobiRotate, sorts the eigenvalues
	* into descending order, and rearranges the columns of v correspondinlgy.
	* \param eigenvalues
	* \param eigenvector (in columns)
	* \param absComparison sort according to the absolute values of the eigenvalues.
	*/
	template < typename MATRIX_TYPE, typename POINT_TYPE >
	void SortEigenvaluesAndEigenvectors(POINT_TYPE &eigenvalues, MATRIX_TYPE &eigenvectors, bool absComparison = false)
	{
		assert(eigenvectors.ColumnsNumber()==eigenvectors.RowsNumber());
		int dimension = eigenvectors.ColumnsNumber();
		int i, j, k;
		float p,q;
		for (i=0; i<dimension-1; i++)
		{
			if (absComparison)
			{
				p = fabs(eigenvalues[k=i]);
				for (j=i+1; j<dimension; j++)
					if ((q=fabs(eigenvalues[j])) >= p)
					{
						p = q;
						k = j;
					}
				p = eigenvalues[k];
			}
			else
			{
				p = eigenvalues[ k=i ];
				for (j=i+1; j<dimension; j++)
					if (eigenvalues[j] >= p)
						p = eigenvalues[ k=j ];
			}

			if (k != i)
			{
				eigenvalues[k] = eigenvalues[i];  // i.e.
				eigenvalues[i] = p;								// swaps the value of the elements i-th and k-th

				for (j=0; j<dimension; j++)
				{
					p = eigenvectors[j][i];										// i.e.
					eigenvectors[j][i] = eigenvectors[j][k];	// swaps the eigenvectors stored in the
					eigenvectors[j][k] = p;										// i-th and the k-th column
				}
			}
		}
	};


  template <typename TYPE>
  inline static TYPE sqr(TYPE a)
  {
    TYPE sqr_arg = a;
    return (sqr_arg == 0 ? 0 : sqr_arg*sqr_arg);
  }

	// Computes (a^2 + b^2)^(1/2) without destructive underflow or overflow.
	template <typename TYPE>
	inline static TYPE pythagora(TYPE a, TYPE b)
	{
		TYPE abs_a = fabs(a);
		TYPE abs_b = fabs(b);
		if (abs_a > abs_b)
      return abs_a*sqrt((TYPE)1.0+sqr(abs_b/abs_a));
		else
			return (abs_b == (TYPE)0.0 ? (TYPE)0.0 : abs_b*sqrt((TYPE)1.0+sqr(abs_a/abs_b)));
  }

	template <typename TYPE>
	inline static TYPE sign(TYPE a, TYPE b)
	{
		return (b >= 0.0 ? fabs(a) : -fabs(a));
  }

	/*!
	*
	*/
	enum SortingStrategy {LeaveUnsorted=0, SortAscending=1, SortDescending=2};
	template< typename MATRIX_TYPE >
	void Sort(MATRIX_TYPE &U, typename MATRIX_TYPE::ScalarType W[], MATRIX_TYPE &V, const SortingStrategy sorting) ;


	/*!
	*	Given a matrix <I>A<SUB>mxn</SUB></I>, this routine computes its singular value decomposition,
	*	i.e. <I>A=UxWxV<SUP>T</SUP></I>. The matrix <I>A</I> will be destroyed!
	*	(This is the implementation described in <I>Numerical Recipies</I>).
	*	\param A	the matrix to be decomposed
	*	\param W	the diagonal matrix of singular values <I>W</I>, stored as a vector <I>W[1...N]</I>
	*	\param V	the matrix <I>V</I> (not the transpose <I>V<SUP>T</SUP></I>)
	*	\param max_iters	max iteration number (default = 30).
	*	\return
	*/
	template <typename MATRIX_TYPE>
		static bool SingularValueDecomposition(MATRIX_TYPE &A, typename MATRIX_TYPE::ScalarType *W, MATRIX_TYPE &V, const SortingStrategy sorting=LeaveUnsorted, const int max_iters=30)
	{
		typedef typename MATRIX_TYPE::ScalarType ScalarType;
		int m = (int) A.RowsNumber();
		int n = (int) A.ColumnsNumber();
		int flag,i,its,j,jj,k,l,nm;
		ScalarType anorm, c, f, g, h, s, scale, x, y, z, *rv1;
		bool convergence = true;

		rv1 = new ScalarType[n];
		g = scale = anorm = 0;
		// Householder reduction to bidiagonal form.
		for (i=0; i<n; i++)
		{
			l = i+1;
			rv1[i] = scale*g;
			g = s = scale = 0.0;
			if (i < m)
			{
				for (k = i; k<m; k++)
					scale += fabs(A[k][i]);
				if (scale)
				{
					for (k=i; k<m; k++)
					{
						A[k][i] /= scale;
						s += A[k][i]*A[k][i];
					}
					f=A[i][i];
					g = -sign<ScalarType>( sqrt(s), f );
					h = f*g - s;
					A[i][i]=f-g;
					for (j=l; j<n; j++)
					{
						for (s=0.0, k=i; k<m; k++)
							s += A[k][i]*A[k][j];
						f = s/h;
						for (k=i; k<m; k++)
							A[k][j] += f*A[k][i];
					}
					for (k=i; k<m; k++)
						A[k][i] *= scale;
				}
			}
			W[i] = scale *g;
			g = s = scale = 0.0;
			if (i < m && i != (n-1))
			{
				for (k=l; k<n; k++)
					scale += fabs(A[i][k]);
				if (scale)
				{
					for (k=l; k<n; k++)
					{
						A[i][k] /= scale;
						s += A[i][k]*A[i][k];
					}
					f = A[i][l];
					g = -sign<ScalarType>(sqrt(s),f);
					h = f*g - s;
					A[i][l] = f-g;
					for (k=l; k<n; k++)
						rv1[k] = A[i][k]/h;
					for (j=l; j<m; j++)
					{
						for (s=0.0, k=l; k<n; k++)
							s += A[j][k]*A[i][k];
						for (k=l; k<n; k++)
							A[j][k] += s*rv1[k];
					}
					for (k=l; k<n; k++)
						A[i][k] *= scale;
				}
			}
      anorm=std::max( anorm, (math::Abs(W[i])+math::Abs(rv1[i])) );
		}
		// Accumulation of right-hand transformations.
		for (i=(n-1); i>=0; i--)
		{
			//Accumulation of right-hand transformations.
			if (i < (n-1))
			{
				if (g)
				{
					for (j=l; j<n;j++) //Double division to avoid possible underflow.
						V[j][i]=(A[i][j]/A[i][l])/g;
					for (j=l; j<n; j++)
					{
						for (s=0.0, k=l; k<n; k++)
							s += A[i][k] * V[k][j];
						for (k=l; k<n; k++)
							V[k][j] += s*V[k][i];
					}
				}
				for (j=l; j<n; j++)
					V[i][j] = V[j][i] = 0.0;
			}
			V[i][i] = 1.0;
			g = rv1[i];
			l = i;
		}
		// Accumulation of left-hand transformations.
    for (i=std::min(m,n)-1; i>=0; i--)
		{
			l = i+1;
			g = W[i];
			for (j=l; j<n; j++)
				A[i][j]=0.0;
			if (g)
			{
				g = (ScalarType)1.0/g;
				for (j=l; j<n; j++)
				{
					for (s=0.0, k=l; k<m; k++)
						s += A[k][i]*A[k][j];
					f = (s/A[i][i])*g;
					for (k=i; k<m; k++)
						A[k][j] += f*A[k][i];
				}
				for (j=i; j<m; j++)
					A[j][i] *= g;
			}
			else
				for (j=i; j<m; j++)
					A[j][i] = 0.0;
			++A[i][i];
		}
		// Diagonalization of the bidiagonal form: Loop over
		// singular values, and over allowed iterations.
		for (k=(n-1); k>=0; k--)
		{
			for (its=1; its<=max_iters; its++)
			{
				flag=1;
				for (l=k; l>=0; l--)
				{
					// Test for splitting.
					nm=l-1;
					// Note that rv1[1] is always zero.
					if ((double)(fabs(rv1[l])+anorm) == anorm)
					{
						flag=0;
						break;
					}
					if ((double)(fabs(W[nm])+anorm) == anorm)
						break;
				}
				if (flag)
				{
					c=0.0;  //Cancellation of rv1[l], if l > 1.
					s=1.0;
					for (i=l ;i<=k; i++)
					{
						f = s*rv1[i];
						rv1[i] = c*rv1[i];
						if ((double)(fabs(f)+anorm) == anorm)
							break;
						g = W[i];
						h = pythagora<ScalarType>(f,g);
						W[i] = h;
						h = (ScalarType)1.0/h;
						c = g*h;
						s = -f*h;
						for (j=0; j<m; j++)
						{
							y = A[j][nm];
							z = A[j][i];
							A[j][nm]	= y*c + z*s;
							A[j][i]		= z*c - y*s;
						}
					}
				}
				z = W[k];
				if (l == k)  //Convergence.
				{
					if (z < 0.0) { // Singular value is made nonnegative.
						W[k] = -z;
						for (j=0; j<n; j++)
							V[j][k] = -V[j][k];
					}
					break;
				}
				if (its == max_iters)
				{
					convergence = false;
				}
				x = W[l]; // Shift from bottom 2-by-2 minor.
				nm = k-1;
				y = W[nm];
				g = rv1[nm];
				h = rv1[k];
				f = ((y-z)*(y+z) + (g-h)*(g+h))/((ScalarType)2.0*h*y);
				g = pythagora<ScalarType>(f,1.0);
				f=((x-z)*(x+z) + h*((y/(f+sign(g,f)))-h))/x;
				c=s=1.0;
				//Next QR transformation:
				for (j=l; j<= nm;j++)
				{
					i = j+1;
					g = rv1[i];
					y = W[i];
					h = s*g;
					g = c*g;
					z = pythagora<ScalarType>(f,h);
					rv1[j] = z;
					c = f/z;
					s = h/z;
					f = x*c + g*s;
					g = g*c - x*s;
					h = y*s;
					y *= c;
					for (jj=0; jj<n; jj++)
					{
						x = V[jj][j];
						z = V[jj][i];
						V[jj][j] = x*c + z*s;
						V[jj][i] = z*c - x*s;
					}
					z = pythagora<ScalarType>(f,h);
					W[j] = z;
					// Rotation can be arbitrary if z = 0.
					if (z)
					{
						z = (ScalarType)1.0/z;
						c = f*z;
						s = h*z;
					}
					f = c*g + s*y;
					x = c*y - s*g;
					for (jj=0; jj<m; jj++)
					{
						y = A[jj][j];
						z = A[jj][i];
						A[jj][j] = y*c + z*s;
						A[jj][i] = z*c - y*s;
					}
				}
				rv1[l] = 0.0;
				rv1[k] = f;
				W[k]	 = x;
			}
		}
		delete []rv1;

		if (sorting!=LeaveUnsorted)
			Sort<MATRIX_TYPE>(A, W, V, sorting);

		return convergence;
	};


	/*!
	*	Sort the singular values computed by the <CODE>SingularValueDecomposition</CODE> procedure and
	* modify the matrices <I>U</I> and <I>V</I> accordingly.
	*/
	// TODO modify the last parameter type
	template< typename MATRIX_TYPE >
	void Sort(MATRIX_TYPE &U, typename MATRIX_TYPE::ScalarType W[], MATRIX_TYPE &V, const SortingStrategy sorting)
	{
		typedef typename MATRIX_TYPE::ScalarType ScalarType;

		assert(U.ColumnsNumber()==V.ColumnsNumber());

		int mu = U.RowsNumber();
		int mv = V.RowsNumber();
		int n  = U.ColumnsNumber();

		//ScalarType* u = &U[0][0];
		//ScalarType* v = &V[0][0];

		for (int i=0; i<n; i++)
		{
			int  k = i;
			ScalarType p = W[i];
			switch (sorting)
			{
			case SortAscending:
				{
					for (int j=i+1; j<n; j++)
					{
						if (W[j] < p)
						{
							k = j;
							p = W[j];
						}
					}
					break;
				}
			case SortDescending:
				{
					for (int j=i+1; j<n; j++)
					{
						if (W[j] > p)
						{
							k = j;
							p = W[j];
						}
					}
					break;
				}
      case LeaveUnsorted: break; // nothing to do.
      }
			if (k != i)
			{
				W[k] = W[i];  // i.e.
				W[i] = p;			// swaps the i-th and the k-th elements

				int j = mu;
				//ScalarType* uji = u + i; // uji = &U[0][i]
				//ScalarType* ujk = u + k; // ujk = &U[0][k]
				//ScalarType* vji = v + i; // vji = &V[0][i]
				//ScalarType* vjk = v + k; // vjk = &V[0][k]
				//if (j)
				//{
				//	for(;;)									for( ; j!=0; --j, uji+=n, ujk+=n)
				//	{												{
				//		p = *uji;								p = *uji;			// i.e.
				//		*uji = *ujk;						*uji = *ujk;	// swap( U[s][i], U[s][k] )
				//		*ujk = p;								*ujk = p;			//
				//		if (!(--j))						}
				//			break;
				//		uji += n;
				//		ujk += n;
				//	}
				//}
				for(int s=0; j!=0; ++s, --j)
					std::swap(U[s][i], U[s][k]);

				j = mv;
				//if (j!=0)
				//{
				//	for(;;)									for ( ; j!=0; --j, vji+=n, ujk+=n)
				//	{												{
				//		p    = *vji;						p    = *vji;	// i.e.
				//		*vji = *vjk;						*vji = *vjk;	// swap( V[s][i], V[s][k] )
				//		*vjk = p;								*vjk = p;			//
				//		if (!(--j))						}
				//			break;
				//		vji += n;
				//		vjk += n;
				//	}
				//}
				for (int s=0; j!=0; ++s, --j)
					std::swap(V[s][i], V[s][k]);
			}
		}
	}


	/*!
	*	Solves AxX = B for a vector X, where A is specified by the matrices <I>U<SUB>mxn</SUB></I>,
	*	<I>W<SUB>nx1</SUB></I> and <I>V<SUB>nxn</SUB></I> as returned by <CODE>SingularValueDecomposition</CODE>.
	*	No input quantities are destroyed, so the routine may be called sequentially with different bxs.
	*	\param x	is the output solution vector (<I>x<SUB>nx1</SUB></I>)
	*	\param b	is the input right-hand side (<I>b<SUB>nx1</SUB></I>)
	*/
	template <typename MATRIX_TYPE>
		static void SingularValueBacksubstitution(const MATRIX_TYPE												&U,
																							const typename MATRIX_TYPE::ScalarType	*W,
																							const MATRIX_TYPE												&V,
																										typename MATRIX_TYPE::ScalarType	*x,
																							const typename MATRIX_TYPE::ScalarType	*b)
	{
		typedef typename MATRIX_TYPE::ScalarType ScalarType;
		unsigned int jj, j, i;
		unsigned int columns_number = U.ColumnsNumber();
		unsigned int rows_number    = U.RowsNumber();
		ScalarType s;
		ScalarType *tmp	=	new ScalarType[columns_number];
		for (j=0; j<columns_number; j++) //Calculate U^T * B.
		{
			s = 0;
			if (W[j]!=0)							//Nonzero result only if wj is nonzero.
			{
				for (i=0; i<rows_number; i++)
					s += U[i][j]*b[i];
				s /= W[j];							//This is the divide by wj .
			}
			tmp[j]=s;
		}
		for (j=0;j<columns_number;j++)	//Matrix multiply by V to get answer.
		{
			s = 0;
			for (jj=0; jj<columns_number; jj++)
				s += V[j][jj]*tmp[jj];
			x[j]=s;
		}
		delete []tmp;
	};

	/*! @} */
}; // end of namespace

#endif
