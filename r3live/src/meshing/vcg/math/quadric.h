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
Revision 1.7  2006/11/13 12:53:40  ponchio
just added an #include <matrix33>

Revision 1.6  2006/10/09 20:23:00  cignoni
Added a minimum method that uses SVD. Unfortunately it is much much slower.

Revision 1.5  2004/12/10 01:31:59  cignoni
added an alternative QuadricMinimization (we should use LRU decomposition!!)

Revision 1.3  2004/10/25 16:23:51  ponchio
typedef ScalarType ScalarType; was a problem on g++

Revision 1.2  2004/10/25 16:15:59  ganovelli
template changed

Revision 1.1  2004/09/14 19:48:27  ganovelli
created


****************************************************************************/
#ifndef __VCGLIB_QUADRIC
#define __VCGLIB_QUADRIC

#include <vcg/space/point3.h>
#include <vcg/space/plane3.h>
#include <vcg/math/matrix33.h>

namespace vcg {
namespace math {


template<typename  Scalar>
class Quadric
{
public:
        typedef Scalar ScalarType;
	ScalarType a[6];		// Matrice 3x3 simmetrica: a11 a12 a13 a22 a23 a33
	ScalarType b[3];		// Vettore r3
	ScalarType c;			// Fattore scalare (se -1 quadrica nulla)

	inline Quadric() { c = -1; }

	// Necessari se si utilizza stl microsoft
	// inline bool operator <  ( const Quadric & q ) const { return false; }
	// inline bool operator == ( const Quadric & q ) const { return true; }

	bool IsValid() const { return c>=0; }
	void SetInvalid() { c = -1.0; }

template< class PlaneType >
	void ByPlane( const PlaneType & p )					// Init dato un piano
	{
		a[0] =  (ScalarType)p.Direction()[0]*p.Direction()[0];	// a11
		a[1] =  (ScalarType)p.Direction()[1]*p.Direction()[0];	// a12 (=a21)
		a[2] =  (ScalarType)p.Direction()[2]*p.Direction()[0];	// a13 (=a31)
		a[3] =  (ScalarType)p.Direction()[1]*p.Direction()[1];	// a22
		a[4] =  (ScalarType)p.Direction()[2]*p.Direction()[1];	// a23 (=a32)
		a[5] =  (ScalarType)p.Direction()[2]*p.Direction()[2];	// a33
		b[0] = (ScalarType)(-2.0)*p.Offset()*p.Direction()[0];
		b[1] = (ScalarType)(-2.0)*p.Offset()*p.Direction()[1];
		b[2] = (ScalarType)(-2.0)*p.Offset()*p.Direction()[2];
		c    =  (ScalarType)p.Offset()*p.Offset();
	}

/* Initializes the quadric as the squared distance from a given line.
   Notice that this code also works for a vcg::Ray<T>, even though the (squared) distance
   from a ray is different "before" its origin.
 */
 template< class LineType >
  void ByLine( const LineType & r ) // Init dato un raggio
  {
    ScalarType K = (ScalarType)(r.Origin()*r.Direction());
    a[0] = (ScalarType)1.0-r.Direction()[0]*r.Direction()[0]; // a11
    a[1] = (ScalarType)-r.Direction()[0]*r.Direction()[1]; // a12 (=a21)
    a[2] = (ScalarType)-r.Direction()[0]*r.Direction()[2]; // a13 (=a31)
    a[3] = (ScalarType)1.0-r.Direction()[1]*r.Direction()[1]; // a22
    a[4] = (ScalarType)-r.Direction()[1]*r.Direction()[2]; // a23 (=a32)
    a[5] = (ScalarType)1.0-r.Direction()[2]*r.Direction()[2]; // a33
    b[0] = (ScalarType)2.0*(r.Direction()[0]*K - r.Origin()[0]);
    b[1] = (ScalarType)2.0*(r.Direction()[1]*K - r.Origin()[1]);
    b[2] = (ScalarType)2.0*(r.Direction()[2]*K - r.Origin()[2]);
    c = -K*K + (ScalarType)(r.Origin()*r.Origin());
  }

	void SetZero()																// Azzera la quadrica
	{
		a[0] = 0;
		a[1] = 0;
		a[2] = 0;
		a[3] = 0;
		a[4] = 0;
		a[5] = 0;
		b[0] = 0;
		b[1] = 0;
		b[2] = 0;
		c    = 0;
	}

void operator = ( const Quadric & q )			// Assegna una quadrica
	{
		//assert( IsValid() );
		assert( q.IsValid() );

		a[0] = q.a[0];
		a[1] = q.a[1];
		a[2] = q.a[2];
		a[3] = q.a[3];
		a[4] = q.a[4];
		a[5] = q.a[5];
		b[0] = q.b[0];
		b[1] = q.b[1];
		b[2] = q.b[2];
		c    = q.c;
	}

  void operator += ( const Quadric & q )			// Somma una quadrica
	{
		assert( IsValid() );
		assert( q.IsValid() );

		a[0] += q.a[0];
		a[1] += q.a[1];
		a[2] += q.a[2];
		a[3] += q.a[3];
		a[4] += q.a[4];
		a[5] += q.a[5];
		b[0] += q.b[0];
		b[1] += q.b[1];
		b[2] += q.b[2];
		c    += q.c;
	}

template <class ResultScalarType>
	ResultScalarType Apply( const Point3<ResultScalarType> & p ) const	// Applica la quadrica al punto p
	{
		assert( IsValid() );
/*
	// Versione Lenta

		Point3d t;
		t[0] = p[0]*a[0] + p[1]*a[1] + p[2]*a[2];
		t[1] = p[0]*a[1] + p[1]*a[3] + p[2]*a[4];
		t[2] = p[0]*a[2] + p[1]*a[4] + p[2]*a[5];
		double k = b[0]*p[0] + b[1]*p[1] + b[2]*p[2];
		double tp = t*p ;
		assert(tp+k+c >= 0);
		return tp + k + c;
	*/

	/* Versione veloce */
		return ResultScalarType (
      p[0]*p[0]*a[0] + 2*p[0]*p[1]*a[1] + 2*p[0]*p[2]*a[2] + p[0]*b[0]
			               +   p[1]*p[1]*a[3] + 2*p[1]*p[2]*a[4] + p[1]*b[1]
			                                  +   p[2]*p[2]*a[5] + p[2]*b[2]	+ c);

	}

// spostare..risolve un sistema 3x3
template<class FLTYPE>
bool Gauss33( FLTYPE x[], FLTYPE C[3][3+1] )
{
    const FLTYPE keps = (FLTYPE)1e-3;
    int i,j,k;

    FLTYPE eps;					// Determina valore cond.
		eps = math::Abs(C[0][0]);
    for(i=1;i<3;++i)
    {
		FLTYPE t = math::Abs(C[i][i]);
		if( eps<t ) eps = t;
    }
    eps *= keps;

    for (i = 0; i < 3 - 1; ++i)    		// Ciclo di riduzione
    {
        int ma = i;				// Ricerca massimo pivot
        FLTYPE vma = math::Abs( C[i][i] );
        for (k = i + 1; k < 3; k++)
        {
            FLTYPE t = math::Abs( C[k][i] );
            if (t > vma)
            {
                vma = t;
                ma  = k;
            }
        }
        if (vma<eps)
            return false;        			// Matrice singolare
        if(i!=ma)				// Swap del massimo pivot
            for(k=0;k<=3;k++)
            {
                FLTYPE t = C[i][k];
                C[i][k] = C[ma][k];
                C[ma][k] = t;
            }
        for (k = i + 1; k < 3; k++)        	//  Riduzione
        {
            FLTYPE s;
            s = C[k][i] / C[i][i];
            for (j = i+1; j <= 3; j++)
                C[k][j] -= C[i][j] * s;
            C[k][i] = 0.0;
        }
    }

        // Controllo finale singolarita'
    if( math::Abs(C[3-1][3- 1])<eps)
        return false;

    for (i=3-1; i>=0; i--)			// Sostituzione
    {
        FLTYPE t;
        for (t = 0.0, j = i + 1; j < 3; j++)
            t += C[i][j] * x[j];
        x[i] = (C[i][3] - t) / C[i][i];
    }

    return true;
}

// determina il punto di errore minimo
template <class ReturnScalarType>
bool Minimum(Point3<ReturnScalarType> &x)
{
		ReturnScalarType C[3][4];
		C[0][0]=a[0]; C[0][1]=a[1]; C[0][2]=a[2];
		C[1][0]=a[1]; C[1][1]=a[3]; C[1][2]=a[4];
		C[2][0]=a[2]; C[2][1]=a[4]; C[2][2]=a[5];

		C[0][3]=-b[0]/2;
		C[1][3]=-b[1]/2;
		C[2][3]=-b[2]/2;
		return Gauss33(&(x[0]),C);
}

// determina il punto di errore minimo usando le fun di inversione di matrice che usano svd
// Molto + lento
template <class ReturnScalarType>
bool MinimumSVD(Point3<ReturnScalarType> &x)
{
    Matrix33<ReturnScalarType> C;
		C[0][0]=a[0]; C[0][1]=a[1]; C[0][2]=a[2];
		C[1][0]=a[1]; C[1][1]=a[3]; C[1][2]=a[4];
		C[2][0]=a[2]; C[2][1]=a[4]; C[2][2]=a[5];
    Invert(C);

		C[0][3]=-b[0]/2;
		C[1][3]=-b[1]/2;
		C[2][3]=-b[2]/2;
		x = C * Point3<ReturnScalarType>(-b[0]/2,-b[1]/2,-b[2]/2) ;
    return  true;
}


bool MinimumNew(Point3<ScalarType> &x) const
{
  ScalarType c0=-b[0]/2;
  ScalarType c1=-b[1]/2;
  ScalarType c2=-b[2]/2;

  ScalarType t125 = a[4]*a[1];
  ScalarType t124 = a[4]*a[4]-a[3]*a[5];
  ScalarType t123 = -a[1]*a[5]+a[4]*a[2];
  ScalarType t122 = a[2]*a[3]-t125;
  ScalarType t121 = -a[2]*a[1]+a[0]*a[4];
  ScalarType t120 = a[2]*a[2];
  ScalarType t119 = a[1]*a[1];
  ScalarType t117 = 1.0/(-a[3]*t120+2*a[2]*t125-t119*a[5]-t124*a[0]);
  x[0] = -(t124*c0+t122*c2-t123*c1)*t117;
  x[1] = (t123*c0-t121*c2+(-t120+a[0]*a[5])*c1)*t117;
  x[2] = -(t122*c0+(t119-a[0]*a[3])*c2+t121*c1)*t117;
  return true;
}
// determina il punto di errore minimo vincolato nel segmento (a,b)
bool Minimum(Point3<ScalarType> &x,Point3<ScalarType> &pa,Point3<ScalarType> &pb){
ScalarType	t1,t2, t4, t5, t8, t9,
	t11,t12,t14,t15,t17,t18,t25,t26,t30,t34,t35,
	t41,t42,t44,t45,t50,t52,t54,
	t56,t21,t23,t37,t64,lambda;

	  t1 = a[4]*pb.z();
	  t2 = t1*pa.y();
      t4 = a[1]*pb.y();
      t5 = t4*pa.x();
      t8 = a[1]*pa.y();
      t9 = t8*pa.x();
      t11 = a[4]*pa.z();
      t12 = t11*pa.y();
      t14 = pa.z()*pa.z();
      t15 = a[5]*t14;
      t17 = a[2]*pa.z();
      t18 = t17*pa.x();
      t21 = 2.0*t11*pb.y();
      t23 = a[5]*pb.z()*pa.z();
      t25 = a[2]*pb.z();
      t26 = t25*pa.x();
      t30 = a[0]*pb.x()*pa.x();
      t34 = 2.0*a[3]*pb.y()*pa.y();
      t35 = t17*pb.x();
      t37 = t8*pb.x();
      t41 = pa.x()*pa.x();
      t42 = a[0]*t41;
      t44 = pa.y()*pa.y();
      t45 = a[3]*t44;
      t50 = 2.0*t30+t34+2.0*t35+2.0*t37-(-b[2]/2)*pa.z()-(-b[0]/2)*pa.x()-2.0*t42-2.0*t45+(-b[1]/2)*pb.y()
+(-b[0]/2)*pb.x()-(-b[1]/2)*pa.y();
      t52 = pb.y()*pb.y();
      t54 = pb.z()*pb.z();
      t56 = pb.x()*pb.x();
      t64 = t5+t37-t9+t30-t18+t35+t26-t25*pb.x()+t2-t1*pb.y()+t23;
      lambda = (2.0*t2+2.0*t5+(-b[2]/2)*pb.z()-4.0*t9-4.0*t12-2.0*t15-4.0*t18+t21+2.0*t23+
2.0*t26+t50)/(-t45-a[3]*t52-a[5]*t54-a[0]*t56-t15-t42+t34-2.0*t12+t21-2.0*t4*pb.x()+
2.0*t64)/2.0;

	  if(lambda<0)  lambda=0;  else	  if(lambda>1)   lambda = 1;

		 x = pa*(1.0-lambda)+pb*lambda;
		 return true;
	}

  void operator *= ( const ScalarType & w )			// Amplifica una quadirca
	{
		assert( IsValid() );

		a[0] *= w;
		a[1] *= w;
		a[2] *= w;
		a[3] *= w;
		a[4] *= w;
		a[5] *= w;
		b[0] *= w;
		b[1] *= w;
		b[2] *= w;
		c    *= w;
	}


};

typedef Quadric<short>  Quadrics;
typedef Quadric<int>	  Quadrici;
typedef Quadric<float>  Quadricf;
typedef Quadric<double> Quadricd;



	} // end namespace math
} // end namespace vcg

#endif
