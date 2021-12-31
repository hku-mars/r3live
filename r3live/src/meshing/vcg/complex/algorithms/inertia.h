/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2005                                                \/)\/    *
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
#ifndef _VCG_INERTIA_
#define _VCG_INERTIA_


#include <eigenlib/Eigen/Core>
#include <eigenlib/Eigen/Eigenvalues>
#include <vcg/complex/algorithms/update/normal.h>

namespace vcg
{
  namespace tri
  {
  /*! \brief Methods for computing Polyhedral Mass properties (like inertia tensor, volume, etc)


  The algorithm is based on a three step reduction of the volume integrals
  to successively simpler integrals. The algorithm is designed to minimize
  the numerical errors that can result from poorly conditioned alignment of
  polyhedral faces. It is also designed for efficiency. All required volume
  integrals of a polyhedron are computed together during a single walk over
  the boundary of the polyhedron; exploiting common subexpressions reduces
  floating point operations.

  For more information, check out:

  <b>Brian Mirtich,</b>
  ``Fast and Accurate Computation of Polyhedral Mass Properties,''
  journal of graphics tools, volume 1, number 2, 1996

  */
template <class MeshType>
class Inertia
{
	typedef typename MeshType::VertexType     VertexType;
	typedef typename MeshType::VertexPointer  VertexPointer;
	typedef typename MeshType::VertexIterator VertexIterator;
	typedef	typename MeshType::ScalarType			ScalarType;
	typedef typename MeshType::FaceType       FaceType;
	typedef typename MeshType::FacePointer    FacePointer;
	typedef typename MeshType::FaceIterator   FaceIterator;
	typedef typename MeshType::ConstFaceIterator   ConstFaceIterator;
	typedef typename MeshType::FaceContainer  FaceContainer;
	typedef typename MeshType::CoordType  CoordType;

private :
	enum {X=0,Y=1,Z=2};
	inline ScalarType SQR(ScalarType &x) const { return x*x;}
	inline ScalarType CUBE(ScalarType &x) const { return x*x*x;}

 int A;   /* alpha */
 int B;   /* beta */
 int C;   /* gamma */

/* projection integrals */
 double P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;

/* face integrals */
 double Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;

/* volume integrals */
 double T0, T1[3], T2[3], TP[3];

public:
 /*! \brief Basic constructor

   When you create a Inertia object, you have to specify the mesh that it refers to.
   The properties are computed at that moment. Subsequent modification of the mesh does not affect these values.
   */
 Inertia(MeshType &m) {Compute(m);}

/* compute various integrations over projection of face */
 void compProjectionIntegrals(FaceType &f)
{
  double a0, a1, da;
  double b0, b1, db;
  double a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
  double a1_2, a1_3, b1_2, b1_3;
  double C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
  double Cab, Kab, Caab, Kaab, Cabb, Kabb;
  int i;

  P1 = Pa = Pb = Paa = Pab = Pbb = Paaa = Paab = Pabb = Pbbb = 0.0;

  for (i = 0; i < 3; i++) {
    a0 = f.V(i)->P()[A];
    b0 = f.V(i)->P()[B];
    a1 = f.V1(i)->P()[A];
    b1 = f.V1(i)->P()[B];
    da = a1 - a0;
    db = b1 - b0;
    a0_2 = a0 * a0; a0_3 = a0_2 * a0; a0_4 = a0_3 * a0;
    b0_2 = b0 * b0; b0_3 = b0_2 * b0; b0_4 = b0_3 * b0;
    a1_2 = a1 * a1; a1_3 = a1_2 * a1;
    b1_2 = b1 * b1; b1_3 = b1_2 * b1;

    C1 = a1 + a0;
    Ca = a1*C1 + a0_2; Caa = a1*Ca + a0_3; Caaa = a1*Caa + a0_4;
    Cb = b1*(b1 + b0) + b0_2; Cbb = b1*Cb + b0_3; Cbbb = b1*Cbb + b0_4;
    Cab = 3*a1_2 + 2*a1*a0 + a0_2; Kab = a1_2 + 2*a1*a0 + 3*a0_2;
    Caab = a0*Cab + 4*a1_3; Kaab = a1*Kab + 4*a0_3;
    Cabb = 4*b1_3 + 3*b1_2*b0 + 2*b1*b0_2 + b0_3;
    Kabb = b1_3 + 2*b1_2*b0 + 3*b1*b0_2 + 4*b0_3;

    P1 += db*C1;
    Pa += db*Ca;
    Paa += db*Caa;
    Paaa += db*Caaa;
    Pb += da*Cb;
    Pbb += da*Cbb;
    Pbbb += da*Cbbb;
    Pab += db*(b1*Cab + b0*Kab);
    Paab += db*(b1*Caab + b0*Kaab);
    Pabb += da*(a1*Cabb + a0*Kabb);
  }

  P1 /= 2.0;
  Pa /= 6.0;
  Paa /= 12.0;
  Paaa /= 20.0;
  Pb /= -6.0;
  Pbb /= -12.0;
  Pbbb /= -20.0;
  Pab /= 24.0;
  Paab /= 60.0;
  Pabb /= -60.0;
}


void CompFaceIntegrals(FaceType &f)
{
	Point3<ScalarType>  n;
	ScalarType w;
  double k1, k2, k3, k4;

  compProjectionIntegrals(f);

  n = f.N();
  w = -f.V(0)->P()*n;
  k1 = 1 / n[C]; k2 = k1 * k1; k3 = k2 * k1; k4 = k3 * k1;

  Fa = k1 * Pa;
  Fb = k1 * Pb;
  Fc = -k2 * (n[A]*Pa + n[B]*Pb + w*P1);

  Faa = k1 * Paa;
  Fbb = k1 * Pbb;
  Fcc = k3 * (SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb
     + w*(2*(n[A]*Pa + n[B]*Pb) + w*P1));

  Faaa = k1 * Paaa;
  Fbbb = k1 * Pbbb;
  Fccc = -k4 * (CUBE(n[A])*Paaa + 3*SQR(n[A])*n[B]*Paab
       + 3*n[A]*SQR(n[B])*Pabb + CUBE(n[B])*Pbbb
       + 3*w*(SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb)
       + w*w*(3*(n[A]*Pa + n[B]*Pb) + w*P1));

  Faab = k1 * Paab;
  Fbbc = -k2 * (n[A]*Pabb + n[B]*Pbbb + w*Pbb);
  Fcca = k3 * (SQR(n[A])*Paaa + 2*n[A]*n[B]*Paab + SQR(n[B])*Pabb
     + w*(2*(n[A]*Paa + n[B]*Pab) + w*Pa));
}


/*! main function to be called.

  It requires a watertight mesh with per face normals.

*/
void Compute(MeshType &m)
{
  tri::UpdateNormal<MeshType>::PerFaceNormalized(m);
  double nx, ny, nz;

  T0 = T1[X] = T1[Y] = T1[Z]
     = T2[X] = T2[Y] = T2[Z]
     = TP[X] = TP[Y] = TP[Z] = 0;
    FaceIterator fi;
    for (fi=m.face.begin(); fi!=m.face.end();++fi) if(!(*fi).IsD() && vcg::DoubleArea(*fi)>std::numeric_limits<float>::min()) {
        FaceType &f=(*fi);

    nx = fabs(f.N()[0]);
    ny = fabs(f.N()[1]);
    nz = fabs(f.N()[2]);
    if (nx > ny && nx > nz) C = X;
    else C = (ny > nz) ? Y : Z;
    A = (C + 1) % 3;
    B = (A + 1) % 3;

    CompFaceIntegrals(f);

    T0 += f.N()[X] * ((A == X) ? Fa : ((B == X) ? Fb : Fc));

    T1[A] += f.N()[A] * Faa;
    T1[B] += f.N()[B] * Fbb;
    T1[C] += f.N()[C] * Fcc;
    T2[A] += f.N()[A] * Faaa;
    T2[B] += f.N()[B] * Fbbb;
    T2[C] += f.N()[C] * Fccc;
    TP[A] += f.N()[A] * Faab;
    TP[B] += f.N()[B] * Fbbc;
    TP[C] += f.N()[C] * Fcca;
  }

  T1[X] /= 2; T1[Y] /= 2; T1[Z] /= 2;
  T2[X] /= 3; T2[Y] /= 3; T2[Z] /= 3;
  TP[X] /= 2; TP[Y] /= 2; TP[Z] /= 2;
}

/*! \brief Return the Volume (or mass) of the mesh.

Meaningful only if the mesh is watertight.
*/
ScalarType Mass()
{
    return static_cast<ScalarType>(T0);
}

/*! \brief Return the Center of Mass (or barycenter) of the mesh.

Meaningful only if the mesh is watertight.
*/
Point3<ScalarType>  CenterOfMass()
{
    Point3<ScalarType>  r;
  r[X] = T1[X] / T0;
  r[Y] = T1[Y] / T0;
  r[Z] = T1[Z] / T0;
    return r;
}
void InertiaTensor(Matrix33<ScalarType> &J ){
    Point3<ScalarType>  r;
  r[X] = T1[X] / T0;
  r[Y] = T1[Y] / T0;
  r[Z] = T1[Z] / T0;
  /* compute inertia tensor */
  J[X][X] = (T2[Y] + T2[Z]);
  J[Y][Y] = (T2[Z] + T2[X]);
  J[Z][Z] = (T2[X] + T2[Y]);
  J[X][Y] = J[Y][X] = - TP[X];
  J[Y][Z] = J[Z][Y] = - TP[Y];
  J[Z][X] = J[X][Z] = - TP[Z];

  J[X][X] -= T0 * (r[Y]*r[Y] + r[Z]*r[Z]);
  J[Y][Y] -= T0 * (r[Z]*r[Z] + r[X]*r[X]);
  J[Z][Z] -= T0 * (r[X]*r[X] + r[Y]*r[Y]);
  J[X][Y] = J[Y][X] += T0 * r[X] * r[Y];
  J[Y][Z] = J[Z][Y] += T0 * r[Y] * r[Z];
  J[Z][X] = J[X][Z] += T0 * r[Z] * r[X];
}

//void InertiaTensor(Matrix44<ScalarType> &J )
void InertiaTensor(Eigen::Matrix3d &J )
{
  J=Eigen::Matrix3d::Identity();
  Point3d  r;
  r[X] = T1[X] / T0;
  r[Y] = T1[Y] / T0;
  r[Z] = T1[Z] / T0;
  /* compute inertia tensor */
  J(X,X) = (T2[Y] + T2[Z]);
  J(Y,Y) = (T2[Z] + T2[X]);
  J(Z,Z) = (T2[X] + T2[Y]);
  J(X,Y) = J(Y,X) = - TP[X];
  J(Y,Z) = J(Z,Y) = - TP[Y];
  J(Z,X) = J(X,Z) = - TP[Z];

  J(X,X) -= T0 * (r[Y]*r[Y] + r[Z]*r[Z]);
  J(Y,Y) -= T0 * (r[Z]*r[Z] + r[X]*r[X]);
  J(Z,Z) -= T0 * (r[X]*r[X] + r[Y]*r[Y]);
  J(X,Y) = J(Y,X) += T0 * r[X] * r[Y];
  J(Y,Z) = J(Z,Y) += T0 * r[Y] * r[Z];
  J(Z,X) = J(X,Z) += T0 * r[Z] * r[X];
}



/*! \brief Return the Inertia tensor the mesh.

  The result is factored as eigenvalues and eigenvectors (as ROWS).
*/
void InertiaTensorEigen(Matrix33<ScalarType> &EV, Point3<ScalarType> &ev )
{
	Eigen::Matrix3d it;
	InertiaTensor(it);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(it);
	Eigen::Vector3d c_val = eig.eigenvalues();
	Eigen::Matrix3d c_vec = eig.eigenvectors(); // eigenvector are stored as columns.
	EV.FromEigenMatrix(c_vec);
	EV.transposeInPlace();
	ev.FromEigenVector(c_val);
}

/** Compute covariance matrix of a mesh, i.e. the integral
        int_{M} { (x-b)(x-b)^T }dx  where b is the barycenter and x spans over the mesh M
 */
static void Covariance(const MeshType & m, vcg::Point3<ScalarType> & bary, vcg::Matrix33<ScalarType> &C)
{
	// find the barycenter
	ConstFaceIterator fi;
	ScalarType area = 0.0;
	bary.SetZero();
	for(fi = m.face.begin(); fi != m.face.end(); ++fi)
		if(!(*fi).IsD())
			{
				bary += vcg::Barycenter( *fi )* vcg::DoubleArea(*fi);
				area+=vcg::DoubleArea(*fi);
			}
	bary/=area;


	C.SetZero();
	// C as covariance of triangle (0,0,0)(1,0,0)(0,1,0)
	vcg::Matrix33<ScalarType> C0;
	C0.SetZero();
	C0[0][0] = C0[1][1] = 2.0;
	C0[0][1] = C0[1][0] = 1.0;
	C0*=1/24.0;

	// integral of (x,y,0) in the same triangle
	CoordType X(1/6.0,1/6.0,0);
	vcg::Matrix33<ScalarType> A, // matrix that bring the vertices to (v1-v0,v2-v0,n)
													DC;
	for(fi = m.face.begin(); fi != m.face.end(); ++fi)
		if(!(*fi).IsD())
		{
			const CoordType &P0 = (*fi).cP(0);
			const CoordType &P1 = (*fi).cP(1);
			const CoordType &P2 = (*fi).cP(2);
			CoordType  n = ((P1-P0)^(P2-P0));
			const float da = n.Norm();
			n/=da*da;

			A.SetColumn(0, P1-P0);
			A.SetColumn(1, P2-P0);
			A.SetColumn(2, n);
			CoordType delta = P0 - bary;

			/* DC is calculated as integral of (A*x+delta) * (A*x+delta)^T over the triangle,
				 where delta = v0-bary
			*/

			DC.SetZero();
			DC+= A*C0*A.transpose();
			vcg::Matrix33<ScalarType> tmp;
			tmp.OuterProduct(A*X,delta);
			DC += tmp + tmp.transpose();
			DC+= tmp;
			tmp.OuterProduct(delta,delta);
			DC+=tmp*0.5;
// 		DC*=fabs(A.Determinant());	// the determinant of A is the jacobian of the change of variables A*x+delta
			DC*=da;											// the determinant of A is also the double area of *fi
			C+=DC;
	}

}
}; // end class Inertia

  } // end namespace tri
} // end namespace vcg


#endif
