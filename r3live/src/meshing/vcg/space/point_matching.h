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

#ifndef _VCG_MATH_POINTMATCHING_H
#define _VCG_MATH_POINTMATCHING_H

#include <vcg/math/quaternion.h>
#include <vcg/math/matrix44.h>

#include <eigenlib/Eigen/Dense>
#include <eigenlib/Eigen/Eigenvalues>
#include  <iostream>

namespace vcg
{

/*! \brief Compute cross covariance

It computes the cross covariance matrix of two set of 3D points P and X;
it returns also the barycenters of P and X.
Ref:

Besl, McKay
A method for registration of 3d Shapes
IEEE TPAMI Vol 14, No 2 1992

*/
template <class S >
void ComputeCrossCovarianceMatrix(const std::vector<Point3<S> > &spVec, Point3<S> &spBarycenter,
                                  const std::vector<Point3<S> > &tpVec, Point3<S> &tpBarycenter,
                                  Eigen::Matrix3d &m)
{
    assert(spVec.size()==tpVec.size());
    m.setZero();
    spBarycenter.SetZero();
    tpBarycenter.SetZero();
    Eigen::Vector3d spe;
    Eigen::Vector3d tpe;
    typename std::vector <Point3<S> >::const_iterator si,ti;
    for(si=spVec.begin(),ti=tpVec.begin();si!=spVec.end();++si,++ti){
        spBarycenter+=*si;
        tpBarycenter+=*ti;
         si->ToEigenVector(spe);
         ti->ToEigenVector(tpe);
        m+=spe*tpe.transpose();
    }
    spBarycenter/=spVec.size();
    tpBarycenter/=tpVec.size();
    spBarycenter.ToEigenVector(spe);
    tpBarycenter.ToEigenVector(tpe);
    m/=spVec.size();
    m-=spe*tpe.transpose();
}

/*! \brief Compute the roto-translation that applied to PMov bring them onto Pfix
 * Rotation is computed as a quaternion.
 *
 * E.g. it find a matrix such that:
 *
 *       Pfix[i] = res * Pmov[i]
 *
 * Ref:
 * Besl, McKay
 * A method for registration of 3d Shapes
 * IEEE TPAMI Vol 14, No 2 1992
 */

template < class  S >
void ComputeRigidMatchMatrix(std::vector<Point3<S> > &Pfix,
                             std::vector<Point3<S> > &Pmov,
                             Quaternion<S>  &q,
                             Point3<S>  &tr)
{
  Eigen::Matrix3d ccm;
  Point3<S> bfix,bmov; // baricenter of src e trg

  ComputeCrossCovarianceMatrix(Pmov,bmov,Pfix,bfix,ccm);

  Eigen::Matrix3d cyc; // the cyclic components of the cross covariance matrix.
  cyc=ccm-ccm.transpose();

  Eigen::Matrix4d  QQ;
  QQ.setZero();
  Eigen::Vector3d D(cyc(1,2),cyc(2,0),cyc(0,1));

  Eigen::Matrix3d RM;
  RM.setZero();
  RM(0,0)=-ccm.trace();
  RM(1,1)=-ccm.trace();
  RM(2,2)=-ccm.trace();
  RM += ccm + ccm.transpose();

  QQ(0,0) = ccm.trace();
  QQ.block<1,3> (0,1) = D.transpose();
  QQ.block<3,1> (1,0) = D;
  QQ.block<3,3> (1,1) = RM;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eig(QQ);
  Eigen::Vector4d eval = eig.eigenvalues();
  Eigen::Matrix4d evec = eig.eigenvectors();
//  std::cout << "EigenVectors:" << std::endl << evec << std::endl;
//  std::cout << "Eigenvalues:" << std::endl << eval << std::endl;
  int ind;
  eval.cwiseAbs().maxCoeff(&ind);

  q=Quaternion<S>(evec.col(ind)[0],evec.col(ind)[1],evec.col(ind)[2],evec.col(ind)[3]);
  Matrix44<S> Rot;
  q.ToMatrix(Rot);
  tr= (bfix - Rot*bmov);
}


/*! \brief Compute the roto-translation that applied to PMov bring them onto Pfix
 * Rotation is computed as a quaternion.
 *
 * E.g. it find a matrix such that:
 *
 *       Pfix[i] = res * Pmov[i]
 *
 * Ref:
 * Besl, McKay
 * A method for registration of 3d Shapes
 * IEEE TPAMI Vol 14, No 2 1992
 */

template < class  S >
void ComputeRigidMatchMatrix(std::vector<Point3<S> > &Pfix,
                             std::vector<Point3<S> > &Pmov,
                             Matrix44<S> &res)
{
    Quaternion<S>  q;
    Point3<S>  tr;
    ComputeRigidMatchMatrix(Pfix,Pmov,q,tr);

    Matrix44<S> Rot;
    q.ToMatrix(Rot);

    Matrix44<S> Trn;
    Trn.SetTranslate(tr);

    res=Trn*Rot;
}


/*
Compute a similarity matching (rigid + uniform scaling)
simply create a temporary point set with the correct scaling factor
*/ 
template <class S>
void ComputeSimilarityMatchMatrix(std::vector<Point3<S> > &Pfix,
                                  std::vector<Point3<S> > &Pmov,
                                  Matrix44<S> &res)
{
  S scalingFactor=0;
  for(size_t i=0;i<( Pmov.size()-1);++i)
  {
    scalingFactor += Distance(Pmov[i],Pmov[i+1])/ Distance(Pfix[i],Pfix[i+1]);
  }
  scalingFactor/=(Pmov.size()-1);

  std::vector<Point3<S> > Pnew(Pmov.size());
  for(size_t i=0;i<Pmov.size();++i)
    Pnew[i]=Pmov[i]/scalingFactor;

  ComputeRigidMatchMatrix(Pfix,Pnew,res);

  Matrix44<S> scaleM; scaleM.SetDiagonal(1.0/scalingFactor);
  res = res * scaleM;
}

} // end namespace

#endif
