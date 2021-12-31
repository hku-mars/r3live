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

#ifndef __VCGLIB_FITTING3
#define __VCGLIB_FITTING3

#include <vector>
#include <vcg/space/plane3.h>
#include <vcg/math/matrix44.h>
#include <vcg/math/matrix33.h>

#include <eigenlib/Eigen/Core>
#include <eigenlib/Eigen/Eigenvalues>

namespace vcg {

/*! \brief compute the covariance matrix of a set of point

  It returns also the barycenter of the point set
  */
template <class S >
void ComputeCovarianceMatrix(const std::vector<Point3<S> > &pointVec, Point3<S> &barycenter, Eigen::Matrix<S,3,3> &m)
{
  // first cycle: compute the barycenter
  barycenter.SetZero();
  typename  std::vector< Point3<S> >::const_iterator pit;
  for( pit = pointVec.begin(); pit != pointVec.end(); ++pit) barycenter+= (*pit);
  barycenter/=pointVec.size();

  // second cycle: compute the covariance matrix
  m.setZero();
  Eigen::Matrix<S,3,1> p;
  for(pit = pointVec.begin(); pit != pointVec.end(); ++pit) {
    ((*pit)-barycenter).ToEigenVector(p);
    m+= p*p.transpose(); // outer product
  }
}

/*! \brief Compute the plane best fitting a set of points

The algorithm used is the classical Covariance matrix eigenvector approach.

*/
template <class S>
void FitPlaneToPointSet(const std::vector< Point3<S> > & pointVec, Plane3<S> & plane)
{
  Eigen::Matrix<S,3,3> covMat = Eigen::Matrix<S,3,3>::Zero();
  Point3<S> b;
  ComputeCovarianceMatrix(pointVec,b,covMat);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<S,3,3> > eig(covMat);
  Eigen::Matrix<S,3,1> eval = eig.eigenvalues();
  Eigen::Matrix<S,3,3> evec = eig.eigenvectors();
  eval = eval.cwiseAbs();
  int minInd;
  eval.minCoeff(&minInd);
  Point3<S> d;
  d[0] = evec(0,minInd);
  d[1] = evec(1,minInd);
  d[2] = evec(2,minInd);

  plane.Init(b,d);
}

/*! \brief compute the weighted covariance matrix of a set of point

  It returns also the weighted barycenter of the point set.
  When computing the covariance matrix  the weights are applied to the points transposed to the origin.
  */
template <class S >
void ComputeWeightedCovarianceMatrix(const std::vector<Point3<S> > &pointVec, const std::vector<S> &weightVec, Point3<S> &bp, Eigen::Matrix<S,3,3> &m)
{
  assert(pointVec.size() == weightVec.size());
  // First cycle: compute the weighted barycenter
  bp.SetZero();
  S wSum=0;
  typename  std::vector< Point3<S> >::const_iterator pit;
  typename  std::vector<  S>::const_iterator wit;
  for( pit = pointVec.begin(),wit=weightVec.begin(); pit != pointVec.end(); ++pit,++wit)
  {
    bp+= (*pit)*(*wit);
    wSum+=*wit;
  }
  bp /=wSum;

  // Second cycle: compute the weighted covariance matrix
  // The weights are applied to the points transposed to the origin.
  m.setZero();
  Eigen::Matrix<S,3,3> A;
  Eigen::Matrix<S,3,1> p;
  for( pit = pointVec.begin(),wit=weightVec.begin(); pit != pointVec.end(); ++pit,++wit)
  {
    (((*pit)-bp)*(*wit)).ToEigenVector(p);
    m+= p*p.transpose(); // outer product
  }
  m/=wSum;
}
/*! \brief Compute the plane best fitting a set of points

The algorithm used is the classical Covariance matrix eigenvector approach.
*/

template <class S>
void WeightedFitPlaneToPointSet(const std::vector< Point3<S> > & pointVec, const std::vector<S> &weightVec, Plane3<S> & plane)
{
  Eigen::Matrix<S,3,3> covMat = Eigen::Matrix<S,3,3>::Zero();
  Point3<S> b;
  ComputeWeightedCovarianceMatrix(pointVec,weightVec, b,covMat);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<S,3,3> > eig(covMat);
  Eigen::Matrix<S,3,1> eval = eig.eigenvalues();
  Eigen::Matrix<S,3,3> evec = eig.eigenvectors();
  eval = eval.cwiseAbs();
  int minInd;
  eval.minCoeff(&minInd);
  Point3<S> d;
  d[0] = evec(0,minInd);
  d[1] = evec(1,minInd);
  d[2] = evec(2,minInd);

  plane.Init(b,d);
}

} // end namespace

#endif
