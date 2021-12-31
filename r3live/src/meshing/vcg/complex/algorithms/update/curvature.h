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

#ifndef VCGLIB_UPDATE_CURVATURE_
#define VCGLIB_UPDATE_CURVATURE_

#include <vcg/space/index/grid_static_ptr.h>
#include <vcg/simplex/face/topology.h>
#include <vcg/simplex/face/pos.h>
#include <vcg/simplex/face/jumping_pos.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/point_sampling.h>
#include <vcg/complex/algorithms/intersection.h>
#include <vcg/complex/algorithms/inertia.h>
#include <eigenlib/Eigen/Core>

namespace vcg {
namespace tri {

/// \ingroup trimesh

/// \headerfile curvature.h vcg/complex/algorithms/update/curvature.h

/// \brief Management, updating and computation of per-vertex and per-face normals.
/**
This class is used to compute or update the normals that can be stored in the vertex or face component of a mesh.
*/

template <class MeshType>
class UpdateCurvature
{

public:
    typedef typename MeshType::FaceType FaceType;
    typedef typename MeshType::FacePointer FacePointer;
    typedef typename MeshType::FaceIterator FaceIterator;
    typedef typename MeshType::VertexIterator VertexIterator;
    typedef typename MeshType::VertContainer VertContainer;
    typedef typename MeshType::VertexType VertexType;
    typedef typename MeshType::VertexPointer VertexPointer;
    typedef vcg::face::VFIterator<FaceType> VFIteratorType;
    typedef typename MeshType::CoordType CoordType;
    typedef typename CoordType::ScalarType ScalarType;


private:
  // aux data struct used by PrincipalDirections
  struct AdjVertex {
    VertexType * vert;
    float doubleArea;
    bool isBorder;
  };


public:
    /// \brief Compute principal direction and magnitudo of curvature.

/*
    Compute principal direction and magniuto of curvature as describe in the paper:
    @InProceedings{bb33922,
  author =	"G. Taubin",
  title =	"Estimating the Tensor of Curvature of a Surface from a
         Polyhedral Approximation",
  booktitle =	"International Conference on Computer Vision",
  year = 	"1995",
  pages =	"902--907",
  URL =  	"http://dx.doi.org/10.1109/ICCV.1995.466840",
  bibsource =	"http://www.visionbib.com/bibliography/describe440.html#TT32253",
    */
  static void PrincipalDirections(MeshType &m)
  {
    tri::RequireVFAdjacency(m);
    vcg::tri::UpdateNormal<MeshType>::PerVertexAngleWeighted(m);
    vcg::tri::UpdateNormal<MeshType>::NormalizePerVertex(m);

    for (VertexIterator vi =m.vert.begin(); vi !=m.vert.end(); ++vi) {
      if ( ! (*vi).IsD() && (*vi).VFp() != NULL) {

        VertexType * central_vertex = &(*vi);

        std::vector<float> weights;
        std::vector<AdjVertex> vertices;

        vcg::face::JumpingPos<FaceType> pos((*vi).VFp(), central_vertex);

        // firstV is the first vertex of the 1ring neighboorhood
        VertexType* firstV = pos.VFlip();
        VertexType* tempV;
        float totalDoubleAreaSize = 0.0f;

        // compute the area of each triangle around the central vertex as well as their total area
        do
        {
          // this bring the pos to the next triangle  counterclock-wise
          pos.FlipF();
          pos.FlipE();

          // tempV takes the next vertex in the 1ring neighborhood
          tempV = pos.VFlip();
          assert(tempV!=central_vertex);
          AdjVertex v;

          v.isBorder = pos.IsBorder();
          v.vert = tempV;
          v.doubleArea = vcg::DoubleArea(*pos.F());
          totalDoubleAreaSize += v.doubleArea;

          vertices.push_back(v);
        }
        while(tempV != firstV);

        // compute the weights for the formula computing matrix M
        for (size_t i = 0; i < vertices.size(); ++i) {
          if (vertices[i].isBorder) {
            weights.push_back(vertices[i].doubleArea / totalDoubleAreaSize);
          } else {
            weights.push_back(0.5f * (vertices[i].doubleArea + vertices[(i-1)%vertices.size()].doubleArea) / totalDoubleAreaSize);
          }
          assert(weights.back() < 1.0f);
        }

        // compute I-NN^t to be used for computing the T_i's
        Matrix33<ScalarType> Tp;
        for (int i = 0; i < 3; ++i)
          Tp[i][i] = 1.0f - powf(central_vertex->cN()[i],2);
        Tp[0][1] = Tp[1][0] = -1.0f * (central_vertex->N()[0] * central_vertex->cN()[1]);
        Tp[1][2] = Tp[2][1] = -1.0f * (central_vertex->cN()[1] * central_vertex->cN()[2]);
        Tp[0][2] = Tp[2][0] = -1.0f * (central_vertex->cN()[0] * central_vertex->cN()[2]);

        // for all neighbors vi compute the directional curvatures k_i and the T_i
        // compute M by summing all w_i k_i T_i T_i^t
        Matrix33<ScalarType> tempMatrix;
        Matrix33<ScalarType> M;
        M.SetZero();
        for (size_t i = 0; i < vertices.size(); ++i) {
          CoordType edge = (central_vertex->cP() - vertices[i].vert->cP());
          float curvature = (2.0f * (central_vertex->cN().dot(edge)) ) / edge.SquaredNorm();
          CoordType T = (Tp*edge).normalized();
          tempMatrix.ExternalProduct(T,T);
          M += tempMatrix * weights[i] * curvature ;
        }

        // compute vector W for the Householder matrix
        CoordType W;
        CoordType e1(1.0f,0.0f,0.0f);
        if ((e1 - central_vertex->cN()).SquaredNorm() > (e1 + central_vertex->cN()).SquaredNorm())
          W = e1 - central_vertex->cN();
        else
          W = e1 + central_vertex->cN();
        W.Normalize();

        // compute the Householder matrix I - 2WW^t
        Matrix33<ScalarType> Q;
        Q.SetIdentity();
        tempMatrix.ExternalProduct(W,W);
        Q -= tempMatrix * 2.0f;

        // compute matrix Q^t M Q
        Matrix33<ScalarType> QtMQ = (Q.transpose() * M * Q);

//        CoordType bl = Q.GetColumn(0);
        CoordType T1 = Q.GetColumn(1);
        CoordType T2 = Q.GetColumn(2);

        // find sin and cos for the Givens rotation
        float s,c;
        // Gabriel Taubin hint and Valentino Fiorin impementation
        float alpha = QtMQ[1][1]-QtMQ[2][2];
        float beta  = QtMQ[2][1];

        float h[2];
        float delta = sqrtf(4.0f*powf(alpha, 2) +16.0f*powf(beta, 2));
        h[0] = (2.0f*alpha + delta) / (2.0f*beta);
        h[1] = (2.0f*alpha - delta) / (2.0f*beta);

        float t[2];
        float best_c, best_s;
        float min_error = std::numeric_limits<ScalarType>::infinity();
        for (int i=0; i<2; i++)
        {
          delta = sqrtf(powf(h[i], 2) + 4.0f);
          t[0] = (h[i]+delta) / 2.0f;
          t[1] = (h[i]-delta) / 2.0f;

          for (int j=0; j<2; j++)
          {
            float squared_t = powf(t[j], 2);
            float denominator = 1.0f + squared_t;
            s = (2.0f*t[j])		/ denominator;
            c = (1-squared_t) / denominator;

            float approximation = c*s*alpha + (powf(c, 2) - powf(s, 2))*beta;
            float angle_similarity = fabs(acosf(c)/asinf(s));
            float error = fabs(1.0f-angle_similarity)+fabs(approximation);
            if (error<min_error)
            {
              min_error = error;
              best_c = c;
              best_s = s;
            }
          }
        }
        c = best_c;
        s = best_s;

        Eigen::Matrix2f minor2x2;
        Eigen::Matrix2f S;


        // diagonalize M
        minor2x2(0,0) = QtMQ[1][1];
        minor2x2(0,1) = QtMQ[1][2];
        minor2x2(1,0) = QtMQ[2][1];
        minor2x2(1,1) = QtMQ[2][2];

        S(0,0) = S(1,1) = c;
        S(0,1) = s;
        S(1,0) = -1.0f * s;

        Eigen::Matrix2f StMS = S.transpose() * minor2x2 * S;

        // compute curvatures and curvature directions
        float Principal_Curvature1 = (3.0f * StMS(0,0)) - StMS(1,1);
        float Principal_Curvature2 = (3.0f * StMS(1,1)) - StMS(0,0);

        CoordType Principal_Direction1 = T1 * c - T2 * s;
        CoordType Principal_Direction2 = T1 * s + T2 * c;

        (*vi).PD1() = Principal_Direction1;
        (*vi).PD2() = Principal_Direction2;
        (*vi).K1() =  Principal_Curvature1;
        (*vi).K2() =  Principal_Curvature2;
      }
    }
  }




  class AreaData
  {
  public:
    float A;
  };

  /** Curvature meseaure as described in the paper:
Robust principal curvatures on Multiple Scales, Yong-Liang Yang, Yu-Kun Lai, Shi-Min Hu Helmut Pottmann
SGP 2004
If pointVSfaceInt==true the covariance is computed by montecarlo sampling on the mesh (faster)
If pointVSfaceInt==false the covariance is computed by (analytic)integration over the surface (slower)
*/

  typedef vcg::GridStaticPtr	<FaceType, ScalarType >		MeshGridType;
  typedef vcg::GridStaticPtr	<VertexType, ScalarType >		PointsGridType;

  static void PrincipalDirectionsPCA(MeshType &m, ScalarType r, bool pointVSfaceInt = true,vcg::CallBackPos * cb = NULL)
  {
    std::vector<VertexType*> closests;
    std::vector<ScalarType> distances;
    std::vector<CoordType> points;
    VertexIterator vi;
    ScalarType area;
    MeshType tmpM;
    typename std::vector<CoordType>::iterator ii;
    vcg::tri::TrivialSampler<MeshType> vs;
    tri::UpdateNormal<MeshType>::PerVertexAngleWeighted(m);
    tri::UpdateNormal<MeshType>::NormalizePerVertex(m);

    MeshGridType mGrid;
    PointsGridType pGrid;

    // Fill the grid used
    if(pointVSfaceInt)
    {
      area = Stat<MeshType>::ComputeMeshArea(m);
      vcg::tri::SurfaceSampling<MeshType,vcg::tri::TrivialSampler<MeshType> >::Montecarlo(m,vs,1000 * area / (2*M_PI*r*r ));
      vi = vcg::tri::Allocator<MeshType>::AddVertices(tmpM,m.vert.size());
      for(size_t y  = 0; y <  m.vert.size(); ++y,++vi)  (*vi).P() =  m.vert[y].P();
      pGrid.Set(tmpM.vert.begin(),tmpM.vert.end());
    }
    else
    {
      mGrid.Set(m.face.begin(),m.face.end());
    }

    int jj = 0;
    for(vi  = m.vert.begin(); vi != m.vert.end(); ++vi)
    {
      vcg::Matrix33<ScalarType> A, eigenvectors;
      vcg::Point3<ScalarType> bp, eigenvalues;
//      int nrot;

      // sample the neighborhood
      if(pointVSfaceInt)
      {
        vcg::tri::GetInSphereVertex<
            MeshType,
            PointsGridType,std::vector<VertexType*>,
            std::vector<ScalarType>,
            std::vector<CoordType> >(tmpM,pGrid,  (*vi).cP(),r ,closests,distances,points);

        A.Covariance(points,bp);
        A*=area*area/1000;
      }
      else{
        IntersectionBallMesh<MeshType,ScalarType>( m ,vcg::Sphere3<ScalarType>((*vi).cP(),r),tmpM );
        vcg::Point3<ScalarType> _bary;
        vcg::tri::Inertia<MeshType>::Covariance(tmpM,_bary,A);
      }

//      Eigen::Matrix3f AA; AA=A;
//      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(AA);
//      Eigen::Vector3f c_val = eig.eigenvalues();
//      Eigen::Matrix3f c_vec = eig.eigenvectors();

//      Jacobi(A,  eigenvalues , eigenvectors, nrot);

      Eigen::Matrix3d AA;
      A.ToEigenMatrix(AA);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(AA);
      Eigen::Vector3d c_val = eig.eigenvalues();
      Eigen::Matrix3d c_vec = eig.eigenvectors(); // eigenvector are stored as columns.
      eigenvectors.FromEigenMatrix(c_vec);
      eigenvalues.FromEigenVector(c_val);
//	  EV.transposeInPlace();
//	  ev.FromEigenVector(c_val);

      // get the estimate of curvatures from eigenvalues and eigenvectors
      // find the 2 most tangent eigenvectors (by finding the one closest to the normal)
      int best = 0; ScalarType bestv = fabs( (*vi).cN().dot(eigenvectors.GetColumn(0).normalized()) );
      for(int i  = 1 ; i < 3; ++i){
        ScalarType prod = fabs((*vi).cN().dot(eigenvectors.GetColumn(i).normalized()));
        if( prod > bestv){bestv = prod; best = i;}
      }

      (*vi).PD1()  = eigenvectors.GetColumn( (best+1)%3).normalized();
      (*vi).PD2()  = eigenvectors.GetColumn( (best+2)%3).normalized();

      // project them to the plane identified by the normal
      vcg::Matrix33<ScalarType> rot;
      ScalarType angle = acos((*vi).PD1().dot((*vi).N()));
      rot.SetRotateRad(  - (M_PI*0.5 - angle),(*vi).PD1()^(*vi).N());
      (*vi).PD1() = rot*(*vi).PD1();
      angle = acos((*vi).PD2().dot((*vi).N()));
      rot.SetRotateRad(  - (M_PI*0.5 - angle),(*vi).PD2()^(*vi).N());
      (*vi).PD2() = rot*(*vi).PD2();


      // copmutes the curvature values
      const ScalarType r5 = r*r*r*r*r;
      const ScalarType r6 = r*r5;
      (*vi).K1() = (2.0/5.0) * (4.0*M_PI*r5 + 15*eigenvalues[(best+2)%3]-45.0*eigenvalues[(best+1)%3])/(M_PI*r6);
      (*vi).K2() = (2.0/5.0) * (4.0*M_PI*r5 + 15*eigenvalues[(best+1)%3]-45.0*eigenvalues[(best+2)%3])/(M_PI*r6);
      if((*vi).K1() < (*vi).K2())	{	std::swap((*vi).K1(),(*vi).K2());
        std::swap((*vi).PD1(),(*vi).PD2());
        if (cb)
        {
          (*cb)(int(100.0f * (float)jj / (float)m.vn),"Vertices Analysis");
          ++jj;
        }													}
    }


  }

/// \brief Computes the discrete mean gaussian curvature.
/**
The algorithm used is the one Desbrun et al. that is based on a discrete analysis of the angles of the faces around a vertex.

It requires FaceFace Adjacency;

For further details, please, refer to: \n

<b>Discrete Differential-Geometry Operators for Triangulated 2-Manifolds </b><br>
<i>Mark Meyer, Mathieu Desbrun, Peter Schroder, Alan H. Barr</i><br>
 VisMath '02, Berlin
*/
static void MeanAndGaussian(MeshType & m)
{
  tri::RequireFFAdjacency(m);

  float area0, area1, area2, angle0, angle1, angle2;
  FaceIterator fi;
  VertexIterator vi;
  typename MeshType::CoordType  e01v ,e12v ,e20v;

  SimpleTempData<VertContainer, AreaData> TDAreaPtr(m.vert);
  SimpleTempData<VertContainer, typename MeshType::CoordType> TDContr(m.vert);

  vcg::tri::UpdateNormal<MeshType>::PerVertexNormalized(m);
  //Compute AreaMix in H (vale anche per K)
  for(vi=m.vert.begin(); vi!=m.vert.end(); ++vi) if(!(*vi).IsD())
  {
    (TDAreaPtr)[*vi].A = 0.0;
    (TDContr)[*vi]  =typename MeshType::CoordType(0.0,0.0,0.0);
    (*vi).Kh() = 0.0;
    (*vi).Kg() = (float)(2.0 * M_PI);
  }

  for(fi=m.face.begin();fi!=m.face.end();++fi) if( !(*fi).IsD())
  {
    // angles
    angle0 = math::Abs(Angle(	(*fi).P(1)-(*fi).P(0),(*fi).P(2)-(*fi).P(0) ));
    angle1 = math::Abs(Angle(	(*fi).P(0)-(*fi).P(1),(*fi).P(2)-(*fi).P(1) ));
    angle2 = M_PI-(angle0+angle1);

    if((angle0 < M_PI/2) && (angle1 < M_PI/2) && (angle2 < M_PI/2))  // triangolo non ottuso
    {
      float e01 = SquaredDistance( (*fi).V(1)->cP() , (*fi).V(0)->cP() );
      float e12 = SquaredDistance( (*fi).V(2)->cP() , (*fi).V(1)->cP() );
      float e20 = SquaredDistance( (*fi).V(0)->cP() , (*fi).V(2)->cP() );

      area0 = ( e20*(1.0/tan(angle1)) + e01*(1.0/tan(angle2)) ) / 8.0;
      area1 = ( e01*(1.0/tan(angle2)) + e12*(1.0/tan(angle0)) ) / 8.0;
      area2 = ( e12*(1.0/tan(angle0)) + e20*(1.0/tan(angle1)) ) / 8.0;

      (TDAreaPtr)[(*fi).V(0)].A  += area0;
      (TDAreaPtr)[(*fi).V(1)].A  += area1;
      (TDAreaPtr)[(*fi).V(2)].A  += area2;

    }
    else // obtuse
    {
      if(angle0 >= M_PI/2)
      {
        (TDAreaPtr)[(*fi).V(0)].A += vcg::DoubleArea<typename MeshType::FaceType>((*fi)) / 4.0;
        (TDAreaPtr)[(*fi).V(1)].A += vcg::DoubleArea<typename MeshType::FaceType>((*fi)) / 8.0;
        (TDAreaPtr)[(*fi).V(2)].A += vcg::DoubleArea<typename MeshType::FaceType>((*fi)) / 8.0;
      }
      else if(angle1 >= M_PI/2)
      {
        (TDAreaPtr)[(*fi).V(0)].A += vcg::DoubleArea<typename MeshType::FaceType>((*fi)) / 8.0;
        (TDAreaPtr)[(*fi).V(1)].A += vcg::DoubleArea<typename MeshType::FaceType>((*fi)) / 4.0;
        (TDAreaPtr)[(*fi).V(2)].A += vcg::DoubleArea<typename MeshType::FaceType>((*fi)) / 8.0;
      }
      else
      {
        (TDAreaPtr)[(*fi).V(0)].A += vcg::DoubleArea<typename MeshType::FaceType>((*fi)) / 8.0;
        (TDAreaPtr)[(*fi).V(1)].A += vcg::DoubleArea<typename MeshType::FaceType>((*fi)) / 8.0;
        (TDAreaPtr)[(*fi).V(2)].A += vcg::DoubleArea<typename MeshType::FaceType>((*fi)) / 4.0;
      }
    }
  }


  for(fi=m.face.begin();fi!=m.face.end();++fi) if( !(*fi).IsD() )
  {
    angle0 = math::Abs(Angle(	(*fi).P(1)-(*fi).P(0),(*fi).P(2)-(*fi).P(0) ));
    angle1 = math::Abs(Angle(	(*fi).P(0)-(*fi).P(1),(*fi).P(2)-(*fi).P(1) ));
    angle2 = M_PI-(angle0+angle1);

    // Skip degenerate triangles.
    if(angle0==0 || angle1==0 || angle1==0) continue;

    e01v = ( (*fi).V(1)->cP() - (*fi).V(0)->cP() ) ;
    e12v = ( (*fi).V(2)->cP() - (*fi).V(1)->cP() ) ;
    e20v = ( (*fi).V(0)->cP() - (*fi).V(2)->cP() ) ;

    TDContr[(*fi).V(0)] += ( e20v * (1.0/tan(angle1)) - e01v * (1.0/tan(angle2)) ) / 4.0;
    TDContr[(*fi).V(1)] += ( e01v * (1.0/tan(angle2)) - e12v * (1.0/tan(angle0)) ) / 4.0;
    TDContr[(*fi).V(2)] += ( e12v * (1.0/tan(angle0)) - e20v * (1.0/tan(angle1)) ) / 4.0;

    (*fi).V(0)->Kg() -= angle0;
    (*fi).V(1)->Kg() -= angle1;
    (*fi).V(2)->Kg() -= angle2;


    for(int i=0;i<3;i++)
    {
      if(vcg::face::IsBorder((*fi), i))
      {
        CoordType e1,e2;
        vcg::face::Pos<FaceType> hp(&*fi, i, (*fi).V(i));
        vcg::face::Pos<FaceType> hp1=hp;

        hp1.FlipV();
        e1=hp1.v->cP() - hp.v->cP();
        hp1.FlipV();
        hp1.NextB();
        e2=hp1.v->cP() - hp.v->cP();
        (*fi).V(i)->Kg() -= math::Abs(Angle(e1,e2));
      }
    }
  }

  for(vi=m.vert.begin(); vi!=m.vert.end(); ++vi) if(!(*vi).IsD() /*&& !(*vi).IsB()*/)
  {
    if((TDAreaPtr)[*vi].A<=std::numeric_limits<ScalarType>::epsilon())
    {
      (*vi).Kh() = 0;
      (*vi).Kg() = 0;
    }
    else
    {
      (*vi).Kh()  = (((TDContr)[*vi].dot((*vi).cN())>0)?1.0:-1.0)*((TDContr)[*vi] / (TDAreaPtr) [*vi].A).Norm();
      (*vi).Kg() /= (TDAreaPtr)[*vi].A;
    }
  }
}


    /// \brief Update the mean and the gaussian curvature of a vertex.

    /**
    The function uses the VF adiacency to walk around the vertex.
    \return It will return the voronoi area around the vertex.  If (norm == true) the mean and the gaussian curvature are normalized.
     Based on the paper  <a href="http://www2.in.tu-clausthal.de/~hormann/papers/Dyn.2001.OTU.pdf">  <em> "Optimizing 3d triangulations using discrete curvature analysis" </em> </a>
      */

    static float ComputeSingleVertexCurvature(VertexPointer v, bool norm = true)
    {
        VFIteratorType vfi(v);
        float A = 0;

        v->Kh() = 0;
        v->Kg() = 2 * M_PI;

        while (!vfi.End()) {
            if (!vfi.F()->IsD()) {
                FacePointer f = vfi.F();
                int i = vfi.I();
                VertexPointer v0 = f->V0(i), v1 = f->V1(i), v2 = f->V2(i);

                float ang0 = math::Abs(Angle(v1->P() - v0->P(), v2->P() - v0->P() ));
                float ang1 = math::Abs(Angle(v0->P() - v1->P(), v2->P() - v1->P() ));
                float ang2 = M_PI - ang0 - ang1;

                float s01 = SquaredDistance(v1->P(), v0->P());
                float s02 = SquaredDistance(v2->P(), v0->P());

                // voronoi cell of current vertex
                if (ang0 >= M_PI/2)
                    A += (0.5f * DoubleArea(*f) - (s01 * tan(ang1) + s02 * tan(ang2)) / 8.0 );
                else if (ang1 >= M_PI/2)
                    A += (s01 * tan(ang0)) / 8.0;
                else if (ang2 >= M_PI/2)
                    A += (s02 * tan(ang0)) / 8.0;
                else  // non obctuse triangle
                    A += ((s02 / tan(ang1)) + (s01 / tan(ang2))) / 8.0;

                // gaussian curvature update
                v->Kg() -= ang0;

                // mean curvature update
                ang1 = math::Abs(Angle(f->N(), v1->N()));
                ang2 = math::Abs(Angle(f->N(), v2->N()));
                v->Kh() += ( (math::Sqrt(s01) / 2.0) * ang1 +
                             (math::Sqrt(s02) / 2.0) * ang2 );
            }

            ++vfi;
        }

        v->Kh() /= 4.0f;

        if(norm) {
            if(A <= std::numeric_limits<float>::epsilon()) {
                v->Kh() = 0;
                v->Kg() = 0;
            }
            else {
                v->Kh() /= A;
                v->Kg() /= A;
            }
        }

        return A;
    }

    static void PerVertex(MeshType & m)
    {
      tri::RequireVFAdjacency(m);

      for(VertexIterator vi = m.vert.begin(); vi != m.vert.end(); ++vi)
        ComputeSingleVertexCurvature(&*vi,false);
    }



/*
    Compute principal curvature directions and value with normal cycle:
    @inproceedings{CohMor03,
    author = {Cohen-Steiner, David   and Morvan, Jean-Marie  },
    booktitle = {SCG '03: Proceedings of the nineteenth annual symposium on Computational geometry},
    title - {Restricted delaunay triangulations and normal cycle}
    year = {2003}
}
    */

    static void PrincipalDirectionsNormalCycle(MeshType & m){
      tri::RequireVFAdjacency(m);
      tri::RequireFFAdjacency(m);
      tri::RequirePerFaceNormal(m);

        typename MeshType::VertexIterator vi;

        for(vi = m.vert.begin(); vi != m.vert.end(); ++vi)
        if(!((*vi).IsD())){
            vcg::Matrix33<ScalarType> m33;m33.SetZero();
            face::JumpingPos<typename MeshType::FaceType> p((*vi).VFp(),&(*vi));
            p.FlipE();
            typename MeshType::VertexType * firstv = p.VFlip();
            assert(p.F()->V(p.VInd())==&(*vi));

            do{
                if( p.F() != p.FFlip()){
                    Point3<ScalarType> normalized_edge = p.F()->V(p.F()->Next(p.VInd()))->cP() - (*vi).P();
                    ScalarType edge_length = normalized_edge.Norm();
                    normalized_edge/=edge_length;
                    Point3<ScalarType> n1 = p.F()->cN();n1.Normalize();
                    Point3<ScalarType> n2 = p.FFlip()->cN();n2.Normalize();
                    ScalarType n1n2 = (n1 ^ n2).dot(normalized_edge);
                    n1n2 = std::max(std::min( ScalarType(1.0),n1n2),ScalarType(-1.0));
                    ScalarType beta = math::Asin(n1n2);
                    m33[0][0] += beta*edge_length*normalized_edge[0]*normalized_edge[0];
                    m33[0][1] += beta*edge_length*normalized_edge[1]*normalized_edge[0];
                    m33[1][1] += beta*edge_length*normalized_edge[1]*normalized_edge[1];
                    m33[0][2] += beta*edge_length*normalized_edge[2]*normalized_edge[0];
                    m33[1][2] += beta*edge_length*normalized_edge[2]*normalized_edge[1];
                    m33[2][2] += beta*edge_length*normalized_edge[2]*normalized_edge[2];
                }
                p.NextFE();
            }while(firstv != p.VFlip());

            if(m33.Determinant()==0.0){ // degenerate case
                (*vi).K1() = (*vi).K2() = 0.0; continue;}

            m33[1][0] = m33[0][1];
            m33[2][0] = m33[0][2];
            m33[2][1] = m33[1][2];

            Eigen::Matrix3d it;
            m33.ToEigenMatrix(it);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(it);
            Eigen::Vector3d c_val = eig.eigenvalues();
            Eigen::Matrix3d c_vec = eig.eigenvectors();

            Point3<ScalarType> lambda;
            Matrix33<ScalarType> vect;
            vect.FromEigenMatrix(c_vec);
            lambda.FromEigenVector(c_val);

            ScalarType bestNormal = 0;
            int bestNormalIndex = -1;
            for(int i = 0; i < 3; ++i)
            {
              float agreeWithNormal =  fabs((*vi).N().Normalize().dot(vect.GetColumn(i)));
                if( agreeWithNormal > bestNormal )
                {
                    bestNormal= agreeWithNormal;
                    bestNormalIndex = i;
                }
            }
            int maxI = (bestNormalIndex+2)%3;
            int minI = (bestNormalIndex+1)%3;
            if(fabs(lambda[maxI]) < fabs(lambda[minI])) std::swap(maxI,minI);

            (*vi).PD1() = *(Point3<ScalarType>*)(& vect[maxI][0]);
            (*vi).PD2() = *(Point3<ScalarType>*)(& vect[minI][0]);
            (*vi).K1() = lambda[2];
            (*vi).K2() = lambda[1];
        }
    }

    static void PerVertexBasicRadialCrossField(MeshType &m, float anisotropyRatio = 1.0 )
    {
      tri::RequirePerVertexCurvatureDir(m);
      CoordType c=m.bbox.Center();
      float maxRad = m.bbox.Diag()/2.0f;

      for(int i=0;i<m.vert.size();++i) {
        CoordType dd = m.vert[i].P()-c;
        dd.Normalize();
        m.vert[i].PD1()=dd^m.vert[i].N();
        m.vert[i].PD1().Normalize();
        m.vert[i].PD2()=m.vert[i].N()^m.vert[i].PD1();
        m.vert[i].PD2().Normalize();
        // Now the anisotropy
        // the idea is that the ratio between the two direction is at most <anisotropyRatio>
        // eg  |PD1|/|PD2| < ratio
        // and |PD1|^2 + |PD2|^2 == 1

        float q =Distance(m.vert[i].P(),c) / maxRad;  // it is in the 0..1 range
        const float minRatio = 1.0f/anisotropyRatio;
        const float maxRatio = anisotropyRatio;
        const float curRatio = minRatio + (maxRatio-minRatio)*q;
        float pd1Len = sqrt(1.0/(1+curRatio*curRatio));
        float pd2Len = curRatio * pd1Len;
        assert(fabs(curRatio - pd2Len/pd1Len)<0.0000001);
        assert(fabs(pd1Len*pd1Len + pd2Len*pd2Len - 1.0f)<0.0001);
        m.vert[i].PD1() *= pd1Len;
        m.vert[i].PD2() *= pd2Len;
      }
    }

};

} // end namespace tri
} // end namespace vcg
#endif
