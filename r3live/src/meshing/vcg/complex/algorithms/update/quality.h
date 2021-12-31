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
#ifndef __VCG_TRI_UPDATE_QUALITY
#define __VCG_TRI_UPDATE_QUALITY
#include <vcg/simplex/face/pos.h>
#include <vcg/simplex/face/topology.h>
#include <vcg/complex/algorithms/update/flag.h>
#include <vcg/complex/algorithms/stat.h>

namespace vcg {
namespace tri {
/// \ingroup trimesh

/// \headerfile quality.h vcg/complex/algorithms/update/quality.h

/// \brief Generation of per-vertex and per-face qualities.
/**
 It works according to various strategy, like geodesic distance from the border (UpdateQuality::VertexGeodesicFromBorder) or curvature ecc.
 This class is templated over the mesh and (like all other Update* classes) has only static members; Typical usage:
\code
MyMeshType m;
UpdateQuality<MyMeshType>::VertexGeodesicFromBorder(m);
\endcode
*/

template <class UpdateMeshType>
class UpdateQuality
{
public:
  typedef UpdateMeshType MeshType;
  typedef typename MeshType::ScalarType     ScalarType;
  typedef typename MeshType::VertexType     VertexType;
  typedef typename MeshType::VertexPointer  VertexPointer;
  typedef typename MeshType::VertexIterator VertexIterator;
  typedef typename MeshType::FaceType       FaceType;
  typedef typename MeshType::FacePointer    FacePointer;
  typedef typename MeshType::FaceIterator   FaceIterator;


/** Assign to each vertex of the mesh a constant quality value. Useful for initialization.
*/
static void VertexConstant(MeshType &m, float q)
{
  tri::RequirePerVertexQuality(m);
  for(VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi) if(!(*vi).IsD())
    (*vi).Q()=q;
}

/** Clamp each vertex of the mesh with a range of values.
*/
static void VertexClamp(MeshType &m, float qmin, float qmax)
{
  tri::RequirePerVertexQuality(m);
  for(VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi) if(!(*vi).IsD())
    (*vi).Q()=std::min(qmax, std::max(qmin,(*vi).Q()));
}

/** Normalize the vertex quality so that it fits in the specified range.
*/
static void VertexNormalize(MeshType &m, float qmin=0.0, float qmax=1.0)
{
  tri::RequirePerVertexQuality(m);
  ScalarType deltaRange = qmax-qmin;
  std::pair<ScalarType,ScalarType> minmax = tri::Stat<MeshType>::ComputePerVertexQualityMinMax(m);
  VertexIterator vi;
  for(vi = m.vert.begin(); vi != m.vert.end(); ++vi)
    (*vi).Q() = qmin+deltaRange*((*vi).Q() - minmax.first)/(minmax.second - minmax.first);
}

/** Normalize the face quality so that it fits in the specified range.
*/
static void FaceNormalize(MeshType &m, float qmin=0.0, float qmax=1.0)
{
  tri::RequirePerFaceQuality(m);
  ScalarType deltaRange = qmax-qmin;
  std::pair<ScalarType,ScalarType> minmax = tri::Stat<MeshType>::ComputePerFaceQualityMinMax(m);
  for(FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
    (*fi).Q() = qmin+deltaRange*((*fi).Q() - minmax.first)/(minmax.second - minmax.first);
}

/** Assign to each face of the mesh a constant quality value. Useful for initialization.
*/
static void FaceConstant(MeshType &m, float q)
{
  tri::RequirePerFaceQuality(m);
  for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi)
    (*fi).Q()=q;
}

/** Assign to each face of the mesh its area.
*/
static void FaceArea(MeshType &m)
{
  tri::RequirePerFaceQuality(m);
  for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi)
    (*fi).Q()=vcg::DoubleArea(*fi)/ScalarType(2.0);
}

static void VertexFromFace( MeshType &m, bool areaWeighted=true)
{
  tri::RequirePerFaceQuality(m);
  tri::RequirePerVertexQuality(m);
  SimpleTempData<typename MeshType::VertContainer, ScalarType> TQ(m.vert,0);
  SimpleTempData<typename MeshType::VertContainer, ScalarType> TCnt(m.vert,0);

  for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi)
    if(!(*fi).IsD())
    {
      ScalarType weight=1.0;
      if(areaWeighted) weight = vcg::DoubleArea(*fi);
      for(int j=0;j<3;++j)
      {
        TQ[(*fi).V(j)]+=(*fi).Q()*weight;
        TCnt[(*fi).V(j)]+=weight;
      }
    }

  for(VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi)
    if(!(*vi).IsD() && TCnt[*vi]>0 )
    {
      (*vi).Q() = TQ[*vi] / TCnt[*vi];
    }
}

static void FaceFromVertex( MeshType &m)
{
  tri::RequirePerFaceQuality(m);
  tri::RequirePerVertexQuality(m);
  for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi) if(!(*fi).IsD())
  {
    (*fi).Q() = ((*fi).V(0)->Q()+(*fi).V(1)->Q()+(*fi).V(2)->Q())/3.0f;
  }
}

static void VertexFromPlane(MeshType &m, const Plane3<ScalarType> &pl)
{
  for(VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi) if(!(*vi).IsD())
    (*vi).Q() =SignedDistancePlanePoint(pl,(*vi).cP());
}

static void VertexFromGaussianCurvatureHG(MeshType &m)
{
  tri::RequirePerVertexQuality(m);
  tri::RequirePerVertexCurvature(m);
    for(VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi) if(!(*vi).IsD())
        (*vi).Q() = (*vi).Kg();
}

static void VertexFromMeanCurvatureHG(MeshType &m)
{
  tri::RequirePerVertexQuality(m);
  tri::RequirePerVertexCurvature(m);
    for(VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi) if(!(*vi).IsD())
        (*vi).Q() = (*vi).Kh();
}

static void VertexFromGaussianCurvatureDir(MeshType &m)
{
  tri::RequirePerVertexQuality(m);
  tri::RequirePerVertexCurvatureDir(m);
    for(VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi) if(!(*vi).IsD())
      (*vi).Q() = (*vi).K1()*(*vi).K2();
}

static void VertexFromMeanCurvatureDir(MeshType &m)
{
  tri::RequirePerVertexQuality(m);
  tri::RequirePerVertexCurvatureDir(m);
    for(VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi) if(!(*vi).IsD())
        (*vi).Q() = ((*vi).K1()+(*vi).K2())/2.0f;
}

/*
 *  Absolute Curvature
 *
 *                  2|H|                if K >= 0
 *  |k1| + |k2| = <
 *                  2 * sqrt(|H|^2-K)   otherwise
 *
 * defs and formulas taken from
 *
 * Improved curvature estimation for watershed segmentation of 3-dimensional meshes
 * S Pulla, A Razdan, G Farin - Arizona State University, Tech. Rep, 2001
 * and from
 * Optimizing 3D triangulations using discrete curvature analysis
 * N Dyn, K Hormann, SJ Kim, D Levin - Mathematical Methods for Curves and Surfaces: Oslo, 2000
 */

static void VertexFromAbsoluteCurvature(MeshType &m)
{
    VertexIterator vi;
    for(vi=m.vert.begin();vi!=m.vert.end();++vi) if(!(*vi).IsD())
    {
        if((*vi).Kg() >= 0)
                    (*vi).Q() = math::Abs( 2*(*vi).Kh() );
        else
              (*vi).Q() = 2*math::Sqrt(math::Abs( (*vi).Kh()*(*vi).Kh() - (*vi).Kg()));
    }
}

/*
 * RMS Curvature =   sqrt(4H^2-2K)
 * def and formula taken from
 *
 * Improved curvature estimation for watershed segmentation of 3-dimensional meshes
 * S Pulla, A Razdan, G Farin - Arizona State University, Tech. Rep, 2001
 */
static void VertexFromRMSCurvature(MeshType &m)
{
    VertexIterator vi;
    for(vi=m.vert.begin();vi!=m.vert.end();++vi) if(!(*vi).IsD())
        (*vi).Q() = math::Sqrt(math::Abs( 4*(*vi).Kh()*(*vi).Kh() - 2*(*vi).Kg()));
}

/*
  Saturate the Face quality so that for each vertex the gradient of the quality is lower than the given threshold value (in absolute value)
  The saturation is done in a conservative way (quality is always decreased and never increased)

  Note: requires FF adjacency.
  */
static void FaceSaturate(MeshType &m, ScalarType gradientThr=1.0)
{
  typedef typename MeshType::CoordType CoordType;
  UpdateFlags<MeshType>::FaceClearV(m);
  std::stack<FacePointer> st;

  st.push(&*m.face.begin());

  while(!st.empty())
    {
     FacePointer fc = st.top();  // the center
     //printf("Stack size %i\n",st.size());
     //printf("Pop elem %i %f\n",st.top() - &*m.vert.begin(), st.top()->Q());
     st.pop();
     fc->SetV();
     std::vector<FacePointer> star;
     typename std::vector<FacePointer>::iterator ffi;
     for (int i=0;i<3;i++)
     {
         FacePointer fnext=fc->FFp(i);
         if (fnext!=fc)star.push_back(fnext);
     }
     CoordType bary0=(fc->P(0)+fc->P(1)+fc->P(2))/3;
     for(ffi=star.begin();ffi!=star.end();++ffi )
     {
       assert(fc!=(*ffi));
       float &qi = (*ffi)->Q();
       CoordType bary1=((*ffi)->P(0)+(*ffi)->P(1)+(*ffi)->P(2))/3;
       float distGeom = Distance(bary0,bary1) / gradientThr;
       // Main test if the quality varies more than the geometric displacement we have to lower something.
       if( distGeom < fabs(qi - fc->Q()))
       {
         // center = 0  other=10 -> other =
         // center = 10 other=0
         if(fc->Q() > qi)  // first case: the center of the star has to be lowered (and re-inserted in the queue).
         {
           //printf("Reinserting center %i \n",vc - &*m.vert.begin());
           fc->Q() = qi+distGeom-(ScalarType)0.00001;
           assert( distGeom > fabs(qi - fc->Q()));
           st.push(fc);
           break;
         }
         else
         {
           // second case: you have to lower qi, the vertex under examination.
           assert( distGeom < fabs(qi - fc->Q()));
           assert(fc->Q() < qi);
           float newQi = fc->Q() + distGeom -(ScalarType)0.00001;
           assert(newQi <= qi);
           assert(fc->Q() < newQi);
           assert( distGeom > fabs(newQi - fc->Q()) );
//             printf("distGeom %f, qi %f, vc->Q() %f, fabs(qi - vc->Q()) %f\n",distGeom,qi,vc->Q(),fabs(qi - vc->Q()));
           qi = newQi;
           (*ffi)->ClearV();
         }
       }
       if(!(*ffi)->IsV())
       {
         st.push( *ffi);
//         printf("Reinserting side %i \n",*vvi - &*m.vert.begin());
         (*ffi)->SetV();
       }
     }
    }
  }

/*
  Saturate the vertex quality so that for each vertex the gradient of the quality is lower than the given threshold value (in absolute value)
  The saturation is done in a conservative way (quality is always decreased and never increased)

  Note: requires VF adjacency.
  */
static void VertexSaturate(MeshType &m, ScalarType gradientThr=1.0)
{
  UpdateFlags<MeshType>::VertexClearV(m);
  std::stack<VertexPointer> st;

  st.push(&*m.vert.begin());

  while(!st.empty())
    {
     VertexPointer vc = st.top();  // the center
     //printf("Stack size %i\n",st.size());
     //printf("Pop elem %i %f\n",st.top() - &*m.vert.begin(), st.top()->Q());
     st.pop();
     vc->SetV();
     std::vector<VertexPointer> star;
     typename std::vector<VertexPointer>::iterator vvi;
     face::VVStarVF<FaceType>(vc,star);
     for(vvi=star.begin();vvi!=star.end();++vvi )
     {
       float &qi = (*vvi)->Q();
       float distGeom = Distance((*vvi)->cP(),vc->cP()) / gradientThr;
       // Main test if the quality varies more than the geometric displacement we have to lower something.
       if( distGeom < fabs(qi - vc->Q()))
       {
         // center = 0  other=10 -> other =
         // center = 10 other=0
         if(vc->Q() > qi)  // first case: the center of the star has to be lowered (and re-inserted in the queue).
         {
           //printf("Reinserting center %i \n",vc - &*m.vert.begin());
           vc->Q() = qi+distGeom-(ScalarType)0.00001;
           assert( distGeom > fabs(qi - vc->Q()));
           st.push(vc);
           break;
         }
         else
         {
           // second case: you have to lower qi, the vertex under examination.
           assert( distGeom < fabs(qi - vc->Q()));
           assert(vc->Q() < qi);
           float newQi = vc->Q() + distGeom -(ScalarType)0.00001;
           assert(newQi <= qi);
           assert(vc->Q() < newQi);
           assert( distGeom > fabs(newQi - vc->Q()) );
//             printf("distGeom %f, qi %f, vc->Q() %f, fabs(qi - vc->Q()) %f\n",distGeom,qi,vc->Q(),fabs(qi - vc->Q()));
           qi = newQi;
           (*vvi)->ClearV();
         }
       }
       if(!(*vvi)->IsV())
       {
         st.push( *vvi);
//         printf("Reinserting side %i \n",*vvi - &*m.vert.begin());
         (*vvi)->SetV();
       }
     }
    }
  }


}; //end class
} // end namespace
} // end namespace
#endif
