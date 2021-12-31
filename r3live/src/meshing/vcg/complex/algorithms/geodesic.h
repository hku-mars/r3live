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
#include <vcg/simplex/face/pos.h>
#include <vcg/simplex/face/topology.h>
#include <vcg/complex/algorithms/update/quality.h>
#include <deque>
#include <functional>
#ifndef __VCGLIB_GEODESIC
#define __VCGLIB_GEODESIC

namespace vcg{
namespace tri{

template <class MeshType>
struct EuclideanDistance{
  typedef typename MeshType::VertexType VertexType;
  typedef typename MeshType::ScalarType  ScalarType;
  typedef typename MeshType::FacePointer FacePointer;

  EuclideanDistance(){}

  ScalarType operator()(const VertexType * v0, const VertexType * v1) const
  {return vcg::Distance(v0->cP(),v1->cP());}

  ScalarType operator()(const FacePointer f0, const FacePointer f1) const
  {return vcg::Distance(Barycenter(*f0),Barycenter(*f1));}
};


template <class MeshType>
class IsotropicDistance{
private:
  // The only member of this class. The attribute handle used to
  typename MeshType::template PerVertexAttributeHandle<float> wH;
public:
  typedef typename MeshType::VertexType VertexType;
  typedef typename MeshType::ScalarType  ScalarType;
  typedef typename MeshType::FacePointer FacePointer;

  /// The constructor reads per vertex quality and transfer it into a per vertex attribute mapping it into the specified range.
  /// The variance parameter specify how the distance is biased by the quality
  /// the distance is scaled by a factor that range from 1/variance to variance according to a linear mapping of quality range.
  ///  So for example if you have a quality distributed in the 0..1 range and you specify a variance of 2 it means
  /// that the distance will be scaled from 0.5 to 2 their original values.

  IsotropicDistance(MeshType &m, float variance)
  {
    // the wH attribute store the scaling factor to be applied to the distance
    wH = tri::Allocator<MeshType>:: template GetPerVertexAttribute<float> (m,"distW");
    float qmin = 1.0f/variance;
    float qmax = variance;
    float qrange = qmax-qmin;
    std::pair<float,float> minmax = Stat<MeshType>::ComputePerVertexQualityMinMax(m);
    float range = minmax.second-minmax.first;
    for(int i=0;i<m.vert.size();++i)
      wH[i]=qmin+((m.vert[i].Q()-minmax.first)/range)*qrange;

//    qDebug("Range %f %f %f",minmax.first,minmax.second,range);
  }

  ScalarType operator()( VertexType * v0,  VertexType * v1)
  {
    float scale = (wH[v0]+wH[v1])/2.0f;
    return (1.0f/scale)*vcg::Distance(v0->cP(),v1->cP());
  }
};


template <class MeshType>
struct BasicCrossFunctor
{
  BasicCrossFunctor(MeshType &m) { tri::RequirePerVertexCurvatureDir(m); }
  typedef typename MeshType::VertexType VertexType;

  Point3f D1(VertexType &v) { return v.PD1(); }
  Point3f D2(VertexType &v) { return v.PD1(); }
};

/**
 * Anisotropic Distance Functor
 *
 * Given a couple of vertexes over the surface (usually an edge)
 * it returns a distance value that is biased according to a tangential cross field.
 * It is assumed that the cross field is smooth enough so that you can safely blend the two directions
 *
 */
template <class MeshType>
class AnisotropicDistance{
  typedef typename MeshType::VertexType VertexType;
  typedef typename MeshType::ScalarType  ScalarType;
  typedef typename MeshType::VertexIterator VertexIterator;

  typename MeshType::template PerVertexAttributeHandle<Point3f> wxH,wyH;
public:
  template <class CrossFunctor > AnisotropicDistance(MeshType &m, CrossFunctor &cf)
  {
    wxH = tri::Allocator<MeshType>:: template GetPerVertexAttribute<Point3f> (m,"distDirX");
    wyH = tri::Allocator<MeshType>:: template GetPerVertexAttribute<Point3f> (m,"distDirY");

    for(VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi)
    {
      wxH[vi]=cf.D1(*vi);
      wyH[vi]=cf.D2(*vi);
    }
   }

  ScalarType operator()( VertexType * v0,  VertexType * v1)
  {
    Point3f dd = v0->cP()-v1->cP();
    float x = (fabs(dd * wxH[v0])+fabs(dd *wxH[v1]))/2.0f;
    float y = (fabs(dd * wyH[v0])+fabs(dd *wyH[v1]))/2.0f;

    return sqrt(x*x+y*y);
  }
};










/*! \brief class for computing approximate geodesic distances on a mesh

  require VF Adjacency relation
\sa trimesh_geodesic.cpp
*/

template <class MeshType>
class Geodesic{

public:

  typedef typename MeshType::VertexType VertexType;
  typedef typename MeshType::VertexIterator VertexIterator;
  typedef typename MeshType::VertexPointer VertexPointer;
  typedef typename MeshType::FacePointer FacePointer;
  typedef typename MeshType::FaceType  FaceType;
  typedef typename MeshType::CoordType  CoordType;
  typedef typename MeshType::ScalarType  ScalarType;



/* Auxiliary class for keeping the heap of vertices to visit and their estimated distance */
  struct VertDist{
    VertDist(){}
    VertDist(VertexPointer _v, ScalarType _d):v(_v),d(_d){}

    VertexPointer v;
    ScalarType d;
  };


  struct DIJKDist{
    DIJKDist(VertexPointer _v):v(_v){}
    VertexPointer v;

    bool operator < (const DIJKDist &o) const
    {
      if( v->Q() != o.v->Q())
        return v->Q() > o.v->Q();
      return v<o.v;
    }
   };

  /* Auxiliary class for keeping the heap of vertices to visit and their estimated distance */
    struct FaceDist{
      FaceDist(FacePointer _f):f(_f){}
      FacePointer f;
      bool operator < (const FaceDist &o) const
      {
        if( f->Q() != o.f->Q())
          return f->Q() > o.f->Q();
        return f<o.f;
      }
    };

  /* Temporary data to associate to all the vertices: estimated distance and boolean flag */
  struct TempData{
    TempData(){}
    TempData(const ScalarType & _d):d(_d),source(0),parent(0){}

    ScalarType d;
    VertexPointer source;//closest source
    VertexPointer parent;
  };

  typedef SimpleTempData<std::vector<VertexType>, TempData >  TempDataType;


  struct pred: public std::binary_function<VertDist,VertDist,bool>{
    pred(){}
    bool operator()(const VertDist& v0, const VertDist& v1) const
    {return (v0.d > v1.d);}
  };

  /*
   *
  curr:   vertex for which distance should be estimated
  d_pw1:  distance of pw1 from the source
  d_curr: distance of curr from the source

The function estimates the distance of pw from the source
in the assumption the mesh is developable (and without holes)
along the path, so that (source,pw1,curr) from a triangle.
All the math is to comput the angles at pw1 and curr with the Erone formula.

The if cases take care of the cases where the angles are obtuse.

              curr
      d_pw1    +
               |      +pw
source+        |
        d_curr +
              pw1

   */
  template <class DistanceFunctor>
  static ScalarType Distance(DistanceFunctor &distFunc,
                             const VertexPointer &pw,
                             const VertexPointer &pw1,
                             const VertexPointer &curr,
                             const ScalarType &d_pw1,
                             const ScalarType &d_curr)
  {
    ScalarType curr_d=0;

    ScalarType ew_c  = distFunc(pw,curr);
    ScalarType ew_w1 = distFunc(pw,pw1);
    ScalarType ec_w1 = distFunc(pw1,curr);
    CoordType w_c =  (pw->cP()-curr->cP()).Normalize() * ew_c;
    CoordType w_w1 = (pw->cP() - pw1->cP()).Normalize() * ew_w1;
    CoordType w1_c =  (pw1->cP() - curr->cP()).Normalize() * ec_w1;

    ScalarType	alpha,alpha_, beta,beta_,theta,h,delta,s,a,b;

    alpha = acos((w_c.dot(w1_c))/(ew_c*ec_w1));
    s = (d_curr + d_pw1+ec_w1)/2;
    a = s/ec_w1;
    b = a*s;
    alpha_ = 2*acos ( std::min<ScalarType>(1.0,sqrt(  (b- a* d_pw1)/d_curr)));

    if ( alpha+alpha_ > M_PI){
      curr_d = d_curr + ew_c;
    }else
    {
      beta_ = 2*acos ( std::min<ScalarType>(1.0,sqrt(  (b- a* d_curr)/d_pw1)));
      beta  = acos((w_w1).dot(-w1_c)/(ew_w1*ec_w1));

      if ( beta+beta_ > M_PI)
        curr_d = d_pw1  + ew_w1;
      else
      {
        theta	= ScalarType(M_PI)-alpha-alpha_;
        delta	= cos(theta)* ew_c;
        h		= sin(theta)* ew_c;
        curr_d = sqrt( pow(h,2)+ pow(d_curr + delta,2));
      }
    }
    return (curr_d);
  }




/*
This is the low level version of the geodesic computation framework.
Starting from the seeds, it assign a distance value to each vertex. The distance of a vertex is its
approximated geodesic distance to the closest seeds.
This is function is not meant to be called (although is not prevented). Instead, it is invoked by
wrapping function.
*/

  template <class DistanceFunctor>
  static  VertexPointer Visit(
      MeshType & m,
      std::vector<VertDist> & seedVec, // the set of seeds to start from
      DistanceFunctor &distFunc,
      ScalarType distance_threshold  = std::numeric_limits<ScalarType>::max(),                    // cut off distance (do no compute anything farther than this value)
      typename MeshType::template PerVertexAttributeHandle<VertexPointer> * vertSource = NULL,    // if present we put in this attribute the closest source for each vertex
      typename MeshType::template PerVertexAttributeHandle<VertexPointer> * vertParent = NULL,    // if present we put in this attribute the parent in the path that goes from the vertex to the closest source
      std::vector<VertexPointer> *InInterval=NULL)
  {
    VertexPointer farthest=0;
//    int t0=clock();
    //Requirements
    if(!HasVFAdjacency(m)) throw vcg::MissingComponentException("VFAdjacency");
    if(!HasPerVertexQuality(m)) throw vcg::MissingComponentException("VertexQuality");
    assert(!seedVec.empty());

    TempDataType TD(m.vert, std::numeric_limits<ScalarType>::max());

    // initialize Heap
    std::vector<VertDist> frontierHeap;
    typename std::vector <VertDist >::iterator ifr;
    for(ifr = seedVec.begin(); ifr != seedVec.end(); ++ifr){
      TD[(*ifr).v].d = (*ifr).d;
      TD[(*ifr).v].source  = (*ifr).v;
      TD[(*ifr).v].parent  = (*ifr).v;
      frontierHeap.push_back(*ifr);
    }
    make_heap(frontierHeap.begin(),frontierHeap.end(),pred());

    ScalarType curr_d,d_curr = 0.0,d_heap;
    ScalarType max_distance=0.0;
//    int t1=clock();
    while(!frontierHeap.empty() && max_distance < distance_threshold)
    {
      pop_heap(frontierHeap.begin(),frontierHeap.end(),pred());
      VertexPointer curr = (frontierHeap.back()).v;
      if (InInterval!=NULL) InInterval->push_back(curr);

      if(vertSource!=NULL)  (*vertSource)[curr] = TD[curr].source;
      if(vertParent!=NULL)  (*vertParent)[curr] = TD[curr].parent;

      d_heap = (frontierHeap.back()).d;
      frontierHeap.pop_back();

      assert(TD[curr].d <= d_heap);
      if(TD[curr].d < d_heap ) // a vertex whose distance has been improved after it was inserted in the queue
        continue;
      assert(TD[curr].d == d_heap);

      d_curr =  TD[curr].d;

      for(face::VFIterator<FaceType>  vfi(curr) ; vfi.f!=0; ++vfi )
      {
        for(int k=0;k<2;++k)
        {
          VertexPointer pw,pw1;
          if(k==0) {
            pw = vfi.f->V1(vfi.z);
            pw1= vfi.f->V2(vfi.z);
          }
          else {
            pw = vfi.f->V2(vfi.z);
            pw1= vfi.f->V1(vfi.z);
          }

          const ScalarType & d_pw1  =  TD[pw1].d;
          {
            const ScalarType inter  = distFunc(curr,pw1);//(curr->P() - pw1->P()).Norm();
            const ScalarType tol = (inter + d_curr + d_pw1)*.0001f;

            if (	(TD[pw1].source != TD[curr].source)||// not the same source
                    (inter + d_curr < d_pw1  +tol   ) ||
                    (inter + d_pw1  < d_curr +tol  ) ||
                    (d_curr + d_pw1  < inter +tol  )   // triangular inequality
                    )
              curr_d = d_curr + distFunc(pw,curr);//(pw->P()-curr->P()).Norm();
            else
              curr_d = Distance(distFunc,pw,pw1,curr,d_pw1,d_curr);
          }

          if(TD[pw].d > curr_d){
            TD[pw].d = curr_d;
            TD[pw].source = TD[curr].source;
            TD[pw].parent = curr;
            frontierHeap.push_back(VertDist(pw,curr_d));
            push_heap(frontierHeap.begin(),frontierHeap.end(),pred());
          }
//          if(isLeaf){
            if(d_curr > max_distance){
              max_distance = d_curr;
              farthest = curr;
            }
//          }
        }
      } // end for VFIterator
    }// end while
//    int t2=clock();

    // Copy found distance onto the Quality (\todo parametric!)
    if (InInterval==NULL)
    {
      for(VertexIterator vi = m.vert.begin(); vi != m.vert.end(); ++vi) if(!(*vi).IsD())
        (*vi).Q() =  TD[&(*vi)].d;
    }
    else
    {
      assert(InInterval->size()>0);
      for(size_t i=0;i<InInterval->size();i++)
        (*InInterval)[i]->Q() =  TD[(*InInterval)[i]].d;
    }
//    int t3=clock();
//    printf("Init  %6.3f\nVisit %6.3f\nFinal %6.3f\n",float(t1-t0)/CLOCKS_PER_SEC,float(t2-t1)/CLOCKS_PER_SEC,float(t3-t2)/CLOCKS_PER_SEC);
    return farthest;
  }

public:
  /*! \brief Given a set of source vertices compute the approximate geodesic distance to all the other vertices

\param m the mesh
\param seedVec a vector of Vertex pointers with the \em sources of the flood fill
\param maxDistanceThr max distance that we travel on the mesh starting from the sources
\param withinDistanceVec a pointer to a vector for storing the vertexes reached within the passed maxDistanceThr
\param sourceSeed pointer to the handle to keep for each vertex its seed
\param parentSeed pointer to the handle to keep for each vertex its parent in the closest tree

Given a mesh and a vector of pointers to seed vertices, this function compute the approximated geodesic
distance from the given sources to all the mesh vertices within the given maximum distance threshold.
The computed distance is stored in the vertex::Quality component.
Optionally for each vertex it can store, in a passed attribute, the corresponding seed vertex
(e.g. the vertex of the source set closest to him) and the 'parent' in a tree forest that connects each vertex to the closest source.

To allocate the attributes:
\code
      typename MeshType::template PerVertexAttributeHandle<VertexPointer> sourcesHandle;
      sourcesHandle =  tri::Allocator<CMeshO>::AddPerVertexAttribute<MeshType::VertexPointer> (m,"sources");

      typename MeshType::template PerVertexAttributeHandle<VertexPointer> parentHandle;
      parentHandle =  tri::Allocator<CMeshO>::AddPerVertexAttribute<MeshType::VertexPointer> (m,"parent");
\endcode

It requires VF adjacency relation (e.g. vertex::VFAdj and face::VFAdj components)
It requires per vertex Quality (e.g. vertex::Quality component)

\warning that this function has ALWAYS at least a linear cost (it use additional attributes that have a linear initialization)
\todo make it O(output) by using incremental mark and persistent attributes.
            */
  static bool Compute( MeshType & m,
                       const std::vector<VertexPointer> & seedVec)
  {
    EuclideanDistance<MeshType> dd;
    return Compute(m,seedVec,dd);
  }

  template <class DistanceFunctor>
  static bool Compute( MeshType & m,
                       const std::vector<VertexPointer> & seedVec,
                       DistanceFunctor &distFunc,
                       ScalarType maxDistanceThr  = std::numeric_limits<ScalarType>::max(),
                       std::vector<VertexPointer> *withinDistanceVec=NULL,
                       typename MeshType::template PerVertexAttributeHandle<VertexPointer> * sourceSeed = NULL,
                       typename MeshType::template PerVertexAttributeHandle<VertexPointer> * parentSeed = NULL
                       )
  {
    if(seedVec.empty())	return false;
    std::vector<VertDist> vdSeedVec;
    typename std::vector<VertexPointer>::const_iterator fi;
    for( fi  = seedVec.begin(); fi != seedVec.end() ; ++fi)
        vdSeedVec.push_back(VertDist(*fi,0.0));
    Visit(m, vdSeedVec, distFunc, maxDistanceThr, sourceSeed, parentSeed, withinDistanceVec);
    return true;
  }

  /* \brief Assigns to each vertex of the mesh its distance to the closest vertex on the boundary

It is just a simple wrapper of the basic Compute()

            Note: update the field Q() of the vertices
            Note: it needs the border bit set.
            */
  static bool DistanceFromBorder(	MeshType & m, typename MeshType::template PerVertexAttributeHandle<VertexPointer> * sources = NULL)
  {
    std::vector<VertexPointer> fro;
    for(VertexIterator vi = m.vert.begin(); vi != m.vert.end(); ++vi)
      if( (*vi).IsB())
        fro.push_back(&(*vi));
    if(fro.empty()) return false;
    EuclideanDistance<MeshType> dd;
    tri::UpdateQuality<MeshType>::VertexConstant(m,0);
    return Compute(m,fro,dd,std::numeric_limits<ScalarType>::max(),0,sources);
  }


  static bool ConvertPerVertexSeedToPerFaceSeed(MeshType &m, const std::vector<VertexPointer> &vertexSeedVec,
                                                 std::vector<FacePointer> &faceSeedVec)
  {
    tri::RequireVFAdjacency(m);
    tri::RequirePerFaceMark(m);

    faceSeedVec.clear();
    tri::UnMarkAll(m);
    for(size_t i=0;i<vertexSeedVec.size();++i)
    {
      for(face::VFIterator<FaceType> vfi(vertexSeedVec[i]);!vfi.End();++vfi)
      {
        if(tri::IsMarked(m,vfi.F())) return false;
        faceSeedVec.push_back(vfi.F());
        tri::Mark(m,vfi.F());
      }
    }
    return true;
  }

  template <class DistanceFunctor>
  static void PerFaceDijsktraCompute(MeshType &m, const std::vector<FacePointer> &seedVec,
                                     DistanceFunctor &distFunc,
                                     ScalarType maxDistanceThr  = std::numeric_limits<ScalarType>::max(),
                                     std::vector<FacePointer> *InInterval=NULL,
                                     FacePointer FaceTarget=NULL,
                                     bool avoid_selected=false)
  {
    tri::RequireFFAdjacency(m);
    tri::RequirePerFaceMark(m);
    tri::RequirePerFaceQuality(m);

    typename MeshType::template PerFaceAttributeHandle<FacePointer> sourceHandle
        = tri::Allocator<MeshType>::template GetPerFaceAttribute<FacePointer> (m,"sources");

    typename MeshType::template PerFaceAttributeHandle<FacePointer> parentHandle
        = tri::Allocator<MeshType>::template GetPerFaceAttribute<FacePointer> (m,"parent");

    std::vector<FaceDist> Heap;
    tri::UnMarkAll(m);
    for(size_t i=0;i<seedVec.size();++i)
    {
      tri::Mark(m,seedVec[i]);
      seedVec[i]->Q()=0;
      sourceHandle[seedVec[i]]=seedVec[i];
      parentHandle[seedVec[i]]=seedVec[i];
      Heap.push_back(FaceDist(seedVec[i]));
      if (InInterval!=NULL) InInterval->push_back(seedVec[i]);
    }

    std::make_heap(Heap.begin(),Heap.end());
    while(!Heap.empty())
    {
      pop_heap(Heap.begin(),Heap.end());
      FacePointer curr = (Heap.back()).f;
      if ((FaceTarget!=NULL)&&(curr==FaceTarget))return;
      Heap.pop_back();

      for(int i=0;i<3;++i)
      {
        if(!face::IsBorder(*curr,i) )
        {
          FacePointer nextF = curr->FFp(i);
          ScalarType nextDist = curr->Q() + distFunc(curr,nextF);
          if( (nextDist < maxDistanceThr) &&
              (!tri::IsMarked(m,nextF) ||  nextDist < nextF->Q()) )
          {
            nextF->Q() = nextDist;
            if ((avoid_selected)&&(nextF->IsS()))continue;
            tri::Mark(m,nextF);
            Heap.push_back(FaceDist(nextF));
            push_heap(Heap.begin(),Heap.end());
            if (InInterval!=NULL) InInterval->push_back(nextF);
            sourceHandle[nextF] = sourceHandle[curr];
            parentHandle[nextF] = curr;
//            printf("Heapsize %i nextDist = %f curr face %i next face %i \n",Heap.size(), nextDist, tri::Index(m,curr), tri::Index(m,nextF));
          }
        }
      }
    }
  }




  template <class DistanceFunctor>
  static void PerVertexDijsktraCompute(MeshType &m, const std::vector<VertexPointer> &seedVec,
                                       DistanceFunctor &distFunc,
                                     ScalarType maxDistanceThr  = std::numeric_limits<ScalarType>::max(),
                                     std::vector<VertexPointer> *InInterval=NULL,bool avoid_selected=false,
                                     VertexPointer target=NULL)
  {
    tri::RequireVFAdjacency(m);
    tri::RequirePerVertexMark(m);
    tri::RequirePerVertexQuality(m);

    typename MeshType::template PerVertexAttributeHandle<VertexPointer> sourceHandle
        = tri::Allocator<MeshType>::template GetPerVertexAttribute<VertexPointer> (m,"sources");

    typename MeshType::template PerVertexAttributeHandle<VertexPointer> parentHandle
        = tri::Allocator<MeshType>::template GetPerVertexAttribute<VertexPointer> (m,"parent");

    std::vector<DIJKDist> Heap;
    tri::UnMarkAll(m);

    for(size_t i=0;i<seedVec.size();++i)
    {
      assert(!tri::IsMarked(m,seedVec[i]));
      tri::Mark(m,seedVec[i]);
      seedVec[i]->Q()=0;
      sourceHandle[seedVec[i]]=seedVec[i];
      parentHandle[seedVec[i]]=seedVec[i];
      Heap.push_back(DIJKDist(seedVec[i]));
      if (InInterval!=NULL) InInterval->push_back(seedVec[i]);
    }

    std::make_heap(Heap.begin(),Heap.end());
    while(!Heap.empty())
    {
      pop_heap(Heap.begin(),Heap.end());
      VertexPointer curr = (Heap.back()).v;
      if ((target!=NULL)&&(target==curr))return;
      Heap.pop_back();
      std::vector<VertexPointer> vertVec;
      face::VVStarVF<FaceType>(curr,vertVec);
      for(size_t i=0;i<vertVec.size();++i)
      {
        VertexPointer nextV = vertVec[i];
        if ((avoid_selected)&&(nextV->IsS()))continue;
        ScalarType nextDist = curr->Q() + distFunc(curr,nextV);
        if( (nextDist < maxDistanceThr) &&
            (!tri::IsMarked(m,nextV) ||  nextDist < nextV->Q()) )
        {
          nextV->Q() = nextDist;
          tri::Mark(m,nextV);
          Heap.push_back(DIJKDist(nextV));
          push_heap(Heap.begin(),Heap.end());
          if (InInterval!=NULL) InInterval->push_back(nextV);
          sourceHandle[nextV] = sourceHandle[curr];
          parentHandle[nextV] = curr;
//          printf("Heapsize %i nextDist = %f curr vert %i next vert %i \n",Heap.size(), nextDist, tri::Index(m,curr), tri::Index(m,nextV));
        }
      }
    }
  }


};// end class
}// end namespace tri
}// end namespace vcg
#endif
