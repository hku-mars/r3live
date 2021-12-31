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

#ifndef __VCGLIB_TRIMESH_STAT
#define __VCGLIB_TRIMESH_STAT

// Standard headers
// VCG headers

#include <vcg/math/histogram.h>
#include <vcg/simplex/face/pos.h>
#include <vcg/simplex/face/topology.h>
#include <vcg/complex/algorithms/closest.h>
#include <vcg/space/index/grid_static_ptr.h>


namespace vcg {
namespace tri{
template <class StatMeshType>
class Stat
{
public:
  typedef StatMeshType MeshType;
  typedef typename MeshType::VertexType     VertexType;
  typedef typename MeshType::VertexPointer  VertexPointer;
  typedef typename MeshType::VertexIterator VertexIterator;
  typedef typename MeshType::ScalarType			ScalarType;
  typedef typename MeshType::FaceType       FaceType;
  typedef typename MeshType::FacePointer    FacePointer;
  typedef typename MeshType::FaceIterator   FaceIterator;
  typedef typename MeshType::EdgeIterator   EdgeIterator;
  typedef typename MeshType::FaceContainer  FaceContainer;
  typedef typename vcg::Box3<ScalarType>  Box3Type;

  static void ComputePerVertexQualityMinMax( MeshType & m, float &minV, float &maxV)
  {
    std::pair<float,float> pp=ComputePerVertexQualityMinMax(m);
    minV=pp.first; maxV=pp.second;
  }
  static std::pair<float,float> ComputePerVertexQualityMinMax( MeshType & m)
  {
//    assert(0);
    tri::RequirePerVertexQuality(m);
    typename MeshType::template PerMeshAttributeHandle  < std::pair<float,float> > mmqH;
    mmqH = tri::Allocator<MeshType>::template GetPerMeshAttribute <std::pair<float,float> >(m,"minmaxQ");

    std::pair<float,float> minmax = std::make_pair(std::numeric_limits<float>::max(),-std::numeric_limits<float>::max());

    for(VertexIterator vi = m.vert.begin(); vi != m.vert.end(); ++vi)
      if(!(*vi).IsD())
      {
        if( (*vi).Q() < minmax.first) minmax.first=(*vi).Q();
        if( (*vi).Q() > minmax.second) minmax.second=(*vi).Q();
      }

    mmqH() = minmax;
    return minmax;
  }

  static void ComputePerFaceQualityMinMax( MeshType & m, float &minV, float &maxV)
  {
    std::pair<float,float> pp=ComputePerFaceQualityMinMax(m);
    minV=pp.first; maxV=pp.second;
  }

  static std::pair<float,float> ComputePerFaceQualityMinMax( MeshType & m)
  {
    tri::RequirePerFaceQuality(m);
    std::pair<float,float> minmax = std::make_pair(std::numeric_limits<float>::max(),-std::numeric_limits<float>::max());

    FaceIterator fi;
    for(fi = m.face.begin(); fi != m.face.end(); ++fi)
      if(!(*fi).IsD())
      {
        if( (*fi).Q() < minmax.first)  minmax.first =(*fi).Q();
        if( (*fi).Q() > minmax.second) minmax.second=(*fi).Q();
      }
    return minmax;
  }

  /**
    \short compute the barycenter of the surface thin-shell.
    E.g. it assume a 'empty' model where all the mass is located on the surface and compute the barycenter of that thinshell.
    Works for any triangulated model (no problem with open, nonmanifold selfintersecting models).
    Useful for computing the barycenter of 2D planar figures.
    */
  static Point3<ScalarType> ComputeShellBarycenter(MeshType & m)
  {
    Point3<ScalarType> barycenter(0,0,0);
    ScalarType areaSum=0;
    FaceIterator fi;
    for(fi = m.face.begin(); fi != m.face.end(); ++fi)
      if(!(*fi).IsD())
      {
        ScalarType area=DoubleArea(*fi);
        barycenter += Barycenter(*fi)*area;
        areaSum+=area;
      }
    return barycenter/areaSum;
  }


  static ScalarType ComputeMeshArea(MeshType & m)
  {
    ScalarType area=0;

    for(FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
      if(!(*fi).IsD())
        area += DoubleArea(*fi);

    return area/ScalarType(2.0);
  }

  static void ComputePerVertexQualityDistribution( MeshType & m, Distribution<float> &h, bool selectionOnly = false)    // V1.0
  {
    tri::RequirePerVertexQuality(m);
    for(VertexIterator vi = m.vert.begin(); vi != m.vert.end(); ++vi)
      if(!(*vi).IsD() &&  ((!selectionOnly) || (*vi).IsS()) )
      {
        assert(!math::IsNAN((*vi).Q()) && "You should never try to compute Histogram with Invalid Floating points numbers (NaN)");
        h.Add((*vi).Q());
      }
  }

  static void ComputePerFaceQualityDistribution( MeshType & m, Distribution<float> &h, bool selectionOnly = false)    // V1.0
  {
    tri::RequirePerFaceQuality(m);
    for(FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
      if(!(*fi).IsD() &&  ((!selectionOnly) || (*fi).IsS()) )
      {
        assert(!math::IsNAN((*fi).Q()) && "You should never try to compute Histogram with Invalid Floating points numbers (NaN)");
        h.Add((*fi).Q());
      }
  }

  static void ComputePerFaceQualityHistogram( MeshType & m, Histogramf &h, bool selectionOnly=false,int HistSize=10000 )
  {
    tri::RequirePerFaceQuality(m);
    std::pair<float,float> minmax = tri::Stat<MeshType>::ComputePerFaceQualityMinMax(m);
    h.Clear();
    h.SetRange( minmax.first,minmax.second, HistSize );
    for(FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
      if(!(*fi).IsD() &&  ((!selectionOnly) || (*fi).IsS()) ){
        assert(!math::IsNAN((*fi).Q()) && "You should never try to compute Histogram with Invalid Floating points numbers (NaN)");
        h.Add((*fi).Q());
      }
  }

  static void ComputePerVertexQualityHistogram( MeshType & m, Histogramf &h, bool selectionOnly = false, int HistSize=10000 )    // V1.0
  {
    tri::RequirePerVertexQuality(m);
    std::pair<float,float> minmax = ComputePerVertexQualityMinMax(m);

    h.Clear();
    h.SetRange( minmax.first,minmax.second, HistSize);
    for(VertexIterator vi = m.vert.begin(); vi != m.vert.end(); ++vi)
      if(!(*vi).IsD() &&  ((!selectionOnly) || (*vi).IsS()) )
      {
        assert(!math::IsNAN((*vi).Q()) && "You should never try to compute Histogram with Invalid Floating points numbers (NaN)");
        h.Add((*vi).Q());
      }
    // Sanity check; If some very wrong value has happened in the Q value,
    // the histogram is messed. If a significant percentage (20% )of the values are all in a single bin
    // we should try to solve the problem. No easy solution here.
    // We choose to compute the get the 1percentile and 99 percentile values as new mixmax ranges
    // and just to be sure enlarge the Histogram.

    if(h.MaxCount() > HistSize/5)
    {
      std::vector<float> QV;
      QV.reserve(m.vn);
      for(VertexIterator vi = m.vert.begin(); vi != m.vert.end(); ++vi)
        if(!(*vi).IsD()) QV.push_back((*vi).Q());

      std::nth_element(QV.begin(),QV.begin()+m.vn/100,QV.end());
      float newmin=*(QV.begin()+m.vn/100);
      std::nth_element(QV.begin(),QV.begin()+m.vn-m.vn/100,QV.end());
      float newmax=*(QV.begin()+m.vn-m.vn/100);

      h.Clear();
      h.SetRange(newmin, newmax, HistSize*50);
      for(VertexIterator vi = m.vert.begin(); vi != m.vert.end(); ++vi)
        if(!(*vi).IsD() && ((!selectionOnly) || (*vi).IsS()) )
          h.Add((*vi).Q());
    }
  }

  static void ComputeEdgeLengthHistogram( MeshType & m, Histogramf &h)
  {
    assert(m.edge.size()>0);
    h.Clear();
    h.SetRange( 0, m.bbox.Diag(), 10000);
    for(EdgeIterator ei = m.edge.begin(); ei != m.edge.end(); ++ei)
    {
      if(!(*ei).IsD())
      {
        h.Add(Distance<float>((*ei).V(0)->P(),(*ei).V(1)->P()));
      }
    }
  }

  static ScalarType ComputeEdgeLengthAverage(MeshType & m)
  {
    Histogramf h;
    ComputeEdgeLengthHistogram(m,h);
    return h.Avg();
  }

  static void ComputeFaceEdgeLengthDistribution( MeshType & m, Distribution<float> &h)
  {
    tri::RequireTriangularMesh(m);
    h.Clear();
    tri::UpdateFlags<MeshType>::FaceBorderFromNone(m);
    for(FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
    {
      if(!(*fi).IsD())
      {
        for(int i=0;i<3;++i)
        {
          h.Add(Distance<float>(fi->P0(i),fi->P1(i)));
          if(fi->IsB(i)) // to be uniform border edges must be added twice...
            h.Add(Distance<float>(fi->P0(i),fi->P1(i)));
        }
      }
    }
  }

  static ScalarType ComputeFaceEdgeLengthAverage(MeshType & m)
  {
    double sum=0;
    for(FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
      if(!(*fi).IsD())
      {
        for(int i=0;i<3;++i)
          sum+=double(Distance<float>(fi->P0(i),fi->P1(i)));
      }
    return sum/(m.fn*3.0);
  }

}; // end class

} //End Namespace tri
} // End Namespace vcg

#endif

