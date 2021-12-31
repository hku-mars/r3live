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
#ifndef __VCG_TRI_UPDATE_SELECTION
#define __VCG_TRI_UPDATE_SELECTION

#include <queue>
#include <vcg/complex/algorithms/update/flag.h>

namespace vcg {
namespace tri {
/// \ingroup trimesh
/// \brief A stack for saving and restoring selection.
/**
  This class is used to save the current selection onto a stack for later use.
  \todo it should be generalized to other attributes with a templated approach.
*/
template <class ComputeMeshType>
class SelectionStack
{
  typedef typename ComputeMeshType::template PerVertexAttributeHandle< bool > vsHandle;
  typedef typename ComputeMeshType::template PerEdgeAttributeHandle< bool >   esHandle;
  typedef typename ComputeMeshType::template PerFaceAttributeHandle< bool >   fsHandle;

public:
  SelectionStack(ComputeMeshType &m)
  {
    _m=&m;
  }

  bool push()
  {
    vsHandle vsH = Allocator<ComputeMeshType>::template AddPerVertexAttribute< bool >(*_m);
    esHandle esH = Allocator<ComputeMeshType>::template AddPerEdgeAttribute< bool >(*_m);
    fsHandle fsH = Allocator<ComputeMeshType>::template AddPerFaceAttribute< bool >  (*_m);
    typename ComputeMeshType::VertexIterator vi;
    for(vi = _m->vert.begin(); vi != _m->vert.end(); ++vi)
      if( !(*vi).IsD() ) vsH[*vi] = (*vi).IsS() ;

    typename ComputeMeshType::EdgeIterator ei;
    for(ei = _m->edge.begin(); ei != _m->edge.end(); ++ei)
      if( !(*ei).IsD() ) esH[*ei] = (*ei).IsS() ;

    typename ComputeMeshType::FaceIterator fi;
    for(fi = _m->face.begin(); fi != _m->face.end(); ++fi)
      if( !(*fi).IsD() ) fsH[*fi] = (*fi).IsS() ;

    vsV.push_back(vsH);
    esV.push_back(esH);
    fsV.push_back(fsH);
    return true;
  }

  bool pop()
  {
    if(vsV.empty()) return false;
    vsHandle vsH = vsV.back();
    esHandle esH = esV.back();
    fsHandle fsH = fsV.back();
    if(! (Allocator<ComputeMeshType>::template IsValidHandle(*_m, vsH))) return false;

    typename ComputeMeshType::VertexIterator vi;
    for(vi = _m->vert.begin(); vi != _m->vert.end(); ++vi)
      if( !(*vi).IsD() )
      {
        if(vsH[*vi]) (*vi).SetS() ;
        else (*vi).ClearS() ;
      }

    typename ComputeMeshType::EdgeIterator ei;
    for(ei = _m->edge.begin(); ei != _m->edge.end(); ++ei)
      if( !(*ei).IsD() )
      {
        if(esH[*ei]) (*ei).SetS() ;
        else (*ei).ClearS() ;
      }
    typename ComputeMeshType::FaceIterator fi;
    for(fi = _m->face.begin(); fi != _m->face.end(); ++fi)
      if( !(*fi).IsD() )
      {
          if(fsH[*fi]) (*fi).SetS() ;
                  else (*fi).ClearS() ;
      }

    Allocator<ComputeMeshType>::template DeletePerVertexAttribute<bool>(*_m,vsH);
    Allocator<ComputeMeshType>::template DeletePerEdgeAttribute<bool>(*_m,esH);
    Allocator<ComputeMeshType>::template DeletePerFaceAttribute<bool>(*_m,fsH);
    vsV.pop_back();
    esV.pop_back();
    fsV.pop_back();
    return true;
  }

private:
  ComputeMeshType *_m;
  std::vector<vsHandle> vsV;
  std::vector<esHandle> esV;
  std::vector<fsHandle> fsV;

};

/// \ingroup trimesh

/// \headerfile selection.h vcg/complex/algorithms/update/selection.h

/// \brief Management, updating and computation of per-vertex and per-face normals.
/**
This class is used to compute or update the normals that can be stored in the vertex or face component of a mesh.
*/

template <class ComputeMeshType>
class UpdateSelection
{

public:
typedef ComputeMeshType MeshType;
typedef	typename MeshType::ScalarType			ScalarType;
typedef typename MeshType::VertexType     VertexType;
typedef typename MeshType::VertexPointer  VertexPointer;
typedef typename MeshType::VertexIterator VertexIterator;
typedef typename MeshType::EdgeIterator   EdgeIterator;
typedef typename MeshType::FaceType       FaceType;
typedef typename MeshType::FacePointer    FacePointer;
typedef typename MeshType::FaceIterator   FaceIterator;
typedef typename vcg::Box3<ScalarType>  Box3Type;

static size_t VertexAll(MeshType &m)
{
    VertexIterator vi;
    for(vi = m.vert.begin(); vi != m.vert.end(); ++vi)
        if( !(*vi).IsD() )	(*vi).SetS();
  return m.vn;
}

static size_t EdgeAll(MeshType &m)
{
    EdgeIterator ei;
    for(ei = m.edge.begin(); ei != m.edge.end(); ++ei)
    if( !(*ei).IsD() )	(*ei).SetS();
  return m.fn;
}
static size_t FaceAll(MeshType &m)
{
    FaceIterator fi;
    for(fi = m.face.begin(); fi != m.face.end(); ++fi)
    if( !(*fi).IsD() )	(*fi).SetS();
  return m.fn;
}

static size_t VertexClear(MeshType &m)
{
    VertexIterator vi;
    for(vi = m.vert.begin(); vi != m.vert.end(); ++vi)
        if( !(*vi).IsD() )	(*vi).ClearS();
  return 0;
}

static size_t EdgeClear(MeshType &m)
{
    EdgeIterator ei;
    for(ei = m.edge.begin(); ei != m.edge.end(); ++ei)
        if( !(*ei).IsD() )	(*ei).ClearS();
  return 0;
}

static size_t FaceClear(MeshType &m)
{
    FaceIterator fi;
    for(fi = m.face.begin(); fi != m.face.end(); ++fi)
        if( !(*fi).IsD() )	(*fi).ClearS();
  return 0;
}

static void Clear(MeshType &m)
{
  VertexClear(m);
  EdgeClear(m);
  FaceClear(m);
}

static size_t FaceCount(MeshType &m)
{
  size_t selCnt=0;
    FaceIterator fi;
  for(fi=m.face.begin();fi!=m.face.end();++fi)
    if(!(*fi).IsD() && (*fi).IsS()) ++selCnt;
  return selCnt;
}

static size_t EdgeCount(MeshType &m)
{
  size_t selCnt=0;
  EdgeIterator ei;
  for(ei=m.edge.begin();ei!=m.edge.end();++ei)
    if(!(*ei).IsD() && (*ei).IsS()) ++selCnt;
  return selCnt;
}

static size_t VertexCount(MeshType &m)
{
  size_t selCnt=0;
    VertexIterator vi;
  for(vi=m.vert.begin();vi!=m.vert.end();++vi)
    if(!(*vi).IsD() && (*vi).IsS()) ++selCnt;
  return selCnt;
}

static size_t FaceInvert(MeshType &m)
{
  size_t selCnt=0;
    FaceIterator fi;
  for(fi=m.face.begin();fi!=m.face.end();++fi)
      if(!(*fi).IsD())
      {
        if((*fi).IsS()) (*fi).ClearS();
        else {
          (*fi).SetS();
          ++selCnt;
        }
      }
  return selCnt;
}

static size_t VertexInvert(MeshType &m)
{
  size_t selCnt=0;
    VertexIterator vi;
  for(vi=m.vert.begin();vi!=m.vert.end();++vi)
      if(!(*vi).IsD())
      {
        if((*vi).IsS()) (*vi).ClearS();
        else {
          (*vi).SetS();
          ++selCnt;
        }
      }
  return selCnt;
}

/// \brief Select all the vertices that are touched by at least a single selected faces
static size_t VertexFromFaceLoose(MeshType &m, bool preserveSelection=false)
{
  size_t selCnt=0;

  if(!preserveSelection) VertexClear(m);
  for(FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
    if( !(*fi).IsD() && (*fi).IsS())
      for(int i = 0; i < (*fi).VN(); ++i)
        if( !(*fi).V(i)->IsS()) { (*fi).V(i)->SetS(); ++selCnt; }
  return selCnt;
}

/// \brief Select all the vertices that are touched by at least a single selected edge
static size_t VertexFromEdgeLoose(MeshType &m, bool preserveSelection=false)
{
  size_t selCnt=0;

  if(!preserveSelection) VertexClear(m);
  for(EdgeIterator ei = m.edge.begin(); ei != m.edge.end(); ++ei)
    if( !(*ei).IsD() && (*ei).IsS())
    {
      if( !(*ei).V(0)->IsS()) { (*ei).V(0)->SetS(); ++selCnt; }
      if( !(*ei).V(1)->IsS()) { (*ei).V(1)->SetS(); ++selCnt; }
    }
  return selCnt;
}

/// \brief Select ONLY the vertices that are touched ONLY by selected faces
/** In other words all the vertices having all the faces incident on them selected.
 \warning Isolated vertices will not selected.
*/
static size_t VertexFromFaceStrict(MeshType &m)
{
  VertexFromFaceLoose(m);
  FaceIterator fi;
  for(fi = m.face.begin(); fi != m.face.end(); ++fi)
    if( !(*fi).IsD() && !(*fi).IsS())
      for(int i = 0; i < (*fi).VN(); ++i)
        (*fi).V(i)->ClearS();
  return VertexCount(m);
}

/// \brief Select ONLY the faces with ALL the vertices selected
static size_t FaceFromVertexStrict(MeshType &m)
{
  size_t selCnt=0;
  FaceClear(m);
  for(FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
    if( !(*fi).IsD())
    {
      bool selFlag=true;
      for(int i = 0; i < (*fi).VN(); ++i)
        if(!(*fi).V(i)->IsS())
          selFlag =false;
      if(selFlag)
      {
        (*fi).SetS();
        ++selCnt;
      }
    }
  return selCnt;
}

/// \brief Select all the faces with at least one selected vertex
static size_t FaceFromVertexLoose(MeshType &m)
{
  size_t selCnt=0;
  FaceClear(m);
  for(FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
    if( !(*fi).IsD())
    {
      bool selVert=false;
      for(int i = 0; i < (*fi).VN(); ++i)
        if((*fi).V(i)->IsS()) selVert=true;

      if(selVert) {
        (*fi).SetS();
        ++selCnt;
      }
    }
  return selCnt;
}

static size_t VertexFromBorderFlag(MeshType &m)
{
  size_t selCnt=0;
  VertexClear(m);
  VertexIterator vi;
  for(vi = m.vert.begin(); vi != m.vert.end(); ++vi)
    if( !(*vi).IsD() )
    {
      if((*vi).IsB() )
      {
        (*vi).SetS();
        ++selCnt;
      }
    }
  return selCnt;
}


static size_t FaceFromBorderFlag(MeshType &m)
{
  tri::RequireTriangularMesh(m);
  size_t selCnt=0;
  FaceClear(m);
  for(FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
    if( !(*fi).IsD() )
    {
      bool bordFlag=false;
      for(int i = 0; i < 3; ++i)
        if((*fi).IsB(i)) bordFlag=true;
      if(bordFlag)
      {
        (*fi).SetS();
        ++selCnt;
      }
    }
  return selCnt;
}

/// \brief This function select the faces that have an edge outside the given range.
static size_t FaceOutOfRangeEdge(MeshType &m, ScalarType MinEdgeThr=0, ScalarType MaxEdgeThr=(std::numeric_limits<ScalarType>::max)())
{
  FaceIterator fi;
  size_t count_fd = 0;
  MinEdgeThr=MinEdgeThr*MinEdgeThr;
  MaxEdgeThr=MaxEdgeThr*MaxEdgeThr;
  for(fi=m.face.begin(); fi!=m.face.end();++fi)
    if(!(*fi).IsD())
      {
        for(int i=0;i<(*fi).VN();++i)
        {
          const ScalarType squaredEdge=SquaredDistance((*fi).V0(i)->cP(),(*fi).V1(i)->cP());
          if((squaredEdge<=MinEdgeThr) || (squaredEdge>=MaxEdgeThr) )
          {
            count_fd++;
            (*fi).SetS();
            break; // skip the rest of the edges of the tri
          }
        }
      }
      return count_fd;
}

/// \brief This function expand current selection to cover the whole connected component.
static size_t FaceConnectedFF(MeshType &m)
{
    // it also assumes that the FF adjacency is well computed.
    assert (HasFFAdjacency(m));
    UpdateFlags<MeshType>::FaceClearV(m);

    std::deque<FacePointer> visitStack;
  size_t selCnt=0;
  FaceIterator fi;
    for(fi = m.face.begin(); fi != m.face.end(); ++fi)
        if( !(*fi).IsD() && (*fi).IsS() && !(*fi).IsV() )
                visitStack.push_back(&*fi);

    while(!visitStack.empty())
    {
            FacePointer fp = visitStack.front();
            visitStack.pop_front();
            assert(!fp->IsV());
            fp->SetV();
      for(int i=0;i<fp->VN();++i) {
                FacePointer ff = fp->FFp(i);
        if(! ff->IsS())
                        {
                            ff->SetS();
                            ++selCnt;
                            visitStack.push_back(ff);
                            assert(!ff->IsV());
                        }
      }
    }
  return selCnt;
}
/// \brief Select ONLY the faces whose quality is in the specified closed interval.
static size_t FaceFromQualityRange(MeshType &m,float minq, float maxq)
{
  size_t selCnt=0;
  FaceClear(m);
  FaceIterator fi;
  assert(HasPerFaceQuality(m));
  for(fi=m.face.begin();fi!=m.face.end();++fi)
      if(!(*fi).IsD())
      {
        if( (*fi).Q()>=minq &&  (*fi).Q()<=maxq )
          {
            (*fi).SetS();
            ++selCnt;
          }
      }
  return selCnt;
}

/// \brief Select ONLY the vertices whose quality is in the specified closed interval.
static size_t VertexFromQualityRange(MeshType &m,float minq, float maxq)
{
  size_t selCnt=0;
    VertexClear(m);
    VertexIterator vi;
    assert(HasPerVertexQuality(m));
  for(vi=m.vert.begin();vi!=m.vert.end();++vi)
      if(!(*vi).IsD())
      {
        if( (*vi).Q()>=minq &&  (*vi).Q()<=maxq )
                    {
                        (*vi).SetS();
                        ++selCnt;
                    }
      }
  return selCnt;
}

static int VertexInBox( MeshType & m, const Box3Type &bb)
{
  int selCnt=0;
  for (VertexIterator vi = m.vert.begin(); vi != m.vert.end(); ++vi) if(!(*vi).IsD())
  {
    if(bb.IsIn((*vi).cP()) ) {
      (*vi).SetS();
      ++selCnt;
    }
  }
  return selCnt;
}


void VertexNonManifoldEdges(MeshType &m)
{
  assert(HasFFTopology(m));

  VertexClear(m);
  for (FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)	if (!fi->IsD())
    {
      for(int i=0;i<fi->VN();++i)
      if(!IsManifold(*fi,i)){
        (*fi).V0(i)->SetS();
        (*fi).V1(i)->SetS();
        }
    }
}

}; // end class

}	// End namespace
}	// End namespace


#endif
