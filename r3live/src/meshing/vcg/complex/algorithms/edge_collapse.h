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
#ifndef __VCG_TETRA_TRI_COLLAPSE
#define __VCG_TETRA_TRI_COLLAPSE


#include<vcg/simplex/face/pos.h>
#include<vcg/simplex/face/topology.h>

namespace vcg{
namespace tri{

template < class VERTEX_TYPE>
class BasicVertexPair {
public:
  inline BasicVertexPair() {}
  inline BasicVertexPair( VERTEX_TYPE * v0, VERTEX_TYPE * v1){V(0) = v0; V(1) = v1; }
  void Sort() {if(V(0)<V(0)) std::swap(V(0),V(0)); }
  VERTEX_TYPE *&V(int i) { return v[i]; }
  VERTEX_TYPE *cV(int i) const { return v[i]; }
private:
  VERTEX_TYPE *v[2]; // remember that v[0] will be deleted and v[1] will survive (eventually with a new position)
};


/** \addtogroup trimesh */
/*@{*/
/** This a static utility class for the edge collapse.
    It provides a common set of useful function for actually making an edge collapse over a trimesh.
    See also the corresponding class in the local optimization framework called TriEdgeCollapse
**/

template <class TRI_MESH_TYPE, class VertexPair>
class EdgeCollapser
{
    public:
  /// The tetrahedral mesh type
  typedef	 TRI_MESH_TYPE TriMeshType;
  /// The face type
  typedef	typename TriMeshType::FaceType FaceType;
    /// The vertex type
    typedef	typename FaceType::VertexType VertexType;
  typedef	typename FaceType::VertexPointer VertexPointer;
  /// The vertex iterator type
  typedef	typename TriMeshType::VertexIterator VertexIterator;
  /// The tetra iterator type
  typedef	typename TriMeshType::FaceIterator FaceIterator;
  /// The coordinate type
    typedef	typename FaceType::VertexType::CoordType CoordType;
  /// The scalar type
  typedef	typename TriMeshType::VertexType::ScalarType ScalarType;
  ///the container of tetrahedron type
  typedef typename TriMeshType::FaceContainer FaceContainer;
  ///the container of vertex type
  typedef typename TriMeshType::VertContainer VertContainer;
  ///half edge type
  //typedef typename TriMeshType::FaceType::EdgeType EdgeType;
    /// vector of pos
//	typedef typename std::vector<EdgeType> EdgeVec;
    ///of VFIterator
    typedef typename vcg::face::VFIterator<FaceType>  VFI;
    /// vector of VFIterator
    typedef typename std::vector<vcg::face::VFIterator<FaceType> > VFIVec;
private:
  struct EdgeSet
  {
     VFIVec av0,av1,av01;
     VFIVec & AV0() { return av0;}
     VFIVec & AV1() { return av1;}
     VFIVec & AV01(){ return av01;}
  };

  static void FindSets(VertexPair &p, EdgeSet &es)
    {
        VertexType * v0 = p.V(0);
        VertexType * v1 = p.V(1);

    es.AV0().clear();  // Facce incidenti in v0
    es.AV1().clear();  // Facce incidenti in v1
    es.AV01().clear(); // Facce incidenti in v0 e v1

        VFI x;

        for( x.f = v0->VFp(), x.z = v0->VFi(); x.f!=0; ++x)
        {
            int zv1 = -1;

            for(int j=0;j<3;++j)
                if( x.f->V(j)==&*v1 )	{
                    zv1 = j;
                    break;
                }
      if(zv1==-1) 	es.AV0().push_back( x ); // la faccia x.f non ha il vertice v1 => e' incidente solo in v0
      else    			es.AV01().push_back( x );
        }

        for( x.f = v1->VFp(), x.z = v1->VFi(); x.f!=0; ++x )
        {
            int zv0 = -1;

            for(int j=0;j<3;++j)
                if( x.f->V(j)==&*v0 )	{
                    zv0 = j;
                    break;
                }
      if(zv0==-1)	es.AV1().push_back( x ); // la faccia x.f non ha il vertice v1 => e' incidente solo in v0
        }
}
/*
    Link Conditions test, as described in

    Topology Preserving Edge Contraction
    T. Dey, H. Edelsbrunner,
    Pub. Inst. Math. 1999

    Lk (sigma) is the set of all the faces of the cofaces of sigma that are disjoint from sigma

    Lk(v0) inters Lk(v1) == Lk(v0-v1)

    To perform these tests using only the VF adjacency we resort to some virtual counters over
    the vertices and the edges, we implement them as std::maps, and we increase these counters
    by running over all the faces around each vertex of the collapsing edge.

    At the end (after adding dummy stuff) we should have
       2 for vertices not shared
       4 for vertices shared
       2 for edges shared
       1 for edges not shared.


*/

public:
  static bool LinkConditions(VertexPair &pos)
  {
    typedef typename vcg::face::VFIterator<FaceType> VFIterator;
    // at the end of the loop each vertex must be counted twice
    // except for boundary vertex.
    std::map<VertexPointer,int> VertCnt;
    std::map<std::pair<VertexPointer,VertexPointer>,int> EdgeCnt;

    // the list of the boundary vertexes for the two endpoints
    std::vector<VertexPointer> BoundaryVertexVec[2];

    // Collect vertexes and edges of V0 and V1
    VFIterator vfi;
    for(int i=0;i<2;++i)
    {
      vfi = VFIterator(pos.V(i));
      for( ;!vfi.End();++vfi)
      {
        ++ VertCnt[vfi.V1()];
        ++ VertCnt[vfi.V2()];
        if(vfi.V1()<vfi.V2()) ++EdgeCnt[std::make_pair(vfi.V1(),vfi.V2())];
                         else ++EdgeCnt[std::make_pair(vfi.V2(),vfi.V1())];
      }
      // Now a loop to add dummy stuff: add the dummy vertex and two dummy edges
      // (and remember to increase the counters for the two boundary vertexes involved)
      typename std::map<VertexPointer,int>::iterator vcmit;
      for(vcmit=VertCnt.begin();vcmit!=VertCnt.end();++vcmit)
      {
        if((*vcmit).second==1) // boundary vertexes are counted only once
          BoundaryVertexVec[i].push_back((*vcmit).first);
      }
      if(BoundaryVertexVec[i].size()==2)
      { // aha! one of the two vertex of the collapse is on the boundary
        // so add dummy vertex and two dummy edges
        VertCnt[0]+=2;
        ++ EdgeCnt[std::make_pair(VertexPointer(0),BoundaryVertexVec[i][0]) ] ;
        ++ EdgeCnt[std::make_pair(VertexPointer(0),BoundaryVertexVec[i][1]) ] ;
        // remember to hide the boundaryness of the two boundary vertexes
        ++VertCnt[BoundaryVertexVec[i][0]];
        ++VertCnt[BoundaryVertexVec[i][1]];
      }
    }

    // Final loop to find cardinality of Lk( V0-V1 )
    // Note that Lk(edge) is only a set of vertices.
    std::vector<VertexPointer> LkEdge;

    for( vfi = VFIterator(pos.V(0)); !vfi.End(); ++vfi)
    {
      if(vfi.V1() == pos.V(1) ) LkEdge.push_back(vfi.V2());
      if(vfi.V2() == pos.V(1) ) LkEdge.push_back(vfi.V1());
    }

    // if the collapsing edge was a boundary edge, we must add the dummy vertex.
    // Note that this implies that Lk(edge) >=2;
    if(LkEdge.size()==1)
    {
      LkEdge.push_back(0);
    }

    // NOW COUNT!!!
    size_t SharedEdgeCnt=0;
    typename std::map<std::pair<VertexPointer,VertexPointer>, int>::iterator eci;
    for(eci=EdgeCnt.begin();eci!=EdgeCnt.end();++eci)
      if((*eci).second == 2) SharedEdgeCnt ++;

    if(SharedEdgeCnt>0) return false;
    size_t SharedVertCnt=0;
    typename std::map<VertexPointer,int>::iterator vci;
    for(vci=VertCnt.begin();vci!=VertCnt.end();++vci)
      if((*vci).second == 4) SharedVertCnt++;

    if(SharedVertCnt != LkEdge.size() ) return false;

    return true;
  }

  // Main function; the one that actually make the collapse
  // remember that v[0] will be deleted and v[1] will survive (eventually with a new position)
  // hint to do a 'collapse onto a vertex simply pass p as the position of the surviving vertex
  static int Do(TriMeshType &m, VertexPair & c, const Point3<ScalarType> &p)
    {
    EdgeSet es;
    FindSets(c,es);
        typename VFIVec::iterator i;
    int n_face_del =0 ;

    for(i=es.AV01().begin();i!=es.AV01().end();++i)
        {
            FaceType  & f = *((*i).f);
            assert(f.V((*i).z) == c.V(0));
            vcg::face::VFDetach(f,((*i).z+1)%3);
            vcg::face::VFDetach(f,((*i).z+2)%3);
            Allocator<TriMeshType>::DeleteFace(m,f);
      n_face_del++;
    }

        //set Vertex Face topology
    for(i=es.AV0().begin();i!=es.AV0().end();++i)
        {
            (*i).f->V((*i).z) = c.V(1);									 // In tutte le facce incidenti in v0, si sostituisce v0 con v1
            (*i).f->VFp((*i).z) = (*i).f->V((*i).z)->VFp(); // e appendo la lista di facce incidenti in v1 a questa faccia
            (*i).f->VFi((*i).z) = (*i).f->V((*i).z)->VFi();
            (*i).f->V((*i).z)->VFp() = (*i).f;
            (*i).f->V((*i).z)->VFi() = (*i).z;
        }

        Allocator<TriMeshType>::DeleteVertex(m,*(c.V(0)));
        c.V(1)->P()=p;
        return n_face_del;
    }

};

}
}
#endif
