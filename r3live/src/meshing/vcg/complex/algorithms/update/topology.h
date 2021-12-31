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

#ifndef __VCG_TRI_UPDATE_TOPOLOGY
#define __VCG_TRI_UPDATE_TOPOLOGY
#include <algorithm>
#include <vector>
#include <vcg/simplex/face/pos.h>
#include <vcg/simplex/face/topology.h>

namespace vcg {
namespace tri {
/// \ingroup trimesh

/// \headerfile topology.h vcg/complex/algorithms/update/topology.h

/// \brief Generation of per-vertex and per-face topological information.

template <class UpdateMeshType>
class UpdateTopology
{

public:
typedef UpdateMeshType MeshType;
typedef typename MeshType::ScalarType     ScalarType;
typedef typename MeshType::VertexType     VertexType;
typedef typename MeshType::VertexPointer  VertexPointer;
typedef typename MeshType::VertexIterator VertexIterator;
typedef typename MeshType::EdgePointer    EdgePointer;
typedef typename MeshType::EdgeIterator    EdgeIterator;
typedef typename MeshType::FaceType       FaceType;
typedef typename MeshType::FacePointer    FacePointer;
typedef typename MeshType::FaceIterator   FaceIterator;


/// \headerfile topology.h vcg/complex/algorithms/update/topology.h

/// \brief Auxiliairy data structure for computing face face adjacency information.
/**
It identifies and edge storing two vertex pointer and a face pointer where it belong.
*/

class PEdge
{
public:

  VertexPointer  v[2];  // the two Vertex pointer are ordered!
  FacePointer    f;     // the face where this edge belong
  int            z;     // index in [0..2] of the edge of the face

  PEdge() {}

  void Set( FacePointer  pf, const int nz )
  {
    assert(pf!=0);
    assert(nz>=0);
    assert(nz<pf->VN());

    v[0] = pf->V(nz);
    v[1] = pf->V(pf->Next(nz));
    assert(v[0] != v[1]); // The face pointed by 'f' is Degenerate (two coincident vertexes)

    if( v[0] > v[1] ) std::swap(v[0],v[1]);
    f    = pf;
    z    = nz;
  }

  inline bool operator <  ( const PEdge & pe ) const
  {
    if( v[0]<pe.v[0] ) return true;
    else if( v[0]>pe.v[0] ) return false;
    else return v[1] < pe.v[1];
  }

  inline bool operator == ( const PEdge & pe ) const
  {
    return v[0]==pe.v[0] && v[1]==pe.v[1];
  }
  /// Convert from edge barycentric coord to the face baricentric coord a point on the current edge.
  /// Face barycentric coordinates are relative to the edge face.
  inline Point3<ScalarType> EdgeBarycentricToFaceBarycentric(ScalarType u) const
  {
    Point3<ScalarType> interp(0,0,0);
    interp[ this->z     ] = u;
    interp[(this->z+1)%3] = 1.0f-u;
    return interp;
  }
};

// Fill a vector with all the edges of the mesh.
// each edge is stored in the vector the number of times that it appears in the mesh, with the referring face.
// optionally it can skip the faux edges (to retrieve only the real edges of a triangulated polygonal mesh)

static void FillEdgeVector(MeshType &m, std::vector<PEdge> &e, bool includeFauxEdge=true)
{
    FaceIterator pf;
    typename std::vector<PEdge>::iterator p;

    // Alloco il vettore ausiliario
    //e.resize(m.fn*3);
    FaceIterator fi;
    int n_edges = 0;
    for(fi = m.face.begin(); fi != m.face.end(); ++fi) if(! (*fi).IsD()) n_edges+=(*fi).VN();
    e.resize(n_edges);

    p = e.begin();
    for(pf=m.face.begin();pf!=m.face.end();++pf)
        if( ! (*pf).IsD() )
            for(int j=0;j<(*pf).VN();++j)
            if(includeFauxEdge || !(*pf).IsF(j))
                {
                    (*p).Set(&(*pf),j);
                    ++p;
                }

    if(includeFauxEdge) assert(p==e.end());
    else e.resize(p-e.begin());
}

static void FillUniqueEdgeVector(MeshType &m, std::vector<PEdge> &Edges, bool includeFauxEdge=true)
{
    FillEdgeVector(m,Edges,includeFauxEdge);
    sort(Edges.begin(), Edges.end());		// Lo ordino per vertici

    typename std::vector< PEdge>::iterator newEnd = std::unique(Edges.begin(), Edges.end());

    Edges.resize(newEnd-Edges.begin());
}

/*! \brief Initialize the edge vector all the edges that can be inferred from current face vector, setting up all the current adjacency relations
 *
 *
 */

static void AllocateEdge(MeshType &m)
{
  // Delete all the edges (if any)
  for(EdgeIterator ei=m.edge.begin();ei!=m.edge.end();++ei)
        tri::Allocator<MeshType>::DeleteEdge(m,*ei);
  tri::Allocator<MeshType>::CompactEdgeVector(m);

  // Compute and add edges
  std::vector<PEdge> Edges;
  FillUniqueEdgeVector(m,Edges);
  assert(m.edge.empty());
  tri::Allocator<MeshType>::AddEdges(m,Edges.size());
  assert(m.edge.size()==Edges.size());

  // Setup adjacency relations
  if(tri::HasEVAdjacency(m))
  {
    for(size_t i=0; i< Edges.size(); ++i)
    {
      m.edge[i].V(0) = Edges[i].v[0];
      m.edge[i].V(1) = Edges[i].v[1];
    }
  }

  if(tri::HasEFAdjacency(m)) // Note it is an unordered relation.
  {
    for(size_t i=0; i< Edges.size(); ++i)
    {
      std::vector<FacePointer> fpVec;
      std::vector<int> eiVec;
      face::EFStarFF(Edges[i].f,Edges[i].z,fpVec,eiVec);
      m.edge[i].EFp() = Edges[i].f;
      m.edge[i].EFi() = Edges[i].z;
    }
  }

  if(tri::HasFEAdjacency(m))
  {
    for(size_t i=0; i< Edges.size(); ++i)
    {
      std::vector<FacePointer> fpVec;
      std::vector<int> eiVec;
      face::EFStarFF(Edges[i].f,Edges[i].z,fpVec,eiVec);
      for(size_t j=0;j<fpVec.size();++j)
        fpVec[j]->FEp(eiVec[j])=&(m.edge[i]);

//      Edges[i].f->FE(Edges[i].z) = &(m.edge[i]);
//      Connect in loop the non manifold
//      FaceType* fpit=fp;
//      int eit=ei;

//      do
//      {
//        faceVec.push_back(fpit);
//        indVed.push_back(eit);
//        FaceType *new_fpit = fpit->FFp(eit);
//        int       new_eit  = fpit->FFi(eit);
//        fpit=new_fpit;
//        eit=new_eit;
//      } while(fpit != fp);


//      m.edge[i].EFp() = Edges[i].f;
//      m.edge[i].EFi() = ;
    }
  }

}

/// \brief Update the Face-Face topological relation by allowing to retrieve for each face what other faces shares their edges.
static void FaceFace(MeshType &m)
{
  RequireFFAdjacency(m);
  if( m.fn == 0 ) return;

  std::vector<PEdge> e;
  FillEdgeVector(m,e);
  sort(e.begin(), e.end());							// Lo ordino per vertici

  int ne = 0;											// Numero di edge reali

  typename std::vector<PEdge>::iterator pe,ps;
  ps = e.begin();pe=e.begin();
  //for(ps = e.begin(),pe=e.begin();pe<=e.end();++pe)	// Scansione vettore ausiliario
  do
  {
    if( pe==e.end() || !(*pe == *ps) )					// Trovo blocco di edge uguali
    {
      typename std::vector<PEdge>::iterator q,q_next;
      for (q=ps;q<pe-1;++q)						// Scansione facce associate
      {
        assert((*q).z>=0);
        //assert((*q).z< 3);
        q_next = q;
        ++q_next;
        assert((*q_next).z>=0);
        assert((*q_next).z< (*q_next).f->VN());
        (*q).f->FFp(q->z) = (*q_next).f;				// Collegamento in lista delle facce
        (*q).f->FFi(q->z) = (*q_next).z;
      }
      assert((*q).z>=0);
      assert((*q).z< (*q).f->VN());
      (*q).f->FFp((*q).z) = ps->f;
      (*q).f->FFi((*q).z) = ps->z;
      ps = pe;
      ++ne;										// Aggiorno il numero di edge
    }
    if(pe==e.end()) break;
    ++pe;
  } while(true);
}

/// \brief Update the Vertex-Face topological relation.
/**
The function allows to retrieve for each vertex the list of faces sharing this vertex.
After this call all the VF component are initialized. Isolated vertices have a null list of faces.
\sa vcg::vertex::VFAdj
\sa vcg::face::VFAdj
*/

static void VertexFace(MeshType &m)
{
  RequireVFAdjacency(m);

  for(VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi)
  {
    (*vi).VFp() = 0;
    (*vi).VFi() = 0; // note that (0,-1) means uninitiazlied while 0,0 is the valid initialized values for isolated vertices.
  }

  for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi)
    if( ! (*fi).IsD() )
    {
      for(int j=0;j<(*fi).VN();++j)
      {
        (*fi).VFp(j) = (*fi).V(j)->VFp();
        (*fi).VFi(j) = (*fi).V(j)->VFi();
        (*fi).V(j)->VFp() = &(*fi);
        (*fi).V(j)->VFi() = j;
      }
    }
}


/// \headerfile topology.h vcg/complex/algorithms/update/topology.h

/// \brief Auxiliairy data structure for computing face face adjacency information.
/**
It identifies and edge storing two vertex pointer and a face pointer where it belong.
*/

class PEdgeTex
{
public:

  typename FaceType::TexCoordType  v[2];		// the two TexCoord are ordered!
  FacePointer    f;                       // the face where this edge belong
  int      z;				      // index in [0..2] of the edge of the face

  PEdgeTex() {}

  void Set( FacePointer  pf, const int nz )
  {
    assert(pf!=0);
    assert(nz>=0);
    assert(nz<3);

    v[0] = pf->WT(nz);
    v[1] = pf->WT(pf->Next(nz));
    assert(v[0] != v[1]); // The face pointed by 'f' is Degenerate (two coincident vertexes)

    if( v[1] < v[0] ) std::swap(v[0],v[1]);
    f    = pf;
    z    = nz;
  }

  inline bool operator <  ( const PEdgeTex & pe ) const
  {
    if( v[0]<pe.v[0] ) return true;
    else if( pe.v[0]<v[0] ) return false;
    else return v[1] < pe.v[1];
  }
  inline bool operator == ( const PEdgeTex & pe ) const
  {
    return (v[0]==pe.v[0]) && (v[1]==pe.v[1]);
  }
  inline bool operator != ( const PEdgeTex & pe ) const
  {
    return (v[0]!=pe.v[0]) || (v[1]!=pe.v[1]);
  }

};


/// \brief Update the Face-Face topological relation so that it reflects the per-wedge texture connectivity

/**
Using this function two faces are adjacent along the FF relation IFF the two faces have matching texture coords along the involved edge.
In other words F1->FFp(i) == F2 iff F1 and F2 have the same tex coords along edge i
*/

static void FaceFaceFromTexCoord(MeshType &m)
{
  RequireFFAdjacency(m);
  RequirePerFaceWedgeTexCoord(m);

  std::vector<PEdgeTex> e;
    FaceIterator pf;
    typename std::vector<PEdgeTex>::iterator p;

    if( m.fn == 0 ) return;

//	e.resize(m.fn*3);								// Alloco il vettore ausiliario
    FaceIterator fi;
    int n_edges = 0;
    for(fi = m.face.begin(); fi != m.face.end(); ++fi) if(! (*fi).IsD()) n_edges+=(*fi).VN();
    e.resize(n_edges);

    p = e.begin();
    for(pf=m.face.begin();pf!=m.face.end();++pf)			// Lo riempio con i dati delle facce
        if( ! (*pf).IsD() )
            for(int j=0;j<(*pf).VN();++j)
            {
                if( (*pf).WT(j) != (*pf).WT((*pf).Next(j)))
                     {
                        (*p).Set(&(*pf),j);
                        ++p;
                     }
            }

    e.resize(p-e.begin());   // remove from the end of the edge vector the unitiailized ones
    //assert(p==e.end()); // this formulation of the assert argument is not really correct, will crash on visual studio
    sort(e.begin(), e.end());

    int ne = 0;											// number of real edges
    typename std::vector<PEdgeTex>::iterator pe,ps;
    ps = e.begin();pe=e.begin();
    //for(ps = e.begin(),pe=e.begin();pe<=e.end();++pe)	// Scansione vettore ausiliario
    do
    {
        if( pe==e.end() || (*pe) != (*ps) )					// Trovo blocco di edge uguali
        {
            typename std::vector<PEdgeTex>::iterator q,q_next;
            for (q=ps;q<pe-1;++q)						// Scansione facce associate
            {
                assert((*q).z>=0);
                assert((*q).z< 3);
                q_next = q;
                ++q_next;
                assert((*q_next).z>=0);
                assert((*q_next).z< (*q_next).f->VN());
                (*q).f->FFp(q->z) = (*q_next).f;				// Collegamento in lista delle facce
                (*q).f->FFi(q->z) = (*q_next).z;
            }
            assert((*q).z>=0);
            assert((*q).z< (*q).f->VN());
            (*q).f->FFp((*q).z) = ps->f;
            (*q).f->FFi((*q).z) = ps->z;
            ps = pe;
            ++ne;										// Aggiorno il numero di edge
        }
        if(pe==e.end()) break;
        ++pe;
    } while(true);
}





/// \brief Test correctness of VFtopology
static void TestVertexFace(MeshType &m)
{
    SimpleTempData<typename MeshType::VertContainer, int > numVertex(m.vert,0);

  assert(tri::HasPerVertexVFAdjacency(m));

    FaceIterator fi;
    for(fi=m.face.begin();fi!=m.face.end();++fi)
    {
        if (!(*fi).IsD())
        {
            numVertex[(*fi).V0(0)]++;
            numVertex[(*fi).V1(0)]++;
            numVertex[(*fi).V2(0)]++;
        }
    }

    VertexIterator vi;
    vcg::face::VFIterator<FaceType> VFi;

    for(vi=m.vert.begin();vi!=m.vert.end();++vi)
    {
        if (!vi->IsD())
        if(vi->VFp()!=0) // unreferenced vertices MUST have VF == 0;
        {
            int num=0;
            assert(vi->VFp() >= &*m.face.begin());
            assert(vi->VFp() <= &m.face.back());
            VFi.f=vi->VFp();
            VFi.z=vi->VFi();
            while (!VFi.End())
            {
                num++;
                assert(!VFi.F()->IsD());
                assert((VFi.F()->V(VFi.I()))==&(*vi));
                ++VFi;
            }
            assert(num==numVertex[&(*vi)]);
        }
    }
}

/// \brief Test correctness of FFtopology (only for 2Manifold Meshes!)
static void TestFaceFace(MeshType &m)
{
  assert(HasFFAdjacency(m));

  for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi)
    {
    if (!fi->IsD())
        {
      for (int i=0;i<(*fi).VN();i++)
            {
        FaceType *ffpi=fi->FFp(i);
        int e=fi->FFi(i);
        //invariant property of FF topology for two manifold meshes
        assert(ffpi->FFp(e) == &(*fi));
        assert(ffpi->FFi(e) == i);

        // Test that the two faces shares the same edge
        // Vertices of the i-th edges of the first face
        VertexPointer v0i= fi->V0(i);
        VertexPointer v1i= fi->V1(i);
        // Vertices of the corresponding edge on the other face
        VertexPointer ffv0i= ffpi->V0(e);
        VertexPointer ffv1i= ffpi->V1(e);

        assert( (ffv0i==v0i) || (ffv0i==v1i) );
        assert( (ffv1i==v0i) || (ffv1i==v1i) );
            }

        }
    }
}

/// Auxiliairy data structure for computing edge edge adjacency information.
/// It identifies an edge storing a vertex pointer and a edge pointer where it belong.
class PVertexEdge
{
public:

  VertexPointer  v;		// the two Vertex pointer are ordered!
  EdgePointer    e;		  // the edge where this vertex belong
  int      z;				      // index in [0..1] of the vertex of the edge

  PVertexEdge(  ) {}
  PVertexEdge( EdgePointer  pe, const int nz )
{
  assert(pe!=0);
  assert(nz>=0);
  assert(nz<2);

  v= pe->V(nz);
  e    = pe;
  z    = nz;
}
inline bool operator  <  ( const PVertexEdge & pe ) const { return ( v<pe.v ); }
inline bool operator ==  ( const PVertexEdge & pe ) const { return ( v==pe.v ); }
inline bool operator !=  ( const PVertexEdge & pe ) const { return ( v!=pe.v ); }
};



static void EdgeEdge(MeshType &m)
{
  RequireEEAdjacency(m);
  std::vector<PVertexEdge> v;
  if( m.en == 0 ) return;

//  printf("Inserting Edges\n");
  for(EdgeIterator pf=m.edge.begin(); pf!=m.edge.end(); ++pf)			// Lo riempio con i dati delle facce
    if( ! (*pf).IsD() )
      for(int j=0;j<2;++j)
      {
//        printf("egde %i ind %i (%i %i)\n",tri::Index(m,&*pf),j,tri::Index(m,pf->V(0)),tri::Index(m,pf->V(1)));
        v.push_back(PVertexEdge(&*pf,j));
      }

//  printf("en = %i (%i)\n",m.en,m.edge.size());
  sort(v.begin(), v.end());							// Lo ordino per vertici

  int ne = 0;											// Numero di edge reali

  typename std::vector<PVertexEdge>::iterator pe,ps;
  // for(ps = v.begin(),pe=v.begin();pe<=v.end();++pe)	// Scansione vettore ausiliario
  ps = v.begin();pe=v.begin();
  do
  {
//    printf("v %i -> e %i\n",tri::Index(m,(*ps).v),tri::Index(m,(*ps).e));
    if( pe==v.end() || !(*pe == *ps) )					// Trovo blocco di edge uguali
    {
      typename std::vector<PVertexEdge>::iterator q,q_next;
      for (q=ps;q<pe-1;++q)						// Scansione edge associati
      {
        assert((*q).z>=0);
        assert((*q).z< 2);
        q_next = q;
        ++q_next;
        assert((*q_next).z>=0);
        assert((*q_next).z< 2);
        (*q).e->EEp(q->z) = (*q_next).e;				// Collegamento in lista delle facce
        (*q).e->EEi(q->z) = (*q_next).z;
      }
      assert((*q).z>=0);
      assert((*q).z< 2);
      (*q).e->EEp((*q).z) = ps->e;
      (*q).e->EEi((*q).z) = ps->z;
      ps = pe;
      ++ne;										// Aggiorno il numero di edge
    }
    if(pe==v.end()) break;
    ++pe;
   } while(true);
}

static void VertexEdge(MeshType &m)
{
  RequireVEAdjacency(m);

  VertexIterator vi;
  EdgeIterator ei;

  for(vi=m.vert.begin();vi!=m.vert.end();++vi)
  {
    (*vi).VEp() = 0;
    (*vi).VEi() = 0;
  }

  for(ei=m.edge.begin();ei!=m.edge.end();++ei)
  if( ! (*ei).IsD() )
  {
    for(int j=0;j<2;++j)
    {
      (*ei).VEp(j) = (*ei).V(j)->VEp();
      (*ei).VEi(j) = (*ei).V(j)->VEi();
      (*ei).V(j)->VEp() = &(*ei);
      (*ei).V(j)->VEi() = j;
    }
  }
}

}; // end class

}	// End namespace
}	// End namespace


#endif
