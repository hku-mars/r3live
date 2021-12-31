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
#ifndef __VCG_TRI_UPDATE_FLAGS
#define __VCG_TRI_UPDATE_FLAGS

#include <vcg/simplex/face/pos.h>

namespace vcg {
namespace tri {
/// \ingroup trimesh

/// \headerfile flag.h vcg/complex/algorithms/update/flag.h

/// \brief Management, updating and computation of per-vertex and per-face flags (like border flags).

/**
This class is used to compute or update some of the flags that can be stored in the mesh components. For now just Border flags (e.g. the flag that tells if a given edge of a face belong to a border of the mesh or not).
*/

template <class UpdateMeshType>
class UpdateFlags
{

public:
  typedef UpdateMeshType MeshType;
  typedef typename MeshType::ScalarType     ScalarType;
  typedef typename MeshType::VertexType     VertexType;
  typedef typename MeshType::VertexPointer  VertexPointer;
  typedef typename MeshType::VertexIterator VertexIterator;
  typedef typename MeshType::EdgeType       EdgeType;
  typedef typename MeshType::EdgePointer    EdgePointer;
  typedef typename MeshType::EdgeIterator   EdgeIterator;
  typedef typename MeshType::FaceType       FaceType;
  typedef typename MeshType::FacePointer    FacePointer;
  typedef typename MeshType::FaceIterator   FaceIterator;

  /// \brief Reset all the mesh flags (vertexes edge faces) setting everithing to zero (the default value for flags)

  static void Clear(MeshType &m)
  {
    if(HasPerVertexFlags(m) )
      for(VertexIterator vi=m.vert.begin(); vi!=m.vert.end(); ++vi)
        (*vi).Flags() = 0;
    if(HasPerEdgeFlags(m) )
      for(EdgeIterator ei=m.edge.begin(); ei!=m.edge.end(); ++ei)
        (*ei).Flags() = 0;
    if(HasPerFaceFlags(m) )
      for(FaceIterator fi=m.face.begin(); fi!=m.face.end(); ++fi)
        (*fi).Flags() = 0;
  }

  static void VertexClear(MeshType &m, unsigned int FlagMask = 0xffffffff)
  {
    RequirePerVertexFlags(m);
    int andMask = ~FlagMask;
    for(VertexIterator vi=m.vert.begin(); vi!=m.vert.end(); ++vi)
      if(!(*vi).IsD()) (*vi).Flags() &= andMask ;
  }

  static void EdgeClear(MeshType &m, unsigned int FlagMask = 0xffffffff)
  {
    RequirePerEdgeFlags(m);
    int andMask = ~FlagMask;
    for(EdgeIterator ei=m.edge.begin(); ei!=m.edge.end(); ++ei)
      if(!(*ei).IsD()) (*ei).Flags() &= andMask ;
  }

  static void FaceClear(MeshType &m, unsigned int FlagMask = 0xffffffff)
  {
    RequirePerFaceFlags(m);
    int andMask = ~FlagMask;
    for(FaceIterator fi=m.face.begin(); fi!=m.face.end(); ++fi)
      if(!(*fi).IsD()) (*fi).Flags() &= andMask ;
  }

  static void VertexSet(MeshType &m, unsigned int FlagMask)
  {
    RequirePerVertexFlags(m);
    for(VertexIterator vi=m.vert.begin(); vi!=m.vert.end(); ++vi)
      if(!(*vi).IsD()) (*vi).Flags() |= FlagMask ;
  }

  static void EdgeSet(MeshType &m, unsigned int FlagMask)
  {
    RequirePerEdgeFlags(m);
    for(EdgeIterator ei=m.edge.begin(); ei!=m.edge.end(); ++ei)
      if(!(*ei).IsD()) (*ei).Flags() |= FlagMask ;
  }

  static void FaceSet(MeshType &m, unsigned int FlagMask)
  {
    RequirePerFaceFlags(m);
    for(FaceIterator fi=m.face.begin(); fi!=m.face.end(); ++fi)
      if(!(*fi).IsD()) (*fi).Flags() |= FlagMask ;
  }



  static void VertexClearV(MeshType &m) { VertexClear(m,VertexType::VISITED);}
  static void VertexClearS(MeshType &m) { VertexClear(m,VertexType::SELECTED);}
  static void VertexClearB(MeshType &m) { VertexClear(m,VertexType::BORDER);}
  static void EdgeClearV(MeshType &m) { EdgeClear(m,EdgeType::VISITED);}
  static void FaceClearV(MeshType &m) { FaceClear(m,FaceType::VISITED);}
  static void FaceClearB(MeshType &m) { FaceClear(m,FaceType::BORDER012);}
  static void FaceClearS(MeshType &m) {FaceClear(m,FaceType::SELECTED);}
  static void FaceClearF(MeshType &m) { FaceClear(m,FaceType::FAUX012);}
  static void FaceClearCreases(MeshType &m) { FaceClear(m,FaceType::CREASE0);
                                              FaceClear(m,FaceType::CREASE1);
                                              FaceClear(m,FaceType::CREASE2);
                                            }

  static void EdgeSetV(MeshType &m) { EdgeSet(m,EdgeType::VISITED);}
  static void VertexSetV(MeshType &m) { VertexSet(m,VertexType::VISITED);}
  static void VertexSetB(MeshType &m) { VertexSet(m,VertexType::BORDER);}
  static void FaceSetV(MeshType &m) { FaceSet(m,FaceType::VISITED);}
  static void FaceSetB(MeshType &m) { FaceSet(m,FaceType::BORDER);}
  static void FaceSetF(MeshType &m) { FaceSet(m,FaceType::FAUX012);}

  /// \brief Compute the border flags for the faces using the Face-Face Topology.

  /**
 \warning Obviously it assumes that the topology has been correctly computed (see: UpdateTopology::FaceFace )
*/
  static void FaceBorderFromFF(MeshType &m)
  {
    RequirePerFaceFlags(m);
    RequireFFAdjacency(m);

    for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi)if(!(*fi).IsD())
      for(int j=0;j<3;++j)
      {
        if(face::IsBorder(*fi,j)) (*fi).SetB(j);
        else (*fi).ClearB(j);
      }
  }


  static void FaceBorderFromVF(MeshType &m)
  {
    RequirePerFaceFlags(m);
    RequireVFAdjacency(m);

    FaceClearB(m);
    int visitedBit=VertexType::NewBitFlag();

    // Calcolo dei bordi
    // per ogni vertice vi si cercano i vertici adiacenti che sono toccati da una faccia sola
    // (o meglio da un numero dispari di facce)

    const int BORDERFLAG[3]={FaceType::BORDER0, FaceType::BORDER1, FaceType::BORDER2};

    for(VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi)
      if(!(*vi).IsD())
      {
        for(face::VFIterator<FaceType> vfi(&*vi) ; !vfi.End(); ++vfi )
        {
          vfi.f->V1(vfi.z)->ClearUserBit(visitedBit);
          vfi.f->V2(vfi.z)->ClearUserBit(visitedBit);
        }
        for(face::VFIterator<FaceType> vfi(&*vi) ; !vfi.End(); ++vfi )
        {
          if(vfi.f->V1(vfi.z)->IsUserBit(visitedBit))  vfi.f->V1(vfi.z)->ClearUserBit(visitedBit);
          else vfi.f->V1(vfi.z)->SetUserBit(visitedBit);
          if(vfi.f->V2(vfi.z)->IsUserBit(visitedBit))  vfi.f->V2(vfi.z)->ClearUserBit(visitedBit);
          else vfi.f->V2(vfi.z)->SetUserBit(visitedBit);
        }
        for(face::VFIterator<FaceType> vfi(&*vi) ; !vfi.End(); ++vfi )
        {
          if(vfi.f->V(vfi.z)< vfi.f->V1(vfi.z)  &&  vfi.f->V1(vfi.z)->IsUserBit(visitedBit))
            vfi.f->Flags() |= BORDERFLAG[vfi.z];
          if(vfi.f->V(vfi.z)< vfi.f->V2(vfi.z)  &&  vfi.f->V2(vfi.z)->IsUserBit(visitedBit))
            vfi.f->Flags() |= BORDERFLAG[(vfi.z+2)%3];
        }
      }
    VertexType::DeleteBitFlag(visitedBit);
  }


  class EdgeSorter
  {
  public:

    VertexPointer v[2];		// Puntatore ai due vertici (Ordinati)
    FacePointer    f;				// Puntatore alla faccia generatrice
    int      z;				// Indice dell'edge nella faccia

    EdgeSorter() {} // Nothing to do


    void Set( const FacePointer pf, const int nz )
    {
      assert(pf!=0);
      assert(nz>=0);
      assert(nz<3);

      v[0] = pf->V(nz);
      v[1] = pf->V((nz+1)%3);
      assert(v[0] != v[1]);

      if( v[0] > v[1] ) std::swap(v[0],v[1]);
      f    = pf;
      z    = nz;
    }

    inline bool operator <  ( const EdgeSorter & pe ) const {
      if( v[0]<pe.v[0] ) return true;
      else if( v[0]>pe.v[0] ) return false;
      else return v[1] < pe.v[1];
    }

    inline bool operator == ( const EdgeSorter & pe ) const
    {
      return v[0]==pe.v[0] && v[1]==pe.v[1];
    }
    inline bool operator != ( const EdgeSorter & pe ) const
    {
      return v[0]!=pe.v[0] || v[1]!=pe.v[1];
    }

  };


  // versione minimale che non calcola i complex flag.
  static void VertexBorderFromNone(MeshType &m)
  {
    RequirePerVertexFlags(m);

    std::vector<EdgeSorter> e;
    typename UpdateMeshType::FaceIterator pf;
    typename std::vector<EdgeSorter>::iterator p;

    if( m.fn == 0 )
      return;

    e.resize(m.fn*3);								// Alloco il vettore ausiliario
    p = e.begin();
    for(pf=m.face.begin();pf!=m.face.end();++pf)			// Lo riempio con i dati delle facce
      if( ! (*pf).IsD() )
        for(int j=0;j<3;++j)
        {
          (*p).Set(&(*pf),j);
          (*pf).ClearB(j);
          ++p;
        }
    assert(p==e.end());
    sort(e.begin(), e.end());							// Lo ordino per vertici

    typename std::vector<EdgeSorter>::iterator pe,ps;
    for(ps = e.begin(), pe = e.begin(); pe < e.end(); ++pe)	// Scansione vettore ausiliario
    {
      if( pe==e.end() ||  *pe != *ps )					// Trovo blocco di edge uguali
      {
        if(pe-ps==1) 	{
          ps->v[0]->SetB();
          ps->v[1]->SetB();
        }/* else
          if(pe-ps!=2)  {  // not twomanyfold!
            for(;ps!=pe;++ps) {
              ps->v[0]->SetB(); // Si settano border anche i complex.
              ps->v[1]->SetB();
            }
          }*/
        ps = pe;
      }
    }
  }

  /// Computes per-face border flags without requiring any kind of topology
  /// It has a O(fn log fn) complexity.
  static void FaceBorderFromNone(MeshType &m)
  {
    RequirePerFaceFlags(m);

    std::vector<EdgeSorter> e;
    typename UpdateMeshType::FaceIterator pf;
    typename std::vector<EdgeSorter>::iterator p;

    for(VertexIterator v=m.vert.begin();v!=m.vert.end();++v)
      (*v).ClearB();

    if( m.fn == 0 )
      return;

    FaceIterator fi;
    int n_edges = 0;
    for(fi = m.face.begin(); fi != m.face.end(); ++fi) if(! (*fi).IsD()) n_edges+=(*fi).VN();
    e.resize(n_edges);

    p = e.begin();
    for(pf=m.face.begin();pf!=m.face.end();++pf)			// Lo riempio con i dati delle facce
      if( ! (*pf).IsD() )
        for(int j=0;j<(*pf).VN();++j)
        {
          (*p).Set(&(*pf),j);
          (*pf).ClearB(j);
          ++p;
        }
    assert(p==e.end());
    sort(e.begin(), e.end());							// Lo ordino per vertici

    typename std::vector<EdgeSorter>::iterator pe,ps;
    ps = e.begin();pe=e.begin();
    do
    {
      if( pe==e.end() ||  *pe != *ps )					// Trovo blocco di edge uguali
      {
        if(pe-ps==1) 	{
          ps->f->SetB(ps->z);
        } /*else
          if(pe-ps!=2)  {  // Caso complex!!
            for(;ps!=pe;++ps)
              ps->f->SetB(ps->z); // Si settano border anche i complex.
          }*/
        ps = pe;
      }
      if(pe==e.end()) break;
      ++pe;
    } while(true);
    //	TRACE("found %i border (%i complex) on %i edges\n",nborder,ncomplex,ne);
  }

  /// Compute the PerVertex Border flag deriving it from the border flag of faces
  static void VertexBorderFromFace(MeshType &m)
  {
    RequirePerFaceFlags(m);
    RequirePerVertexFlags(m);
    for(VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi)
      (*vi).ClearB();

    for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi)
      if(!(*fi).IsD())
      {
        for(int z=0;z<(*fi).VN();++z)
          if( (*fi).IsB(z) )
          {
            (*fi).V(z)->SetB();
            (*fi).V((*fi).Next(z))->SetB();
          }
      }
  }


  /// \brief Marks feature edges according to two signed dihedral angles.
  /// Actually it marks as fauxedges all the non feature edges,
  /// e.g. the edge such that the signed dihedral angle between the normal of two faces sharing it, is between the two given thresholds.
  /// In this way all the near planar edges are marked as Faux Edges (e.g. edges to be ignored)
  /// Note that it uses the signed dihedral angle convention (negative for concave edges and positive for convex ones);
  static void FaceFauxSignedCrease(MeshType &m, float AngleRadNeg, float AngleRadPos )
  {
    RequirePerFaceFlags(m);
    RequireFFAdjacency(m);
    //initially Nothing is faux (e.g all crease)
    FaceClearF(m);
    // Then mark faux only if the signed angle is the range.
    for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi) if(!(*fi).IsD())
    {
      for(int z=0;z<(*fi).VN();++z)
      {
        if(!face::IsBorder(*fi,z) )
        {
          ScalarType angle = DihedralAngleRad(*fi,z);
          if(angle>AngleRadNeg && angle<AngleRadPos)
            (*fi).SetF(z);
        }
      }
    }
  }


  /// \brief Marks feature edges according to a given angle
  /// Actually it marks as fauxedges all the non feature edges,
  /// e.g. the edge such that the angle between the normal of two faces sharing it is less than the given threshold.
  /// In this way all the near planar edges are marked as Faux Edges (e.g. edges to be ignored)
  static void FaceFauxCrease(MeshType &m,float AngleRad)
  {
    RequirePerFaceFlags(m);
    RequireFFAdjacency(m);
    RequirePerFaceNormal(m);

    typename MeshType::FaceIterator f;

    //initially everything is faux (e.g all internal)
    FaceSetF(m);
    for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi)
    {
      if(!(*fi).IsD())
      {
        for(int z=0;z<(*fi).VN();++z)
        {
          if( face::IsBorder(*fi,z) )  (*fi).ClearF(z);
          else
          {
            if(Angle((*fi).N(), (*fi).FFp(z)->N()) > AngleRad)
              (*fi).ClearF(z);
          }
        }
      }
    }
  }

}; // end class

}	// End namespace tri
}	// End namespace vcg


#endif
