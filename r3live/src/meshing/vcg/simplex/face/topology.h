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

#ifndef _VCG_FACE_TOPOLOGY
#define _VCG_FACE_TOPOLOGY

#include <vcg/simplex/face/pos.h>
#include <set>

namespace vcg {
namespace face {
/** \addtogroup face */
/*@{*/

/** Return a boolean that indicate if the face is complex.
    @param j Index of the edge
    @return true se la faccia e' manifold, false altrimenti
*/
template <class FaceType>
inline bool IsManifold( FaceType const & f, const int j )
{
  assert(f.cFFp(j) != 0); // never try to use this on uncomputed topology
  if(FaceType::HasFFAdjacency())
      return ( f.cFFp(j) == &f || &f == f.cFFp(j)->cFFp(f.cFFi(j)) );
  else
    return true;
}

/** Return a boolean that indicate if the j-th edge of the face is a border.
    @param j Index of the edge
    @return true if j is an edge of border, false otherwise
*/
template <class FaceType>
inline bool IsBorder(FaceType const & f,  const int j )
{
  if(FaceType::HasFFAdjacency())
      return f.cFFp(j)==&f;
    //return f.IsBorder(j);

  assert(0);
  return true;
}

/*! \brief Compute the signed dihedral angle between the normals of two adjacent faces
 *
 * The angle between the normal is signed according to the concavity/convexity of the
 * dihedral angle: negative if the edge shared between the two faces is concave, positive otherwise.
 * The surface it is assumend to be oriented.
 * It simply use the projection of  the opposite vertex onto the plane of the other one.
 * It does not assume anything on face normals.
*
*     v0 ___________ vf1
*       |\          |
*       |  \i1   f1 |
*       |    \      |
*       |f0  i0\    |
*       |        \  |
*       |__________\|
*    vf0             v1
*/

template <class FaceType>
inline typename FaceType::ScalarType DihedralAngleRad(FaceType & f,  const int i )
{
  typedef typename FaceType::ScalarType ScalarType;
  typedef typename FaceType::CoordType CoordType;
  typedef typename FaceType::VertexType VertexType;

  FaceType *f0 = &f;
  FaceType *f1 = f.FFp(i);
  int i0=i;
  int i1=f.FFi(i);
  VertexType *vf0 = f0->V2(i0);
  VertexType *vf1 = f1->V2(i1);

  CoordType n0 = NormalizedNormal(*f0);
  CoordType n1 = NormalizedNormal(*f1);
  ScalarType off0 = n0*vf0->P();
  ScalarType off1 = n1*vf1->P();

  ScalarType dist01 = off0 - n0*vf1->P();
  ScalarType dist10 = off1 - n1*vf0->P();

  // just to be sure use the sign of the largest in absolute value;
  ScalarType sign;
  if(fabs(dist01) > fabs(dist10)) sign = dist01;
  else sign=dist10;

  ScalarType angleRad=Angle(f0->N(),f1->N());

  if(sign > 0 ) return angleRad;
  else return -angleRad;
}

/// Count border edges of the face
template <class FaceType>
inline int BorderCount(FaceType const & f)
{
  if(FaceType::HasFFAdjacency())
  {
    int t = 0;
      if( IsBorder(f,0) ) ++t;
      if( IsBorder(f,1) ) ++t;
      if( IsBorder(f,2) ) ++t;
      return t;
  }
    else 	return 3;
}


/// Counts the number of incident faces in a complex edge
template <class FaceType>
inline int ComplexSize(FaceType & f, const int e)
{
  if(FaceType::HasFFAdjacency())
  {
    if(face::IsBorder<FaceType>(f,e))  return 1;
    if(face::IsManifold<FaceType>(f,e)) return 2;

    // Non manifold case
    Pos< FaceType > fpos(&f,e);
    int cnt=0;
    do
    {
          fpos.NextF();
      assert(!fpos.IsBorder());
      assert(!fpos.IsManifold());
          ++cnt;
      }
      while(fpos.f!=&f);
    assert (cnt>2);
      return cnt;
  }
  assert(0);
    return 2;
}


/** This function check the FF topology correctness for an edge of a face.
    It's possible to use it also in non-two manifold situation.
        The function cannot be applicated if the adjacencies among faces aren't defined.
        @param f the face to be checked
        @param e Index of the edge to be checked
*/
template <class FaceType>
bool FFCorrectness(FaceType & f, const int e)
{
  if(f.FFp(e)==0) return false;   // Not computed or inconsistent topology

  if(f.FFp(e)==&f) // Border
  {
   if(f.FFi(e)==e) return true;
   else return false;
  }

  if(f.FFp(e)->FFp(f.FFi(e))==&f) // plain two manifold
  {
    if(f.FFp(e)->FFi(f.FFi(e))==e) return true;
    else return false;
  }

  // Non Manifold Case
  // all the faces must be connected in a loop.

  Pos< FaceType > curFace(&f,e);  // Build the half edge
    int cnt=0;
  do
    {
        if(curFace.IsManifold()) return false;
        if(curFace.IsBorder()) return false;
        curFace.NextF();
        cnt++;
    assert(cnt<100);
    }
  while ( curFace.f != &f);
  return true;
}


/** This function detach the face from the adjacent face via the edge e.
    It's possible to use  this function it ONLY in non-two manifold situation.
        The function cannot be applicated if the adjacencies among faces aren't defined.
        @param f the face to be detached
        @param e Index of the edge to be detached
        \note it updates border flag and faux flags (the detached edge has it border bit flagged and faux bit cleared)
*/
template <class FaceType>
void FFDetachManifold(FaceType & f, const int e)
{
    assert(FFCorrectness<FaceType>(f,e));
    assert(!IsBorder<FaceType>(f,e));  // Never try to detach a border edge!
    FaceType *ffp = f.FFp(e);
    //int ffi=f.FFp(e);
    int ffi=f.FFi(e);

    f.FFp(e)=&f;
    f.FFi(e)=e;
    ffp->FFp(ffi)=ffp;
    ffp->FFi(ffi)=ffi;

    f.SetB(e);
    f.ClearF(e);
    ffp->SetB(ffi);
    ffp->ClearF(ffi);

    assert(FFCorrectness<FaceType>(f,e));
    assert(FFCorrectness<FaceType>(*ffp,ffi));
}

/** This function detach the face from the adjacent face via the edge e.
    It's possible to use it also in non-two manifold situation.
        The function cannot be applicated if the adjacencies among faces aren't defined.
        @param f the face to be detached
        @param e Index of the edge to be detached
*/

template <class FaceType>
void FFDetach(FaceType & f, const int e)
{
    assert(FFCorrectness<FaceType>(f,e));
    assert(!IsBorder<FaceType>(f,e));  // Never try to detach a border edge!
    int complexity=ComplexSize(f,e);
    (void) complexity;
    assert(complexity>0);

    Pos< FaceType > FirstFace(&f,e);  // Build the half edge
    Pos< FaceType > LastFace(&f,e);  // Build the half edge
    FirstFace.NextF();
    LastFace.NextF();
    int cnt=0;

    // then in case of non manifold face continue to advance LastFace
    // until I find it become the one that
    // preceed the face I want to erase

    while ( LastFace.f->FFp(LastFace.z) != &f)
    {
        assert(ComplexSize(*LastFace.f,LastFace.z)==complexity);
        assert(!LastFace.IsManifold());   // We enter in this loop only if we are on a non manifold edge
        assert(!LastFace.IsBorder());
        LastFace.NextF();
        cnt++;
        assert(cnt<100);
    }

    assert(LastFace.f->FFp(LastFace.z)==&f);
    assert(f.FFp(e)== FirstFace.f);

    // Now we link the last one to the first one, skipping the face to be detached;
    LastFace.f->FFp(LastFace.z) = FirstFace.f;
    LastFace.f->FFi(LastFace.z) = FirstFace.z;
    assert(ComplexSize(*LastFace.f,LastFace.z)==complexity-1);

    // At the end selfconnect the chosen edge to make a border.
    f.FFp(e) = &f;
    f.FFi(e) = e;
    assert(ComplexSize(f,e)==1);

    assert(FFCorrectness<FaceType>(*LastFace.f,LastFace.z));
    assert(FFCorrectness<FaceType>(f,e));
}


/** This function attach the face (via the edge z1) to another face (via the edge z2). It's possible to use it also in non-two manifold situation.
        The function cannot be applicated if the adjacencies among faces aren't define.
        @param z1 Index of the edge
        @param f2 Pointer to the face
        @param z2 The edge of the face f2
*/
template <class FaceType>
void FFAttach(FaceType * &f, int z1, FaceType *&f2, int z2)
{
    //typedef FEdgePosB< FACE_TYPE > ETYPE;
    Pos< FaceType > EPB(f2,z2);
    Pos< FaceType > TEPB;
    TEPB = EPB;
    EPB.NextF();
    while( EPB.f != f2)  //Alla fine del ciclo TEPB contiene la faccia che precede f2
    {
        TEPB = EPB;
        EPB.NextF();
    }
    //Salvo i dati di f1 prima di sovrascrivere
  FaceType *f1prec = f->FFp(z1);
  int z1prec = f->FFi(z1);
    //Aggiorno f1
    f->FFp(z1) = TEPB.f->FFp(TEPB.z);
    f->FFi(z1) = TEPB.f->FFi(TEPB.z);
    //Aggiorno la faccia che precede f2
    TEPB.f->FFp(TEPB.z) = f1prec;
    TEPB.f->FFi(TEPB.z) = z1prec;
}

/** This function attach the face (via the edge z1) to another face (via the edge z2).
        It is not possible to use it also in non-two manifold situation.
        The function cannot be applicated if the adjacencies among faces aren't define.
        @param z1 Index of the edge
        @param f2 Pointer to the face
        @param z2 The edge of the face f2
*/
template <class FaceType>
void FFAttachManifold(FaceType * &f1, int z1, FaceType *&f2, int z2)
{
  assert(IsBorder<FaceType>(*f1,z1));
  assert(IsBorder<FaceType>(*f2,z2));
  assert(f1->V0(z1) == f2->V0(z2) || f1->V0(z1) == f2->V1(z2));
  assert(f1->V1(z1) == f2->V0(z2) || f1->V1(z1) == f2->V1(z2));
  f1->FFp(z1) = f2;
  f1->FFi(z1) = z2;
  f2->FFp(z2) = f1;
  f2->FFi(z2) = z1;
}

// This one should be called only on uniitialized faces.
template <class FaceType>
void FFSetBorder(FaceType * &f1, int z1)
{
  assert(f1->FFp(z1)==0 || IsBorder(*f1,z1));

  f1->FFp(z1)=f1;
  f1->FFi(z1)=z1;
}

template <class FaceType>
void AssertAdj(FaceType & f)
{
  (void)f;
  assert(f.FFp(0)->FFp(f.FFi(0))==&f);
  assert(f.FFp(1)->FFp(f.FFi(1))==&f);
  assert(f.FFp(2)->FFp(f.FFi(2))==&f);

  assert(f.FFp(0)->FFi(f.FFi(0))==0);
  assert(f.FFp(1)->FFi(f.FFi(1))==1);
  assert(f.FFp(2)->FFi(f.FFi(2))==2);
}

/**
 * Check if the given face is oriented as the one adjacent to the specified edge.
 * @param f Face to check the orientation
 * @param z Index of the edge
 */
template <class FaceType>
bool CheckOrientation(FaceType &f, int z)
{
    if (IsBorder(f, z))
        return true;
    else
    {
        FaceType *g = f.FFp(z);
        int gi = f.FFi(z);
        if (f.V0(z) == g->V1(gi))
            return true;
        else
            return false;
    }
}


/**
 * This function change the orientation of the face by inverting the index of two vertex.
 * @param z Index of the edge
 */
template <class FaceType>
void SwapEdge(FaceType &f, const int z) { SwapEdge<FaceType,true>(f,z); }

template <class FaceType, bool UpdateTopology>
void SwapEdge(FaceType &f, const int z)
{
    // swap V0(z) with V1(z)
    std::swap(f.V0(z), f.V1(z));

    // Managemnt of faux edge information (edge z is not affected)
    bool Faux1 = f.IsF((z+1)%3);
    bool Faux2 = f.IsF((z+2)%3);
    if(Faux1) f.SetF((z+2)%3); else f.ClearF((z+2)%3);
    if(Faux2) f.SetF((z+1)%3); else f.ClearF((z+1)%3);

    if(f.HasFFAdjacency() && UpdateTopology)
    {
        // store information to preserve topology
        int z1 = (z+1)%3;
        int z2 = (z+2)%3;
        FaceType *g1p = f.FFp(z1);
        FaceType *g2p = f.FFp(z2);
        int g1i = f.FFi(z1);
        int g2i = f.FFi(z2);

        // g0 face topology is not affected by the swap

        if (g1p != &f)
        {
            g1p->FFi(g1i) = z2;
            f.FFi(z2) = g1i;
        }
        else
        {
            f.FFi(z2) = z2;
        }

        if (g2p != &f)
        {
            g2p->FFi(g2i) = z1;
            f.FFi(z1) = g2i;
        }
        else
        {
            f.FFi(z1) = z1;
        }

        // finalize swap
        f.FFp(z1) = g2p;
        f.FFp(z2) = g1p;
    }
}

/*! Perform a simple edge collapse
 * Basic link conditions
 *
*/
template <class FaceType>
bool FFLinkCondition(FaceType &f, const int z)
{
  typedef typename FaceType::VertexType VertexType;
  typedef typename vcg::face::Pos< FaceType > PosType;

  VertexType *v0=f.V0(z);
  VertexType *v1=f.V1(z);

  PosType p0(&f,v0);
  PosType p1(&f,v1);
  std::vector<VertexType *>v0Vec;
  std::vector<VertexType *>v1Vec;
  VVOrderedStarFF(p0,v0Vec);
  VVOrderedStarFF(p1,v1Vec);
  std::set<VertexType *> v0set;
  v0set.insert(v0Vec.begin(),v0Vec.end());
  assert(v0set.size() == v0Vec.size());
  int cnt =0;
  for(size_t i=0;i<v1Vec.size();++i)
    if(v0set.find(v1Vec[i]) != v0set.end())
      cnt++;

  if(face::IsBorder(f,z) && (cnt==1)) return true;
  if(!face::IsBorder(f,z) && (cnt==2)) return true;
  //assert(0);
  return false;
}

/*! Perform a simple edge collapse
 * The edge z is collapsed and the vertex V(z) is collapsed onto the vertex V1(Z)
 * It assumes that the mesh is Manifold.
 * If the mesh is not manifold it will crash (there will be faces with deleted vertexes around)
 *           f12
 *   surV ___________
 *       |\          |
 *       |  \    f1  |
 *   f01 |    \ z1   | f11
 *       | f0 z0\    |
 *       |        \  |
 *       |__________\|
 *          f02      delV
 */
template <class MeshType>
void FFEdgeCollapse(MeshType &m, typename MeshType::FaceType &f, const int z)
{
  typedef typename MeshType::FaceType FaceType;
  typedef typename MeshType::VertexType VertexType;
  typedef typename vcg::face::Pos< FaceType > PosType;
  FaceType *f0 = &f;
  int z0=z;
  FaceType *f1 = f.FFp(z);
  int z1=f.FFi(z);

  VertexType *delV=f.V0(z);
  VertexType *surV=f.V1(z);

  // Collect faces that have to be updated
  PosType delPos(f0,delV);
  std::vector<PosType> faceToBeChanged;
  VFOrderedStarFF(delPos,faceToBeChanged);

  // Topology Stitching
  FaceType *f01= 0,*f02= 0,*f11= 0,*f12= 0;
  int       i01=-1, i02=-1, i11=-1, i12=-1;
  // Note that the faux bit is preserved only if both of the edges to be stiched are faux.
  bool f0Faux = f0->IsF((z0+1)%3) && f0->IsF((z0+2)%3);
  bool f1Faux = f1->IsF((z1+1)%3) && f1->IsF((z1+2)%3);

  if(!face::IsBorder(*f0,(z0+1)%3)) { f01 = f0->FFp((z0+1)%3); i01=f0->FFi((z0+1)%3); FFDetachManifold(*f0,(z0+1)%3);}
  if(!face::IsBorder(*f0,(z0+2)%3)) { f02 = f0->FFp((z0+2)%3); i02=f0->FFi((z0+2)%3); FFDetachManifold(*f0,(z0+2)%3);}
  if(!face::IsBorder(*f1,(z1+1)%3)) { f11 = f1->FFp((z1+1)%3); i11=f1->FFi((z1+1)%3); FFDetachManifold(*f1,(z1+1)%3);}
  if(!face::IsBorder(*f1,(z1+2)%3)) { f12 = f1->FFp((z1+2)%3); i12=f1->FFi((z1+2)%3); FFDetachManifold(*f1,(z1+2)%3);}

  // Final Pass to update the vertex ptrs in all the involved faces
  for(size_t i=0;i<faceToBeChanged.size();++i) {
    assert(faceToBeChanged[i].V() == delV);
    faceToBeChanged[i].F()->V(faceToBeChanged[i].VInd()) =surV;
  }

  if(f01 && f02)
  {
    FFAttachManifold(f01,i01,f02,i02);
    if(f0Faux) {f01->SetF(i01); f02->SetF(i02);}
  }
  if(f11 && f12)  {
    FFAttachManifold(f11,i11,f12,i12);
    if(f1Faux) {f11->SetF(i11); f12->SetF(i12);}
  }
  tri::Allocator<MeshType>::DeleteFace(m,*f0);
  if(f1!=f0) tri::Allocator<MeshType>::DeleteFace(m,*f1);
  tri::Allocator<MeshType>::DeleteVertex(m,*delV);
}

/*!
* Perform a Geometric Check about the normals of a edge flip.
* return trues if after the flip the normals does not change more than the given threshold angle;
* it assumes that the flip is topologically correct.
*
*	\param f	the face
*	\param z	the edge index
*   \param angleRad the threshold angle
*
*  oldD1 ___________ newD1
*       |\          |
*       |  \        |
*       |    \      |
*       |  f  z\    |
*       |        \  |
*       |__________\|
* newD0               oldD0
*/

template <class FaceType>
bool CheckFlipEdgeNormal(FaceType &f, const int z, const float angleRad)
{
  typedef typename FaceType::VertexType VertexType;
  typedef typename VertexType::CoordType CoordType;
  typedef typename VertexType::ScalarType ScalarType;

  VertexType *OldDiag0 = f.V0(z);
  VertexType *OldDiag1 = f.V1(z);

  VertexType *NewDiag0 = f.V2(z);
  VertexType *NewDiag1 = f.FFp(z)->V2(f.FFi(z));

  assert((NewDiag1 != NewDiag0) && (NewDiag1 != OldDiag0) && (NewDiag1 != OldDiag1));

  CoordType oldN0 = NormalizedNormal( NewDiag0->cP(),OldDiag0->cP(),OldDiag1->cP());
  CoordType oldN1 = NormalizedNormal( NewDiag1->cP(),OldDiag1->cP(),OldDiag0->cP());
  CoordType newN0 = NormalizedNormal( OldDiag0->cP(),NewDiag1->cP(),NewDiag0->cP());
  CoordType newN1 = NormalizedNormal( OldDiag1->cP(),NewDiag0->cP(),NewDiag1->cP());
  if(AngleN(oldN0,newN0) > angleRad) return false;
  if(AngleN(oldN0,newN1) > angleRad) return false;
  if(AngleN(oldN1,newN0) > angleRad) return false;
  if(AngleN(oldN1,newN1) > angleRad) return false;

  return true;
}

/*!
* Perform a Topological check to see if the z-th edge of the face f can be flipped.
* No Geometric test are done. (see CheckFlipEdgeNormal)
*	\param f	pointer to the face
*	\param z	the edge index
*/
template <class FaceType>
bool CheckFlipEdge(FaceType &f, int z)
{
  typedef typename FaceType::VertexType VertexType;
  typedef typename vcg::face::Pos< FaceType > PosType;

  if (z<0 || z>2)  return false;

    // boundary edges cannot be flipped
  if (face::IsBorder(f, z)) return false;

    FaceType *g = f.FFp(z);
    int		 w = f.FFi(z);

    // check if the vertices of the edge are the same
  // e.g. the mesh has to be well oriented
    if (g->V(w)!=f.V1(z) || g->V1(w)!=f.V(z) )
        return false;

    // check if the flipped edge is already present in the mesh
  // f_v2 and g_v2 are the vertices of the new edge
  VertexType *f_v2 = f.V2(z);
    VertexType *g_v2 = g->V2(w);

  // just a sanity check. If this happens the mesh is not manifold.
  if (f_v2 == g_v2) return false;

  // Now walk around f_v2, one of the two vertexes of the new edge
  // and check that it does not already exists.

  PosType pos(&f, (z+2)%3, f_v2);
  PosType startPos=pos;
    do
    {
        pos.NextE();
    if (g_v2 == pos.VFlip())
            return false;
    }
  while (pos != startPos);

    return true;
}

/*!
* Flip the z-th edge of the face f.
* Check for topological correctness first using <CODE>CheckFlipFace()</CODE>.
*	\param f	pointer to the face
*	\param z	the edge index
*
* Note: For <em>edge flip</em> we intend the swap of the diagonal of the rectangle
*       formed by the face \a f and the face adjacent to the specified edge.
*/
template <class FaceType>
void FlipEdge(FaceType &f, const int z)
{
    assert(z>=0);
    assert(z<3);
    assert( !IsBorder(f,z) );
    assert( face::IsManifold<FaceType>(f, z));

    FaceType *g = f.FFp(z);
    int		 w = f.FFi(z);

    assert( g->V(w)	== f.V1(z) );
    assert( g->V1(w)== f.V(z) );
    assert( g->V2(w)!= f.V(z) );
    assert( g->V2(w)!= f.V1(z) );
    assert( g->V2(w)!= f.V2(z) );

    f.V1(z) = g->V2(w);
    g->V1(w) = f.V2(z);

    f.FFp(z)				= g->FFp((w+1)%3);
    f.FFi(z)				= g->FFi((w+1)%3);
    g->FFp(w)				= f.FFp((z+1)%3);
    g->FFi(w)				= f.FFi((z+1)%3);
    f.FFp((z+1)%3)				= g;
    f.FFi((z+1)%3)	= (w+1)%3;
    g->FFp((w+1)%3)			= &f;
    g->FFi((w+1)%3) = (z+1)%3;

    if(f.FFp(z)==g)
    {
        f.FFp(z) = &f;
        f.FFi(z) = z;
    }
    else
    {
        f.FFp(z)->FFp( f.FFi(z) ) = &f;
        f.FFp(z)->FFi( f.FFi(z) ) = z;
    }
    if(g->FFp(w)==&f)
    {
        g->FFp(w)=g;
        g->FFi(w)=w;
    }
    else
    {
        g->FFp(w)->FFp( g->FFi(w) ) = g;
        g->FFp(w)->FFi( g->FFi(w) ) = w;
    }
}

template <class FaceType>
void VFDetach(FaceType & f)
{
  VFDetach(f,0);
  VFDetach(f,1);
  VFDetach(f,2);
}

// Stacca la faccia corrente dalla catena di facce incidenti sul vertice z,
// NOTA funziona SOLO per la topologia VF!!!
// usata nelle classi di collapse
template <class FaceType>
void VFDetach(FaceType & f, int z)
{
    if(f.V(z)->VFp()==&f )  //if it is the first face detach from the begin
    {
        int fz = f.V(z)->VFi();
        f.V(z)->VFp() = f.VFp(fz);
        f.V(z)->VFi() = f.VFi(fz);
    }
    else  // scan the list of faces in order to finde the current face f to be detached
    {
    VFIterator<FaceType> x(f.V(z)->VFp(),f.V(z)->VFi());
    VFIterator<FaceType> y;

        for(;;)
        {
            y = x;
            ++x;
            assert(x.f!=0);
            if(x.f==&f) // found!
            {
                y.f->VFp(y.z) = f.VFp(z);
                y.f->VFi(y.z) = f.VFi(z);
                break;
            }
        }
    }
}

/// Append a face in VF list of vertex f->V(z)
template <class FaceType>
void VFAppend(FaceType* & f, int z)
{
    typename FaceType::VertexType *v = f->V(z);
    if (v->VFp()!=0)
    {
        FaceType *f0=v->VFp();
        int z0=v->VFi();
        //append
        f->VFp(z)=f0;
        f->VFi(z)=z0;
    }
    v->VFp()=f;
    v->VFi()=z;
}

/*!
* \brief Compute the set of vertices adjacent to a given vertex using VF adjacency
*
*	\param vp	pointer to the vertex whose star has to be computed.
*	\param starVec a std::vector of Vertex pointer that is filled with the adjacent vertices.
*
*/

template <class FaceType>
void VVStarVF( typename FaceType::VertexType* vp, std::vector<typename FaceType::VertexType *> &starVec)
{
    typedef typename FaceType::VertexType* VertexPointer;
    starVec.clear();
    face::VFIterator<FaceType> vfi(vp);
    while(!vfi.End())
            {
                starVec.push_back(vfi.F()->V1(vfi.I()));
                starVec.push_back(vfi.F()->V2(vfi.I()));
                ++vfi;
            }

    std::sort(starVec.begin(),starVec.end());
    typename std::vector<VertexPointer>::iterator new_end = std::unique(starVec.begin(),starVec.end());
    starVec.resize(new_end-starVec.begin());
}

/*!
 * \brief Compute the set of vertices adjacent to a given vertex using VF adjacency.
 *
 * The set is faces is extended of a given number of step
 *	\param vp	pointer to the vertex whose star has to be computed.
 *  \param num_step the number of step to extend the star
 *	\param vertVec a std::vector of Ve pointer that is filled with the adjacent faces.
 */
template <class FaceType>
void VVExtendedStarVF(typename FaceType::VertexType* vp,
                      const int num_step,
                      std::vector<typename FaceType::VertexType *> &vertVec)
    {
        typedef typename FaceType::VertexType VertexType;
        ///initialize front
        vertVec.clear();
        vcg::face::VVStarVF<FaceType>(vp,vertVec);
        ///then dilate front
        ///for each step
        for (int step=0;step<num_step-1;step++)
        {
            std::vector<VertexType *> toAdd;
            for (unsigned int i=0;i<vertVec.size();i++)
            {
                std::vector<VertexType *> Vtemp;
                vcg::face::VVStarVF<FaceType>(vp,Vtemp);
                toAdd.insert(toAdd.end(),Vtemp.begin(),Vtemp.end());
            }
            vertVec.insert(vertVec.end(),toAdd.begin(),toAdd.end());
            std::sort(vertVec.begin(),vertVec.end());
            typename std::vector<typename FaceType::VertexType *>::iterator new_end=std::unique(vertVec.begin(),vertVec.end());
            int dist=distance(vertVec.begin(),new_end);
            vertVec.resize(dist);
        }
    }

/*!
* \brief Compute the set of faces adjacent to a given vertex using VF adjacency.
*
*	\param vp	pointer to the vertex whose star has to be computed.
*	\param faceVec a std::vector of Face pointer that is filled with the adjacent faces.
*   \param indexes a std::vector of integer of the vertex as it is seen from the faces
*/
template <class FaceType>
void VFStarVF( typename FaceType::VertexType* vp,
               std::vector<FaceType *> &faceVec,
               std::vector<int> &indexes)
{
    typedef typename FaceType::VertexType* VertexPointer;
    faceVec.clear();
    indexes.clear();
    face::VFIterator<FaceType> vfi(vp);
    while(!vfi.End())
    {
        faceVec.push_back(vfi.F());
        indexes.push_back(vfi.I());
        ++vfi;
    }
}


/*!
* \brief Compute the set of faces incident onto a given edge using FF adjacency.
*
*	\param fp	pointer to the face whose star has to be computed
*	\param ei	the index of the edge
*	\param faceVec a std::vector of Face pointer that is filled with the faces incident on that edge.
*   \param indexes a std::vector of integer of the edge position as it is seen from the faces
*/
template <class FaceType>
void EFStarFF( FaceType* fp, int ei,
               std::vector<FaceType *> &faceVec,
               std::vector<int> &indVed)
{
  assert(fp->FFp(ei)!=0);
  faceVec.clear();
  indVed.clear();
  FaceType* fpit=fp;
  int eit=ei;
  do
  {
    faceVec.push_back(fpit);
    indVed.push_back(eit);
    FaceType *new_fpit = fpit->FFp(eit);
    int       new_eit  = fpit->FFi(eit);
    fpit=new_fpit;
    eit=new_eit;
  } while(fpit != fp);
}


    /* Compute the set of faces adjacent to a given face using FF adjacency.
    * The set is faces is extended of a given number of step
    *	\param fp	pointer to the face whose star has to be computed.
    *  \param num_step the number of step to extend the star
    *	\param faceVec a std::vector of Face pointer that is filled with the adjacent faces.
    */
    template <class FaceType>
    static void FFExtendedStarFF(FaceType *fp,
                                 const int num_step,
                                 std::vector<FaceType*> &faceVec)
    {
        ///initialize front
        faceVec.push_back(fp);
        ///then dilate front
        ///for each step
        for (int step=0;step<num_step;step++)
        {
            std::vector<FaceType*> toAdd;
            for (unsigned int i=0;i<faceVec.size();i++)
            {
                FaceType *f=faceVec[i];
                for (int k=0;k<3;k++)
                {
                    FaceType *f1=f->FFp(k);
                    if (f1==f)continue;
                    toAdd.push_back(f1);
                }
            }
            faceVec.insert(faceVec.end(),toAdd.begin(),toAdd.end());
            std::sort(faceVec.begin(),faceVec.end());
            typename std::vector<FaceType*>::iterator new_end=std::unique(faceVec.begin(),faceVec.end());
            int dist=distance(faceVec.begin(),new_end);
            faceVec.resize(dist);
        }
    }

/*!
 * \brief Compute the set of faces adjacent to a given vertex using VF adjacency.
 *
 * The set is faces is extended of a given number of step
 *	\param vp	pointer to the vertex whose star has to be computed.
 *  \param num_step the number of step to extend the star
 *	\param faceVec a std::vector of Face pointer that is filled with the adjacent faces.
 */
template <class FaceType>
void VFExtendedStarVF(typename FaceType::VertexType* vp,
                             const int num_step,
                             std::vector<FaceType*> &faceVec)
    {
        ///initialize front
        faceVec.clear();
        std::vector<int> indexes;
        vcg::face::VFStarVF<FaceType>(vp,faceVec,indexes);
        ///then dilate front
        ///for each step
        for (int step=0;step<num_step;step++)
        {
            std::vector<FaceType*> toAdd;
            for (unsigned int i=0;i<faceVec.size();i++)
            {
                FaceType *f=faceVec[i];
                for (int k=0;k<3;k++)
                {
                    FaceType *f1=f->FFp(k);
                    if (f1==f)continue;
                    toAdd.push_back(f1);
                }
            }
            faceVec.insert(faceVec.end(),toAdd.begin(),toAdd.end());
            std::sort(faceVec.begin(),faceVec.end());
            typename std::vector<FaceType*>::iterator new_end=std::unique(faceVec.begin(),faceVec.end());
            int dist=distance(faceVec.begin(),new_end);
            faceVec.resize(dist);
        }
    }

/*!
 * \brief Compute the ordered set of vertices adjacent to a given vertex using FF adiacency
 *
 * \param startPos a Pos<FaceType> indicating the vertex whose star has to be computed.
 * \param vertexVec a std::vector of VertexPtr filled vertices around the given vertex.
 *
*/
template <class FaceType>
void VVOrderedStarFF(Pos<FaceType> &startPos,
                     std::vector<typename FaceType::VertexType *> &vertexVec)
{
  vertexVec.clear();
  std::vector<Pos<FaceType> > posVec;
  VFOrderedStarFF(startPos,posVec);
  for(size_t i=0;i<posVec.size();++i)
    vertexVec.push_back(posVec[i].VFlip());
}

/*!
 * \brief Compute the ordered set of faces adjacent to a given vertex using FF adiacency
 *
 * \param startPos a Pos<FaceType> indicating the vertex whose star has to be computed.
 * \param posVec a std::vector of Pos filled with Pos arranged around the passed vertex.
 *
*/
template <class FaceType>
void VFOrderedStarFF(Pos<FaceType> &startPos,
                     std::vector<Pos<FaceType> > &posVec)
{
  posVec.clear();
  bool foundBorder=false;
  size_t firstBorderInd;
  Pos<FaceType> curPos=startPos;
  do
  {
    assert(curPos.IsManifold());
    if(curPos.IsBorder() && !foundBorder) {
      foundBorder=true;
      firstBorderInd = posVec.size();
    }
    posVec.push_back(curPos);
    curPos.FlipF();
    curPos.FlipE();
  } while(curPos!=startPos);
  // if we found a border we visited each face exactly twice,
  // and we have to extract the border-to-border pos sequence
  if(foundBorder)
  {
    size_t halfSize=posVec.size()/2;
    assert((posVec.size()%2)==0);
    posVec.erase(posVec.begin()+firstBorderInd+1+halfSize, posVec.end());
    posVec.erase(posVec.begin(),posVec.begin()+firstBorderInd+1);
    assert(posVec.size()==halfSize);
  }
}

/*!
 * \brief Compute the ordered set of faces adjacent to a given vertex using FF adiacency
 *
 * \param startPos a Pos<FaceType> indicating the vertex whose star has to be computed.
 * \param faceVec a std::vector of Face pointer that is filled with the adjacent faces.
 * \param edgeVec a std::vector of indexes filled with the indexes of the corresponding edges shared between the faces.
 *
*/

template <class FaceType>
void VFOrderedStarFF(Pos<FaceType> &startPos,
                        std::vector<FaceType*> &faceVec,
                        std::vector<int> &edgeVec)
{
  std::vector<Pos<FaceType> > posVec;
  VFOrderedStarFF(startPos,posVec);
  faceVec.clear();
  edgeVec.clear();
  for(size_t i=0;i<posVec.size();++i)
  {
    faceVec.push_back(posVec[i].F());
    edgeVec.push_back(posVec[i].E());
  }
}

/*!
* Check if two faces share and edge through the FF topology.
*	\param f0,f1 the two face to be checked
* \param i0,i1 the index of the shared edge;
*/

template <class FaceType>
bool ShareEdgeFF(FaceType *f0,FaceType *f1, int *i0=0, int *i1=0)
{
  assert((!f0->IsD())&&(!f1->IsD()));
  for (int i=0;i<3;i++)
      if (f0->FFp(i)==f1)
      {
        if((i0!=0) && (i1!=0)) {
          *i0=i;
          *i1=f0->FFi(i);
        }
        return true;
      }
  return false;
}

/*!
* Count the number of vertices shared between two faces.
*	\param f0,f1 the two face to be checked
* ;
*/
template <class FaceType>
int CountSharedVertex(FaceType *f0,FaceType *f1)
{
  int sharedCnt=0;
  for (int i=0;i<3;i++)
      for (int j=0;j<3;j++)
          if (f0->V(i)==f1->V(j)) {
                  sharedCnt++;
              }
  return sharedCnt;
}

/*!
* find the first shared vertex between two faces.
*	\param f0,f1 the two face to be checked
* \param i,j the indexes of the shared vertex in the two faces. Meaningful only if there is one single shared vertex
* ;
*/
template <class FaceType>
bool FindSharedVertex(FaceType *f0,FaceType *f1, int &i, int &j)
{
  for (i=0;i<3;i++)
      for (j=0;j<3;j++)
          if (f0->V(i)==f1->V(j)) return true;

  i=-1;j=-1;
  return false;
}

/*!
* find the first shared edge between two faces.
*	\param f0,f1 the two face to be checked
* \param i,j the indexes of the shared edge in the two faces. Meaningful only if there is a shared edge
*
*/
template <class FaceType>
bool FindSharedEdge(FaceType *f0,FaceType *f1, int &i, int &j)
{
  for (i=0;i<3;i++)
      for (j=0;j<3;j++)
        if( ( f0->V0(i)==f1->V0(j) || f0->V0(i)==f1->V1(j) ) &&
            ( f0->V1(i)==f1->V0(j) || f0->V1(i)==f1->V1(j) ) )
            return true;
  i=-1;j=-1;
  return false;
}

/*!
* find the faces that shares the two vertices
* \param v0,v1 the two vertices
* \param f0,f1 the two faces , counterclokwise order
*
*/
template <class FaceType>
bool FindSharedFaces(typename FaceType::VertexType *v0,
                     typename FaceType::VertexType *v1,
                     FaceType *&f0,
                     FaceType *&f1,
                     int &e0,
                     int &e1)
{
    std::vector<FaceType*> faces0;
    std::vector<FaceType*> faces1;
    std::vector<int> index0;
    std::vector<int> index1;
    VFStarVF<FaceType>(v0,faces0,index0);
    VFStarVF<FaceType>(v1,faces1,index1);
    ///then find the intersection
    std::sort(faces0.begin(),faces0.end());
    std::sort(faces1.begin(),faces1.end());
    std::vector<FaceType*> Intersection;
    std::set_intersection(faces0.begin(),faces0.end(),faces1.begin(),faces1.end(),std::back_inserter(Intersection));
    if (Intersection.size()<2)return false; ///no pair of faces share the 2 vertices
    assert(Intersection.size()==2);//otherwhise non manifoldess
    f0=Intersection[0];
    f1=Intersection[1];
    FindSharedEdge(f0,f1,e0,e1);
    ///and finally check if the order is right
    if (f0->V(e0)!=v0)
    {
        std::swap(f0,f1);
        std::swap(e0,e1);
    }
    return true;
}

/*@}*/
}	 // end namespace
}	 // end namespace

#endif

