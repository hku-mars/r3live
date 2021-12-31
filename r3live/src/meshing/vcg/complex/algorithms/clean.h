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

#ifndef __VCGLIB_CLEAN
#define __VCGLIB_CLEAN

// VCG headers
#include <vcg/complex/complex.h>
#include <vcg/simplex/face/pos.h>
#include <vcg/simplex/face/topology.h>
#include <vcg/complex/algorithms/closest.h>
#include <vcg/space/index/grid_static_ptr.h>
#include <vcg/space/index/spatial_hashing.h>
#include <vcg/complex/algorithms/update/selection.h>
#include <vcg/complex/algorithms/update/flag.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/space/triangle3.h>

namespace vcg {
namespace tri{

template <class ConnectedMeshType>
class ConnectedComponentIterator
{
public:
  typedef ConnectedMeshType MeshType;
  typedef typename MeshType::VertexType     VertexType;
  typedef typename MeshType::VertexPointer  VertexPointer;
  typedef typename MeshType::VertexIterator VertexIterator;
  typedef typename MeshType::ScalarType     ScalarType;
  typedef typename MeshType::FaceType       FaceType;
  typedef typename MeshType::FacePointer    FacePointer;
  typedef typename MeshType::FaceIterator   FaceIterator;
  typedef typename MeshType::ConstFaceIterator   ConstFaceIterator;
  typedef typename MeshType::FaceContainer  FaceContainer;

public:
  void operator ++()
  {
    FacePointer fpt=sf.top();
    sf.pop();
    for(int j=0;j<3;++j)
      if( !face::IsBorder(*fpt,j) )
      {
        FacePointer l=fpt->FFp(j);
        if( !tri::IsMarked(*mp,l) )
        {
          tri::Mark(*mp,l);
          sf.push(l);
        }
      }
  }

  void start(MeshType &m, FacePointer p)
  {
    mp=&m;
    while(!sf.empty()) sf.pop();
    UnMarkAll(m);
    assert(p);
    assert(!p->IsD());
    tri::Mark(m,p);
    sf.push(p);
  }

  bool completed() {
    return sf.empty();
  }

  FacePointer operator *()
  {
    return sf.top();
  }
private:
  std::stack<FacePointer> sf;
  MeshType *mp;
};


///
/** \addtogroup trimesh */
/*@{*/
/// Class of static functions to clean//restore meshs.
template <class CleanMeshType>
class Clean
{

public:
  typedef CleanMeshType MeshType;
  typedef typename MeshType::VertexType     VertexType;
  typedef typename MeshType::VertexPointer  VertexPointer;
  typedef typename MeshType::VertexIterator VertexIterator;
  typedef typename MeshType::ConstVertexIterator ConstVertexIterator;
  typedef typename MeshType::EdgeIterator   EdgeIterator;
  typedef typename MeshType::EdgePointer  EdgePointer;
  typedef typename MeshType::CoordType   CoordType;
  typedef typename MeshType::ScalarType ScalarType;
  typedef typename MeshType::FaceType       FaceType;
  typedef typename MeshType::FacePointer    FacePointer;
  typedef typename MeshType::FaceIterator   FaceIterator;
  typedef typename MeshType::ConstFaceIterator   ConstFaceIterator;
  typedef typename MeshType::FaceContainer  FaceContainer;
  typedef typename vcg::Box3<ScalarType>  Box3Type;

  typedef GridStaticPtr<FaceType, ScalarType > TriMeshGrid;
  typedef Point3<ScalarType> Point3x;

  /* classe di confronto per l'algoritmo di eliminazione vertici duplicati*/
  class RemoveDuplicateVert_Compare{
  public:
    inline bool operator()(VertexPointer const &a, VertexPointer const &b)
    {
      return (*a).cP() < (*b).cP();
    }
  };


            /** This function removes all duplicate vertices of the mesh by looking only at their spatial positions.
            Note that it does not update any topology relation that could be affected by this like the VT or TT relation.
            the reason this function is usually performed BEFORE building any topology information.
            */
            static int RemoveDuplicateVertex( MeshType & m, bool RemoveDegenerateFlag=true)    // V1.0
            {
                if(m.vert.size()==0 || m.vn==0) return 0;

                std::map<VertexPointer, VertexPointer> mp;
                size_t i,j;
                VertexIterator vi;
                int deleted=0;
                int k=0;
                size_t num_vert = m.vert.size();
                std::vector<VertexPointer> perm(num_vert);
                for(vi=m.vert.begin(); vi!=m.vert.end(); ++vi, ++k)
                    perm[k] = &(*vi);

                RemoveDuplicateVert_Compare c_obj;

                std::sort(perm.begin(),perm.end(),c_obj);

                j = 0;
                i = j;
                mp[perm[i]] = perm[j];
                ++i;
                for(;i!=num_vert;)
                {
                    if( (! (*perm[i]).IsD()) &&
                        (! (*perm[j]).IsD()) &&
                        (*perm[i]).P() == (*perm[j]).cP() )
                    {
                        VertexPointer t = perm[i];
                        mp[perm[i]] = perm[j];
                        ++i;
                        Allocator<MeshType>::DeleteVertex(m,*t);
                        deleted++;
                    }
                    else
                    {
                        j = i;
                        ++i;
                    }
                }

        for(FaceIterator fi = m.face.begin(); fi!=m.face.end(); ++fi)
                    if( !(*fi).IsD() )
                        for(k = 0; k < 3; ++k)
                            if( mp.find( (typename MeshType::VertexPointer)(*fi).V(k) ) != mp.end() )
                            {
                                (*fi).V(k) = &*mp[ (*fi).V(k) ];
                            }


        for(EdgeIterator ei = m.edge.begin(); ei!=m.edge.end(); ++ei)
          if( !(*ei).IsD() )
            for(k = 0; k < 2; ++k)
              if( mp.find( (typename MeshType::VertexPointer)(*ei).V(k) ) != mp.end() )
              {
                (*ei).V(k) = &*mp[ (*ei).V(k) ];
              }
        if(RemoveDegenerateFlag) RemoveDegenerateFace(m);
        if(RemoveDegenerateFlag && m.en>0) {
          RemoveDegenerateEdge(m);
          RemoveDuplicateEdge(m);
        }
                return deleted;
      }

            class SortedPair
                  {
                  public:
                   SortedPair() {}
                       SortedPair(unsigned int v0, unsigned int v1, EdgePointer _fp)
                      {
                          v[0]=v0;v[1]=v1;
                          fp=_fp;
                          if(v[0]>v[1]) std::swap(v[0],v[1]);
                      }
                      bool operator < (const SortedPair &p) const
                      {
                          return (v[1]!=p.v[1])?(v[1]<p.v[1]):
                                     (v[0]<p.v[0]);				}

                      bool operator == (const SortedPair &s) const
                      {
                       if( (v[0]==s.v[0]) && (v[1]==s.v[1]) ) return true;
                       return false;
                      }

                      unsigned int v[2];
                      EdgePointer fp;
                  };
      class SortedTriple
            {
            public:
             SortedTriple() {}
                 SortedTriple(unsigned int v0, unsigned int v1, unsigned int v2,FacePointer _fp)
                {
                    v[0]=v0;v[1]=v1;v[2]=v2;
                    fp=_fp;
                    std::sort(v,v+3);
                }
                bool operator < (const SortedTriple &p) const
                {
                    return (v[2]!=p.v[2])?(v[2]<p.v[2]):
                            (v[1]!=p.v[1])?(v[1]<p.v[1]):
                               (v[0]<p.v[0]);				}

                bool operator == (const SortedTriple &s) const
                {
                 if( (v[0]==s.v[0]) && (v[1]==s.v[1]) && (v[2]==s.v[2]) ) return true;
                 return false;
                }

                unsigned int v[3];
                FacePointer fp;
            };


            /** This function removes all duplicate faces of the mesh by looking only at their vertex reference.
            So it should be called after unification of vertices.
            Note that it does not update any topology relation that could be affected by this like the VT or TT relation.
            the reason this function is usually performed BEFORE building any topology information.
            */
            static int RemoveDuplicateFace( MeshType & m)    // V1.0
            {
                FaceIterator fi;
                std::vector<SortedTriple> fvec;
                for(fi=m.face.begin();fi!=m.face.end();++fi)
                    if(!(*fi).IsD())
                        {
                         fvec.push_back(SortedTriple(	tri::Index(m,(*fi).V(0)),
                                                                                    tri::Index(m,(*fi).V(1)),
                                                                                    tri::Index(m,(*fi).V(2)),
                                                                                                                                                                        &*fi));
                        }
                assert (size_t(m.fn) == fvec.size());
                //for(int i=0;i<fvec.size();++i) qDebug("fvec[%i] = (%i %i %i)(%i)",i,fvec[i].v[0],fvec[i].v[1],fvec[i].v[2],tri::Index(m,fvec[i].fp));
                std::sort(fvec.begin(),fvec.end());
                int total=0;
                for(int i=0;i<int(fvec.size())-1;++i)
                {
                    if(fvec[i]==fvec[i+1])
                    {
                        total++;
                        tri::Allocator<MeshType>::DeleteFace(m, *(fvec[i].fp) );
                        //qDebug("deleting face %i (pos in fvec %i)",tri::Index(m,fvec[i].fp) ,i);
                    }
                }
                return total;
            }

            /** This function removes all duplicate faces of the mesh by looking only at their vertex reference.
            So it should be called after unification of vertices.
            Note that it does not update any topology relation that could be affected by this like the VT or TT relation.
            the reason this function is usually performed BEFORE building any topology information.
            */
            static int RemoveDuplicateEdge( MeshType & m)    // V1.0
            {
              //assert(m.fn == 0 && m.en >0); // just to be sure we are using an edge mesh...
                if (m.en==0)return 0;
              std::vector<SortedPair> eVec;
              for(EdgeIterator ei=m.edge.begin();ei!=m.edge.end();++ei)
                if(!(*ei).IsD())
                {
                  eVec.push_back(SortedPair(	tri::Index(m,(*ei).V(0)), tri::Index(m,(*ei).V(1)), &*ei));
                }
              assert (size_t(m.en) == eVec.size());
              //for(int i=0;i<fvec.size();++i) qDebug("fvec[%i] = (%i %i %i)(%i)",i,fvec[i].v[0],fvec[i].v[1],fvec[i].v[2],tri::Index(m,fvec[i].fp));
              std::sort(eVec.begin(),eVec.end());
              int total=0;
              for(int i=0;i<int(eVec.size())-1;++i)
              {
                if(eVec[i]==eVec[i+1])
                {
                  total++;
                  tri::Allocator<MeshType>::DeleteEdge(m, *(eVec[i].fp) );
                  //qDebug("deleting face %i (pos in fvec %i)",tri::Index(m,fvec[i].fp) ,i);
                }
              }
              return total;
            }
            static int CountUnreferencedVertex( MeshType& m)
            {
              return RemoveUnreferencedVertex(m,false);
            }


            /** This function removes that are not referenced by any face. The function updates the vn counter.
            @param m The mesh
            @return The number of removed vertices
            */
            static int RemoveUnreferencedVertex( MeshType& m, bool DeleteVertexFlag=true)   // V1.0
            {
                FaceIterator fi;
                EdgeIterator ei;
                VertexIterator vi;
                int referredBit = VertexType::NewBitFlag();

                int j;
                int deleted = 0;

                for(vi=m.vert.begin();vi!=m.vert.end();++vi)
                    (*vi).ClearUserBit(referredBit);

                for(fi=m.face.begin();fi!=m.face.end();++fi)
                    if( !(*fi).IsD() )
                        for(j=0;j<3;++j)
                            (*fi).V(j)->SetUserBit(referredBit);

                for(ei=m.edge.begin();ei!=m.edge.end();++ei)
                    if( !(*ei).IsD() ){
                      (*ei).V(0)->SetUserBit(referredBit);
                      (*ei).V(1)->SetUserBit(referredBit);
                    }

                for(vi=m.vert.begin();vi!=m.vert.end();++vi)
                    if( (!(*vi).IsD()) && (!(*vi).IsUserBit(referredBit)))
                    {
                        if(DeleteVertexFlag) Allocator<MeshType>::DeleteVertex(m,*vi);
                        ++deleted;
                    }
                VertexType::DeleteBitFlag(referredBit);
                return deleted;
            }

      /**
      Degenerate vertices are vertices that have coords with invalid floating point values,
      All the faces incident on deleted vertices are also deleted
            */
      static int RemoveDegenerateVertex(MeshType& m)
      {
                VertexIterator vi;
                int count_vd = 0;

                for(vi=m.vert.begin(); vi!=m.vert.end();++vi)
                    if(math::IsNAN( (*vi).P()[0]) ||
             math::IsNAN( (*vi).P()[1]) ||
                         math::IsNAN( (*vi).P()[2]) )
                    {
                        count_vd++;
                        Allocator<MeshType>::DeleteVertex(m,*vi);
                    }

                FaceIterator fi;
                int count_fd = 0;

                for(fi=m.face.begin(); fi!=m.face.end();++fi)
                if(!(*fi).IsD())
                    if( (*fi).V(0)->IsD() ||
                            (*fi).V(1)->IsD() ||
                            (*fi).V(2)->IsD() )
                                    {
                                        count_fd++;
                                        Allocator<MeshType>::DeleteFace(m,*fi);
                                    }
                return count_vd;
            }

      /**
      Degenerate faces are faces that are Topologically degenerate,
      i.e. have two or more vertex reference that link the same vertex
      (and not only two vertexes with the same coordinates).
      All Degenerate faces are zero area faces BUT not all zero area faces are degenerate.
      We do not take care of topology because when we have degenerate faces the
      topology calculation functions crash.
      */
      static int RemoveDegenerateFace(MeshType& m)
      {
                int count_fd = 0;

        for(FaceIterator fi=m.face.begin(); fi!=m.face.end();++fi)
                    if(!(*fi).IsD())
                    {
                            if((*fi).V(0) == (*fi).V(1) ||
                                 (*fi).V(0) == (*fi).V(2) ||
                                 (*fi).V(1) == (*fi).V(2) )
                            {
                                count_fd++;
                                Allocator<MeshType>::DeleteFace(m,*fi);
                            }
                    }
                return count_fd;
            }

      static int RemoveDegenerateEdge(MeshType& m)
            {
              int count_ed = 0;

              for(EdgeIterator ei=m.edge.begin(); ei!=m.edge.end();++ei)
                if(!(*ei).IsD())
                {
                    if((*ei).V(0) == (*ei).V(1) )
                    {
                      count_ed++;
                      Allocator<MeshType>::DeleteEdge(m,*ei);
                    }
                }
              return count_ed;
            }

            static int RemoveNonManifoldVertex(MeshType& m)
            {
        /*int count_vd = */
        CountNonManifoldVertexFF(m,true);
        /*int count_fd = */
        tri::UpdateSelection<MeshType>::FaceFromVertexLoose(m);
                int count_removed = 0;
                FaceIterator fi;
                for(fi=m.face.begin(); fi!=m.face.end();++fi)
                    if(!(*fi).IsD() && (*fi).IsS())
                        Allocator<MeshType>::DeleteFace(m,*fi);
                VertexIterator vi;
                for(vi=m.vert.begin(); vi!=m.vert.end();++vi)
                    if(!(*vi).IsD() && (*vi).IsS()) {
                        ++count_removed;
                        Allocator<MeshType>::DeleteVertex(m,*vi);
                    }
                    return count_removed;
            }


      /// Removal of faces that were incident on a non manifold edge.

      // Given a mesh with FF adjacency
      // it search for non manifold vertices and duplicate them.
      // Duplicated vertices are moved apart according to the move threshold param.
      // that is a percentage of the average vector from the non manifold vertex to the barycenter of the incident faces.

      static int SplitNonManifoldVertex(MeshType& m, ScalarType moveThreshold)
      {
        RequireFFAdjacency(m);
        typedef std::pair<FacePointer,int> FaceInt; // a face and the index of the vertex that we have to change
        //
        std::vector<std::pair<VertexPointer, std::vector<FaceInt> > >ToSplitVec;

        SelectionStack<MeshType> ss(m);
        ss.push();
        CountNonManifoldVertexFF(m,true);
        UpdateFlags<MeshType>::VertexClearV(m);
        for (FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)	if (!fi->IsD())
        {
          for(int i=0;i<3;i++)
            if((*fi).V(i)->IsS() && !(*fi).V(i)->IsV())
            {
              (*fi).V(i)->SetV();
              face::Pos<FaceType> startPos(&*fi,i);
              face::Pos<FaceType> curPos = startPos;
              std::set<FaceInt> faceSet;
              do
              {
                faceSet.insert(std::make_pair(curPos.F(),curPos.VInd()));
                curPos.NextE();
              } while (curPos != startPos);

              ToSplitVec.push_back(make_pair((*fi).V(i),std::vector<FaceInt>()));

              typename std::set<FaceInt>::const_iterator iii;

              for(iii=faceSet.begin();iii!=faceSet.end();++iii)
                ToSplitVec.back().second.push_back(*iii);
            }
        }
        ss.pop();
        // Second step actually add new vertices and split them.
        typename tri::Allocator<MeshType>::template PointerUpdater<VertexPointer> pu;
        VertexIterator firstVp = tri::Allocator<MeshType>::AddVertices(m,ToSplitVec.size(),pu);
        for(size_t i =0;i<ToSplitVec.size();++i)
        {
//          qDebug("Splitting Vertex %i",ToSplitVec[i].first-&*m.vert.begin());
          VertexPointer np=ToSplitVec[i].first;
          pu.Update(np);
          firstVp->ImportData(*np);
          // loop on the face to be changed, and also compute the movement vector;
          CoordType delta(0,0,0);
          for(size_t j=0;j<ToSplitVec[i].second.size();++j)
          {
            FaceInt ff=ToSplitVec[i].second[j];
            ff.first->V(ff.second)=&*firstVp;
            delta+=Barycenter(*(ff.first))-np->cP();
          }
          delta /= ToSplitVec[i].second.size();
          firstVp->P() = firstVp->P() + delta * moveThreshold;
          firstVp++;
        }

        return ToSplitVec.size();
      }


      // Auxiliary function for sorting the non manifold faces according to their area. Used in  RemoveNonManifoldFace
      struct CompareAreaFP {
          bool operator ()(FacePointer const& f1, FacePointer const& f2) const {
             return DoubleArea(*f1) < DoubleArea(*f2);
          }
      };

      /// Removal of faces that were incident on a non manifold edge.
      static int RemoveNonManifoldFace(MeshType& m)
      {
                FaceIterator fi;
                int count_fd = 0;
                std::vector<FacePointer> ToDelVec;

                for(fi=m.face.begin(); fi!=m.face.end();++fi)
                    if (!fi->IsD())
                    {
                        if ((!IsManifold(*fi,0))||
                                (!IsManifold(*fi,1))||
                                (!IsManifold(*fi,2)))
                                      ToDelVec.push_back(&*fi);
                    }

          std::sort(ToDelVec.begin(),ToDelVec.end(),CompareAreaFP());

          for(size_t i=0;i<ToDelVec.size();++i)
          {
            if(!ToDelVec[i]->IsD())
            {
            FaceType &ff= *ToDelVec[i];
              if ((!IsManifold(ff,0))||
                                (!IsManifold(ff,1))||
                                  (!IsManifold(ff,2)))
              {
                for(int j=0;j<3;++j)
                    if(!face::IsBorder<FaceType>(ff,j))
                      vcg::face::FFDetach<FaceType>(ff,j);

                Allocator<MeshType>::DeleteFace(m,ff);
                count_fd++;
              }
            }
          }
                return count_fd;
            }

      /*
      The following functions remove faces that are geometrically "bad" according to edges and area criteria.
      They remove the faces that are out of a given range of area or edges (e.g. faces too large or too small, or with edges too short or too long)
      but that could be topologically correct.
      These functions can optionally take into account only the selected faces.
      */
      template<bool Selected>
          static int RemoveFaceOutOfRangeAreaSel(MeshType& m, ScalarType MinAreaThr=0, ScalarType MaxAreaThr=(std::numeric_limits<ScalarType>::max)())
            {
                FaceIterator fi;
                int count_fd = 0;
        MinAreaThr*=2;
        MaxAreaThr*=2;
        for(fi=m.face.begin(); fi!=m.face.end();++fi)
                if(!(*fi).IsD())
              if(!Selected || (*fi).IsS())
              {
                const ScalarType doubleArea=DoubleArea<FaceType>(*fi);
                          if((doubleArea<=MinAreaThr) || (doubleArea>=MaxAreaThr) )
                          {
                                  Allocator<MeshType>::DeleteFace(m,*fi);
                              count_fd++;
                          }
              }
                return count_fd;
            }

        // alias for the old style. Kept for backward compatibility
      static int RemoveZeroAreaFace(MeshType& m) { return RemoveFaceOutOfRangeArea(m);}

      // Aliases for the functions that do not look at selection
      static int RemoveFaceOutOfRangeArea(MeshType& m, ScalarType MinAreaThr=0, ScalarType MaxAreaThr=(std::numeric_limits<ScalarType>::max)())
      {
        return RemoveFaceOutOfRangeAreaSel<false>(m,MinAreaThr,MaxAreaThr);
      }

            /**
             * Is the mesh only composed by quadrilaterals?
             */
            static bool IsBitQuadOnly(const MeshType &m)
      {
        typedef typename MeshType::FaceType F;
              tri::RequirePerFaceFlags(m);
                for (ConstFaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi) if (!fi->IsD()) {
          unsigned int tmp = fi->Flags()&(F::FAUX0|F::FAUX1|F::FAUX2);
          if ( tmp != F::FAUX0 && tmp != F::FAUX1 && tmp != F::FAUX2) return false;
        }
        return true;
      }


  /**
* Is the mesh only composed by triangles? (non polygonal faces)
*/
  static bool IsBitTriOnly(const MeshType &m)
  {
    tri::RequirePerFaceFlags(m);
    for (ConstFaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi) {
      if ( !fi->IsD()  &&  fi->IsAnyF() ) return false;
    }
    return true;
  }

  static bool IsBitPolygonal(const MeshType &m){
    return !IsBitTriOnly(m);
  }

  /**
   * Is the mesh only composed by quadrilaterals and triangles? (no pentas, etc)
   * It assumes that the bits are consistent. In that case there can be only a single faux edge.
   */
  static bool IsBitTriQuadOnly(const MeshType &m)
  {
    tri::RequirePerFaceFlags(m);
    typedef typename MeshType::FaceType F;
    for (ConstFaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi) if (!fi->IsD()) {
      unsigned int tmp = fi->cFlags()&(F::FAUX0|F::FAUX1|F::FAUX2);
      if ( tmp!=F::FAUX0 && tmp!=F::FAUX1 && tmp!=F::FAUX2 && tmp!=0 ) return false;
    }
    return true;
  }

  /**
   * How many quadrilaterals?
   * It assumes that the bits are consistent. In that case we count the tris with a single faux edge and divide by two.
   */
  static int CountBitQuads(const MeshType &m)
  {
    tri::RequirePerFaceFlags(m);
    typedef typename MeshType::FaceType F;
    int count=0;
    for (ConstFaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi) if (!fi->IsD()) {
      unsigned int tmp = fi->cFlags()&(F::FAUX0|F::FAUX1|F::FAUX2);
      if ( tmp==F::FAUX0 || tmp==F::FAUX1 || tmp==F::FAUX2) count++;
    }
    return count / 2;
  }

  /**
   * How many triangles? (non polygonal faces)
   */
  static int CountBitTris(const MeshType &m)
  {
    tri::RequirePerFaceFlags(m);
    int count=0;
    for (ConstFaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi) if (!fi->IsD()) {
      if (!(fi->IsAnyF())) count++;
    }
    return count;
  }

  /**
   * How many polygons of any kind? (including triangles)
   * it assumes that there are no faux vertexes (e.g vertices completely surrounded by faux edges)
   */
  static int CountBitPolygons(const MeshType &m)
  {
    tri::RequirePerFaceFlags(m);
    int count = 0;
    for (ConstFaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi) if (!fi->IsD())  {
      if (fi->IsF(0)) count++;
      if (fi->IsF(1)) count++;
      if (fi->IsF(2)) count++;
    }
    return m.fn - count/2;
  }

  /**
  * The number of polygonal faces is
  *  FN - EN_f (each faux edge hides exactly one triangular face or in other words a polygon of n edges has n-3 faux edges.)
  * In the general case where a The number of polygonal faces is
  *	 FN - EN_f + VN_f
  *	where:
  *	 EN_f is the number of faux edges.
  *	 VN_f is the number of faux vertices (e.g vertices completely surrounded by faux edges)
  * as a intuitive proof think to a internal vertex that is collapsed onto a border of a polygon:
  * it deletes 2 faces, 1 faux edges and 1 vertex so to keep the balance you have to add back the removed vertex.
  */
  static int CountBitLargePolygons(MeshType &m)
  {
    tri::RequirePerFaceFlags(m);
    UpdateFlags<MeshType>::VertexSetV(m);
    // First loop Clear all referenced vertices
    for (FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
      if (!fi->IsD())
        for(int i=0;i<3;++i) fi->V(i)->ClearV();


    // Second Loop, count (twice) faux edges and mark all vertices touched by non faux edges
    // (e.g vertexes on the boundary of a polygon)
    int countE = 0;
    for (FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
      if (!fi->IsD())  {
        for(int i=0;i<3;++i)
        {
          if (fi->IsF(i))
            countE++;
          else
          {
            fi->V0(i)->SetV();
            fi->V1(i)->SetV();
          }
        }
      }
    // Third Loop, count the number of referenced vertexes that are completely surrounded by faux edges.

    int countV = 0;
    for (VertexIterator vi = m.vert.begin(); vi != m.vert.end(); ++vi)
      if (!vi->IsD() && !vi->IsV()) countV++;

    return m.fn - countE/2 + countV ;
  }


  /**
  * Checks that the mesh has consistent per-face faux edges
  * (the ones that merges triangles into larger polygons).
  * A border edge should never be faux, and faux edges should always be
  * reciprocated by another faux edges.
  * It requires FF adjacency.
  */
  static bool HasConsistentPerFaceFauxFlag(const MeshType &m)
  {
    RequireFFAdjacency(m);
    RequirePerFaceFlags(m);

    for (ConstFaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
      if(!(*fi).IsD())
        for (int k=0; k<3; k++)
          if( ( fi->IsF(k) != fi->cFFp(k)->IsF(fi->cFFi(k)) ) ||
              ( fi->IsF(k) && face::IsBorder(*fi,k)) )
          {
            return false;
          }
    return true;
  }

  /**
   * Count the number of non manifold edges in a polylinemesh, e.g. the edges where there are more than 2 incident faces.
   *
   */
  static int CountNonManifoldEdgeEE( MeshType & m, bool SelectFlag=false)
  {
    assert(m.fn == 0 && m.en >0); // just to be sure we are using an edge mesh...
    RequireEEAdjacency(m);
    tri::UpdateTopology<MeshType>::EdgeEdge(m);

    if(SelectFlag) UpdateSelection<MeshType>::VertexClear(m);

    int nonManifoldCnt=0;
    SimpleTempData<typename MeshType::VertContainer, int > TD(m.vert,0);

    // First Loop, just count how many faces are incident on a vertex and store it in the TemporaryData Counter.
    EdgeIterator ei;
    for (ei = m.edge.begin(); ei != m.edge.end(); ++ei)	if (!ei->IsD())
    {
      TD[(*ei).V(0)]++;
      TD[(*ei).V(1)]++;
    }

    tri::UpdateFlags<MeshType>::VertexClearV(m);
    // Second Loop, Check that each vertex have been seen 1 or 2 times.
    for (VertexIterator vi = m.vert.begin(); vi != m.vert.end(); ++vi)	if (!vi->IsD())
    {
      if( TD[vi] >2 )
      {
        if(SelectFlag) (*vi).SetS();
        nonManifoldCnt++;
      }
    }
    return nonManifoldCnt;
  }

      /**
       * Count the number of non manifold edges in a mesh, e.g. the edges where there are more than 2 incident faces.
       *
       * Note that this test is not enough to say that a mesh is two manifold,
       * you have to count also the non manifold vertexes.
       */
      static int CountNonManifoldEdgeFF( MeshType & m, bool SelectFlag=false)
      {
        RequireFFAdjacency(m);
        int nmfBit[3];
        nmfBit[0]= FaceType::NewBitFlag();
        nmfBit[1]= FaceType::NewBitFlag();
        nmfBit[2]= FaceType::NewBitFlag();


        UpdateFlags<MeshType>::FaceClear(m,nmfBit[0]+nmfBit[1]+nmfBit[2]);

        if(SelectFlag){
          UpdateSelection<MeshType>::VertexClear(m);
          UpdateSelection<MeshType>::FaceClear(m);
        }

        int edgeCnt = 0;
        for (FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
        {
          if (!fi->IsD())
          {
            for(int i=0;i<3;++i)
            if(!IsManifold(*fi,i))
              {
                if(!(*fi).IsUserBit(nmfBit[i]))
                  {
                      ++edgeCnt;
                      if(SelectFlag)
                      {
                        (*fi).V0(i)->SetS();
                        (*fi).V1(i)->SetS();
                      }
                      // follow the ring of faces incident on edge i;
                      face::Pos<FaceType> nmf(&*fi,i);
                      do
                      {
                        if(SelectFlag) nmf.F()->SetS();
                        nmf.F()->SetUserBit(nmfBit[nmf.E()]);
                        nmf.NextF();
                      }
                      while(nmf.f != &*fi);
                  }
              }
            }
          }
        FaceType::DeleteBitFlag(nmfBit[2]);
        FaceType::DeleteBitFlag(nmfBit[1]);
        FaceType::DeleteBitFlag(nmfBit[0]);
        return edgeCnt;
      }

      /** Count (and eventually select) non 2-Manifold vertexes of a mesh
       * e.g. the vertices with a non 2-manif. neighbourhood but that do not belong to not 2-manif edges.
       * typical situation two cones connected by one vertex.
       */
      static int CountNonManifoldVertexFF( MeshType & m, bool selectVert = true )
            {
        RequireFFAdjacency(m);
        if(selectVert) UpdateSelection<MeshType>::VertexClear(m);

                int nonManifoldCnt=0;
                SimpleTempData<typename MeshType::VertContainer, int > TD(m.vert,0);

        // First Loop, just count how many faces are incident on a vertex and store it in the TemporaryData Counter.
                FaceIterator fi;
                for (fi = m.face.begin(); fi != m.face.end(); ++fi)	if (!fi->IsD())
                {
                    TD[(*fi).V(0)]++;
                    TD[(*fi).V(1)]++;
                    TD[(*fi).V(2)]++;
                }

                tri::UpdateFlags<MeshType>::VertexClearV(m);
        // Second Loop.
        // mark out of the game the vertexes that are incident on non manifold edges.
        for (fi = m.face.begin(); fi != m.face.end(); ++fi) if (!fi->IsD())
          {
            for(int i=0;i<3;++i)
              if (!IsManifold(*fi,i))  {
                  (*fi).V0(i)->SetV();
                  (*fi).V1(i)->SetV();
            }
          }
        // Third Loop, for safe vertexes, check that the number of faces that you can reach starting
        // from it and using FF is the same of the previously counted.
                for (fi = m.face.begin(); fi != m.face.end(); ++fi)	if (!fi->IsD())
                {
                    for(int i=0;i<3;i++) if(!(*fi).V(i)->IsV()){
                        (*fi).V(i)->SetV();
                        face::Pos<FaceType> pos(&(*fi),i);

                        int starSizeFF = pos.NumberOfIncidentFaces();

                        if (starSizeFF != TD[(*fi).V(i)])
                        {
              if(selectVert) (*fi).V(i)->SetS();
                            nonManifoldCnt++;
                        }
                    }
                }
                return nonManifoldCnt;
            }

      static void CountEdges( MeshType & m, int &count_e, int &boundary_e )
      {
        tri::RequireFFAdjacency(m);
        count_e=0;
        boundary_e=0;
        UpdateFlags<MeshType>::FaceClearV(m);
        bool counted =false;
        for(FaceIterator fi=m.face.begin();fi!=m.face.end();fi++)
        {
          if(!((*fi).IsD()))
          {
            (*fi).SetV();
            count_e +=3; //assume that we have to increase the number of edges with three
            for(int j=0; j<3; j++)
            {
              if (face::IsBorder(*fi,j)) //If this edge is a border edge
                boundary_e++; // then increase the number of boundary edges
              else if (IsManifold(*fi,j))//If this edge is manifold
              {
                if((*fi).FFp(j)->IsV()) //If the face on the other side of the edge is already selected
                  count_e--; // we counted one edge twice
              }
              else//We have a non-manifold edge
              {
                vcg::face::Pos<FaceType> hei(&(*fi), j , fi->V(j));
                vcg::face::Pos<FaceType> he=hei;
                he.NextF();
                while (he.f!=hei.f)// so we have to iterate all faces that are connected to this edge
                {
                  if (he.f->IsV())// if one of the other faces was already visited than this edge was counted already.
                  {
                    counted=true;
                    break;
                  }
                  else
                  {
                    he.NextF();
                  }
                }
                if (counted)
                {
                  count_e--;
                  counted=false;
                }
              }
            }
          }
        }
      }


  static int CountHoles( MeshType & m)
  {
    int numholev=0;
    FaceIterator fi;

    FaceIterator gi;
    vcg::face::Pos<FaceType> he;
    vcg::face::Pos<FaceType> hei;

    std::vector< std::vector<Point3x> > holes; //indices of vertices

    vcg::tri::UpdateFlags<MeshType>::VertexClearS(m);

    gi=m.face.begin(); fi=gi;

    for(fi=m.face.begin();fi!=m.face.end();fi++)//for all faces do
    {
      for(int j=0;j<3;j++)//for all edges
      {
        if(fi->V(j)->IsS()) continue;

        if(face::IsBorder(*fi,j))//found an unvisited border edge
        {
          he.Set(&(*fi),j,fi->V(j)); //set the face-face iterator to the current face, edge and vertex
          std::vector<Point3x> hole; //start of a new hole
          hole.push_back(fi->P(j)); // including the first vertex
          numholev++;
          he.v->SetS(); //set the current vertex as selected
          he.NextB(); //go to the next boundary edge


          while(fi->V(j) != he.v)//will we do not encounter the first boundary edge.
          {
            Point3x newpoint = he.v->P(); //select its vertex.
            if(he.v->IsS())//check if this vertex was selected already, because then we have an additional hole.
            {
              //cut and paste the additional hole.
              std::vector<Point3x> hole2;
              int index = static_cast<int>(find(hole.begin(),hole.end(),newpoint)
                                           - hole.begin());
              for(unsigned int i=index; i<hole.size(); i++)
                hole2.push_back(hole[i]);

              hole.resize(index);
              if(hole2.size()!=0) //annoying in degenerate cases
                holes.push_back(hole2);
            }
            hole.push_back(newpoint);
            numholev++;
            he.v->SetS(); //set the current vertex as selected
            he.NextB(); //go to the next boundary edge
          }
          holes.push_back(hole);
        }
      }
    }
    return static_cast<int>(holes.size());
  }

      /*
  Compute the set of connected components of a given mesh
  it fills a vector of pair < int , faceptr > with, for each connecteed component its size and a represnant
 */
      static int CountConnectedComponents(MeshType &m)
      {
        std::vector< std::pair<int,FacePointer> > CCV;
        return ConnectedComponents(m,CCV);
      }

      static int ConnectedComponents(MeshType &m, std::vector< std::pair<int,FacePointer> > &CCV)
      {
        tri::RequireFFAdjacency(m);
        CCV.clear();
        tri::UpdateSelection<MeshType>::FaceClear(m);
        std::stack<FacePointer> sf;
        FacePointer fpt=&*(m.face.begin());
        for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi)
        {
          if(!((*fi).IsD()) && !(*fi).IsS())
          {
            (*fi).SetS();
            CCV.push_back(std::make_pair(0,&*fi));
            sf.push(&*fi);
            while (!sf.empty())
            {
              fpt=sf.top();
              ++CCV.back().first;
              sf.pop();
              for(int j=0;j<3;++j)
              {
                if( !face::IsBorder(*fpt,j) )
                {
                  FacePointer l = fpt->FFp(j);
                  if( !(*l).IsS() )
                  {
                    (*l).SetS();
                    sf.push(l);
                  }
                }
              }
            }
          }
        }
        return int(CCV.size());
      }


            /**
            GENUS.

            A topologically invariant property of a surface defined as
            the largest number of non-intersecting simple closed curves that can be
            drawn on the surface without separating it.

      Roughly speaking, it is the number of holes in a surface.
            The genus g of a closed surface, also called the geometric genus, is related to the
            Euler characteristic by the relation $chi$ by $chi==2-2g$.

            The genus of a connected, orientable surface is an integer representing the maximum
            number of cuttings along closed simple curves without rendering the resultant
            manifold disconnected. It is equal to the number of handles on it.

            For general polyhedra the <em>Euler Formula</em> is:

                  V - E + F = 2 - 2G - B

            where V is the number of vertices, F is the number of faces, E is the
            number of edges, G is the genus and B is the number of <em>boundary polygons</em>.

            The above formula is valid for a mesh with one single connected component.
            By considering multiple connected components the formula becomes:

                  V - E + F = 2C - 2Gs - B   ->   2Gs = - ( V-E+F +B -2C)

            where C is the number of connected components and Gs is the sum of
            the genus of all connected components.

            Note that in the case of a mesh with boundaries the intuitive meaning of Genus is less intuitive that it could seem.
            A closed sphere, a sphere with one hole (e.g. a disk) and a sphere with two holes (e.g. a tube) all of them have Genus == 0

            */

            static int MeshGenus(int nvert,int nedges,int nfaces, int numholes, int numcomponents)
            {
                return -((nvert + nfaces - nedges + numholes - 2 * numcomponents) / 2);
            }

            static int MeshGenus(MeshType &m)
            {
                int nvert=m.vn;
                int nfaces=m.fn;
                int boundary_e,nedges;
                CountEdges(m,nedges,boundary_e);
                int numholes=CountHoles(m);
                int numcomponents=CountConnectedComponents(m);
                int G=MeshGenus(nvert,nedges,nfaces,numholes,numcomponents);
                return G;
            }

            /**
             * Check if the given mesh is regular, semi-regular or irregular.
             *
             * Each vertex of a \em regular mesh has valence 6 except for border vertices
             * which have valence 4.
             *
             * A \em semi-regular mesh is derived from an irregular one applying
             * 1-to-4 subdivision recursively. (not checked for now)
             *
             * All other meshes are \em irregular.
             */
            static void IsRegularMesh(MeshType &m, bool &Regular, bool &Semiregular)
            {
              RequireVFAdjacency(m);
                Regular = true;

                VertexIterator vi;

                // for each vertex the number of edges are count
                for (vi = m.vert.begin(); vi != m.vert.end(); ++vi)
                {
                    if (!vi->IsD())
                    {
                        face::Pos<FaceType> he((*vi).VFp(), &*vi);
                        face::Pos<FaceType> ht = he;

                        int n=0;
                        bool border=false;
                        do
                        {
                            ++n;
                            ht.NextE();
                            if (ht.IsBorder())
                                border=true;
                        }
                        while (ht != he);

                        if (border)
                            n = n/2;

                        if ((n != 6)&&(!border && n != 4))
                        {
                            Regular = false;
                            break;
                        }
                    }
                }

                if (!Regular)
                    Semiregular = false;
                else
                {
                    // For now we do not account for semi-regularity
                    Semiregular = false;
                }
            }


  static bool IsCoherentlyOrientedMesh(MeshType &m)
  {
    for (FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
      if (!fi->IsD())
        for(int i=0;i<3;++i)
          if(!face::CheckOrientation(*fi,i))
            return false;

    return true;
  }

  static void OrientCoherentlyMesh(MeshType &m, bool &Oriented, bool &Orientable)
  {
    RequireFFAdjacency(m);
    assert(&Oriented != &Orientable);
    assert(m.face.back().FFp(0));    // This algorithms require FF topology initialized

    Orientable = true;
    Oriented = true;

    tri::UpdateSelection<MeshType>::FaceClear(m);
    std::stack<FacePointer> faces;
    for (FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
    {
      if (!fi->IsD() && !fi->IsS())
      {
        // each face put in the stack is selected (and oriented)
        fi->SetS();
        faces.push(&(*fi));

        // empty the stack
        while (!faces.empty())
        {
          FacePointer fp = faces.top();
          faces.pop();

          // make consistently oriented the adjacent faces
          for (int j = 0; j < 3; j++)
          {
            // get one of the adjacent face
            FacePointer fpaux = fp->FFp(j);
            int iaux = fp->FFi(j);

            if (!fpaux->IsD() && fpaux != fp && face::IsManifold<FaceType>(*fp, j))
            {
              if (!CheckOrientation(*fpaux, iaux))
              {
                Oriented = false;

                if (!fpaux->IsS())
                {
                  face::SwapEdge<FaceType,true>(*fpaux, iaux);
                  assert(CheckOrientation(*fpaux, iaux));
                }
                else
                {
                  Orientable = false;
                  break;
                }
              }

              // put the oriented face into the stack

              if (!fpaux->IsS())
              {
                fpaux->SetS();
                faces.push(fpaux);
              }
            }
          }
        }
      }

      if (!Orientable)	break;
    }
  }


  /// Flip the orientation of the whole mesh flipping all the faces (by swapping the first two vertices)
            static void FlipMesh(MeshType &m, bool selected=false)
      {
        for (FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi) if(!(*fi).IsD())
          if(!selected || (*fi).IsS())
        {
                   face::SwapEdge<FaceType,false>((*fi), 0);
                 if (HasPerWedgeTexCoord(m))
                            std::swap((*fi).WT(0),(*fi).WT(1));
        }
      }
      /// Flip a mesh so that its normals are orented outside.
      /// Just for safety it uses a voting scheme.
      /// It assumes that
      /// mesh has already has coherent normals.
      /// mesh is watertight and signle component.
      static bool FlipNormalOutside(MeshType &m)
      {
        if(m.vert.empty()) return false;

        tri::UpdateNormal<MeshType>::PerVertexAngleWeighted(m);
        tri::UpdateNormal<MeshType>::NormalizePerVertex(m);

        std::vector< VertexPointer > minVertVec;
        std::vector< VertexPointer > maxVertVec;

        // The set of directions to be choosen
        std::vector< Point3x > dirVec;
        dirVec.push_back(Point3x(1,0,0));
        dirVec.push_back(Point3x(0,1,0));
        dirVec.push_back(Point3x(0,0,1));
        dirVec.push_back(Point3x( 1, 1,1));
        dirVec.push_back(Point3x(-1, 1,1));
        dirVec.push_back(Point3x(-1,-1,1));
        dirVec.push_back(Point3x( 1,-1,1));
        for(size_t i=0;i<dirVec.size();++i)
        {
          Normalize(dirVec[i]);
          minVertVec.push_back(&*m.vert.begin());
          maxVertVec.push_back(&*m.vert.begin());
        }
        for (VertexIterator vi = m.vert.begin(); vi != m.vert.end(); ++vi) if(!(*vi).IsD())
        {
          for(size_t i=0;i<dirVec.size();++i)
          {
            if( (*vi).cP().dot(dirVec[i]) < minVertVec[i]->P().dot(dirVec[i])) minVertVec[i] = &*vi;
            if( (*vi).cP().dot(dirVec[i]) > maxVertVec[i]->P().dot(dirVec[i])) maxVertVec[i] = &*vi;
          }
        }

        int voteCount=0;
        ScalarType angleThreshold = cos(math::ToRad(85.0));
        for(size_t i=0;i<dirVec.size();++i)
        {
//          qDebug("Min vert along (%f %f %f) is %f %f %f",dirVec[i][0],dirVec[i][1],dirVec[i][2],minVertVec[i]->P()[0],minVertVec[i]->P()[1],minVertVec[i]->P()[2]);
//          qDebug("Max vert along (%f %f %f) is %f %f %f",dirVec[i][0],dirVec[i][1],dirVec[i][2],maxVertVec[i]->P()[0],maxVertVec[i]->P()[1],maxVertVec[i]->P()[2]);
          if(minVertVec[i]->N().dot(dirVec[i]) > angleThreshold ) voteCount++;
          if(maxVertVec[i]->N().dot(dirVec[i]) < -angleThreshold ) voteCount++;
        }
//        qDebug("votecount = %i",voteCount);
        if(voteCount < int(dirVec.size())/2) return false;
        FlipMesh(m);
        return true;
      }

      // Search and remove small single triangle folds
      // - a face has normal opposite to all other faces
      // - choose the edge that brings to the face f1 containing the vertex opposite to that edge.
      static int RemoveFaceFoldByFlip(MeshType &m, float normalThresholdDeg=175, bool repeat=true)
      {
        RequireFFAdjacency(m);
        RequirePerVertexMark(m);
            //Counters for logging and convergence
            int count, total = 0;

            do {
                tri::UpdateTopology<MeshType>::FaceFace(m);
                tri::UnMarkAll(m);
                count = 0;

                ScalarType NormalThrRad = math::ToRad(normalThresholdDeg);
                ScalarType eps = 0.0001; // this epsilon value is in absolute value. It is a distance from edge in baricentric coords.
                //detection stage
                for(FaceIterator fi=m.face.begin();fi!= m.face.end();++fi ) if(!(*fi).IsV())
                { Point3<ScalarType> NN = vcg::NormalizedNormal((*fi));
                  if( vcg::Angle(NN,vcg::NormalizedNormal(*(*fi).FFp(0))) > NormalThrRad &&
                      vcg::Angle(NN,vcg::NormalizedNormal(*(*fi).FFp(1))) > NormalThrRad &&
                      vcg::Angle(NN,vcg::NormalizedNormal(*(*fi).FFp(2))) > NormalThrRad )
                  {
                    (*fi).SetS();
                    //(*fi).C()=Color4b(Color4b::Red);
                    // now search the best edge to flip
                    for(int i=0;i<3;i++)
                    {
                      Point3<ScalarType> &p=(*fi).P2(i);
                      Point3<ScalarType> L;
                      bool ret = vcg::InterpolationParameters((*(*fi).FFp(i)),vcg::Normal(*(*fi).FFp(i)),p,L);
                      if(ret && L[0]>eps && L[1]>eps && L[2]>eps)
                      {
                        (*fi).FFp(i)->SetS();
                        (*fi).FFp(i)->SetV();
                        //(*fi).FFp(i)->C()=Color4b(Color4b::Green);
                        if(face::CheckFlipEdge<FaceType>( *fi, i ))  {
                                face::FlipEdge<FaceType>( *fi, i );
                                ++count; ++total;
                            }
                      }
                    }
                  }
                }

                // tri::UpdateNormal<MeshType>::PerFace(m);
            }
            while( repeat && count );
            return total;
        }


    static int RemoveTVertexByFlip(MeshType &m, float threshold=40, bool repeat=true)
    {
      RequireFFAdjacency(m);
      RequirePerVertexMark(m);
        //Counters for logging and convergence
        int count, total = 0;

        do {
            tri::UpdateTopology<MeshType>::FaceFace(m);
            tri::UnMarkAll(m);
            count = 0;

            //detection stage
            for(unsigned int index = 0 ; index < m.face.size(); ++index )
            {
                FacePointer f = &(m.face[index]);    float sides[3]; Point3<float> dummy;
                sides[0] = Distance(f->P(0), f->P(1));
                sides[1] = Distance(f->P(1), f->P(2));
                sides[2] = Distance(f->P(2), f->P(0));
                // Find largest triangle side
                int i = std::find(sides, sides+3, std::max( std::max(sides[0],sides[1]), sides[2])) - (sides);
                if( tri::IsMarked(m,f->V2(i) )) continue;

                if( PSDist(f->P2(i),f->P(i),f->P1(i),dummy)*threshold <= sides[i] )
                {
                    tri::Mark(m,f->V2(i));
                    if(face::CheckFlipEdge<FaceType>( *f, i ))  {
                        // Check if EdgeFlipping improves quality
                        FacePointer g = f->FFp(i); int k = f->FFi(i);
                        Triangle3<float> t1(f->P(i), f->P1(i), f->P2(i)), t2(g->P(k), g->P1(k), g->P2(k)),
                                         t3(f->P(i), g->P2(k), f->P2(i)), t4(g->P(k), f->P2(i), g->P2(k));

                        if ( std::min( QualityFace(t1), QualityFace(t2) ) < std::min( QualityFace(t3), QualityFace(t4) ))
                        {
                            face::FlipEdge<FaceType>( *f, i );
                            ++count; ++total;
                        }
                    }

                }
            }

            // tri::UpdateNormal<MeshType>::PerFace(m);
        }
        while( repeat && count );
        return total;
    }

    static int RemoveTVertexByCollapse(MeshType &m, float threshold=40, bool repeat=true)
    {
      RequirePerVertexMark(m);
        //Counters for logging and convergence
        int count, total = 0;

        do {
            tri::UnMarkAll(m);
            count = 0;

            //detection stage
            for(unsigned int index = 0 ; index < m.face.size(); ++index )
            {
                FacePointer f = &(m.face[index]);    float sides[3]; Point3<float> dummy;
                sides[0] = Distance(f->P(0), f->P(1)); sides[1] = Distance(f->P(1), f->P(2)); sides[2] = Distance(f->P(2), f->P(0));
                int i = std::find(sides, sides+3, std::max( std::max(sides[0],sides[1]), sides[2])) - (sides);
                if( tri::IsMarked(m,f->V2(i) )) continue;

                if( PSDist(f->P2(i),f->P(i),f->P1(i),dummy)*threshold <= sides[i] )
                {
                    tri::Mark(m,f->V2(i));

                    int j = Distance(dummy,f->P(i))<Distance(dummy,f->P1(i))?i:(i+1)%3;
                    f->P2(i) = f->P(j);  tri::Mark(m,f->V(j));
                    ++count; ++total;
                }
            }


            tri::Clean<MeshType>::RemoveDuplicateVertex(m);
            tri::Allocator<MeshType>::CompactFaceVector(m);
            tri::Allocator<MeshType>::CompactVertexVector(m);
        }
        while( repeat && count );

        return total;
    }

    static bool SelfIntersections(MeshType &m, std::vector<FaceType*> &ret)
    {
      RequirePerFaceMark(m);
      ret.clear();
      int referredBit = FaceType::NewBitFlag();
      tri::UpdateFlags<MeshType>::FaceClear(m,referredBit);

      TriMeshGrid gM;
      gM.Set(m.face.begin(),m.face.end());

      for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi) if(!(*fi).IsD())
      {
        (*fi).SetUserBit(referredBit);
        Box3< ScalarType> bbox;
        (*fi).GetBBox(bbox);
        std::vector<FaceType*> inBox;
        vcg::tri::GetInBoxFace(m, gM, bbox,inBox);
        bool Intersected=false;
        typename std::vector<FaceType*>::iterator fib;
        for(fib=inBox.begin();fib!=inBox.end();++fib)
        {
          if(!(*fib)->IsUserBit(referredBit) && (*fib != &*fi) )
            if(Clean<MeshType>::TestFaceFaceIntersection(&*fi,*fib)){
              ret.push_back(*fib);
              if(!Intersected) {
                ret.push_back(&*fi);
                Intersected=true;
              }
            }
        }
        inBox.clear();
      }

      FaceType::DeleteBitFlag(referredBit);
      return (ret.size()>0);
    }

      /**
      This function simply test that the vn and fn counters be consistent with the size of the containers and the number of deleted simplexes.
      */
      static bool IsSizeConsistent(MeshType &m)
      {
        int DeletedVertexNum=0;
        for (VertexIterator vi = m.vert.begin(); vi != m.vert.end(); ++vi)
          if((*vi).IsD()) DeletedVertexNum++;

        int DeletedFaceNum=0;
        for (FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
          if((*fi).IsD()) DeletedFaceNum++;

        if(size_t(m.vn+DeletedVertexNum) != m.vert.size()) return false;
        if(size_t(m.fn+DeletedFaceNum) != m.face.size()) return false;

        return true;
      }

      /**
      This function simply test that all the faces have a consistent face-face topology relation.
      useful for checking that a topology modifying algorithm does not mess something.
      */
      static bool IsFFAdjacencyConsistent(MeshType &m)
      {
        RequireFFAdjacency(m);

        for (FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
          if(!(*fi).IsD())
          {
            for(int i=0;i<3;++i)
              if(!FFCorrectness(*fi, i)) return false;
          }
        return true;
      }

/**
      This function simply test that a mesh has some reasonable tex coord.
      */
      static bool HasConsistentPerWedgeTexCoord(MeshType &m)
      {
        tri::RequirePerFaceWedgeTexCoord(m);

        for (FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
          if(!(*fi).IsD())
          { FaceType &f=(*fi);
                     if( ! ( (f.WT(0).N() == f.WT(1).N()) && (f.WT(0).N() == (*fi).WT(2).N()) )  )
                            return false; // all the vertices must have the same index.

                     if((*fi).WT(0).N() <0) return false; // no undefined texture should be allowed
          }
        return true;
      }

      /**
  Simple check that there are no face with all collapsed tex coords.
  */
      static bool HasZeroTexCoordFace(MeshType &m)
      {
        tri::RequirePerFaceWedgeTexCoord(m);

        for (FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
          if(!(*fi).IsD())
          {
            if( (*fi).WT(0).P() == (*fi).WT(1).P() && (*fi).WT(0).P() == (*fi).WT(2).P() ) return false;
          }
        return true;
      }


  /**
        This function test if two triangular faces of a mesh intersect.
        It assumes that the faces (as storage) are different (e.g different address)
        If the two faces are different but coincident (same set of vertexes) return true.
        if the faces share an edge no test is done.
        if the faces share only a vertex, the opposite edge is tested against the face
  */
  static	bool TestFaceFaceIntersection(FaceType *f0,FaceType *f1)
    {
    assert(f0!=f1);
    int sv = face::CountSharedVertex(f0,f1);
    if(sv==3) return true;
    if(sv==0) return (vcg::IntersectionTriangleTriangle<FaceType>((*f0),(*f1)));
    //  if the faces share only a vertex, the opposite edge (as a segment) is tested against the face
    //  to avoid degenerate cases where the two triangles have the opposite edge on a common plane
    //  we offset the segment to test toward the shared vertex
    if(sv==1)
    {
      int i0,i1; ScalarType a,b;
      face::FindSharedVertex(f0,f1,i0,i1);
      Point3f shP = f0->V(i0)->P()*0.5;
      if(vcg::IntersectionSegmentTriangle(Segment3<ScalarType>((*f0).V1(i0)->P()*0.5+shP,(*f0).V2(i0)->P()*0.5+shP), *f1, a, b) )
      {
        // a,b are the param coords of the intersection point of the segment.
        if(a+b>=1 || a<=EPSIL || b<=EPSIL ) return false;
        return true;
      }
      if(vcg::IntersectionSegmentTriangle(Segment3<ScalarType>((*f1).V1(i1)->P()*0.5+shP,(*f1).V2(i1)->P()*0.5+shP), *f0, a, b) )
      {
        // a,b are the param coords of the intersection point of the segment.
        if(a+b>=1 || a<=EPSIL || b<=EPSIL ) return false;
        return true;
      }

     }
        return false;
    }



/**
      This function merge all the vertices that are closer than the given radius
*/
  static int MergeCloseVertex(MeshType &m, const ScalarType radius)
  {
    int mergedCnt=0;
    mergedCnt = ClusterVertex(m,radius);
    RemoveDuplicateVertex(m,true);
    return mergedCnt;
  }

  static int ClusterVertex(MeshType &m, const ScalarType radius)
  {
    if(m.vn==0) return 0;
    // some spatial indexing structure does not work well with deleted vertices...
    tri::Allocator<MeshType>::CompactVertexVector(m);
    typedef vcg::SpatialHashTable<VertexType, ScalarType> SampleSHT;
    SampleSHT sht;
    tri::VertTmark<MeshType> markerFunctor;
    typedef vcg::vertex::PointDistanceFunctor<ScalarType> VDistFunct;
    std::vector<VertexType*> closests;
    int mergedCnt=0;
    sht.Set(m.vert.begin(), m.vert.end());
    UpdateFlags<MeshType>::VertexClearV(m);
    for(VertexIterator viv = m.vert.begin(); viv!= m.vert.end(); ++viv)
      if(!(*viv).IsD() && !(*viv).IsV())
      {
        (*viv).SetV();
        Point3<ScalarType> p = viv->cP();
        Box3<ScalarType> bb(p-Point3<ScalarType>(radius,radius,radius),p+Point3<ScalarType>(radius,radius,radius));
        GridGetInBox(sht, markerFunctor, bb, closests);
        // qDebug("Vertex %i has %i closest", &*viv - &*m.vert.begin(),closests.size());
        for(size_t i=0; i<closests.size(); ++i)
        {
          ScalarType dist = Distance(p,closests[i]->cP());
          if(dist < radius && !closests[i]->IsV())
          {
            //													printf("%f %f \n",dist,radius);
            mergedCnt++;
            closests[i]->SetV();
            closests[i]->P()=p;
          }
        }
      }
    return mergedCnt;
  }


static std::pair<int,int>  RemoveSmallConnectedComponentsSize(MeshType &m, int maxCCSize)
{
  typedef std::pair<int, typename MeshType::FacePointer> IdxFace;
  std::vector<IdxFace> CCV;
      const int TotalCC=ConnectedComponents(m, CCV);
      int DeletedCC=0;
	  if (TotalCC < 2)
        return std::make_pair(TotalCC,DeletedCC);
      std::sort(CCV.begin(), CCV.end(), [](const IdxFace& i, const IdxFace& j) { return i.first > j.first; });
      ConnectedComponentIterator<MeshType> ci;
      for(unsigned i=1; i<CCV.size(); ++i)
      {
        if(CCV[i].first<maxCCSize)
        {
          DeletedCC++;
          for(ci.start(m,CCV[i].second);!ci.completed();++ci)
            Allocator<MeshType>::DeleteFace(m,(**ci));
        }
      }
      return std::make_pair(TotalCC,DeletedCC);
}


/// Remove the connected components smaller than a given diameter
// it returns a pair with the number of connected components and the number of deleted ones.
static std::pair<int,int> RemoveSmallConnectedComponentsDiameter(MeshType &m, ScalarType maxDiameter)
{
  std::vector< std::pair<int, typename MeshType::FacePointer> > CCV;
      const int TotalCC=ConnectedComponents(m, CCV);
      int DeletedCC=0;
	  if (TotalCC < 2)
        return std::make_pair(TotalCC,DeletedCC);
      std::vector<Box3f::ScalarType> diags;
      ConnectedComponentIterator<MeshType> ci;
      for(unsigned i=0;i<CCV.size();++i)
      {
        Box3f bb;
        for(ci.start(m,CCV[i].second);!ci.completed();++ci)
        {
            bb.Add((*ci)->P(0));
            bb.Add((*ci)->P(1));
            bb.Add((*ci)->P(2));
        }
        diags.emplace_back(bb.Diag());
      }
      std::vector<unsigned> indices(CCV.size());
      std::iota(indices.begin(), indices.end(), 0);
      std::sort(indices.begin(), indices.end(), [&diags](unsigned i, unsigned j) { return diags[i] > diags[j]; });
      for (size_t idx=1; idx<indices.size(); ++idx)
      {
        const unsigned i(indices[idx]);
        if(diags[i]<maxDiameter)
        {
          DeletedCC++;
          for(ci.start(m,CCV[i].second);!ci.completed();++ci)
            tri::Allocator<MeshType>::DeleteFace(m,(**ci));
        }
      }
      return std::make_pair(TotalCC,DeletedCC);
}

/// Remove the connected components greater than a given diameter
// it returns a pair with the number of connected components and the number of deleted ones.
static std::pair<int,int> RemoveHugeConnectedComponentsDiameter(MeshType &m, ScalarType minDiameter)
{
  std::vector< std::pair<int, typename MeshType::FacePointer> > CCV;
      const int TotalCC=ConnectedComponents(m, CCV);
      int DeletedCC=0;
	  if (TotalCC < 2)
        return std::make_pair(TotalCC,DeletedCC);
      tri::ConnectedComponentIterator<MeshType> ci;
      for(unsigned int i=0;i<CCV.size();++i)
      {
        Box3f bb;
        std::vector<typename MeshType::FacePointer> FPV;
        for(ci.start(m,CCV[i].second);!ci.completed();++ci)
        {
            FPV.push_back(*ci);
            bb.Add((*ci)->P(0));
            bb.Add((*ci)->P(1));
            bb.Add((*ci)->P(2));
        }
        if(bb.Diag()>minDiameter)
        {
                    DeletedCC++;
          typename std::vector<typename MeshType::FacePointer>::iterator fpvi;
          for(fpvi=FPV.begin(); fpvi!=FPV.end(); ++fpvi)
                        tri::Allocator<MeshType>::DeleteFace(m,(**fpvi));
        }
      }
      return std::make_pair(TotalCC,DeletedCC);
}

        }; // end class
        /*@}*/

    } //End Namespace Tri
} // End Namespace vcg
#endif
