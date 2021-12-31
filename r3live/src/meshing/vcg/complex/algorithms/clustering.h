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

#ifndef __VCGLIB_CLUSTERING
#define __VCGLIB_CLUSTERING

#include<vcg/complex/complex.h>
#include <vcg/complex/algorithms/clean.h>
#include<vcg/space/triangle3.h>
#include<vcg/complex/algorithms/update/topology.h>
#include<vcg/space/index/grid_util.h>

#include <iostream>
#include <math.h>
#include <limits>

// some stuff for portable hashes...
#ifdef WIN32
 #ifndef __MINGW32__
  #include <hash_map>
  #include <hash_set>
  #define STDEXT stdext
 #else
  #include <ext/hash_map>
  #include <ext/hash_set>
  #define STDEXT __gnu_cxx
 #endif
#else
 #include <ext/hash_map>
 #include <ext/hash_set>
 #define STDEXT __gnu_cxx
#endif



namespace vcg{
namespace tri{
#define HASH_P0 73856093
#define HASH_P1 19349663
#define HASH_P2 83492791

class HashedPoint3i : public Point3i
{
public:

  size_t Hash() const
  {
    return (V(0)*HASH_P0 ^ V(1)*HASH_P1 ^ V(2)*HASH_P2);
  }

  operator size_t () const
  {return Hash();}
};

// needed for gcc compilation
#ifndef _MSC_VER
}} namespace STDEXT {
  template <> struct hash<vcg::tri::HashedPoint3i>{
  inline	size_t	operator ()(const vcg::tri::HashedPoint3i &p) const {return size_t(p);}
};
} namespace vcg{ namespace tri{
#endif

//
template<class MeshType  >
class  NearestToCenter
{
  typedef typename MeshType::ScalarType ScalarType;
  typedef typename MeshType::CoordType CoordType;
  typedef typename MeshType::VertexType  VertexType;
  typedef typename MeshType::FaceType  FaceType;
  typedef BasicGrid<typename MeshType::ScalarType> GridType;

public:
  inline void AddVertex(MeshType &/*m*/, GridType &g, Point3i &pi, VertexType &v)
  {
    CoordType c;
    g.IPiToBoxCenter(pi,c);
    ScalarType newDist = Distance(c,v.cP());
    if(!valid || newDist < bestDist)
    {
      valid=true;
      bestDist=newDist;
      bestPos=v.cP();
      bestN=v.cN();
      orig=&v;
    }
  }
  inline void AddFaceVertex(MeshType &/*m*/, FaceType &/*f*/, int /*i*/)    {		assert(0);}
  NearestToCenter(): valid(false){}

  CoordType bestPos;
  CoordType bestN;
  ScalarType bestDist;
  bool valid;
  int id;
  VertexType *orig;
  CoordType Pos() const
  {
    assert(valid);
    return bestPos;
  }
  Color4b Col() const {return Color4b::White;}
  CoordType N() const {return bestN;}
  VertexType * Ptr() const {return orig;}

};

template<class MeshType>
class  AverageColorCell
{
  typedef typename MeshType::CoordType CoordType;
  typedef typename MeshType::FaceType  FaceType;
  typedef typename MeshType::VertexType  VertexType;

  typedef BasicGrid<typename MeshType::ScalarType> GridType;

public:
  inline void AddFaceVertex(MeshType &/*m*/, FaceType &f, int i)
  {
    p+=f.cV(i)->cP();
    c+=CoordType(f.cV(i)->C()[0],f.cV(i)->C()[1],f.cV(i)->C()[2]);

    // we prefer to use the un-normalized face normal so small faces facing away are dropped out
    // and the resulting average is weighed with the size of the faces falling here.
    n+=f.cN();
    cnt++;
  }
  inline void AddVertex(MeshType &m, GridType &/*g*/, Point3i &/*pi*/, VertexType &v)
  {
    p+=v.cP();
    n+=v.cN();
    if(tri::HasPerVertexColor(m))
       c+=CoordType(v.C()[0],v.C()[1],v.C()[2]);
    cnt++;
  }

  AverageColorCell(): p(0,0,0), n(0,0,0), c(0,0,0),cnt(0){}
  CoordType p;
  CoordType n;
  CoordType c;
  int cnt;
  int id;
  Color4b Col() const
  {
    return Color4b(c[0]/cnt,c[1]/cnt,c[2]/cnt,255);
  }

  CoordType      N() const {return n;}
  VertexType * Ptr() const {return 0;}
  CoordType    Pos() const { return p/cnt; }
};


/*
  Metodo di clustering
*/
template<class MeshType, class CellType>
class Clustering
{
 public:
  typedef typename MeshType::ScalarType  ScalarType;
  typedef typename MeshType::CoordType CoordType;
  typedef typename MeshType::VertexType  VertexType;
  typedef typename MeshType::FaceType  FaceType;
  typedef typename MeshType::VertexPointer  VertexPointer;
  typedef typename MeshType::VertexIterator VertexIterator;
  typedef typename MeshType::FaceIterator   FaceIterator;

  // DuplicateFace == bool means that during the clustering doublesided surface (like a thin shell) that would be clustered to a single surface
  // will be merged into two identical but opposite faces.
  // So in practice:
  // DuplicateFace=true a model with looks ok if you enable backface culling
  // DuplicateFace=false a model with looks ok if you enable doublesided lighting and disable backfaceculling

  bool DuplicateFaceParam;

  // This class keeps the references to the three cells where a face has its vertexes.
    class SimpleTri
  {
  public:
    CellType *v[3];
    int ii(int i) const {return *((int *)(&(v[i])));}
    bool operator < ( const SimpleTri &p) const {
      return	(v[2]!=p.v[2])?(v[2]<p.v[2]):
                (v[1]!=p.v[1])?(v[1]<p.v[1]):
                (v[0]<p.v[0]);
      }

    // Sort the vertex of the face maintaining the original face orientation (it only ensure that v0 is the minimum)
    void sortOrient()
    {
      if(v[1] < v[0] && v[1] < v[2] ) { std::swap(v[0],v[1]); std::swap(v[1],v[2]); return; } // v1 was the minimum
      if(v[2] < v[0] && v[2] < v[1] ) { std::swap(v[0],v[2]); std::swap(v[1],v[2]); return; } // v2 was the minimum
      return; // v0 was the minimum;
    }
    void sort()
    {
      if(v[0] > v[1] ) std::swap(v[0],v[1]); // now v0 < v1
      if(v[0] > v[2] ) std::swap(v[0],v[2]); // now v0 is the minimum
      if(v[1] > v[2] ) std::swap(v[1],v[2]); // sorted!
    }
    // Hashing Function;
    operator size_t () const
    {
      return (ii(0)*HASH_P0 ^ ii(1)*HASH_P1 ^ ii(2)*HASH_P2);
    }
  };


  // The init function Take two parameters
  // _size is the approximate total number of cells composing the grid surrounding the objects (usually a large number)
  //       eg _size==1.000.000 means a 100x100x100 grid
  // _cellsize is the absolute length of the edge of the grid cell.
  //       eg _cellsize==2.0 means that all the vertexes in a 2.0x2.0x2.0 cell are clustered togheter

  // Notes:
  // _size is used only if the cell edge IS zero.
  // _cellsize gives you an absolute measure of the maximum error introduced
  //           during the simplification (e.g. half of the cell edge length)


  void Init(Box3<ScalarType> _mbb, int _size, ScalarType _cellsize=0)
  {
        GridCell.clear();
        TriSet.clear();
    Grid.bbox=_mbb;
    ///inflate the bb calculated
      ScalarType infl = (_cellsize == (ScalarType)0) ? (Grid.bbox.Diag() / _size) : (_cellsize);
      Grid.bbox.min-=CoordType(infl,infl,infl);
      Grid.bbox.max+=CoordType(infl,infl,infl);
    Grid.dim  = Grid.bbox.max - Grid.bbox.min;
    if(		_cellsize==0)
        BestDim( _size, Grid.dim, Grid.siz );
    else
      Grid.siz = Point3i::Construct(Grid.dim / _cellsize);

				// find voxel size
		Grid.voxel[0] = Grid.dim[0]/Grid.siz[0];
		Grid.voxel[1] = Grid.dim[1]/Grid.siz[1];
		Grid.voxel[2] = Grid.dim[2]/Grid.siz[2];
  }

  BasicGrid<ScalarType> Grid;

#ifdef _MSC_VER
  STDEXT::hash_set<SimpleTri> TriSet;
  typedef typename STDEXT::hash_set<SimpleTri>::iterator TriHashSetIterator;
#else
  struct SimpleTriHashFunc{
    inline	size_t	operator ()(const SimpleTri &p) const {return size_t(p);}
  };
  STDEXT::hash_set<SimpleTri,SimpleTriHashFunc> TriSet;
  typedef typename STDEXT::hash_set<SimpleTri,SimpleTriHashFunc>::iterator TriHashSetIterator;
#endif

  STDEXT::hash_map<HashedPoint3i,CellType> GridCell;


	void AddPointSet(MeshType &m, bool UseOnlySelected=false)
	{
		VertexIterator vi;
		for(vi=m.vert.begin();vi!=m.vert.end();++vi)
			if(!(*vi).IsD())
				if(!UseOnlySelected || (*vi).IsS())
					{
						HashedPoint3i pi;
						Grid.PToIP((*vi).cP(), pi );
						GridCell[pi].AddVertex(m,Grid,pi,*(vi));
					}
	}

  void AddMesh(MeshType &m)
  {
    FaceIterator fi;
    for(fi=m.face.begin();fi!=m.face.end();++fi) if(!(*fi).IsD())
            {
                HashedPoint3i pi;
                SimpleTri st;
                for(int i=0;i<3;++i)
                {
                    Grid.PToIP((*fi).cV(i)->cP(), pi );
                    st.v[i]=&(GridCell[pi]);
                    st.v[i]->AddFaceVertex(m,*(fi),i);
                }
                if( (st.v[0]!=st.v[1]) && (st.v[0]!=st.v[2]) && (st.v[1]!=st.v[2]) )
                { // if we allow the duplication of faces we sort the vertex only partially (to maintain the original face orientation)
                    if(DuplicateFaceParam) st.sortOrient();
                                        else st.sort();
                    TriSet.insert(st);
                }
            //  printf("Inserted %8i triangles, clustered to %8i tri and %i cells\n",distance(m.face.begin(),fi),TriSet.size(),GridCell.size());
            }
  }

  int CountPointSet() {return GridCell.size(); }

  void SelectPointSet(MeshType &m)
  {
        typename STDEXT::hash_map<HashedPoint3i,CellType>::iterator gi;
                UpdateSelection<MeshType>::VertexClear(m);
        for(gi=GridCell.begin();gi!=GridCell.end();++gi)
    {
      VertexType *ptr=(*gi).second.Ptr();
            if(ptr && ( ptr >= &*m.vert.begin() )  &&  ( ptr <= &*(m.vert.end() - 1) )  )
                    ptr->SetS();
    }
    }
  void ExtractPointSet(MeshType &m)
  {
    m.Clear();

        if (GridCell.empty()) return;

    Allocator<MeshType>::AddVertices(m,GridCell.size());
    typename STDEXT::hash_map<HashedPoint3i,CellType>::iterator gi;
    int i=0;
    for(gi=GridCell.begin();gi!=GridCell.end();++gi)
    {
      m.vert[i].P()=(*gi).second.Pos();
      m.vert[i].N()=(*gi).second.N();
     if(HasPerVertexColor(m))
        m.vert[i].C()=(*gi).second.Col();
            ++i;
    }

  }

  void ExtractMesh(MeshType &m)
  {
    m.Clear();

    if (GridCell.empty())  return;

    Allocator<MeshType>::AddVertices(m,GridCell.size());
    typename STDEXT::hash_map<HashedPoint3i,CellType>::iterator gi;
    int i=0;
    for(gi=GridCell.begin();gi!=GridCell.end();++gi)
    {
      m.vert[i].P()=(*gi).second.Pos();
      m.vert[i].N()=(*gi).second.N();
      if(HasPerVertexColor(m))
        m.vert[i].C()=(*gi).second.Col();
      (*gi).second.id=i;
      ++i;
    }

    Allocator<MeshType>::AddFaces(m,TriSet.size());
    TriHashSetIterator ti;
    i=0;
    for(ti=TriSet.begin();ti!=TriSet.end();++ti)
    {
      m.face[i].V(0)=&(m.vert[(*ti).v[0]->id]);
      m.face[i].V(1)=&(m.vert[(*ti).v[1]->id]);
      m.face[i].V(2)=&(m.vert[(*ti).v[2]->id]);
      // if we are merging faces even when opposite we choose
      // the best orientation according to the averaged normal
      if(!DuplicateFaceParam)
      {
          CoordType N=vcg::Normal(m.face[i]);
      int badOrient=0;
      if( N.dot((*ti).v[0]->N()) <0) ++badOrient;
      if( N.dot((*ti).v[1]->N()) <0) ++badOrient;
      if( N.dot((*ti).v[2]->N()) <0) ++badOrient;
      if(badOrient>2)
          std::swap(m.face[i].V(0),m.face[i].V(1));
      }
      i++;
    }

  }
}; //end class clustering
 } // namespace tri
} // namespace vcg

#endif
