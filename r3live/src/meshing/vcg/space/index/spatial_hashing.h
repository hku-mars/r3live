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

#ifndef VCGLIB_SPATIAL_HASHING
#define VCGLIB_SPATIAL_HASHING

#include <vcg/space/index/grid_util.h>
#include <vcg/space/index/grid_closest.h>
//#include <map>
#include <vector>
#include <algorithm>
#ifdef _WIN32
 #ifndef __MINGW32__
  #include <hash_map>
  #define STDEXT stdext
 #else
  #include <ext/hash_map>
  #define STDEXT __gnu_cxx
 #endif
#else  // We are in the *nix gcc branch
#if (__GNUC__ ==4) && (__GNUC_MINOR__ > 3) && (defined(__DEPRECATED))
  #undef __DEPRECATED // since gcc 4.4 <ext/hash_map> was deprecated and generate warnings. Relax Deprecation Just for this...
  #define ___WE_UNDEFINED_DEPRECATED__
#endif
 #include <ext/hash_map>
 #define STDEXT __gnu_cxx
#if defined(___WE_UNDEFINED_DEPRECATED__)
#define __DEPRECATED
#endif
#endif


namespace vcg{



	// hashing function
	struct HashFunctor : public std::function<size_t (Point3i)>
	{
		enum
		  { // parameters for hash table
				bucket_size = 4, // 0 < bucket_size
				min_buckets = 8
			};

		size_t operator()(const Point3i &p) const
		{
			const size_t _HASH_P0 = 73856093u;
			const size_t _HASH_P1 = 19349663u;
			const size_t _HASH_P2 = 83492791u;

			return size_t(p.V(0))*_HASH_P0 ^  size_t(p.V(1))*_HASH_P1 ^  size_t(p.V(2))*_HASH_P2;
		}

		bool operator()(const Point3i &s1, const Point3i &s2) const
		{ // test if s1 ordered before s2
			return (s1 < s2);
		}
	};




	/** Spatial Hash Table
	Spatial Hashing as described in
	"Optimized Spatial Hashing for Coll	ision Detection of Deformable Objects",
	Matthias Teschner and Bruno Heidelberger and Matthias Muller and Danat Pomeranets and Markus Gross
	*/
	template < typename ObjType,class FLT=double>
	class SpatialHashTable:public BasicGrid<FLT>, public SpatialIndex<ObjType,FLT>
	{

	public:
	typedef SpatialHashTable SpatialHashType;
	typedef ObjType* ObjPtr;
	typedef typename ObjType::ScalarType ScalarType;
	typedef Point3<ScalarType> CoordType;
	typedef typename BasicGrid<FLT>::Box3x Box3x;

	// Hash table definition
	// the hash index directly the grid structure.
	// We use a MultiMap because we need to store many object (faces) inside each cell of the grid.

	typedef typename STDEXT::hash_multimap<Point3i, ObjType *, HashFunctor> HashType;
	typedef typename HashType::iterator HashIterator;
	HashType hash_table; // The real HASH TABLE **************************************

	// This vector is just a handy reference to all the allocated cells,
	// becouse hashed multimaps does not expose a direct list of all the different keys.
	std::vector<Point3i> AllocatedCells;

	// Class to abstract a HashIterator (that stores also the key,
	// while the interface of the generic spatial indexing need only simple object (face) pointers.

	struct CellIterator
	{
		CellIterator(){}
		HashIterator t;
		ObjPtr &operator *(){return (t->second); }
		ObjPtr operator *() const {return (t->second); }
		bool operator != (const CellIterator & p) const {return t!=p.t;}
		void operator ++() {t++;}
	};

  inline bool Empty() const
  {
    return hash_table.empty();
  }

    size_t CellSize(const Point3i &cell)
    {
        return hash_table.count(cell);
    }

		inline bool EmptyCell(const Point3i &cell) const
		{
		 return hash_table.find(cell) == hash_table.end();
		}

		void UpdateAllocatedCells()
		{
			AllocatedCells.clear();
			if(hash_table.empty()) return;
			AllocatedCells.push_back(hash_table.begin()->first);
			for(HashIterator fi=hash_table.begin();fi!=hash_table.end();++fi)
			{
				if(AllocatedCells.back()!=fi->first) AllocatedCells.push_back(fi->first);
			}
		}
protected:

	///insert a new cell
	void InsertObject(ObjType* s, const Point3i &cell)
	{
		//if(hash_table.count(cell)==0) AllocatedCells.push_back(cell);
		hash_table.insert(typename HashType::value_type(cell, s));
	}

    ///remove all the objects in a cell
    void RemoveCell(const Point3i &/*cell*/)
    {
    }


    bool RemoveObject(ObjType* s, const Point3i &cell)
    {
        std::pair<HashIterator,HashIterator> CellRange = hash_table.equal_range(cell);
        CellIterator first; first.t=CellRange.first;
        CellIterator end; end.t=CellRange.second;
        for(CellIterator ci = first; ci!=end;++ci)
        {
            if (*ci == s)
               {
                   hash_table.erase(ci.t);
                   return true;
               }
        }
        return false;
    }

    public:

		vcg::Box3i Add( ObjType* s)
		{
			Box3<ScalarType> b;
			s->GetBBox(b);
			vcg::Box3i bb;
	  this->BoxToIBox(b,bb);
			//then insert all the cell of bb
			for (int i=bb.min.X();i<=bb.max.X();i++)
				for (int j=bb.min.Y();j<=bb.max.Y();j++)
					for (int k=bb.min.Z();k<=bb.max.Z();k++)
						InsertObject(s,vcg::Point3i(i,j,k));

			return bb;
		}

        ///Remove all the objects contained in the cell containing s
        // it removes s too.
        bool RemoveCell(ObjType* s)
        {
            Point3i pi;
            PToIP(s->cP(),pi);
            std::pair<HashIterator,HashIterator> CellRange = hash_table.equal_range(pi);
            hash_table.erase(CellRange.first,CellRange.second);
            return true;
        }    ///insert a new cell

        int CountInSphere(const Point3<ScalarType> &p, const ScalarType radius, std::vector<HashIterator> &inSphVec)
        {
          Box3x b(p-Point3f(radius,radius,radius),p+Point3f(radius,radius,radius));
          vcg::Box3i bb;
          this->BoxToIBox(b,bb);
          ScalarType r2=radius*radius;
          inSphVec.clear();

          for (int i=bb.min.X();i<=bb.max.X();i++)
            for (int j=bb.min.Y();j<=bb.max.Y();j++)
              for (int k=bb.min.Z();k<=bb.max.Z();k++)
              {
                std::pair<HashIterator,HashIterator> CellRange = hash_table.equal_range(Point3i(i,j,k));
                for(HashIterator hi = CellRange.first; hi!=CellRange.second;++hi)
                {
                  if(SquaredDistance(p,hi->second->cP()) <= r2)
                    inSphVec.push_back(hi);
                }
              }
          return inSphVec.size();
        }

        int RemoveInSphere(const Point3<ScalarType> &p, const ScalarType radius)
        {
          std::vector<HashIterator> inSphVec;
          CountInSphere(p,radius,inSphVec);
            for(typename std::vector<HashIterator>::iterator vi=inSphVec.begin(); vi!=inSphVec.end();++vi)
                hash_table.erase(*vi);

            return inSphVec.size();
        }
        // Specialized version that is able to take in input a
        template<class DistanceFunctor>
        int RemoveInSphereNormal(const Point3<ScalarType> &p, const Point3<ScalarType> &n, DistanceFunctor &DF, const ScalarType radius)
        {
            Box3x b(p-Point3f(radius,radius,radius),p+Point3f(radius,radius,radius));
            vcg::Box3i bb;
            this->BoxToIBox(b,bb);
            int cnt=0;
            std::vector<HashIterator> toDel;

            for (int i=bb.min.X();i<=bb.max.X();i++)
                for (int j=bb.min.Y();j<=bb.max.Y();j++)
                    for (int k=bb.min.Z();k<=bb.max.Z();k++)
                    {
                        std::pair<HashIterator,HashIterator> CellRange = hash_table.equal_range(Point3i(i,j,k));
                        for(HashIterator hi = CellRange.first; hi!=CellRange.second;++hi)
                        {
                            if(DF(p,n,hi->second->cP(),hi->second->cN()) <= radius)
                            {
                                cnt++;
                                toDel.push_back(hi);
                            }
                        }
                    }
            for(typename std::vector<HashIterator>::iterator vi=toDel.begin(); vi!=toDel.end();++vi)
                hash_table.erase(*vi);

            return cnt;
        }

	// This version of the removal is specialized for the case where
		// an object has a pointshaped box and using the generic bbox interface is just a waste of time.

        void RemovePunctual( ObjType *s)
        {
            Point3i pi;
            PToIP(s->cP(),pi);
            std::pair<HashIterator,HashIterator> CellRange = hash_table.equal_range(pi);
            for(HashIterator hi = CellRange.first; hi!=CellRange.second;++hi)
            {
                if (hi->second == s)
                   {
                       hash_table.erase(hi);
                       return;
                   }
            }
        }

        void Remove( ObjType* s)
        {
            Box3<ScalarType> b;
            s->GetBBox(b);
            vcg::Box3i bb;
            BoxToIBox(b,bb);
            //then remove the obj from all the cell of bb
            for (int i=bb.min.X();i<=bb.max.X();i++)
                for (int j=bb.min.Y();j<=bb.max.Y();j++)
                    for (int k=bb.min.Z();k<=bb.max.Z();k++)
                        RemoveObject(s,vcg::Point3i(i,j,k));
        }

		/// set an empty spatial hash table
		void InitEmpty(const Box3x &_bbox, vcg::Point3i grid_size)
		{
			Box3x b;
			Box3x &bbox = this->bbox;
			CoordType &dim = this->dim;
			Point3i &siz = this->siz;
			CoordType &voxel = this->voxel;

			assert(!_bbox.IsNull());
			bbox=_bbox;
			dim  = bbox.max - bbox.min;
			assert((grid_size.V(0)>0)&&(grid_size.V(1)>0)&&(grid_size.V(2)>0));
			siz=grid_size;

			voxel[0] = dim[0]/siz[0];
			voxel[1] = dim[1]/siz[1];
			voxel[2] = dim[2]/siz[2];
			hash_table.clear();
		}

		/// Insert a mesh in the grid.
		template <class OBJITER>
			void Set(const OBJITER & _oBegin, const OBJITER & _oEnd, const Box3x &_bbox=Box3x() )
		{
			OBJITER i;
			Box3x b;
			Box3x &bbox = this->bbox;
			CoordType &dim = this->dim;
			Point3i &siz = this->siz;
			CoordType &voxel = this->voxel;

			int _size=(int)std::distance<OBJITER>(_oBegin,_oEnd);
			if(!_bbox.IsNull()) this->bbox=_bbox;
			else
			{
				for(i = _oBegin; i!= _oEnd; ++i)
				{
					(*i).GetBBox(b);
					this->bbox.Add(b);
				}
				///inflate the bb calculated
				bbox.Offset(bbox.Diag()/100.0) ;
			}

			dim  = bbox.max - bbox.min;
			BestDim( _size, dim, siz );
			// find voxel size
			voxel[0] = dim[0]/siz[0];
			voxel[1] = dim[1]/siz[1];
			voxel[2] = dim[2]/siz[2];

			for(i = _oBegin; i!= _oEnd; ++i)
				Add(&(*i));
		}


		///return the simplexes of the cell that contain p
		void GridReal( const Point3<ScalarType> & p, CellIterator & first, CellIterator & last )
		{
			vcg::Point3i _c;
			this->PToIP(p,_c);
			Grid(_c,first,last);
		}

		///return the simplexes on a specified cell
		void Grid( int x,int y,int z, CellIterator & first, CellIterator & last )
		{
			this->Grid(vcg::Point3i(x,y,z),first,last);
		}

		///return the simplexes on a specified cell
		void Grid( const Point3i & _c, CellIterator & first, CellIterator & end )
		{
			std::pair<HashIterator,HashIterator> CellRange = hash_table.equal_range(_c);
			first.t=CellRange.first;
			end.t=CellRange.second;
		}

		void Clear()
		{
			hash_table.clear();
			AllocatedCells.clear();
		}


		template <class OBJPOINTDISTFUNCTOR, class OBJMARKER>
			ObjPtr  GetClosest(OBJPOINTDISTFUNCTOR & _getPointDistance, OBJMARKER & _marker,
			const CoordType & _p, const ScalarType & _maxDist,ScalarType & _minDist, CoordType & _closestPt)
		{
			return (vcg::GridClosest<SpatialHashType,OBJPOINTDISTFUNCTOR,OBJMARKER>(*this,_getPointDistance,_marker, _p,_maxDist,_minDist,_closestPt));
		}


		template <class OBJPOINTDISTFUNCTOR, class OBJMARKER, class OBJPTRCONTAINER,class DISTCONTAINER, class POINTCONTAINER>
			unsigned int GetKClosest(OBJPOINTDISTFUNCTOR & _getPointDistance,OBJMARKER & _marker,
			const unsigned int _k, const CoordType & _p, const ScalarType & _maxDist,OBJPTRCONTAINER & _objectPtrs,
			DISTCONTAINER & _distances, POINTCONTAINER & _points)
		{
			return (vcg::GridGetKClosest<SpatialHashType,
				OBJPOINTDISTFUNCTOR,OBJMARKER,OBJPTRCONTAINER,DISTCONTAINER,POINTCONTAINER>
				(*this,_getPointDistance,_marker,_k,_p,_maxDist,_objectPtrs,_distances,_points));
		}

		template <class OBJPOINTDISTFUNCTOR, class OBJMARKER, class OBJPTRCONTAINER, class DISTCONTAINER, class POINTCONTAINER>
		unsigned int GetInSphere(OBJPOINTDISTFUNCTOR & _getPointDistance,
		OBJMARKER & _marker,
		const CoordType & _p,
		const ScalarType & _r,
		OBJPTRCONTAINER & _objectPtrs,
		DISTCONTAINER & _distances,
		POINTCONTAINER & _points)
		{
			return(vcg::GridGetInSphere<SpatialHashType,
				OBJPOINTDISTFUNCTOR,OBJMARKER,OBJPTRCONTAINER,DISTCONTAINER,POINTCONTAINER>
				(*this,_getPointDistance,_marker,_p,_r,_objectPtrs,_distances,_points));
		}

		template <class OBJMARKER, class OBJPTRCONTAINER>
			unsigned int GetInBox(OBJMARKER & _marker,
			const Box3x _bbox,
			OBJPTRCONTAINER & _objectPtrs)
		{
			return(vcg::GridGetInBox<SpatialHashType,OBJMARKER,OBJPTRCONTAINER>
				  (*this,_marker,_bbox,_objectPtrs));
		}

		template <class OBJRAYISECTFUNCTOR, class OBJMARKER>
			ObjPtr DoRay(OBJRAYISECTFUNCTOR & _rayIntersector, OBJMARKER & _marker, const Ray3<ScalarType> & _ray, const ScalarType & _maxDist, ScalarType & _t)
		{
			return(vcg::GridDoRay<SpatialHashType,OBJRAYISECTFUNCTOR,OBJMARKER>
				  (*this,_rayIntersector,_marker,_ray,_maxDist,_t));
		}


	}; // end class

	/** Spatial Hash Table Dynamic
	Update the Hmark value on the simplex for dynamic updating of contents of the cell.
	The simplex must have the HMark() function.
	*/
	template < typename ContainerType,class FLT=double>
	class DynamicSpatialHashTable: public SpatialHashTable<ContainerType,FLT>
	{
	public:
		typedef typename SpatialHashTable<ContainerType,FLT>::CoordType CoordType;
		typedef typename SpatialHashTable<ContainerType,FLT>::ObjType ObjType;
		typedef typename SpatialHashTable<ContainerType,FLT>::ObjPtr ObjPtr;
		typedef typename SpatialHashTable<ContainerType,FLT>::Box3x Box3x;
		typedef typename SpatialHashTable<ContainerType,FLT>::CellIterator CellIterator;

		void _UpdateHMark(ObjType* s){ s->HMark() = this->tempMark;}

		void getInCellUpdated(vcg::Point3i cell,std::vector<ObjPtr> &elems)
		{
			CellIterator first,last,l;
			Grid(cell,first,last);
			for (l=first;l!=last;l++)
			{
				if ((l->second)>=(**l).HMark())
					elems.push_back(&(**l));
			}
		}

	};





}// end namespace

#endif
