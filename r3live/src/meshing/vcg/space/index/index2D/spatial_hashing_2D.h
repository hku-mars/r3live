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

#ifndef VCGLIB_SPATIAL_HASHING_2D
#define VCGLIB_SPATIAL_HASHING_2D

#include <vcg/space/index/index2D/grid_util_2D.h>
#include <vcg/space/index/index2D/grid_closest_2D.h>
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
	struct HashFunctor2D : public std::unary_function<Point2i, size_t>
	{
		enum
		{ // parameters for hash table
			bucket_size = 4, // 0 < bucket_size
			min_buckets = 8
		};

        size_t operator()(const Point2i &p) const
        {
            const size_t _HASH_P0 = 73856093u;
            const size_t _HASH_P1 = 19349663u;
            //const size_t _HASH_P2 = 83492791u;

            return size_t(p.V(0))*_HASH_P0 ^  size_t(p.V(1))*_HASH_P1;// ^  size_t(p.V(2))*_HASH_P2;
        }

        bool operator()(const Point2i &s1, const Point2i &s2) const
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
    class SpatialHashTable2D:public BasicGrid2D<FLT>, public SpatialIndex2D<ObjType,FLT>
    {

    public:
        typedef SpatialHashTable2D SpatialHashType;
        typedef ObjType* ObjPtr;
        typedef typename ObjType::ScalarType ScalarType;
        typedef Point2<ScalarType> CoordType;
        typedef typename BasicGrid2D<FLT>::Box2x Box2x;

        // Hash table definition
        // the hash index directly the grid structure.
        // We use a MultiMap because we need to store many object (faces) inside each cell of the grid.

		typedef typename STDEXT::hash_multimap<Point2i, ObjType *, HashFunctor2D> HashType;
		typedef typename HashType::iterator HashIterator;
		HashType hash_table; // The real HASH TABLE **************************************

        // This vector is just a handy reference to all the allocated cells,
        // becouse hashed multimaps does not expose a direct list of all the different keys.
        std::vector<Point2i> AllocatedCells;

        ///the size of the diagonal of each cell
        ScalarType cell_size;

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
        void InsertObject(ObjType* s, const Point2i &cell)
        {
            hash_table.insert(typename HashType::value_type(cell, s));
        }

        ///remove all the objects in a cell
        void RemoveCell(const Point3i &/*cell*/)
        {
        }


        bool RemoveObject(ObjType* s, const Point2i &cell)
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

        void AddBox(ObjType* s,
                    const Box2<ScalarType> &b)
        {
            vcg::Box2i bb;
            this->BoxToIBox(b,bb);
            //then insert all the cell of bb
            for (int i=bb.min.X();i<=bb.max.X();i++)
                for (int j=bb.min.Y();j<=bb.max.Y();j++)
                    InsertObject(s,vcg::Point2i(i,j));
        }

        void AddBoxes(ObjType* s,
                    const std::vector<Box2<ScalarType> > &boxes)
        {
            std::vector<vcg::Point2i> Indexes;
            for (unsigned int i=0;i<boxes.size();i++)
            {
                vcg::Box2i bb;
                this->BoxToIBox(boxes[i],bb);
                //then insert all the cell of bb
                for (int i=bb.min.X();i<=bb.max.X();i++)
                    for (int j=bb.min.Y();j<=bb.max.Y();j++)
                        Indexes.push_back(vcg::Point2i(i,j));
            }
            std::sort(Indexes.begin(),Indexes.end());
            std::vector<vcg::Point2i>::iterator it=std::unique(Indexes.begin(),Indexes.end());
            Indexes.resize( it - Indexes.begin() );

            for (int i=0;i<Indexes.size();i++)
                InsertObject(s,Indexes[i]);
        }

    public:

        void Add( ObjType* s,bool subdivideBox=false)
        {

            if (!subdivideBox)
            {
                Box2<ScalarType> b;
                s->GetBBox(b);
                AddBox(s,b);
            }
            else
            {
                std::vector<Box2<ScalarType> > Boxes;
                s->GetSubBBox(cell_size,Boxes);
                //for (unsigned int i=0;i<Boxes.size();i++)
                //    AddBox(s,Boxes[i]);
                AddBoxes(s,Boxes);
            }
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

    /*	int RemoveInSphere(const Point3<ScalarType> &p, const ScalarType radius)
        {
            Box3x b(p-Point3f(radius,radius,radius),p+Point3f(radius,radius,radius));
            vcg::Box3i bb;
            this->BoxToIBox(b,bb);
            ScalarType r2=radius*radius;
            int cnt=0;
            std::vector<HashIterator> toDel;

            for (int i=bb.min.X();i<=bb.max.X();i++)
                for (int j=bb.min.Y();j<=bb.max.Y();j++)
                    for (int k=bb.min.Z();k<=bb.max.Z();k++)
                    {
                        std::pair<HashIterator,HashIterator> CellRange = hash_table.equal_range(Point3i(i,j,k));
                        for(HashIterator hi = CellRange.first; hi!=CellRange.second;++hi)
                        {
                            if(SquaredDistance(p,hi->second->cP()) <= r2)
                            {
                                cnt++;
                                toDel.push_back(hi);
                            }
                        }
                    }
                    for(typename std::vector<HashIterator>::iterator vi=toDel.begin(); vi!=toDel.end();++vi)
                        hash_table.erase(*vi);

                    return cnt;
        }*/
        //// Specialized version that is able to take in input a
        //template<class DistanceFunctor>
        //int RemoveInSphereNormal(const Point3<ScalarType> &p, const Point3<ScalarType> &n, DistanceFunctor &DF, const ScalarType radius)
        //{
        //	Box3x b(p-Point3f(radius,radius,radius),p+Point3f(radius,radius,radius));
        //	vcg::Box3i bb;
        //	this->BoxToIBox(b,bb);
        //	int cnt=0;
        //	std::vector<HashIterator> toDel;

        //	for (int i=bb.min.X();i<=bb.max.X();i++)
        //		for (int j=bb.min.Y();j<=bb.max.Y();j++)
        //			for (int k=bb.min.Z();k<=bb.max.Z();k++)
        //			{
        //				std::pair<HashIterator,HashIterator> CellRange = hash_table.equal_range(Point3i(i,j,k));
        //				for(HashIterator hi = CellRange.first; hi!=CellRange.second;++hi)
        //				{
        //					if(DF(p,n,hi->second->cP(),hi->second->cN()) <= radius)
        //					{
        //						cnt++;
        //						toDel.push_back(hi);
        //					}
        //				}
        //			}
        //			for(typename std::vector<HashIterator>::iterator vi=toDel.begin(); vi!=toDel.end();++vi)
        //				hash_table.erase(*vi);

        //			return cnt;
        //}

        //// This version of the removal is specialized for the case where
        //// an object has a pointshaped box and using the generic bbox interface is just a waste of time.

        //void RemovePunctual( ObjType *s)
        //{
        //	Point3i pi;
        //	PToIP(s->cP(),pi);
        //	std::pair<HashIterator,HashIterator> CellRange = hash_table.equal_range(pi);
        //	for(HashIterator hi = CellRange.first; hi!=CellRange.second;++hi)
        //	{
        //		if (hi->second == s)
        //		{
        //			hash_table.erase(hi);
        //			return;
        //		}
        //	}
        //}

        void Remove( ObjType* s)
        {
            Box2<ScalarType> b;
            s->GetBBox(b);
            vcg::Box2i bb;
            BoxToIBox(b,bb);
            //then remove the obj from all the cell of bb
            for (int i=bb.min.X();i<=bb.max.X();i++)
                for (int j=bb.min.Y();j<=bb.max.Y();j++)
                    //for (int k=bb.min.Z();k<=bb.max.Z();k++)
                        RemoveObject(s,vcg::Point2i(i,j));//,k));
        }

        /// set an empty spatial hash table
        void InitEmpty(const Box2x &_bbox, vcg::Point2i grid_size)
        {
            Box2x b;
            Box2x &bbox = this->bbox;
            CoordType &dim = this->dim;
            Point2i &siz = this->siz;
            CoordType &voxel = this->voxel;

            assert(!_bbox.IsNull());
            bbox=_bbox;
            dim  = bbox.max - bbox.min;
            assert((grid_size.V(0)>0)&&(grid_size.V(1)>0));//&&(grid_size.V(2)>0));
            siz=grid_size;

            voxel[0] = dim[0]/siz[0];
            voxel[1] = dim[1]/siz[1];
      cell_size=voxel.Norm();

            hash_table.clear();
        }

        /// Insert a mesh in the grid.
        /*template <class OBJITER>
        void Set(const OBJITER & _oBegin, const OBJITER & _oEnd, const Box2x &_bbox=Box2x() )
        {
            OBJITER i;
            Box2x b;
            Box2x &bbox = this->bbox;
            CoordType &dim = this->dim;
            Point2i &siz = this->siz;
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
            BestDim2D( _size, dim, siz );
            // find voxel size
            voxel[0] = dim[0]/siz[0];
            voxel[1] = dim[1]/siz[1];
            //voxel[2] = dim[2]/siz[2];

            for(i = _oBegin; i!= _oEnd; ++i)
                Add(&(*i));
        }*/


        /// Insert a mesh in the grid.
        template <class OBJITER>
        void Set(const OBJITER & _oBegin, const OBJITER & _oEnd,
                 bool subdivideBox=false,const Box2x &_bbox=Box2x() )
        {
            OBJITER i;
            Box2x b;
            Box2x &bbox = this->bbox;
            CoordType &dim = this->dim;
            Point2i &siz = this->siz;
            CoordType &voxel = this->voxel;

            int _size=(int)std::distance<OBJITER>(_oBegin,_oEnd);
            if(!_bbox.IsNull()) this->bbox=_bbox;
            else
            {
                for(i = _oBegin; i!= _oEnd; ++i)
                {
                    (*i)->GetBBox(b);
                    this->bbox.Add(b);
                }
                ///inflate the bb calculated
                bbox.Offset(bbox.Diag()/100.0) ;
            }

            dim  = bbox.max - bbox.min;
            BestDim2D( _size, dim, siz );
            // find voxel size
            voxel[0] = dim[0]/siz[0];
            voxel[1] = dim[1]/siz[1];
            cell_size=voxel.Norm();

            for(i = _oBegin; i!= _oEnd; ++i)
            {
                Add(&(*i),subdivideBox);
            }
        }

                /// Insert a mesh in the grid.
        template <class OBJITER>
        void SetByPointers(const OBJITER & _oBegin, const OBJITER & _oEnd,
                           const Point2i & cellsize=Point2i(-1,-1), bool subdivideBox=false,const Box2x &_bbox=Box2x() )
        {
            OBJITER i;
            Box2x b;
            Box2x &bbox = this->bbox;
            CoordType &dim = this->dim;
            Point2i &siz = this->siz;
            CoordType &voxel = this->voxel;

            int _size=(int)std::distance<OBJITER>(_oBegin,_oEnd);
            if(!_bbox.IsNull()) this->bbox=_bbox;
            else
            {
                for(i = _oBegin; i!= _oEnd; ++i)
                {
                    (*i)->GetBBox(b);
                    this->bbox.Add(b);
                }
                ///inflate the bb calculated
                bbox.Offset(bbox.Diag()/100.0) ;
            }

            if (cellsize[0] < 0 && cellsize[1] < 0)
                        {
                            // cell size estimation
                            dim  = bbox.max - bbox.min;
                            BestDim2D( _size, dim, siz );
                            voxel[0] = dim[0]/siz[0];
                            voxel[1] = dim[1]/siz[1];
                        }
                        else
                        {
                            // cell size assignment
                            voxel[0] = cellsize[0];
                            voxel[1] = cellsize[1];
                        }

            cell_size=voxel.Norm();

            for(i = _oBegin; i!= _oEnd; ++i)
            {
                Add(*i,subdivideBox);
            }
        }

        ///return the simplexes of the cell that contain p
        void GridReal( const Point2<ScalarType> & p, CellIterator & first, CellIterator & last )
        {
            vcg::Point2i _c;
            this->PToIP(p,_c);
            Grid(_c,first,last);
        }

        ///return the simplexes on a specified cell
        void Grid( int x,int y, CellIterator & first, CellIterator & last )
        {
            this->Grid(vcg::Point2i(x,y),first,last);
        }

        ///return the simplexes on a specified cell
        void Grid( const Point2i & _c, CellIterator & first, CellIterator & end )
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


        /*template <class OBJPOINTDISTFUNCTOR, class OBJMARKER>
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
        }*/

        template <class OBJMARKER, class OBJPTRCONTAINER>
        unsigned int GetInBox(OBJMARKER & _marker,
                            const Box2x _bbox,
                            OBJPTRCONTAINER & _objectPtrs)
        {
            _objectPtrs.clear();
            return(vcg::GridGetInBox2D<SpatialHashType,OBJMARKER,OBJPTRCONTAINER>
                (*this,_marker,_bbox,_objectPtrs));
        }

        template <class OBJMARKER, class OBJPTRCONTAINER>
        unsigned int GetInBoxes(OBJMARKER & _marker,
                            const std::vector<Box2x> &_bbox,
                            OBJPTRCONTAINER  &_objectPtrs)
        {
            _objectPtrs.clear();
            return(vcg::GridGetInBoxes2D<SpatialHashType,OBJMARKER,OBJPTRCONTAINER>
                  (*this,_marker,_bbox,_objectPtrs));
        }
    }; // end class



    } // end namespace

#endif
