/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2005                                                \/)\/    *
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
/****************************************************************************
History

$Log: not supported by cvs2svn $
Revision 1.16  2006/10/02 09:34:03  cignoni
Re-added diff 1.12 by Pietroni (cancelled by previous rollback):
in GridDoRay function the RayIterator must be initialized with maximum distance

Revision 1.15  2006/10/02 09:28:45  cignoni
Reverted to version 1.10 to nullify dangerous marfr960's changes

Revision 1.10  2006/05/17 12:48:52  pietroni
corrected bug in GridGetInBox function

Revision 1.9  2006/01/27 09:58:47  corsini
fix signed/unsigned mismatch

Revision 1.8  2005/12/06 18:00:39  pietroni
added deleted objects control for GridClosest() function call

Revision 1.7  2005/12/02 00:30:27  cignoni
Corrected typename usage and removed excess ';' from end of template functions, for gcc compiling

Revision 1.6  2005/10/03 13:57:32  pietroni
added GridGetInSphere and GridGetInBox functions

Revision 1.5  2005/10/02 23:18:06  cignoni
Small bug in the computation of the intersection between the todo box and the grid bbox that failed for extrema points.

Revision 1.4  2005/09/30 15:12:16  cignoni
Completely rewrote the GridClosest, now it:
- works for point out of the grid
- expands the box in a distance coherent way
- does not re-visit already visited cells
- shorter code!!
( still to be tested :) )

Revision 1.3  2005/09/30 13:15:48  pietroni
added functions:
- GetKClosest
- DoRay

Revision 1.2  2005/09/28 08:27:11  cignoni
Added a control to avoid multiple check of the same cells during radial expansion
Still miss some code to properly initialize when point is out of the BBox of the grid.

Revision 1.1  2005/09/27 15:09:38  cignoni
First Version


****************************************************************************/

///** Returns the closest posistion of a point _p and its distance
//@param _p a 3d point
//@param _maxDist maximum distance not to search beyond.
//@param _getPointDistance (templated type) a functor object used to calculate distance from a grid object to the point _p.
//@param _minDist the returned closest distance
//@param _closestPt the returned closest point
//@return The closest element
//*/
///*
//	A DISTFUNCT object must implement an operator () with signature:
//		bool operator () (const ObjType& obj, const CoordType & _p, ScalarType & min_dist, CoordType & _closestPt);
//*/
#ifndef __VCGLIB_GRID_CLOSEST
#define __VCGLIB_GRID_CLOSEST

#include <vcg/space/index/space_iterators.h>

namespace vcg{

	template <class SPATIAL_INDEX, class OBJPOINTDISTFUNCTOR, class OBJMARKER>
		typename SPATIAL_INDEX::ObjPtr  GridClosest(SPATIAL_INDEX &Si, 
		OBJPOINTDISTFUNCTOR _getPointDistance,
		OBJMARKER & _marker, 
		const typename OBJPOINTDISTFUNCTOR::QueryType  & _p_obj,
		const typename SPATIAL_INDEX::ScalarType & _maxDist,
		typename SPATIAL_INDEX::ScalarType & _minDist,
		typename SPATIAL_INDEX:: CoordType &_closestPt)
	{
		typedef typename SPATIAL_INDEX::ObjPtr ObjPtr;
		typedef SPATIAL_INDEX SpatialIndex;
		typedef typename SPATIAL_INDEX::CoordType CoordType;
		typedef typename SPATIAL_INDEX::ScalarType ScalarType;
		typedef typename SPATIAL_INDEX::Box3x Box3x;
		 
		Point3<ScalarType> _p = OBJPOINTDISTFUNCTOR::Pos(_p_obj);
		// Initialize min_dist with _maxDist to exploit early rejection test.
		_minDist = _maxDist;

		ObjPtr winner=NULL;
		_marker.UnMarkAll();
		ScalarType newradius = Si.voxel.Norm();
		ScalarType radius;
		Box3i iboxdone,iboxtodo;
		CoordType t_res;
		typename SPATIAL_INDEX::CellIterator first,last,l;
		if(Si.bbox.IsInEx(_p))
		{
			Point3i _ip;
			Si.PToIP(_p,_ip); 
			Si.Grid( _ip[0],_ip[1],_ip[2], first, last );
			for(l=first;l!=last;++l)
			{
				ObjPtr elem=&(**l);
				if (!elem->IsD())
				{
					if (_getPointDistance((**l), _p_obj,_minDist, t_res))  // <-- NEW: use of distance functor
					{
						winner=elem;
						_closestPt=t_res;
						newradius=_minDist; // 
					}
					_marker.Mark(elem);
				}
			}
			iboxdone=Box3i(_ip,_ip);
		}

		int ix,iy,iz;
		Box3i ibox(Point3i(0,0,0),Si.siz-Point3i(1,1,1));
		do
		{
			radius=newradius;
			Box3x boxtodo=Box3x(_p,radius);
			//boxtodo.Intersect(Si.bbox);
			Si.BoxToIBox(boxtodo, iboxtodo);
			iboxtodo.Intersect(ibox);
			if(!boxtodo.IsNull())
			{
				for (ix=iboxtodo.min[0]; ix<=iboxtodo.max[0]; ix++) 
					for (iy=iboxtodo.min[1]; iy<=iboxtodo.max[1]; iy++) 
						for (iz=iboxtodo.min[2]; iz<=iboxtodo.max[2]; iz++) 
							if(ix<iboxdone.min[0] || ix>iboxdone.max[0] ||  // this test is to avoid to re-process already analyzed cells.
								iy<iboxdone.min[1] || iy>iboxdone.max[1] ||
								iz<iboxdone.min[2] || iz>iboxdone.max[2] )
							{
								Si.Grid( ix, iy, iz, first, last );
								for(l=first;l!=last;++l) if (!(**l).IsD())							     
								{
									ObjPtr elem=&(**l);
									if (!elem->IsD())
									{
										if( ! _marker.IsMarked(elem))
										{
											if (_getPointDistance((**l), _p_obj, _minDist, t_res)) 
											{
												winner=elem;
												_closestPt=t_res;
											};
											_marker.Mark(elem);
										}
									}
								}
							}
			}
			if(!winner) newradius=radius+Si.voxel.Norm();
			else newradius = _minDist;
			iboxdone=iboxtodo;
		}
		while (_minDist>radius);

		return winner;
	}

	template <class SPATIALINDEXING,class OBJPOINTDISTFUNCTOR, class OBJMARKER, 
	class OBJPTRCONTAINER, class DISTCONTAINER, class POINTCONTAINER>
		unsigned int GridGetKClosest(SPATIALINDEXING &_Si,
									 OBJPOINTDISTFUNCTOR & _getPointDistance,
									 OBJMARKER & _marker,
									 const unsigned int _k,
									 const typename SPATIALINDEXING::CoordType & _p, 
									 const typename SPATIALINDEXING::ScalarType & _maxDist,
									 OBJPTRCONTAINER & _objectPtrs,
									 DISTCONTAINER & _distances,
									 POINTCONTAINER & _points)
	{
		typedef vcg::ClosestIterator<SPATIALINDEXING,OBJPOINTDISTFUNCTOR,OBJMARKER> ClosestIteratorType;
		ClosestIteratorType	Cli=ClosestIteratorType(_Si,_getPointDistance);
		Cli.SetMarker(_marker);
		Cli.Init(_p,_maxDist);
		unsigned int i=0;
		_objectPtrs.clear();
		_distances.clear();
		_points.clear();
		while ((!Cli.End())&&(i<_k))
		{
			_objectPtrs.push_back(&(*Cli));
			_distances.push_back(Cli.Dist());
			_points.push_back(Cli.NearestPoint());
			++Cli;
			i++;
		}
		return (i);
	}

	template <class SPATIALINDEXING,class OBJRAYISECTFUNCTOR, class OBJMARKER>
		typename SPATIALINDEXING::ObjPtr GridDoRay(SPATIALINDEXING &_Si,
												   OBJRAYISECTFUNCTOR &_rayIntersector, 
												   OBJMARKER &_marker, 
												   const Ray3<typename SPATIALINDEXING::ScalarType> & _ray, 
												   const typename SPATIALINDEXING::ScalarType & _maxDist,
												   typename SPATIALINDEXING::ScalarType & _t) 
	{
		typedef vcg::RayIterator<SPATIALINDEXING,OBJRAYISECTFUNCTOR,OBJMARKER> RayIteratorType;
		RayIteratorType RayIte=RayIteratorType(_Si,_rayIntersector,_maxDist);
		RayIte.SetMarker(_marker);
		RayIte.Init(_ray);

		if (!RayIte.End())
		{
			_t=RayIte.Dist();
			return(&(*RayIte));
		}
		return 0;
	}

	
	template <class SPATIALINDEXING, class OBJPOINTDISTFUNCTOR, class OBJMARKER, class OBJPTRCONTAINER, class DISTCONTAINER, class POINTCONTAINER>
		unsigned int GridGetInSphere(SPATIALINDEXING &_Si,
									OBJPOINTDISTFUNCTOR & _getPointDistance, 
									OBJMARKER & _marker,
									const typename SPATIALINDEXING::CoordType & _p,
									const typename SPATIALINDEXING::ScalarType & _r,
									OBJPTRCONTAINER & _objectPtrs,
									DISTCONTAINER & _distances, 
									POINTCONTAINER & _points) 
	{
			typedef vcg::ClosestIterator<SPATIALINDEXING,OBJPOINTDISTFUNCTOR,OBJMARKER> ClosestIteratorType;
			ClosestIteratorType	Cli=ClosestIteratorType(_Si,_getPointDistance);
			Cli.SetMarker(_marker);
			Cli.Init(_p,_r);
			_objectPtrs.clear();
			_distances.clear();
			_points.clear();
			while (!Cli.End())
			{
				_objectPtrs.push_back(&(*Cli));
				_distances.push_back(Cli.Dist());
				_points.push_back(Cli.NearestPoint());
				++Cli;
			}
			return ((int)_objectPtrs.size());
		}

		template <class SPATIALINDEXING,class OBJMARKER, class OBJPTRCONTAINER>
			unsigned int GridGetInBox(SPATIALINDEXING &_Si,
			OBJMARKER & _marker, 
			const vcg::Box3<typename SPATIALINDEXING::ScalarType> &_bbox,
			OBJPTRCONTAINER & _objectPtrs) 
		{
			typename SPATIALINDEXING::CellIterator first,last,l;
			_objectPtrs.clear();
			vcg::Box3i ibbox;
			Box3i Si_ibox(Point3i(0,0,0),_Si.siz-Point3i(1,1,1));
			_Si.BoxToIBox(_bbox, ibbox);
			ibbox.Intersect(Si_ibox);
			_marker.UnMarkAll();
			if (ibbox.IsNull())
				return 0;
			else
			{
				int ix,iy,iz;
				for (ix=ibbox.min[0]; ix<=ibbox.max[0]; ix++) 
					for (iy=ibbox.min[1]; iy<=ibbox.max[1]; iy++) 
						for (iz=ibbox.min[2]; iz<=ibbox.max[2]; iz++) 
						{
							_Si.Grid( ix, iy, iz, first, last );
							for(l=first;l!=last;++l) 
								if (!(**l).IsD())
								{
									typename SPATIALINDEXING::ObjPtr elem=&(**l);
									vcg::Box3<typename SPATIALINDEXING::ScalarType> box_elem;
									elem->GetBBox(box_elem);
									if(( ! _marker.IsMarked(elem))&&(box_elem.Collide(_bbox))){
										_objectPtrs.push_back(elem);
										_marker.Mark(elem);
									}
								}
						}
						return (static_cast<unsigned int>(_objectPtrs.size()));
			}
		}

	}//end namespace vcg
#endif

