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

/****************************************************************************
History

$Log: not supported by cvs2svn $
Revision 1.1  2005/09/28 17:19:28  m_di_benedetto
First Commit.


****************************************************************************/

#ifndef __VCGLIB_SPATIALINDEX_H
#define __VCGLIB_SPATIALINDEX_H

// standard headers
#include <assert.h>

// vcg headers
#include <vcg/space/point3.h>
#include <vcg/space/ray3.h>
#include <vcg/space/box3.h>

namespace vcg {

/****************************************************************************
Class SpatialIndex

Description:
	This class exposes the base interface for all spatial indexing data
	structures, i.e. grids, bounding volume trees.

Template Parameters:
	OBJTYPE:      Type of the indexed objects.
	SCALARTYPE:   Scalars type for structure's internal data (may differ from
	              object's scalar type).

****************************************************************************/

template <class OBJTYPE, class SCALARTYPE>
class SpatialIndex {
public:
	/**************************************************************************
	Commonly used typedefs.
	**************************************************************************/
	typedef SpatialIndex<OBJTYPE, SCALARTYPE> ClassType;
	typedef OBJTYPE ObjType;
	typedef SCALARTYPE ScalarType;
	typedef ObjType * ObjPtr;
	typedef Point3<ScalarType> CoordType;
	typedef vcg::Box3<ScalarType> BoxType;

	/**************************************************************************
	Method Set.

	Description:
		The Set method initializes the spatial structure.

	Template Parameters:
		OBJITER:  Objects Container's iterator type.

	Method Parameters:
		_oBegin : [IN] begin objects container's iterator
		_oEnd   : [IN] end objects container's iterator

	Return Value:
		None.

	**************************************************************************/
	template <class OBJITER>
	void Set(const OBJITER & _oBegin, const OBJITER & _oEnd) {
		assert(0);      // this is a base interface.
		(void)_oBegin;  // avoid "unreferenced parameter" compiler warning.
		(void)_oEnd;
	}

  /**************************************************************************
  Method Empty.
  Description:
    check if the spatial structure is empty.

  Return Value:
    true if it is empty.
  **************************************************************************/

  bool Empty() {
    assert(0);      // this is a base interface.
    return true;
  }

	/**************************************************************************
	Method GetClosest.

	Description:
		The GetClosest method finds the closest object given a point.
		It also finds the closest point and minimum distance.

	Template Parameters:
		OBJPOINTDISTFUNCTOR : Object-Point distance functor type;
		                      this type must implement an operator () with signature
													  bool operator () (const ObjType & obj, const CoordType & p, ScalarType & d, CoordType & q)
													 where:
													   obj [IN] is a reference to the current object being tested,
													   p [IN] is the query point,
													   d [IN/OUT] is in input the reject distance and in output the closest distance,
													   q [OUT] is the closest point.
													 The operator returns true if the closest distance is less than input reject distance.
		OBJMARKER           : The type of a marker functor.

	Method Parameters:
		_getPointDistance : [IN] Functor for point-distance calculation.
		_marker           : [IN] Functor for marking objects already tested.
		_p                : [IN] The query point.
		_maxDist          : [IN] Maximum reject distance.
		_minDist          : [OUT] Closest distance.
		_closestPt        : [OUT] Closest point.

	Return Value:
		A pointer to the closest object (if any).

	**************************************************************************/
	template <class OBJPOINTDISTFUNCTOR, class OBJMARKER>
	ObjPtr GetClosest(
		OBJPOINTDISTFUNCTOR & _getPointDistance, OBJMARKER & _marker, const CoordType & _p, const ScalarType & _maxDist,
		ScalarType & _minDist, CoordType & _closestPt) {
		assert(0);
		(void)_getPointDistance;
		(void)_marker;
		(void)_p;
		(void)_maxDist;
		(void)_minDist;
		(void)_closestPt;
		return ((ObjPtr)0);
	}

	/**************************************************************************
	Method GetKClosest.

	Description:
		The GetKClosest method finds the K closest object given a point.
		It also finds the closest points and minimum distances.

	Template Parameters:
		OBJPOINTDISTFUNCTOR : Object-Point distance functor type;
		                      this type must implement an operator () with signature
													  bool operator () (const ObjType & obj, const CoordType & p, ScalarType & d, CoordType & q)
													 where:
													   obj [IN] is a reference to the current object being tested,
													   p [IN] is the query point,
													   d [IN/OUT] is in input the reject distance and in output the closest distance,
													   q [OUT] is the closest point.
													 The operator returns true if the closest distance is less than input reject distance.
		OBJMARKER           : The type of a marker functor.
		OBJPTRCONTAINER     : The type of a object pointers container.
		DISTCONTAINER       : The type of a container which, in return, will contain the closest distances.
		POINTCONTAINER      : The type of a container which, in return, will contain the closest points.

	Method Parameters:
		_getPointDistance : [IN] Functor for point-distance calculation.
		_marker           : [IN] Functor for marking objects already tested.
		_k                : [IN] The number of closest objects to search for.
		_p                : [IN] The query point.
		_maxDist          : [IN] Maximum reject distance.
		_objectPtrs       : [OUT] Container which, in return, will contain pointers to the closest objects.
		_distances        : [OUT] Container which, in return, will contain the closest distances.
		_objectPtrs       : [OUT] Container which, in return, will contain the closest points.

	Return Value:
		The number of closest objects found.

	**************************************************************************/
	template <class OBJPOINTDISTFUNCTOR, class OBJMARKER, class OBJPTRCONTAINER, class DISTCONTAINER, class POINTCONTAINER>
	unsigned int GetKClosest(
		OBJPOINTDISTFUNCTOR & _getPointDistance, OBJMARKER & _marker, const unsigned int _k, const CoordType & _p, const ScalarType & _maxDist,
		OBJPTRCONTAINER & _objectPtrs, DISTCONTAINER & _distances, POINTCONTAINER & _points) {
		assert(0);
		(void)_getPointDistance;
		(void)_marker;
		(void)_k;
		(void)_p;
		(void)_maxDist;
		(void)_objectPtrs;
		(void)_distances;
		(void)_points;
		return (0);
	}
	
	
	/**************************************************************************
	Method GetInSphere.

	Description:
		The GetInSphere method finds all the objects in the specified sphere

	Template Parameters:
		OBJPOINTDISTFUNCTOR : Object-Point distance functor type;
		                      this type must implement an operator () with signature
													  bool operator () (const ObjType & obj, const CoordType & p, ScalarType & d, CoordType & q)
													 where:
													   obj [IN] is a reference to the current object being tested,
													   p [IN] is the query point,
													   d [IN/OUT] is in input the reject distance and in output the closest distance,
													   q [OUT] is the closest point.
													 The operator returns true if the closest distance is less than input reject distance.
		OBJMARKER           : The type of a marker functor.
		OBJPTRCONTAINER     : The type of a object pointers container.
		DISTCONTAINER       : The type of a container which, in return, will contain the closest distances.
		POINTCONTAINER      : The type of a container which, in return, will contain the closest points.

	Method Parameters:
		_getPointDistance : [IN] Functor for point-distance calculation.
		_marker           : [IN] Functor for marking objects already tested.
		_p                : [IN] The query point.
		_r		          : [IN]  The radius of the specified sphere.
		_objectPtrs       : [OUT] Container which, in return, will contain pointers to the in-sphere objects.
		_distances        : [OUT] Container which, in return, will contain the in-sphere distances.
		_objectPtrs       : [OUT] Container which, in return, will contain the in-sphere nearests points for each object.

	Return Value:
		The number of in-sphere objects found.

	**************************************************************************/
	template <class OBJPOINTDISTFUNCTOR, class OBJMARKER, class OBJPTRCONTAINER, class DISTCONTAINER, class POINTCONTAINER>
	unsigned int GetInSphere(
		OBJPOINTDISTFUNCTOR & _getPointDistance, OBJMARKER & _marker,const CoordType & _p, const ScalarType & _r,OBJPTRCONTAINER & _objectPtrs, DISTCONTAINER & _distances, POINTCONTAINER & _points) {
		assert(0);
		(void)_getPointDistance;
		(void)_marker;
		(void)_p;
		(void)_r;
		(void)_objectPtrs;
		(void)_distances;
		(void)_points;
		return (0);
	}

	/**************************************************************************
	Method GetInBox.

	Description:
		The GetInBox returns all the object in the specified bbox

	Template Parameters:

		OBJMARKER           : The type of a marker functor.
		OBJPTRCONTAINER     : The type of a object pointers container.
	
	Method Parameters:
		_marker           : [IN] Functor for marking objects already tested.
		_bbox             : [IN] The bounding box of spatial query.
		_objectPtrs       : [OUT] Container which, in return, will contain pointers to the closest objects.
		

	Return Value:
		The number of in-box objects found.

	**************************************************************************/
	template <class OBJMARKER, class OBJPTRCONTAINER>
	unsigned int GetInBox(OBJMARKER & _marker, const BoxType _bbox,OBJPTRCONTAINER & _objectPtrs) {
		assert(0);
		(void)_marker;
		(void)_bbox;
		(void)_objectPtrs;
		return (0);
	}
	

	
	/**************************************************************************
	Method DoRay.

	Description:
		The DoRay method finds the first object in the structure hit by a ray.

	Template Parameters:
		OBJRAYISECTFUNCTOR : Object-Ray intersection functor type;
		                      this type must implement an operator () with signature
													  bool operator () (const ObjType & obj, const Ray3<scalarType> ray, ScalarType & t)
													 where:
													   obj [IN] is a reference to the current object being tested,
													   ray [IN] is the query ray,
													   t [OUT] is the parameter of the ray equation at which intersection occurs.
													 The operator returns true if the the object has been hit by the ray (i.e. they intersect).
		OBJMARKER          : The type of a marker functor.

	Method Parameters:
		_rayIntersector : [IN] Functor for object-ray intersection.
		_marker         : [IN] Functor for marking objects already tested.
		_ray            : [IN] The query ray.
		_maxDist        : [IN] Maximum reject distance.
		_t              : [OUT] the parameter of the ray equation at which intersection occurs.

	Return Value:
		A pointer to the first object hit by the ray (if any).

	**************************************************************************/
	template <class OBJRAYISECTFUNCTOR, class OBJMARKER>
	ObjPtr DoRay(OBJRAYISECTFUNCTOR & _rayIntersector, OBJMARKER & _marker, const Ray3<ScalarType> & _ray, const ScalarType & _maxDist, ScalarType & _t) {
		assert(0);
		(void)_rayIntersector;
		(void)_marker;
		(void)_ray;
		(void)_maxDist;
		(void)_t;
		return ((ObjPtr)0);
	}

};

} // end namespace vcg

#endif // #ifndef __VCGLIB_SPATIALINDEX_H
