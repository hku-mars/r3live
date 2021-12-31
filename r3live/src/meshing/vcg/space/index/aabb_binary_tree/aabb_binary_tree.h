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
Revision 1.5  2005/09/29 22:18:16  m_di_benedetto
Added frustum culling methods, renamed some parameters.

Revision 1.4  2005/09/28 21:23:03  m_di_benedetto
Added Import() to box and barycenter functors
to handle tree and objects with different ScalarType.

Revision 1.3  2005/09/28 20:10:41  m_di_benedetto
First Commit.


****************************************************************************/

#ifndef __VCGLIB_AABBBINARYTREEINDEX_H
#define __VCGLIB_AABBBINARYTREEINDEX_H

// vcg headers
#include <vcg/space/index/base.h>
#include <vcg/space/index/aabb_binary_tree/base.h>
#include <vcg/space/index/aabb_binary_tree/closest.h>
#include <vcg/space/index/aabb_binary_tree/frustum_cull.h>
#include <vcg/space/index/aabb_binary_tree/kclosest.h>
#include <vcg/space/index/aabb_binary_tree/ray.h>
#include <wrap/utils.h>

/***************************************************************************/

namespace vcg {

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATA = EmptyClass>
class AABBBinaryTreeIndex : public SpatialIndex<OBJTYPE, SCALARTYPE> {
public:
	typedef AABBBinaryTreeIndex<OBJTYPE, SCALARTYPE, NODEAUXDATA> ClassType;
	typedef OBJTYPE ObjType;
	typedef SCALARTYPE ScalarType;
	typedef NODEAUXDATA NodeAuxData;
	typedef ObjType * ObjPtr;
	typedef Point3<ScalarType> CoordType;
	typedef AABBBinaryTree<ObjType, ScalarType, NodeAuxData> TreeType;

	inline TreeType & Tree(void) {
		return (this->tree);
	}

	inline const TreeType & Tree(void) const {
		return (this->tree);
	}

	bool Empty()
	{
		return (this->tree.pRoot == 0);
	}

	void Clear(void)
	{
		this->tree.Clear();
	}

	template <class OBJITER>
	inline void Set(const OBJITER & _oBegin, const OBJITER & _oEnd) {
		GetPointerFunctor getPtr;
		GetBox3Functor getBox;
		GetBarycenter3Functor getBarycenter;
		//const unsigned int divs = 100;
		//const unsigned int size = (unsigned int)(std::distance(_oBegin, _oEnd));
		//const unsigned int maxObjectsPerLeaf = (size < divs) ? (size) : ((unsigned int)((float)(std::distance(_oBegin, _oEnd)) / (float)divs));
		const unsigned int maxObjectsPerLeaf = 10;
		const ScalarType leafBoxMaxVolume = ((ScalarType)0);
		const bool useVariance = true;

		(void)(this->tree.Set(_oBegin, _oEnd, getPtr, getBox, getBarycenter, maxObjectsPerLeaf, leafBoxMaxVolume, useVariance));
	}

	template <class OBJITERATOR, class OBJITERATORPTRFUNCT, class OBJBOXFUNCT, class OBJBARYCENTERFUNCT>
	inline bool Set(const OBJITERATOR & _oBegin, const OBJITERATOR & _oEnd, OBJITERATORPTRFUNCT & _objPtr, OBJBOXFUNCT & _objBox, OBJBARYCENTERFUNCT & _objBarycenter, const unsigned int _maxElemsPerLeaf = 1, const ScalarType & _leafBoxMaxVolume = ((ScalarType)0), const bool _useVariance = true) {
		return (this->tree.Set(_oBegin, _oEnd, _objPtr, _objBox, _objBarycenter, _maxElemsPerLeaf, _leafBoxMaxVolume, _useVariance));
	}

	template <class OBJPOINTDISTFUNCTOR, class OBJMARKER>
	inline ObjPtr GetClosest(
		OBJPOINTDISTFUNCTOR & _getPointDistance, OBJMARKER & _marker, 
		const typename OBJPOINTDISTFUNCTOR::QueryType & _p, const ScalarType & _maxDist,
		ScalarType & _minDist, CoordType & _closestPt) {
		(void)_marker;
		return (AABBBinaryTreeClosest<TreeType>::Closest(this->tree, _getPointDistance, _p, _maxDist, _minDist, _closestPt));
	}

	template <class OBJPOINTDISTFUNCTOR, class OBJMARKER, class OBJPTRCONTAINER, class DISTCONTAINER, class POINTCONTAINER>
	inline unsigned int GetKClosest(
		OBJPOINTDISTFUNCTOR & _getPointDistance, OBJMARKER & _marker, const unsigned int _k, const CoordType & _p, const ScalarType & _maxDist,
		OBJPTRCONTAINER & _objectPtrs, DISTCONTAINER & _distances, POINTCONTAINER & _points) {
		(void)_marker;
		return (AABBBinaryTreeKClosest<TreeType>::KClosest(this->tree, _getPointDistance, _k, _p, _maxDist, _objectPtrs, _distances, _points));
	}

	template <class OBJRAYISECTFUNCTOR, class OBJMARKER>
	inline ObjPtr DoRay(OBJRAYISECTFUNCTOR & _rayIntersector, OBJMARKER & _marker, const Ray3<ScalarType> & _ray, const ScalarType & _maxDist, ScalarType & _t) {
		(void)_marker;
		return (AABBBinaryTreeRay<TreeType>::Ray(this->tree, _rayIntersector, _ray, _maxDist, _t));
	}

	inline void InitializeFrustumCull(void) {
		(void)(AABBBinaryTreeFrustumCull<TreeType>::Initialize(this->tree));
	}

	inline void FrustumCull(const Plane3<ScalarType> _frustumPlanes[6], const unsigned int _minNodeObjectsCount) {
		(void)(AABBBinaryTreeFrustumCull<TreeType>::FrustumCull(this->tree, _frustumPlanes, _minNodeObjectsCount));
	}

protected:
	TreeType tree;

};

}	// end namespace vcg

#endif // #ifndef __VCGLIB_AABBBINARYTREEINDEX_H
