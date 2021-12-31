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
Revision 1.1  2005/09/26 18:33:16  m_di_benedetto
First Commit.


****************************************************************************/

#ifndef __VCGLIB_AABBBINARYTREE_KCLOSEST_H
#define __VCGLIB_AABBBINARYTREE_KCLOSEST_H

// stl headers
#include <queue>
#include <deque>

// vcg headers
#include <vcg/space/index/aabb_binary_tree/base.h>
#include <wrap/utils.h>

/***************************************************************************/

namespace vcg {

template <class TREETYPE>
class AABBBinaryTreeKClosest {
public:
	typedef AABBBinaryTreeKClosest<TREETYPE> ClassType;
	typedef TREETYPE TreeType;
	typedef typename TreeType::ScalarType ScalarType;
	typedef typename TreeType::CoordType CoordType;
	typedef typename TreeType::NodeType NodeType;
	typedef typename TreeType::ObjPtr ObjPtr;

protected:
		class ClosestObjType {
		public:
			ObjPtr pObj;
			ScalarType minDist;
			CoordType closestPt;
		};

		class CompareClosest {
		public:
			bool operator () (const ClosestObjType & a, const ClosestObjType & b) {
				return (a.minDist < b.minDist);
			}
		};

		typedef std::priority_queue<ClosestObjType, std::deque<ClosestObjType>, CompareClosest> PQueueType;

public:
	template <class OBJPOINTDISTANCEFUNCT, class OBJPTRCONTAINERTYPE, class DISTCONTAINERTYPE, class POINTCONTAINERTYPE>
	static inline unsigned int KClosest(TreeType & tree, OBJPOINTDISTANCEFUNCT & getPointDistance, const unsigned int k, const CoordType & p, const ScalarType & maxDist, OBJPTRCONTAINERTYPE & objects, DISTCONTAINERTYPE & distances, POINTCONTAINERTYPE & points) {
		typedef std::vector<NodeType *> NodePtrVector;
		typedef typename NodePtrVector::const_iterator NodePtrVector_ci;

		if (k == 0) {
			return (0);
		}

		NodeType * pRoot = tree.pRoot;

		if (pRoot == 0) {
			return (0);
		}

		PQueueType pq;
		ScalarType mindmax = maxDist;

		ClassType::DepthFirstCollect(pRoot, getPointDistance, k, p, mindmax, pq);

		const unsigned int sz = (unsigned int)(pq.size());

		while (!pq.empty()) {
			ClosestObjType cobj = pq.top();
			pq.pop();
			objects.push_back(cobj.pObj);
			distances.push_back(cobj.minDist);
			points.push_back(cobj.closestPt);
		}

		return (sz);
	}

protected:
	template <class OBJPOINTDISTANCEFUNCT>
	static void DepthFirstCollect(NodeType * node, OBJPOINTDISTANCEFUNCT & getPointDistance, const unsigned int k, const CoordType & p, ScalarType & mindmax, PQueueType & pq) {
		const CoordType dc = Abs(p - node->boxCenter);

		if (pq.size() >= k) {
			const ScalarType dmin = LowClampToZero(dc - node->boxHalfDims).SquaredNorm();
			if (dmin >= mindmax) {
				return;
			}
		}

		if (node->IsLeaf()) {
			bool someInserted = true;
			for (typename TreeType::ObjPtrVectorConstIterator si=node->oBegin; si!=node->oEnd; ++si) {
				ScalarType minDst = (pq.size() >= k) ? (pq.top().minDist) : (mindmax);
				ClosestObjType cobj;
				if (getPointDistance(*(*si), p, minDst, cobj.closestPt)) {
					someInserted = true;
					cobj.pObj = (*si);
					cobj.minDist = minDst;
					if (pq.size() >= k) {
						pq.pop();
					}
					pq.push(cobj);
				}
			}
			if (someInserted) {
				if (pq.size() >= k) {
					const ScalarType dmax = pq.top().minDist;
					const ScalarType sqdmax = dmax * dmax;
					if (sqdmax < mindmax) {
						mindmax = sqdmax;
					}
				}
			}
		}
		else {
			if (node->children[0] != 0) {
				ClassType::DepthFirstCollect(node->children[0], getPointDistance, k, p, mindmax, pq);
			}
			if (node->children[1] != 0) {
				ClassType::DepthFirstCollect(node->children[1], getPointDistance, k, p, mindmax, pq);
			}
		}
	}

};

} // end namespace vcg

#endif // #ifndef __VCGLIB_AABBBINARYTREE_KCLOSEST_H
