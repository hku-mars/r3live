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

#ifndef __VCGLIB_AABBBINARYTREE_CLOSEST_H
#define __VCGLIB_AABBBINARYTREE_CLOSEST_H

// stl headers
#include <limits>
#include <vector>

// vcg headers
#include <vcg/math/base.h>
#include <vcg/space/index/aabb_binary_tree/base.h>
#include <wrap/utils.h>

/***************************************************************************/

namespace vcg {

template <class TREETYPE>
class AABBBinaryTreeClosest {
public:
	typedef AABBBinaryTreeClosest<TREETYPE> ClassType;
	typedef TREETYPE TreeType;
	typedef typename TreeType::ScalarType ScalarType;
	typedef typename TreeType::CoordType CoordType;
	typedef typename TreeType::NodeType NodeType;
	typedef typename TreeType::ObjPtr ObjPtr;

	template <class OBJPOINTDISTANCEFUNCT>
	static inline ObjPtr Closest(TreeType & tree, OBJPOINTDISTANCEFUNCT & getPointDistance, const CoordType & p, const ScalarType & maxDist, ScalarType & minDist, CoordType & q) {
		typedef OBJPOINTDISTANCEFUNCT ObjPointDistanceFunct;
		typedef std::vector<NodeType *> NodePtrVector;
		typedef typename NodePtrVector::const_iterator NodePtrVector_ci;

		NodeType * pRoot = tree.pRoot;

		if (pRoot == 0) {
			return (0);
		}

		NodePtrVector clist1;
		NodePtrVector clist2;
		NodePtrVector leaves;

		NodePtrVector * candidates = &clist1;
		NodePtrVector * newCandidates = &clist2;

		clist1.reserve(tree.pObjects.size());
		clist2.reserve(tree.pObjects.size());
		leaves.reserve(tree.pObjects.size());

		clist1.resize(0);
		clist2.resize(0);
		leaves.resize(0);

		ScalarType minMaxDist = maxDist * maxDist;

		candidates->push_back(pRoot);

		while (!candidates->empty()) {
			newCandidates->resize(0);

			for (NodePtrVector_ci bv=candidates->begin(); bv!=candidates->end(); ++bv) {
				const CoordType dc = Abs(p - (*bv)->boxCenter);
				const ScalarType maxDist = (dc + (*bv)->boxHalfDims).SquaredNorm();
				(*bv)->ScalarValue() = LowClampToZero(dc - (*bv)->boxHalfDims).SquaredNorm();
				if (maxDist < minMaxDist) {
					minMaxDist = maxDist;
				}
			}

			for (NodePtrVector_ci ci=candidates->begin(); ci!=candidates->end(); ++ci) {
				if ((*ci)->ScalarValue() < minMaxDist) {
					if ((*ci)->IsLeaf()) {
						leaves.push_back(*ci);
					}
					else {
						if ((*ci)->children[0] != 0) {
							newCandidates->push_back((*ci)->children[0]);
						}
						if ((*ci)->children[1] != 0) {
							newCandidates->push_back((*ci)->children[1]);
						}
					}
				}
			}
			NodePtrVector * cSwap = candidates;
			candidates = newCandidates;
			newCandidates = cSwap;
		}

		clist1.clear();
		clist2.clear();

		ObjPtr closestObject = 0;
		CoordType closestPoint;
		ScalarType closestDist = math::Sqrt(minMaxDist) + std::numeric_limits<ScalarType>::epsilon();
		ScalarType closestDistSq = closestDist * closestDist;


		for (NodePtrVector_ci ci=leaves.begin(); ci!=leaves.end(); ++ci) {
			if ((*ci)->ScalarValue() < closestDistSq) {
				for (typename TreeType::ObjPtrVectorConstIterator si=(*ci)->oBegin; si!=(*ci)->oEnd; ++si) {
					if (getPointDistance(*(*si), p, closestDist, closestPoint)) {
						closestDistSq = closestDist * closestDist;
						closestObject = (*si);
            q = closestPoint;
            minDist = closestDist;
					}
				}
			}
		}

		leaves.clear();

		return (closestObject);
	}

};

} // end namespace vcg

#endif // #ifndef __VCGLIB_AABBBINARYTREE_CLOSEST_H
