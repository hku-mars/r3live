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
Revision 1.3  2005/10/05 01:40:56  m_di_benedetto
Node children are now tested in ascending ray-T order.

Revision 1.2  2005/09/28 19:48:31  m_di_benedetto
Removed hit point parameter, #included aabbtree base.

Revision 1.1  2005/09/26 18:33:16  m_di_benedetto
First Commit.


****************************************************************************/

#ifndef __VCGLIB_AABBBINARYTREE_RAY_H
#define __VCGLIB_AABBBINARYTREE_RAY_H

// vcg headers
#include <vcg/space/ray3.h>
#include <vcg/space/index/aabb_binary_tree/base.h>

/***************************************************************************/

namespace vcg {

template <class TREETYPE>
class AABBBinaryTreeRay {
public:
	typedef AABBBinaryTreeRay<TREETYPE> ClassType;
	typedef TREETYPE TreeType;
	typedef typename TreeType::ScalarType ScalarType;
	typedef typename TreeType::CoordType CoordType;
	typedef typename TreeType::NodeType NodeType;
	typedef typename TreeType::ObjPtr ObjPtr;

	template <class OBJRAYISECTFUNCT>
	static inline ObjPtr Ray(TreeType & tree, OBJRAYISECTFUNCT & rayIntersection, const Ray3<ScalarType> & ray, const ScalarType & maxDist, ScalarType & t) {
		typedef std::vector<NodeType *> NodePtrVector;
		typedef typename NodePtrVector::const_iterator NodePtrVector_ci;

		NodeType * pRoot = tree.pRoot;

		if (pRoot == 0) {
			return (0);
		}

		const CoordType & rayDirection = ray.Direction();

		t = maxDist / rayDirection.Norm();

		Ray3Ex rayex;
		rayex.r = ray;
		rayex.invDirection[0] = ((ScalarType)1) / rayDirection[0];
		rayex.invDirection[1] = ((ScalarType)1) / rayDirection[1];
		rayex.invDirection[2] = ((ScalarType)1) / rayDirection[2];
		rayex.sign[0] = (rayex.invDirection[0] < ((ScalarType)0)) ? (1) : (0);
		rayex.sign[1] = (rayex.invDirection[1] < ((ScalarType)0)) ? (1) : (0);
		rayex.sign[2] = (rayex.invDirection[2] < ((ScalarType)0)) ? (1) : (0);

		ObjPtr closestObj = 0;
		ClassType::DepthFirstRayIsect(pRoot, rayIntersection, rayex, t, closestObj);

		return (closestObj);
	}

protected:
	class Ray3Ex {
	public:
		Ray3<ScalarType> r;
		CoordType invDirection;
		unsigned char sign[3];
	};

	static inline bool IntersectionBoxRay(const CoordType & boxCenter, const CoordType & boxHalfDims, const Ray3Ex & ray, ScalarType & t0) {
		const CoordType bounds[2] = {
			boxCenter - boxHalfDims,
			boxCenter + boxHalfDims
		};
		ScalarType tmin, tmax;
		ScalarType tcmin, tcmax;
		const CoordType & origin = ray.r.Origin();

		tmin = (bounds[ray.sign[0]][0] - origin[0]) * ray.invDirection[0];
		tmax = (bounds[1 - ray.sign[0]][0] - origin[0]) * ray.invDirection[0];

		tcmin = (bounds[ray.sign[1]][1] - origin[1]) * ray.invDirection[1];
		tcmax = (bounds[1 - ray.sign[1]][1] - origin[1]) * ray.invDirection[1];

		if ((tmin > tcmax) || (tcmin > tmax)) { return (false);	}

		if (tcmin > tmin) { tmin = tcmin; }
		if (tcmax < tmax) { tmax = tcmax; }
		tcmin = (bounds[ray.sign[2]][2] - origin[2]) * ray.invDirection[2];
		tcmax = (bounds[1-ray.sign[2]][2] - origin[2]) * ray.invDirection[2];

		if ((tmin > tcmax) || (tcmin > tmax)) { return (false); }

		if (tcmin > tmin) { tmin = tcmin; }
		//if (tcmax < tmax) { tmax = tcmax; }
		t0 = (tmin >= ((ScalarType)0)) ? (tmin) :((ScalarType)0);

		return (true);
	}

	template <class OBJRAYISECTFUNCT>
	static inline void DepthFirstRayIsect(const NodeType * node, OBJRAYISECTFUNCT & rayIntersection, const Ray3Ex & ray, ScalarType & rayT, ObjPtr & closestObj) {
		if (node == 0) {
			return;
		}

		ScalarType rt;
		if (!ClassType::IntersectionBoxRay(node->boxCenter, node->boxHalfDims, ray, rt)) {
			return;
		}

		if (rt >= rayT) {
			return;
		}

		if (node->IsLeaf()) {
			for (typename TreeType::ObjPtrVectorConstIterator si=node->oBegin; si!=node->oEnd; ++si) {
				if (rayIntersection(*(*si), ray.r, rt)) {
					if (rt < rayT) {
						rayT = rt;
						closestObj = (*si);
					}
				}
			}
			return;
		}

		ClassType::DepthFirstRayIsect(node->children[0], rayIntersection, ray, rayT, closestObj);
		ClassType::DepthFirstRayIsect(node->children[1], rayIntersection, ray, rayT, closestObj);
	}

};

}	// end namespace vcg

#endif // #ifndef __VCGLIB_AABBBINARYTREE_RAY_H
