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
Revision 1.8  2005/11/30 09:57:13  m_di_benedetto
Added methods to flag visibility.

Revision 1.7  2005/10/26 11:42:03  m_di_benedetto
Added PASS_THROUGH flags.

Revision 1.6  2005/10/15 19:14:35  m_di_benedetto
Modified objapplyfunctor to nodeapplyfunctor.

Revision 1.5  2005/10/05 01:59:56  m_di_benedetto
First Commit, new version.

Revision 1.3  2005/09/29 22:20:49  m_di_benedetto
Removed '&' in FrustumCull() method.

Revision 1.2  2005/09/28 19:57:18  m_di_benedetto
#included aabb tree base.

Revision 1.1  2005/09/26 18:33:16  m_di_benedetto
First Commit.


****************************************************************************/

#ifndef __VCGLIB_AABBBINARYTREE_FRUSTUMCULL_H
#define __VCGLIB_AABBBINARYTREE_FRUSTUMCULL_H

// std headers
/* EMPTY */

// vcg headers
#include <vcg/space/point3.h>
#include <vcg/space/plane3.h>
#include <vcg/space/index/aabb_binary_tree/base.h>

/***************************************************************************/

namespace vcg {

template <class TREETYPE>
class AABBBinaryTreeFrustumCull {
public:
	typedef AABBBinaryTreeFrustumCull<TREETYPE> ClassType;
	typedef TREETYPE TreeType;
	typedef typename TreeType::ScalarType ScalarType;
	typedef typename TreeType::CoordType CoordType;
	typedef typename TreeType::NodeType NodeType;
	typedef typename TreeType::ObjPtr ObjPtr;

protected:
	class VFrustumPlane {
	public:
		CoordType normal;
		ScalarType offset;
		unsigned int pVertexIndex[3];
	};

	class VFrustum {
	public:
		VFrustumPlane planes[6];
	};

public:
	enum {
		FC_FIRST_PLANE_BIT					= 0,
		FC_PARTIALLY_VISIBLE_BIT		= (1 << (FC_FIRST_PLANE_BIT + 3)),
		FC_FULLY_VISIBLE_BIT				= (1 << (FC_FIRST_PLANE_BIT + 4)),
		FC_PASS_THROUGH_FIRST_BIT		= (FC_FIRST_PLANE_BIT + 5)
	};

	static inline bool IsPartiallyVisible(const NodeType * node) {
		return ((node->Flags() & FC_PARTIALLY_VISIBLE_BIT) != 0);
	}

	static inline bool IsFullyVisible(const NodeType * node) {
		return ((node->Flags() & FC_FULLY_VISIBLE_BIT) != 0);
	}

	static inline void SetFullyVisible(NodeType * node) {
		node->Flags() |= FC_FULLY_VISIBLE_BIT;
	}

	static inline void SetInvisible(NodeType * node) {
		node->Flags() &= ~(FC_FULLY_VISIBLE_BIT | FC_PARTIALLY_VISIBLE_BIT);
	}

	static inline bool IsVisible(const NodeType * node) {
		return ((node->Flags() & (FC_PARTIALLY_VISIBLE_BIT | FC_FULLY_VISIBLE_BIT)) != 0);
	}

	static inline unsigned int PassThrough(const NodeType * node) {
		return ((node->Flags() >> FC_PASS_THROUGH_FIRST_BIT) & 0x03);
	}

	static inline void Initialize(TreeType & tree) {
		NodeType * pRoot = tree.pRoot;
		if (pRoot == 0) {
			return;
		}
		ClassType::InitializeNodeFlagsRec(pRoot);
	}

	template <class NODEAPPLYFUNCTOR>
	static inline void FrustumCull(TreeType & tree, const Point3<ScalarType> & viewerPosition, const Plane3<ScalarType> frustumPlanes[6], const unsigned int minNodeObjectsCount, NODEAPPLYFUNCTOR & nodeApply) {
		NodeType * pRoot = tree.pRoot;
		if (pRoot == 0) {
			return;
		}

		VFrustum frustum;
		for (int i=0; i<6; ++i) {
			frustum.planes[i].normal = frustumPlanes[i].Direction();
			frustum.planes[i].offset = frustumPlanes[i].Offset();
			frustum.planes[i].pVertexIndex[0] = (frustum.planes[i].normal[0] >= ((ScalarType)0)) ? (1) : (0);
			frustum.planes[i].pVertexIndex[1] = (frustum.planes[i].normal[1] >= ((ScalarType)0)) ? (1) : (0);
			frustum.planes[i].pVertexIndex[2] = (frustum.planes[i].normal[2] >= ((ScalarType)0)) ? (1) : (0);
		}

		const unsigned char inMask = 0x3F;
		ClassType::NodeVsFrustum(tree.pRoot, viewerPosition, frustum, inMask, minNodeObjectsCount, nodeApply);
	}

protected:

	static inline void InitializeNodeFlagsRec(NodeType * node) {
		//node->Flags() &= ~(0x7F);
		node->Flags() = 0;
		if (node->children[0] != 0) {
			ClassType::InitializeNodeFlagsRec(node->children[0]);
		}
		if (node->children[1] != 0) {
			ClassType::InitializeNodeFlagsRec(node->children[1]);
		}
	}

	template <class NODEAPPLYFUNCTOR>
	static inline void NodeVsFrustum(NodeType * node, const Point3<ScalarType> & viewerPosition, const VFrustum & f, unsigned char inMask, unsigned int minNodeObjectsCount, NODEAPPLYFUNCTOR & nodeApply) {
		if (node == 0) {
			return;
		}

		const CoordType bminmax[2] = {
			node->boxCenter - node->boxHalfDims,
			node->boxCenter + node->boxHalfDims,
		};
		const unsigned int firstFail = (unsigned int)((node->Flags() >> FC_FIRST_PLANE_BIT) & 0x7);
		const VFrustumPlane * fp = f.planes + firstFail;
		unsigned char k = 1 << firstFail;
		unsigned char newMask = 0x0;
		bool fullInside = true;

		node->Flags() &= ~(1 << 25);
		node->Flags() &= ~(FC_PARTIALLY_VISIBLE_BIT | FC_FULLY_VISIBLE_BIT | (0x03 << FC_PASS_THROUGH_FIRST_BIT));

		if ((k & inMask) != 0) {
			if (
				((fp->normal[0] * bminmax[fp->pVertexIndex[0]][0]) +
				(fp->normal[1] * bminmax[fp->pVertexIndex[1]][1]) +
				(fp->normal[2] * bminmax[fp->pVertexIndex[2]][2]) -
				(fp->offset)) < ((ScalarType)0)
			) {
				return;
			}

			if (
				((fp->normal[0] * bminmax[1 - fp->pVertexIndex[0]][0]) +
				(fp->normal[1] * bminmax[1 - fp->pVertexIndex[1]][1]) +
				(fp->normal[2] * bminmax[1 - fp->pVertexIndex[2]][2]) -
				(fp->offset)) < ((ScalarType)0)
			) {
				newMask |=  k;
				fullInside = false;
			}
		}

		k = 1;
		for (unsigned int i=0; k<=inMask; ++i, k<<=1) {
			if ((i != firstFail) && ((k & inMask) != 0)) {
				fp = f.planes + i;
				if (
					((fp->normal[0] * bminmax[fp->pVertexIndex[0]][0]) +
					(fp->normal[1] * bminmax[fp->pVertexIndex[1]][1]) +
					(fp->normal[2] * bminmax[fp->pVertexIndex[2]][2]) -
					(fp->offset)) < ((ScalarType)0)
				) {
					node->Flags() = (node->Flags() & ((~0x0) & (0x7 << ClassType::FC_FIRST_PLANE_BIT))) | (i << ClassType::FC_FIRST_PLANE_BIT);
					return;
				}

				if (
					((fp->normal[0] * bminmax[1 - fp->pVertexIndex[0]][0]) +
					(fp->normal[1] * bminmax[1 - fp->pVertexIndex[1]][1]) +
					(fp->normal[2] * bminmax[1 - fp->pVertexIndex[2]][2]) -
					(fp->offset)) < ((ScalarType)0)
				) {
					newMask |=  k;
					fullInside = false;
				}
			}
		}

		// Intermediate BVs containing a sufficient number of objects are marked fully visible even if they don't
		if (fullInside || (node->ObjectsCount() <= minNodeObjectsCount)) {
			node->Flags() |= FC_FULLY_VISIBLE_BIT;
			nodeApply(*node);
			return;
		}

		node->Flags() |= FC_PARTIALLY_VISIBLE_BIT;

		if ((node->IsLeaf()))
		{
			nodeApply(*node);
			return;
		}

		//ClassType::NodeVsFrustum(node->children[0], viewerPosition, f, newMask, minNodeObjectsCount, nodeApply);
		//ClassType::NodeVsFrustum(node->children[1], viewerPosition, f, newMask, minNodeObjectsCount, nodeApply);
		ScalarType dt;
		if (node->splitAxis == 0) {
			dt = viewerPosition[0] - node->boxCenter[0];
		}
		else if (node->splitAxis == 1) {
			dt = viewerPosition[1] - node->boxCenter[1];
		}
		else {
			dt = viewerPosition[2] - node->boxCenter[2];
		}

		if (dt <= (ScalarType)0) {
			ClassType::NodeVsFrustum(node->children[0], viewerPosition, f, newMask, minNodeObjectsCount, nodeApply);
			ClassType::NodeVsFrustum(node->children[1], viewerPosition, f, newMask, minNodeObjectsCount, nodeApply);
		}
		else {
			ClassType::NodeVsFrustum(node->children[1], viewerPosition, f, newMask, minNodeObjectsCount, nodeApply);
			ClassType::NodeVsFrustum(node->children[0], viewerPosition, f, newMask, minNodeObjectsCount, nodeApply);
		}

		const bool c0 = (node->children[0] != 0) && ClassType::IsVisible(node->children[0]);
		const bool c1 = (node->children[1] != 0) && ClassType::IsVisible(node->children[1]);

		if (c0 != c1) {
				node->Flags() &= ~(0x03 << FC_PASS_THROUGH_FIRST_BIT);
			if (c0) {
				node->Flags() |= (0x01 << FC_PASS_THROUGH_FIRST_BIT);
			}
			else {
				node->Flags() |= (0x02 << FC_PASS_THROUGH_FIRST_BIT);
			}
		}
	}

};

}	// end namespace vcg

#endif // #ifndef __VCGLIB_AABBBINARYTREE_FRUSTUMCULL_H
