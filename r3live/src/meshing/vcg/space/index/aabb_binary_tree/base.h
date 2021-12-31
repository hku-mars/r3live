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
Revision 1.2  2005/10/05 01:43:28  m_di_benedetto
Removed "parent" pointer class member in Node class.

Revision 1.1  2005/09/28 19:44:49  m_di_benedetto
First Commit.


****************************************************************************/

#ifndef __VCGLIB_AABBBINARYTREEBASE_H
#define __VCGLIB_AABBBINARYTREEBASE_H

// standard headers
#include <assert.h>

// stl headers
#include <vector>

// vcg headers
#include <vcg/space/point3.h>
#include <vcg/space/box3.h>

/***************************************************************************/

namespace vcg {

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
class AABBBinaryTree {
	public:
		typedef AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE> ClassType;
		typedef OBJTYPE ObjType;
		typedef ObjType * ObjPtr;
		typedef SCALARTYPE ScalarType;
		typedef NODEAUXDATATYPE NodeAuxDataType;
		typedef Point3<ScalarType> CoordType;

		typedef std::vector<ObjPtr> ObjPtrVector;
		typedef typename ObjPtrVector::iterator ObjPtrVectorIterator;
		typedef typename ObjPtrVector::const_iterator ObjPtrVectorConstIterator;

	public:
		class AABBBinaryTreeNode {
			public:
				CoordType boxCenter;
				CoordType boxHalfDims;
				ObjPtrVectorIterator oBegin;
				ObjPtrVectorIterator oEnd;
				AABBBinaryTreeNode * children[2];
				unsigned char splitAxis;
				NodeAuxDataType auxData;

				inline AABBBinaryTreeNode(void);
				inline ~AABBBinaryTreeNode(void);

				inline void Clear(void);
				inline bool IsLeaf(void) const;
				inline unsigned int ObjectsCount(void) const;

				inline unsigned int & Flags(void);
				inline const unsigned int & Flags(void) const;

				inline ScalarType & ScalarValue(void);
				inline const ScalarType & ScalarValue(void) const;

				inline int & IntValue(void);
				inline const int & IntValue(void) const;

				inline unsigned int & UIntValue(void);
				inline const unsigned int & UIntValue(void) const;

				inline void * & PtrValue(void);
				inline const void * & PtrValue(void) const;

			protected:
				union SharedDataUnion {
					unsigned int flags;
					int intValue;
					unsigned int uintValue;
					ScalarType scalarValue;
					void * ptrValue;
				};

				SharedDataUnion sharedData;
		};

		typedef AABBBinaryTreeNode NodeType;

		ObjPtrVector pObjects;
		NodeType * pRoot;

		inline AABBBinaryTree(void);
		inline ~AABBBinaryTree(void);

		inline void Clear(void);

		template <class OBJITERATOR, class OBJITERATORPTRFUNCT, class OBJBOXFUNCT, class OBJBARYCENTERFUNCT>
		inline bool Set(const OBJITERATOR & oBegin, const OBJITERATOR & oEnd, OBJITERATORPTRFUNCT & objPtr, OBJBOXFUNCT & objBox, OBJBARYCENTERFUNCT & objBarycenter, const unsigned int maxElemsPerLeaf = 1, const ScalarType & leafBoxMaxVolume = ((ScalarType)0), const bool useVariance = true);

	protected:
		template <class OBJBOXFUNCT, class OBJBARYCENTERFUNCT>
		inline static NodeType * BoundObjects(const ObjPtrVectorIterator & oBegin, const ObjPtrVectorIterator & oEnd, const unsigned int size, const unsigned int maxElemsPerLeaf, const ScalarType & leafBoxMaxVolume, const bool useVariance, OBJBOXFUNCT & getBox, OBJBARYCENTERFUNCT & getBarycenter);

		template <class OBJBARYCENTERFUNCT>
		inline static int BalanceMedian(const ObjPtrVectorIterator & oBegin, const ObjPtrVectorIterator & oEnd, const int size, const int splitAxis, OBJBARYCENTERFUNCT & getBarycenter, ObjPtrVectorIterator & medianIter);
};



template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::AABBBinaryTree(void) {
	this->pObjects.clear();
	this->pRoot = 0;
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::~AABBBinaryTree(void) {
	this->pObjects.clear();
	delete this->pRoot;
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
void AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::Clear(void) {
	this->pObjects.clear();
	delete this->pRoot;
	this->pRoot = 0;
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
template <class OBJITERATOR, class OBJITERATORPTRFUNCT, class OBJBOXFUNCT, class OBJBARYCENTERFUNCT>
bool AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::Set(const OBJITERATOR & oBegin, const OBJITERATOR & oEnd, OBJITERATORPTRFUNCT & objPtr, OBJBOXFUNCT & objBox, OBJBARYCENTERFUNCT & objBarycenter, const unsigned int maxElemsPerLeaf, const ScalarType & leafBoxMaxVolume, const bool useVariance) {
	this->Clear();

	if ((maxElemsPerLeaf == 0) && (leafBoxMaxVolume <= ((ScalarType)0))) {
		return (false);
	}

	const unsigned int size = (unsigned int)std::distance(oBegin, oEnd);

	this->pObjects.reserve(size);
	for (OBJITERATOR oi=oBegin; oi!=oEnd; ++oi) {
		this->pObjects.push_back(objPtr(*oi));
	}

	this->pRoot = ClassType::BoundObjects(this->pObjects.begin(), this->pObjects.end(), size, maxElemsPerLeaf, leafBoxMaxVolume, useVariance, objBox, objBarycenter);

	return (this->pRoot != 0);
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
template <class OBJBOXFUNCT, class OBJBARYCENTERFUNCT>
typename AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::NodeType * AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::BoundObjects(const ObjPtrVectorIterator & oBegin, const ObjPtrVectorIterator & oEnd, const unsigned int size, const unsigned int maxElemsPerLeaf, const ScalarType & leafBoxMaxVolume, const bool useVariance, OBJBOXFUNCT & getBox, OBJBARYCENTERFUNCT & getBarycenter) {
	if (size <= 0) {
		return (0);
	}

	NodeType * pNode = new NodeType();
	if (pNode == 0) {
		return (0);
	}

	pNode->children[0] = 0;
	pNode->children[1] = 0;

	pNode->oBegin = oBegin;
	pNode->oEnd = oEnd;

	Box3<ScalarType> bbox;
	bbox.SetNull();
	for (ObjPtrVectorConstIterator oi=pNode->oBegin; oi!=pNode->oEnd; ++oi) {
		Box3<ScalarType> tbox;
		getBox(*(*oi), tbox);
		bbox.Add(tbox);
	}

	pNode->boxCenter = bbox.Center();
	pNode->boxHalfDims = bbox.Dim() / ((ScalarType)2);

	const bool bMaxObjectsReached = (((maxElemsPerLeaf > 0) && (size <= maxElemsPerLeaf)) || (size == 1));
	const bool bMaxVolumeReached = ((leafBoxMaxVolume > ((ScalarType)0)) && (bbox.Volume() <= leafBoxMaxVolume));
	const bool isLeaf = bMaxObjectsReached || bMaxVolumeReached;

	if (isLeaf) {
		pNode->splitAxis = 0;
		return (pNode);
	}

	CoordType pSplit;

	if (useVariance) {
		CoordType mean((ScalarType)0, (ScalarType)0, (ScalarType)0);
		CoordType variance((ScalarType)0, (ScalarType)0, (ScalarType)0);
		for (ObjPtrVectorIterator oi=oBegin; oi!=oEnd; ++oi) {
			CoordType bc;
			getBarycenter(*(*oi), bc);
			mean += bc;
			variance[0] += bc[0] * bc[0];
			variance[1] += bc[1] * bc[1];
			variance[2] += bc[2] * bc[2];
		}
		variance[0] -= (mean[0] * mean[0]) / ((ScalarType)size);
		variance[1] -= (mean[1] * mean[1]) / ((ScalarType)size);
		variance[2] -= (mean[2] * mean[2]) / ((ScalarType)size);
		pSplit = variance;
	}
	else {
		pSplit = pNode->boxHalfDims;
	}

	ScalarType maxDim = pSplit[0];
	unsigned char splitAxis = 0;
	if (maxDim < pSplit[1]) {
		maxDim = pSplit[1];
		splitAxis = 1;
	}
	if (maxDim < pSplit[2]) {
		maxDim = pSplit[2];
		splitAxis = 2;
	}

	pNode->splitAxis = splitAxis;

	ObjPtrVectorIterator median;
	const int lSize = ClassType::BalanceMedian(pNode->oBegin, pNode->oEnd, size, splitAxis, getBarycenter, median);
	const int rSize = size - lSize;

	if (lSize > 0) {
		pNode->children[0] = ClassType::BoundObjects(pNode->oBegin, median, lSize, maxElemsPerLeaf, leafBoxMaxVolume, useVariance, getBox, getBarycenter);
		if (pNode->children[0] == 0) {
			delete pNode;
			return (0);
		}
	}

	if (rSize > 0) {
		pNode->children[1] = ClassType::BoundObjects(median, pNode->oEnd, rSize, maxElemsPerLeaf, leafBoxMaxVolume, useVariance, getBox, getBarycenter);
		if (pNode->children[1] == 0) {
			delete pNode;
			return (0);
		}
	}

	return (pNode);
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
template <class OBJBARYCENTERFUNCT>
int AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::BalanceMedian(const ObjPtrVectorIterator & oBegin, const ObjPtrVectorIterator & oEnd, const int size, const int splitAxis, OBJBARYCENTERFUNCT & getBarycenter, ObjPtrVectorIterator & medianIter) {
	const int iMedian = (size + 1) / 2;

	ObjPtrVectorIterator l, r, i, j;
	ObjPtr iTmp;
	ScalarType pos;
	ObjPtrVectorIterator median = oBegin + iMedian;
	CoordType bc;

	l = oBegin;
	r = oEnd - 1;

	while (l < r) {
		getBarycenter(*(*r), bc);
		pos = bc[splitAxis];

		i = l;
		j = r - 1;

		while (true) {
			getBarycenter(*(*i), bc);
			while ((bc[splitAxis] <= pos) && (i < r)) {
				i++;
				getBarycenter(*(*i), bc);
			}
			getBarycenter(*(*j), bc);
			while ((bc[splitAxis] > pos) && (j > l)) {
				j--;
				getBarycenter(*(*j), bc);
			}
			if (i >= j) {
				break;
			}
			iTmp = (*i);
			(*i) = (*j);
			(*j) = iTmp;
		}

		iTmp = (*i);
		(*i) = (*r);
		(*r) = iTmp;

		if (i >= (median)) {
			r = i - 1;
		}
		if (i <= (median)) {
			l = i + 1;
		}
	}

	medianIter = median;

	return (iMedian);
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::AABBBinaryTreeNode::AABBBinaryTreeNode(void) {
	this->children[0] = 0;
	this->children[1] = 0;
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::AABBBinaryTreeNode::~AABBBinaryTreeNode(void) {
	delete this->children[0];
	delete this->children[1];
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
void AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::AABBBinaryTreeNode::Clear(void) {
	delete this->children[0];
	this->children[0] = 0;

	delete this->children[1];
	this->children[1] = 0;
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
bool AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::AABBBinaryTreeNode::IsLeaf(void) const {
	return ((this->children[0] == 0) && (this->children[1] == 0));
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
unsigned int AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::AABBBinaryTreeNode::ObjectsCount(void) const {
	return ((unsigned int)(std::distance(this->oBegin, this->oEnd)));
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
unsigned int & AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::AABBBinaryTreeNode::Flags(void) {
	return (this->sharedData.flags);
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
const unsigned int & AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::AABBBinaryTreeNode::Flags(void) const {
	return (this->sharedData.flags);
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
int & AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::AABBBinaryTreeNode::IntValue(void) {
	return (this->sharedData.intValue);
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
const int & AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::AABBBinaryTreeNode::IntValue(void) const {
	return (this->sharedData.intValue);
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
unsigned int & AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::AABBBinaryTreeNode::UIntValue(void) {
	return (this->sharedData.uintValue);
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
const unsigned int & AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::AABBBinaryTreeNode::UIntValue(void) const {
	return (this->sharedData.uintValue);
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
typename AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::ScalarType & AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::AABBBinaryTreeNode::ScalarValue(void) {
	return (this->sharedData.scalarValue);
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
const typename AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::ScalarType & AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::AABBBinaryTreeNode::ScalarValue(void) const {
	return (this->sharedData.scalarValue);
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
void * & AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::AABBBinaryTreeNode::PtrValue(void) {
	return (this->sharedData.ptrValue);
}

template <class OBJTYPE, class SCALARTYPE, class NODEAUXDATATYPE>
const void * & AABBBinaryTree<OBJTYPE, SCALARTYPE, NODEAUXDATATYPE>::AABBBinaryTreeNode::PtrValue(void) const {
	return (this->sharedData.ptrValue);
}

}	// end namespace vcg

#endif // #ifndef __VCGLIB_AABBBINARYTREEBASE_H
