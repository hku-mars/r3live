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


#ifndef VCG_SPACE_INDEX_OCTREETEMPLATE_H
#define VCG_SPACE_INDEX_OCTREETEMPLATE_H

#include <vcg/space/point3.h>
#include <vcg/space/box3.h>
#include <vector>


namespace vcg
{

/* Octree Template

Tiene un dataset volumetrico come un octree
Assunzione che la grandezza sia una potenza di due
La prof max e' fissa.
E' un octree in cui il dato e' nella cella dell'octree.
Anche i nodi non foglia hanno il dato Voxel

Assunzioni sul tipo voxel:
che abbia definiti gli operatori per poterci fare sopra pushpull.

Si tiene int invece di puntatori per garantirsi reallocazione dinamica.
I dati veri e propri stanno in un vettore di nodi
*/

template <typename VOXEL_TYPE, class SCALAR_TYPE>
class OctreeTemplate
{
protected:
	struct Node;

public:
	// Octree Type Definitions
	typedef unsigned long long				ZOrderType;
	typedef SCALAR_TYPE								ScalarType;
	typedef VOXEL_TYPE				        VoxelType;
	typedef VoxelType							  * VoxelPointer;
	typedef vcg::Point3i							CenterType;
	static const ScalarType						EXPANSION_FACTOR;
	typedef Node											NodeType;
	typedef	int												NodeIndex;
	typedef NodeType								* NodePointer;
	typedef	vcg::Box3<ScalarType>			BoundingBoxType;
	typedef vcg::Point3<ScalarType>	CoordinateType;

protected:
	/*
	* Inner structures:
	* Contains the information related to the octree node
	*/
	struct Node
	{
		// Default constructor: fill the data members with non-meaningful values
		Node()
		{
			parent = NULL;
			level  = -1;
		}

		// Constructor: create a new Node
		Node(NodePointer parent, int level)
		{
			this->parent	= parent;
			this->level		= (char) level;
		}

    virtual NodePointer &Son(int sonIndex) = 0;

    virtual bool	IsLeaf() = 0;

		// The position of the center of the node in integer coords in the 0..2^(2*sz) -1 range
		// The root has position (lsz/2,lsz/2,lsz/2)
		CenterType	center;
		char				level;
		NodePointer parent;
		VoxelType		voxel;
	};

	/*
	* Inner struct: Node
	*/
	struct InnerNode : public Node
	{
		InnerNode() : Node() {};
		InnerNode(NodePointer parent, int level) : Node(parent, level)
		{
			memset(&sons[0], 0, 8*sizeof(Node*));
		}

		inline NodePointer &Son(int sonIndex)
		{
			assert(0<=sonIndex && sonIndex<=8);
			return sons[sonIndex];
		}

		inline bool IsLeaf()
		{
			return false;
		}

		NodePointer sons[8];
	};

	/*
	* Inner struct: Leaf
	*/
	struct Leaf : public Node
	{
		Leaf() : Node() {};
		Leaf(NodePointer parent, int level) : Node(parent, level) {}

		inline NodePointer &Son(int /*sonIndex*/)
		{
			assert(false);
			static NodePointer p = NULL;
			return p;
		}

		inline bool IsLeaf()
		{
			return true;
		}
	};

public:
	// Inizializza l'octree
	void Initialize(int maximumDepth)
	{
		this->maximumDepth	= maximumDepth;
		size	= 1<< maximumDepth;			// e.g. 1*2^maxDepth
		lSize = 1<<(maximumDepth+1);	// e.g. 1*2^(maxDepth+1)

		InnerNode *root = new InnerNode(NULL,0);
		nodes.clear();
		nodes.push_back( root );
		root->center	= CenterType(size, size, size);

		ScalarType szf = (ScalarType) size;
		leafDimension  = boundingBox.Dim();
		leafDimension /= szf;
		leafDiagonal	 = leafDimension.Norm();
	};

	// Return the octree bounding-box
	inline BoundingBoxType		BoundingBox()								{ return boundingBox;				}

	// Return the Voxel of the n-th node
	inline VoxelPointer	Voxel(const NodePointer n)				{ return &(n->voxel);				}

	// Return the octree node count
	inline int					NodeCount()									const { return int(nodes.size()); }

	// Return the root index
        inline 				NodePointer Root()					const	{ return nodes[0];					}

	// Return the level of the n-th node
	inline char					Level(const NodePointer n)	const	{ return n->level;					}

	// Return the referente to the i-th son of the n-th node
	inline NodePointer&	Son(NodePointer n, int i)		const	{ return n->Son(i);					}

	// Return the parent index of the n-th node
	inline NodePointer	Parent(const NodePointer n) const	{ return n->parent;					}

	// Return the index of the current node in its father
	int WhatSon(NodePointer n) const
	{
		if(n==Root())
			assert(false);

		NodePointer parent = Parent(n);
		for(int i=0;i<8;++i)
			if(parent->Son(i)==n)
				return i;

		return -1;
	}

	// Return the center of the n-th node
	inline CenterType CenterInOctreeCoordinates(const NodePointer n) const { return n->center;}

	/*!
	* Return the center of the n-th node expressed in world-coordinate
	* \param NodePointer the pointer to the node whose center in world coordinate has to be computed
	*/
	inline void CenterInWorldCoordinates(const NodePointer n, CoordinateType &wc_Center) const
	{
		assert(0<=n && n<NodeCount());

		int shift = maximumDepth - Level(n) + 1;
		CoordinateType ocCenter = CenterInOctreeCoordinates(n);
		CoordinateType nodeSize = boundingBox.Dim()/float(1<<Level(n));
		wc_Center.X() = boundingBox.min.X() + (nodeSize.X()*(0.5f+(ocCenter.X()>>shift)));
		wc_Center.Y() = boundingBox.min.Y() + (nodeSize.Y()*(0.5f+(ocCenter.Y()>>shift)));
		wc_Center.Z() = boundingBox.min.Z() + (nodeSize.Z()*(0.5f+(ocCenter.Z()>>shift)));
	};

	// Given a node (even not leaf) it returns the center of the box it represent.
	// the center is expressed not in world-coordinates.
	// e.g. the root is (sz/2,sz/2,sz/2);
	// and the finest element in the grid in lower left corner has center (.5, .5, .5)
	/*

	4----------------   4----------------   4----------------
	|   |   |   |   |   |       |       |   |               |
	3---+---+---+---|   3       |       |   3               |
	|   |   |   |   |   |       |       |   |               |
	2---+---+---+---|   2---+---+---+---|   2       c       |
	|   |   |   |   |   |       |       |   |               |
	1---+---+---+---|   1   b   +       |   1               |
	| a |   |   |   |   |       |       |   |               |
	0---1---2---3---4   0---1---2---3---4   0---1---2---3---4

	This is a tree with maxdepth==2, so sz is 2^2=4

	a) a leaf at the deepest level 2 has position (.5,.5)
	b) a mid node (lev 1) has position (1,1)
	c) root has level 0 and position (sz/2,sz/2) = (2,2)

	The center of a node has integer coords in the 2^(MaxDepth+1) range.

	The other approach is to use position as a bit string
	codifying the tree path, but in this case you have to
	supply also the level (e.g. the string length)
	you desire. The lower left corner node is always 0 ( (,) for the root (0,0) level 1, and (00,00) for level 2)

	|              ~~~              |
	|      0~~      |      1~~      |
	|  00~  |  01~  |  10~  |  11~  |
	|000|001|010|011|100|101|110|111|

	The interesting properties is that
	if your octree represent a space [minv,maxv] and you want
	to find the octree cell containing a point p in [minv,maxv]
	you just have to convert p in the range [0,sz) truncate it to an integer and use it as a path.
	For example, consider an octree of depth 3, representing a range [0..100)
	sz=8 (each cell contains form 0 to 12.5
	the point
	5		-> 0.4	-> path is 000
	45	-> 3.6	-> path is 011
	50  -> 4.0	-> path is 100
	100 -> 8		-> ERROR the interval is right open!!!

	Note how each cell is meant to contains a right open interval (e.g. the first cell contains [0,12.5) and the second [12.5,25) and so on)

	The center of each cell can simply be obtained by adding .5 to the path of the leaves.
	*/
	CoordinateType Center(NodePointer n) const
	{
		CoordinateType center;
		center.Import(GetPath(n));
		center+=Point3f(.5f,.5f,.5f);

		//TODO verify the assert
		assert(center==nodes[n]->center);

		return center;
	}

	// Return the bounding-box of the n-th node expressed in world-coordinate
	BoundingBoxType BoundingBoxInWorldCoordinates(const NodePointer n)
	{
		char level = Level(n);
		int shift  = maximumDepth-level+1;
		CoordinateType	nodeDim = boundingBox.Dim()/float(1<<level);
		CenterType			center	 = CenterInOctreeCoordinates(n);
		BoundingBoxType	nodeBB;
		nodeBB.min.X() = boundingBox.min.X() + (nodeDim.X()*(center.X()>>shift));
		nodeBB.min.Y() = boundingBox.min.Y() + (nodeDim.Y()*(center.Y()>>shift));
		nodeBB.min.Z() = boundingBox.min.Z() + (nodeDim.Z()*(center.Z()>>shift));
		nodeBB.max = nodeBB.min+nodeDim;
		return nodeBB;
	};

	/*!
	* Return the bounding-box of a node expressed in world-coordinate
	* \param NodePointer  the node whose bounding-box has to be computed
	* \param wc_BB				the bounding-box of the node in world coordinta
	*/
	inline void BoundingBoxInWorldCoordinates(const NodePointer n, BoundingBoxType &wc_bb) const
	{
		char level = Level(n);
		int	 shift = maximumDepth - level + 1;
		CoordinateType node_dimension = boundingBox.Dim()/ScalarType(1<<level);
		wc_bb.min.X()	= boundingBox.min.X()+(node_dimension.X()*(n->center.X()>>shift));
		wc_bb.min.Y()	= boundingBox.min.Y()+(node_dimension.Y()*(n->center.Y()>>shift));
		wc_bb.min.Z()	= boundingBox.min.Z()+(node_dimension.Z()*(n->center.Z()>>shift));
		wc_bb.max	= wc_bb.min+node_dimension;
	};

	// Return one of the 8 subb box of a given box.
	BoundingBoxType SubBox(BoundingBoxType &lbb, int i)
	{
		BoundingBoxType bs;
		if (i&1)	bs.min.X()=(lbb.min.X()+(bs.max.X()=lbb.max.X()))/2.0f;
		else			bs.max.X()=((bs.min.X()=lbb.min.X())+lbb.max.X())/2.0f;
		if (i&2)	bs.min.Y()=(lbb.min.Y()+(bs.max.Y()=lbb.max.Y()))/2.0f;
		else			bs.max.Y()=((bs.min.Y()=lbb.min.Y())+lbb.max.Y())/2.0f;
		if (i&4)	bs.min.Z()=(lbb.min.Z()+(bs.max.Z()=lbb.max.Z()))/2.0f;
		else			bs.max.Z()=((bs.min.Z()=lbb.min.Z())+lbb.max.Z())/2.0f;

		return bs;
	}

	// Given the bounding-box and the center (both in world-coordinates)
	// of a node, return the bounding-box (in world-coordinats) of the i-th son
	BoundingBoxType SubBoxAndCenterInWorldCoordinates(BoundingBoxType &lbb, CoordinateType &center, int i)
	{
		BoundingBoxType bs;
		if (i&1)
		{
			bs.min[0]=center[0];
			bs.max[0]=lbb.max[0];
		}
		else
		{
			bs.min[0]=lbb.min[0];
			bs.max[0]=center[0];
		}
		if (i&2)
		{
			bs.min[1]=center[1];
			bs.max[1]=lbb.max[1];
		}
		else
		{
			bs.max[1]=center[1];
			bs.min[1]=lbb.min[1];
		}
		if (i&4)
		{
			bs.min[2]=center[2];
			bs.max[2]=lbb.max[2];
		}
		else
		{
			bs.max[2]=center[2];
			bs.min[2]=lbb.min[2];
		}
		return bs;
	};

	/*
	* Add a new Node to the octree.
	*	The created node is the i-th son of the node pointed to by parent.
	*	Return the pointer to the new node
	*/
	NodePointer NewNode(NodePointer parent, int i)
	{
		assert(0<=i && i<8);
		assert(Son(parent, i)==NULL);

		//int index  = NodeCount();
		char level = Level(parent)+1;

		Node *node = (level<maximumDepth)? (Node*) new InnerNode(parent, level) : (Node*) new Leaf(parent, level);
		nodes.push_back( node );
		Son(parent, i) = node;

		CenterType *parentCenter = &(parent->center);
		int displacement = 1<<(maximumDepth-level);
		node->center.X() = parentCenter->X() + ((i&1)? displacement : -displacement);
		node->center.Y() = parentCenter->Y() + ((i&2)? displacement : -displacement);
		node->center.Z() = parentCenter->Z() + ((i&4)? displacement : -displacement);

		return node;
	}

	// Aggiunge un nodo all'octree nella posizione specificata e al livello specificato.
	// Vengono inoltre inseriti tutti gli antenati mancanti per andare dalla radice
	// al nodo ed al livello specificato seguendo path.
	NodePointer AddNode(CenterType path)
	{
		//the input coordinates must be in the range 0..2^maxdepth
		assert(path[0]>=0 && path[0]<size);
		assert(path[1]>=0 && path[1]<size);
		assert(path[2]>=0 && path[2]<size);

		NodePointer curNode	= Root();
		int rootLevel				= 0;
		int shiftLevel			= maximumDepth-1;

		while(shiftLevel >= rootLevel)
		{
			int nextSon=0;
			if((path[0]>>shiftLevel)%2) nextSon +=1;
			if((path[1]>>shiftLevel)%2) nextSon +=2;
			if((path[2]>>shiftLevel)%2) nextSon +=4;
			NodePointer nextNode = Son(curNode, nextSon);
			if(nextNode!=NULL) // nessun nodo pu aver Root() per figlio
				curNode = nextNode;
			else
			{
				NodePointer newNode = NewNode(curNode, nextSon);
				assert(Son(curNode, nextSon)==newNode); // TODO delete an assignment
				curNode=newNode;
			}
			--shiftLevel;
		}
		return curNode;
	}

	/*!
	* Given a query point, compute the z_order of the leaf where this point would be contained.
	* This leaf not necessarily must be exist!
	*/
	// Convert the point p coordinates to the integer based representation
	// in the range 0..size, where size is 2^maxdepth
	CenterType Interize(const CoordinateType &pf) const
	{
		CenterType pi;

		assert(pf.X()>=boundingBox.min.X() &&  pf.X()<=boundingBox.max.X());
		assert(pf.Y()>=boundingBox.min.Y() &&  pf.Y()<=boundingBox.max.Y());
		assert(pf.Z()>=boundingBox.min.Z() &&  pf.Z()<=boundingBox.max.Z());

		pi.X() = int((pf.X() - boundingBox.min.X()) * size / (boundingBox.max.X() - boundingBox.min.X()));
		pi.Y() = int((pf.Y() - boundingBox.min.Y()) * size / (boundingBox.max.Y() - boundingBox.min.Y()));
		pi.Z() = int((pf.Z() - boundingBox.min.Z()) * size / (boundingBox.max.Z() - boundingBox.min.Z()));

		return pi;
	}

	// Inverse function of Interize;
	// Return to the original coords space (not to the original values!!)
	CoordinateType DeInterize(const CenterType &pi ) const
	{
		CoordinateType pf;

		assert(pi.X()>=0 && pi.X()<size);
		assert(pi.Y()>=0 && pi.Y()<size);
		assert(pi.Z()>=0 && pi.Z()<size);

		pf.X() = pi.X() * (boundingBox.max.X() - boundingBox.min.X()) / size + boundingBox.min.X();
		pf.Y() = pi.Y() * (boundingBox.max.Y() - boundingBox.min.Y()) / size + boundingBox.min.Y();
		pf.Z() = pi.Z() * (boundingBox.max.Z() - boundingBox.min.Z()) / size + boundingBox.min.Z();

		return pf;
	}

	// Compute the z-ordering integer value for a given node;
	// this value can be used to compute a complete ordering of the nodes of a given level of the octree.
	// It assumes that the octree has a max depth of 10.
	ZOrderType ZOrder(NodePointer n)											const	{ return ZOrder(GetPath(n), Level(n)); }
	ZOrderType ComputeZOrder(const CoordinateType &query) const { return ZOrder(CenterType::Construct(Interize(query)), maximumDepth); };

	inline ZOrderType ZOrder(const CenterType &path, const char level) const
	{
		ZOrderType finalPosition = 0;
		ZOrderType currentPosition;

		for(int i=0; i<level; ++i)
		{
			currentPosition = 0;
			int mask=1<<i;
			if(path[0]&mask) currentPosition|=1;
			if(path[1]&mask) currentPosition|=2;
			if(path[2]&mask) currentPosition|=4;
			currentPosition = currentPosition<<(i*3);
			finalPosition	 |= currentPosition;
		}
		return finalPosition;
	};

	// Funzione principale di accesso secondo un path;
	// restituisce l'indice del voxel di profondita' massima
	// che contiene il punto espresso in range 0..2^maxk
	NodePointer DeepestNode(CenterType path, int MaxLev)
	{
		assert(path[0]>=0 && path[0]<size);
		assert(path[1]>=0 && path[1]<size);
		assert(path[2]>=0 && path[2]<size);

		NodePointer curNode	= Root();
		int shift						= maximumDepth-1;

		while(shift && Level(curNode) < MaxLev)
		{
			int son = 0;
			if((path[0]>>shift)%2) son +=1;
			if((path[1]>>shift)%2) son +=2;
			if((path[2]>>shift)%2) son +=4;
			NodePointer nextNode = Son(curNode, son);
			if(nextNode!=NULL)
				curNode=nextNode;
			else
				break;

			--shift;
		}
		return curNode;
	}


	// Return the 'path' from root to the specified node;
	// the path is coded as a point3s; each bit of each component code the direction in one level
	// only the last 'level' bits of the returned value are meaningful
	// for example for the root no bit are meaningfull (path is 0)
	// for the first level only one bit of each one of the three components are maninguful;
	CenterType GetPath(NodePointer n) const
	{
		if(n==Root())
			return CenterType(0,0,0);

		CenterType	path(0,0,0);

		int shift, mask, son;
		int startingLevel = int(Level(n));
		while (n!=Root())
		{
			shift = startingLevel-Level(n); //nodes[n].level
			mask = 1 << shift;							// e.g. 1*2^shift
			son = WhatSon(n);
			if(son&1) path[0] |= mask;
			if(son&2) path[1] |= mask;
			if(son&4) path[2] |= mask;
			n = Parent(n);									// nodes[n].parent
		}
		return path;
	}

	// Dato un punto 3D nello spazio restituisce un array contenente
	// i puntatori ai nodi che lo contengono, dalla radice fino alle foglie.
	// I nodi mancanti dalla radice fino a profondit maxDepth vengono aggiunti.
	// In posizione i ci sar il nodo di livello i.
	// Restituisce lo z-order del punto p
	ZOrderType BuildRoute(const CoordinateType &p, NodePointer *&route)
	{
		assert( boundingBox.min.X()<=p.X() && p.X()<=boundingBox.max.X() );
		assert( boundingBox.min.Y()<=p.Y() && p.Y()<=boundingBox.max.Y() );
		assert( boundingBox.min.Z()<=p.Z() && p.Z()<=boundingBox.max.Z() );

		route[0]						= Root();
		NodePointer curNode = Root();
		int shift						= maximumDepth-1;
		CenterType path			= CenterType::Construct(Interize(p));
		while(shift >= 0)
		{
			int son = 0;
			if((path[0]>>shift)%2) son +=1;
			if((path[1]>>shift)%2) son +=2;
			if((path[2]>>shift)%2) son +=4;
			NodePointer nextNode = Son(curNode, son);

			if(nextNode!=NULL)
			{
				route[maximumDepth-shift] = nextNode;
				curNode										= nextNode;
			}
			else
			{
				NodePointer newNode = NewNode(curNode, son);
				route[maximumDepth-shift] = newNode;
				curNode = newNode;
			}
			--shift;
		}
		return ZOrder(route[maximumDepth]);
	}; //end of BuildRoute


	// Restituisce il percorso dalla radice fino al nodo di profondit
	// massima presente nell'octree contenente il nodo p. Nessun nuovo nodo  aggiunto
	// all'octree. In route sono inseriti gli indici dei nodi contenti p, dalla radice
	// fino al nodo di profontid massima presente; nelle eventuali posizioni rimaste
	// libere  inserito il valore -1. Restituisce true se il punto p cade in una foglia
	// dell'otree, false altrimenti
	bool GetRoute(const CoordinateType &p, NodePointer *&route)
	{
		assert( boundingBox.min.X()<=p.X() && p.X()<=boundingBox.max.X() );
		assert( boundingBox.min.Y()<=p.Y() && p.Y()<=boundingBox.max.Y() );
		assert( boundingBox.min.Z()<=p.Z() && p.Z()<=boundingBox.max.Z() );

		memset(route, NULL, maximumDepth*sizeof(NodePointer));

		CenterType path				 = CenterType::Construct(Interize(p));
		int shift							 = maximumDepth-1;
		NodePointer finalLevel = Root();
		NodePointer curNode		 = Root();

		while(shift >= finalLevel)
		{
			int son=0;
			if((path[0]>>shift)%2) son +=1;
			if((path[1]>>shift)%2) son +=2;
			if((path[2]>>shift)%2) son +=4;
			NodePointer nextNode = Son(curNode, son);
			if(nextNode!=NULL)
			{
				route[maximumDepth-shift] = nextNode;
				curNode = nextNode;
			}
			else
				return false;

			--shift;
		}
		return true;
	}; //end of GetReoute

	// Data una bounding-box bb_query, calcola l'insieme dei nodi di
	// profondit depth il cui bounding-box ha intersezione non nulla con
	// bb (la bounding-box dell'octree); i puntatori a tali nodi sono
	// inseriti progressivamente in contained_nodes.
	// The vector nodes must be cleared before calling this method.
	void ContainedNodes
	(
		BoundingBoxType						 &query,
		std::vector< NodePointer > &nodes,
		int													depth,
		NodePointer									n,
		BoundingBoxType						 &nodeBB)
	{
		if (!query.Collide(nodeBB))
			return;

		if (Level(n)==depth)
			nodes.push_back(n);
		else
		{
			NodePointer	son;
			BoundingBoxType	sonBB;
			CoordinateType  nodeCenter = nodeBB.Center();
			for (int s=0; s<8; s++)
			{
				son = Son(n, s);
				if (son!=NULL)
				{
					sonBB = SubBoxAndCenterInWorldCoordinates(nodeBB, nodeCenter, s);
					ContainedNodes(query, nodes, depth, son, sonBB);
				}
			}
		}
	}; //end of ContainedNodes


	// Data una bounding-box bb, calcola l'insieme delle foglie il cui
	// bounding-box ha intersezione non nulla con bb; i loro indici
	// sono inseriti all'interno di leaves.
	void ContainedLeaves(
		BoundingBoxType						 &query,
		std::vector< NodePointer > &leaves,
		NodePointer									node,
		BoundingBoxType						 &nodeBB
		)
	{
		NodePointer	son;
		BoundingBoxType	sonBB;
		CoordinateType nodeCenter = nodeBB.Center();
		for (int s=0; s<8; s++)
		{
			son = Son(node, s); //nodes[nodeIndex].sonIndex[s]
			if (son!=NULL)
			{
				sonBB = SubBoxAndCenterInWorldCoordinates(nodeBB, nodeCenter, s);
				if ( query.Collide(sonBB) )
				{
					if ( son->IsLeaf() )
						leaves.push_back(son);
					else
						ContainedLeaves(query, leaves, son, sonBB);
				}
			}
		}
	}; //end of ContainedLeaves


	/*
	* Octree Data Members
	*/
public:
	// the size of the finest grid available (2^maxDepth)
	int						size;

	// double the size(2^maxDepth)
	int						lSize;

	// The allowed maximum depth
	int						maximumDepth;

	// The dimension of a leaf
	CoordinateType	leafDimension;

	// The diagonal of a leaf
	ScalarType			leafDiagonal;

	// The Octree nodes
	std::vector< Node* > nodes;

	// The bounding box containing the octree (in world coordinate)
	BoundingBoxType				boundingBox;
}; //end of class OctreeTemplate

template <typename VOXEL_TYPE, class SCALAR_TYPE>
const SCALAR_TYPE OctreeTemplate<VOXEL_TYPE, SCALAR_TYPE>::EXPANSION_FACTOR  = SCALAR_TYPE(0.035);
}

#endif //VCG_SPACE_INDEX_OCTREETEMPLATE_H
