#ifndef KDTREE_H
#define KDTREE_H

#include "../../point3.h"
#include "../../box3.h"
#include "mlsutils.h"
#include "priorityqueue.h"
#include <vector>
#include <limits>
#include <iostream>

template<typename _DataType>
class ConstDataWrapper
{
public:
		typedef _DataType DataType;
		inline ConstDataWrapper()
			: mpData(0), mStride(0), mSize(0)
		{}
		inline ConstDataWrapper(const DataType* pData, int size, int stride = sizeof(DataType))
			: mpData(reinterpret_cast<const unsigned char*>(pData)), mStride(stride), mSize(size)
		{}
		inline const DataType& operator[] (int i) const
		{
			return *reinterpret_cast<const DataType*>(mpData + i*mStride);
		}
		inline size_t size() const { return mSize; }
protected:
		const unsigned char* mpData;
		int mStride;
		size_t mSize;
};

template<class MeshType>
class VertexConstDataWrapper :public  ConstDataWrapper<typename MeshType::CoordType>
{
public:
  inline VertexConstDataWrapper(MeshType &m):
    ConstDataWrapper<typename MeshType::CoordType> ( &(m.vert[0].P()), m.vert.size(), sizeof(typename MeshType::VertexType))
     {}
};

/**
 * This class allows to create a Kd-Tree thought to perform the k-nearest neighbour query
 */
template<typename _Scalar>
class KdTree
{
public:

	typedef _Scalar Scalar;
	typedef vcg::Point3<Scalar> VectorType;
	typedef vcg::Box3<Scalar> AxisAlignedBoxType;

	struct Node
	{
		union {
                        //standard node
			struct {
				Scalar splitValue;
				unsigned int firstChildId:24;
				unsigned int dim:2;
				unsigned int leaf:1;
			};
                        //leaf
			struct {
				unsigned int start;
				unsigned short size;
			};
                };
	};
	typedef std::vector<Node> NodeList;

        // return the protected members which store the nodes and the points list
	inline const NodeList& _getNodes(void) { return mNodes; }
	inline const std::vector<VectorType>& _getPoints(void) { return mPoints; }


	void setMaxNofNeighbors(unsigned int k);
	inline int getNofFoundNeighbors(void) { return mNeighborQueue.getNofElements(); }
	inline const VectorType& getNeighbor(int i) { return mPoints[ mNeighborQueue.getIndex(i) ]; }
	inline unsigned int getNeighborId(int i) { return mIndices[mNeighborQueue.getIndex(i)]; }
	inline float getNeighborSquaredDistance(int i) { return mNeighborQueue.getWeight(i); }

public:

	KdTree(const ConstDataWrapper<VectorType>& points, unsigned int nofPointsPerCell = 16, unsigned int maxDepth = 64);

	~KdTree();

	void doQueryK(const VectorType& p);

protected:

	// element of the stack
	struct QueryNode
	{
			QueryNode() {}
			QueryNode(unsigned int id) : nodeId(id) {}
			unsigned int nodeId;  // id of the next node
			Scalar sq;            // squared distance to the next node
	};

	// used to build the tree: split the subset [start..end[ according to dim and splitValue,
	// and returns the index of the first element of the second subset
	unsigned int split(int start, int end, unsigned int dim, float splitValue);

	void createTree(unsigned int nodeId, unsigned int start, unsigned int end, unsigned int level, unsigned int targetCellsize, unsigned int targetMaxDepth);

protected:

        AxisAlignedBoxType mAABB; //BoundingBox
        NodeList mNodes; //kd-tree nodes
        std::vector<VectorType> mPoints; //points read from the input DataWrapper
        std::vector<int> mIndices; //points indices

        HeapMaxPriorityQueue<int,Scalar> mNeighborQueue; //used to perform the knn-query
        QueryNode mNodeStack[64]; //used in the implementation of the knn-query
};

template<typename Scalar>
KdTree<Scalar>::KdTree(const ConstDataWrapper<VectorType>& points, unsigned int nofPointsPerCell, unsigned int maxDepth)
        : mPoints(points.size()), mIndices(points.size())
{
        // compute the AABB of the input
        mPoints[0] = points[0];
        mAABB.Set(mPoints[0]);
        for (unsigned int i=1 ; i<mPoints.size() ; ++i)
        {
                mPoints[i] = points[i];
                mIndices[i] = i;
                mAABB.Add(mPoints[i]);
        }

        mNodes.reserve(4*mPoints.size()/nofPointsPerCell);

        //first node inserted (no leaf). The others are made by the createTree function (recursively)
        mNodes.resize(1);
        mNodes.back().leaf = 0;
        createTree(0, 0, mPoints.size(), 1, nofPointsPerCell, maxDepth);
}

template<typename Scalar>
KdTree<Scalar>::~KdTree()
{
}

template<typename Scalar>
void KdTree<Scalar>::setMaxNofNeighbors(unsigned int k)
{
        mNeighborQueue.setMaxSize(k);
}

/** Performs the kNN query.
        *
        * This algorithm uses the simple distance to the split plane to prune nodes.
        * A more elaborated approach consists to track the closest corner of the cell
        * relatively to the current query point. This strategy allows to save about 5%
        * of the leaves. However, in practice the slight overhead due to this tracking
        * reduces the overall performance.
        *
        * This algorithm also use a simple stack while a priority queue using the squared
        * distances to the cells as a priority values allows to save about 10% of the leaves.
        * But, again, priority queue insertions and deletions are quite involved, and therefore
        * a simple stack is by far much faster.
  *
  * The result of the query, the k-nearest neighbors, are internally stored into a stack, where the
  * topmost element [0] is NOT the nearest but the farthest!! (they are not sorted but arranged into a heap)
        */
template<typename Scalar>
void KdTree<Scalar>::doQueryK(const VectorType& queryPoint)
{
        mNeighborQueue.init();
        mNeighborQueue.insert(0xffffffff, std::numeric_limits<Scalar>::max());

        mNodeStack[0].nodeId = 0;
        mNodeStack[0].sq = 0.f;
        unsigned int count = 1;

        while (count)
        {
                //we select the last node (AABB) inserted in the stack
                QueryNode& qnode = mNodeStack[count-1];

                //while going down the tree qnode.nodeId is the nearest sub-tree, otherwise,
                //in backtracking, qnode.nodeId is the other sub-tree that will be visited iff
                //the actual nearest node is further than the split distance.
                Node& node = mNodes[qnode.nodeId];

                //if the distance is less than the top of the max-heap, it could be one of the k-nearest neighbours
                if (qnode.sq < mNeighborQueue.getTopWeight())
                {
                        //when we arrive to a lef
                        if (node.leaf)
                        {
                                --count; //pop of the leaf

                                //end is the index of the last element of the leaf in mPoints
                                unsigned int end = node.start+node.size;
                                //adding the element of the leaf to the heap
                                for (unsigned int i=node.start ; i<end ; ++i)
                                                mNeighborQueue.insert(i, vcg::SquaredNorm(queryPoint - mPoints[i]));
                        }
                        //otherwise, if we're not on a leaf
                        else
                        {   
                                // the new offset is the distance between the searched point and the actual split coordinate
                                float new_off = queryPoint[node.dim] - node.splitValue;

                                //left sub-tree
                                if (new_off < 0.)
                                {
                                                mNodeStack[count].nodeId  = node.firstChildId;
                                                //in the father's nodeId we save the index of the other sub-tree (for backtracking)
                                                qnode.nodeId = node.firstChildId+1;
                                }
                                //right sub-tree (same as above)
                                else
                                {
                                                mNodeStack[count].nodeId  = node.firstChildId+1;
                                                qnode.nodeId = node.firstChildId;
                                }
                                //distance is inherited from the father (while descending the tree it's equal to 0)
                                mNodeStack[count].sq = qnode.sq;
                                //distance of the father is the squared distance from the split plane
                                qnode.sq = new_off*new_off;
                                ++count;
                        }
                }
                else
                {
                        // pop
                        --count;
                }
        }
}

/**
 * Split the subarray between start and end in two part, one with the elements less than splitValue,
 * the other with the elements greater or equal than splitValue. The elements are compared
 * using the "dim" coordinate [0 = x, 1 = y, 2 = z].
 */
template<typename Scalar>
unsigned int KdTree<Scalar>::split(int start, int end, unsigned int dim, float splitValue)
{
        int l(start), r(end-1);
        for ( ; l<r ; ++l, --r)
        {
                while (l < end && mPoints[l][dim] < splitValue)
                        l++;
                while (r >= start && mPoints[r][dim] >= splitValue)
                        r--;
                if (l > r)
                        break;
                std::swap(mPoints[l],mPoints[r]);
                std::swap(mIndices[l],mIndices[r]);
        }
        //returns the index of the first element on the second part
        return (mPoints[l][dim] < splitValue ? l+1 : l);
}

/** recursively builds the kdtree
        *
        *  The heuristic is the following:
        *   - if the number of points in the node is lower than targetCellsize then make a leaf
        *   - else compute the AABB of the points of the node and split it at the middle of
        *     the largest AABB dimension.
        *
        *  This strategy might look not optimal because it does not explicitly prune empty space,
        *  unlike more advanced SAH-like techniques used for RT. On the other hand it leads to a shorter tree,
        *  faster to traverse and our experience shown that in the special case of kNN queries,
        *  this strategy is indeed more efficient (and much faster to build). Moreover, for volume data
        *  (e.g., fluid simulation) pruning the empty space is useless.
        *
        *  Actually, storing at each node the exact AABB (we therefore have a binary BVH) allows
        *  to prune only about 10% of the leaves, but the overhead of this pruning (ball/ABBB intersection)
        *  is more expensive than the gain it provides and the memory consumption is x4 higher !
        */
template<typename Scalar>
void KdTree<Scalar>::createTree(unsigned int nodeId, unsigned int start, unsigned int end, unsigned int level, unsigned int targetCellSize, unsigned int targetMaxDepth)
{
        //select the first node
        Node& node = mNodes[nodeId];
        AxisAlignedBoxType aabb;

        //putting all the points in the bounding box
        aabb.Set(mPoints[start]);
        for (unsigned int i=start+1 ; i<end ; ++i)
                aabb.Add(mPoints[i]);

        //bounding box diagonal
        VectorType diag = aabb.max - aabb.min;

        //the split "dim" is the dimension of the box with the biggest value
        unsigned int dim = vcg::MaxCoeffId(diag);
        node.dim = dim;
        //we divide the bounding box in 2 partitions, considering the average of the "dim" dimension
        node.splitValue = Scalar(0.5*(aabb.max[dim] + aabb.min[dim]));

        //midId is the index of the first element in the second partition
        unsigned int midId = split(start, end, dim, node.splitValue);


        node.firstChildId = mNodes.size();
        mNodes.resize(mNodes.size()+2);

        {
                // left child
                unsigned int childId = mNodes[nodeId].firstChildId;
                Node& child = mNodes[childId];
                if (midId - start <= targetCellSize || level>=targetMaxDepth)
                {
                                child.leaf = 1;
                                child.start = start;
                                child.size = midId - start;
                }
                else
                {
                                child.leaf = 0;
                                createTree(childId, start, midId, level+1, targetCellSize, targetMaxDepth);
                }
        }

        {
                // right child
                unsigned int childId = mNodes[nodeId].firstChildId+1;
                Node& child = mNodes[childId];
                if (end - midId <= targetCellSize || level>=targetMaxDepth)
                {
                        child.leaf = 1;
                        child.start = midId;
                        child.size = end - midId;
                }
                else
                {
                        child.leaf = 0;
                        createTree(childId, midId, end, level+1, targetCellSize, targetMaxDepth);
                }
        }
}

#endif

