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

#ifndef VCG_SPACE_INDEX_OCTREE_H
#define VCG_SPACE_INDEX_OCTREE_H

#include <stdlib.h>

#ifdef __glut_h__
#include <vcg/space/color4.h>
#include <wrap/gl/space.h>
#endif

#include <vcg/space/index/base.h>
#include <vcg/space/index/octree_template.h>
#include <vcg/space/box3.h>

namespace vcg
{
    /*!
    * Given an object or an object pointer, return the reference to the object
    */
    template <typename TYPE>
    struct Dereferencer
    {
        static				TYPE& Ref(TYPE &t)				{ return ( t);	}
        static				TYPE& Ref(TYPE* &t)				{ return (*t);	}
        static const	TYPE& Ref(const TYPE &t)	{ return ( t);	}
        static const	TYPE& Ref(const TYPE* &t) { return (*t);	}
    };


    /*!
    * Given a type, return the type
    */
    template <typename T>
    class ReferenceType
    {
    public:
        typedef T Type;
    };


    /*!
    * Given as type a pointer to type, return the type
    */
    template <typename T>
    class ReferenceType<T *>
    {
    public:
        typedef typename ReferenceType<T>::Type Type;
    };



    /*!
    * The type of the octree voxels
    */
    struct Voxel
    {
        Voxel() { count = begin = end = -1; }

        void SetRange(const int begin, const int end)
        {
            this->begin = begin;
            this->end		= end;
            count				= end-begin;
        };

        void AddRange(const Voxel *voxel)
        {
            assert(voxel->end>end);

            count += voxel->count;
            end		 = voxel->end;
        };

        int begin;
        int end;
        int count;
    };


    template < class OBJECT_TYPE, class SCALAR_TYPE>
    class Octree : public vcg::OctreeTemplate< Voxel, SCALAR_TYPE >, public vcg::SpatialIndex< OBJECT_TYPE, SCALAR_TYPE >
    {
    protected:
         struct Neighbour;

    public:
        typedef						SCALAR_TYPE													ScalarType;
        typedef						OBJECT_TYPE													ObjectType;
        typedef typename	Octree::Leaf											* LeafPointer;
        typedef typename	Octree::InnerNode									* InnerNodePointer;
        typedef typename	ReferenceType<OBJECT_TYPE>::Type	* ObjectPointer;

        typedef 					vcg::Voxel													VoxelType;
        typedef						VoxelType													* VoxelPointer;

        typedef vcg::OctreeTemplate< VoxelType, SCALAR_TYPE >	TemplatedOctree;
        typedef typename TemplatedOctree::ZOrderType 					ZOrderType;

        typedef typename TemplatedOctree::BoundingBoxType 		BoundingBoxType;
        typedef typename TemplatedOctree::CenterType 					CenterType;
        typedef typename TemplatedOctree::CoordinateType			CoordType;

        typedef typename TemplatedOctree::NodeType						NodeType;
        typedef typename TemplatedOctree::NodePointer 				NodePointer;
        typedef typename TemplatedOctree::NodeIndex						NodeIndex;

        typedef typename std::vector< Neighbour >::iterator   NeighbourIterator;

    protected:
        /***********************************************
        *     INNER DATA STRUCTURES AND PREDICATES     *
        ***********************************************/
        /*!
        * Structure used during the sorting of the dataset
        */
        template < typename LEAF_TYPE >
        struct ObjectPlaceholder
        {
            typedef LEAF_TYPE* LeafPointer;

            ObjectPlaceholder() { z_order = object_index = -1, leaf_pointer = NULL;}

            ObjectPlaceholder(ZOrderType zOrder, void* leafPointer, unsigned int objectIndex)
            {
                z_order				= zOrder;
                leaf_pointer	= leafPointer;
                object_index	= objectIndex;
            }

            ZOrderType		z_order;
            LeafPointer		leaf_pointer;
            unsigned int	object_index;
        };


        /*!
        * Predicate used during the sorting of the dataset
        */
        template <typename LEAF_TYPE >
        struct ObjectSorter
        {
            inline bool operator()(const ObjectPlaceholder< LEAF_TYPE > &first, const ObjectPlaceholder< LEAF_TYPE > &second)
            {
                return (first.z_order<second.z_order);
            }
        };

        /*!
        * Structure which holds the reference to the object and the position of the mark for that object
        */
        struct ObjectReference
        {
            ObjectReference() {pMark=NULL; pObject=NULL;}

            unsigned char *pMark;
            ObjectPointer  pObject;
        };

        /*
        * The generic item in the neighbors vector computed by GetNearestNeighbors;
        */
        struct Neighbour
        {
            Neighbour()
            {
                this->object	 = NULL;
                this->distance = -1.0f;
            };

            Neighbour(ObjectPointer &object, CoordType &point, ScalarType distance)
            {
                this->object	 = object;
                this->point		 = point;
                this->distance = distance;
            }

            inline bool operator<(const Neighbour &n) const
            {
                return distance<n.distance;
            }


            ObjectPointer		object;
            CoordType				point;
            ScalarType			distance;
        };

public:
      Octree()
        {
        marks=0;
        }
        ~Octree()
        {
            if(marks) delete []marks;
            int node_count = TemplatedOctree::NodeCount();
            for (int i=0; i<node_count; i++)
                delete TemplatedOctree::nodes[i];
            TemplatedOctree::nodes.clear();
        }


        /*!
        * Populate the octree
        */
        template < class OBJECT_ITERATOR >
        void Set(const OBJECT_ITERATOR & bObj, const OBJECT_ITERATOR & eObj /*, vcg::CallBackPos *callback=NULL*/)
        {
            // Compute the bounding-box enclosing the whole dataset
            typedef Dereferencer<typename ReferenceType<typename OBJECT_ITERATOR::value_type>::Type >	DereferencerType;
            BoundingBoxType bounding_box, obj_bb;
            bounding_box.SetNull();
            for (OBJECT_ITERATOR iObj=bObj; iObj!=eObj; iObj++)
            {
                (*iObj).GetBBox(obj_bb);
                bounding_box.Add(obj_bb);
            }

            //...and expand it a bit more
            BoundingBoxType resulting_bb(bounding_box);
            CoordType offset = bounding_box.Dim()*Octree::EXPANSION_FACTOR;
            CoordType center = bounding_box.Center();
            resulting_bb.Offset(offset);
      ScalarType longest_side = vcg::math::Max( resulting_bb.DimX(), resulting_bb.DimY(), resulting_bb.DimZ())/2.0f;
            resulting_bb.Set(center);
            resulting_bb.Offset(longest_side);
            TemplatedOctree::boundingBox = resulting_bb;

            // Try to find a reasonable octree depth
            int dataset_dimension = int(std::distance(bObj, eObj));

            int primitives_per_voxel;
            int depth = 4;
            do
            {
                int		number_of_voxel = 1<<(3*depth); // i.e. 8^depth
                float density					= float(number_of_voxel)/float(depth);
                primitives_per_voxel	= int(float(dataset_dimension)/density);
                depth++;
            }
            while (primitives_per_voxel>25 && depth<15);
            TemplatedOctree::Initialize(++depth);

            // Sort the dataset (using the lebesgue space filling curve...)
            std::string message("Indexing dataset...");
            NodePointer *route = new NodePointer[depth+1];
            OBJECT_ITERATOR iObj = bObj;

            //if (callback!=NULL) callback(int((i+1)*100/dataset_dimension), message.c_str());

            std::vector< ObjectPlaceholder< NodeType > > placeholders/*(dataset_dimension)*/;
            vcg::Box3<ScalarType>		object_bb;
            vcg::Point3<ScalarType> hit_leaf;
            for (int i=0; i<dataset_dimension; i++, iObj++)
            {
                (*iObj).GetBBox(object_bb);
                hit_leaf  = object_bb.min;

                while (object_bb.IsIn(hit_leaf))
                {
                    int placeholder_index = int(placeholders.size());
                    placeholders.push_back( ObjectPlaceholder< NodeType >() );
                    placeholders[placeholder_index].z_order			 = TemplatedOctree::BuildRoute(hit_leaf, route);
                    placeholders[placeholder_index].leaf_pointer = route[depth];
                    placeholders[placeholder_index].object_index = i;

                    hit_leaf.X() += TemplatedOctree::leafDimension.X();
                    if (hit_leaf.X()>object_bb.max.X())
                    {
                        hit_leaf.X() = object_bb.min.X();
                        hit_leaf.Z()+= TemplatedOctree::leafDimension.Z();
                        if (hit_leaf.Z()>object_bb.max.Z())
                        {
                            hit_leaf.Z() = object_bb.min.Z();
                            hit_leaf.Y()+= TemplatedOctree::leafDimension.Y();
                        }
                    }
                }
            }
            delete []route;

            int placeholder_count = int(placeholders.size());

            // Allocate the mark array
            global_mark				= 1;
            marks							= new unsigned char[placeholder_count];
            memset(&marks[0], 0, sizeof(unsigned char)*placeholder_count);

            std::sort(placeholders.begin(), placeholders.end(), ObjectSorter< NodeType >());
            std::vector< NodePointer > filled_leaves(placeholder_count);
            sorted_dataset.resize( placeholder_count );
            for (int i=0; i<placeholder_count; i++)
            {
                std::advance((iObj=bObj), placeholders[i].object_index);
                sorted_dataset[i].pObject	= &DereferencerType::Ref(*iObj);
                sorted_dataset[i].pMark		= &marks[i];
                filled_leaves[i]					= placeholders[i].leaf_pointer;
            }

            // The dataset is sorted and the octree is built, but the indexing information aren't stored yet in the octree:
            // we assign to each leaf the range inside the sorted dataset of the primitives contained inside the leaf
            int begin									= -1;
            NodePointer initial_leaf	= NULL;
            for (int end=0; end<placeholder_count; )
            {
                begin = end;
                initial_leaf = filled_leaves[begin];
                do end++;
                while (end<placeholder_count && initial_leaf==filled_leaves[end]);

                VoxelType *voxel = TemplatedOctree::Voxel(initial_leaf);
                voxel->SetRange(begin, end);
            }

            // The octree is built, the dataset is sorted but only the leaves are indexed:
            // we propagate the indexing information bottom-up to the root
            IndexInnerNodes( TemplatedOctree::Root() );
        } //end of Set

        /*!
        * Finds the closest object to a given point.
        */
        template <class OBJECT_POINT_DISTANCE_FUNCTOR, class OBJECT_MARKER>
        ObjectPointer GetClosest
        (
            OBJECT_POINT_DISTANCE_FUNCTOR & distance_functor,
            OBJECT_MARKER									& /*marker*/,
            const CoordType								& query_point,
            const ScalarType							& max_distance,
            ScalarType										& distance,
            CoordType											& point,
            bool														allow_zero_distance = true
        )
        {
            BoundingBoxType query_bb;
            ScalarType sphere_radius;
            if (!GuessInitialBoundingBox(query_point, max_distance, sphere_radius, query_bb))
                return NULL;

            std::vector< NodePointer > leaves;

            //unsigned int object_count;
            //int					 leaves_count;

            IncrementMark();
            AdjustBoundingBox(query_bb, sphere_radius, max_distance, leaves, 1);

            if (sphere_radius>max_distance)
                return NULL;

            std::vector< Neighbour > neighbors;
            RetrieveContainedObjects(query_point, distance_functor, max_distance, allow_zero_distance, leaves, neighbors);

            typename std::vector< Neighbour >::iterator first = neighbors.begin();
            typename std::vector< Neighbour >::iterator last	= neighbors.end();
            std::partial_sort(first, first+1, last);

            distance	= neighbors[0].distance;
            point			= neighbors[0].point;
            return			neighbors[0].object;
        }; //end of GetClosest

        /*!
        * Retrieve the k closest element to the query point
        */
        template <class OBJECT_POINT_DISTANCE_FUNCTOR, class OBJECT_MARKER, class OBJECT_POINTER_CONTAINER, class DISTANCE_CONTAINER, class POINT_CONTAINER>
        unsigned int GetKClosest
            (
            OBJECT_POINT_DISTANCE_FUNCTOR & distance_functor,
            OBJECT_MARKER									& /*marker*/,
            unsigned int										k,
            const CoordType								& query_point,
            const ScalarType							& max_distance,
            OBJECT_POINTER_CONTAINER			& objects,
            DISTANCE_CONTAINER						& distances,
            POINT_CONTAINER								& points,
            bool													  sort_per_distance   = true,
            bool													  allow_zero_distance = true
            )
        {
            BoundingBoxType query_bb;
            ScalarType sphere_radius;
            if (!GuessInitialBoundingBox(query_point, max_distance, sphere_radius, query_bb))
                return 0;

            std::vector< NodePointer > leaves;
            std::vector< Neighbour	 > neighbors;

            unsigned int object_count;
            float				 k_distance;

OBJECT_RETRIEVER:
            IncrementMark();
            AdjustBoundingBox(query_bb, sphere_radius, max_distance, leaves, k);
            object_count = RetrieveContainedObjects(query_point, distance_functor, max_distance, allow_zero_distance, leaves, neighbors);

            if (sphere_radius<max_distance && object_count<k)
                goto OBJECT_RETRIEVER;

            NeighbourIterator first = neighbors.begin();
            NeighbourIterator last	= neighbors.end();

      object_count = std::min(k, object_count);
            if (sort_per_distance)  std::partial_sort< NeighbourIterator >(first, first+object_count, last );
            else										std::nth_element < NeighbourIterator >(first, first+object_count, last );

            k_distance = neighbors[object_count-1].distance;
            if (k_distance>sphere_radius && sphere_radius<max_distance)
                goto OBJECT_RETRIEVER;

            return CopyQueryResults<OBJECT_POINTER_CONTAINER, DISTANCE_CONTAINER, POINT_CONTAINER>(neighbors, object_count, objects, distances, points);
        }; //end of GetKClosest


        /*!
        * Returns all the objects contained inside a specified sphere
        */
        template <class OBJECT_POINT_DISTANCE_FUNCTOR, class OBJECT_MARKER, class OBJECT_POINTER_CONTAINER, class DISTANCE_CONTAINER, class POINT_CONTAINER>
        unsigned int GetInSphere
            (
                OBJECT_POINT_DISTANCE_FUNCTOR		&distance_functor,
                OBJECT_MARKER										&/*marker*/,
                const CoordType									&sphere_center,
                const ScalarType								&sphere_radius,
                OBJECT_POINTER_CONTAINER				&objects,
                DISTANCE_CONTAINER							&distances,
                POINT_CONTAINER									&points,
                bool													 sort_per_distance   = false,
                bool													 allow_zero_distance = false
            )
        {
            // Define the minimum bounding-box containing the sphere
            BoundingBoxType query_bb(sphere_center, sphere_radius);

            // If that bounding-box don't collide with the octree bounding-box, simply return 0
            if (!TemplatedOctree::boundingBox.Collide(query_bb))
                return 0;

            std::vector< NodePointer > leaves;
            std::vector< Neighbour	 > neighbors;

            IncrementMark();
            ContainedLeaves(query_bb, leaves, TemplatedOctree::Root(), TemplatedOctree::boundingBox);

            int	leaves_count = int(leaves.size());
            if (leaves_count==0)
                return 0;

            int object_count = RetrieveContainedObjects(sphere_center, distance_functor, sphere_radius, allow_zero_distance, leaves, neighbors);

            NeighbourIterator first = neighbors.begin();
            NeighbourIterator last	= neighbors.end();
            if (sort_per_distance)  std::partial_sort< NeighbourIterator >(first, first+object_count, last );
            else										std::nth_element < NeighbourIterator >(first, first+object_count, last );

            return CopyQueryResults<OBJECT_POINTER_CONTAINER, DISTANCE_CONTAINER, POINT_CONTAINER>(neighbors, object_count, objects, distances, points);
        };//end of GetInSphere

        /*!
        * Returns all the objects lying inside the specified bbox
        */
            template <class OBJECT_MARKER, class OBJECT_POINTER_CONTAINER>
            unsigned int GetInBox
                (
                    OBJECT_MARKER							&/*marker*/,
                    const BoundingBoxType			&query_bounding_box,
                    OBJECT_POINTER_CONTAINER	&objects
                    )
            {
                //if the query bounding-box don't collide with the octree bounding-box, simply return 0
                if (!query_bounding_box.Collide())
                {
                    objects.clear();
                    return 0;
                }

                //otherwise, retrieve the leaves and fill the container with the objects contained
                std::vector< NodePointer > leaves;
                unsigned int object_count;
                int					 leaves_count;

                TemplatedOctree::ContainedLeaves(query_bounding_box, leaves, TemplatedOctree::Root(), TemplatedOctree::boundingBox);
                leaves_count = int(leaves.size());
                if (leaves_count==0)
                    return 0;

                IncrementMark();
                for (int i=0; i<leaves_count; i++)
                {
                    VoxelType *voxel = TemplatedOctree::Voxel(leaves[i]);
                    int begin = voxel->begin;
                    int end		= voxel->end;
                    for ( ; begin<end; begin++)
                    {
                        ObjectReference *ref	= &sorted_dataset[begin];
                        if (IsMarked(ref))
                            continue;

                        Mark(ref);
                        objects.push_back(ref->pObject);
                    } //end of for ( ; begin<end; begin++)
                } // end of for (int i=0; i<leavesCount; i++)

                return int(objects.size());
            }; //end of GetInBox

    protected:
        /*!
        * Contains pointer to the objects in the dataset.
        * The pointers are sorted so that the object pointed to result ordered in the space
        */
        std::vector< ObjectReference >	sorted_dataset;

        /*!
        * Markers used to avoid duplication of the same result during a query
        */
        unsigned char	*marks;
        unsigned char  global_mark;

        /*!
        * The expansion factor used to solve the spatial queries
        * The current expansion factor is computed on the basis of the last expansion factor
        * and on the history of these values, through the following heuristic:
        *		current_expansion_factor = alpha*last_expansion_factor + (1.0f-alpha)*mean_expansion_factor
        * where alpha = 1.0/3.0;
        */
        //float last_expansion_factor;
        //float mean_expansion_factor;
        //float ALPHA;
        //float ONE_MINUS_ALPHA;

    protected:
        /*!
        */
        inline void IncrementMark()
        {
            // update the marks
            global_mark = (global_mark+1)%255;
            if (global_mark == 0)
            {
                memset(&marks[0], 0, sizeof(unsigned char)*int(sorted_dataset.size()));
                global_mark++;
            }
        };//end of IncrementMark

        /*
        */
        inline bool IsMarked(const ObjectReference *ref) const
        { return *ref->pMark == global_mark; };

        /*
        */
        inline void Mark(const ObjectReference *ref)
        { *ref->pMark = global_mark;};

        /*!
        * Guess an initial bounding-box from which starting the research of the closest point(s).
        * \return true iff it's possibile to find a sphere, centered in query_point and having radius max_distance at most, which collide the octree bounding-box.
        */
        inline bool GuessInitialBoundingBox(const CoordType &query_point, const ScalarType max_distance, ScalarType &sphere_radius, BoundingBoxType &query_bb)
        {
            // costruisco una bounging box centrata in query di dimensione pari a quella di una foglia.
            // e controllo se in tale bounging box sono contenute un numero di elementi >= a k.
            // Altrimenti espando il bounding box.
            query_bb.Set(query_point);

            // the radius of the sphere centered in query
            sphere_radius = 0.0f;

            // if the bounding-box doesn't intersect the bounding-box of the octree, then it must be immediately expanded
            if (!query_bb.IsIn(query_point))
            {
                do
                {
                    query_bb.Offset(TemplatedOctree::leafDiagonal);
                    sphere_radius += TemplatedOctree::leafDiagonal;
                }
                while ( !TemplatedOctree::boundingBox.Collide(query_bb) || sphere_radius>max_distance);
            }
            return (sphere_radius<=max_distance);
        };

        /*!
        * Modify the bounding-box used during the query until either at least k points
        *	are contained inside the box or the box radius became greater than the threshold distance
        * Return the number of leaves contained inside the bounding-box
        */
        inline int AdjustBoundingBox
            (
                BoundingBoxType							&	query_bb,
                ScalarType									&	sphere_radius,
                const ScalarType							max_allowed_distance,
                std::vector< NodePointer >	&	leaves,
                const int											required_object_count
            )
        {
            int leaves_count;
            int object_count;
            do
            {
                leaves.clear();

                query_bb.Offset(TemplatedOctree::leafDiagonal);
                sphere_radius+= TemplatedOctree::leafDiagonal;

                TemplatedOctree::ContainedLeaves(query_bb, leaves, TemplatedOctree::Root(), TemplatedOctree::boundingBox);

                leaves_count = int(leaves.size());
                object_count = 0;
                for (int i=0; i<leaves_count; i++)
                    object_count += TemplatedOctree::Voxel( leaves[i] )->count;
            }
            while (object_count<required_object_count && sphere_radius<max_allowed_distance);

            return leaves_count;
        }

        /*!
        * Retrieves the objects contained inside the leaves whose distance isn't greater than max_distance.
        *	Returns the number of valid objects
        */
        template < class OBJECT_POINT_DISTANCE_FUNCTOR >
        inline int RetrieveContainedObjects
            (
            const CoordType									query_point,
            OBJECT_POINT_DISTANCE_FUNCTOR	& distance_functor,
            const ScalarType								max_allowed_distance,
            bool														allow_zero_distance,
            std::vector< NodePointer	>		&	leaves,
            std::vector< Neighbour		>		&	neighbors
            )
        {
            CoordType	closest_point;
            neighbors.clear();
            for (int i=0, leaves_count=int(leaves.size()); i<leaves_count; i++)
            {
                VoxelType	*voxel	= TemplatedOctree::Voxel(leaves[i]);
                int begin					= voxel->begin;
                int end						= voxel->end;
                for ( ; begin<end; begin++)
                {
                    ObjectReference * ref	= &sorted_dataset[begin];
                    if (IsMarked(ref))
                        continue;

                    ScalarType distance = max_allowed_distance;
                    if (!distance_functor(*ref->pObject, query_point, distance, closest_point))
                        continue;

                    Mark(ref);
                    if ((distance!=ScalarType(0.0) || allow_zero_distance))
                        neighbors.push_back( Neighbour(ref->pObject, closest_point, distance) );
                } //end of for ( ; begin<end; begin++)
            } // end of for (int i=0; i<leavesCount; i++)
            return int(neighbors.size());
        };

        /*!
        * Copy the results of a query
        */
        template <class OBJECT_POINTER_CONTAINER, class DISTANCE_CONTAINER, class POINT_CONTAINER>
        inline int CopyQueryResults
        (
            std::vector< Neighbour	 > &neighbors,
            const unsigned int					object_count,
            OBJECT_POINTER_CONTAINER	 &objects,
            DISTANCE_CONTAINER				 &distances,
            POINT_CONTAINER						 &points
        )
        {
            // copy the nearest object into
            points.resize(		object_count	);
            distances.resize(	object_count	);
            objects.resize(		object_count	);

            typename POINT_CONTAINER::iterator					iPoint		= points.begin();
            typename DISTANCE_CONTAINER::iterator			  iDistance	= distances.begin();
            typename OBJECT_POINTER_CONTAINER::iterator iObject		= objects.begin();
            for (unsigned int n=0; n<object_count; n++, iPoint++, iDistance++, iObject++)
            {
                (*iPoint)		 = neighbors[n].point;
                (*iDistance) = neighbors[n].distance;
                (*iObject)	 = neighbors[n].object;
            }
            return object_count;
        }

        /*!
        * When all the leaves are indexed, this procedure is called in order to propagate the indexing information to the inner nodes
        */
        void IndexInnerNodes(NodePointer n)
        {
            assert(n!=NULL);

            VoxelPointer current_voxel = TemplatedOctree::Voxel(n);
            VoxelPointer son_voxel;
            for (int s=0; s<8; s++)
            {
                NodePointer son_index = TemplatedOctree::Son(n, s);
                if (son_index!=NULL)
                {
                    if (TemplatedOctree::Level(son_index)!=TemplatedOctree::maximumDepth)
                        IndexInnerNodes(son_index);

                    son_voxel = TemplatedOctree::Voxel(son_index);
                    current_voxel->AddRange( son_voxel );
                }
            }
        }; // end of IndexInnerNodes
    };

#ifdef __glut_h__
    /************************************************************************/
    /* Rendering                                                            */
    /************************************************************************/
protected:
    /*!
    * Structure which holds the rendering settings
    */
    struct OcreeRenderingSetting
    {
        OcreeRenderingSetting()
        {
            color						= vcg::Color4b(155, 155, 155, 255);
            isVisible				= false;
            minVisibleDepth	= 1;
            maxVisibleDepth	= 4;
        };

        int						isVisible;
        int						minVisibleDepth;
        int						maxVisibleDepth;
        vcg::Color4b	color;
    };

public:
    /*
    * Draw the octree in a valid OpenGL context according to the rendering settings
    */
    void DrawOctree(vcg::Box3f &boundingBox, NodePointer n)
    {
        char level = TemplatedOctree::Level(n);
        NodePointer son;
        if (rendering_settings.minVisibleDepth>level)
        {
            for (int s=0; s<8; s++)
                if ((son=Son(n, s))!=0)
                    DrawOctree(TemplatedOctree::SubBox(boundingBox, s), son);
        }
        else
        {
            vcg::glBoxWire(boundingBox);
            if (level<rendering_settings.maxVisibleDepth)
                for (int s=0; s<8; s++)
                    if ((son=Son(n, s))!=0)
                        DrawOctree(TemplatedOctree::SubBox(boundingBox, s), son);
        }
    };

        OcreeRenderingSetting					rendering_settings;
#endif
} //end of namespace vcg

#endif //VCG_SPACE_INDEX_OCTREE_H
