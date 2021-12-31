#ifndef VCG_USED_TYPES_H
#define VCG_USED_TYPES_H

#include <vcg/space/point3.h>
#include <vcg/space/box3.h>
#include <vcg/space/color4.h>
#include <vcg/math/shot.h>
#include <vcg/space/texcoord2.h>
#include <vcg/space/triangle3.h>

#include <vcg/container/derivation_chain.h>
#include <vcg/complex/all_types.h>
#include <vcg/simplex/vertex/component.h>
#include <vcg/simplex/vertex/component_ocf.h>
#include <vcg/simplex/vertex/base.h>
#include <vcg/simplex/face/component.h>
#include <vcg/simplex/face/component_ocf.h>
#include <vcg/simplex/face/component_polygon.h>
#include <vcg/simplex/face/base.h>
#include <vcg/simplex/edge/component.h>
#include <vcg/simplex/edge/base.h>
#include <vcg/connectors/hedge_component.h>
#include <vcg/connectors/hedge.h>

namespace vcg{

// dummy mesh

struct _Vertex;
struct _Edge  ;
struct _Face  ;
struct _HEdge ;

struct DummyTypes{
        typedef _Vertex VertexType; 		// simplex types
        typedef _Edge EdgeType;
        typedef _Face FaceType;
        typedef char TetraType;
        typedef _HEdge HEdgeType; 		// connector types

        typedef vcg::Point3<bool> CoordType;
        typedef char ScalarType;

        typedef VertexType * VertexPointer;
        typedef EdgeType *	EdgePointer		;
        typedef FaceType * FacePointer		;
        typedef TetraType * TetraPointer	;
        typedef HEdgeType * HEdgePointer	;

    static void Name(std::vector<std::string> & /*name*/){}
        template < class LeftV>
        void ImportData(const LeftV  & /*left*/ ) {}
};

template <class A>
    struct Use{
        template <class T> struct AsVertexType: public T{typedef A VertexType;	typedef VertexType * VertexPointer	;};
        template <class T> struct AsEdgeType: public T{typedef A EdgeType;			typedef EdgeType *	EdgePointer			;};
        template <class T> struct AsFaceType: public T{typedef A FaceType;			typedef FaceType * FacePointer			;};
        template <class T> struct AsTetraType: public T{typedef A TetraType;		typedef TetraType * TetraPointer		;};
        template <class T> struct AsHEdgeType: public T{typedef A HEdgeType;		typedef HEdgeType * HEdgePointer		;};
};

template <template <typename> class A = DefaultDeriver, template <typename> class B = DefaultDeriver,
                    template <typename> class C = DefaultDeriver, template <typename> class D = DefaultDeriver,
                    template <typename> class E = DefaultDeriver, template <typename> class F = DefaultDeriver,
                    template <typename> class G = DefaultDeriver, template <typename> class H = DefaultDeriver >
                    class UsedTypes: public Arity12<DummyTypes,
                                Use<  Vertex	<UsedTypes< A, B, C, D , E, F, G, H > > > :: template AsVertexType,
                                Use<  Edge		<UsedTypes< A, B, C, D , E, F, G, H > > > :: template AsEdgeType,
                                Use<  Face		<UsedTypes< A, B, C, D , E, F, G, H > > > :: template AsFaceType,
                                Use<  HEdge	  <UsedTypes< A, B, C, D , E, F, G, H > > > :: template AsHEdgeType,
                            A, B, C, D, E, F, G, H
                    >  {
};





struct _UsedTypes: public UsedTypes<
    Use<_Vertex>::AsVertexType,
    Use<_Edge  >::AsEdgeType,
    Use<_Face  >::AsFaceType,
    Use<_HEdge >::AsHEdgeType
>{};

struct _Vertex: public  Vertex<_UsedTypes>{};
struct _Edge  : public  Edge<_UsedTypes>{};
struct _Face  : public  Face<_UsedTypes>{};
struct _HEdge : public  HEdge<_UsedTypes>{};

};

#endif // USED_TYPES_H
