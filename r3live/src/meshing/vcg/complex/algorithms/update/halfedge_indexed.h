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

#ifndef __VCGLIB_HALFEDGE_
#define __VCGLIB_HALFEDGE_

#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/halfedge_topology.h>

namespace vcg
{
    namespace tri{
        /// \ingroup trimesh
        /// \brief This class is used to build edge based data structure from indexed data structure and viceversa
        /**
                */

        template <class MeshType  >
                class UpdateHalfEdges{
                public:
            typedef typename MeshType::VertexType VertexType;
            typedef typename MeshType::VertexPointer VertexPointer;
            typedef typename MeshType::VertexIterator VertexIterator;
            typedef typename MeshType::HEdgePointer HEdgePointer;
            typedef typename MeshType::HEdgeType HEdgeType;
            typedef typename MeshType::EdgePointer EdgePointer;
            typedef typename MeshType::EdgeType EdgeType;
            typedef typename MeshType::EdgeIterator EdgeIterator;
            typedef typename MeshType::HEdgeIterator HEdgeIterator;
            typedef typename MeshType::FaceIterator FaceIterator;
            typedef typename MeshType::FaceType FaceType;

            struct VertexPairEdgePtr{
                VertexPairEdgePtr(VertexPointer _v0,VertexPointer _v1,HEdgePointer _ep):v0(_v0),v1(_v1),ep(_ep){if(v0>v1) std::swap(v0,v1);}
                bool operator <(const VertexPairEdgePtr &o) const {return (v0 == o.v0)? (v1<o.v1):(v0<o.v0);}
                bool operator ==(const VertexPairEdgePtr &o) const {return (v0 == o.v0)&& (v1==o.v1);}

                VertexPointer v0,v1;
                HEdgePointer ep;
            };
            struct FacePtrInt{
                FacePtrInt ( FaceType * _f,int _i):f(_f),i(_i){}
                FaceType * f;
                int i;
            };

            typedef std::vector<bool> BitVector;

            /**
                        build a half-edge data structure from an indexed data structure. Note that the half-edges are allocated here for the first time.
                        If you have a mesh where there are already edges, they will be removed and the data lost, so do not use this function
                        to just "update" the topology of half edges.
                        **/
            static void FromIndexed(MeshType & m){
                assert(HasFVAdjacency(m));
                assert(HasHOppAdjacency(m));
                assert(HasHNextAdjacency(m));

                typename MeshType::template PerFaceAttributeHandle<BitVector> flagVisited =
                        vcg::tri::Allocator<MeshType>::template AddPerFaceAttribute<BitVector>(m,"");
                std::vector<FacePtrInt > borderEdges;

                // allocate all new half edges
                FaceIterator fi;
                unsigned int n_edges = 0;

                // count how many half edge to allocate
                for(fi = m.face.begin(); fi != m.face.end(); ++fi) if(! (*fi).IsD())
                {n_edges+=(*fi).VN();
                    for(int i = 0; i < (*fi).VN(); ++i)
                        if(vcg::face::IsBorder<FaceType>((*fi),(i)))
                            ++n_edges;
                }

                m.hedge.clear();
                m.hn = 0;

                // allocate the half edges
                typename MeshType::HEdgeIterator ei = vcg::tri::Allocator<MeshType>::AddHEdges(m,n_edges);


                std::vector<VertexPairEdgePtr> all;
                int firstEdge = 0;
                for(fi = m.face.begin(); fi != m.face.end(); ++fi)if(!(*fi).IsD()){
                    assert((*fi).VN()>2);
                    if(flagVisited[*fi].empty()) {flagVisited[*fi].resize((*fi).VN());}

                    for(int i  = 0; i < (*fi).VN(); ++i,++ei)
                    {
                        (*ei).HVp() = (*fi).V(i);
                        (*ei).HNp() = &m.hedge[firstEdge + (i +1) % (*fi).VN()];
                        if(MeshType::HEdgeType::HasHFAdjacency())
                            (*ei).HFp() = &(*fi);
                        if( MeshType::FaceType::HasFHAdjacency())
                            (*fi).FHp() = &(*ei);
                        if(MeshType::HEdgeType::HasHPrevAdjacency())
                            (*ei).HPp()	= &m.hedge[firstEdge + (i +(*fi).VN()-1) % (*fi).VN()];
                        if(HasVHAdjacency(m))
                            (*ei).HVp()->VHp() = &(*ei);
                        all.push_back(VertexPairEdgePtr((*fi).V(i), (*fi).V((*fi).Next(i)),&(*ei)));// it will be used to link the hedges

                        if(  vcg::face::IsBorder<FaceType>((*fi),(i)))
                            borderEdges.push_back(FacePtrInt(&(*fi),i));
                    }
                    firstEdge += (*fi).VN();
                }

                // add all the border hedges
                int borderLength;
                typename std::vector<FacePtrInt >::iterator ebi;
                for( ebi = borderEdges.begin(); ebi != borderEdges.end(); ++ebi)
                    if( !flagVisited[(*ebi).f][(*ebi).i])// not already inserted
                    {

                        borderLength = 0;
                        vcg::face::Pos<FaceType> bp((*ebi).f,(*ebi).i);

                        //FaceType * start = (*ebi).f;
                        VertexType * start = ((*ebi).f)->V((*ebi).i);
                        do{
                            all.push_back( VertexPairEdgePtr ( bp.f->V( bp.f->Next(bp.z) ),bp.f->V( bp.z ),&(*ei)));
                            (*ei).HVp()	=  bp.f->V(bp.f->Next(bp.z)) ;
                            flagVisited[bp.f][bp.z] = true;
                            ++ei;
                            bp.NextB();
                            ++borderLength;
                        }while (bp.v != start);
                        //}while (bp.f != start);


                        // run over the border edges to link the adjacencies
                        for(int be = 0; be < borderLength; ++be)
                        {
                            if(MeshType::HEdgeType::HasHFAdjacency())
                                m.hedge[firstEdge + be].HFp() = NULL;

                            if(MeshType::HEdgeType::HasHPrevAdjacency())
                                m.hedge[firstEdge + be].HPp() = &m.hedge[firstEdge + (be +borderLength-1) % borderLength];

                            m.hedge[firstEdge + be].HNp() = &m.hedge[firstEdge + (be +1) % borderLength];
                        }
                        firstEdge+=borderLength;
                    }

                vcg::tri::Allocator<MeshType>:: template DeletePerFaceAttribute<BitVector>(m,flagVisited );

                std::sort(all.begin(),all.end());
                assert(all.size() == n_edges);

                for(unsigned int i = 0 ; i < all.size(); )
                    if(all[i]  == all[i+1])
                    {
                        all[i].ep->HOp()	= all[i+1].ep;
                        all[i+1].ep->HOp() = all[i].ep;
                        i+=2;
                    }
                    else
                    {
                        all[i].ep->HOp() = all[i].ep;
                        i+=1;
                    }

                if(HasEHAdjacency(m) && HasHEAdjacency(m))
                {
                    assert(m.edge.size() == 0 || m.edge.size() == n_edges/2);

                    if ( m.edge.size() == 0 )
                    {
                        m.en = 0;
                        // allocate the edges
                        typename MeshType::EdgeIterator edge_i = vcg::tri::Allocator<MeshType>::AddEdges(m,n_edges/2);

                        for(ei = m.hedge.begin(); ei != m.hedge.end(); ++ei)
                        {
                            if((*ei).HEp() == NULL)
                            {
                                (*ei).HEp() = &(*edge_i);
                                (*ei).HOp()->HEp() = &(*edge_i);

                                (*edge_i).EHp() = &(*ei);

                                ++edge_i;
                            }
                        }

                    }
                    else
                    {

                        if(HasEVAdjacency(m) && HasHEAdjacency(m) && HasEHAdjacency(m))
                        {
                            //update edge relations

                            for(typename MeshType::EdgeIterator ei1 = m.edge.begin(); ei1 != m.edge.end(); ++ei1 )
                            {
                                vector<HEdgePointer> hedges = HalfEdgeTopology<MeshType>::get_incident_hedges((*ei1).V(0));

                                for(typename vector<HEdgePointer>::iterator hi = hedges.begin(); hi != hedges.end(); ++hi)
                                {
                                    if((*hi)->HOp()->HVp() == (*ei1).V(1))
                                    {

                                        assert((*hi)->HEp() == NULL);
                                        assert((*hi)->HOp()->HEp() == NULL);

                                        // EH
                                        (*ei1).EHp() = *hi;

                                        // HE
                                        (*hi)->HEp() = &(*ei1);
                                        (*hi)->HOp()->HEp() = &(*ei1);

                                        break;
                                    }
                                }
                            }
                        }
                    }

                }

            }

            /**
        Checks pointers FHEp() are valid
        **/
            static bool CheckConsistency_FHp(MeshType &  m){
                assert(MeshType::FaceType::HasFHAdjacency());
                FaceIterator fi;
                for(fi = m.face.begin(); fi != m.face.end(); ++fi)
                    if(!(*fi).IsD()){
                    if((*fi).FHp() <  &(*m.hedge.begin())) return false;
                    if((*fi).FHp() >  &(m.hedge.back())) return false;
                }
                return true;
            }

            /**
        Checks that half edges and face relation are consistent
        **/
            static bool CheckConsistency(MeshType & m){
                assert(MeshType::HEdgeType::HasHNextAdjacency());
                assert(MeshType::HEdgeType::HasHOppAdjacency());
                assert(MeshType::HEdgeType::HasHVAdjacency());
                assert(MeshType::FaceType::HasFHAdjacency());

                //bool hasHEF = ( MeshType::HEdgeType::HasHFAdjacency());
                bool hasHP = ( MeshType::HEdgeType::HasHPrevAdjacency());

                FaceIterator fi;
                HEdgePointer ep,ep1;
                int cnt = 0;

                if( MeshType::HEdgeType::HasHFAdjacency() )
                {
                    int iDb = 0;
                    for(fi = m.face.begin(); fi != m.face.end(); ++fi,++iDb)
                        if(!(*fi).IsD())
                        {
                        ep = ep1 = (*fi).FHp();

                        do{
                            if(ep->IsD())
                                return false; // the hedge should not be connected, it has been deleted
                            if( ! ep->HFp())
                                return false;
                            if(ep->HFp() != &(*fi))
                                return false;// hedge is not pointing to the rigth face
                            ep = ep->HNp();
                            if(cnt++ > m.hn)
                                return false; // hedges are ill connected (HENp())

                        }while(ep!=ep1);

                    }
                }

                HEdgePointer epPrev;
                HEdgeIterator hi;
                //bool extEdge ;
                for( hi  = m.hedge.begin(); hi != m.hedge.end(); ++hi)
                    if(!(*hi).IsD())
                    {
                        //cnt = 0;
                        epPrev = ep = ep1 = &(*hi);
                        //do{
                        //extEdge = (ep->HFp()==NULL);
                        if(hasHP)
                        {
                            if( !ep->HPp())
                                return false;
                            if( ep->HPp() == ep)
                                return false; // the previous of an edge cannot be the edge itself
                            if( ep->HNp()->HPp() != ep)
                                return false; // next and prev relation are not mutual
                            if( ep->HPp()->IsD())
                                return false; //
                        }

                        if( ! ep->HOp() )
                            return false;

                        if( ep->HOp()  == ep)
                            return false; // opposite relation is not mutual

                        if( ep->HOp()->IsD())
                            return false;

                        if( ep->HOp()->HOp() != ep)
                            return false; // opposite relation is not mutual

                        if( HasHFAdjacency(m) )
                        {
                            if(ep->HFp())
                            {
                                if( ep->HFp()->IsD())
                                    return false; // pointed face must not be deleted
                            }
                        }

                                                if( HasHEAdjacency(m) && (m.en!=0))
                        {
                            if( ! ep->HEp())
                                return false; //halfedge must point to an edge

                            if( ep->HEp()->IsD())
                                return false; // pointed edge must not be deleted

                            if(ep->HEp() != ep->HOp()->HEp())
                                return false;   // he and opposite he must point to the same edge

                            if(ep->HEp()->EHp() != ep && ep->HEp()->EHp() != ep->HOp() )
                                return false;   // halfedge points to an edge not pointing it or its opposite

                        }


                        if( !ep->HNp() )
                            return false;

                        if( ep->HNp() == ep )
                            return false; //  the next of an hedge cannot be the hedge itself

                        if( ep->HNp()->IsD())
                            return false; //

                                                if(hasHP)
                        if( ep->HNp()->HPp() != ep)
                            return false; //

                        if( HasHVAdjacency(m) )
                        {
                            if( ! ep->HVp() )
                                return false; // halfedge must point to a vertex

                            if( ep->HVp()->IsD() )
                                return false; // pointed vertex must not be deleted

                            if( HasVHAdjacency(m) )
                                if( ! (ep->HVp()->VHp()) )
                                    return false; //  halfedge points to a vertex pointing NULL

                        }


                        ep = ep->HNp();
                        if( ep->HVp() != epPrev->HOp()->HVp())
                            return false;

                        epPrev = ep;

                        // if(cnt++ > m.hn)
                        //     return false; // edges are ill connected (HENp())

                        //}while(ep!=ep1);
                    }

                if(HasEHAdjacency(m) && HasHEAdjacency(m))
                    for(EdgeIterator ei  = m.edge.begin(); ei != m.edge.end(); ++ei)
                    {
                        if(!(*ei).IsD())
                        {
                            if( !(*ei).EHp())
                                return false;   //edge must have a valid pointer to his halfedge

                            if( (*ei).EHp()->HEp() !=  &(*ei) )
                                return false; // edge's halfedge must point to the edge itself

                            if( (*ei).EHp()->IsD())
                                return false;
                        }
                    }

                if(HasVHAdjacency(m))
                    for(VertexIterator vi  = m.vert.begin(); vi != m.vert.end(); ++vi)
                    {
                        if( !(*vi).IsD() )
                            if( (*vi).VHp() )
                            {
                                if( (*vi).VHp()->HVp() !=  &(*vi) )
                                    return false;
                                if( (*vi).VHp()->IsD())
                                    return false;
                            }
                    }


                return true;
            }

            /** Set the relations HFp(), FHp() from a loop of edges to a face
        */
        private:
            static void SetRelationsLoopFace(HEdgeType * e0, FaceType * f){
                assert(HEdgeType::HasHNextAdjacency());
                assert(FaceType::HasFHAdjacency());

                HEdgeType *e = e0;
                assert(e!=NULL);
                do{ e->HFp() = f; e = e->HNp(); } while(e != e0);
                f->FHp() = e0;
            }

            /**
        Merge the two faces. This will probably become a class template or a functor
        */
            static void MergeFaces(FaceType *, FaceType *){}

            /**
        Find previous hedge in the loop
        */
            static HEdgeType *  PreviousEdge(HEdgeType * e0){
                HEdgeType *  ep = e0;
                do{
                    if(ep->HNp() == e0) return ep;
                    ep = ep->HNp();
                }while(ep!=e0);
                assert(0); // degenerate loop
                return 0;
            }

        public:
            /** Adds an edge between the sources of e0 and e1 and set all the topology relations.
        If the edges store the pointers to the faces then a new face is created.
    <--- e1 ---- X <------e1_HEPp---
                 ^
                 ||
             ei0 || ei1
                 ||
                  v
         ----e0_HEPp-> X ----- e0 ------>
        */
            static void AddHEdge(MeshType &m, HEdgeType * e0, HEdgeType * e1){
                assert(e1!=e0->HNp());
                assert(e0!=e1->HNp());
                bool hasP =  MeshType::HEdgeType::HasHPrevAdjacency();
                assert(e0->HOp() != e1); // the hedge already exists
                assert(e0!=e1->HNp());

                std::vector<typename MeshType::HEdgePointer* > toUpdate;
                toUpdate.push_back(&e0);
                toUpdate.push_back(&e1);
                HEdgeIterator ei0  = vcg::tri::Allocator<MeshType>::AddHEdges(m,2,toUpdate);

                HEdgeIterator ei1 = ei0; ++ei1;
                (*ei0).HNp() = e1;(*ei0).HVp() = e0->HVp();
                (*ei1).HNp() = e0;(*ei1).HVp() = e1->HVp();

                HEdgePointer e0_HEPp = 0,e1_HEPp = 0,ep =0;
                if(hasP){
                    e0_HEPp = e0->HPp();
                    e1_HEPp = e1->HPp();
                }else{// does not have pointer to previous, it must be computed
                    ep = e0;
                    do{
                        if(ep->HNp() == e0) e0_HEPp = ep;
                        if(ep->HNp() == e1) e1_HEPp = ep;
                        ep = ep->HNp();
                    }while(ep!=e0);
                }
                if(hasP){
                    (*ei0).HPp() = e0->HPp();
                    (*ei1).HPp() = e1->HPp();
                    e0->HPp() = &(*ei1);
                    e1->HPp() = &(*ei0);
                }
                e0_HEPp -> HNp() = &(*ei0);
                e1_HEPp -> HNp() = &(*ei1);

                (*ei0).HOp() = &(*ei1);
                (*ei1).HOp() = &(*ei0);


                if( HEdgeType::HasHFAdjacency() && FaceType::HasFHAdjacency()){
                    FaceIterator fi0  = vcg::tri::Allocator<MeshType>::AddFaces(m,1);
                    m.face.back().ImportData(*e0->HFp());

                    SetRelationsLoopFace(&(*ei0),e1->HFp());		// one loop to the old face
                    SetRelationsLoopFace(&(*ei1),&m.face.back());	// the other  to the new face
                }
            }

            /** Detach the topology relations of a given edge
    <--- e->HENPp -X --- <---------eO_HEPp---
                   ^
                   ||
               e   || e->HEOp()
                   ||
                    v
         ----e_HEPp--> X ----- e->HEOp->HENPp() ------>

        */
            static void RemoveHEdge(MeshType &m, HEdgeType * e){
                assert(MeshType::HEdgeType::HasHNextAdjacency());
                assert(MeshType::HEdgeType::HasHOppAdjacency());
                assert(MeshType::FaceType::HasFHAdjacency());

                bool hasP =  MeshType::HEdgeType::HasHPrevAdjacency();
                HEdgePointer e_HEPp,eO_HEPp;

                if(hasP){
                    e_HEPp = e->HPp();
                    eO_HEPp = e->HOp()->HPp();
                }else{
                    e_HEPp = PreviousEdge(e);
                    eO_HEPp = PreviousEdge(e->HOp());
                }

                assert(e_HEPp->HNp() == e);
                assert(eO_HEPp->HNp() == e->HOp());
                e_HEPp->HNp() = e->HOp()->HNp();
                eO_HEPp->HNp() = e-> HNp();

                if(hasP) {
                    e->HOp()->HNp()->HPp() = e_HEPp;
                    e->HNp()->HPp() = eO_HEPp;

                    e->HPp() = NULL;
                    e-> HOp()->HPp() = NULL;
                }


                // take care of the faces
                if(MeshType::HEdgeType::HasHFAdjacency()){
                    MergeFaces(e_HEPp->HFp(),eO_HEPp->HFp());
                    vcg::tri::Allocator<MeshType>::DeleteFace(m,*eO_HEPp->HFp());
                    SetRelationsLoopFace(e_HEPp,e_HEPp->HFp());

                }
                vcg::tri::Allocator<MeshType>::DeleteHEdge(m,*e->HOp());
                vcg::tri::Allocator<MeshType>::DeleteHEdge(m,*e);

            }

        };// end class
        template <class MeshType  >
                struct UpdateIndexed{
            typedef typename MeshType::VertexType VertexType;
            typedef typename MeshType::VertexPointer VertexPointer;
            typedef typename MeshType::HEdgePointer HEdgePointer;
            typedef typename MeshType::HEdgeType HEdgeType;
            typedef typename MeshType::HEdgeIterator HEdgeIterator;
            typedef typename MeshType::FaceIterator FaceIterator;
            typedef typename MeshType::FaceType FaceType;

            struct VertexPairEdgePtr{
                VertexPairEdgePtr(VertexPointer _v0,VertexPointer _v1,HEdgePointer _ep):v0(_v0),v1(_v1),ep(_ep){if(v0>v1) std::swap(v0,v1);}
                bool operator <(const VertexPairEdgePtr &o) const {return (v0 == o.v0)? (v1<o.v1):(v0<o.v0);}
                bool operator ==(const VertexPairEdgePtr &o) const {return (v0 == o.v0)&& (v1==o.v1);}

                VertexPointer v0,v1;
                HEdgePointer ep;
            };

            /**
                        builds an indexed data structure from a half-edge data structure.
                        Note: if  the half edge have the pointer to face
                        their relation FV (face-vertex) will be computed and the data possibly stored in the
                        face will be preserved.
                        **/
            static void FromHalfEdges(  MeshType & m ){
                assert(HasFVAdjacency(m));
                assert(MeshType::HEdgeType::HasHNextAdjacency());
                assert(MeshType::HEdgeType::HasHVAdjacency());
                assert(MeshType::HEdgeType::HasHOppAdjacency());
                assert(MeshType::FaceType::HasFHAdjacency());
                bool hasHEF;
                //bool createFace,hasHEF,hasFHE;

                //				typename MeshType::template PerHEdgeAttributeHandle<bool> hV = Allocator<MeshType>::template AddPerHEdgeAttribute<bool>(m,"");


                typename MeshType::HEdgeIterator ei;
                typename MeshType::FacePointer fp;
                typename MeshType::FaceIterator fi;
                typename MeshType::HEdgePointer ep,epF;
                //int vi = 0;
                vcg::SimpleTempData<typename MeshType::HEdgeContainer,bool> hV(m.hedge);

                hasHEF = (MeshType::HEdgeType::HasHFAdjacency());
                assert( !hasHEF || (hasHEF && m.fn>0));

                // if the edgetype has the pointer to face
                // it is assumed the the edget2face pointer (HEFp) are correct
                // and the faces are allocated
                for ( ei = m.hedge.begin(); ei != m.hedge.end(); ++ei)
                    if(!(*ei).IsD())								// it has not been deleted
                        if(!hasHEF || ( hasHEF &&  (*ei).HFp()!=NULL))	// if it has a pointer to the face it is
                            // not null (i.e. it is not a border edge)
                            if(!hV[(*ei)] )									// it has not be visited yet
                            {
                    if(!hasHEF)// if it has
                        fp = &(* Allocator<MeshType>::AddFaces(m,1));
                    else
                        fp = (*ei).HFp();

                    ep = epF = &(*ei);
                    std::vector<VertexPointer> vpts;
                    do{vpts.push_back((*ep).HVp()); ep=ep->HNp();}while(ep!=epF);
                    //int idbg  =fp->VN();
                    if(size_t(fp->VN()) != vpts.size()){
                        fp->Dealloc();
                        fp ->Alloc(vpts.size());
                    }
                    //int idbg1  =fp->VN();
                    for(size_t i  = 0; i < vpts.size();++i) fp ->V(i) = vpts[i];// set the pointer from face to vertex

                    hV[(*ei)] = true;
                }
                //Allocator<MeshType>::DeletePerHEdgeAttribute(m,hV);
            }

        };
    } // end namespace vcg
}
#endif // __VCGLIB_EDGE_SUPPORT
