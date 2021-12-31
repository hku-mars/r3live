#ifndef VCG_HEDGE_TOPOLOGY
#define VCG_HEDGE_TOPOLOGY

#include <vcg/connectors/halfedge_pos.h>

using namespace std;
using namespace vcg::hedge;
using namespace vcg::tri;

namespace vcg
{
    namespace tri
    {

        /*!
          * \brief Class containing functions to modify the topology of a halfedge based mesh
          *
          */
        template <class MeshType> class HalfEdgeTopology
        {
            public:

                typedef typename MeshType::VertexPointer VertexPointer;
                typedef typename MeshType::EdgePointer EdgePointer;
                typedef typename MeshType::HEdgePointer HEdgePointer;
                typedef typename MeshType::FacePointer FacePointer;

                typedef typename MeshType::VertexIterator VertexIterator;
                typedef typename MeshType::EdgeIterator EdgeIterator;
                typedef typename MeshType::HEdgeIterator HEdgeIterator;
                typedef typename MeshType::FaceIterator FaceIterator;


                /*!
                  * Collpases an edge shared by two quads, generating only quads.
                  * Made by a series of a vertex rotation and a diagonal collapse.
                  *
                  * \param m Mesh
                  * \param hp hegde to be collapsed
                  * \param vp Vertex that will be rotated
                  *
                  * \return Pointer to the new vertex
                  */
                static VertexPointer edge_collapse_quad(MeshType &m, HEdgePointer hp, VertexPointer vp)
                {
                    assert(vp);
                    assert(hp);
                    assert(MeshType::HEdgeType::HasHVAdjacency());
                    assert(hp->HVp() == vp || hp->HOp()->HVp() == vp);
                    assert(hp->HFp()->VN() == 4);
                    assert(hp->HOp()->HFp()->VN() == 4);


                    VertexPointer vp_opp;
                    FacePointer fp;

                    if( hp->HVp() == vp )
                        hp = hp->HOp();

                    //retrieve opposite vertex and right face for diagonal collapse
                    vp_opp = hp->HVp();
                    fp = hp->HFp();

                    VertexPointer vp_rot = vertex_rotate_quad( vp );

                    assert(vp_rot == vp);

                    return diagonal_collapse_quad( m, fp, vp );
                }

                /*!
                  * Collpases a diagonal in a quad.
                  *
                  *
                  * \param m Mesh
                  * \param fp Face where diagonal resides
                  * \param vp One of the two vertices of the diagonal
                  *
                  * \return Pointer to the new vertex
                  */
                static VertexPointer diagonal_collapse_quad(MeshType &m, FacePointer fp, VertexPointer vp)
                {

                    assert(MeshType::VertexType::HasVHAdjacency());
                    assert(MeshType::FaceType::HasFHAdjacency());
                    assert(MeshType::HEdgeType::HasHVAdjacency());
                    assert(MeshType::HEdgeType::HasHFAdjacency());
                    assert(MeshType::HEdgeType::HasHOppAdjacency());
                    assert(MeshType::HEdgeType::HasHPrevAdjacency());

                    assert(fp);
                    assert(fp->FHp());
                    assert(fp->VN() == 4);
                    assert(!fp->IsD());

                    assert( !has_doublet_quad(fp) );
                    assert(!is_singlet_quad(fp));

                    bool HasHE = MeshType::HEdgeType::HasHEAdjacency();
                    bool HasEH = MeshType::EdgeType::HasEHAdjacency();

                    HEdgePointer hp;

                    vector<VertexPointer> vps = getVertices(fp);

                    assert(vps.size()==4);

                    for(unsigned int i = 0; i< vps.size(); i++)
                    {
                        // all vertices must be different
                        assert( count(vps.begin(), vps.end(), vps[i]) == 1 );
                    }

                    hp = fp->FHp();

                    while(hp->HVp() != vp)
                        hp= hp->HNp();

                    vector<HEdgePointer> hps = getHEdges(fp,hp);

                    assert(vp == hps[0]->HVp());

                    VertexPointer opposite_vertex = hps[2]->HVp();

                    vp->P() = (vp->P() + opposite_vertex->P() )/2;
                    change_vertex(opposite_vertex, vp);

                    hps[0]->HOp()->HOp()=hps[1]->HOp();
                    hps[1]->HOp()->HOp()=hps[0]->HOp();

                    hps[2]->HOp()->HOp()=hps[3]->HOp();
                    hps[3]->HOp()->HOp()=hps[2]->HOp();

                    for(int i=0; i<4; i++)
                    {
                        if(hps[i]->HVp()->VHp() == hps[i])
                            hps[i]->HVp()->VHp() = hps[(i+4-1)%4]->HOp();
                    }

                    if(HasHE)
                    {
                        hps[1]->HOp()->HEp()=hps[0]->HEp();
                        hps[3]->HOp()->HEp()=hps[2]->HEp();
                    }

                    if(HasEH && HasHE)
                    {
                        for(int i=0; i<3; i+=2)
                        {
                            if(hps[i]->HEp()->EHp() == hps[i])
                                hps[i]->HEp()->EHp() = hps[i]->HOp();
                        }
                    }

                    // there are no faces, remove hedges and edge
                    /*
                                      /\
                      hps[1]->HOp()  /  \  hps[0]->HOp()
                                    /    \
                              ------      ------
                              |     \    /     |
                              |      \  /      |
                              |_______\/_______|
                      */

                    bool b[2];

                    b[0] = (hps[0]->HOp()->HFp() == NULL && hps[1]->HOp()->HFp() == NULL);
                    b[1] = (hps[2]->HOp()->HFp() == NULL && hps[3]->HOp()->HFp() == NULL);

                    for( int i=0, j=0; i < 4; i+=2, j++ )
                    {
                        if(b[j])
                        {
                            if(HasHE)
                                Allocator<MeshType>::DeleteEdge(m, *(hps[i]->HEp()) );

                            Allocator<MeshType>::DeleteHEdge(m, *(hps[i]->HOp()) );
                            Allocator<MeshType>::DeleteHEdge(m, *(hps[i+1]->HOp()) );

                            hps[i+1]->HVp()->VHp() = NULL;

                            if(!b[(j+1)%2])
                            {
                                hps[i]->HOp()->HNp()->HPp() = hps[i+1]->HOp()->HPp();
                                hps[i+1]->HOp()->HPp()->HNp() = hps[i]->HOp()->HNp();

                                if(vp->VHp() == hps[i+1]->HOp())
                                    vp->VHp() = hps[(i+3)%4]->HOp();
                            }

                            else
                                vp->VHp() = NULL;

                        }
                    }


                    Allocator<MeshType>::DeleteFace(m, *(fp) );
                    Allocator<MeshType>::DeleteVertex(m, *(opposite_vertex) );
                    if(HasHE)
                    {
                        Allocator<MeshType>::DeleteEdge(m, *(hps[1]->HEp()) );
                        Allocator<MeshType>::DeleteEdge(m, *(hps[3]->HEp()) );
                    }
                    Allocator<MeshType>::DeleteHEdge(m, *(hps[0]) );
                    Allocator<MeshType>::DeleteHEdge(m, *(hps[1]) );
                    Allocator<MeshType>::DeleteHEdge(m, *(hps[2]) );
                    Allocator<MeshType>::DeleteHEdge(m, *(hps[3]) );

                    return vp;

                }

                /*!
                  * Removes a doublet merging the two quads in one
                  *
                  * \param m Mesh
                  * \param vp Vertex shared by the two consecutive edges of the doublet
                  *
                  * \return Pointer to the new face
                  */
                static FacePointer doublet_remove_quad(MeshType &m, VertexPointer vp)
                {
                    assert(vp);

                    HEdgePointer hp = vp->VHp();

                    assert(hp);

                    FacePointer fp1 = hp->HFp();
                    FacePointer fp2 = hp->HOp()->HFp();

                    assert(!is_singlet_quad(fp1));
                    assert(!is_singlet_quad(fp2));


                    assert( fp1 );
                    assert( fp2 );

                    assert( hp->HOp()->HNp()->HOp() == hp->HPp() );

                    assert( fp1->VN() == 4);
                    assert( fp2->VN() == 4);

                    // end of check

                    hp->HNp()->HPp() = hp->HOp()->HPp();
                    hp->HOp()->HPp()->HNp() =  hp->HNp();

                    hp->HPp()->HPp()->HNp() = hp->HOp()->HNp()->HNp();
                    hp->HOp()->HNp()->HNp()->HPp() = hp->HPp()->HPp();


                    if( hp->HOp()->HVp()->VHp() == hp->HOp() )
                        hp->HOp()->HVp()->VHp() = hp->HNp();

                    if( hp->HPp()->HVp()->VHp() == hp->HPp() )
                        hp->HPp()->HVp()->VHp() = hp->HPp()->HPp()->HNp();


                    hp->HNp()->HPp()->HFp() = fp1;
                    hp->HOp()->HNp()->HNp()->HFp() = fp1;

                    if(fp1->FHp() == hp || fp1->FHp() == hp->HPp())
                        fp1->FHp() = hp->HNp();


                    Allocator<MeshType>::DeleteVertex(m, *vp);
                    if(MeshType::HEdgeType::HasHEAdjacency())
                    {
                        Allocator<MeshType>::DeleteEdge(m, *(hp->HEp()) );
                        Allocator<MeshType>::DeleteEdge(m, *(hp->HPp()->HEp()) );
                    }
                    Allocator<MeshType>::DeleteHEdge(m, *hp );
                    Allocator<MeshType>::DeleteHEdge(m, *(hp->HOp()) );
                    Allocator<MeshType>::DeleteHEdge(m, *(hp->HPp()) );
                    Allocator<MeshType>::DeleteHEdge(m, *(hp->HPp()->HOp()) );
                    Allocator<MeshType>::DeleteFace(m, *fp2 );


                    return fp1;


                }


                /*!
                  * Removes a singlet replacing it with an edge
                  *
                  * \param m Mesh
                  * \param fp Face that should be a singlet quad
                  *
                  * \return Pointer to an halfdedge representing the new edge
                  */
                static HEdgePointer singlet_remove_quad(MeshType &m, FacePointer fp)
                {
                    /*
                          2
                        /    \
                       /      \
                       |       |
                       |       |
                       |   1   |
                       |  /\   |
                       \  | | /
                        \ \/ /
                          3
                      */

                    assert( is_singlet_quad(fp) );

                    bool HasHE = MeshType::HEdgeType::HasHEAdjacency();
                    bool HasEH = MeshType::EdgeType::HasEHAdjacency();

                    vector<HEdgePointer> ext_hedges;

                    vector<HEdgePointer> int_hedges = getHEdges(fp);

                    Allocator<MeshType>::DeleteFace( m, *(fp) );

                    for(typename vector<HEdgePointer>::iterator hi = int_hedges.begin(); hi != int_hedges.end();++hi)
                    {

                        if((*hi)->HOp()->HFp() != fp)
                            ext_hedges.push_back((*hi)->HOp());

                        else if(vertex_valence((*hi)->HVp()) == 1)
                        {
                            Allocator<MeshType>::DeleteVertex( m, *((*hi)->HVp()) );
                            if(HasHE)
                                Allocator<MeshType>::DeleteEdge( m, *((*hi)->HEp()) );
                        }

                    }

                    for(typename vector<HEdgePointer>::iterator hi = int_hedges.begin(); hi != int_hedges.end();++hi)
                        Allocator<MeshType>::DeleteHEdge( m, *(*hi) );

                    assert(ext_hedges.size() == 2);


                    if(ext_hedges[0]->HFp() || ext_hedges[1]->HFp())
                    {
                        ext_hedges[0]->HOp() = ext_hedges[1];
                        ext_hedges[1]->HOp() = ext_hedges[0];

                        if(HasHE)
                        {
                            Allocator<MeshType>::DeleteEdge( m, *(ext_hedges[1]->HEp()) );

                            ext_hedges[1]->HEp() = ext_hedges[0]->HEp();

                            if(HasEH)
                                ext_hedges[0]->HEp()->EHp() = ext_hedges[0];
                        }

                        ext_hedges[0]->HVp()->VHp() = ext_hedges[0];
                        ext_hedges[1]->HVp()->VHp() = ext_hedges[1];

                        return ext_hedges[0];
                    }

                    else
                    {
                        ext_hedges[0]->HVp()->VHp() = NULL;
                        ext_hedges[1]->HVp()->VHp() = NULL;

                        if(HasHE)
                        {
                            Allocator<MeshType>::DeleteEdge( m, *( ext_hedges[0]->HEp()) );
                            Allocator<MeshType>::DeleteEdge( m, *( ext_hedges[1]->HEp()) );
                        }
                        Allocator<MeshType>::DeleteHEdge( m, *( ext_hedges[0]) );
                        Allocator<MeshType>::DeleteHEdge( m, *( ext_hedges[1]) );

                        return NULL;
                    }

                }


                /*!
                  * Rotates a non-border edge shared by two quads
                  *
                  * \param hp Edge to be rotated
                  * \param cw flag denoting a clockwise or counter-clockwise rotation
                  *
                  * \return Pointer to the rotated edge
                  */
                static HEdgePointer edge_rotate_quad(HEdgePointer hp, bool cw)
                {

                    assert( MeshType::HEdgeType::HasHFAdjacency() );
                    assert( MeshType::HEdgeType::HasHOppAdjacency() );
                    assert( MeshType::FaceType::HasFHAdjacency() );


                    FacePointer fp1 = hp->HFp();
                    FacePointer fp2 = hp->HOp()->HFp();


                    assert( fp1 );
                    assert( fp1->VN() == 4 );

                    assert( fp2 );
                    assert( fp2->VN() == 4 );

                    assert(!is_singlet_quad(fp1));
                    assert(!is_singlet_quad(fp2));

                    assert(!has_doublet_quad(fp1));
                    assert(!has_doublet_quad(fp2));

                    vector<FacePointer> fps;
                    typedef vector<HEdgePointer> hedge_vect;
                    vector<hedge_vect> hps;

                    fps.push_back(fp1);
                    fps.push_back(fp2);


                    hps.push_back( getHEdges( fp1, hp ) );
                    hps.push_back( getHEdges( fp2, hp->HOp() ) );


                    for(int i=0; i< 2; i++)
                    {

                        int j = (i+1)%2;

                        // uguali sia per cw che per ccw
                        hps[i][1]->HPp() = hps[j][3];
                        hps[j][3]->HNp() = hps[i][1];

                        if(hps[j][0]->HVp()->VHp() == hps[j][0])
                            hps[j][0]->HVp()->VHp() = hps[i][1];

                        if(cw)
                        {
                            hps[i][0]->HNp() = hps[j][3];
                            hps[j][3]->HPp() = hps[i][0];

                            hps[j][2]->HNp() = hps[j][0];
                            hps[j][0]->HPp() = hps[j][2];

                            hps[j][0]->HVp() = hps[j][3]->HVp();

                            hps[j][3]->HFp() = fps[i];

                            if(fps[j]->FHp() == hps[j][3])
                                fps[j]->FHp() = hps[j][0];
                        }
                        else
                        {
                            hps[i][0]->HNp() = hps[i][2];
                            hps[i][2]->HPp() = hps[i][0];

                            hps[i][1]->HNp() = hps[j][0];
                            hps[j][0]->HPp() = hps[i][1];

                            hps[j][0]->HVp() = hps[i][2]->HVp();

                            hps[i][1]->HFp() = fps[j];

                            if(fps[i]->FHp() == hps[i][1])
                                fps[i]->FHp() = hps[i][0];
                        }

                    }

                    return hp;

                }



                /*!
                  * Rotates a non-border vertex shared by only quads
                  *
                  * \param vp Vertex to be rotated
                  *
                  * \return Pointer to the rotated vertex
                  */
                static VertexPointer vertex_rotate_quad(VertexPointer vp)
                {

                    assert(MeshType::VertexType::HasVHAdjacency());
                    assert( vp->VHp() );

                    Pos<MeshType> p(vp->VHp(), true);

                    HEdgePointer hep = p.HE();

                    typedef vector<HEdgePointer> hedge_vect;
                    vector<hedge_vect> hedges;

                    do
                    {
                        assert( p.F() );
                        assert( p.F()->VN() == 4);

                        hedges.push_back(getHEdges(p.F(), p.HE()));

                        p.FlipE();
                        p.FlipF();

                    }while(p.HE() != hep);


                    int size = hedges.size();

                    for(int i=0; i< size; i++)
                    {
                        hedges[i][0]->HNp() = hedges[i][2];
                        hedges[i][2]->HPp() = hedges[i][0];

                        assert(hedges[i][0]->HOp() == hedges[(i+size-1)%size][3]);

                        hedges[i][2]->HNp() = hedges[(i+1)%size][1];
                        hedges[(i+1)%size][1]->HPp() = hedges[i][2];

                        hedges[(i+1)%size][1]->HNp() = hedges[i][3];
                        hedges[i][3]->HPp() = hedges[(i+1)%size][1];

                        hedges[(i+1)%size][1]->HFp() = hedges[i][3]->HFp();

                        if(hedges[i][3]->HVp()->VHp() == hedges[i][3])
                            hedges[i][3]->HVp()->VHp() = hedges[(i+1)%size][1];

                        hedges[i][3]->HVp() = hedges[(i+1)%size][2]->HVp();

                        if(hedges[i][0]->HFp()->FHp() == hedges[i][1])
                            hedges[i][0]->HFp()->FHp() = hedges[i][0];

                    }
                    return vp;

                }


                /*!
                  * Collapses a generic edge
                  *
                  * \param m Mesh
                  * \param hp Edge to be collapsed
                  * \param vp Vertex to be deleted
                  *
                  * \return Pointer to the other vertex belonging to the collapsed edge
                  */
                static VertexPointer edge_collapse(MeshType &m, HEdgePointer hp, VertexPointer vp)
                {

                    assert(MeshType::VertexType::HasVHAdjacency());
                    assert(MeshType::HEdgeType::HasHOppAdjacency());
                    assert(MeshType::HEdgeType::HasHVAdjacency());
                    assert(MeshType::HEdgeType::HasHPrevAdjacency());

                    if( hp->HFp() )
                        assert(hp->HFp()->VN() > 3);

                    if( hp->HOp()->HFp())
                        assert(hp->HOp()->HFp()->VN() > 3);

                    assert(hp->HFp() || hp->HOp()->HFp());
                    assert(hp->HVp() == vp || hp->HOp()->HVp() == vp);


                    HEdgePointer hopp = hp->HOp();

                    VertexPointer vp1;

                    if( hp->HVp() == vp )
                        vp1 = hopp->HVp();
                    else
                        vp1 = hp->HVp();

                    change_vertex( vp, vp1);

                    //HP
                    hp->HNp()->HPp() = hp->HPp();
                    hopp->HNp()->HPp() = hopp->HPp();

                    //HN
                    hp->HPp()->HNp() = hp->HNp();
                    hopp->HPp()->HNp() = hopp->HNp();

                    //FH
                    if( hp->HFp() )
                        if( hp->HFp()->FHp() == hp )
                            hp->HFp()->FHp() = hp->HNp();

                    if( hopp->HFp() )
                        if( hopp->HFp()->FHp() == hopp )
                            hopp->HFp()->FHp() = hopp->HNp();

                    // VH
                    if( vp1->VHp() == hopp )
                        vp1->VHp() = hopp->HNp();

                    if(HasHEAdjacency(m))
                        Allocator<MeshType>::DeleteEdge(m,*(hp->HEp()));
                    Allocator<MeshType>::DeleteHEdge(m,*hp);
                    Allocator<MeshType>::DeleteHEdge(m,*hopp);
                    Allocator<MeshType>::DeleteVertex(m,*vp);

                    return vp1;

                }

                /*!
                  * Adds a face in a mesh, checking if the operation is possible.
                  *
                  * \param m Mesh
                  * \param vps Vector of vertices (in ccw order) that will belong to the new face
                  *
                  * \return Pointer to the new face if it has been inserted, NULL otherwise
                  */
                static FacePointer add_face(MeshType &m, vector<VertexPointer> &vps)
                {

                    assert(MeshType::VertexType::HasVHAdjacency());
                    assert(MeshType::HEdgeType::HasHVAdjacency());
                    assert(MeshType::HEdgeType::HasHFAdjacency());
                    assert(MeshType::HEdgeType::HasHOppAdjacency());
                    assert(MeshType::HEdgeType::HasHPrevAdjacency());

                    unsigned int size = vps.size();

                    assert(size >= 3); //there must be at least 3 vertices

                    for(unsigned int i = 0; i< size; i++)
                    {
                        // all vertices must be different
                        assert( count(vps.begin(), vps.end(), vps[i]) == 1 );
                    }

                    vector<HEdgePointer> hps;

                    while(hps.size() < size)
                        if( !can_add_hedge(vps, hps) )
                            return NULL;

                    vector<bool> non_manifold_vertices(size, false);

                    return add_face_unsafe( m,vps, hps, non_manifold_vertices);

                }

                /*!
                  * Removes a face in a mesh, checking if the operation is possible
                  *
                  * \param m Mesh
                  * \param fp face to be removed
                  *
                  * \retval true if face has been removed
                  * \retval false otherwise
                  */
                static bool remove_face(MeshType &m, FacePointer fp)
                {

                    assert(MeshType::VertexType::HasVHAdjacency());
                    assert(MeshType::FaceType::HasFHAdjacency());
                    assert(MeshType::HEdgeType::HasHVAdjacency());
                    assert(MeshType::HEdgeType::HasHFAdjacency());
                    assert(MeshType::HEdgeType::HasHOppAdjacency());
                    assert(MeshType::HEdgeType::HasHPrevAdjacency());

                    if( can_remove_face(fp) )
                    {
                        remove_face_unsafe(m, fp);
                        return true;
                    }

                    return false;
                }

            protected:

                /*!
                  * Adds a face in a mesh without any check
                  *
                  * \param m Mesh
                  * \param vps Vector of vertices (in ccw order) that will belong to the new face
                  *
                  * \return Pointer to the new face
                  */
                static FacePointer add_face_unsafe(MeshType &m, vector<VertexPointer> &vps)
                {
                    unsigned int size = vps.size();

                    vector<HEdgePointer> hps;
                    vector<bool> non_manifold_vertices;

                    while(hps.size() < size)
                    {
                        if( can_add_hedge(vps, hps) )
                            non_manifold_vertices.push_back( false );
                        else
                            non_manifold_vertices.push_back( hps.back() == NULL );
                    }

                    return add_face_unsafe(m,vps,hps, non_manifold_vertices);
                }


                /*!
                  * Adds a face in a mesh without any check
                  *
                  * \param m Mesh
                  * \param vps Vector of vertices (in ccw order) that will belong to the new face
                  * \param hps Vector of hedges (in ccw order) that will belong to the new face
                  * \param non_manifold_vertices Vector of booleans denoting on the i-th position if the i-th vertex is non-manifold
                  *
                  * \return Pointer to the new face
                  */
                static FacePointer add_face_unsafe(MeshType &m, vector<VertexPointer> &vps, vector<HEdgePointer> &hps, vector<bool> &non_manifold_vertices)
                {

                    assert(MeshType::VertexType::HasVHAdjacency());
                    assert(MeshType::HEdgeType::HasHVAdjacency());
                    assert(MeshType::HEdgeType::HasHFAdjacency());
                    assert(MeshType::HEdgeType::HasHOppAdjacency());
                    assert(MeshType::HEdgeType::HasHPrevAdjacency());

                    unsigned int size = vps.size();

                    assert(size >= 3); //there must be at least 3 vertices

//                    for(unsigned int i = 0; i< size; i++)
//                    {
//                        // all vertices must be different
//                        assert( count(vps.begin(), vps.end(), vps[i]) == 1 );
//                    }

                    bool HasHE = MeshType::HEdgeType::HasHEAdjacency();
                    bool HasEH = MeshType::EdgeType::HasEHAdjacency();

                    HEdgeIterator hi;

                    assert(hps.size() == size);

                    HEdgePointer nullPointer = NULL;
                    int edge_n = count(hps.begin(), hps.end(), nullPointer);

                    FacePointer fp;

                    FaceIterator fi = Allocator<MeshType>::AddFaces(m,1);
                    (*fi).Alloc( size );
                    fp = &(*fi);

                    if(edge_n > 0)
                    {

                        EdgeIterator ei;

                        fp->SetD();

                        if(HasEH || HasHE)
                        {
                            ei = Allocator<MeshType>::AddEdges(m,edge_n);
                            for(EdgeIterator ei1 = ei; ei1 != m.edge.end(); ++ei1)
                                (*ei1).SetD();
                        }

                        typename Allocator<MeshType>::template PointerUpdater<HEdgePointer> pu;

                        if(m.hedge.empty())
                            pu.oldBase = 0;
                        else
                        {
                            pu.oldBase = &*(m.hedge.begin());
                            pu.oldEnd = &m.hedge.back()+1;
                        }

                        hi = Allocator<MeshType>::AddHEdges(m,2*edge_n);

                        pu.newBase = &*(m.hedge.begin());
                        pu.newEnd = &m.hedge.back()+1;

                        //undelete face
                        fp->ClearD();

                        //undelete edges
                        for(EdgeIterator ei1 = ei; ei1 != m.edge.end(); ++ei1)
                            (*ei1).ClearD();

                        // update hedge pointers (if needed)
                        if( pu.NeedUpdate() )
                            for(typename vector<HEdgePointer>::iterator hpsi = hps.begin(); hpsi != hps.end(); ++hpsi)
                            {
                                if((*hpsi))
                                    pu.Update(*hpsi);
                            }


                        HEdgeIterator hi1 = hi;
                        HEdgeIterator hi2 = hi;

                        ++hi2;

                        EdgeIterator ei1 = ei;

                        for(; hi2 != m.hedge.end(); ++hi1, ++hi2)
                        {
                            // EH
                            if(HasEH)
                                (*ei1).EHp() = &(*hi1);

                            // HE
                            if(HasHE)
                            {
                                (*hi1).HEp() = &(*ei1);
                                (*hi2).HEp() = &(*ei1);
                            }

                            //HO
                            (*hi1).HOp() = &(*hi2);
                            (*hi2).HOp() = &(*hi1);

                            // HF
                            (*hi1).HFp() = fp;

                            ++hi1;
                            ++hi2;
                        }
                    }

                    vector<HEdgePointer> hps1;

                    for(unsigned int i = 0; i < size; i++)
                    {
                        if(hps[i] == NULL)
                        {
                                hps1.push_back(&(*hi));
                                ++hi;
                                ++hi;
                        }
                        else
                            hps1.push_back(hps[i]);
                    }

                    assert( hps1.size() == size );

                    for(unsigned int i = 0; i < size; i++)
                    {

                        int next = (i+1)%size;

                        // hedge already exisitng
                        if(hps[i])
                        {
                            hps1[i]->HFp() = fp;

                            // next hedge was disconnected
                            if(!hps[next])
                            {

                                hps1[next]->HOp()->HNp() = hps1[i]->HNp();

                                hps1[i]->HNp()->HPp() = hps1[next]->HOp();

                                hps1[i]->HNp() = hps1[next];

                                hps1[next]->HPp() = hps1[i];
                            }
                        }

                        // hedge wasn't existing, vertex was disconnected
                        else
                        {
                            //HV
                            hps1[i]->HVp() = vps[i];
                            hps1[i]->HOp()->HVp() = vps[next];


                            hps1[i]->HNp() = hps1[next];

                            // next hedge was existing (vertex was disconnected)
                            if(hps[next])
                            {
                                hps1[i]->HOp()->HPp() = hps1[next]->HPp();
                                hps1[next]->HPp()->HNp() = hps1[i]->HOp();
                            }

                            //vertex was detached
                            else
                            {
                                // after face insertion vertex will become non-manifold
                                if(non_manifold_vertices[next])
                                {
                                    Pos<MeshType> p(vps[next]->VHp(), true);

                                    while(p.F())
                                    {

                                        p.FlipE();
                                        p.FlipF();

                                        if(p.HE() == vps[next]->VHp())
                                            assert(0); //can't add a connection, there is no space
                                    }


                                    p.HE()->HPp()->HNp() = hps1[i]->HOp();
                                    hps1[i]->HOp()->HPp() = p.HE()->HPp();

                                    p.HE()->HPp() = hps1[next]->HOp();
                                    hps1[next]->HOp()->HNp() = p.HE();

                                }
                                else
                                {
                                    hps1[i]->HOp()->HPp() = hps1[next]->HOp();
                                    hps1[next]->HOp()->HNp() = hps1[i]->HOp();
                                }

                            }


                            hps1[next]->HPp() = hps1[i];

                            //VH
                            if( !vps[i]->VHp())
                                vps[i]->VHp() = hps1[i];
                        }
                    }

                    //FH
                    fp->FHp() = hps1.front();

                    return fp;

                }

                /*!
                  * Removes a face in a mesh, without any check
                  *
                  * \param m Mesh
                  * \param fp Face to be removed
                  *
                  */
                static void remove_face_unsafe (MeshType &m, FacePointer fp)
                {

                    vector<HEdgePointer> hps = getHEdges(fp);

                    int size = hps.size();

                    for( int i = 0; i< size; i++ )
                    {
                        if( hps[i]->HOp()->HFp() )
                        {
                            hps[i]->HFp() = NULL;

                            if( !hps[(i+size-1)%size]->HOp()->HFp() )
                            {
                                // HP
                                hps[i]->HPp() = hps[(i+size-1)%size]->HOp()->HPp();
                                hps[(i+size-1)%size]->HOp()->HPp()->HNp() = hps[i];
                            }

                            if( !hps[(i+1)%size]->HOp()->HFp() )
                            {
                                // HN
                                hps[i]->HNp() = hps[(i+1)%size]->HOp()->HNp();
                                hps[(i+1)%size]->HOp()->HNp()->HPp() = hps[i];
                            }
                        }
                        else
                        {
                            Allocator<MeshType>::DeleteHEdge( m, *hps[i] );
                            Allocator<MeshType>::DeleteHEdge( m, *(hps[i]->HOp()) );

                            if(MeshType::HEdgeType::HasHEAdjacency())
                                Allocator<MeshType>::DeleteEdge( m, *(hps[i]->HEp()) );

                            if( !hps[(i+size-1)%size]->HOp()->HFp() )
                            {
                                hps[i]->HOp()->HNp()->HPp() = hps[(i+size-1)%size]->HOp()->HPp();
                                hps[(i+size-1)%size]->HOp()->HPp()->HNp() = hps[i]->HOp()->HNp();
                            }

                        }

                    }

                    for( int i = 0; i< size; i++ )
                    {
                        if( hps[i]->HVp()->VHp()->IsD() )
                        {
                            if( !hps[i]->IsD() )
                                hps[i]->HVp()->VHp() = hps[i];

                            else if( !hps[(i+size-1)%size]->IsD() )
                                hps[i]->HVp()->VHp() = hps[(i+size-1)%size]->HOp();

                            else //search for a hedge (hedge can be found only if the vertex is non-manifold)
                            {
                                bool manifold = true;

                                Pos<MeshType> p(hps[i]->HVp()->VHp(), true);

                                p.HE()->SetV();

                                p.FlipE();
                                p.FlipF();

                                while( !p.HE()->IsV() )
                                {
                                    if( !p.HE()->IsD() )
                                    {
                                        manifold = false;
                                        hps[i]->HVp()->VHp() = p.HE();
                                        break;
                                    }

                                    p.FlipE();
                                    p.FlipF();
                                }

                                p.HE()->ClearV();

                                if(manifold)
                                    hps[i]->HVp()->VHp() = NULL;

                            }
                        }

                    }

                    Allocator<MeshType>::DeleteFace(m,*fp);

                }

                /*!
                  * Checks if the next hedge can be inserted into hps.
                  * If true, inserts the hedge into hps. If false, inserts NULL.
                  *
                  * \param vps Vector of vertices (in ccw order) that will belong to the new face
                  * \param hps Vector of hedges already checked
                  *
                  * \retval true if hedge can be inserted
                  * \retval false otherwise
                  */
                static bool can_add_hedge( vector<VertexPointer> &vps, vector<HEdgePointer> &hps )
                {

                    unsigned int i = hps.size();

                    assert( i < vps.size() );

                    HEdgePointer he = vps[i]->VHp();

                    if(!he) //vertex is detached
                    {
                        hps.push_back(NULL);
                        return true;
                    }
                    else
                    {
                        bool disconnected = false;

                        bool hasEdge = false;

                        unsigned int size = vps.size();

                        Pos<MeshType> p(he, false);

                        he->SetV();

                        while(p.V() != vps[(i+1)%size])
                        {
                            if(!hasEdge)
                                hasEdge= ( find( vps.begin(), vps.end(), p.V()) != (vps.end() ) );

                            p.FlipV();

                            p.FlipE();
                            p.FlipF();

                            p.FlipV();

                            if(p.HE()->IsV())
                            {
                                disconnected = true;
                                break;
                            }

                        }

                        he->ClearV();

                        if(disconnected) // edge does not exist
                        {
                            hps.push_back(NULL);

                            // if hasEdge is false after inserting the face there will be a non-manifold vertex
                            return hasEdge;
                        }

                        else //edge already existing
                        {
                            // try to insert consecutve hedges if they will belong to the new face
                            while( (p.V() ==  vps[(i+1)%size])  && (i < size) )
                            {
                                hps.push_back( p.HE() );

                                if(p.HE()->HFp() != NULL)
                                    return false;

                                i++;
                                p.FlipE();
                                p.FlipV();
                            }
                            return true;
                        }
                    }
                }

                public:
                /*!
                  * Checks if a face can be removed
                  *
                  * \param fp Face to check
                  *
                  * \retval true if the face can be removed
                  * \retval false otherwise
                  */
                static bool can_remove_face(FacePointer fp)
                {

                    assert(fp);
                    assert(!fp->IsD());

                    Pos<MeshType> p(fp->FHp(), true);

                    do
                    {
                        vector<FacePointer> incident_faces = get_incident_faces( p.V() );

                        unsigned int size = incident_faces.size();

                        if(size > 2)
                        {
                            for(unsigned int i = 0; i < size; i++)
                            {
                                if(incident_faces[i] == NULL)
                                    if(incident_faces[(i+1)%size] != fp && incident_faces[((i+size)-1)%size] != fp )
                                        return false;
                            }
                        }

                        p.FlipV();
                        p.FlipE();

                    }while( p.HE() != fp->FHp() );

                    return true;
                }

                /*!
                  * Checks if a diagonal can be collapsed
                  *
                  * \param hp Hedge whose vertex is one of the two vertices of the diagonal
                  *
                  * \retval true if diagonal can be collapsed
                  * \retval false if diagonal cannot be collapsed
                  */
                static bool check_diagonal_collapse_quad(HEdgePointer hp)
                {

                    assert(hp);
                    assert(hp->HFp());
                    assert(hp->HFp()->VN() == 4);
                    assert(!hp->IsD());

                    vector<FacePointer> faces;

                    HEdgePointer hopp = hp->HNp()->HNp();
                    vector<FacePointer> faces1 = get_incident_faces(hp->HVp(), hp);
                    vector<FacePointer> faces2 = get_incident_faces(hp->HNp()->HNp()->HVp(), hopp);

                    faces.assign(faces1.begin()+1, faces1.end());
                    faces.assign(faces2.begin()+1, faces2.end());


                    // First check:

                    unsigned int size = faces.size();
                    bool null_face = false;


                    // if size <=2 check is ok
                    if(size > 2)
                    {
                        for(unsigned int i = 0; i < size; i++)
                        {
                            if(faces[i] == NULL)
                            {
                                if(faces[(i+1)%size] != NULL && faces[((i+size)-1)%size] != NULL )
                                {
                                    if(null_face)
                                        return false;

                                    null_face=true;
                                }
                            }
                        }
                    }

                    // End of first check


                    // Second check

                    set<VertexPointer> set1;
                    set<VertexPointer> set2;

                    vector<VertexPointer> vect1 = getVertices(hp->HVp());
                    vector<VertexPointer> vect2 = getVertices(hp->HNp()->HNp()->HVp());

                    set1.insert(vect1.begin(), vect1.end());
                    set2.insert(vect2.begin(), vect2.end());

                    size = vect1.size();
                    if(vect2.size() < size)
                        size = vect2.size();

                    vector<VertexPointer> intersection(size);

                    typename vector<VertexPointer>::iterator it;
                    it = set_intersection(set1.begin(), set1.end(), set2.begin(), set2.end(), intersection.begin());

                    size = it- intersection.begin();

                    assert( size >= 2 );

                   return (size==2);

                    // End of second check

                }

                /*!
                  * Checks if a vertex is non-manifold, comparing local and global information (slow)
                  *
                  * \param m Mesh
                  * \param vp Vertex to check
                  *
                  * \retval true if vertex is non-manifold
                  * \retval false if verex is manifold
                  */
                static bool is_nonManifold_vertex(MeshType &m, VertexPointer vp)
                {
                    assert(vp);
                    assert(!vp->IsD());

                    set<HEdgePointer> set1;
                    for(HEdgeIterator hi = m.hedge.begin(); hi != m.hedge.end(); ++hi)
                    {
                        if(!(*hi).IsD() && (*hi).HVp() == vp)
                          set1.insert(&(*hi));
                    }


                    vector<HEdgePointer> vect2 = get_incident_hedges(vp);

                    set<HEdgePointer> set2;
                    set2.insert(vect2.begin(), vect2.end());

                    return !equal(set1.begin(), set1.end(), set2.begin());

                }

                /*!
                  * Checks if a vertex is non-manifold, based only on local informations
                  *
                  * \param vp Vertex to check
                  *
                  * \retval true if vertex is non-manifold
                  * \retval false if verex is manifold
                  */
                static bool is_nonManifold_vertex(VertexPointer vp)
                {
                    assert(vp);
                    assert(!vp->IsD());

                    vector<FacePointer> faces = get_incident_faces(vp);

                    unsigned int size = faces.size();
                    int null_count = 0;

                    if(size > 2)
                    {
                        for(unsigned int i = 0; i < size; i++)
                        {
                            if(faces[i] == NULL)
                            {
                                if(null_count > 0)
                                    return true;
                                else
                                    null_count++;
                            }
                        }
                    }

                    return false;

                }

                /*!
                  * Shortcut to get the second vertex of an edge
                  *
                  * \param hp Hedge
                  *
                  * \return Opposite vertex
                  */
                static VertexPointer opp_vert(HEdgePointer hp)
                {
                    return hp->HOp()->HVp();
                }

                /*!
                  * Gets vertices on the 1-ring of a vertex
                  *
                  * \param vp Vertex. It must be a non-border vertex.
                  *
                  * \return Vector containing vertices
                  */
                static vector<VertexPointer> getVertices(VertexPointer vp)
                {
                    assert(vp);
                    assert(!vp->IsD());

                    HEdgePointer hp = vp->VHp();

                    vector<VertexPointer> ret;

                    if( !hp )
                        return ret;

                    Pos<MeshType> p(hp);

                    do
                    {
                        if(p.F())
                        {
                            assert(!p.F()->IsD());

                            ret.push_back( opp_vert( p.HE() ) );

                            ret.push_back( opp_vert( p.HE()->HNp() ) );


                        }
                        p.FlipE();
                        p.FlipF();

                    }while( p.HE() != hp);

                    return ret;
                }


                /*!
                  * Gets faces on the 1-ring of a vertex
                  *
                  * \param vp Vertex
                  *
                  * \return Set containing faces
                  */
                static set<FacePointer> getFaces(VertexPointer vp)
                {
                    assert(vp);
                    assert(!vp->IsD());

                    set<FacePointer> ret;

                    vector<VertexPointer> vertices = getVertices(vp);

                    for(typename vector<VertexPointer>::iterator vi = vertices.begin(); vi!= vertices.end(); ++vi)
                    {
                        vector<FacePointer> incident_faces = get_incident_faces(*vi);
                        ret.insert(incident_faces.begin(), incident_faces.end());
                    }

                    return ret;

                }


                /*!
                  * Checks if a face is a singlet
                  *
                  * \param fp Face to check
                  *
                  * \retval true if face is a singlet
                  * \retval false if face isn't a singlet
                  */
                static bool is_singlet_quad(FacePointer fp)
                {
                    assert(fp);
                    assert(fp->FHp());
                    assert(!fp->IsD());

                    Pos<MeshType> p( fp->FHp() );

                    do
                    {
                        if( vertex_valence(p.V()) == 1 )
                            return true;

                        p.FlipV();
                        p.FlipE();

                    }while(p.HE() != fp->FHp());

                    return false;

                }

                /*!
                  * Gets all vertices incident to a face
                  *
                  * \param fp Face
                  * \param starting_he A hedge in the face from which to start
                  *
                  * \return Vector containing the incident vertices
                  */
                static vector<VertexPointer> getVertices(FacePointer fp, HEdgePointer starting_he = NULL)
                {
                    assert(fp);
                    assert(!fp->IsD());

                    if(!starting_he)
                        starting_he = fp->FHp();

                    assert( starting_he->HFp() == fp );

                    Pos<MeshType> p( starting_he, true );

                    vector<VertexPointer> ret;


                    do
                    {
                        assert(!(p.V()->IsD()));

                        ret.push_back( p.V() );

                        p.FlipV();
                        p.FlipE();

                        assert(ret.size() <= (unsigned int)(fp->VN()));

                    }while(p.HE() != starting_he);

                    return ret;

                }

            protected:
                /*!
                  * Gets all hedges incident to a face
                  *
                  * \param fp Face
                  * \param starting_he A hedge in the face from which to start
                  *
                  * \return Vector containing the incident hedges
                  */
                static vector<HEdgePointer> getHEdges(FacePointer fp, HEdgePointer starting_he = NULL)
                {
                    assert(fp);
                    assert(!fp->IsD());

                    if(starting_he)
                        assert( starting_he->HFp() == fp );
                    else
                        starting_he = fp->FHp();

                    Pos<MeshType> p( starting_he, true );

                    vector<HEdgePointer> ret;

                    do
                    {
                        ret.push_back( p.HE() );

                        p.FlipV();
                        p.FlipE();

                        assert(ret.size() <= (unsigned int) (fp->VN()));

                    }while(p.HE() != starting_he);

                    return ret;

                }

            public:

                /*!
                  * Gets all faces incident to a vertex
                  *
                  * \param vp Vertex
                  * \param starting_he A hedge from which to start
                  *
                  * \return Vector containing the incident faces
                  */
                static vector<FacePointer> get_incident_faces(VertexPointer vp, HEdgePointer starting_he = NULL)
                {
                    assert(vp);
                    assert(!vp->IsD());

                    if(starting_he)
                        assert( starting_he->HVp() == vp );
                    else
                        starting_he = vp->VHp();

                    vector<FacePointer> ret;

                    if(!starting_he)
                        return ret;

                    Pos<MeshType> p( starting_he, true );

                    do
                    {
                        ret.push_back( p.F() );

                        p.FlipE();
                        p.FlipF();

                    }while(p.HE() != starting_he);

                    return ret;

                }


                static vector<FacePointer> get_adjacent_faces(FacePointer fp)
                {
                    assert(fp);
                    assert(!fp->IsD());

                    vector<FacePointer> ret;

                    Pos<MeshType> p( fp->FHp() );
                    assert(p.F() == fp);

                    do
                    {
                        p.FlipF();
                        ret.push_back( p.F() );
                        p.FlipF();

                        p.FlipV();
                        p.FlipE();

                    } while(p.HE() != fp->FHp());

                    return ret;

                }

                /*!
                  * Gets all hedges incident to a vertex
                  *
                  * \param vp Vertex
                  * \param starting_he A hedge from which to start navigation
                  *
                  * \return Vector containing the incident hedges
                  */
                static vector<HEdgePointer> get_incident_hedges(VertexPointer vp, HEdgePointer starting_he = NULL)
                {
                    assert(vp);
                    assert(!vp->IsD());

                    if(starting_he)
                        assert( starting_he->HVp() == vp );
                    else
                        starting_he = vp->VHp();

                    vector<HEdgePointer> ret;

                    if(!starting_he)
                        return ret;

                    Pos<MeshType> p( starting_he, true );

                    do
                    {
                        assert(!p.HE()->IsD());

                        ret.push_back( p.HE() );

                        p.FlipE();
                        p.FlipF();


                    }while(p.HE() != starting_he);

                    return ret;

                }

                /*!
                  * Checks if a face has doublets
                  *
                  * \param fp Face to check
                  *
                  * \retval true if face has at least a doublet
                  * \retval false if face hasn't any doublet
                  */
                static bool has_doublet_quad(FacePointer fp)
                {
                    return ( !find_doublet_hedges_quad(fp).empty() );
                }

                /*!
                  * Gets all hedges whose vertex is into a doublet
                  *
                  * \param fp Face to check
                  *
                  * \return Vector containing the hedges
                  */
                static vector<HEdgePointer> find_doublet_hedges_quad(FacePointer fp)
                {
                    assert(fp);
                    assert(fp->FHp());
                    assert(!fp->IsD());

                    vector<HEdgePointer> ret;

                    Pos<MeshType> p( fp->FHp(), true );

                    do
                    {

                        if(vertex_valence(p.V()) == 2 && !isBorderVertex(p.V()))
                                 ret.push_back(p.HE());

                        assert(ret.size() <= 4);

                        p.FlipV();
                        p.FlipE();

                    }while(p.HE() != fp->FHp());

                    return ret;

                }

                /*!
                  * Checks if a vertex is a border vertex
                  *
                  * \param vp Vertex to check
                  *
                  * \retval true if vertex is a border vertex
                  * \retval false if vertex isn't a border vertex
                  */
                static bool isBorderVertex(VertexPointer vp)
                {
                    assert(vp);
                    assert(!vp->IsD());

                    if( !(vp->VHp()) )
                        return true;

                    Pos<MeshType> p( vp->VHp() );

                    do
                    {
                        if(!p.F())
                            return true;

                        p.FlipE();
                        p.FlipF();

                    }while(p.HE() != vp->VHp());

                    return false;
                }

                /*!
                  * Computes valence of a vertex
                  *
                  * \param vp Vertex
                  *
                  * \return Vertex valence
                  */
                static int vertex_valence(VertexPointer vp)
                {
                    assert(vp);
                    assert(!vp->IsD());

                    if( !(vp->VHp()) )
                        return 0;

                    int ret = 0;

                    Pos<MeshType> p( vp->VHp() );

                    do
                    {
                        assert(!p.HE()->IsD());
                        ret++;

                        p.FlipE();
                        p.FlipF();

                    }while(p.HE() != vp->VHp());

                    return ret;
                }

                /*!
                  * Connects to a new vertex all hedges incident to a vertex
                  *
                  * \param old_vp the old vertex to be disconnected
                  * \param new_vp the new vertex to be connected
                  *
                  */
            protected:
                static void change_vertex(VertexPointer old_vp, VertexPointer new_vp)
                {
                    assert(old_vp);
                    assert(new_vp);
                    assert(old_vp != new_vp);
                    assert(!old_vp->IsD());

                    Pos<MeshType> p(old_vp->VHp(),true);

                    p.HE()->SetV();

                    do
                    {
                        p.HE()->HVp() = new_vp;

                        p.FlipE();
                        p.FlipF();

                    }while( !p.HE()->IsV() );

                    p.HE()->ClearV();

                    if( !new_vp->VHp() )
                        new_vp->VHp() = old_vp->VHp();

                }

            };

    }
}

#endif // VCG_HEDGE_TOPOLOGY

