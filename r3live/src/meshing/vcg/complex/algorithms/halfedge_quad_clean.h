#ifndef HALFEDGEQUADCLEAN_H
#define HALFEDGEQUADCLEAN_H

#include <vcg/complex/algorithms/update/halfedge_topology.h>
#include <queue>
#include <set>
#include<vcg/math/base.h>
#include<valarray>
#include<cmath>


namespace vcg
{
    namespace tri
    {
        /*!
          * \brief The class provides methods for detecting doublets and singlets and removing them
          *
          */

        template<class MeshType> class HalfedgeQuadClean
        {

        protected:

            typedef typename MeshType::VertexPointer VertexPointer;
            typedef typename MeshType::EdgePointer EdgePointer;
            typedef typename MeshType::HEdgePointer HEdgePointer;
            typedef typename MeshType::FacePointer FacePointer;

            typedef typename MeshType::VertexIterator VertexIterator;
            typedef typename MeshType::EdgeIterator EdgeIterator;
            typedef typename MeshType::HEdgeIterator HEdgeIterator;
            typedef typename MeshType::FaceIterator FaceIterator;

            /*! Adds to a queue all faces on the 1-ring of a given vertex
              *
              * \param q Queue of faces
              * \param vp Pointer to the vertex
              *
              */
            static void add_faces(queue<FacePointer> &q, VertexPointer vp)
            {
                vector<FacePointer> faces = HalfEdgeTopology<MeshType>::get_incident_faces(vp);

                for(typename vector<FacePointer>::iterator fi = faces.begin(); fi != faces.end(); ++fi)
                {
                    if(*fi)
                        q.push(*fi);
                }
            }

            /*! Removes doublets from all the faces into a queue until the queue empties
              *
              * \param m Mesh
              * \param faces Set of all faces modified by the removals
              * \param q Queue of faces to clean
              *
              */
            static void remove_doublets(MeshType &m, set<FacePointer> &faces, queue<FacePointer> &q)
            {

                while(!q.empty())
                {
                    FacePointer fp = q.front();
                    q.pop();

                    if( !fp->IsD() )
                    {
                        faces.insert(remove_doublet(m,fp, &q));
                    }
                }
            }

            /*! Removes a doublet and all the other ones that the removal may have generated
              *
              * \param m Mesh
              * \param fp Pointer to the face to clean from doublets
              * \param Queue of faces to check for new doublets
              *
              * \return The new face generated after doublet removal
              */
            static FacePointer remove_doublet(MeshType &m, FacePointer fp, queue<FacePointer> *q = NULL)
            {
                vector<HEdgePointer> hedges = HalfEdgeTopology<MeshType>::find_doublet_hedges_quad(fp);

                assert(hedges.size() <= 4);

                switch(hedges.size())
                {
                    // No doublets
                    case 0:
                        return NULL;

                    // A single doublet
                    case 1:
                        if(q)
                        {
                            add_faces(*q, hedges[0]->HNp()->HVp());

                            add_faces(*q, hedges[0]->HPp()->HVp());
                        }

                        return HalfEdgeTopology<MeshType>::doublet_remove_quad(m, hedges[0]->HVp());

                    // Two doublets on the same face
                    case 2:
                        {

                            if(q)
                            {
                                add_faces(*q, hedges[0]->HNp()->HVp());

                                add_faces(*q, hedges[0]->HPp()->HVp());
                            }

                           FacePointer fp1 = HalfEdgeTopology<MeshType>::doublet_remove_quad(m, hedges[0]->HVp());

                           // Removal of the doublet may generate a singlet
                           if(HalfEdgeTopology<MeshType>::is_singlet_quad(fp1))
                           {

                                HEdgePointer hp = HalfEdgeTopology<MeshType>::singlet_remove_quad(m, fp1);

                                if(hp)
                                {
                                    if(q)
                                    {
                                        if(hp->HFp())
                                            q->push(hp->HFp());
                                        if(hp->HOp()->HFp())
                                            q->push(hp->HOp()->HFp());
                                    }

                                    int valence1, valence2;

                                    valence1 = HalfEdgeTopology<MeshType>::vertex_valence(hp->HVp());
                                    valence2 = HalfEdgeTopology<MeshType>::vertex_valence(hp->HOp()->HVp());

                                    // Check if the removal of the singlet generated other singlets, then iteratively remove them
                                    while(valence1 == 1 || valence2 == 1)
                                    {

                                        assert(! (valence1 == 1 && valence2 == 1));

                                        FacePointer singlet_pointer;

                                        if(valence1 == 1 )
                                            singlet_pointer = hp->HFp();
                                        else
                                            singlet_pointer = hp->HOp()->HFp();

                                        hp = HalfEdgeTopology<MeshType>::singlet_remove_quad(m, singlet_pointer);

                                        if(!hp)
                                            break;

                                        if(q)
                                        {
                                            if(hp->HFp())
                                                q->push(hp->HFp());
                                            if(hp->HOp()->HFp())
                                                q->push(hp->HOp()->HFp());
                                        }

                                        valence1 = HalfEdgeTopology<MeshType>::vertex_valence(hp->HVp());
                                        valence2 = HalfEdgeTopology<MeshType>::vertex_valence(hp->HOp()->HVp());

                                    }
                                }
                           }

                           return fp1;
                       }

                     // Four doublets: simply remove one of the two faces
                    case 4:
                        HalfEdgeTopology<MeshType>::remove_face(m,fp->FHp()->HOp()->HFp());
                        return fp;

                    default:
                        assert(0);
                  }
            }




        public:

            /*! Removes doublets from all the faces on the 1-ring of the given vertices
              *
              * \param m Mesh
              * \param Set of all faces modified by the removals
              * \param vertices Vector of vertices that will be
              *
              */
            static void remove_doublets(MeshType &m, set<FacePointer> &faces, vector<VertexPointer> vertices)
            {
                queue<FacePointer> q;

                for(typename vector<VertexPointer>::iterator vi = vertices.begin(); vi != vertices.end(); ++vi)
                {
                    vector<FacePointer> inc_faces = HalfEdgeTopology<MeshType>::get_incident_faces(*vi);

                    for(typename vector<FacePointer>::iterator fi = inc_faces.begin(); fi != inc_faces.end(); ++fi)
                    {
                        if(*fi)
                            if( !((*fi)->IsD()) )
                                q.push(*fi);
                    }
                }

                remove_doublets(m, faces, q);
            }

            /*! Removes doublets from all the faces on the 1-ring of the given vertices
              *
              * \param m Mesh
              * \param vertices Vector of vertices that will be
              *
              */
            static void remove_doublets(MeshType &m, vector<VertexPointer> vertices)
            {
                set<FacePointer> faces;

                remove_doublets(m,faces,vertices);
            }

            /*! Removes doublets from a set of faces
              *
              * \param m Mesh
              * \param set of faces to clean
              *
              */
            static void remove_doublets(MeshType &m, set<FacePointer> &faces)
            {

                queue<FacePointer> q;

                for(typename set<FacePointer>::iterator fi = faces.begin(); fi != faces.end(); ++fi)
                {
                    if(*fi)
                        if( !((*fi)->IsD()) )
                            q.push(*fi);
                }

                remove_doublets(m, faces, q);

            }


            /*! Removes all doublets from a mesh
              *
              * \param m Mesh
              *
              * \return Number of doublets removed
              */
            static int remove_doublets(MeshType &m)
            {
                int count;
                int removed = 0;
                do
                {
                    count = 0;
                    for(FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
                    {
                        if( !((*fi).IsD()) )
                        {
                            if(remove_doublet(m, &(*fi)))
                                count++;
                        }
                    }

                    removed += count;

                }while(count != 0);

                return removed;
            }

            /*! Removes all singlets from a mesh
              *
              * \param m Mesh
              *
              * \return Number of singlets removed
              */
            static int remove_singlets(MeshType &m)
            {
                int removed = 0;
                int count;
                do
                {
                    count = 0;
                    for(FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
                    {
                        if( !((*fi).IsD()) )
                        {
                            if( HalfEdgeTopology<MeshType>::is_singlet_quad(&(*fi)) )
                            {
                                HalfEdgeTopology<MeshType>::singlet_remove_quad(m, &(*fi));
                                count++;
                            }
                        }
                    }

                    removed += count;

                }while(count != 0);

                return removed;
            }

            /*! Checks if a mesh has singlets
              *
              * \param m Mesh
              *
              * \return Value indicating whether mesh has singlets
              */
            static bool has_singlets(MeshType &m)
            {

                for(FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
                {
                    if( !((*fi).IsD()) )
                    {
                        if( HalfEdgeTopology<MeshType>::is_singlet_quad(&(*fi)) )
                            return true;
                    }
                }

                return false;
            }

            /*! Checks if a mesh has doublets
              *
              * \param m Mesh
              *
              * \return Value indicating whether mesh has doublets
              */
            static bool has_doublets(MeshType &m)
            {

                for(FaceIterator fi = m.face.begin(); fi != m.face.end(); ++fi)
                {
                    if( !((*fi).IsD()) )
                    {
                        if( HalfEdgeTopology<MeshType>::has_doublet_quad(&(*fi)) )
                            return true;
                    }
                }

                return false;
            }

            /*! Performs rotation of selected edges computing the best rotation
              *
              *
              * \param m Mesh
              * \param hedges Vector of halfedges representing the edges to rotate
              * \param faces Set of modified faces (every modified face will be inserted into this set)
              *
              */
            template<class PriorityType>
            static void flip_edges(MeshType &m, vector<HEdgePointer> &hedges, set<FacePointer> &faces)
            {

                for(typename vector<HEdgePointer>::iterator hi = hedges.begin(); hi != hedges.end(); ++hi)
                {
                    // edge must be shared by two faces
                    if((*hi)->HFp() && (*hi)->HOp()->HFp())
                    {
                        if(!(*hi)->IsD() && !(*hi)->HFp()->IsD() && !(*hi)->HOp()->HFp()->IsD())
                        {
                            typename PriorityType::FlipType fliptype = PriorityType::best_flip( *hi );

                            if(fliptype != PriorityType::NO_FLIP)
                            {

                                vector<VertexPointer> vertices;

                                // Record old vertices for future removal of doublets
                                vertices.push_back((*hi)->HVp());
                                vertices.push_back((*hi)->HOp()->HVp());

                                // Add modified faces into the set of faces
                                faces.insert((*hi)->HFp());
                                faces.insert((*hi)->HOp()->HFp());

                                bool cw = (fliptype == PriorityType::CW_FLIP);
                                HalfEdgeTopology<MeshType>::edge_rotate_quad((*hi),cw);

                                // After a rotation doublets may have generated
                                remove_doublets(m, faces, vertices);

                            }
                        }
                    }
                }
            }

            /*! Performs edge rotations on the entire mesh computing the best rotation for each edge
              *
              *
              * \param m Mesh
              *
              */
            template <class PriorityType>
            static int flip_edges(MeshType &m)
            {
                int count;
                int ret=0;
                do
                {
                    count = 0;
                    for(typename MeshType::EdgeIterator ei = m.edge.begin(); ei != m.edge.end(); ++ei)
                    {

                        if( !(ei->IsD()) )
                        {
                            HEdgePointer hp = ei->EHp();

                            if(hp->HFp() && hp->HOp()->HFp())
                            {
                                typename PriorityType::FlipType fliptype = PriorityType::best_flip( hp );

                                if(fliptype != PriorityType::NO_FLIP)
                                {
                                    vector<VertexPointer> vertices;

                                    // Record old vertices for future removal of doublets
                                    vertices.push_back(hp->HVp());
                                    vertices.push_back(hp->HOp()->HVp());

                                    bool cw = (fliptype == PriorityType::CW_FLIP);
                                    HalfEdgeTopology<MeshType>::edge_rotate_quad(hp,cw);

                                    // After a rotation doublets may have generated
                                    remove_doublets(m, vertices);

                                    count++;
                                }
                            }

                        }
                    }

                    ret+=count;

                }while(count != 0);

                return ret;
            }

        };


        /*!
          * \brief Generic priority for edge rotations
          *
          */
        template <class MeshType> class EdgeFlipPriority
        {
        public:
            typedef typename MeshType::HEdgePointer HEdgePointer;

            /// Possible types of rotation
            enum FlipType { NO_FLIP, CW_FLIP, CCW_FLIP};

            /*!
              * Computes the best rotation to perform
              *
              * \param hp Pointer to an halfedge representing the edge to rotate
              *
              * \return The best type of rotation
              */
            static FlipType best_flip( HEdgePointer hp);
        };

        /*!
          * \brief Priority based on maximizing vertices regularity
          *
          */
        template <class MeshType> class VertReg: public EdgeFlipPriority<MeshType>
        {

        public:

            typedef typename MeshType::VertexPointer VertexPointer;
            typedef typename MeshType::EdgePointer EdgePointer;
            typedef typename MeshType::HEdgePointer HEdgePointer;
            typedef typename MeshType::FacePointer FacePointer;

            typedef EdgeFlipPriority<MeshType> Base;
            typedef typename Base::FlipType FlipType;

            /// Default Constructor
            VertReg(){}

            ~VertReg(){}

            /*!
              * Computes the best rotation to perform for maximizing vertices regularity
              *
              * \param hp Pointer to an halfedge representing the edge to rotate
              *
              * \return The best type of rotation
              */
            static FlipType best_flip( HEdgePointer hp)
            {
                assert(hp);
                assert(!hp->IsD());

                vector<VertexPointer> vps1 = HalfEdgeTopology<MeshType>::getVertices(hp->HFp(), hp);

                vector<VertexPointer> vps2 = HalfEdgeTopology<MeshType>::getVertices(hp->HOp()->HFp(), hp->HOp());

                valarray<double> valences(6);

                /*
                 Indices of vertices into vector valences:

                  3-------2
                  |       |
                  |  f1   |
                  |       |
                  0-------1
                  |       |
                  |  f2   |
                  |       |
                  4-------5

                  */

                // Compute valencies of vertices
                for(int i=0; i<4; i++)
                    valences[i] = HalfEdgeTopology<MeshType>::vertex_valence(vps1[i]) - 4 ;

                valences[4] = HalfEdgeTopology<MeshType>::vertex_valence(vps2[2]) - 4;
                valences[5] = HalfEdgeTopology<MeshType>::vertex_valence(vps2[3]) - 4;


                // Vector containing sums of the valencies in all the possible cases: no rotation, ccw rotation, cw rotation
                vector<int> sums;

                // positions:
                // sums[0]: now (No rotation)
                // sums[1]: flipping ccw
                // sums[2]: flipping cw

                // No rotation
                sums.push_back( pow(valences, 2.0).sum() );

                // CCW
                valences[0]--;
                valences[1]--;
                valences[2]++;
                valences[4]++;

                sums.push_back( pow(valences, 2.0).sum() );

                // CW
                valences[2]--;
                valences[4]--;
                valences[3]++;
                valences[5]++;

                sums.push_back( pow(valences, 2.0).sum() );


                if( sums[2]<= sums[1] && sums[2]< sums[0] )
                    return Base::CW_FLIP;

                else if( sums[1]< sums[2] && sums[1]< sums[0] )
                    return Base::CCW_FLIP;

                return Base::NO_FLIP;

            }

        };

        /*!
          * \brief Priority based on minimizing homeometry
          *
          */
        template <class MeshType> class Homeometry: public EdgeFlipPriority<MeshType>
        {

        public:

            typedef typename MeshType::VertexPointer VertexPointer;
            typedef typename MeshType::EdgePointer EdgePointer;
            typedef typename MeshType::HEdgePointer HEdgePointer;
            typedef typename MeshType::FacePointer FacePointer;


            typedef EdgeFlipPriority<MeshType> Base;
            typedef typename Base::FlipType FlipType;

            /// Default Constructor
            Homeometry(){}

            ~Homeometry(){}

            /*!
              * Computes the best rotation to perform for minimizing the distance from homeometry
              *
              * \param hp Pointer to an halfedge representing the edge to rotate
              *
              * \return The best type of rotation
              */
            static FlipType best_flip( HEdgePointer hp)
            {
                assert(hp);
                assert(!hp->IsD());

                vector<VertexPointer> face1 = HalfEdgeTopology<MeshType>::getVertices(hp->HFp(), hp);
                vector<VertexPointer> face2 = HalfEdgeTopology<MeshType>::getVertices(hp->HOp()->HFp(), hp->HOp());

                // Vector containing sums of the valencies in all the possible cases: no rotation, ccw rotation, cw rotation
                vector<int> sums;

                // positions:
                // sums[0]: now (No rotation)
                // sums[1]: flipping ccw
                // sums[2]: flipping cw

                // No rotation
                sums.push_back( distance_from_homeometry(face1, face2, 0) );

                // CCW
                face1[1] = face2[2];
                face2[1] = face1[2];

                sums.push_back( distance_from_homeometry(face1, face2, 1) );

                // CW
                face1[2] = face2[3];
                face2[2] = face1[3];

                sums.push_back( distance_from_homeometry(face1, face2, 2) );


                if( sums[2]<= sums[1] && sums[2]< sums[0] )
                    return Base::CW_FLIP;

                else if( sums[1]< sums[2] && sums[1]< sums[0] )
                    return Base::CCW_FLIP;

                return Base::NO_FLIP;

            }

        protected:

            /*!
              * Computes the area of a quad
              *
              * \param vertices Vector of the four vertices of the quad
              *
              * \return Area of the quad
              */
            static float area(vector<VertexPointer> &vertices)
            {

                assert(vertices.size() == 4);

                float tri1 =  Norm( (vertices[1]->cP() - vertices[0]->cP()) ^ (vertices[2]->cP() - vertices[0]->cP()) );
                float tri2 =  Norm( (vertices[2]->cP() - vertices[0]->cP()) ^ (vertices[3]->cP() - vertices[0]->cP()) );

                return (tri1+tri2) / 2;
            }

            /*!
              * Computes the distance of two faces from being homeometirc
              *
              * \param face1 Vector of vertices belonging to the first face
              * \param face2 Vector of vertices belonging to the second face
              * \param i Index of the edge to compute
              *
              * \return The computed homeometry
              */
            static float distance_from_homeometry(vector<VertexPointer> &face1, vector<VertexPointer> &face2, int i)
            {
                // Ideal edge length
                float mu = sqrt( (area(face1) + area(face2)) / 2 );

                // Length of the i-th edge (the edge changed with a rotation)
                float edge_length = Distance( face1[i]->cP(), face1[i+1]->cP() );

                // Length of the diagonals
                valarray<float> diagonals(4);

                diagonals[0] = Distance( face1[0]->cP(), face1[2]->cP() );
                diagonals[1] = Distance( face1[1]->cP(), face1[3]->cP() );
                diagonals[2] = Distance( face2[0]->cP(), face2[2]->cP() );
                diagonals[3] = Distance( face2[1]->cP(), face2[3]->cP() );

                // Ideal diagonal length
                float ideal_diag_length = SQRT_TWO*mu;

                float sum_diagonals = pow(diagonals - ideal_diag_length, 2.0).sum();

                return  (pow (edge_length - mu , static_cast<float>(2.0)) + sum_diagonals);
            }

        };

    }
}
#endif // HALFEDGEQUADCLEAN_H
