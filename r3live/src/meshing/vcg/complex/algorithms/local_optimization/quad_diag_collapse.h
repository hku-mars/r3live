#ifndef QUAD_DIAGONAL_COLLAPSE_H
#define QUAD_DIAGONAL_COLLAPSE_H

#include<vcg/connectors/halfedge_pos.h>
#include<vcg/complex/algorithms/local_optimization.h>
#include<vcg/complex/algorithms/smooth.h>

#include<set>

#include<vcg/space/ray3.h>

#include<vcg/complex/algorithms/halfedge_quad_clean.h>

namespace vcg{
    namespace tri{


        /*!
          * \brief Generic class for checking feasibility of collapses
          *
          */
        template<class MeshType, class TriMeshType >
        class FeasibilityCheck
        {
        public:
            typedef typename MeshType::HEdgePointer HEdgePointer;
            typedef typename TriMeshType::FaceType TriFaceType;
            typedef typename vcg::GridStaticPtr<TriFaceType, typename TriFaceType::ScalarType> GRID;
            typedef typename TriMeshType::CoordType CoordType;

            static bool check_feasible(HEdgePointer hp, CoordType &V1, CoordType &V2, TriMeshType &tm, GRID &grid);
        };

        /*!
          * \brief Generic class for weighting collapses
          *
          */
        template<class MeshType, class TriMeshType >
        class OperationWeight
        {
        public:
            typedef typename MeshType::HEdgePointer HEdgePointer;
            typedef typename TriMeshType::FaceType TriFaceType;
            typedef typename vcg::GridStaticPtr<TriFaceType, typename TriFaceType::ScalarType> GRID;
            typedef typename TriMeshType::CoordType CoordType;

            static float compute_weight(HEdgePointer hp, CoordType &P, TriMeshType &tm, GRID &grid);
        };


        /*!
          * \brief Class that provides methods for checking and weighting collapses using fitmaps
          *
          */
        template<class MeshType, class TriMeshType >
        class FitmapsCollapse : public FeasibilityCheck<MeshType, TriMeshType>, public OperationWeight<MeshType, TriMeshType>
        {

        protected:

            typedef typename MeshType::VertexPointer VertexPointer;
            typedef typename MeshType::HEdgePointer HEdgePointer;

            typedef typename TriMeshType::FaceType TriFaceType;
            typedef typename vcg::GridStaticPtr<TriFaceType, typename TriFaceType::ScalarType> GRID;

            typedef typename TriMeshType::CoordType CoordType;
            typedef typename TriMeshType::ScalarType ScalarType;
            typedef typename TriMeshType::FacePointer FacePointer;

            typedef typename TriMeshType::template PerVertexAttributeHandle<float> Fitmap_attr;

        public:

            /// Coefficient that will be multiplied with the value of the M-Fitmap
            static float& Mfit_coeff()
            {
                static float coeff = 1.5;
                return coeff;
            }

            /*! Checks if an operation is feasible using M-Fitmap
              *
              * \param hp Pointer to an halfedge that identifies a diagonal
              * \param V1 Coordinates of the first point of the diagonal
              * \param V2 Coordinates of the second point of the diagonal
              * \param tm Reference mesh
              * \param grid Spatial index used for raycasting
              *
              * \return Value indicating whether diagnoal is collapsible
              */
            static bool check_feasible(HEdgePointer hp, CoordType &V1, CoordType &V2, TriMeshType &tm, GRID &grid)
            {
                float length = Distance( V1, V2 );

                Fitmap_attr M_Fit = tri::Allocator<TriMeshType>::template GetPerVertexAttribute<float>(tm,"M-Fitmap");

                CoordType P = (V1+V2)/2;
                float fitmap = compute_fitmap(hp, P, tm, grid, M_Fit);

                return length <= fitmap/Mfit_coeff();
            }

            /*! Computes the weight of a diagonal using S-Fitmap
              *
              * \param hp Pointer to an halfedge that identifies a diagonal
              * \param P Coordinates of the point on which fitmap will be computed
              * \param tm Reference mesh
              * \param grid Spatial index used for raycasting
              *
              * \return Computed weight
              */
            static float compute_weight(HEdgePointer hp, CoordType &P, TriMeshType &tm, GRID &grid)
            {
                Fitmap_attr S_Fit = tri::Allocator<TriMeshType>::template GetPerVertexAttribute<float>(tm,"S-Fitmap");

                return compute_fitmap(hp, P, tm, grid, S_Fit);
            }

        protected:

            /*!
              * Performs the computation of a fitmap on a given point
              *
              * \param hp Pointer to an halfedge that identifies a diagonal
              * \param P Coordinates of the point on which fitmap will be computed
              * \param tm Reference mesh
              * \param grid Spatial index used for raycasting
              * \param attr Fitmap type (S or M)
              *
              * \return Computed value of the fitmap
              */
            static float compute_fitmap(HEdgePointer hp, CoordType &P, TriMeshType &tm, GRID &grid, Fitmap_attr &attr)
            {
                CoordType N(0,0,0);

                vector<VertexPointer> vertices = HalfEdgeTopology<MeshType>::getVertices(hp->HFp());

                assert(vertices.size() == 4);

                N += vcg::Normal<typename MeshType::CoordType>(vertices[0]->cP(), vertices[1]->cP(), vertices[2]->cP());
                N += vcg::Normal<typename MeshType::CoordType>(vertices[2]->cP(), vertices[3]->cP(), vertices[0]->cP());

                N.Normalize();

                CoordType C(0,0,0);
                FacePointer T = getClosestFaceRay(tm, grid, P, N, &C, NULL);

                float result = 1.0;

                if(T)
                {
                    float w0;
                    float w1;
                    float w2;
                    vcg::InterpolationParameters(*T, N, C, w0, w1, w2);

                    float s0 = attr[T->V(0)];
                    float s1 = attr[T->V(1)];
                    float s2 = attr[T->V(2)];

                    result = (w0*s0 + w1*s1 + w2*s2)/(w0+w1+w2);
                }

                return result;
            }


            static FacePointer getClosestFaceRay(TriMeshType &m, GRID &grid, CoordType P, CoordType raydir, CoordType* closest, ScalarType* minDist)
            {

                ScalarType diag = m.bbox.Diag();

                raydir.Normalize();

                Ray3<typename GRID::ScalarType> ray;

                ray.SetOrigin(P);
                ScalarType t;

                FacePointer f = 0;
                FacePointer fr = 0;

                vector<CoordType> closests;
                vector<ScalarType> minDists;
                vector<FacePointer> faces;

                ray.SetDirection(-raydir);

                f = vcg::tri::DoRay<TriMeshType,GRID>(m, grid, ray, diag/4.0, t);

                if (f)
                {
                    closests.push_back(ray.Origin() + ray.Direction()*t);
                    minDists.push_back(fabs(t));
                    faces.push_back(f);
                }

                ray.SetDirection(raydir);

                fr = vcg::tri::DoRay<TriMeshType,GRID>(m, grid, ray, diag/4.0, t);

                if (fr)
                {
                    closests.push_back(ray.Origin() + ray.Direction()*t);
                    minDists.push_back(fabs(t));
                    faces.push_back(fr);
                }

                if (fr)
                    if (fr->N()*raydir<0)
                        fr=0; // discard: inverse normal;

                if (minDists.size() == 0)
                {
                    if (closest) *closest=P;
                    if (minDist) *minDist=0;
                    f = 0;
                }
                else
                {
                    int minI = std::min_element(minDists.begin(),minDists.end()) - minDists.begin();
                    if (closest) *closest= closests[minI];
                    if (minDist) *minDist= minDists[minI];
                    f = faces[minI];
                }

                return f;

            }

        };



        /*!
          * \brief Class implementing simplification of quad meshes by diagonal collapses
          *
          */
        template<class MeshType, class MYTYPE, class TriMeshType, class OptimizationType>
        class QuadDiagonalCollapseBase: public LocalOptimization<MeshType>::LocModType
        {

        protected:

            typedef Pos<MeshType> PosType;
            typedef typename MeshType::VertexPointer VertexPointer;
            typedef typename MeshType::FacePointer FacePointer;
            typedef typename MeshType::HEdgePointer HEdgePointer;
            typedef typename LocalOptimization<MeshType>::HeapElem HeapElem;
            typedef typename LocalOptimization<MeshType>::HeapType HeapType;
            typedef typename MeshType::ScalarType ScalarType;
            typedef typename MeshType::CoordType CoordType;
            typedef typename TriMeshType::FaceType TriFaceType;
            typedef typename vcg::GridStaticPtr<TriFaceType, typename TriFaceType::ScalarType> GRID;

            /// Vertex returned by the collapse
            VertexPointer ret;

            /// Global mark for updating
            static int& GlobalMark()
            {
                static int im=0;
                return im;
            }

            /// Local mark for updating
            int localMark;

            /// Priority in the heap
            ScalarType _priority;

            /// Set of modified faces
            set<FacePointer> faces;

            /// Halfedge that identifies the diagonal to collapse
            HEdgePointer hp;

        public:

            /// Reference mesh used for smoothing
            static TriMeshType* &refMesh()
            {
                static TriMeshType* m = NULL;
                return m;
            }

            /// Spatial index for smoothing
            static GRID* &grid()
            {
                static GRID* grid = NULL;
                return grid;
            }

            /// Number of smoothing iterations to be performed
            static unsigned int &smoothing_iterations()
            {
                static unsigned int iterations = 5;
                return iterations;
            }

            /// Default Constructor
            QuadDiagonalCollapseBase(){}

            /*!
              * Constructor
              *
              * \param he Pointer to an halfedge representing a diagonal
              * \param mark Temporal mark of the operation
              */
            QuadDiagonalCollapseBase(HEdgePointer he, int mark)
            {
                localMark = mark;
                hp = he;
                _priority = ComputePriority();
            }

            ~QuadDiagonalCollapseBase()
            {
                faces.clear();
            }

            /*!
              * Computes priority of the operation as the length of the diagonal
              *
              * \return Priority
              */
            ScalarType ComputePriority()
            {
                CoordType V1 = hp->HVp()->cP();
                CoordType V2 = hp->HNp()->HNp()->HVp()->cP();

                _priority = Distance( V1, V2 );
                return _priority;
            }

            virtual const char *Info(MeshType &m)
            {
                static char buf[60];
                sprintf(buf,"(%d - %d) %g\n", hp->HVp()-&m.vert[0], hp->HNp()->HNp()->HVp()-&m.vert[0], -_priority);
                return buf;
            }

            /*!
              * Performs the collapse and the optimizations
              *
              * \param m Mesh
              *
              */
            inline void Execute(MeshType &m)
            {

                // Collapse the diagonal
                ret = HalfEdgeTopology<MeshType>::diagonal_collapse_quad(m,hp->HFp(), hp->HVp());

                if(ret->VHp())
                {
                    set<FacePointer> tmp = HalfEdgeTopology<MeshType>::getFaces(ret);

                    vector<FacePointer> incident_faces = HalfEdgeTopology<MeshType>::get_incident_faces(ret,ret->VHp());

                    faces.insert(incident_faces.begin(), incident_faces.end());

                    HalfedgeQuadClean<MeshType>::remove_doublets(m, faces);

                    // Optimization by edge rotations
                    if(!ret->IsD())
                    {
                        vector<HEdgePointer> hedges = HalfEdgeTopology<MeshType>::get_incident_hedges(ret,ret->VHp());

                        HalfedgeQuadClean<MeshType>:: template flip_edges<OptimizationType>(m, hedges, faces);
                    }

                    faces.insert(tmp.begin(), tmp.end());

                    // Set of all vertices to smooth
                    set<VertexPointer> vertices;

                    for(typename set<FacePointer>::iterator fi = faces.begin(); fi != faces.end(); ++fi)
                    {
                        if(*fi)
                        {
                            if( !((*fi)->IsD()))
                            {
                                vector<VertexPointer> aux = HalfEdgeTopology<MeshType>::getVertices(*fi);

                                vertices.insert(aux.begin(),aux.end());
                            }
                        }
                    }


                    // Smoothing
                    for(unsigned int i = 0; i < smoothing_iterations(); i++)
                    {
                        for(typename set<VertexPointer>::iterator vi = vertices.begin(); vi!= vertices.end(); ++vi)
                            if(!HalfEdgeTopology<MeshType>::isBorderVertex(*vi))
                                Smooth<MeshType>::VertexCoordLaplacianReproject(m,*grid(), *refMesh(),*vi);
                    }


                    // Add all faces modified by smoothing into the set of modified faces
                    for(typename set<VertexPointer>::iterator vi = vertices.begin(); vi!= vertices.end(); ++vi)
                    {
                        vector<FacePointer> tmp_faces = HalfEdgeTopology<MeshType>::get_incident_faces(*vi);

                        faces.insert(tmp_faces.begin(), tmp_faces.end());
                    }

                }


            }

            /*!
              * Updates the heap of operations.
              * For each modified face inserts into the heap two consecutive halfedges representing the two diagonals
              *
              * \param h_ret Heap to be updated
              *
              */
            inline  void UpdateHeap(HeapType & h_ret)
            {

                GlobalMark()++;

                for(typename set<FacePointer>::iterator fi = faces.begin(); fi != faces.end(); ++fi)
                {
                    if(*fi)
                    {
                        if( !((*fi)->IsD()))
                        {
                            (*fi)->IMark() = GlobalMark();

                            HEdgePointer start_he = (*fi)->FHp();

                            h_ret.push_back( HeapElem( new MYTYPE( start_he, GlobalMark() ) ) );
                            std::push_heap( h_ret.begin(),h_ret.end() );

                            h_ret.push_back( HeapElem( new MYTYPE( start_he->HNp(), GlobalMark() ) ) );
                            std::push_heap( h_ret.begin(),h_ret.end() );
                        }
                    }
                }
            }


            ModifierType IsOfType()
            {
                return QuadDiagCollapseOp;
            }

            /*!
              * Checks if the operation can be done without generation of degenerate configurations
              *
              * \return Value indicating whether the operation can be performed
              */
            inline bool IsFeasible()
            {
                FacePointer fp = hp->HFp();

                if(!fp)
                    return false;

                if(fp->IsD() || fp->VN() !=4)
                    return false;

                return ( HalfEdgeTopology<MeshType>::check_diagonal_collapse_quad(hp));

            }

            /*!
              * Checks if the operation is up to date
              *
              * \return Value indicating whether operation is up to date
              */
            inline bool IsUpToDate()
            {
                FacePointer fp = hp->HFp();

                if(fp)
                    return (!hp->IsD() && localMark >= fp->IMark() );

                return false;

            }

            /*!
              * Gets the priority of the operation
              *
              * \return Value indicating the priority
              */
            virtual ScalarType Priority() const
            {
                return _priority;
            }

            /*!
              * Initializes a heap with all the possible diagonal collapses of the mesh
              * For each face inserts two consecutive halfedges representing the two diagonals
              *
              * \param m Mesh
              * \param h_ret heap to be initialized
              *
              */
            static void Init(MeshType &m,HeapType &h_ret)
            {
                // Grid and reference mesh must be initialized
                assert(grid());
                assert(refMesh());

                assert(!HalfedgeQuadClean<MeshType>::has_doublets(m));
                assert(!HalfedgeQuadClean<MeshType>::has_singlets(m));

                vcg::tri::InitFaceIMark(m);

                h_ret.clear();

                typename MeshType::FaceIterator fi;
                for(fi = m.face.begin(); fi != m.face.end();++fi)
                {
                    if(!(*fi).IsD())
                    {

                        h_ret.push_back( HeapElem(new MYTYPE( (*fi).FHp(), IMark(m))));

                        h_ret.push_back( HeapElem(new MYTYPE( (*fi).FHp()->HNp(), IMark(m))));

                    }
                }

            }


        };

        /*!
          * \brief Class implementing simplification of quad meshes by diagonal collapses
          * priority of the operations is weighted with a value computed by class WeightType
          * Feasibility is checked with class CheckType
          *
          */
        template<class MeshType, class MYTYPE, class TriMeshType, class OptimizationType, class WeightType, class CheckType >
        class QuadDiagonalCollapse: public QuadDiagonalCollapseBase<MeshType, MYTYPE, TriMeshType, OptimizationType>
        {

        protected:

            typedef Pos<MeshType> PosType;
            typedef typename MeshType::VertexPointer VertexPointer;
            typedef typename MeshType::FacePointer FacePointer;
            typedef typename MeshType::HEdgePointer HEdgePointer;
            typedef typename LocalOptimization<MeshType>::HeapElem HeapElem;
            typedef typename LocalOptimization<MeshType>::HeapType HeapType;
            typedef typename MeshType::ScalarType ScalarType;
            typedef typename MeshType::CoordType CoordType;

            typedef typename TriMeshType::FaceType TriFaceType;
            typedef typename vcg::GridStaticPtr<TriFaceType, typename TriFaceType::ScalarType> GRID;

        public:

            /// Default constructor
            QuadDiagonalCollapse(){}

            /*!
              * Constructor
              *
              * \param he Pointer to an halfedge representing a diagonal
              * \param mark Temporal mark of the operation
              */
            QuadDiagonalCollapse(HEdgePointer he, int mark)
            {
                this->localMark = mark;
                this->hp = he;
                this->_priority = this->ComputePriority();
            }

            /*!
              * Computes priority of the operation as length * weight
              *
              * \return Priority
              */
            ScalarType ComputePriority()
            {
                CoordType V1 = this->hp->HVp()->cP();
                CoordType V2 = this->hp->HNp()->HNp()->HVp()->cP();

                CoordType P = (V1+V2)/2;
                float weight = WeightType::compute_weight(this->hp, P, *(this->refMesh()), *(this->grid()));

                this->_priority = Distance( V1, V2 ) * weight;
                return this->_priority;
            }

            /*!
              * Checks if the operation can be done without generation of degenerate configurations
              *
              * \return Value indicating whether the operation can be performed
              */
            bool IsFeasible()
            {
                FacePointer fp = this->hp->HFp();

                if(!fp)
                    return false;

                if(fp->IsD() || fp->VN() !=4)
                    return false;

                if(!HalfEdgeTopology<MeshType>::check_diagonal_collapse_quad(this->hp))
                    return false;

                CoordType V1 = this->hp->HVp()->cP();
                CoordType V2 = this->hp->HNp()->HNp()->HVp()->cP();

                return CheckType::check_feasible(this->hp, V1, V2, *(this->refMesh()), *(this->grid()));

            }

        };


    }//end namespace tri
}//end namespace vcg

#endif // QUAD_DIAGONAL_COLLAPSE_H
