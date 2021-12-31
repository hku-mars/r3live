/****************************************************************************
* MeshLab                                                           o o     *
* A versatile mesh processing toolbox                             o     o   *
*                                                                _   O  _   *
* Copyright(C) 2007                                                \/)\/    *
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

#ifndef OVERLAP_ESTIMATION_H
#define OVERLAP_ESTIMATION_H

#include <vcg/math/gen_normal.h>
#include <vcg/math/random_generator.h>
#include <vcg/space/index/grid_static_ptr.h>
#include <vcg/complex/algorithms/closest.h>
#include <vcg/complex/algorithms/point_sampling.h>

#include <qdatetime.h>

using namespace std;
using namespace vcg;

/** \brief This class provides a strategy to estimate the overlap percentage of two range maps/point clouds.
 *
 * This class can be used, for exemple, into an automatic alignment process to check the quality of the
 * transformation found; the idea is that bad alignments should have a small overlap. Two points are
 * considered 'overlapping in the righ way' if they are close (i.e distance is less then \c consensusDist)
 * and at the same time points' normals match quite well (i.e the angle between them is less then
 *  \c consensusNormalsAngle). The test to compute the overlap is perfomed on a given number of points
 * (2500 is the default) sampled in a normal equalized way (default) or uniformly.
 * \author Francesco Tonarelli
 */
template<class MESH_TYPE> class OverlapEstimation
{
    public:

    typedef MESH_TYPE MeshType;
    typedef typename MeshType::ScalarType ScalarType;
    typedef typename MeshType::CoordType CoordType;
    typedef typename MeshType::VertexType VertexType;
    typedef typename MeshType::FaceType FaceType;
    typedef typename MeshType::VertexPointer VertexPointer;
    typedef typename MeshType::VertexIterator VertexIterator;
    typedef typename vector<VertexPointer>::iterator VertexPointerIterator;
    typedef GridStaticPtr<VertexType, ScalarType > MeshGrid;
    typedef tri::VertTmark<MeshType> MarkerVertex;

    private:
    /** Private simple class needed to perform sampling of pointers to vertexes. */
    class VertexPointerSampler
    {
        public:

        MeshType* m;  //this is needed for advanced sampling (i.e poisson sampling)

        VertexPointerSampler(){ m = new MeshType(); m->Tr.SetIdentity(); m->sfn=0; }
        ~VertexPointerSampler(){ if(m) delete m; }
        vector<VertexType*> sampleVec;

        void AddVert(VertexType &p){ sampleVec.push_back(&p); } //this function is the only we really need
        void AddFace(const FaceType &f, const CoordType &p){}
        void AddTextureSample(const FaceType &, const CoordType &, const Point2i &){}
    };

    public:
    /** \brief Public class to hold parameters. Used to avoid endless list of parameters inside functions.
      * \author Francesco Tonarelli
      */
    class Parameters
    {
        public:
        int samples;                                    ///< Number of samples to check to compute the overlap. Higher values get more accurancy but requires more time.
        int bestScore;                                  ///< Score to overcome to paint \c mMov . If overlap estimation is called many times inside a loop, you can set this value in each iteration to paint \c mMov and see only the best overlap achived.
        float consensusDist;                            ///< Consensus distance. Lower values should gat more accurancy; high values can lead to performance hit.
        float consensusNormalsAngle;                    ///< Holds the the consensus angle for normals, in gradients. Lower values decrease accurancy, particulary for range maps with many peaks and high frequencies.
        float threshold;                                ///< Consensus percentage requested to win consensus. Used to paint \c mMov. If the overlap overcames the \c threshold (and \c bestScore), \c mMov is painted.
        bool normalEqualization;                        ///< Allows to use normal equalization sampling in consensus. If set to \c false uniform sampling is used instead. Uniform sampling is faster but less accurate.
        bool paint;                                     ///< Allows painting of \c mMov according to consensus. See Paint() for details.
        void (*log)(int level, const char * f, ... );   ///< Pointer to a log function.

        /** Constructor with default values. */
        Parameters()
        {
            samples = 2500;
            bestScore = 0;
            consensusDist = 2.0f;
            consensusNormalsAngle = 0.965f;   //15 degrees.
            threshold = 0.0f;
            normalEqualization = true;
            paint = false;
            log = NULL;
        }
    };

    private:
    MeshType* mFix;                             /** Pointer to mesh \c mFix. */
    MeshType* mMov;                             /** Pointer to mesh \c mMov. */
    vector<vector<int> >* normBuckets;          //structure to hold normals bucketing. Needed for normal equalized sampling during consensus
    MeshGrid* gridFix;                          //variable to manage uniform grid
    MarkerVertex markerFunctorFix;              //variable to manage uniform grid

    public:
    /** Default constructor. */
    OverlapEstimation() : normBuckets(NULL), gridFix(NULL){}
    /** Default destructor. Deallocates structures. */
    ~OverlapEstimation(){
        if(normBuckets) delete normBuckets;
        if(gridFix) delete gridFix;
    }
    /** Set the fix mesh \c mFix. */
    void SetFix(MeshType& m){ mFix = &m; }
    /** Set the move mesh \c mMov. */
    void SetMove(MeshType& m){ mMov = &m; }

    /** Paint \c mMov according to the overlap estimation result. Works only if \c Compute() or \c Check() have
     *  been previously called with \c Parameters.paint=true .<br>Legend: \arg \e red: points overlaps correctly.
     *  \arg \e blue: points are too far to overlap. \arg \e yellow: points are close, but normals mismatch.
     */
    void Paint()
    {
        for(VertexIterator vi=mMov->vert.begin(); vi!=mMov->vert.end(); vi++){
            if(!(*vi).IsD()){
                if((*vi).Q()==0.0) (*vi).C() = Color4b::Red;
                if((*vi).Q()==1.0) (*vi).C() = Color4b::Yellow;
                if((*vi).Q()==2.0) (*vi).C() = Color4b::Blue;
            }
        }
    }

    /** Initializes structures.
     *  @param param A reference to a \c Parameter class containing all the desidered options to estimate overlap.
     *  \return \c true if everything goes right.
     */
    bool Init(Parameters& param){
        //builds the uniform grid with mFix vertices
        gridFix = new MeshGrid();
        SetupGrid();

        //if requested, group normals of mMov into 30 buckets. Buckets are used for Vertex Normal Equalization
        //in consensus. Bucketing is done here once for all to speed up consensus.
        if(normBuckets) {normBuckets->clear(); delete normBuckets; }
        if(param.normalEqualization){
            normBuckets = BucketVertexNormal(mMov->vert, 30);
            assert(normBuckets);
        }
        return true;
    }

    /** Compute the overlap estimation between \c mFix and \c mMov.
     *  @param param A reference to a \c Parameter class containing all the desidered options to estimate overlap.
     *  \return The percentage of overlap in the range \c [0..1] .
     */
    float Compute(Parameters& param)
    {
        return Check(param)/float(param.samples);
    }

    /** Compute the overlap estimation between \c mFix and \c mMov.
     *  @param param A reference to a \c Parameter class containing all the desidered options to estimate overlap.
     *  \return The number of points that overlap correctly. This number is in the range \c [0..param.samples] .
     */
    //IMPORTANT: per vertex normals of mMov and mFix MUST BE PROVIDED YET NORMALIZED!!!
    int Check(Parameters& param)
    {
        //pointer to a function to compute distance beetween points
        vertex::PointDistanceFunctor<ScalarType> PDistFunct;

        //if no buckets are provided get a vector of vertex pointers sampled uniformly
        //else, get a vector of vertex pointers sampled in a normal equalized manner; used as query points
        vector<VertexPointer> queryVert;
        if(param.normalEqualization){
            assert(normBuckets);
            for(unsigned int i=0; i<mMov->vert.size(); i++) queryVert.push_back(&(mMov->vert[i]));//do a copy of pointers to vertexes
            SampleVertNormalEqualized(queryVert, param.samples);
        }
        else{
            SampleVertUniform(*mMov, queryVert, param.samples);
        }
        assert(queryVert.size()!=0);

        //init variables for consensus
        float consDist = param.consensusDist*(mMov->bbox.Diag()/100.0f);  //consensus distance
        int cons_succ = int(param.threshold*(param.samples/100.0f));      //score needed to pass consensus
        int consensus = 0;                  //counts vertices in consensus
        float dist;                         //holds the distance of the closest vertex found
        VertexType* closestVertex = NULL;   //pointer to the closest vertex
        Point3<ScalarType> queryNrm;        //the query point normal for consensus
        CoordType queryPnt;                 //the query point for consensus
        CoordType closestPnt;               //the closest point found in consensus
        Matrix33<ScalarType> inv33_matMov(mMov->Tr,3);          //3x3 matrix needed to transform normals
        Matrix33<ScalarType> inv33_matFix(Inverse(mFix->Tr),3); //3x3 matrix needed to transform normals

        //consensus loop
        VertexPointerIterator vi; int i;
        for(i=0, vi=queryVert.begin(); vi!=queryVert.end(); vi++, i++)
        {
            dist = -1.0f;
            //set query point; vertex coord is transformed properly in fix mesh coordinates space; the same for normals
            queryPnt = Inverse(mFix->Tr) * (mMov->Tr * (*vi)->P());
            queryNrm = inv33_matFix * (inv33_matMov * (*vi)->N());
            //if query point is bbox, the look for a vertex in cDist from the query point
            if(mFix->bbox.IsIn(queryPnt)) closestVertex = gridFix->GetClosest(PDistFunct,markerFunctorFix,queryPnt,consDist,dist,closestPnt);
            else closestVertex=NULL;  //out of bbox, we consider the point not in consensus...

            if(closestVertex!=NULL && dist < consDist){
                assert(closestVertex->P()==closestPnt); //coord and vertex pointer returned by getClosest must be the same

                //point is in consensus distance, now we check if normals are near
                if(queryNrm.dot(closestVertex->N())>param.consensusNormalsAngle)  //15 degrees
                {
                    consensus++;  //got consensus
                    if(param.paint) (*vi)->Q() = 0.0f;  //store 0 as quality
                }
                else{
                    if(param.paint) (*vi)->Q() = 1.0f;  //store 1 as quality
                }
            }
            else{
                if(param.paint) (*vi)->Q() = 2.0f;  //store 2 as quality
            }
        }

        //Paint the mesh only if required and if consensus is the best ever found. Colors have been stores as numbers into quality attribute
        if(param.paint){
            if(consensus>=param.bestScore && consensus>=cons_succ) Paint();
        }

        return consensus;
    }

    private:
    /** Fill the vector \c vert with \c sampleNum pointers to vertexes sampled uniformly from mesh \c m .
      * @param m Source mesh.
      * @param vert Destination vector.
      * @param sampleNum Requested number of vertexes.
      */
    void SampleVertUniform(MESH_TYPE& m, vector<typename MESH_TYPE::VertexPointer>& vert, int sampleNum)
    {
        VertexPointerSampler sampler;
        tri::SurfaceSampling<MeshType, VertexPointerSampler>::VertexUniform(m, sampler, sampleNum);
        for(unsigned int i=0; i<sampler.sampleVec.size(); i++) vert.push_back(sampler.sampleVec[i]);
    }
    /** Buckets normals of the vertexes contained in \c vert .
      * \return A vector of vectors containing indexes to \c vert .
      */
    vector<vector<int> >* BucketVertexNormal(typename MESH_TYPE::VertContainer& vert, int bucketDim = 30)
    {
        static vector<Point3f> NV;
        if(NV.size()==0) GenNormal<float>::Uniform(bucketDim,NV);

        vector<vector<int> >* BKT = new vector<vector<int> >(NV.size()); //NV size is greater then bucketDim, so don't change this!

        int ind;
        for(int i=0;i<vert.size();++i){
            ind=GenNormal<float>::BestMatchingNormal(vert[i].N(),NV);
            (*BKT)[ind].push_back(i);
        }

        return BKT;
    }
    /** Samples the \c vert vector in a normal equalized way.
      * \return \c SampleNum pointers to vertexes sampled in a normal equalized way. Pointers are stored in
      * the \c vert (i.e it is an \c in/out parameter).
      */
    bool SampleVertNormalEqualized(vector<typename MESH_TYPE::VertexPointer>& vert, int SampleNum)
    {
        assert(normBuckets);
        // vettore di contatori per sapere quanti punti ho gia' preso per ogni bucket
        vector<int> BKTpos(normBuckets->size(),0);

        if(SampleNum >= int(vert.size())) SampleNum= int(vert.size()-1);

        int ind;
        for(int i=0;i<SampleNum;){
            ind=LocRnd(normBuckets->size()); // Scelgo un Bucket
            int &CURpos = BKTpos[ind];
            vector<int> &CUR = (*normBuckets)[ind];

            if(CURpos<int(CUR.size())){
                swap(CUR[CURpos], CUR[ CURpos + LocRnd((*normBuckets)[ind].size()-CURpos)]);
                swap(vert[i],vert[CUR[CURpos]]);
                ++BKTpos[ind];
                ++i;
            }
        }

        vert.resize(SampleNum);
        return true;
    }
    /** Function to retrieve a static random number generator object.
      * \return A \c SubtractiveRingRNG object.
      */
    static math::SubtractiveRingRNG &LocRnd(){
        static math::SubtractiveRingRNG myrnd(time(NULL));
        return myrnd;
    }
    /** Gets a random number in the interval \c [0..n] . Number is
      * produced by a \c SubtractiveRingRNG object initialized once for all.
      * \return A random number in the interval \c [0..n] .
      */
    static int LocRnd(int n){
        return LocRnd().generate(n);
    }
    /** Put \c mFix into a grid. */
    inline void SetupGrid()
    {
        gridFix->Set(mFix->vert.begin(),mFix->vert.end());
        markerFunctorFix.SetMesh(mFix);
    }
};

#endif // OVERLAP_ESTIMATION_H
