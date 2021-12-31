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
#ifndef _AUTOALIGN_4PCS_H_
#define _AUTOALIGN_4PCS_H_

/**
implementation of the 4PCS method from the paper:
"4-Points Congruent Sets for Robust Pairwise Surface Registration"
D.Aiger, N.Mitra D.Cohen-Or, SIGGRAPH 2008
ps: the name of the variables are out of vcg standard but like the one
used in the paper pseudocode.
*/

#include <vcg/complex/complex.h>
#include <vcg/space/point_matching.h>
#include <vcg/complex/algorithms/closest.h>
#include <wrap/io_trimesh/export_ply.h>

// note: temporary (callback.h should be moved inside vcg)
typedef bool AACb( const int pos,const char * str );

namespace vcg{
namespace tri{

template <class MeshType>
class FourPCS {
public:
    /* mesh only for using spatial indexing functions (to remove) */
  class PVertex;    // dummy prototype never used
  class PFace;

  class PUsedTypes: public vcg::UsedTypes < vcg::Use<PVertex>::template AsVertexType,
                                            vcg::Use<PFace  >::template AsFaceType >{};

  class PVertex : public vcg::Vertex< PUsedTypes,vcg::vertex::BitFlags,vcg::vertex::Coord3f ,vcg::vertex::Mark>{};
  class PFace   : public vcg::Face<   PUsedTypes> {};
  class PMesh   : public vcg::tri::TriMesh< std::vector<PVertex>, std::vector<PFace> > {};

  typedef typename MeshType::ScalarType ScalarType;
  typedef typename MeshType::CoordType CoordType;
  typedef typename MeshType::VertexIterator VertexIterator;
  typedef typename MeshType::VertexType VertexType;
  typedef vcg::Point4< vcg::Point3<ScalarType> > FourPoints;
  typedef vcg::GridStaticPtr<typename PMesh::VertexType, ScalarType > GridType;

  /* class for Parameters */
  struct Param
  {
    ScalarType delta;   // Approximation Level
    int feetsize;       // how many points in the neighborhood of each of the 4 points
    ScalarType f;       // overlap estimation as a percentage
    int scoreFeet;      // how many of the feetsize points must match (max feetsize*4) to try an early interrupt
    int scoreAln;       // how good must be the alignement	to end the process successfully

    void Default(){
      delta = 0.5;
      feetsize = 25;
      f = 0.5;
      scoreFeet = 50;
      scoreAln = 200;
    }
  };

  Param par;	/// parameters

public:
  void Init(MeshType &_P,MeshType &_Q);
  bool Align( int   L, vcg::Matrix44f & result, vcg::CallBackPos * cb = NULL );		// main function

private:
  struct Couple: public std::pair<int,int>
  {
    Couple(const int & i, const int & j, float d):std::pair<int,int>(i,j),dist(d){}
    Couple(float d):std::pair<int,int>(0,0),dist(d){}
    float dist;
    const bool operator < (const   Couple & o) const {return dist < o.dist;}
    int & operator[](const int &i){return (i==0)? first : second;}
  };

  /* returns the closest point between to segments x1-x2 and x3-x4.  */
  void IntersectionLineLine(const CoordType & x1,const CoordType & x2,const CoordType & x3,const CoordType & x4, CoordType&x)
  {
    CoordType a = x2-x1, b = x4-x3, c = x3-x1;
    x = x1 + a * ((c^b).dot(a^b)) / (a^b).SquaredNorm();
  }




  struct Candidate
  {
    Candidate(){}
    Candidate(FourPoints _p,vcg::Matrix44<ScalarType>_T):p(_p),T(_T){}
    FourPoints  p;
    vcg::Matrix44<ScalarType> T;
    ScalarType err;
    int score;
    int base; // debug: for which base
    inline bool operator <(const Candidate & o) const {return score > o.score;}
  };


  MeshType	*P;	  // mesh from which the coplanar base is selected
  MeshType			*Q;											// mesh where to find the correspondences
  std::vector<int> mapsub;					// subset of index to the vertices in Q


  PMesh     Invr;										// invariants

  std::vector< Candidate > U;
  Candidate winner;
  int iwinner;											// winner == U[iwinner]

  FourPoints B;											// coplanar base
  std::vector<FourPoints> bases;		// used bases
  ScalarType side;									// side
  std::vector<VertexType*> ExtB[4]; // selection of vertices "close" to the four point
  std::vector<VertexType*> subsetP; // random selection on P
  ScalarType radius;

  ScalarType Bangle;
  std::vector<Couple > R1/*,R2*/;
  ScalarType r1,r2;

  // class for the point  'ei'
  struct EPoint{
    EPoint(vcg::Point3<ScalarType> _p, int _i):pos(_p),pi(_i){}
    vcg::Point3<ScalarType> pos;
    int pi;		//index to R[1|2]
    void GetBBox(vcg::Box3<ScalarType> & b){b.Add(pos);}
  };

    GridType *ugrid; // griglia
    vcg::GridStaticPtr<typename MeshType::VertexType, ScalarType > ugridQ;
    vcg::GridStaticPtr<typename MeshType::VertexType, ScalarType > ugridP;

    bool SelectCoplanarBase();												// on P
    bool FindCongruent() ;														// of base B, on Q, with approximation delta
    void ComputeR1R2(ScalarType d1,ScalarType d2);

    bool IsTransfCongruent(FourPoints fp,vcg::Matrix44<ScalarType> & mat, float &  trerr);
    int EvaluateSample(Candidate & fp, CoordType & tp, CoordType & np, const float &  angle);
    void EvaluateAlignment(Candidate & fp);
    void TestAlignment(Candidate & fp);

    /* debug tools */
public:
    std::vector<vcg::Matrix44f> allTr;// tutte le trasformazioni provate
    FILE * db;
    char namemesh1[255],namemesh2[255];
    int n_base;
    void InitDebug(const char * name1, const char * name2){
        db = fopen("debugPCS.txt","w");
        sprintf(&namemesh1[0],"%s",name1);
        sprintf(&namemesh2[0],"%s",name2);
        n_base = 0;
    }

    void FinishDebug(){
        fclose(db);
    }
    //void SaveALN(char * name,vcg::Matrix44f mat ){
    //	FILE * o = fopen(name,"w");
    //	fprintf(o,"2\n%s\n#\n",namemesh1);
    //	for(int  i = 0 ; i < 4; ++i)
    //		fprintf(o,"%f %f %f %f\n",mat[i][0],mat[i][1],mat[i][2],mat[i][3]);
    //	fprintf(o,"%s\n#\n",namemesh2);
    //	fprintf(o,"1.0 0.0 0.0 0.0 \n");
    //	fprintf(o,"0.0 1.0 0.0 0.0 \n");
    //	fprintf(o,"0.0 0.0 1.0 0.0 \n");
    //	fprintf(o,"0.0 0.0 0.0 1.0 \n");

    //	fclose(o);
    //}

};

template <class MeshType>
void FourPCS<MeshType>:: Init(MeshType &_P,MeshType &_Q)
{
        P = &_P;Q=&_Q;
        ugridQ.Set(Q->vert.begin(),Q->vert.end());
        ugridP.Set(P->vert.begin(),P->vert.end());

        float ratio = 800 / (float) Q->vert.size();
        for(int vi = 0; vi < Q->vert.size(); ++vi)
        if(rand()/(float) RAND_MAX < ratio)
            mapsub.push_back(vi);

        for(int vi = 0; vi < P->vert.size(); ++vi)
        if(rand()/(float) RAND_MAX < ratio)
            subsetP.push_back(&P->vert[vi]);

        // estimate neigh distance
        float avD = 0.0;
        for(int i = 0 ; i < 100; ++i){
            int ri = rand()/(float) RAND_MAX * Q->vert.size() -1;
            std::vector< CoordType > samples,d_samples;
            std::vector<ScalarType > dists;
            std::vector<VertexType* > ress;
            vcg::tri::GetKClosestVertex<
                    MeshType,
                    vcg::GridStaticPtr<typename MeshType::VertexType, ScalarType>,
                    std::vector<VertexType*>,
                    std::vector<ScalarType>,
                    std::vector< CoordType > >(*Q,ugridQ,2,Q->vert[ri].cP(),Q->bbox.Diag(), ress,dists, samples);
            assert(ress.size() == 2);
            avD+=dists[1];
        }
        avD	/=100;						// average vertex-vertex distance
        avD /= sqrt(ratio);		// take into account the ratio

        par.delta = avD * par.delta;
        side = P->bbox.Dim()[P->bbox.MaxDim()]*par.f; //rough implementation

    }

template <class MeshType>
bool
FourPCS<MeshType>::SelectCoplanarBase(){

    vcg::tri::UpdateBounding<MeshType>::Box(*P);

    // choose the inter point distance
    ScalarType dtol = side*0.1; //rough implementation

    //choose the first two points
    int i = 0,ch;

    // first point random
    ch = (rand()/(float)RAND_MAX)*(P->vert.size()-2);
    B[0] = P->vert[ch].P();
//printf("B[0] %d\n",ch);
    // second a point at distance d+-dtol
    for(i = 0; i < P->vert.size(); ++i){
        ScalarType dd = (P->vert[i].P() - B[0]).Norm();
        if(  ( dd < side + dtol) && (dd > side - dtol)){
            B[1] = P->vert[i].P();
//printf("B[1] %d\n",i);
            break;
        }
    }
    if(i ==  P->vert.size())
        return false;

    // third point at distance d from B[1] and forming a right angle
    int best = -1; ScalarType bestv=std::numeric_limits<float>::max();
    for(i = 0; i < P->vert.size(); ++i){
        int id = rand()/(float)RAND_MAX *  (P->vert.size()-1);
        ScalarType dd = (P->vert[id].P() - B[1]).Norm();
        if(  ( dd < side + dtol) && (dd > side - dtol)){
            ScalarType angle =  fabs( ( P->vert[id].P()-B[1]).normalized().dot((B[1]-B[0]).normalized()));
            if( angle < bestv){
                bestv = angle;
                best = id;
            }
        }
    }
    if(best == -1)
        return false;
    B[2] = P->vert[best].P();
//printf("B[2] %d\n",best);

    CoordType n = ((B[0]-B[1]).normalized() ^ (B[2]-B[1]).normalized()).normalized();
    CoordType B4 = B[1] +  (B[0]-B[1]) + (B[2]-B[1]);
    VertexType * v =0;
    ScalarType radius = dtol*4.0;

        std::vector<typename MeshType::VertexType*> closests;
        std::vector<ScalarType> distances;
        std::vector<CoordType> points;

         vcg::tri::GetInSphereVertex<
                    MeshType,
                    vcg::GridStaticPtr<typename MeshType::VertexType, ScalarType >,
                    std::vector<typename MeshType::VertexType*>,
                    std::vector<ScalarType>,
                    std::vector<CoordType>
                >(*P,ugridP,B4,radius,closests,distances,points);

        if(closests.empty())
            return false;
     best = -1;  bestv=std::numeric_limits<float>::max();
        for(i = 0; i <closests.size(); ++i){
         ScalarType angle = fabs((closests[i]->P() - B[1]).normalized().dot(n));
            if( angle < bestv){
                bestv = angle;
                best = i;
            }
        }
        B[3] =  closests[best]->P();

//printf("B[3] %d\n", (typename MeshType::VertexType*)closests[best] - &(*P->vert.begin()));

        // compute r1 and r2
        CoordType x;
        std::swap(B[1],B[2]);
        IntersectionLineLine(B[0],B[1],B[2],B[3],x);

        r1 = (x - B[0]).dot(B[1]-B[0]) / (B[1]-B[0]).SquaredNorm();
        r2 = (x - B[2]).dot(B[3]-B[2]) / (B[3]-B[2]).SquaredNorm();

        if( ((B[0]+(B[1]-B[0])*r1)-(B[2]+(B[3]-B[2])*r2)).Norm() > par.delta )
            return false;

        radius  =side*0.5;
        std::vector< CoordType > samples,d_samples;
        std::vector<ScalarType > dists;

        for(int i  = 0 ; i< 4; ++i){
            vcg::tri::GetKClosestVertex<
                MeshType,
                vcg::GridStaticPtr<typename MeshType::VertexType, ScalarType >,
                std::vector<VertexType*>,
                std::vector<ScalarType>,
                std::vector< CoordType > >(*P,ugridP, par.feetsize ,B[i],radius, ExtB[i],dists, samples);
        }

    //for(int i  = 0 ; i< 4; ++i)
 //		printf("%d ",ExtB[i].size());
    //	printf("\n");
return true;

}


template <class MeshType>
bool FourPCS<MeshType>::IsTransfCongruent(FourPoints fp, vcg::Matrix44<ScalarType> & mat, float &  trerr){

  std::vector<vcg::Point3<ScalarType> > fix;
  std::vector<vcg::Point3<ScalarType> > mov;
  for(int i = 0 ; i < 4; ++i) mov.push_back(B[i]);
  for(int i = 0 ; i < 4; ++i) fix.push_back(fp[i]);

  vcg::Point3<ScalarType> n,p;
  n = (( B[1]-B[0]).normalized() ^ ( B[2]- B[0]).normalized())*( B[1]- B[0]).Norm();
  p =  B[0] + n;
  mov.push_back(p);
  n = (( fp[1]-fp[0]).normalized() ^ (fp[2]- fp[0]).normalized())*( fp[1]- fp[0]).Norm();
  p =  fp[0] + n;
  fix.push_back(p);

  vcg::ComputeRigidMatchMatrix(fix,mov,mat);

  ScalarType err = 0.0;
  for(int i = 0; i < 4; ++i) err+= (mat * mov[i] - fix[i]).SquaredNorm();

  trerr = vcg::math::Sqrt(err);
  return  err  < par.delta* par.delta*4.0;
}

template <class MeshType>
void
FourPCS<MeshType>::ComputeR1R2(ScalarType d1,ScalarType d2){
    int vi,vj;
    R1.clear();
    //R2.clear();
    int start = clock();
    for(vi = 0; vi  < mapsub.size(); ++vi) for(vj = vi; vj < mapsub.size(); ++vj){
            ScalarType d = ((Q->vert[mapsub[vi]]).P()-(Q->vert[mapsub[vj]]).P()).Norm();
            if( (d < d1+ side*0.5) && (d > d1-side*0.5))
            {
                R1.push_back(Couple(mapsub[vi],mapsub[vj],d ));
                R1.push_back(Couple(mapsub[vj],mapsub[vi],d));
            }
    }
    //for( vi  = 0;  vi   < mapsub.size(); ++ vi ) for( vj  =  vi ;  vj  < mapsub.size(); ++ vj ){
    //		ScalarType d = ((Q->vert[mapsub[vi]]).P()-(Q->vert[mapsub[vj]]).P()).Norm();
    //	 	if( (d < d2+side*0.5) && (d > d2-side*0.5))
    //		{
    //			R2.push_back(Couple(mapsub[vi],mapsub[vj],d));
    //			R2.push_back(Couple(mapsub[vj],mapsub[vi],d));
    //		}
    //}

    std::sort(R1.begin(),R1.end());
//	std::sort(R2.begin(),R2.end());
}

template <class MeshType>
bool FourPCS<MeshType>::FindCongruent() { // of base B, on Q, with approximation delta
    bool done = false;
    std::vector<EPoint> R2inv;
    int n_closests = 0, n_congr = 0;
    int ac =0 ,acf = 0,tr = 0,trf =0;
    ScalarType d1,d2;
    d1 = (B[1]-B[0]).Norm();
    d2 = (B[3]-B[2]).Norm();

    int start = clock();
    //int vi,vj;

    typename PMesh::VertexIterator vii;
    typename std::vector<Couple>::iterator bR1,eR1,bR2,eR2,ite,cite;
    bR1 = std::lower_bound<typename std::vector<Couple>::iterator,Couple>(R1.begin(),R1.end(),Couple(d1-par.delta*2.0));
    eR1 = std::lower_bound<typename std::vector<Couple>::iterator,Couple>(R1.begin(),R1.end(),Couple(d1+par.delta*2.0));
    bR2 = std::lower_bound<typename std::vector<Couple>::iterator,Couple>(R1.begin(),R1.end(),Couple(d2-par.delta*2.0));
    eR2 = std::lower_bound<typename std::vector<Couple>::iterator,Couple>(R1.begin(),R1.end(),Couple(d2+par.delta*2.0));

    // in  [bR1,eR1) there are all the pairs ad a distance d1 +- par.delta
    // in  [bR1,eR1) there are all the pairs ad a distance d2 +- par.delta

    if(bR1 == R1.end()) return false;// if there are no such pairs return
    if(bR2 == R1.end()) return false; // if there are no such pairs return

    // put [bR1,eR1) in a mesh to have the search operator for free (lazy me)
    Invr.Clear();
    int i = &(*bR1)-&(*R1.begin());
    for(ite = bR1; ite != eR1;++ite){
        vii = vcg::tri::Allocator<PMesh>::AddVertices(Invr,1);
        (*vii).P() = Q->vert[R1[i][0]].P() + (Q->vert[R1[i][1]].P()-Q->vert[R1[i][0]].P()) * r1;
        ++i;
    }
    if(Invr.vert.empty() ) return false;

    // index remaps a vertex of Invr to its corresponding point in R1
    typename PMesh::template PerVertexAttributeHandle<int> id = vcg::tri::Allocator<PMesh>::template AddPerVertexAttribute<int>(Invr,std::string("index"));
    i = &(*bR1)-&(*R1.begin());
    for(vii = Invr.vert.begin(); vii != Invr.vert.end();++vii,++i)  id[vii] = i;

    vcg::tri::UpdateBounding<PMesh>::Box(Invr);
    //	printf("Invr size %d\n",Invr.vn);

    ugrid = new GridType();
    ugrid->Set(Invr.vert.begin(),Invr.vert.end());

    i = &(*bR2)-&(*R1.begin());
    // R2inv contains all the points generated by the couples in R2 (with the reference to remap into R2)
    for(ite = bR2; ite != eR2;++ite){
        R2inv.push_back( EPoint( Q->vert[R1[i][0]].P() + (Q->vert[R1[i][1]].P()-Q->vert[R1[i][0]].P()) * r2,i));
        ++i;
    }

    n_closests = 0; n_congr = 0; ac =0 ; acf = 0; tr = 0; trf = 0;
    printf("R2Inv.size  = %d \n",R2inv.size());
    for(uint i = 0 ; i < R2inv.size() ; ++i){

        std::vector<typename PMesh::VertexType*> closests;

        // for each point in R2inv get all the points in R1 closer than par.delta
        vcg::Matrix44<ScalarType> mat;
        vcg::Box3f bb;
        bb.Add(R2inv[i].pos+vcg::Point3f(par.delta * 0.1,par.delta * 0.1 , par.delta * 0.1 ));
        bb.Add(R2inv[i].pos-vcg::Point3f(par.delta * 0.1,par.delta* 0.1  , par.delta* 0.1));

        vcg::tri::GetInBoxVertex<PMesh,GridType,std::vector<typename PMesh::VertexType*> >
             (Invr,*ugrid,bb,closests);

         n_closests+=closests.size();
         for(uint ip = 0; ip < closests.size(); ++ip){
                FourPoints p;
                p[0] = Q->vert[R1[id[closests[ip]]][0]].P();
                p[1] = Q->vert[R1[id[closests[ip]]][1]].P();
                p[2] = Q->vert[R1[ R2inv[i].pi][0]].P();
                p[3] = Q->vert[R1[ R2inv[i].pi][1]].P();

                float trerr;
              n_base++;
                    if(!IsTransfCongruent(p,mat,trerr)) {
                        trf++;
                        //char name[255];
                        //sprintf(name,"faileTR_%d_%f.aln",n_base,trerr);
                        //fprintf(db,"TransCongruent %s\n", name);
                        //SaveALN(name, mat);
                    }
                    else{
                        tr++;
                    n_congr++;
                        U.push_back(Candidate(p,mat));
                        EvaluateAlignment(U.back());
                        U.back().base = bases.size()-1;

                        if( U.back().score > par.scoreFeet){
                            TestAlignment(U.back());
                            if(U.back().score > par.scoreAln)
                                {
                                    done = true; break;
                                }
                            }
                        //char name[255];
                        //sprintf(name,"passed_score_%5d_%d.aln",U.back().score,n_base);
                        //fprintf(db,"OK TransCongruent %s, score: %d \n", name,U.back().score);
                        //SaveALN(name, mat);
                    }
                }
     }

     delete ugrid;
     vcg::tri::Allocator<PMesh>::DeletePerVertexAttribute(Invr,id);
     printf("n_closests %5d = (An %5d ) + ( Tr %5d ) + (OK) %5d\n",n_closests,acf,trf,n_congr);

     return done;
//	 printf("done n_closests %d congr %d in %f s\n ",n_closests,n_congr,(clock()-start)/(float)CLOCKS_PER_SEC);
//	 printf("angle:%d %d, trasf %d %d\n",ac,acf,tr,trf);
}



template <class MeshType>
int FourPCS<MeshType>::EvaluateSample(Candidate & fp, CoordType & tp, CoordType & np, const float &  cosAngle)
{
  VertexType*   v;
  ScalarType   dist ;
  radius = par.delta;
  tp = fp.T * tp;

  vcg::Point4<ScalarType> np4;
  np4 = fp.T * vcg::Point4<ScalarType>(np[0],np[1],np[2],0.0);
  np[0] = np4[0]; np[1] = np4[1]; 	np[2] = np4[2];

  v = 0;
  //v = vcg::tri::GetClosestVertex<
  //	MeshType,
  //	vcg::GridStaticPtr<typename MeshType::VertexType, ScalarType >
  //  >(*Q,ugridQ,tp,radius,  dist  );
  typename MeshType::VertexType vq;
  vq.P() = tp;
  vq.N() = np;
  v = vcg::tri::GetClosestVertexNormal<
      MeshType,
      vcg::GridStaticPtr<typename MeshType::VertexType, ScalarType >
      >(*Q,ugridQ,vq,radius,  dist  );

  if(v!=0)
    if( v->N().dot(np) - cosAngle >0)  return 1; else return -1;

}


template <class MeshType>
void
FourPCS<MeshType>::EvaluateAlignment(Candidate  & fp){
        int n_delta_close = 0;
        for(int i  = 0 ; i< 4; ++i) {
            for(uint j = 0; j < ExtB[i].size();++j){
                CoordType np = ExtB[i][j]->cN();;
                CoordType tp  = ExtB[i][j]->P();
                n_delta_close+=EvaluateSample(fp,tp,np,0.9);
            }
        }
        fp.score = n_delta_close;
}

template <class MeshType>
void
FourPCS<MeshType>::TestAlignment(Candidate  & fp){
        radius = par.delta;
        int n_delta_close = 0;
        for(uint j = 0; j < subsetP.size();++j){
                CoordType np = subsetP[j]->N();
                CoordType tp  = subsetP[j]->P();
                n_delta_close+=EvaluateSample(fp,tp,np,0.6);
             }
        fp.score =  n_delta_close;
}


template <class MeshType>
bool
FourPCS<MeshType>::	 Align(  int L, vcg::Matrix44f & result, vcg::CallBackPos * cb ){ // main loop

    int bestv = 0;
    bool found;
    int n_tries = 0;
    U.clear();

    if(L==0)
    {
        L = (log(1.0-0.9999) / log(1.0-pow((float)par.f,3.f)))+1;
        printf("using %d bases\n",L);
    }

    ComputeR1R2(side*1.4,side*1.4);

    for(int t  = 0; t  < L; ++t ){
        do{
            n_tries = 0;
            do{
                n_tries++;
                found = SelectCoplanarBase();
                }
                while(!found && (n_tries <50));
                if(!found) {
                    par.f*=0.98;
                    side = P->bbox.Dim()[P->bbox.MaxDim()]*par.f; //rough implementation
                    ComputeR1R2(side*1.4,side*1.4);
                }
        } while (!found && (par.f >0.1));

        if(par.f <0.1) {
            printf("FAILED");
            return false;
            }
        bases.push_back(B);
        if(cb) cb(t*100/L,"trying bases");
        if(FindCongruent())
            break;
    }

    if(U.empty()) return false;

    std::sort(U.begin(),U.end());

    bestv  = -std::numeric_limits<float>::max();
    iwinner = 0;

    for(int i = 0 ; i <  U.size() ;++i)
     {
        TestAlignment(U[i]);
        if(U[i].score > bestv){
            bestv = U[i].score;
            iwinner = i;
            }
    }

    printf("Best score: %d \n", bestv);

    winner =  U[iwinner];
    result = winner.T;

    // deallocations
    Invr.Clear();

    return true;
}

    } // namespace tri
} // namespace vcg
#endif
