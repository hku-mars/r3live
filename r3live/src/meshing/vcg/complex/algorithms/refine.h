/***********F*****************************************************************
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

#ifndef __VCGLIB_REFINE
#define __VCGLIB_REFINE

#include <functional>
#include <map>
#include <vcg/space/sphere3.h>
#include <vcg/space/plane3.h>
#include <vcg/simplex/face/pos.h>
#include <vcg/simplex/face/topology.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/flag.h>
#include <vcg/space/triangle3.h>

namespace vcg{
namespace tri{

/* A very short intro about the generic refinement framework,
    the main fuction is the

 template<class MESH_TYPE,class MIDPOINT, class EDGEPRED>
 bool RefineE(MESH_TYPE &m, MIDPOINT mid, EDGEPRED ep,bool RefineSelected=false, CallBackPos *cb = 0)

 You have to provide two functor objects to this, one for deciding what edge has to be spltted and one to decide position and new values for the attributes of the new point.

 for example the minimal EDGEPRED is

 template <class MESH_TYPE, class FLT> class EdgeLen
 {
   public:
     FLT thr2;
     bool operator()(face::Pos<typename MESH_TYPE::FaceType> ep) const
     {
            return SquaredDistance(ep.f->V(ep.z)->P(), ep.f->V1(ep.z)->P())>thr2;
     }
 };

 With a bit of patience you can customize to make also slicing operation.

*/


/* The table which encodes how to subdivide a triangle depending
   on the splitted edges is organized as such:

    TriNum (the first number):    encodes the number of triangles
    TV (the following 4 triples): encodes the resulting triangles where
          0, 1, 2 are the original vertices of the triangles and 3, 4, 5
          (mp01, mp12, mp20) are the midpoints of the three edges.

   In the case two edges are splitted the triangle has 2 possible splittings:
we need to choose a diagonal of the resulting trapezoid.
'swap' encodes the two diagonals to test: if diag1 < diag2 we swap the diagonal
like this (140, 504 -> 150, 514) (the second vertex of each triangles is replaced
 by the first vertex of the other one).
            2
           / \
          5---4
         /     \
        0-------1

*/

class Split {
public:
    int TriNum;			// number of triangles
    int TV[4][3];   // The triangles coded as the following convention
                                    //     0..2 vertici originali del triangolo
                                    //     3..5 mp01, mp12, mp20 midpoints of the three edges
    int swap[2][2]; // the two diagonals to test for swapping
    int TE[4][3];   // the edge-edge correspondence between refined triangles and the old one
                                    //      (3) means the edge of the new triangle is internal;
};

const Split SplitTab[8]={
/* m20 m12 m01 */
/*  0   0   0 */	{1, {{0,1,2},{0,0,0},{0,0,0},{0,0,0}}, {{0,0},{0,0}},  {{0,1,2},{0,0,0},{0,0,0},{0,0,0}} },
/*  0   0   1 */	{2, {{0,3,2},{3,1,2},{0,0,0},{0,0,0}}, {{0,0},{0,0}},  {{0,3,2},{0,1,3},{0,0,0},{0,0,0}} },
/*  0   1   0 */	{2, {{0,1,4},{0,4,2},{0,0,0},{0,0,0}}, {{0,0},{0,0}},  {{0,1,3},{3,1,2},{0,0,0},{0,0,0}} },
/*  0   1   1 */	{3, {{3,1,4},{0,3,2},{4,2,3},{0,0,0}}, {{0,4},{3,2}},  {{0,1,3},{0,3,2},{1,3,3},{0,0,0}} },
/*  1   0   0 */	{2, {{0,1,5},{5,1,2},{0,0,0},{0,0,0}}, {{0,0},{0,0}},  {{0,3,2},{3,1,2},{0,0,0},{0,0,0}} },
/*  1   0   1 */	{3, {{0,3,5},{3,1,5},{2,5,1},{0,0,0}}, {{3,2},{5,1}},  {{0,3,2},{0,3,3},{2,3,1},{0,0,0}} },
/*  1   1   0 */	{3, {{2,5,4},{0,1,5},{4,5,1},{0,0,0}}, {{0,4},{5,1}},  {{2,3,1},{0,3,2},{3,3,1},{0,0,0}} },
/*  1   1   1 */	//{4, {{0,3,5},{3,1,4},{5,4,2},{3,4,5}}, {{0,0},{0,0}},  {{0,3,2},{0,1,3},{3,1,2},{3,3,3}} },
/*  1   1   1 */	{4, {{3,4,5},{0,3,5},{3,1,4},{5,4,2}}, {{0,0},{0,0}},  {{3,3,3},{0,3,2},{0,1,3},{3,1,2}} },
};


template <class MeshType>
struct BaseInterpolator
{
  typedef typename face::Pos<typename MeshType::FaceType> PosType;
  typedef typename MeshType::VertexType VertexType;
  void operator()(VertexType &, PosType  ){}
};

// Basic subdivision class
// This class must provide methods for finding the position of the newly created vertices
// In this implemenation we simply put the new vertex in the MidPoint position.
// Color and TexCoords are interpolated accordingly.
// This subdivision class allow also the correct interpolation of userdefined data by
// providing, in the constructor, an interpolator functor that will be called for each new vertex to be created.

template<class MESH_TYPE, class InterpolatorFunctorType = BaseInterpolator< MESH_TYPE> >
struct MidPoint : public   std::function<typename MESH_TYPE::CoordType (face::Pos<typename MESH_TYPE::FaceType>)>
{
     typedef typename face::Pos<typename MESH_TYPE::FaceType> PosType;
     typedef typename MESH_TYPE::VertexType VertexType;

     MidPoint(MESH_TYPE *_mp,
                InterpolatorFunctorType *_intFunc=0) {
       mp=_mp;
       intFunc =_intFunc;
     }

     MESH_TYPE *mp;
     InterpolatorFunctorType *intFunc; /// This callback is called to fill up

    void operator()(VertexType &nv, PosType  ep){
        assert(mp);
        nv.P()=   (ep.f->V(ep.z)->P()+ep.f->V1(ep.z)->P())/2.0;

        if( tri::HasPerVertexNormal(*mp))
            nv.N()= (ep.f->V(ep.z)->N()+ep.f->V1(ep.z)->N()).normalized();

        if( tri::HasPerVertexColor(*mp))
            nv.C().lerp(ep.f->V(ep.z)->C(),ep.f->V1(ep.z)->C(),.5f);

        if( tri::HasPerVertexQuality(*mp))
            nv.Q() = ((ep.f->V(ep.z)->Q()+ep.f->V1(ep.z)->Q())) / 2.0;

        if( tri::HasPerVertexTexCoord(*mp))
            nv.T().P() = ((ep.f->V(ep.z)->T().P()+ep.f->V1(ep.z)->T().P())) / 2.0;
        if(intFunc)
          (*intFunc)(nv,ep);
    }

    Color4<typename MESH_TYPE::ScalarType> WedgeInterp(Color4<typename MESH_TYPE::ScalarType> &c0, Color4<typename MESH_TYPE::ScalarType> &c1)
    {
        Color4<typename MESH_TYPE::ScalarType> cc;
        return cc.lerp(c0,c1,0.5f);
    }

    template<class FL_TYPE>
    TexCoord2<FL_TYPE,1> WedgeInterp(TexCoord2<FL_TYPE,1> &t0, TexCoord2<FL_TYPE,1> &t1)
    {
        TexCoord2<FL_TYPE,1> tmp;
        assert(t0.n()== t1.n());
        tmp.n()=t0.n();
        tmp.t()=(t0.t()+t1.t())/2.0;
        return tmp;
    }
};



template<class MESH_TYPE>
struct MidPointArc : public std::function<typename MESH_TYPE::CoordType (face::Pos<typename MESH_TYPE::FaceType>)>
{
    void operator()(typename MESH_TYPE::VertexType &nv, face::Pos<typename MESH_TYPE::FaceType> ep)
    {
        const typename MESH_TYPE::ScalarType EPS =1e-10;
        typename MESH_TYPE::CoordType vp = (ep.f->V(ep.z)->P()+ep.f->V1(ep.z)->P())/2.0;
        typename MESH_TYPE::CoordType  n = (ep.f->V(ep.z)->N()+ep.f->V1(ep.z)->N())/2.0;
        typename MESH_TYPE::ScalarType w =n.Norm();
        if(w<EPS) { nv.P()=(ep.f->V(ep.z)->P()+ep.f->V1(ep.z)->P())/2.0; return;}
        n/=w;
        typename MESH_TYPE::CoordType d0 = ep.f->V(ep.z)->P() - vp;
        typename MESH_TYPE::CoordType d1 = ep.f->V1(ep.z)->P()- vp;
        typename MESH_TYPE::ScalarType d=Distance(ep.f->V(ep.z)->P(),ep.f->V1(ep.z)->P())/2.0;

        typename MESH_TYPE::CoordType  nn = ep.f->V1(ep.z)->N() ^ ep.f->V(ep.z)->N();
        typename MESH_TYPE::CoordType  np = n ^ d0; //vector perpendicular to the edge plane, normal is interpolated
        np.Normalize();
        double sign=1;
        if(np*nn<0) sign=-1; // se le normali non divergono sposta il punto nella direzione opposta

        typename MESH_TYPE::CoordType n0=ep.f->V(ep.z)->N() -np*(ep.f->V(ep.z)->N()*np);
        n0.Normalize();
        typename MESH_TYPE::CoordType n1=ep.f->V1(ep.z)->N()-np*(ep.f->V1(ep.z)->N()*np);
        assert(n1.Norm()>EPS);
        n1.Normalize();
        typename MESH_TYPE::ScalarType cosa0=n0*n;
        typename MESH_TYPE::ScalarType cosa1=n1*n;
        if(2-cosa0-cosa1<EPS) {nv.P()=(ep.f->V(ep.z)->P()+ep.f->V1(ep.z)->P())/2.0;return;}
        typename MESH_TYPE::ScalarType cosb0=(d0*n)/d;
        typename MESH_TYPE::ScalarType cosb1=(d1*n)/d;
        assert(1+cosa0>EPS);
        assert(1+cosa1>EPS);
        typename MESH_TYPE::ScalarType delta0=d*(cosb0 +sqrt( ((1-cosb0*cosb0)*(1-cosa0))/(1+cosa0)) );
        typename MESH_TYPE::ScalarType delta1=d*(cosb1 +sqrt( ((1-cosb1*cosb1)*(1-cosa1))/(1+cosa1)) );
        assert(delta0+delta1<2*d);
        nv.P()=vp+n*sign*(delta0+delta1)/2.0;
        return ;
    }

    // Aggiunte in modo grezzo le due wedgeinterp
    Color4<typename MESH_TYPE::ScalarType> WedgeInterp(Color4<typename MESH_TYPE::ScalarType> &c0, Color4<typename MESH_TYPE::ScalarType> &c1)
    {
        Color4<typename MESH_TYPE::ScalarType> cc;
        return cc.lerp(c0,c1,0.5f);
    }

    template<class FL_TYPE>
    TexCoord2<FL_TYPE,1> WedgeInterp(TexCoord2<FL_TYPE,1> &t0, TexCoord2<FL_TYPE,1> &t1)
    {
        TexCoord2<FL_TYPE,1> tmp;
        assert(t0.n()== t1.n());
        tmp.n()=t0.n();
        tmp.t()=(t0.t()+t1.t())/2.0;
        return tmp;
    }

};

/*
Versione Della Midpoint basata sul paper:
S. Karbacher, S. Seeger, G. Hausler
A non linear subdivision scheme for triangle meshes

    Non funziona!
    Almeno due problemi:
    1) il verso delle normali influenza il risultato (e.g. si funziona solo se le normali divergono)
         Risolvibile controllando se le normali divergono
  2) gli archi vanno calcolati sul piano definito dalla normale interpolata e l'edge.
         funziona molto meglio nelle zone di sella e non semplici.

*/
template<class MESH_TYPE>
struct MidPointArcNaive : public std::function<typename MESH_TYPE::CoordType (face::Pos<typename MESH_TYPE::FaceType>)>
{
    typename MESH_TYPE::CoordType operator()(face::Pos<typename MESH_TYPE::FaceType>  ep)
    {

        typename MESH_TYPE::CoordType vp = (ep.f->V(ep.z)->P()+ep.f->V1(ep.z)->P())/2.0;
        typename MESH_TYPE::CoordType  n = (ep.f->V(ep.z)->N()+ep.f->V1(ep.z)->N())/2.0;
        n.Normalize();
        typename MESH_TYPE::CoordType d0 = ep.f->V(ep.z)->P() - vp;
        typename MESH_TYPE::CoordType d1 = ep.f->V1(ep.z)->P()- vp;
        typename MESH_TYPE::ScalarType d=Distance(ep.f->V(ep.z)->P(),ep.f->V1(ep.z)->P())/2.0;

        typename MESH_TYPE::ScalarType cosa0=ep.f->V(ep.z)->N()*n;
        typename MESH_TYPE::ScalarType cosa1=ep.f->V1(ep.z)->N()*n;
        typename MESH_TYPE::ScalarType cosb0=(d0*n)/d;
        typename MESH_TYPE::ScalarType cosb1=(d1*n)/d;

        typename MESH_TYPE::ScalarType delta0=d*(cosb0 +sqrt( ((1-cosb0*cosb0)*(1-cosa0))/(1+cosa0)) );
        typename MESH_TYPE::ScalarType delta1=d*(cosb1 +sqrt( ((1-cosb1*cosb1)*(1-cosa1))/(1+cosa1)) );

        return vp+n*(delta0+delta1)/2.0;
    }
};


// Basic Predicate that tells if a given edge must be splitted.
// the constructure requires the threshold.
// VERY IMPORTANT REQUIREMENT: this function must be symmetric
// e.g. it must return the same value if the Pos is VFlipped.
// If this function is not symmetric the Refine can crash.

template <class MESH_TYPE, class FLT>
class EdgeLen
{
    FLT squaredThr;
public:
    EdgeLen(){};
    EdgeLen(FLT threshold) {setThr(threshold);}
    void setThr(FLT threshold) {squaredThr = threshold*threshold; }
    bool operator()(face::Pos<typename MESH_TYPE::FaceType> ep) const
    {
        return SquaredDistance(ep.V()->P(), ep.VFlip()->P())>squaredThr;
    }
};

/*********************************************************/
/*********************************************************

Given a mesh the following function refines it according to two functor objects:

- a predicate that tells if a given edge must be splitted

- a functor that gives you the new poistion of the created vertices (starting from an edge)

If RefineSelected is true only selected faces are taken into account for being splitted.

Requirement: FF Adjacency and Manifoldness

**********************************************************/
/*********************************************************/
template <class VertexPointer>
class RefinedFaceData
    {
        public:
        RefinedFaceData(){
            ep[0]=0;ep[1]=0;ep[2]=0;
            vp[0]=0;vp[1]=0;vp[2]=0;
        }
        bool ep[3];
        VertexPointer vp[3];
    };

template<class MESH_TYPE,class MIDPOINT, class EDGEPRED>
bool RefineE(MESH_TYPE &m, MIDPOINT &mid, EDGEPRED &ep,bool RefineSelected=false, CallBackPos *cb = 0)
{
    // common typenames
    typedef typename MESH_TYPE::VertexIterator VertexIterator;
    typedef typename MESH_TYPE::FaceIterator FaceIterator;
    typedef typename MESH_TYPE::VertexPointer VertexPointer;
    typedef typename MESH_TYPE::FacePointer FacePointer;
    typedef typename MESH_TYPE::FaceType FaceType;
    typedef typename MESH_TYPE::FaceType::TexCoordType TexCoordType;
    assert(tri::HasFFAdjacency(m));
    tri::UpdateFlags<MESH_TYPE>::FaceBorderFromFF(m);
    typedef face::Pos<FaceType>  PosType;

    int j,NewVertNum=0,NewFaceNum=0;

    typedef RefinedFaceData<VertexPointer> RFD;
    typedef typename MESH_TYPE :: template PerFaceAttributeHandle<RFD> HandleType;
    HandleType RD  = tri::Allocator<MESH_TYPE>:: template AddPerFaceAttribute<RFD> (m,std::string("RefineData"));

    // Callback stuff
    int step=0;
    int PercStep=std::max(1,m.fn/33);

    // First Loop: We analyze the mesh to compute the number of the new faces and new vertices
    FaceIterator fi;
  for(fi=m.face.begin(),j=0;fi!=m.face.end();++fi) if(!(*fi).IsD())
    {
        if(cb && (++step%PercStep)==0) (*cb)(step/PercStep,"Refining...");
        // skip unselected faces if necessary
        if(RefineSelected && !(*fi).IsS()) continue;

        for(j=0;j<3;j++)
            {
                if(RD[fi].ep[j]) continue;

                PosType edgeCur(&*fi,j);
                if(RefineSelected && ! edgeCur.FFlip()->IsS()) continue;
                if(!ep(edgeCur)) continue;

                RD[edgeCur.F()].ep[edgeCur.E()]=true;
                ++NewFaceNum;
                ++NewVertNum;
                assert(edgeCur.IsManifold());
                if(!edgeCur.IsBorder())
                {
                    edgeCur.FlipF();
                    edgeCur.F()->SetV();
                    RD[edgeCur.F()].ep[edgeCur.E()]=true;
                    ++NewFaceNum;
                }
            }

  } // end face loop

    if(NewVertNum ==0 )
        {
            tri::Allocator<MESH_TYPE> :: template DeletePerFaceAttribute<RefinedFaceData<VertexPointer> >  (m,RD);
            return false;
        }
    VertexIterator lastv = tri::Allocator<MESH_TYPE>::AddVertices(m,NewVertNum);

    // Secondo loop: We initialize a edge->vertex map

    for(fi=m.face.begin();fi!=m.face.end();++fi) if(!(*fi).IsD())
  {
       if(cb && (++step%PercStep)==0)(*cb)(step/PercStep,"Refining...");
     for(j=0;j<3;j++)
         {
                // skip unselected faces if necessary
                if(RefineSelected && !(*fi).IsS()) continue;
                for(j=0;j<3;j++)
                {
                    PosType edgeCur(&*fi,j);
                    if(RefineSelected && ! edgeCur.FFlip()->IsS()) continue;

                    if( RD[edgeCur.F()].ep[edgeCur.E()] &&  RD[edgeCur.F()].vp[edgeCur.E()] ==0 )
                    {
                        RD[edgeCur.F()].vp[edgeCur.E()] = &*lastv;
                        mid(*lastv,edgeCur);
                        if(!edgeCur.IsBorder())
                        {
                            edgeCur.FlipF();
                            assert(RD[edgeCur.F()].ep[edgeCur.E()]);
                            RD[edgeCur.F()].vp[edgeCur.E()] = &*lastv;
                        }
                        ++lastv;
                    }
                }
         }
  }

    assert(lastv==m.vert.end()); // critical assert: we MUST have used all the vertex that we forecasted we need

    FaceIterator lastf = tri::Allocator<MESH_TYPE>::AddFaces(m,NewFaceNum);
    FaceIterator oldendf = lastf;

/*
 *               v0
 *
 *
 *               f0
 *
 *       mp01     f3     mp02
 *
 *
 *       f1               f2
 *
 *v1            mp12                v2
 *
*/

    VertexPointer vv[6];	// The six vertices that arise in the single triangle splitting
    //     0..2 Original triangle vertices
    //     3..5 mp01, mp12, mp20 midpoints of the three edges
    FacePointer nf[4];   // The (up to) four faces that are created.

    TexCoordType wtt[6];  // per ogni faccia sono al piu' tre i nuovi valori
    // di texture per wedge (uno per ogni edge)

    int fca=0,fcn =0;
    for(fi=m.face.begin();fi!=oldendf;++fi) if(!(*fi).IsD())
    {
      if(cb && (++step%PercStep)==0)(*cb)(step/PercStep,"Refining...");
      fcn++;
      vv[0]=(*fi).V(0);
      vv[1]=(*fi).V(1);
      vv[2]=(*fi).V(2);
      vv[3] = RD[fi].vp[0];
      vv[4] = RD[fi].vp[1];
      vv[5] = RD[fi].vp[2];

      int ind=((&*vv[3])?1:0)+((&*vv[4])?2:0)+((&*vv[5])?4:0);

      nf[0]=&*fi;
      int i;
      for(i=1;i<SplitTab[ind].TriNum;++i){
        nf[i]=&*lastf; ++lastf; fca++;
        if(RefineSelected || (*fi).IsS()) (*nf[i]).SetS();
        nf[i]->ImportData(*fi);
//		if(tri::HasPerFaceColor(m))
//  		nf[i]->C()=(*fi).cC();
      }


      if(tri::HasPerWedgeTexCoord(m))
        for(i=0;i<3;++i)	{
          wtt[i]=(*fi).WT(i);
          wtt[3+i]=mid.WedgeInterp((*fi).WT(i),(*fi).WT((i+1)%3));
        }

      int orgflag=	(*fi).Flags();
      for(i=0;i<SplitTab[ind].TriNum;++i)
        for(j=0;j<3;++j){
          (*nf[i]).V(j)=&*vv[SplitTab[ind].TV[i][j]];

          if(tri::HasPerWedgeTexCoord(m)) //analogo ai vertici...
            (*nf[i]).WT(j)=wtt[SplitTab[ind].TV[i][j]];

          assert((*nf[i]).V(j)!=0);
          if(SplitTab[ind].TE[i][j]!=3){
            if(orgflag & (MESH_TYPE::FaceType::BORDER0<<(SplitTab[ind].TE[i][j])))
              (*nf[i]).SetB(j);
            else
              (*nf[i]).ClearB(j);
          }
          else (*nf[i]).ClearB(j);
        }

      if(SplitTab[ind].TriNum==3 &&
         SquaredDistance(vv[SplitTab[ind].swap[0][0]]->P(),vv[SplitTab[ind].swap[0][1]]->P()) <
         SquaredDistance(vv[SplitTab[ind].swap[1][0]]->P(),vv[SplitTab[ind].swap[1][1]]->P()) )
      { // swap the last two triangles
        (*nf[2]).V(1)=(*nf[1]).V(0);
        (*nf[1]).V(1)=(*nf[2]).V(0);
        if(tri::HasPerWedgeTexCoord(m)){ //swap also textures coordinates
          (*nf[2]).WT(1)=(*nf[1]).WT(0);
          (*nf[1]).WT(1)=(*nf[2]).WT(0);
        }

        if((*nf[1]).IsB(0)) (*nf[2]).SetB(1); else (*nf[2]).ClearB(1);
        if((*nf[2]).IsB(0)) (*nf[1]).SetB(1); else (*nf[1]).ClearB(1);
        (*nf[1]).ClearB(0);
        (*nf[2]).ClearB(0);
      }
    }

    assert(lastf==m.face.end());	 // critical assert: we MUST have used all the faces that we forecasted we need and that we previously allocated.
    assert(!m.vert.empty());
    for(fi=m.face.begin();fi!=m.face.end();++fi) if(!(*fi).IsD()){
      assert((*fi).V(0)>=&*m.vert.begin() && (*fi).V(0)<=&m.vert.back() );
      assert((*fi).V(1)>=&*m.vert.begin() && (*fi).V(1)<=&m.vert.back() );
      assert((*fi).V(2)>=&*m.vert.begin() && (*fi).V(2)<=&m.vert.back() );
    }
    tri::UpdateTopology<MESH_TYPE>::FaceFace(m);

    tri::Allocator<MESH_TYPE> :: template DeletePerFaceAttribute<RefinedFaceData<VertexPointer> >  (m,RD);

    return true;
}

/*************************************************************************/
// simple wrapper of the base refine for lazy coder that do not need a edge predicate

template<class MESH_TYPE,class MIDPOINT>
bool Refine(MESH_TYPE &m, MIDPOINT mid, typename MESH_TYPE::ScalarType thr=0,bool RefineSelected=false, CallBackPos *cb = 0)
{
    EdgeLen <MESH_TYPE, typename MESH_TYPE::ScalarType> ep(thr);
  return RefineE(m,mid,ep,RefineSelected,cb);
}
/*************************************************************************/

/*
Modified Butterfly interpolation scheme,
as presented in
Zorin, Schroeder
Subdivision for modeling and animation
Siggraph 2000 Course Notes
*/

/*

    vul-------vu--------vur
      \      /  \      /
       \    /    \    /
        \  /  fu  \  /
         vl--------vr
        /  \  fd  /  \
       /    \    /    \
      /      \  /      \
    vdl-------vd--------vdr

*/

template<class MESH_TYPE>
struct MidPointButterfly : public std::function<typename MESH_TYPE::CoordType (face::Pos<typename MESH_TYPE::FaceType>)>
{
  MESH_TYPE &m;
  MidPointButterfly(MESH_TYPE &_m):m(_m){}

    void operator()(typename MESH_TYPE::VertexType &nv, face::Pos<typename MESH_TYPE::FaceType>  ep)
    {
        face::Pos<typename MESH_TYPE::FaceType> he(ep.f,ep.z,ep.f->V(ep.z));
        typename MESH_TYPE::CoordType *vl,*vr;
        typename MESH_TYPE::CoordType *vl0,*vr0;
        typename MESH_TYPE::CoordType *vu,*vd,*vul,*vur,*vdl,*vdr;
        vl=&he.v->P();
        he.FlipV();
        vr=&he.v->P();

        if( tri::HasPerVertexColor(m))
            nv.C().lerp(ep.f->V(ep.z)->C(),ep.f->V1(ep.z)->C(),.5f);

        if(he.IsBorder())
        {
            he.NextB();
            vr0=&he.v->P();
            he.FlipV();
            he.NextB();
            assert(&he.v->P()==vl);
            he.NextB();
            vl0=&he.v->P();
            nv.P()=((*vl)+(*vr))*(9.0/16.0)-((*vl0)+(*vr0))/16.0 ;
        }
        else
        {
            he.FlipE();he.FlipV();
            vu=&he.v->P();
            he.FlipF();he.FlipE();he.FlipV();
            vur=&he.v->P();
            he.FlipV();he.FlipE();he.FlipF(); assert(&he.v->P()==vu); // back to vu (on the right)
            he.FlipE();
            he.FlipF();he.FlipE();he.FlipV();
            vul=&he.v->P();
            he.FlipV();he.FlipE();he.FlipF(); assert(&he.v->P()==vu); // back to vu (on the left)
            he.FlipV();he.FlipE();he.FlipF(); assert(&he.v->P()==vl);// again on vl (but under the edge)
            he.FlipE();he.FlipV();
            vd=&he.v->P();
            he.FlipF();he.FlipE();he.FlipV();
            vdl=&he.v->P();
            he.FlipV();he.FlipE();he.FlipF(); assert(&he.v->P()==vd);// back to vd (on the right)
            he.FlipE();
            he.FlipF();he.FlipE();he.FlipV();
            vdr=&he.v->P();

            nv.P()=((*vl)+(*vr))/2.0+((*vu)+(*vd))/8.0 - ((*vul)+(*vur)+(*vdl)+(*vdr))/16.0;
            }
    }

    /// Aggiunte in modo grezzo le due wedge interp
    Color4<typename MESH_TYPE::ScalarType> WedgeInterp(Color4<typename MESH_TYPE::ScalarType> &c0, Color4<typename MESH_TYPE::ScalarType> &c1)
    {
        Color4<typename MESH_TYPE::ScalarType> cc;
        return cc.lerp(c0,c1,0.5f);
    }

    template<class FL_TYPE>
    TexCoord2<FL_TYPE,1> WedgeInterp(TexCoord2<FL_TYPE,1> &t0, TexCoord2<FL_TYPE,1> &t1)
    {
        TexCoord2<FL_TYPE,1> tmp;
        assert(t0.n()== t1.n());
        tmp.n()=t0.n();
        tmp.t()=(t0.t()+t1.t())/2.0;
        return tmp;
    }
};


#if 0
            int rule=0;
            if(vr==vul) rule+=1;
            if(vl==vur) rule+=2;
            if(vl==vdr) rule+=4;
            if(vr==vdl) rule+=8;
            switch(rule){
/*      */
/*      */			case  0 :	return ((*vl)+(*vr))/2.0+((*vu)+(*vd))/8.0 - ((*vul)+(*vur)+(*vdl)+(*vdr))/16.0;
/* ul   */  		case  1 : return (*vl*6 + *vr*10 + *vu + *vd*3 - *vur - *vdl -*vdr*2 )/16.0;
/* ur   */  		case  2 : return (*vr*6 + *vl*10 + *vu + *vd*3 - *vul - *vdr -*vdl*2 )/16.0;
/* dr   */  		case  4 : return (*vr*6 + *vl*10 + *vd + *vu*3 - *vdl - *vur -*vul*2 )/16.0;
/* dl   */  		case  8 : return (*vl*6 + *vr*10 + *vd + *vu*3 - *vdr - *vul -*vur*2 )/16.0;
/* ul,ur */  		case  3 : return (*vl*4 + *vr*4 + *vd*2 + - *vdr - *vdl )/8.0;
/* dl,dr */  		case 12 : return (*vl*4 + *vr*4 + *vu*2 + - *vur - *vul )/8.0;

/* ul,dr */  		case  5 :
/* ur,dl */  		case 10 :
                                default:
                                    return (*vl+ *vr)/2.0;
            }



#endif
/*
    vul-------vu--------vur
          \      /  \      /
             \    /    \    /
        \  /  fu  \  /
         vl--------vr
        /  \  fd  /  \
       /    \    /    \
      /      \  /      \
    vdl-------vd--------vdr

*/

// Versione modificata per tenere di conto in meniara corretta dei vertici con valenza alta

template<class MESH_TYPE>
struct MidPointButterfly2 : public std::function<typename MESH_TYPE::CoordType (face::Pos<typename MESH_TYPE::FaceType>)>
{
    typename MESH_TYPE::CoordType operator()(face::Pos<typename MESH_TYPE::FaceType>  ep)
    {
double Rules[11][10] =
{
    {.0}, // valenza 0
    {.0}, // valenza 1
    {.0}, // valenza 2
    {  .4166666667, -.08333333333 , -.08333333333  }, // valenza 3
    {  .375       ,  .0           ,  -0.125        ,  .0          }, // valenza 4
    {  .35        ,  .03090169945 ,  -.08090169945 , -.08090169945,  .03090169945	}, // valenza 5
    {  .5         ,  .125         ,  -0.0625       ,  .0          ,  -0.0625      , 0.125       }, // valenza 6
    {  .25        ,  .1088899050  , -.06042933822  , -.04846056675, -.04846056675, -.06042933822,  .1088899050  }, // valenza 7
    {  .21875     ,  .1196383476  , -.03125        , -.05713834763, -.03125      , -.05713834763, -.03125      ,.1196383476  }, // valenza 8
    {  .1944444444,  .1225409480  , -.00513312590  , -.05555555556, -.03407448880, -.03407448880, -.05555555556, -.00513312590, .1225409480  }, // valenza 9
    {  .175       ,  .1213525492  ,  .01545084973  , -.04635254918, -.04045084973, -.025        , -.04045084973, -.04635254918,  .01545084973,  .1213525492  } // valenza 10
};

face::Pos<typename MESH_TYPE::FaceType> he(ep.f,ep.z,ep.f->V(ep.z));
    typename MESH_TYPE::CoordType *vl,*vr;
    vl=&he.v->P();
    vr=&he.VFlip()->P();
    if(he.IsBorder())
        {he.FlipV();
        typename MESH_TYPE::CoordType *vl0,*vr0;
            he.NextB();
            vr0=&he.v->P();
            he.FlipV();
            he.NextB();
            assert(&he.v->P()==vl);
            he.NextB();
            vl0=&he.v->P();
            return ((*vl)+(*vr))*(9.0/16.0)-((*vl0)+(*vr0))/16.0 ;
        }

    int kl=0,kr=0; // valence of left and right vertices
    bool bl=false,br=false; // if left and right vertices are of border
  face::Pos<typename MESH_TYPE::FaceType> heStart=he;assert(he.v->P()==*vl);
    do { // compute valence of left vertex
        he.FlipE();he.FlipF();
        if(he.IsBorder()) bl=true;
        ++kl;
    }	while(he!=heStart);

    he.FlipV();heStart=he;assert(he.v->P()==*vr);
    do { // compute valence of right vertex
        he.FlipE();he.FlipF();
        if(he.IsBorder()) br=true;
        ++kr;
    }	while(he!=heStart);
  if(br||bl) return MidPointButterfly<MESH_TYPE>()( ep );
    if(kr==6 && kl==6) return MidPointButterfly<MESH_TYPE>()( ep );
    // TRACE("odd vertex among valences of %i %i\n",kl,kr);
    typename MESH_TYPE::CoordType newposl=*vl*.75, newposr=*vr*.75;
    he.FlipV();heStart=he; assert(he.v->P()==*vl);
    int i=0;
    if(kl!=6)
    do { // compute position  of left vertex
        newposl+= he.VFlip()->P() * Rules[kl][i];
        he.FlipE();he.FlipF();
        ++i;
    }	while(he!=heStart);
    i=0;he.FlipV();heStart=he;assert(he.v->P()==*vr);
    if(kr!=6)
    do { // compute position of right vertex
        newposr+=he.VFlip()->P()* Rules[kr][i];
        he.FlipE();he.FlipF();
        ++i;
    }	while(he!=heStart);
    if(kr==6) return newposl;
    if(kl==6) return newposr;
    return newposl+newposr;
    }
};

/* The two following classes are the functor and the predicate that you need for using the refine framework to cut a mesh along a linear interpolation of the quality.
   This can be used for example to slice a mesh with a plane. Just set the quality value as distance from plane and then functor and predicate
   initialized 0 and invoke the refine

  MyMesh A;
  tri::UpdateQuality:MyMesh>::VertexFromPlane(plane);
  QualityMidPointFunctor<MyMesh> slicingfunc(0.0);
  QualityEdgePredicate<MyMesh> slicingpred(0.0);
  tri::UpdateTopology<MyMesh>::FaceFace(A);
  RefineE<MyMesh, QualityMidPointFunctor<MyMesh>, QualityEdgePredicate<MyMesh> > (A, slicingfunc, slicingpred, false);

  Note that they store in the vertex quality the plane distance.
  */

template<class MESH_TYPE>
class QualityMidPointFunctor : public std::function<typename MESH_TYPE::CoordType (face::Pos<typename MESH_TYPE::FaceType>)>
{
public:
  typedef Point3<typename MESH_TYPE::ScalarType> Point3x;
  typedef typename MESH_TYPE::ScalarType ScalarType;

  ScalarType thr;

  QualityMidPointFunctor(ScalarType _thr):thr(_thr){}


  void operator()(typename MESH_TYPE::VertexType &nv, const face::Pos<typename MESH_TYPE::FaceType> &ep){
    Point3x p0=ep.f->V0(ep.z)->P();
    Point3x p1=ep.f->V1(ep.z)->P();
    ScalarType q0=ep.f->V0(ep.z)->Q()-thr;
    ScalarType q1=ep.f->V1(ep.z)->Q()-thr;
    double pp= q0/(q0-q1);
    nv.P()=p1*pp + p0*(1.0-pp);
    nv.Q()=thr;
    }

    Color4<typename MESH_TYPE::ScalarType> WedgeInterp(Color4<typename MESH_TYPE::ScalarType> &c0, Color4<typename MESH_TYPE::ScalarType> &c1)
    {
        Color4<typename MESH_TYPE::ScalarType> cc;
        return cc.lerp(c0,c1,0.5f);
    }

    template<class FL_TYPE>
    TexCoord2<FL_TYPE,1> WedgeInterp(TexCoord2<FL_TYPE,1> &t0, TexCoord2<FL_TYPE,1> &t1)
    {
        TexCoord2<FL_TYPE,1> tmp;
        assert(t0.n()== t1.n());
        tmp.n()=t0.n();
        tmp.t()=(t0.t()+t1.t())/2.0;
        return tmp;
    }
};


template <class MESH_TYPE>
class QualityEdgePredicate
{
  public:
  typedef Point3<typename MESH_TYPE::ScalarType> Point3x;
  typedef typename MESH_TYPE::ScalarType ScalarType;
  ScalarType thr;
  QualityEdgePredicate(const ScalarType &thr):thr(thr) {}
  bool operator()(face::Pos<typename MESH_TYPE::FaceType> ep)
    {
    ScalarType q0=ep.f->V0(ep.z)->Q()-thr;
    ScalarType q1=ep.f->V1(ep.z)->Q()-thr;
    if(q0>q1) std::swap(q0,q1);
    if ( q0*q1 > 0) return false;
    // now a small check to be sure that we do not make too thin crossing.
    double pp= q0/(q0-q1);
    if(fabs(pp)< 0.001) return false;
    return true;
  }
};


template<class MESH_TYPE>
struct MidPointSphere : public std::function<typename MESH_TYPE::CoordType (face::Pos<typename MESH_TYPE::FaceType>)>
{
    Sphere3<typename MESH_TYPE::ScalarType> sph;
    typedef Point3<typename MESH_TYPE::ScalarType> Point3x;

    void operator()(typename MESH_TYPE::VertexType &nv, face::Pos<typename MESH_TYPE::FaceType>  ep){
        Point3x &p0=ep.f->V0(ep.z)->P();
        Point3x &p1=ep.f->V1(ep.z)->P();
    nv.P()= sph.c+((p0+p1)/2.0 - sph.c ).Normalize();
    }

    Color4<typename MESH_TYPE::ScalarType> WedgeInterp(Color4<typename MESH_TYPE::ScalarType> &c0, Color4<typename MESH_TYPE::ScalarType> &c1)
    {
        Color4<typename MESH_TYPE::ScalarType> cc;
        return cc.lerp(c0,c1,0.5f);
    }

    template<class FL_TYPE>
    TexCoord2<FL_TYPE,1> WedgeInterp(TexCoord2<FL_TYPE,1> &t0, TexCoord2<FL_TYPE,1> &t1)
    {
        TexCoord2<FL_TYPE,1> tmp;
        assert(t0.n()== t1.n());
        tmp.n()=t0.n();
        tmp.t()=(t0.t()+t1.t())/2.0;
        return tmp;
    }
};


template <class FLT>
class EdgeSplSphere
{
    public:
  Sphere3<FLT> sph;
    bool operator()(const Point3<FLT> &p0, const  Point3<FLT> &p1) const
    {
        if(Distance(sph,p0)>0) {
            if(Distance(sph,p1)>0) return false;
            else return true;
        }
        else if(Distance(sph,p1)<=0) return false;
        return true;
    }
};

template<class TRIMESH_TYPE>
struct CenterPointBarycenter : public std::function<typename TRIMESH_TYPE::CoordType (typename TRIMESH_TYPE::FacePointer)>
{
    typename TRIMESH_TYPE::CoordType operator()(typename TRIMESH_TYPE::FacePointer f){
        return vcg::Barycenter<typename TRIMESH_TYPE::FaceType>(*f);
    }
};

/// \brief Triangle split
/// Simple templated function for splitting a triangle with a internal point.
///  It can be templated on a CenterPoint class that is used to generate the position of the internal point.


template<class TRIMESH_TYPE, class CenterPoint=CenterPointBarycenter <TRIMESH_TYPE> >
class TriSplit
{
public:
  typedef typename TRIMESH_TYPE::FaceType FaceType;
  typedef typename TRIMESH_TYPE::VertexType VertexType;

  static void Apply(FaceType *f,
                    FaceType * f1,FaceType * f2,
                    VertexType * vB, CenterPoint	Center)
  {
    vB->P() = Center(f);

    //i tre vertici della faccia da dividere
    VertexType *V0,*V1,*V2;
    V0 = f->V(0);
    V1 = f->V(1);
    V2 = f->V(2);

    //risistemo la faccia di partenza
    (*f).V(2) = &(*vB);
    //Faccia nuova #1
    (*f1).V(0) = &(*vB);
    (*f1).V(1) = V1;
    (*f1).V(2) = V2;
    //Faccia nuova #2
    (*f2).V(0) = V0;
    (*f2).V(1) = &(*vB);
    (*f2).V(2) = V2;

    if(f->HasFFAdjacency())
    {
      //adiacenza delle facce adiacenti a quelle aggiunte
      f->FFp(1)->FFp(f->FFi(1)) = f1;
      f->FFp(2)->FFp(f->FFi(2)) = f2;

      //adiacenza ff
      FaceType *  FF0,*FF1,*FF2;
      FF0 = f->FFp(0);
      FF1 = f->FFp(1);
      FF2 = f->FFp(2);

      //Indici di adiacenza ff
      char FFi0,FFi1,FFi2;
      FFi0 = f->FFi(0);
      FFi1 = f->FFi(1);
      FFi2 = f->FFi(2);

      //adiacenza della faccia di partenza
      (*f).FFp(1) = &(*f1);
      (*f).FFi(1) = 0;
      (*f).FFp(2) = &(*f2);
      (*f).FFi(2) = 0;

      //adiacenza della faccia #1
      (*f1).FFp(0) = f;
      (*f1).FFi(0) = 1;

      (*f1).FFp(1) = FF1;
      (*f1).FFi(1) = FFi1;

      (*f1).FFp(2) = &(*f2);
      (*f1).FFi(2) = 1;

      //adiacenza della faccia #2
      (*f2).FFp(0) = f;
      (*f2).FFi(0) = 2;

      (*f2).FFp(1) = &(*f1);
      (*f2).FFi(1) = 2;

      (*f2).FFp(2) = FF2;
      (*f2).FFi(2) = FFi2;
    }
  }
}; // end class TriSplit

} // namespace tri
} // namespace vcg

#endif
