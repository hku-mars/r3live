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
/****************************************************************************
  History
  
$Log: not supported by cvs2svn $
Revision 1.6  2007/02/02 04:11:00  tarini
added parameter theta (from conformal to equiareal) to AreaPresTextureOptimizer.
Improved feature lists (comments).

Revision 1.5  2007/02/02 01:39:58  tarini
added three general-utility global functions for texture coordinates: SmoothTextureCoords, IsFoldFree, MarkFolds (see descriptions)

Revision 1.4  2007/02/02 01:23:47  tarini
added a few general comments on AreaPreserving optimizer, recapping optimizer features.

Revision 1.3  2007/02/02 01:18:15  tarini
First version: general virtual class for texture optimizers. A subclass for area preservation.


****************************************************************************/

#ifndef __VCGLIB__TEXTCOOORD_OPTIMIZATION
#define __VCGLIB__TEXTCOOORD_OPTIMIZATION

#include <vcg/container/simple_temporary_data.h>


/*

SINGLE PATCH TEXTURE OPTIMIZATIONS

A set of classes to perform optimizations of disk->disk parametrization.

Requires texture coords to be defined per vertex (replicate seams).

*/


namespace vcg
{
namespace tri
{


/* Base class for all Texture Optimizers*/
template<class MESH_TYPE> 
class TextureOptimizer{
protected:
  MESH_TYPE &m;
  SimpleTempData<typename MESH_TYPE::VertContainer, int > isFixed;
public:
  
  /* Tpyes */
  typedef MESH_TYPE MeshType;
  typedef typename MESH_TYPE::VertexIterator VertexIterator;
  typedef typename MESH_TYPE::FaceIterator FaceIterator;
  typedef typename MESH_TYPE::VertexType VertexType;
  typedef typename MESH_TYPE::FaceType FaceType;
  typedef typename MESH_TYPE::ScalarType ScalarType;
  
  
  /* Access functions */
  const MeshType & Mesh() const {return m;}
  MeshType & Mesh() {return m;}
   
  /* Constructior */
  TextureOptimizer(MeshType &_m):m(_m),isFixed(_m.vert){
    assert(m.HasPerVertexTexture());
  }
  
  // initializes on current geometry 
  virtual void TargetCurrentGeometry()=0;
  
  // performs an interation. Returns largest movement.
  virtual ScalarType Iterate()=0;
  
  // performs an iteration (faster, but it does not tell how close it is to stopping)
  virtual void IterateBlind()=0;
  
  // performs <steps> iteration
  virtual ScalarType IterateN(int step){
    for (int i=0; i<step-1; i++) {
      this->IterateBlind();
    }
    if (step>1) return this->Iterate(); else return 0;
  }
 
  // performs iterations until convergence.
  bool IterateUntilConvergence(ScalarType threshold=0.0001, int maxite=5000){
    int i;
    while (Iterate()>threshold) {
      if (i++>maxite) return false;
    }
    return true;
  }
  
  // desctuctor: free temporary field
  ~TextureOptimizer(){
    isFixed.Stop();
  };
  
  // set the current border as fixed (forced to stay in position during text optimization)
  void SetBorderAsFixed(){
    isFixed.Start();
    for (VertexIterator v=m.vert.begin(); v!=m.vert.end(); v++) {
		  isFixed[v]=(v->IsB())?1:0; 
	  }  
  }
  
  // everything moves, no vertex must fixed during texture optimization)
  void SetNothingAsFixed(){
    isFixed.Start();
    for (VertexIterator v=m.vert.begin(); v!=m.vert.end(); v++) {
		  isFixed[v]=0; 
	  }  
  }
  
  // fix a given vertex
  void FixVertex(const VertexType *v, bool fix=true){
    isFixed[v]=(fix)?1:0;
  }
  
  
};



/*
AREA PRESERVING TEXTURE OPTIMIZATION

as in: Degener, P., Meseth, J., Klein, R. 
       "An adaptable surface parameterization method."
       Proc. of the 12th International Meshing oundtable, 201–213 [2003].

Features:
  
:) - Balances angle and area distortions (best results!).
:) - Can choose how to balance area and angle preservation (see SetTheta)
       theta=0 -> pure conformal (use MIPS instead!)
       theta=3 -> good balance between area and angle preservation
       theta>3 -> care more about area than about angles
:( - Slowest method.
:( - Requires a fixed boundary, else expands forever in texture space (unless theta=0).
:( - Diverges in presence of flipped faces (unless theta=0).
:( - Requires a speed parameter to be set. 
       Speed too large => when close, bounces back and forth around minimum, w/o getting any closer.
       Lower speed => longer convercence times
*/

template<class MESH_TYPE> 
class AreaPreservingTextureOptimizer:public TextureOptimizer<MESH_TYPE>{
public:
  /* Types */
  typedef MESH_TYPE MeshType;
  typedef typename MESH_TYPE::VertexIterator VertexIterator;
  typedef typename MESH_TYPE::FaceIterator FaceIterator;
  typedef typename MESH_TYPE::VertexType VertexType;
  typedef typename MESH_TYPE::FaceType FaceType;
  typedef typename MESH_TYPE::ScalarType ScalarType;
  

private:
  typedef TextureOptimizer<MESH_TYPE> Super; // superclass (commodity)
  
  // extra data per face: [0..3] -> cotangents. [4] -> area*2
  SimpleTempData<typename MESH_TYPE::FaceContainer, Point4<ScalarType> > data;
  SimpleTempData<typename MESH_TYPE::VertContainer, Point2<ScalarType> > sum;
  
  ScalarType totArea;
  ScalarType speed;
  
  int theta;
  
public:
   
  // constructor and destructor
  AreaPreservingTextureOptimizer(MeshType &_m):Super(_m),data(_m.face),sum(_m.vert){
    speed=0.001;
    theta=3;
  }
  
  ~AreaPreservingTextureOptimizer(){
    data.Stop();
    sum.Stop();
    Super::isFixed.Stop();
  }
  
  void SetSpeed(ScalarType _speed){
    speed=_speed;
  }

  ScalarType GetSpeed(){
    return speed;
  }
  
  // sets the parameter theta:
  // good parameters are in 1..3
  //  0 = converge to pure conformal, ignore area preservation
  //  3 = good balance between area and conformal
  // >3 = area more important, angle preservation less important
  void SetTheta(int _theta){
    theta=_theta;
  }

  int GetTheta(){
    return theta;
  }
  
  void IterateBlind(){
    /* todo: do as iterate, but without */ 
    Iterate();
  }
  
  ScalarType Iterate(){
    
    ScalarType max; // max displacement
    
    #define v0 (f->V0(i)->T().P())
    #define v1 (f->V1(i)->T().P())
    #define v2 (f->V2(i)->T().P())
	  for (VertexIterator v=Super::m.vert.begin(); v!=Super::m.vert.end(); v++) {
		  sum[v].SetZero();
	  }

	  ScalarType tot_proj_area=0;
	  for (FaceIterator f=Super::m.face.begin(); f!=Super::m.face.end(); f++) {
		  int i=0;
		  double area2 = ((v1-v0) ^ (v2-v0));
		  tot_proj_area+=area2;
	  }

	  double scale= 1.0; //tot_proj_area / tot_area ;

	  for (FaceIterator f=Super::m.face.begin(); f!=Super::m.face.end(); f++) {
		  int i=0; ScalarType area2 = ((v1-v0) ^ (v2-v0));
		  for (i=0; i<3; i++){
			  ScalarType 
				  a = (v1-v0).Norm(),
				  b =  ((v1-v0) * (v2-v0))/a,
			    c = area2 / a,
			    
				  m0= data[f][i] / area2,
				  m1= data[f][(i+1)%3] / area2,
				  m2= data[f][(i+2)%3] / area2,
				  
				  mx= (b-a)/area2,
				  my= c/area2, // 1.0/a
				  mA= data[f][3]/area2 * scale,
				  e = m0*((b-a)*(b-a)+c*c) + m1*(b*b+c*c) + m2*a*a, // as obvious
				  M1= mA + 1.0/mA,
				  M2= mA - 1.0/mA,
				  px= e*my,
				  py=-e*mx,
				  qx= m1*b+ m2*a,
				  qy= m1*c,

				  /* linear weightings

				  dx= (OMEGA) * (my * M2) + 
				      (1-OMEGA) * ( px - 2.0*qx),
				  dy= (OMEGA) * (-mx * M2) + 
				      (1-OMEGA) * ( py - 2.0*qy),*/
				
				  // exponential weighting
				  // 2d gradient
				
				  dx=// M1
				  	 //*M1 // ^ theta-1 
				  	 pow(M1,theta-1)
				  	 *(px*(M1+ theta*M2) - 2.0*qx*M1), 
				  dy=// M1
					   //*M1 // ^ theta-1 
					   pow(M1,theta-1)
					   *(py*(M1+ theta*M2) - 2.0*qy*M1), 

				  gy= dy/c,
				  gx= (dx - gy*b) / a;

				  // 3d gradient

			    sum[f->V(i)]+= ( (v1-v0) * gx + (v2-v0) * gy ) * data[f][3]; 
		  }
	  }
  	max=0; // max displacement  	
  	speed=0.001;
	  for (VertexIterator v=Super::m.vert.begin(); v!=Super::m.vert.end(); v++) 
    if (  !Super::isFixed[v] ) //if (!v->IsB()) 
    {
      ScalarType n=sum[v].Norm();
		  if ( n > 1 ) { sum[v]/=n; n=1.0;}
		  if ( n*speed<=0.1 ); {
		    v->T().P()-=(sum[v] * speed ) /** scale*/;
		    if (max<n) max=n;
      }
	 	  //else rejected++;
  	}
  	return max;
  	#undef v0
    #undef v1 
    #undef v2 
  	//printf("rejected %d\n",rejected);
  }
  
  void TargetCurrentGeometry(){
    
    Super::isFixed.Start();
    data.Start();
    sum.Start();
    
	  totArea=0;
	  for (FaceIterator f=Super::m.face.begin(); f!=Super::m.face.end(); f++) {
		  double area2 = 	((f->V(1)->P() - f->V(0)->P() )^(f->V(2)->P() - f->V(0)->P() )).Norm();
		  totArea+=area2;
		  //if (  Super::isFixed[f->V1(0)] )
		  for (int i=0; i<3; i++){
			  data[f][i]=(
				(f->V1(i)->P() - f->V0(i)->P() )*(f->V2(i)->P() - f->V0(i)->P() )
			  )/area2;
			  data[f][3]=area2;
		  }
	  }
  }
  
};



/* texture coords general utility functions */
/*++++++++++++++++++++++++++++++++++++++++++*/

// returns false if any fold is present (faster than MarkFolds)
template<class MESH_TYPE>
bool IsFoldFree(MESH_TYPE &m){
  
  assert(m.HasPerVertexTexture());
  
  typedef typename MESH_TYPE::VertexType::TextureType::PointType PointType;
  typedef typename MESH_TYPE::VertexType::TextureType::PointType::ScalarType ScalarType;
  
  ScalarType lastsign=0;
  for (typename MESH_TYPE::FaceIterator f=m.face.begin(); f!=m.face.end(); f++){
    ScalarType sign=((f->V(1)->T().P()-f->V(0)->T().P()) ^ (f->V(2)->T().P()-f->V(0)->T().P()));
    if (sign!=0) {
      if (sign*lastsign<0) return false;
      lastsign=sign;
    }
  }
  return true;
}

// detects and marks folded faces, by setting their quality to 0 (or 1 otherwise)
// returns number of folded faces
template<class MESH_TYPE>
int MarkFolds(MESH_TYPE &m){
  
  assert(m.HasPerVertexTexture());
  assert(m.HasPerFaceQuality());
  
  typedef typename MESH_TYPE::VertexType::TextureType::PointType PointType;
  typedef typename MESH_TYPE::VertexType::TextureType::PointType::ScalarType ScalarType;
  
  SimpleTempData<typename MESH_TYPE::FaceContainer, short> sign(m.face);
  sign.Start(0);
  
  // first pass, determine predominant sign
  int npos=0, nneg=0;
  ScalarType lastsign=0;
  for (typename MESH_TYPE::FaceIterator f=m.face.begin(); f!=m.face.end(); f++){
    ScalarType fsign=((f->V(1)->T().P()-f->V(0)->T().P()) ^ (f->V(2)->T().P()-f->V(0)->T().P()));
    if (fsign<0) { sign[f]=-1;  nneg++; }
    if (fsign>0) { sign[f]=+1; npos++; }
  }
  
  // second pass, detect folded faces
  int res=0;
  short gsign= (nneg>npos)?-1:+1;
  for (typename MESH_TYPE::FaceIterator f=m.face.begin(); f!=m.face.end(); f++){
    if (sign[f]*gsign<0){
      res++;
      f->Q()=0;
    } else f->Q()=1;
  }
  
  sign.Stop();
  
  return res;
}

// Smooths texture coords.
// (can be useful to remove folds, 
//  e.g. these created when obtaining tecture coordinates after projections)
template<class MESH_TYPE>
void SmoothTextureCoords(MESH_TYPE &m){
  
  assert(m.HasPerVertexTexture());
  
  typedef typename MESH_TYPE::VertexType::TextureType::PointType PointType;
  
  SimpleTempData<typename MESH_TYPE::VertContainer, int> div(m.vert);
  SimpleTempData<typename MESH_TYPE::VertContainer, PointType > sum(m.vert);
  
  div.Start();
  sum.Start();
  
	for (typename MESH_TYPE::VertexIterator v=m.vert.begin(); v!=m.vert.end(); v++) {
		sum[v].SetZero();
    div[v]=0;
	}

	for (typename MESH_TYPE::FaceIterator f=m.face.begin(); f!=m.face.end(); f++){
		div[f->V(0)] +=2; sum[f->V(0)] += f->V(2)->T().P(); sum[f->V(0)] += f->V(1)->T().P();
		div[f->V(1)] +=2; sum[f->V(1)] += f->V(0)->T().P(); sum[f->V(1)] += f->V(2)->T().P();
		div[f->V(2)] +=2; sum[f->V(2)] += f->V(1)->T().P(); sum[f->V(2)] += f->V(0)->T().P();
	}

	for (typename MESH_TYPE::VertexIterator v=m.vert.begin(); v!=m.vert.end(); v++) // if (!v->IsB()) 
  {
		if (v->div>0) {
			v->T().P() = sum[v]/div[v];
		}
	}
	
	div.Stop();
  sum.Stop();

}



}	}	// End namespace vcg::tri

#endif //  __VCGLIB__TEXTCOOORD_OPTIMIZATION
