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
****************************************************************************/

#ifndef __VCG_DECIMATION_COLLAPSE
#define __VCG_DECIMATION_COLLAPSE

#include<vcg/complex/algorithms/local_optimization.h>
#include<vcg/simplex/tetrahedron/pos.h>
#include<vcg/complex/tetramesh/edge_collapse.h>
#include<vcg/space/point3.h>


struct FAIL{
	static int VOL(){static int vol=0; return vol++;}
	static int LKF(){static int lkf=0; return lkf++;}
	static int LKE(){static int lke=0; return lke++;}
	static int LKV(){static int lkv=0; return lkv++;}
	static int OFD(){static int ofd=0; return ofd++;}
	static int BOR(){static int bor=0; return bor++;}

};

namespace vcg{
namespace tetra{

/** \addtogroup tetramesh */
/*@{*/
/// This Class is specialization of LocalModification for the edge collapse
/// It wraps the atomic operation EdgeCollapse to be used in a optimizatin routine.
/// Note that it has knowledge of the heap of the class LocalOptimization because
/// it is responsible of updating it after a collapse has been performed

template<class TETRA_MESH_TYPE>
class TetraEdgeCollapse: public LocalOptimization<TETRA_MESH_TYPE>::LocModType
{
 
  /// The tetrahedral mesh type
  //typedef	typename TETRA_MESH_TYPE TETRA_MESH_TYPE;
  /// The tetrahedron type
  typedef	typename TETRA_MESH_TYPE::TetraType TetraType;
  /// The vertex type
  typedef	typename TetraType::VertexType VertexType;
  /// The coordinate type
  typedef	typename TetraType::VertexType::CoordType CoordType;
  /// The scalar type
  typedef	typename TETRA_MESH_TYPE::VertexType::ScalarType ScalarType;
  /////the base type class
  //typedef typename vcg::tri::LocalModification LocalMod;
  /// The HEdgePos type
  typedef Pos<TetraType> PosType;
  /// The HEdgePos Loop type
  typedef PosLoop<TetraType> PosLType;
  /// definition of the heap element
	typedef typename LocalOptimization<TETRA_MESH_TYPE>::HeapElem HeapElem;
private:

///the new point that substitute the edge
Point3<ScalarType> _NewPoint;
///the pointer to edge collapser method
vcg::tetra::EdgeCollapse<TETRA_MESH_TYPE> _EC;
///mark for up_dating
static int& _Imark(){ static int im=0; return im;}
///the pos of collapse 
PosType pos;
///pointer to vertex that remain
VertexType *vrem;
/// priority in the heap
ScalarType _priority;

public:
/// Default Constructor
	TetraEdgeCollapse()
		{}

///Constructor with postype
	TetraEdgeCollapse(PosType p,int mark)
		{    
			_Imark() = mark;
			pos=p;
			_priority = _AspectRatioMedia(p);
		}

  ~TetraEdgeCollapse()
		{}

private:

///Return the aspect Ratio media of the tetrahedrons
///that share the adge to collapse
ScalarType _AspectRatioMedia(PosType p)
{
  PosLType posl=PosLType(p.T(),p.F(),p.E(),p.V());
  posl.Reset();
  int num=0;
  ScalarType ratio_media=0.f;
  while(!posl.LoopEnd())
  {
    ratio_media+=posl.T()->AspectRatio();
    posl.NextT();
    num++;
  }
  ratio_media=ratio_media/num;
  return (ratio_media);
}


///Modify pos and alfa to obtain the collapse that minimize the error
ScalarType _VolumePreservingError(PosType &pos,CoordType &new_point,int nsteps)
{
  VertexType *ve0=(pos.T()->V(Tetra::VofE(pos.E(),0)));
  VertexType *ve1=(pos.T()->V(Tetra::VofE(pos.E(),1)));
  bool ext_v0=ve0->IsB();
  bool ext_v1=ve1->IsB();

  ScalarType best_error=0.f;
   if ((ext_v0)&&(!ext_v1))
      new_point=ve0->P();
   else
   if ((!ext_v0)&&(ext_v1))
      new_point=ve1->P();
   else
   if ((!ext_v0)&&(!ext_v1))
	 {/*CoordType g;
	 g.SetZero();
	 g+=ve0->cP();
	 g+=ve1->cP();
	 g/=2;*/
     new_point=(ve0->cP()+ve1->cP())/2.f;
	 }
   else
   if ((ext_v0)&&(ext_v1))//both are external vertex
   {
    ScalarType step=1.f/(nsteps-1);
    ScalarType Vol_Original=_EC.VolumeOriginal();
    for (int i=0;i<nsteps;i++)
    {
      best_error=1000000.f;
      ScalarType alfatemp=step*((ScalarType)i);
			//CoordType g;
			// g.SetZero();
		 //g+=ve0->cP()*alfatemp;
		 //g+=ve1->cP()*(1-alfatemp);
	  //CoordType newPTemp=g;
      CoordType newPTemp=(ve0->cP()*alfatemp) +(ve1->cP()*(1.f-alfatemp));
      //the error is the absolute value of difference of volumes
      ScalarType error=fabs(Vol_Original-_EC.VolumeSimulateCollapse(pos,newPTemp));
      if(error<best_error)
      {
       new_point=newPTemp;
       best_error=error;
      }
    }
   }
   return (best_error);
}



public:
  
  virtual const char *Info(TETRA_MESH_TYPE &m) {
    static char buf[60];
    //sprintf(buf,"collapse %i -> %i %f\n", pos.()-&m.vert[0], pos.VFlip()-&m.vert[0],_priority);
    return buf;
  }

  ScalarType ComputePriority()
  { 
		return (_priority = _AspectRatioMedia(this->pos));
  }

  ScalarType ComputeError()
  {
	  vrem=(pos.T()->V(Tetra::VofE(pos.E(),0)));
      return (_VolumePreservingError(pos,_NewPoint,5));// magic number....parametrize!
  }

  void Execute(TETRA_MESH_TYPE &tm)
  {
   // _EC.FindSets(pos);
	assert(!vrem->IsD());
    int del=_EC.DoCollapse(pos,_NewPoint);
	tm.tn-=del;
	tm.vn-=1;
  }
  
  void UpdateHeap(typename LocalOptimization<TETRA_MESH_TYPE>::HeapType & h_ret)
  {
    assert(!vrem->IsD());
		_Imark()++;
    VTIterator<TetraType> VTi(vrem->VTb(),vrem->VTi());
    while (!VTi.End())
    {
	  VTi.Vt()->ComputeVolume();
      for (int j=0;j<6;j++)
      {
		vcg::tetra::Pos<TetraType> p=Pos<TetraType>(VTi.Vt(),Tetra::FofE(j,0),j,Tetra::VofE(j,0));
		assert(!p.T()->V(p.V())->IsD());
		assert(!p.T()->IsD());
        h_ret.push_back(HeapElem(new TetraEdgeCollapse<TETRA_MESH_TYPE>(p,_Imark())));
		std::push_heap(h_ret.begin(),h_ret.end());
		// update the mark of the vertices
		VTi.Vt()->V(Tetra::VofE(j,0))->IMark() = _Imark();
      }
      ++VTi;
    }
  }

  /// return the type of operation

  ModifierType IsOfType(){ return TetraEdgeCollapseOp;}

  bool IsFeasible(){
				vcg::tetra::EdgeCollapse<TETRA_MESH_TYPE>::Reset();	
				_EC.FindSets(pos);
				ComputeError();
				return(_EC.CheckPreconditions(pos,_NewPoint));
				}

  bool IsUpToDate(){
	   	if (!pos.T()->IsD())
				{
        VertexType *v0=pos.T()->V(Tetra::VofE(pos.E(),0));
		VertexType *v1=pos.T()->V(Tetra::VofE(pos.E(),1));
		assert(!v0->IsD());
		assert(!v1->IsD());
			if(! (( (!v0->IsD()) && (!v1->IsD())) &&
							 _Imark()>=v0->IMark() &&
							 _Imark()>=v1->IMark()))
			{
				FAIL::OFD(); 
				return false;
			}
			else 
				return true;
		  }
			else 
			return false;
	}

	virtual ScalarType Priority() const {
		return _priority;
	}

	/// perform initialization
	static void Init(TETRA_MESH_TYPE &m,typename LocalOptimization<TETRA_MESH_TYPE>::HeapType& h_ret){
		h_ret.clear();
		typename TETRA_MESH_TYPE::TetraIterator ti;
		for(ti = m.tetra.begin(); ti != m.tetra.end();++ti)
		if(!(*ti).IsD()){
			(*ti).ComputeVolume();
	   for (int j=0;j<6;j++)
		{
			PosType p=PosType(&*ti,Tetra::FofE(j,0),j,Tetra::VofE(j,0));
			assert(!p.T()->V(p.V())->IsD());
			assert(!p.T()->IsD());
			h_ret.push_back(HeapElem(new TetraEdgeCollapse<TETRA_MESH_TYPE>(p,m.IMark)));
		}
		}
	}

};
}//end namespace tetra
}//end namespace vcg
#endif
