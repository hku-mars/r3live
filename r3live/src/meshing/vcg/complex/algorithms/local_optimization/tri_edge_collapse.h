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


#ifndef __VCG_DECIMATION_TRICOLLAPSE
#define __VCG_DECIMATION_TRICOLLAPSE

#include<vcg/complex/algorithms/edge_collapse.h>
#include<vcg/simplex/face/pos.h>
#include<vcg/complex/algorithms/local_optimization.h>
#include<vcg/complex/algorithms/update/topology.h>

namespace vcg{
namespace tri{

/** \addtogroup trimesh */
/*@{*/
/// This Class is specialization of LocalModification for the edge collapse.
/// It wraps the atomic operation EdgeCollapse to be used in a optimizatin routine.
/// Note that it has knowledge of the heap of the class LocalOptimization because
/// it is responsible of updating it after a collapse has been performed; 
/// This is the base class of all the specialized collapse classes like for example Quadric Edge Collapse. 
/// Each derived class 

template<class TriMeshType, class VertexPair, class MYTYPE>
class TriEdgeCollapse: public LocalOptimization<TriMeshType>::LocModType
{
public:
 /// static data to gather statistical information about the reasons of collapse failures
  class FailStat { 
  public:
	static int &Volume()           {static int vol=0; return vol;}
	static int &LinkConditionFace(){static int lkf=0; return lkf;}
	static int &LinkConditionEdge(){static int lke=0; return lke;}
	static int &LinkConditionVert(){static int lkv=0; return lkv;}
	static int &OutOfDate()        {static int ofd=0; return ofd;}
	static int &Border()           {static int bor=0; return bor;}
  static void Init() 
  {
   Volume()           =0;
   LinkConditionFace()=0;
   LinkConditionEdge()=0;
   LinkConditionVert()=0;
   OutOfDate()        =0;
   Border()           =0;
  }
};
protected:
  typedef	typename TriMeshType::FaceType FaceType;
  typedef	typename TriMeshType::FaceType::VertexType VertexType;
//	typedef	typename VertexType::EdgeType EdgeType;
  typedef	typename FaceType::VertexType::CoordType CoordType;
  typedef	typename TriMeshType::VertexType::ScalarType ScalarType;
  typedef typename LocalOptimization<TriMeshType>::HeapElem HeapElem;
	typedef typename LocalOptimization<TriMeshType>::HeapType HeapType;

  TriMeshType *mt;
	///the pair to collapse 
  VertexPair pos;

	///mark for up_dating
	static int& GlobalMark(){ static int im=0; return im;}

	///mark for up_dating
	int localMark;
	
	/// priority in the heap
	ScalarType _priority;

	public:
	/// Default Constructor
	inline	TriEdgeCollapse()
			{}
	///Constructor with postype
   inline TriEdgeCollapse(const VertexPair &p, int mark, BaseParameterClass *pp)
			{    
				localMark = mark;
				pos=p;
        _priority = ComputePriority(pp);
			}

		~TriEdgeCollapse()
			{}

private:


public:

  inline ScalarType ComputePriority(BaseParameterClass *)
  { 
		_priority = Distance(pos.V(0)->cP(),pos.V(1)->cP()); 
    return _priority;
  }

  virtual const char *Info(TriMeshType &m) {
		mt = &m;
    static char buf[60];
      sprintf(buf,"%i -> %i %g\n", int(pos.V(0)-&m.vert[0]), int(pos.V(1)-&m.vert[0]),-_priority);
   return buf;
  }
 
  inline void Execute(TriMeshType &m, BaseParameterClass *)
  {	
    CoordType MidPoint=(pos.V(0)->P()+pos.V(1)->P())/2.0;
    EdgeCollapser<TriMeshType,VertexPair>::Do(m, pos, MidPoint);
  }
  
  static bool IsSymmetric(BaseParameterClass *) { return true;}
  
  // This function is called after an action to re-add in the heap elements whose priority could have been changed.
  // in the plain case we just put again in the heap all the edges around the vertex resulting from the previous collapse: v[1].
  // if the collapse is not symmetric you should add also backward edges (because v0->v1 collapse could be different from v1->v0)

  inline  void UpdateHeap(HeapType & h_ret, BaseParameterClass *pp)
  {
    GlobalMark()++;
    VertexType *v[2];
    v[0]= pos.V(0);v[1]=pos.V(1);
    v[1]->IMark() = GlobalMark();

    // First loop around the remaining vertex to unmark visited flags
    vcg::face::VFIterator<FaceType> vfi(v[1]);
    while (!vfi.End()){
      vfi.V1()->ClearV();
      vfi.V2()->ClearV();
      ++vfi;
    }

    // Second Loop: add all the outgoing edges around v[1]
    // for each face add the two edges outgoing from v[1] and not visited.
    vfi = face::VFIterator<FaceType>(v[1]);
    while (!vfi.End())
    {
      assert(!vfi.F()->IsD());
      if( !(vfi.V1()->IsV()) && (vfi.V1()->IsRW()))
      {
        vfi.V1()->SetV();
        h_ret.push_back(HeapElem(new MYTYPE(VertexPair( vfi.V(),vfi.V1() ),GlobalMark(),pp)));
        std::push_heap(h_ret.begin(),h_ret.end());
        if(! this->IsSymmetric(pp)){
          h_ret.push_back(HeapElem(new MYTYPE(VertexPair( vfi.V1(),vfi.V()),GlobalMark(),pp)));
          std::push_heap(h_ret.begin(),h_ret.end());
        }
      }
      if(  !(vfi.V2()->IsV()) && (vfi.V2()->IsRW()))
      {
        vfi.V2()->SetV();
        h_ret.push_back(HeapElem(new MYTYPE(VertexPair(vfi.F()->V(vfi.I()),vfi.F()->V2(vfi.I())),GlobalMark(),pp)));
        std::push_heap(h_ret.begin(),h_ret.end());
        if(! this->IsSymmetric(pp)){
          h_ret.push_back(HeapElem(new MYTYPE(VertexPair (vfi.F()->V1(vfi.I()),vfi.F()->V(vfi.I())),GlobalMark(),pp)));
          std::push_heap(h_ret.begin(),h_ret.end());
        }
      }
      //        if(vfi.V1()->IsRW() && vfi.V2()->IsRW() )
      //        {
      //          h_ret.push_back(HeapElem(new MYTYPE(EdgeType(vfi.V1(),vfi.V2()),this->GlobalMark())));
      //          std::push_heap(h_ret.begin(),h_ret.end());
      //          if(IsSymmetric()){
      //            h_ret.push_back(HeapElem(new MYTYPE(EdgeType(vfi.V2(),vfi.V1()), this->GlobalMark())));
      //            std::push_heap(h_ret.begin(),h_ret.end());
      //          }
      //        }
      ++vfi;
    } // end while
  }

  ModifierType IsOfType(){ return TriEdgeCollapseOp;}

  inline bool IsFeasible(BaseParameterClass *){
    return EdgeCollapser<TriMeshType,VertexPair>::LinkConditions(pos);
	}

  inline bool IsUpToDate() const
  {
      VertexType *v0=pos.cV(0);
      VertexType *v1=pos.cV(1);
			if( v0->IsD() || v1->IsD() ||
				 localMark < v0->IMark()  ||
		  	 localMark < v1->IMark()   )
			{
				++FailStat::OutOfDate();
				return false;
      }
        return true;
	}

	virtual ScalarType Priority() const {
	return _priority;
  }

  static void Init(TriMeshType &m, HeapType &h_ret, BaseParameterClass *pp)
  {
    vcg::tri::UpdateTopology<TriMeshType>::VertexFace(m);
    h_ret.clear();
    typename TriMeshType::FaceIterator fi;
    for(fi = m.face.begin(); fi != m.face.end();++fi)
    if(!(*fi).IsD()){
       for (int j=0;j<3;j++)
      {
        VertexPair p((*fi).V0(j), (*fi).V1(j));
        p.Sort();
        h_ret.push_back(HeapElem(new MYTYPE(p, IMark(m),pp)));
        //printf("Inserting in heap coll %3i ->%3i %f\n",p.V()-&m.vert[0],p.VFlip()-&m.vert[0],h_ret.back().locModPtr->Priority());
      }
    }
	}

};
}//end namespace tri
}//end namespace vcg

#endif
