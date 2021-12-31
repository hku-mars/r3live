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
  $Log: not supported by cvs2svn $
  Revision 1.20  2007/01/19 09:13:09  cignoni
  Added Finalize() method to the interface

  Revision 1.19  2007/01/11 11:48:33  ganovelli
  currMetric inizialied to heap.front() (it was heap.back()- wrong)

  Revision 1.18  2006/12/11 14:09:44  ganovelli
  added missing initialization of currMetric

  Revision 1.17  2006/06/09 07:28:43  m_di_benedetto
  Corrected ClearHeap(): iterator "hi" not decrementable if it was the first of the container.

  Revision 1.16  2005/11/10 15:38:46  cignoni
  Added casts to remove warnings

  Revision 1.15  2005/10/02 23:23:52  cignoni
  Changed the sense of the < operator for heap: it is reversed according to the stl where highest score elements must float in the heap
  Completed TimeBudget Termination condition.
  Parametrized the ClearHeap procedure now there is a HeapSimplexRatio param. Removed dirty printf.

  Revision 1.14  2005/04/14 11:34:33  ponchio
  *** empty log message ***

  Revision 1.13  2005/01/19 10:33:50  cignoni
  Improved ClearHeap management

  Revision 1.12  2004/12/10 01:02:48  cignoni
  added an inline and removed loggng

  Revision 1.11  2004/12/03 21:14:39  ponchio
  Fixed memory leak...

  Revision 1.10  2004/11/23 10:37:17  cignoni
  Added a member with a cached copy of the floating Priority() value inside the HeapElem to optimize operator< in heap updating operator

  Revision 1.9  2004/11/05 10:03:47  fiorin
  Added ModifierType::TriEdgeFlipOp

  Revision 1.8  2004/10/25 07:02:56  ganovelli
  some inline function, logs on file (precompiler directive)

  Revision 1.7  2004/09/29 17:08:39  ganovelli
  changed > to < in heapelem comparison

  Revision 1.6  2004/09/28 09:57:08  cignoni
  Better Doxygen docs

  Revision 1.5  2004/09/15 10:40:20  ponchio
  typedef LocalOptimization HeapType -> public:

  Revision 1.4  2004/09/08 15:10:59  ganovelli
  *** empty log message ***

  Revision 1.3  2004/07/27 09:46:15  cignoni
  First working version of the LocalOptimization/Simplification Framework

  Revision 1.1  2004/07/15 12:04:14  ganovelli
  minor changes

  Revision 1.2  2004/07/09 10:22:56  ganovelli
  working draft

  Revision 1.1  2004/07/08 08:25:15  ganovelli
  first draft

****************************************************************************/

#ifndef __VCGLIB_LOCALOPTIMIZATION
#define __VCGLIB_LOCALOPTIMIZATION
#include<vector>
#include<algorithm>
#include<time.h>
#include<math.h>
#include<vcg/complex/complex.h>

namespace vcg{
// Base class for Parameters
// all parameters must be derived from this.
class BaseParameterClass { };

template<class MeshType>
class LocalOptimization;

enum ModifierType{	TetraEdgeCollapseOp, TriEdgeSwapOp, TriVertexSplitOp,
				TriEdgeCollapseOp,TetraEdgeSpliOpt,TetraEdgeSwapOp, TriEdgeFlipOp,
				QuadDiagCollapseOp, QuadEdgeCollapseOp};
/** \addtogroup tetramesh */
/*@{*/
/// This abstract class define which functions  a local modification to be used in the LocalOptimization.
template <class MeshType>
class LocalModification
{
 public:
        typedef typename LocalOptimization<MeshType>::HeapType HeapType;
        typedef typename MeshType::ScalarType ScalarType;


  inline LocalModification(){}
  virtual ~LocalModification(){}
  
	/// return the type of operation
	virtual ModifierType IsOfType() = 0 ;

	/// return true if the data have not changed since it was created
  virtual bool IsUpToDate() const = 0 ;

	/// return true if no constraint disallow this operation to be performed (ex: change of topology in edge collapses)
  virtual bool IsFeasible(BaseParameterClass *pp) = 0;

	/// Compute the priority to be used in the heap
  virtual ScalarType ComputePriority(BaseParameterClass *pp)=0;

	/// Return the priority to be used in the heap (implement static priority)
	virtual ScalarType Priority() const =0;

  /// Perform the operation
  virtual void Execute(MeshType &m, BaseParameterClass *pp)=0;

	/// perform initialization
  static void Init(MeshType &m, HeapType&, BaseParameterClass *pp);

	/// An approximation of the size of the heap with respect of the number of simplex
    /// of the mesh. When this number is exceeded a clear heap purging is performed. 
    /// so it is should be reasonably larger than the minimum expected size to avoid too frequent clear heap
    /// For example for symmetric edge collapse a 5 is a good guess. 
    /// while for non symmetric edge collapse a larger number like 9 is a better choice
  static float HeapSimplexRatio(BaseParameterClass *) {return 6.0f;}

  virtual const char *Info(MeshType &) {return 0;}
	/// Update the heap as a consequence of this operation
  virtual void UpdateHeap(HeapType&, BaseParameterClass *pp)=0;
};	//end class local modification


/// LocalOptimization:
/// This class implements the algorihms running on 0-1-2-3-simplicial complex that are based on local modification
/// The local modification can be and edge_collpase, or an edge_swap, a vertex plit...as far as they implement
/// the interface defined in LocalModification.
/// Implementation note: in order to keep the local modification itself indepented by its use in this class, they are not
/// really derived by LocalModification. Instead, a wrapper is done to this purpose (see vcg/complex/tetramesh/decimation/collapse.h)

template<class MeshType>
class LocalOptimization
{
public:
  LocalOptimization(MeshType &mm, BaseParameterClass *_pp): m(mm){ ClearTermination();e=0.0;HeapSimplexRatio=5; pp=_pp;}

	struct  HeapElem;
	// scalar type
	typedef typename MeshType::ScalarType ScalarType;
	// type of the heap
	typedef typename std::vector<HeapElem> HeapType;	
	// modification type	
  typedef  LocalModification <MeshType>  LocModType;
	// modification Pointer type	
  typedef  LocalModification <MeshType> * LocModPtrType;
	


	/// termination conditions	
	 enum LOTermination {	
      LOnSimplices	= 0x01,	// test number of simplicies	
			LOnVertices		= 0x02, // test number of verticies
			LOnOps			= 0x04, // test number of operations
			LOMetric		= 0x08, // test Metric (error, quality...instance dependent)
			LOTime			= 0x10  // test how much time is passed since the start
		} ;

	int tf;
	
  int nPerfmormedOps,
		nTargetOps,
		nTargetSimplices,
		nTargetVertices;

	float	timeBudget;
  clock_t	start;
	ScalarType currMetric;
	ScalarType targetMetric;
  BaseParameterClass *pp;

  // The ratio between Heap size and the number of simplices in the current mesh
  // When this value is exceeded a ClearHeap Start;

  float HeapSimplexRatio; 

	void SetTerminationFlag		(int v){tf |= v;}
	void ClearTerminationFlag	(int v){tf &= ~v;}
	bool IsTerminationFlag		(int v){return ((tf & v)!=0);}

	void SetTargetSimplices	(int ts			){nTargetSimplices	= ts;	SetTerminationFlag(LOnSimplices);	}	 	
	void SetTargetVertices	(int tv			){nTargetVertices	= tv;	SetTerminationFlag(LOnVertices);	} 
	void SetTargetOperations(int to			){nTargetOps		= to;	SetTerminationFlag(LOnOps);			} 

	void SetTargetMetric	(ScalarType tm	){targetMetric		= tm;	SetTerminationFlag(LOMetric);		} 
	void SetTimeBudget		(float tb		){timeBudget		= tb;	SetTerminationFlag(LOTime);			} 

  void ClearTermination()
  {
    tf=0;
    nTargetSimplices=0;
    nTargetOps=0;
    targetMetric=0;
    timeBudget=0;
    nTargetVertices=0;
  }
	/// the mesh to optimize
	MeshType & m;



	///the heap of operations
	HeapType h;

  ///the element of the heap
  // it is just a wrapper of the pointer to the localMod. 
  // std heap does not work for
  // pointers and we want pointers to have heterogenous heaps. 

  struct HeapElem
  {
		inline HeapElem(){locModPtr = NULL;}
	  ~HeapElem(){}

    ///pointer to instance of local modifier
    LocModPtrType locModPtr;
    float pri;

   
    inline HeapElem( LocModPtrType _locModPtr)
    {
		  locModPtr = _locModPtr;
      pri=float(locModPtr->Priority());
    };

    /// STL heap has the largest element as the first one.
    /// usually we mean priority as an error so we should invert the comparison
    inline bool operator <(const HeapElem & h) const
    { 
		  return (pri > h.pri);
		  //return (locModPtr->Priority() < h.locModPtr->Priority());
	  }

    bool IsUpToDate() const
    {
			return locModPtr->IsUpToDate();
		}
  };



  /// Default distructor
  ~LocalOptimization(){ 
    typename HeapType::iterator i;
    for(i = h.begin(); i != h.end(); i++)
      delete (*i).locModPtr;
  };
	
	double e;

  /// main cycle of optimization
  bool DoOptimization()
  {
    start=clock();
		nPerfmormedOps =0;
		while( !GoalReached() && !h.empty())
			{
        if(h.size()> m.SimplexNumber()*HeapSimplexRatio )  ClearHeap();
				std::pop_heap(h.begin(),h.end());
        LocModPtrType  locMod   = h.back().locModPtr;
				currMetric=h.back().pri;
        h.pop_back();
        				
        if( locMod->IsUpToDate() )
				{	
          //printf("popped out: %s\n",locMod->Info(m));
         	// check if it is feasible
          if (locMod->IsFeasible(this->pp))
					{
						nPerfmormedOps++;
            locMod->Execute(m,this->pp);
            locMod->UpdateHeap(h,this->pp);
						}
				}
        //else printf("popped out unfeasible\n");
				delete locMod;
			}
		return !(h.empty());
  }
 
// It removes from the heap all the operations that are no more 'uptodate' 
// (e.g. collapses that have some recently modified vertices)
// This function  is called from time to time by the doOptimization (e.g. when the heap is larger than fn*3)
void ClearHeap()
{
	typename HeapType::iterator hi;
	//int sz=h.size();
	for(hi=h.begin();hi!=h.end();)
	{
    if(!(*hi).locModPtr->IsUpToDate())
		{
			delete (*hi).locModPtr;
			*hi=h.back();
			if(&*hi==&h.back()) 
			{
				hi=h.end();
				h.pop_back();
				break;
			}
		    h.pop_back();
			continue;
		}
		++hi;
	}
	//qDebug("\nReduced heap from %7i to %7i (fn %7i) ",sz,h.size(),m.fn);
	make_heap(h.begin(),h.end());
}

	///initialize for all vertex the temporary mark must call only at the start of decimation
	///by default it takes the first element in the heap and calls Init (static funcion) of that type
	///of local modification. 
  template <class LocalModificationType> void Init()
	{
    vcg::tri::InitVertexIMark(m);
		
    // The expected size of heap depends on the type of the local modification we are using..
    HeapSimplexRatio = LocalModificationType::HeapSimplexRatio(pp);
		
    LocalModificationType::Init(m,h,pp);
    std::make_heap(h.begin(),h.end());
    if(!h.empty()) currMetric=h.front().pri;
	}


	template <class LocalModificationType> void Finalize()
	{
    LocalModificationType::Finalize(m,h,pp);
	}


	/// say if the process is to end or not: the process ends when any of the termination conditions is verified
	/// override this function to implemetn other tests
	bool GoalReached(){
		assert ( ( ( tf & LOnSimplices	)==0) ||  ( nTargetSimplices!= -1));
		assert ( ( ( tf & LOnVertices	)==0) ||  ( nTargetVertices	!= -1));
		assert ( ( ( tf & LOnOps		)==0) ||  ( nTargetOps		!= -1));
		assert ( ( ( tf & LOMetric		)==0) ||  ( targetMetric	!= -1));
		assert ( ( ( tf & LOTime		)==0) ||  ( timeBudget		!= -1));

		if ( IsTerminationFlag(LOnSimplices) &&	( m.SimplexNumber()<= nTargetSimplices)) return true;
		if ( IsTerminationFlag(LOnVertices)  &&  ( m.VertexNumber() <= nTargetVertices)) return true;
		if ( IsTerminationFlag(LOnOps)		   && (nPerfmormedOps	== nTargetOps)) return true;
		if ( IsTerminationFlag(LOMetric)		 &&  ( currMetric		> targetMetric)) return true;
    if ( IsTerminationFlag(LOTime) )
    {
      clock_t cur = clock();
      if(cur<start) // overflow of tick counter;
        return true; // panic
      else
       if ( (cur - start)/(double)CLOCKS_PER_SEC > timeBudget) return true;
    }
		return false;
	}



///erase from the heap the operations that are out of date
  void ClearHeapOld()
  {
		typename HeapType::iterator hi;
		for(hi=h.begin();hi!=h.end();++hi)
			if(!(*hi).locModPtr->IsUpToDate())
			{
				*hi=h.back();
				h.pop_back();
				if(hi==h.end()) break;
			}
			//printf("\nReduced heap from %i to %i",sz,h.size());
			make_heap(h.begin(),h.end());
  }

};//end class decimation

}//end namespace
#endif
