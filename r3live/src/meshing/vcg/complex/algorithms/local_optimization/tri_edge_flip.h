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

#ifndef __VCG_DECIMATION_TRIFLIP
#define __VCG_DECIMATION_TRIFLIP

#include <vcg/complex/algorithms/local_optimization.h>
#include <vcg/simplex/face/topology.h>
#include <vcg/space/triangle3.h>

namespace vcg
{
namespace tri
{
/** \addtogroup trimesh */
/* @{ */

class PlanarEdgeFlipParameter : public BaseParameterClass
{
public:
  PlanarEdgeFlipParameter() {SetDefaultParams();}
  void SetDefaultParams()
  {
    CoplanarAngleThresholdDeg=0.1f;
  }

  float CoplanarAngleThresholdDeg;
};

/*!
 *	This Class is specialization of LocalModification for the edge flip
 *	It wraps the atomic operation EdgeFlip to be used in a optimization routine.
 * Note that it has knowledge of the heap of the class LocalOptimization because
 * it is responsible of updating it after a flip has been performed
 * This is the simplest edge flipping class. 
 * It flips an edge only if two adjacent faces are coplanar and the 
 * quality of the faces improves after the flip.
 */
template <class TRIMESH_TYPE, class MYTYPE,
            typename TRIMESH_TYPE::ScalarType (*QualityFunc)(
            		Point3<typename TRIMESH_TYPE::ScalarType> const & p0,
            		Point3<typename TRIMESH_TYPE::ScalarType> const & p1, 
            		Point3<typename TRIMESH_TYPE::ScalarType> const & p2) = Quality>
class PlanarEdgeFlip :
	public LocalOptimization< TRIMESH_TYPE>::LocModType
{
protected:
	typedef typename TRIMESH_TYPE::FaceType FaceType;
	typedef typename TRIMESH_TYPE::FacePointer FacePointer;
	typedef typename TRIMESH_TYPE::FaceIterator FaceIterator;
	typedef typename TRIMESH_TYPE::VertexType VertexType;
	typedef typename TRIMESH_TYPE::ScalarType ScalarType;
	typedef typename TRIMESH_TYPE::VertexPointer VertexPointer;
	typedef typename TRIMESH_TYPE::CoordType CoordType;
	typedef vcg::face::Pos<FaceType> PosType;
	typedef typename LocalOptimization<TRIMESH_TYPE>::HeapElem HeapElem;
	typedef typename LocalOptimization<TRIMESH_TYPE>::HeapType HeapType;

	/*! 
	 *	the pos of the flipping
	 */
	PosType _pos;

	/*!
	 * priority in the heap
	 */
	ScalarType _priority;

	/*!
	 * Mark for updating
	 */
	int _localMark;

	/*!
	 *	mark for up_dating
	 */
	static int& GlobalMark()
	{
		static int im = 0;
		return im;
	}
	
  static void Insert(HeapType& heap, PosType& p, int mark, BaseParameterClass *pp)
	{
		if(!p.IsBorder() && p.F()->IsW() && p.FFlip()->IsW()) {
      MYTYPE* newflip = new MYTYPE(p, mark,pp);
			heap.push_back(HeapElem(newflip));
			std::push_heap(heap.begin(), heap.end());
		}
	}
	
public:
	/*!
	 *	Default constructor
	 */
	inline PlanarEdgeFlip()
	{
	}
	
	
	/*!
	 *	Constructor with <I>pos</I> type
	 */
  inline PlanarEdgeFlip(PosType pos, int mark,BaseParameterClass *pp)
	{
		_pos = pos;
		_localMark = mark;
    _priority = this->ComputePriority(pp);
	}
	
	
	/*!
	 *	Copy Constructor 
	 */
	inline PlanarEdgeFlip(const PlanarEdgeFlip &par)
	{
		_pos = par.GetPos();
		_localMark = par.GetMark();
		_priority = par.Priority();
	}
	
	
	/*!
	 */
	~PlanarEdgeFlip()
	{
	}
	
	
	/*!
	 * Parameter 
	 */

	inline PosType GetPos() const
	{
		return _pos;
	}
	
	inline int GetMark()const
	{
		return _localMark;
	}
	
	
	/*!
	 *	Return the LocalOptimization type
	 */
	ModifierType IsOfType()
	{
		return TriEdgeFlipOp;
	}
	
	
	/*!
	 * Check if the pos is updated
	 */
  bool IsUpToDate() const
	{
    int lastMark = _pos.F()->cV(0)->IMark();
    lastMark = std::max<int>(lastMark, _pos.F()->V(1)->IMark());
    lastMark = std::max<int>(lastMark, _pos.F()->V(2)->IMark());

		return ( _localMark >= lastMark );
	}

	/*!
	 * 
	 Check if this flipping operation can be performed.
	 It is a topological and geometrical check. 
	 */
  virtual bool IsFeasible(BaseParameterClass *_pp)
	{
    PlanarEdgeFlipParameter *pp=(PlanarEdgeFlipParameter *)_pp;
		if(!vcg::face::CheckFlipEdge(*this->_pos.F(), this->_pos.E()))
			return false;
		
    if( math::ToDeg( Angle(_pos.FFlip()->cN(), _pos.F()->cN()) ) > pp->CoplanarAngleThresholdDeg )
			return false;

		CoordType v0, v1, v2, v3;
		int i = _pos.E();
		
		v0 = _pos.F()->P0(i);
		v1 = _pos.F()->P1(i);
		v2 = _pos.F()->P2(i);
		v3 = _pos.F()->FFp(i)->P2(_pos.F()->FFi(i));

		// Take the parallelogram formed by the adjacent faces of edge
		// If a corner of the parallelogram on extreme of edge to flip is >= 180
		// the flip produce two identical faces - avoid this
		if( (Angle(v2 - v0, v1 - v0) + Angle(v3 - v0, v1 - v0) >= M_PI) ||
		    (Angle(v2 - v1, v0 - v1) + Angle(v3 - v1, v0 - v1) >= M_PI))
			return false;
		
		// if any of two faces adj to edge in non writable, the flip is unfeasible
		if(!_pos.F()->IsW() || !_pos.F()->FFp(i)->IsW())
			return false;
		
		return true;
	}

	/*!
	 *	Compute the priority of this optimization
	 */
	/*
	     1  
	    /|\
	   / | \
	  2  |  3 
	   \ | /
	    \|/
	     0
	 */
  ScalarType ComputePriority(BaseParameterClass *)
	{
		CoordType v0, v1, v2, v3;
		int i = _pos.E();
		v0 = _pos.F()->P0(i);
		v1 = _pos.F()->P1(i);
		v2 = _pos.F()->P2(i);
		v3 = _pos.F()->FFp(i)->P2(_pos.F()->FFi(i));

		ScalarType Qa = QualityFunc(v0, v1, v2);
		ScalarType Qb = QualityFunc(v0, v3, v1);

		ScalarType QaAfter = QualityFunc(v1, v2, v3);
		ScalarType QbAfter = QualityFunc(v0, v3, v2);
		
		// < 0 if the average quality of faces improves after flip
		_priority = (Qa + Qb - QaAfter - QbAfter) / (ScalarType)2.0;
		
		return _priority;
	}

	/*!
	 * Return the priority of this optimization
	 */
	virtual ScalarType Priority() const
	{
		return _priority;
	}

	/*!
	 * Execute the flipping of the edge
	 */
  void Execute(TRIMESH_TYPE &m, BaseParameterClass *)
	{
		int i = _pos.E();
		int j = _pos.F()->FFi(i);
		FacePointer f1 = _pos.F();
		FacePointer f2 = _pos.F()->FFp(i);
		
		vcg::face::FlipEdge(*_pos.F(), _pos.E());
		
		// avoid texture coordinates swap after flip
		if(tri::HasPerWedgeTexCoord(m)) {
			f2->WT((j + 1) % 3) = f1->WT((i + 2) % 3);
			f1->WT((i + 1) % 3) = f2->WT((j + 2) % 3);
		}
	}

	/*!
	 */
	const char* Info(TRIMESH_TYPE &m)
	{
		static char dump[60];
		sprintf(dump,"%lu -> %lu %g\n", tri::Index(m,_pos.F()->V(0)), tri::Index(m,_pos.F()->V(1)),-_priority);
		return dump;
	}

	/*!
	 */
  static void Init(TRIMESH_TYPE &mesh, HeapType &heap, BaseParameterClass *pp)
	{
		heap.clear();
		FaceIterator fi;
		for(fi = mesh.face.begin(); fi != mesh.face.end(); ++fi) {
			if(!(*fi).IsD() && (*fi).IsW()) {
				for(unsigned int i = 0; i < 3; i++) {
					if( !(*fi).IsB(i) && !((*fi).FFp(i)->IsD()) && (*fi).FFp(i)->IsW() ) {
						if((*fi).V1(i) - (*fi).V0(i) > 0) {
							PosType p(&*fi, i);
              Insert(heap, p,  IMark(mesh),pp);
						}
							//heap.push_back( HeapElem( new MYTYPE(PosType(&*fi, i), mesh.IMark() )) );
					} //endif
				} //endfor
			}
		} //endfor
	}

	/*!
	 */
  virtual void UpdateHeap(HeapType &heap, BaseParameterClass *pp)
	{
		GlobalMark()++;
		
		// after flip, the new edge just created is the next edge
		int flipped = (_pos.E() + 1) % 3;
		
		PosType pos(_pos.F(), flipped);

		pos.F()->V(0)->IMark() = GlobalMark();
		pos.F()->V(1)->IMark() = GlobalMark();
		pos.F()->V(2)->IMark() = GlobalMark();
		pos.F()->FFp(flipped)->V2(pos.F()->FFi(flipped))->IMark() = GlobalMark();

		pos.FlipF(); pos.FlipE();
    Insert(heap, pos, GlobalMark(),pp);

		pos.FlipV(); pos.FlipE();
    Insert(heap, pos, GlobalMark(),pp);

		pos.FlipV(); pos.FlipE();
		pos.FlipF(); pos.FlipE();
    Insert(heap, pos, GlobalMark(),pp);

		pos.FlipV(); pos.FlipE();
    Insert(heap, pos, GlobalMark(),pp);
	}
}; // end of PlanarEdgeFlip class


template <class TRIMESH_TYPE, class MYTYPE>
class TriEdgeFlip : public PlanarEdgeFlip<TRIMESH_TYPE, MYTYPE>
{
protected:
	typedef typename TRIMESH_TYPE::FaceType FaceType;
	typedef typename TRIMESH_TYPE::ScalarType ScalarType;
	typedef typename TRIMESH_TYPE::CoordType CoordType;
	typedef vcg::face::Pos<FaceType> PosType;

public:
	/*!
	 *	Default constructor
	 */
	inline TriEdgeFlip() {}

	/*!
	 *	Constructor with <I>pos</I> type
	 */
  inline TriEdgeFlip(const PosType pos, int mark, BaseParameterClass *pp)
	{
		this->_pos = pos;
		this->_localMark = mark;
    this->_priority = ComputePriority(pp);
	}

	/*!
	 *	Copy Constructor 
	 */
	inline TriEdgeFlip(const TriEdgeFlip &par)
	{
		this->_pos = par.GetPos();
		this->_localMark = par.GetMark();
		this->_priority = par.Priority();
	}
	

  ScalarType ComputePriority(BaseParameterClass *)
	{
		/*
		     1  
		    /|\
		   / | \
		  2  |  3 
		   \ | /
		    \|/
		     0
		 */
		CoordType v0, v1, v2, v3;
		int i = this->_pos.E();
		v0 = this->_pos.F()->P0(i);
		v1 = this->_pos.F()->P1(i);
		v2 = this->_pos.F()->P2(i);
		v3 = this->_pos.F()->FFp(i)->P2(this->_pos.F()->FFi(i));
		
		// if the sum of angles in v2 e v3 is > 180, then the triangle
		// pair is not a delaunay triangulation
		ScalarType alpha = math::Abs(Angle(v0 - v2, v1 - v2));
		ScalarType beta = math::Abs(Angle(v0 - v3, v1 - v3));
		this->_priority = 180 - math::ToDeg((alpha + beta));
		return this->_priority;
	}
};


// This kind of flip minimize the variance of number of incident faces
// on the vertices of two faces involved in the flip
template <class TRIMESH_TYPE, class MYTYPE>
class TopoEdgeFlip : public PlanarEdgeFlip<TRIMESH_TYPE, MYTYPE>
{
protected:
	typedef typename TRIMESH_TYPE::VertexPointer VertexPointer;
	
	typedef typename TRIMESH_TYPE::FaceType FaceType;
	typedef typename TRIMESH_TYPE::FacePointer FacePointer;
	typedef typename TRIMESH_TYPE::ScalarType ScalarType;
	typedef typename TRIMESH_TYPE::CoordType CoordType;
	typedef vcg::face::Pos<FaceType> PosType;

	typedef typename LocalOptimization<TRIMESH_TYPE>::HeapElem HeapElem;
	typedef typename LocalOptimization<TRIMESH_TYPE>::HeapType HeapType;

	typedef typename TRIMESH_TYPE::FaceIterator FaceIterator;
	typedef typename TRIMESH_TYPE::VertexIterator VertexIterator;

public:
	/*!
	 *	Default constructor
	 */
	inline TopoEdgeFlip() {}

	/*!
	 *	Constructor with <I>pos</I> type
	 */
  inline TopoEdgeFlip(const PosType pos, int mark, BaseParameterClass *pp)
	{
		this->_pos = pos;
		this->_localMark = mark;
    this->_priority = ComputePriority(pp);
	}

	/*!
	 *	Copy Constructor 
	 */
	inline TopoEdgeFlip(const TopoEdgeFlip &par)
	{
		this->_pos = par.GetPos();
		this->_localMark = par.GetMark();
		this->_priority = par.Priority();
	}
	

  ScalarType ComputePriority(BaseParameterClass *)
	{
		/*
		     1  
		    /|\
		   / | \
		  2  |  3 
		   \ | /
		    \|/
		     0
		 */
		VertexPointer v0, v1, v2, v3;
		int i = this->_pos.E();
		v0 = this->_pos.F()->V0(i);
		v1 = this->_pos.F()->V1(i);
		v2 = this->_pos.F()->V2(i);
		v3 = this->_pos.F()->FFp(i)->V2(this->_pos.F()->FFi(i));
		
		// This kind of flip minimize the variance of number of incident faces
		// on the vertices of two faces involved in the flip
		
		ScalarType avg = (v0->Q() + v1->Q() + v2->Q() + v3->Q()) / 4.0; 
		
		ScalarType varbefore = (powf(v0->Q() - avg, 2.0) + 
		                        powf(v1->Q() - avg, 2.0) + 
				                powf(v2->Q() - avg, 2.0) + 
				                powf(v3->Q() - avg, 2.0)) / 4.0;
		
		ScalarType varafter = (powf(v0->Q() - 1 - avg, 2.0) +
                               powf(v1->Q() - 1 - avg, 2.0) +
		                       powf(v2->Q() + 1 - avg, 2.0) +
	 	                       powf(v3->Q() + 1 - avg, 2.0)) / 4.0; 
		
		this->_priority = varafter - varbefore;
		return this->_priority;
	}
	
	
	/*!
	 * Execute the flipping of the edge
	 */
	void Execute(TRIMESH_TYPE &m)
	{
		int i = this->_pos.E();
		FacePointer f1 = this->_pos.F();
		FacePointer f2 = f1->FFp(i);
		int j = f1->FFi(i);
		
		// update the number of faces adjacent to vertices
		f1->V0(i)->Q()--;
		f1->V1(i)->Q()--;
		f1->V2(i)->Q()++;
		f2->V2(j)->Q()++;
		
		// do the flip
		vcg::face::FlipEdge(*this->_pos.F(), this->_pos.E());
		
		// avoid texture coordinates swap after flip 
		if (tri::HasPerWedgeTexCoord(m)) {
			f2->WT((j + 1) % 3) = f1->WT((i + 2) % 3);
			f1->WT((i + 1) % 3) = f2->WT((j + 2) % 3);
		}
	}
	
	
  static void Init(TRIMESH_TYPE &m, HeapType &heap,BaseParameterClass *pp)
	{
		// reset quality field for each vertex
		VertexIterator vi;
		for(vi = m.vert.begin(); vi != m.vert.end(); ++vi)
			if(!(*vi).IsD())
				(*vi).Q() = 0;
		
		// for each vertex, put the number of incident faces in quality field
		FaceIterator fi;
		for(fi = m.face.begin(); fi != m.face.end(); ++fi)
			if(!(*fi).IsD())
				for(int i = 0; i < 3; i++)
					(*fi).V(i)->Q()++;
		
    TriEdgeFlip<TRIMESH_TYPE, MYTYPE>::Init(m, heap, pp);
	}
	
	
	void UpdateHeap(HeapType &heap)
	{
		this->GlobalMark()++;

		VertexPointer v0, v1, v2, v3;
		int flipped = (this->_pos.E() + 1) % 3;
		FacePointer f1 = this->_pos.F();
		FacePointer f2 = this->_pos.F()->FFp(flipped);

		v0 = f1->V0(flipped);
		v1 = f1->V1(flipped);
		v2 = f1->V2(flipped);
		v3 = f2->V2(f1->FFi(flipped));

		v0->IMark() = this->GlobalMark();
		v1->IMark() = this->GlobalMark();
		v2->IMark() = this->GlobalMark();
		v3->IMark() = this->GlobalMark();
		
		// edges of the first face, except the flipped edge
		for(int i = 0; i < 3; i++) if(i != flipped) {
			PosType newpos(f1, i);
			Insert(heap, newpos, this->GlobalMark());
		}

		// edges of the second face, except the flipped edge
		for(int i = 0; i < 3; i++) if(i != f1->FFi(flipped)) {
			PosType newpos(f2, i);
			Insert(heap, newpos, this->GlobalMark());
		}

		// every edge with v0, v1 v3 of f1
		for(int i = 0; i < 3; i++) {
			PosType startpos(f1, i);
			PosType pos(startpos);

			do { // go to the first border (if there is one)
				pos.NextE();
			} while(pos != startpos && !pos.IsBorder());
			
			// if a border is reached, set startpos here
			if(pos.IsBorder())
				startpos = pos;

			do {
				VertexPointer v = pos.VFlip();
				if(v != v0 && v != v1 && v != v2 && v != v3)
				Insert(heap, pos, this->GlobalMark());

				pos.NextE();
			} while(pos != startpos && !pos.IsBorder());
		}

		PosType startpos(f2, (f1->FFi(flipped) + 2) % 3);
		PosType pos(startpos);

		do { // go to the first border (if there is one)
			pos.NextE();
		} while(pos != startpos && !pos.IsBorder());
		
		// if a border is reached, set startpos here
		if(pos.IsBorder())
			startpos = pos;

		do {
			VertexPointer v = pos.VFlip();
			if(v != v0 && v != v1 && v != v2 && v != v3)
			Insert(heap, pos, this->GlobalMark());

			pos.NextE();
		} while(pos != startpos && !pos.IsBorder());
	}
};


} // end of namespace tri
} // end of namespace vcg

#endif
