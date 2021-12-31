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

#ifndef __VCG_TETRA_POS
#define __VCG_TETRA_POS

namespace vcg {
  namespace tetra {

 /** \addtogroup tetra */
/*@{*/


  /**  Class VTIterator.
 This is a vertex - tetrahedron iterator
		@param MTTYPE (Template Parameter) Specifies the type of the tetrahedron.
 */
template < class MTTYPE> 
class VTIterator
{
public:
	/// The tetrahedron type 
	typedef  MTTYPE TetraType;
private:
	/// Pointer to a tetrahedron
	TetraType *_vt;
	/// Index of one vertex
	int _vi;
	/// Default Constructor
public:
	VTIterator(){}
	/// Constructor which associates the EdgePos elementet with a face and its edge
	VTIterator(TetraType  * const tp, int const zp)
	{	
		_vt=tp;
		_vi=zp;
  }

	~VTIterator(){};

  /// Return the tetrahedron stored in the half edge
inline TetraType* & Vt()
{
	return _vt;
} 

  	/// Return the index of vertex as seen from the tetrahedron
inline int & Vi()
{
	return _vi;
}

/// Return the index of vertex as seen from the tetrahedron
inline const int & Vi() const
{
		return _vi;
}

inline bool End(){return (Vt()==NULL);}

	/// move on the next tetrahedron that share the vertex
void operator++() 
{
		int vi=Vi();
		TetraType * tw = Vt();
		Vt() = tw->TVp(vi);
		Vi() = tw->TVi(vi);

		assert((Vt()==NULL)||((tw->V(vi))==(Vt()->V(Vi()))));
}

};


/**  Templated over the class tetrahedron, it stores a \em position over a tetrahedron in a mesh.
	It contain a pointer to the current tetrahedron, 
	the index of one face,edge and a edge's incident vertex.
 */
template < class MTTYPE> 
class Pos
{
public:

  /// The tetrahedron type
	typedef MTTYPE TetraType;
	/// The vertex type
	typedef	typename TetraType::VertexType VertexType;
  /// The coordinate type
	typedef	typename TetraType::VertexType::CoordType CoordType;
  ///The HEdgePos type
	typedef Pos<TetraType> BasePosType;

private:
	/// Pointer to the tetrahedron of the half-edge
	TetraType *_t;
	/// Index of the face
	char _f;
	/// Index of the edge
	char _e;
	/// Pointer to the vertex
	char _v;

public:
	/// Default constructor
	Pos(){SetNull();};
	/// Constructor which associates the half-edge elementet with a face, its edge and its vertex
	Pos(TetraType * const tp, char const fap,char const ep,
		char const vp){_t=tp;_f=fap;_e=ep;_v=vp;}

	~Pos(){};
  
  	/// Return the tetrahedron stored in the half edge
	inline TetraType* & T()
  {
		return _t;
	} 

  	/// Return the tetrahedron stored in the half edge
	inline  TetraType* const & T() const
	{
		return _t;
	}

  	/// Return the index of face as seen from the tetrahedron
	inline char & F()
	{
		return _f;
	}

  	/// Return the index of face as seen from the tetrahedron
	inline const char & F() const
	{
		return _f;
	}

	/// Return the index of face as seen from the tetrahedron
	inline char & E()
	{
		return _e;
	}

  	/// Return the index of face as seen from the tetrahedron
	inline const char & E() const
	{
		return _e;
	}

/// Return the index of vertex as seen from the tetrahedron
	inline char & V()
	{
		return _v;
	}

  	/// Return the index of vertex as seen from the tetrahedron
	inline const char & V() const
	{
		return _v;
	}

	/// Operator to compare two half-edge
	inline bool operator == ( BasePosType const & p ) const {
			return (T()==p.T() && F()==p.F() && E==p.E() && V==p.V());
	} 

	/// Operator to compare two half-edge
	inline bool operator != ( BasePosType const & p ) const {
			return (!((*this)==p));
	} 
   
	/// Set to null the half-edge
	void SetNull(){
		T()=0;
		F()=-1;
		E()=-1;
		V()=-1;
	}

	/// Check if the half-edge is null
	bool IsNull() const {
		return ((T()==0) || (F()<0) || (E()<0) || (V()<0));
	}


	/// Changes edge maintaining the same face and the same vertex
	void FlipE()
	{
		
		//take the absolute index of the tree edges of the faces
    char e0=vcg::Tetra::EofF(_f ,0);
		char e1=vcg::Tetra::EofF(_f ,1);
		char e2=vcg::Tetra::EofF(_f ,2);
		//eliminate the same as himself
		if (e0==E())
			{
			 e0=e1;
			 e1=e2;
			}
		else
		if (e1==E())
			{
			 e1=e2;
			}

		//now choose the one that preserve the same vertex
     if ((vcg::Tetra::VofE(e1,0)==V())||(vcg::Tetra::VofE(e1,1)==V()))
			E()=e1;
		else
			E()=e0;
	}


	/// Changes vertex maintaining the same face and the same edge
	void FlipV()
	{
		// in the same edge choose the one that change
		char v0=vcg::Tetra::VofE(E(),0);
		char v1=vcg::Tetra::VofE(E(),1);
		if (v0!=V())
			V()=v0;
		else
			V()=v1;
	}

	/// Changes face maintaining the same vertex and the same edge
	void FlipF()
	{
    char f0=vcg::Tetra::FofE(E(),0);
		char f1=vcg::Tetra::FofE(E(),1);
		if (f0!=F())
			F()=f0;
		else
			F()=f1;
	}

	/// Changes tetrahedron maintaining the same face edge and vertex'... to finish
	void FlipT()
	{
		
		//save the two vertices of the old edge
		VertexType *v0=T()->V(vcg::Tetra::VofE(E(),0));
		VertexType *v1=T()->V(vcg::Tetra::VofE(E(),1));

    //get the current vertex
    VertexType *vcurr=T()->V(V());

		//get new tetrahedron according to faceto face topology
		TetraType *nt=T()->TTp(F());
		char nfa=T()->TTi(F());
		if (nfa!=-1)
		{
			//find the right edge
      char ne0=vcg::Tetra::EofF(nfa,0);
			char ne1=vcg::Tetra::EofF(nfa,1);
			char ne2=vcg::Tetra::EofF(nfa,2);
      
      //the vertices of new edges
      VertexType *vn0=nt->V(vcg::Tetra::VofE(ne0,0));
      VertexType *vn1=nt->V(vcg::Tetra::VofE(ne0,1));
			//verify that the two vertices of tetrahedron are identical
			if (((vn0==v0)&&(vn1==v1))||((vn1==v0)&&(vn0==v1)))
			  E()=ne0;
			else
      {
        vn0=nt->V(vcg::Tetra::VofE(ne1,0));
        vn1=nt->V(vcg::Tetra::VofE(ne1,1));
			  if (((vn0==v0)&&(vn1==v1))||((vn1==v0)&&(vn0==v1)))
			    E()=ne1;
        else
        {
#ifdef _DEBUG
          vn0=nt->V(vcg::Tetra::VofE(ne2,0));
          vn1=nt->V(vcg::Tetra::VofE(ne2,1));
          assert(((vn0==v0)&&(vn1==v1))||((vn1==v0)&&(vn0==v1)));
#endif
          E()=ne2;
        }
      }

      //find the right vertex
      vn0=nt->V(vcg::Tetra::VofE(E(),0));
#ifdef _DEBUG
      vn1=nt->V(vcg::Tetra::VofE(E(),1));
      assert((vn0==vcurr)||(vn1==vcurr));
#endif
      if (vn0==vcurr)
        V()=vcg::Tetra::VofE(E(),0);
      else
        V()=vcg::Tetra::VofE(E(),1);

      T()=nt;
      assert(T()->V(V())==vcurr);
			F()=nfa;
    }
  }

	///returns the next half edge on the same edge
	void NextT( )
	{
		#ifdef _DEBUG
		VertexType *vold=T()->V(V());
		#endif
		FlipT();
		FlipF();
		#ifdef _DEBUG
		VertexType *vnew=T()->V(V());
		assert(vold==vnew);
		#endif	
	}

	void Assert()
	#ifdef _DEBUG
	{	
		HETYPE ht=*this;
		ht.FlipT();
		ht.FlipT();
		assert(ht==*this);

		ht=*this;
		ht.FlipF();
		ht.FlipF();
		assert(ht==*this);

		ht=*this;
		ht.FlipE();
		ht.FlipE();
		assert(ht==*this);

		ht=*this;
		ht.FlipV();
		ht.FlipV();
		assert(ht==*this);
	}
	#else
	{}
	#endif
};

///this pos structure jump on  next tetrahedron if find an external face
template < class MTTYPE> 
class PosJump:public Pos<MTTYPE>
{
private:
	MTTYPE *_t_initial;
	short int _back;
public :
	typedef  MTTYPE  TetraType;
  PosJump(const TetraType*  tp,const int  fap,const int  ep,
		int  vp){this->T()=tp;this->F()=fap;this->E()=ep;this->V()=vp;_t_initial=tp;_back=0;}

	void NextT()
  {
#ifdef _DEBUG
		int cont=0;
#endif
		MTTYPE *tpred=this->T();
		Pos<MTTYPE>::NextT();
		//external face
		if (tpred==this->T())
		{
			while (this->T()!=_t_initial)
			{
				Pos<MTTYPE>::NextT();
				#ifdef _DEBUG
				 cont++;
				 assert (cont<500);
				#endif
			}
			_back++;
			if (_back==1)
			{
			  Pos<MTTYPE>::NextT();
			}	
    }
	}
};

///this pos structure jump on  next tetrahedron in rotational sense if find an external face
template < class MTTYPE> 
class PosLoop:public Pos<MTTYPE>
{
private:
	MTTYPE *_t_initial;
	bool _jump;
  bool _loop;
public :
	typedef  MTTYPE  TetraType;
PosLoop(TetraType*  tp,const int  fap,const int  ep,
		int vp){this->T()=tp;this->F()=fap;this->E()=ep;this->V()=vp;_t_initial=tp;_jump=false;_loop=false;}

  bool LoopEnd()
  {
    return (_loop);
  }
  
  bool Jump()
  {
    return(_jump);
  }
  
  void Reset()
  {
    _loop=false;
    _jump=false;
  }

	void NextT()
  {	
#ifdef _DEBUG
    TetraType *t_old=this->T();
#endif
		TetraType *tpred=this->T();
		Pos<TetraType>::NextT();
    _loop=false;
    _jump=false;

		//external face
		if (tpred==this->T())
		{
      tpred=this->T();
      //jump next one
      Pos<TetraType>::NextT();
      //find the next external face
			while (tpred!=this->T())
			{
        tpred=this->T();
				Pos<TetraType>::NextT();
			}
			////reset right rotation sense
			//  Pos<TetraType>::NextT();
        _jump=true;
    }	
    if (this->T()==_t_initial)
      _loop=true;
#ifdef _DEBUG
    if (_loop==false)
      assert(t_old!=this->T());
#endif
  }

};
//@}
  }//end namespace tetra
}//end namespace vcg

#endif
