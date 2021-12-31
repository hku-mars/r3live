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
Revision 1.1  2007/05/09 10:31:53  ganovelli
added



****************************************************************************/
#ifndef __VCG_TETRA_PLUS
#define __VCG_TETRA_PLUS

#include <vcg/space/point3.h>
#include <vcg/space/texcoord2.h>
#include <vcg/space/color4.h>
#include <vcg/simplex/tetrahedron/component.h>

namespace vcg {

/*------------------------------------------------------------------*/ 
/* 
The base class of all the recusive definition chain. It is just a container of the typenames of the various simplexes.
These typenames must be known form all the derived classes.
*/

template <class BVT, class BET, class BFT, class BTT>
class TetraTypeHolder{
  public:
  typedef BVT VertexType;
  typedef typename VertexType::CoordType CoordType;
  typedef typename VertexType::ScalarType ScalarType;
  typedef BET EdgeType;
  typedef BFT FaceType;
  typedef BTT TetraType;
  typedef BVT *VertPointer;
  typedef BET *EdgePointer;
  typedef BFT *FacePointer;
  typedef BTT *TetraPointer;
  static void Name(std::vector<std::string> & name){}


 // prot
 
};

/* The base class form which we start to add our components.
it has the empty definition for all the standard members (coords, color flags)
Note:
in order to avoid both virtual classes and ambiguous definitions all 
the subsequent overrides must be done in a sequence of derivation.

In other words we cannot derive and add in a single derivation step 
(with multiple ancestor), both the real (non-empty) normal and color but 
we have to build the type a step a time (deriving from a single ancestor at a time). 


*/ 
template <class BVT, class BET=DumET, class BFT=DumFT, class BTT=DumTT>
class TetraBase: public  tetra::EmptyVertexRef<
                         tetra::EmptyAdj<
                         TetraTypeHolder <BVT, BET, BFT, BTT> > > {
};



// Metaprogramming Core

template <class BVT, class BET, class BFT,class BTT,
          template <typename> class A> 
          class TetraArity1: public A<TetraBase<BVT,BET,BFT,BTT> > {};

template <class BVT, class BET, typename BFT, class BTT,
          template <typename> class A, template <typename> class B> 
          class TetraArity2: public B<TetraArity1<BVT,BET,BFT,BTT, A> > {};

template <class BVT, class BET, typename BFT,class BTT,
          template <typename> class A, template <typename> class B, 
          template <typename> class C > 
          class TetraArity3: public C<TetraArity2<BVT,BET,BFT,BTT, A, B> > {};

template <class BVT, class BET, typename BFT,class BTT,
          template <typename> class A, template <typename> class B, 
          template <typename> class C, template <typename> class D> 
          class TetraArity4: public D<TetraArity3<BVT,BET,BFT,BTT, A, B, C> > {};

template <class BVT, class BET, typename BFT,class BTT,
          template <typename> class A, template <typename> class B, 
          template <typename> class C, template <typename> class D,
          template <typename> class E > 
          class TetraArity5: public E<TetraArity4<BVT,BET,BFT,BTT, A, B, C, D> > {};

template <class BVT, class BET, typename BFT,class BTT,
          template <typename> class A, template <typename> class B, 
          template <typename> class C, template <typename> class D,
          template <typename> class E, template <typename> class F > 
          class TetraArity6: public F<TetraArity5<BVT,BET,BFT,BTT, A, B, C, D, E> > {};

template <class BVT, class BET, typename BFT,class BTT,
          template <typename> class A, template <typename> class B, 
          template <typename> class C, template <typename> class D,
          template <typename> class E, template <typename> class F, 
          template <typename> class G  > 
          class TetraArity7: public G<TetraArity6<BVT,BET,BFT,BTT, A, B, C, D, E, F> > {};

template <class BVT, class BET, typename BFT,class BTT,
          template <typename> class A, template <typename> class B, 
          template <typename> class C, template <typename> class D,
          template <typename> class E, template <typename> class F, 
          template <typename> class G, template <typename> class H  > 
          class TetraArity8: public H<TetraArity7<BVT,BET,BFT,BTT, A, B, C, D, E, F, G> > {};

/* The Real Big Face class;

The class __FaceArityMax__ is the one that is the Last to be derived,
and therefore is the only one to know the real members 
(after the many overrides) so all the functions with common behaviour 
using the members defined in the various Empty/nonEmpty component classes 
MUST be defined here. 

I.e. IsD() that uses the overridden Flags() member must be defined here.

*/

template <class BVT, class BET, typename BFT,class BTT,
          template <typename> class A, template <typename> class B, 
          template <typename> class C, template <typename> class D, 
          template <typename> class E, template <typename> class F,
          template <typename> class G, template <typename> class H,
          template <typename> class I  > 
          class TetraArityMax: public I<TetraArity8<BVT,BET,BFT,BTT, A, B, C, D, E, F, G, H> > {

// ----- Flags stuff -----
public:
  
   	enum { 
		
		DELETED     = 0x00000001,		// Face is deleted from the mesh
		NOTREAD     = 0x00000002,		// Face of the mesh is not readable
		NOTWRITE    = 0x00000004,		// Face of the mesh is not writable
    VISITED     = 0x00000010,		// Face has been visited. Usualy this is a per-algorithm used bit. 
		SELECTED    = 0x00000020,		// Face is selected. Algorithms should try to work only on selected face (if explicitly requested)
		// Border _flags, it is assumed that BORDERi = BORDER0<<i 
		BORDER0     = 0x00000040,
		BORDER1     = 0x00000080,
		BORDER2     = 0x00000100,
		BORDER3     = 0x00000200,
		// Crease _flags,  it is assumed that FEATUREi = FEATURE0<<i 
		// First user bit
		USER0       = 0x00004000
			};

 
 	///  checks if the Face is deleted
	bool IsD() const {return (this->Flags() & DELETED) != 0;}
	///  checks if the Face is readable
	bool IsR() const {return (this->Flags() & NOTREAD) == 0;}
	///  checks if the Face is modifiable
	bool IsW() const {return (this->Flags() & NOTWRITE)== 0;}
	/// This funcion checks whether the Face is both readable and modifiable
	bool IsRW() const {return (this->Flags() & (NOTREAD | NOTWRITE)) == 0;}
	///  checks if the Face is Modified
	bool IsS() const {return (this->Flags() & SELECTED) != 0;}
	///  checks if the Face is Modified
	bool IsV() const {return (this->Flags() & VISITED) != 0;}
	
	/** Set the flag value
		@param flagp Valore da inserire nel flag
	*/
	void SetFlags(int flagp) {this->Flags()=flagp;}

	/** Set the flag value
		@param flagp Valore da inserire nel flag
	*/
	void ClearFlags() {this->Flags()=0;}

	///  deletes the Face from the mesh
	void SetD() {this->Flags() |=DELETED;}
	///  un-delete a Face
	void ClearD() {this->Flags() &=(~DELETED);}
	///  marks the Face as readable
	void SetR() {this->Flags() &=(~NOTREAD);}
	///  marks the Face as not readable
	void ClearR() {this->Flags() |=NOTREAD;}
	///  marks the Face as writable
	void SetW() {this->Flags() &=(~NOTWRITE);}
	///  marks the Face as notwritable
	void ClearW() {this->Flags() |=NOTWRITE;}
	///  select the Face
	void SetS()		{this->Flags() |=SELECTED;}
	/// Un-select a Face
  void ClearS()	{this->Flags() &= ~SELECTED;}
	///  select the Face
	void SetV()		{this->Flags() |=VISITED;}
	/// Un-select a Face
  void ClearV()	{this->Flags() &= ~VISITED;}
	
	/// This function checks if the face is selected
	bool IsB(int i) const {return (this->Flags() & (BORDER0<<i)) != 0;}
	/// This function select the face
  void SetB(int i)		{this->Flags() |=(BORDER0<<i);}
	/// This funcion execute the inverse operation of SetS()
	void ClearB(int i)	{this->Flags() &= (~(BORDER0<<i));}
	
	///  Return the first bit that is not still used
	static int &FirstUnusedBitFlag()
	{
	  static int b =USER0;
	  return b;
	}

	/// Allocate a bit among the flags that can be used by user. It updates the FirstUnusedBitFlag.
	static inline int NewBitFlag()
	{
	  int bitForTheUser = FirstUnusedBitFlag();
	  FirstUnusedBitFlag()=FirstUnusedBitFlag()<<1;
	  return bitForTheUser;
	}

	/// De-allocate a pre allocated bit. It updates the FirstUnusedBitFlag.
	// Note you must deallocate bit in the inverse order of the allocation (as in a stack)
	static inline bool DeleteBitFlag(int bitval)
	{
	  if(FirstUnusedBitFlag()>>1==bitval) {
		FirstUnusedBitFlag() = FirstUnusedBitFlag()>>1;
		return true;
	  }
	  assert(0);
	  return false;
	}

	/// This function checks if the given user bit is true
	bool IsUserBit(int userBit){return (this->Flags() & userBit) != 0;}

	/// This function set the given user bit
	void SetUserBit(int userBit){this->Flags() |=userBit;}

	/// This function clear the given user bit
	void ClearUserBit(int userBit){this->Flags() &= (~userBit);}

 template<class BoxType>
  void GetBBox( BoxType & bb ) const
  {
	  bb.Set(this->P(0));
	  bb.Add(this->P(1));
	  bb.Add(this->P(2));
  }


};

template < typename T=int>
class TetraDefaultDeriver : public T {};
          
/*

These are the three main classes that are used by the library user to define its own Facees.
The user MUST specify the names of all the type involved in a generic complex.
so for example when defining a Face of a trimesh you must know the name of the type of the edge and of the face.
Typical usage example:

A Face with coords, flags and normal for use in a standard trimesh:

class MyFaceNf   : public FaceSimp2< VertProto, EdgeProto, MyFaceNf, face::Flag, face::Normal3f  > {};


A Face with coords, and normal for use in a tetrahedral mesh AND in a standard trimesh:

class TetraFace   : public FaceSimp3< VertProto, EdgeProto, TetraFace, TetraProto, face::Coord3d, face::Normal3f  > {};


A summary of the components that can be added to a face (see components.h for details):
          
VertexRef
Mark                                            //Incremental mark (int)
VTAdj                                           //Topology vertex face adjacency
                                                 (pointers to next face in the ring of the vertex
TTAdj                                           //topology: face face adj
                                                  pointers to adjacent faces

*/

template <class BVT, class BET, class BFT, class BTT,
          template <typename> class A = TetraDefaultDeriver, template <typename> class B = TetraDefaultDeriver,
          template <typename> class C = TetraDefaultDeriver, template <typename> class D = TetraDefaultDeriver,
          template <typename> class E = TetraDefaultDeriver, template <typename> class F = TetraDefaultDeriver,
          template <typename> class G = TetraDefaultDeriver, template <typename> class H = TetraDefaultDeriver,
          template <typename> class I = TetraDefaultDeriver > 
              class TetraSimp3: public TetraArityMax<BVT,BET,BFT,BTT, A, B, C, D, E, F, G, H, I>  {};
class DumTT;
template <class BVT, class BET, class BFT, 
          template <typename> class A = TetraDefaultDeriver, template <typename> class B = TetraDefaultDeriver,
          template <typename> class C = TetraDefaultDeriver, template <typename> class D = TetraDefaultDeriver,
          template <typename> class E = TetraDefaultDeriver, template <typename> class F = TetraDefaultDeriver,
          template <typename> class G = TetraDefaultDeriver, template <typename> class H = TetraDefaultDeriver,
          template <typename> class I = TetraDefaultDeriver > 
              class TetraSimp2: public TetraArityMax<BVT,BET,BFT,DumTT, A, B, C, D, E, F, G, H, I>  {};


}// end namespace
#endif

