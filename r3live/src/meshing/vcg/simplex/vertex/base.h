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
#ifndef __VCG_MESH
#error "This file should not be included alone. It is automatically included by complex.h"
#endif
#ifndef __VCG_VERTEX_PLUS
#define __VCG_VERTEX_PLUS

namespace vcg {


/* The base class form which we start to add our components.
it has the empty definition for all the standard members (coords, color flags)
Note:
in order to avoid both virtual classes and ambiguous definitions all 
the subsequent overrides must be done in a sequence of derivation.

In other words we cannot derive and add in a single derivation step 
(with multiple ancestor), both the real (non-empty) normal and color but 
we have to build the type a step a time (deriving from a single ancestor at a time). 

 The Real Big Vertex class;

The class __VertexArityMax__ is the one that is the Last to be derived,
and therefore is the only one to know the real members 
(after the many overrides) so all the functions with common behaviour 
using the members defined in the various Empty/nonEmpty component classes 
MUST be defined here. 

I.e. IsD() that uses the overridden Flags() member must be defined here.

*/

template <class UserTypes,
          template <typename> class A, template <typename> class B,
          template <typename> class C, template <typename> class D,
          template <typename> class E, template <typename> class F,
          template <typename> class G, template <typename> class H,
          template <typename> class I, template <typename> class J,
          template <typename> class K, template <typename> class L>
class VertexArityMax: public Arity12<vertex::EmptyCore<UserTypes>, A, B, C, D, E, F, G, H, I, J, K, L> {

// ----- Flags stuff -----
public:



 	enum { 
		
		DELETED    = 0x0001,		// This bit indicate that the vertex is deleted from the mesh
		NOTREAD    = 0x0002,		// This bit indicate that the vertex of the mesh is not readable
		NOTWRITE   = 0x0004,		// This bit indicate that the vertex is not modifiable
		MODIFIED   = 0x0008,		// This bit indicate that the vertex is modified
		VISITED    = 0x0010,		// This bit can be used to mark the visited vertex
		SELECTED   = 0x0020,		// This bit can be used to select 
		BORDER     = 0x0100,    // Border Flag
		USER0      = 0x0200			// First user bit
  };
 	
    bool IsD() const {return (this->cFlags() & DELETED) != 0;} ///  checks if the vertex is deleted
    bool IsR() const {return (this->cFlags() & NOTREAD) == 0;} ///  checks if the vertex is readable
    bool IsW() const {return (this->cFlags() & NOTWRITE)== 0;}///  checks if the vertex is modifiable
    bool IsRW() const {return (this->cFlags() & (NOTREAD | NOTWRITE)) == 0;}/// This funcion checks whether the vertex is both readable and modifiable
    bool IsS() const {return (this->cFlags() & SELECTED) != 0;}///  checks if the vertex is Selected
    bool IsB() const {return (this->cFlags() & BORDER) != 0;}///  checks if the vertex is a border one
    bool IsV() const {return (this->cFlags() & VISITED) != 0;}///  checks if the vertex Has been visited
	

	/** Set the flag value
		@param flagp Valore da inserire nel flag
	*/
	void SetFlags(int flagp) {this->Flags()=flagp;}

	/** Set the flag value
		@param flagp Valore da inserire nel flag
	*/
	void ClearFlags() {this->Flags()=0;}
	void SetD() {this->Flags() |=DELETED;}///  deletes the vertex from the mesh
	void ClearD() {this->Flags() &=(~DELETED);}///  un-delete a vertex
	void SetR() {this->Flags() &=(~NOTREAD);}///  marks the vertex as readable
	void ClearR() {this->Flags() |=NOTREAD;}///  marks the vertex as not readable
	void ClearW() {this->Flags() |=NOTWRITE;}///  marks the vertex as writable
	void SetW() {this->Flags() &=(~NOTWRITE);}///  marks the vertex as not writable
	void SetS()		{this->Flags() |=SELECTED;}///  select the vertex
	void ClearS()	{this->Flags() &= ~SELECTED;}/// Un-select a vertex
	void SetB()		{this->Flags() |=BORDER;}
	void ClearB()	{this->Flags() &=~BORDER;}
	void SetV()		{this->Flags() |=VISITED;}
	void ClearV()	{this->Flags() &=~VISITED;}
	
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
  {	  bb.Set(this->cP());  }

          };

          
/*

These are the three main classes that are used by the library user to define its own vertexes.
The user MUST specify the names of all the type involved in a generic complex.
so for example when defining a vertex of a trimesh you must know the name of the type of the edge and of the face.
Typical usage example:

A vertex with coords, flags and normal for use in a standard trimesh:

class VertexNf   : public VertexSimp2< VertexNf, EdgeProto, FaceProto, vert::Coord3d, vert::Flag, vert::Normal3f  > {};


A vertex with coords, and normal for use in a tetrahedral mesh AND in a standard trimesh:

class TetraVertex   : public VertexSimp3< TetraVertex, EdgeProto, FaceProto, TetraProto, vert::Coord3d, vert::Normal3f  > {};


A summary of the available vertex attributes (see component.h for more details):
          
Coord3f,  Coord3d, 
Normal3s,  Normal3f,  Normal3d
Mark                              //a int component (incremental mark)
BitFlags
TexCoord2s,  TexCoord2f,  TexCoord2d
Color4b
Qualitys, Qualityf, Qualityd
VFAdj                             //topology (vertex->face adjacency)
*/

template <class UserTypes,
          template <typename> class A = DefaultDeriver, template <typename> class B = DefaultDeriver,
          template <typename> class C = DefaultDeriver, template <typename> class D = DefaultDeriver,
          template <typename> class E = DefaultDeriver, template <typename> class F = DefaultDeriver,
          template <typename> class G = DefaultDeriver, template <typename> class H = DefaultDeriver,
					template <typename> class I = DefaultDeriver, template <typename> class J = DefaultDeriver,
					template <typename> class K = DefaultDeriver, template <typename> class L = DefaultDeriver>
							class Vertex: public VertexArityMax<UserTypes, A, B, C, D, E, F, G, H, I, J, K, L>  {
			 public: typedef AllTypes::AVertexType IAm; typedef UserTypes TypesPool;};

}// end namespace
#endif
