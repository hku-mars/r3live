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
#ifndef __VCG_FACE_PLUS
#define __VCG_FACE_PLUS

namespace vcg {

/*------------------------------------------------------------------*/ 
/* 
The base class of all the recusive definition chain. It is just a container of the typenames of the various simplexes.
These typenames must be known form all the derived classes.
*/

template <class UserTypes>
				class FaceTypeHolder: public UserTypes {
  public:

	template <class LeftF>
	void ImportData(const LeftF & ){}
    static void Name(std::vector<std::string> & /* name */){}


 // prot
        inline int VN()  const { return 3;}
        inline int Prev(const int & i) const { return (i+(3-1))%3;}
        inline int Next(const int & i) const { return (i+1)%3;}
	inline void Alloc(const int & ){}
	inline void Dealloc(){}
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
template <class UserTypes>
class FaceBase: public
			face::EmptyCore< FaceTypeHolder <UserTypes> > {
};


/* The Real Big Face class;

The class __FaceArityMax__ is the one that is the Last to be derived,
and therefore is the only one to know the real members 
(after the many overrides) so all the functions with common behaviour 
using the members defined in the various Empty/nonEmpty component classes 
MUST be defined here. 

I.e. IsD() that uses the overridden Flags() member must be defined here.

*/

template < class UserTypes,
          template <typename> class A, template <typename> class B, 
          template <typename> class C, template <typename> class D, 
          template <typename> class E, template <typename> class F,
          template <typename> class G, template <typename> class H,
          template <typename> class I, template <typename> class J > 
					class FaceArityMax: public J<Arity9<FaceBase<UserTypes>, A, B, C, D, E, F, G, H, I> > {

public:
	typedef typename  FaceArityMax::ScalarType ScalarType;
// ----- Flags stuff -----

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
		BORDER012     = BORDER0 | BORDER1 | BORDER2 ,
		// Face Orientation Flags, used efficiently compute point face distance  
		NORMX				= 0x00000200,
		NORMY				= 0x00000400,
		NORMZ				= 0x00000800,
		// Crease _flags,  it is assumed that CREASEi = CREASE0<<i 
		CREASE0    = 0x00008000,
		CREASE1    = 0x00010000,
		CREASE2    = 0x00020000,
		// Faux edges. (semantics: when a mesh is polygonal, edges which are inside a polygonal face are "faux"
		FAUX0       = 0x00040000,
		FAUX1       = 0x00080000,
		FAUX2       = 0x00100000,
		FAUX012     = FAUX0 | FAUX1 | FAUX2 ,
		// First user bit
		USER0       = 0x00200000
			};

 
 	///  checks if the Face is deleted
    bool IsD() const {return (this->cFlags() & DELETED) != 0;}
	///  checks if the Face is readable
	bool IsR() const {return (this->cFlags() & NOTREAD) == 0;}
	///  checks if the Face is modifiable
	bool IsW() const {return (this->cFlags() & NOTWRITE)== 0;}
	/// This funcion checks whether the Face is both readable and modifiable
	bool IsRW() const {return (this->cFlags() & (NOTREAD | NOTWRITE)) == 0;}
	///  checks if the Face is Modified
	bool IsS() const {return (this->cFlags() & SELECTED) != 0;}
	///  checks if the Face is Modified
	bool IsV() const {return (this->cFlags() & VISITED) != 0;}
	
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
	bool IsB(int i) const {return (this->cFlags() & (BORDER0<<i)) != 0;}
	/// This function select the face
  void SetB(int i)		{this->Flags() |=(BORDER0<<i);}
	/// This funcion execute the inverse operation of SetS()
	void ClearB(int i)	{this->Flags() &= (~(BORDER0<<i));}

	/// This function checks if the face is selected
	bool IsCrease(int i) const {return (this->cFlags() & (CREASE0<<i)) != 0;}
	/// This function select the face
	void SetCrease(int i){this->Flags() |=(CREASE0<<i);}
	/// This funcion execute the inverse operation of SetS()
	void ClearCrease(int i)	{this->Flags() &= (~(CREASE0<<i));}

	/// This function checks if a given side of the face is a feature/internal edge
	/// it is used by some importer to mark internal 
	/// edges of polygonal faces that have been triangulated
	bool IsF(int i) const {return (this->cFlags() & (FAUX0<<i) ) != 0;}
	bool IsAnyF() const {return (this->cFlags() & (FAUX0|FAUX1|FAUX2)) != 0;}
	/// This function select the face
	void SetF(int i)		{this->Flags() |=(FAUX0<<i);}
	/// This funcion execute the inverse operation of SetS()
	void ClearF(int i)	{this->Flags() &= (~(FAUX0<<i));}
	void ClearAllF() { this->Flags() &= (~(FAUX0|FAUX1|FAUX2)); }
	
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


  void GetBBox(Box3<ScalarType>& bb ) const
  {
    if(this->IsD()) {
        bb.SetNull();
        return;
      }
        bb.Set(this->cP(0));
        bb.Add(this->cP(1));
        bb.Add(this->cP(2));
  }


};


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
NormalFromVert, WedgeNormal
Normal3s, Normal3f, Normal3d
WedgeTexCoord2s, WedgeTexCoord2f, WedgeTexCoord2d
BitFlags
WedgeColor, Color4b
Qualitys, Qualityf, Qualityd
Mark                                            //Incremental mark (int)
VFAdj                                           //Topology vertex face adjacency
                                                 (pointers to next face in the ring of the vertex
FFAdj                                           //topology: face face adj
                                                  pointers to adjacent faces

*/

template <class UserTypes,
          template <typename> class A = DefaultDeriver, template <typename> class B = DefaultDeriver,
          template <typename> class C = DefaultDeriver, template <typename> class D = DefaultDeriver,
          template <typename> class E = DefaultDeriver, template <typename> class F = DefaultDeriver,
          template <typename> class G = DefaultDeriver, template <typename> class H = DefaultDeriver,
          template <typename> class I = DefaultDeriver, template <typename> class J = DefaultDeriver >
							class Face: public FaceArityMax<UserTypes, A, B, C, D, E, F, G, H, I, J>  {
							public: typedef AllTypes::AFaceType IAm; typedef UserTypes TypesPool;};


}// end namespace
#endif

