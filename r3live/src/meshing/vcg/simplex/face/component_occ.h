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

/* 
OCC = Optional Component Compact
compare with OCF(Optional Component Fast)
*/

#ifndef __VCG_MESH
#error "This file should not be included alone. It is automatically included by complex.h"
#endif
#ifndef __VCG_FACE_PLUS_COMPONENT_OCC
#define __VCG_FACE_PLUS_COMPONENT_OCC

#include <vcg/simplex/face/component.h>
#include <vcg/container/vector_occ.h>
#include <vcg/space/plane3.h>


namespace vcg {
  namespace face {

	///*-------------------------- WedgeTexCoordOcc ----------------------------------------*/ 

	template <class A, class T> class WedgeTexCoordOcc: public T {
	public:
		typedef A WedgeTexCoordType;
		typedef typename T::FaceType FaceType;
		WedgeTexCoordType &WT(const int&i) {return CAT< vector_occ<FaceType>,WedgeTexCoordType>::Instance()->Get((FaceType*)this);}
	  static bool HasWedgeTexCoord()   { return true; }
		static bool HasWedgeTexCoordOcc()   { return true; }
	};

	template <class T> class WedgeTexCoordfOcc: public WedgeTexCoordOcc<TexCoord2<float,1>, T> {};

	///*-------------------------- FACEINFO ----------------------------------------*/ 

	template <class A, class T> class InfoOccBase: public T {
	public:
		typedef A InfoType;
		typedef typename T::FaceType FaceType;	
		InfoType &N() {return CAT< vector_occ<FaceType>,InfoType>::Instance()->Get((FaceType*)this);}
	  static bool HasInfo()   { return true; }
		static bool HasInfoOcc()   { return true; }
	};

	template <class T> class InfoOcc: public InfoOccBase<int, T> {};

///*-------------------------- NORMAL ----------------------------------------*/ 

	template <class A, class T> class NormalOcc: public T {
	public:
		typedef A NormalType;
		typedef typename T::FaceType FaceType;	
		NormalType &N() {return CAT< vector_occ<FaceType>,NormalType>::Instance()->Get((FaceType*)this);}
	  static bool HasFaceNormal()   { return true; }
		static bool HasFaceNormalOcc()   { return true; }
	};

	template <class T> class Normal3sOcc: public NormalOcc<vcg::Point3s, T> {};
	template <class T> class Normal3fOcc: public NormalOcc<vcg::Point3f, T> {};
	template <class T> class Normal3dOcc: public NormalOcc<vcg::Point3d, T> {};

///*-------------------------- MARK ----------------------------------------*/ 

	template <class T> class MarkOcc: public T {
	public:
		typedef int MarkType;
		typedef typename T::FaceType FaceType;	
		int &IMark() {return CAT< vector_occ<FaceType>,MarkType>::Instance()->Get((MarkType*)this);}
	  static bool HasFaceMark()   { return true; }
		static bool HasFaceMarkOcc()   { return true; }
	  inline void InitIMark()    { IMark() = 0; }
	};

///*-------------------------- COLOR ----------------------------------------*/ 

template <class A, class T> class ColorOcc: public T {
public:
		typedef A ColorType;
		typedef typename T::FaceType FaceType;	
		ColorType &C() { return CAT< vector_occ<FaceType>,ColorType>::Instance()->Get((FaceType*)this); }
		static bool HasFaceColor()   { return true; }
		static bool HasfaceColorOcc()   { return true; }
};

template <class T> class Color4bOcc: public ColorOcc<vcg::Color4b, T> {};

/*----------------------------- VFADJ ---------------------------------------*/ 

// questo tipo serve per tenere tutte le informazioni sull'adiacenza dentro una
// singola classe
template <class FP>
struct VFAdjTypeSup {
		FP  _vfp[3]; 
		char _vfi[3];
	};

template <class A, class T> class VFAdjOccBase: public T {
public:
//	typedef A VFAdjType;
	typedef VFAdjTypeSup<typename T::VertexPointer> VFAdjType;
	typedef typename T::FaceType FaceType;	
	typedef typename T::FacePointer FacePointer;	

  	FacePointer &VFp(const int j) {
		return (CAT< vector_occ<FaceType>,VFAdjTypeSup<FacePointer> >::Instance()->Get((FaceType*)this))._vfp[j];}

   	FacePointer cVFp(const int j) const {
		return (CAT< vector_occ<FaceType>,VFAdjTypeSup<FacePointer> >::Instance()->Get((FaceType*)this))._vfp[j];}

  char &VFi(const int j) { return (CAT< vector_occ<FaceType>,VFAdjTypeSup<FacePointer> >::Instance()->Get((FaceType*)this))._vfi[j];}

  static bool HasVFAdjacency()   {   return true; }
  static bool HasVFAdjacencyOcc()   { return true; }
};

template <class T> class VFAdjOcc : public VFAdjOccBase<VFAdjTypeSup<typename T::FacePointer>,T>{};

/*----------------------------- FFADJ -----------------------------------*/ 

// questo tipo serve per tenere tutte le informazioni sull'adiacenza dentro una
// singola classe
template <class FP>
struct FFAdjTypeSup {
		FP  _ffp[3]; 
		char _ffi[3];
	};

template <class A, class T> class FFAdjOccBase: public T {
public:
	
//	typedef  A FFAdjType;
	typedef FFAdjTypeSup<typename T::FacePointer> FFAdjType;
	typedef typename T::FaceType FaceType;
	typedef typename T::FacePointer FacePointer;
	
  	FacePointer &FFp(const int j) {
		return (CAT< vector_occ<FaceType>,FFAdjTypeSup<FacePointer> >::Instance()->Get((FaceType*)this))._ffp[j];}

   	FacePointer const  FFp(const int j) const { 
		return (CAT< vector_occ<FaceType>,FFAdjTypeSup<FacePointer> >::Instance()->Get((FaceType*)this))._ffp[j];}

	FacePointer const cFFp(const int j) const {
   		 return (CAT< vector_occ<FaceType>,FFAdjTypeSup<FacePointer> >::Instance()->Get((FaceType*)this))._ffp[j];}
 
  	char &FFi(const int j) {
  		 return (CAT< vector_occ<FaceType>,FFAdjTypeSup<FacePointer> >::Instance()->Get((FaceType*)this))._ffi[j];}  

  	char cFFi(const int j) const{
   		return (CAT< vector_occ<FaceType>,FFAdjTypeSup<FacePointer> >::Instance()->Get((FaceType*)this ))._ffi[j];
	}  

  static bool HasFFAdjacency()   {   return true; }
  static bool HasFFAdjacencyOcc()   { return true; }

};

template <class T> class FFAdjOcc : public FFAdjOccBase<FFAdjTypeSup<typename T::FacePointer>,T>{};

template <class T> class VertexRefOcc: public T {
public:

  typedef typename T::VertexType VertexType;
  typedef typename T::FaceType FaceType;
  typedef typename T::CoordType CoordType;

  inline typename T::VertexType *       & V( const int j ) 	     { assert(j>=0 && j<3); 
		return (CAT< vector_occ<FaceType>,VertexRef<T> >::Instance()->Get((FaceType*)this)).V(j); }

  inline typename T::VertexType * const & V( const int j ) const { assert(j>=0 && j<3); 
		return (CAT< vector_occ<FaceType>,VertexRef<T> >::Instance()->Get((FaceType*)this)).V(j); }

	inline typename T::VertexType * const  cV( const int j ) const { assert(j>=0 && j<3);	
		return (CAT< vector_occ<FaceType>,VertexRef<T> >::Instance()->Get((FaceType*)this)).V(j); }

	// Shortcut per accedere ai punti delle facce
	inline       typename T::CoordType & P( const int j ) 	    {	assert(j>=0 && j<3);	return	V(j)->P();	}
	inline const typename T::CoordType & P( const int j ) const	{	assert(j>=0 && j<3);	return  V(j)->cP(); }
	inline const typename T::CoordType &cP( const int j ) const	{	assert(j>=0 && j<3);	return  V(j)->cP(); }

	/** Return the pointer to the ((j+1)%3)-th vertex of the face.
		@param j Index of the face vertex.
	 */
	inline		VertexType *       &  V0( const int j )       { return V(j);}
	inline       	VertexType *       &  V1( const int j )       { return V((j+1)%3);}
	inline       	VertexType *       &  V2( const int j )       { return V((j+2)%3);}
	inline const 	VertexType * const &  V0( const int j ) const { return V(j);}
	inline const 	VertexType * const &  V1( const int j ) const { return V((j+1)%3);}
	inline const 	VertexType * const &  V2( const int j ) const { return V((j+2)%3);}
	inline const 	VertexType * const & cV0( const int j ) const { return cV(j);}
	inline const 	VertexType * const & cV1( const int j ) const { return cV((j+1)%3);}
	inline const 	VertexType * const & cV2( const int j ) const { return cV((j+2)%3);}

	/// Shortcut per accedere ai punti delle facce
	inline       	CoordType &  P0( const int j )       { return V(j)->P();}
	inline       	CoordType &  P1( const int j )       { return V((j+1)%3)->P();}
	inline       	CoordType &  P2( const int j )       { return V((j+2)%3)->P();}
	inline const 	CoordType &  P0( const int j ) const { return V(j)->P();}
	inline const 	CoordType &  P1( const int j ) const { return V((j+1)%3)->P();}
	inline const 	CoordType &  P2( const int j ) const { return V((j+2)%3)->P();}
	inline const 	CoordType & cP0( const int j ) const { return cV(j)->P();}
	inline const 	CoordType & cP1( const int j ) const { return cV((j+1)%3)->P();}
	inline const 	CoordType & cP2( const int j ) const { return cV((j+2)%3)->P();}

  static bool HasVertexRef()   { return true; }
};
  } // end namespace face

	template < class, class, class > class TriMesh;

	namespace tri
  {
/*		template < class VertContainerType, class FaceType >
			bool HasVFAdjacency (const TriMesh < VertContainerType , vector_occ< FaceType > > & m) 
		{
			if(  FaceType::HasVFAdjacencyOcc()) return m.face.IsEnabledAttribute< typename FaceType::VFAdjType >();
			else return  FaceType::HasVFAdjacency();
		}

		template < class VertContainerType, class FaceType >
			bool HasFFAdjacency (const TriMesh < VertContainerType , vector_occ< FaceType > > & m) 
		{
			if(FaceType::HasFFAdjacencyOcc()) return m.face.IsEnabledAttribute<typename FaceType::FFAdjType >();
			else return FaceType::HasFFAdjacency();
		}

		template < class VertContainerType, class FaceType >
			bool HasPerWedgeTexCoord (const TriMesh < VertContainerType , vector_occ< FaceType > > & m) 
		{
			if(FaceType::HasWedgeTexCoordOcc()) return m.face.IsEnabledAttribute<typename FaceType::WedgeTexCoordType >();
			else return FaceType::HasWedgeTexCoord();
		}

		template < class VertContainerType, class FaceType >
			bool HasPerFaceColor (const TriMesh < VertContainerType , vector_occ< FaceType > > & m) 
		{
			if(FaceType::HasFaceColorOcc()) return m.face.IsEnabledAttribute<typename FaceType::ColorType>();
			else return FaceType::HasFaceColor();
		}

		template < class VertContainerType, class FaceType >
			bool HasPerFaceMark (const TriMesh < VertContainerType , vector_occ< FaceType > > & m) 
		{
			if(FaceType::HasFaceMarkOcc()) return m.face.IsEnabledAttribute<typename FaceType::MarkType>();
			else return FaceType::HasFaceMark();
		}
*/
	}; // end namesace tri
}// end namespace vcg
#endif
