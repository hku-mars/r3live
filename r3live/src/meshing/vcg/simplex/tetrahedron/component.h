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
#ifndef __VCG_TETRAHEDRON_PLUS_COMPONENT
#define __VCG_TETRAHEDRON_PLUS_COMPONENT 

#include <vector>
#include <vcg/space/tetra3.h>

namespace vcg {
  namespace tetra {
/*
Some naming Rules
All the Components that can be added to a vertex should be defined in the namespace vert:

*/

/*-------------------------- VERTEX ----------------------------------------*/ 
template <class T> class EmptyVertexRef: public T {
public:
 // typedef typename T::VertexType VertexType;
 // typedef typename T::CoordType CoordType;
  inline typename T::VertexType *       & V( const int j ) 	    {	assert(0);		static typename T::VertexType *vp=0; return vp; }
  inline typename T::VertexType * const & V( const int j ) const {	assert(0);		static typename T::VertexType *vp=0; return vp; }
	inline typename T::VertexType * const  cV( const int j ) const {	assert(0);		static typename T::VertexType *vp=0; return vp;	}
	inline       typename T::CoordType & P( const int j ) 	    {	assert(0);		static typename T::CoordType coord(0, 0, 0); return coord;	}
	inline const typename T::CoordType & P( const int j ) const {	assert(0);		static typename T::CoordType coord(0, 0, 0); return coord;	}
	inline const typename T::CoordType &cP( const int j ) const	{	assert(0);		static typename T::CoordType coord(0, 0, 0); return coord;	}
  static bool HasVertexRef()   { return false; }
	static void Name(std::vector<std::string> & name){T::Name(name);}

};
template <class T> class VertexRef: public T {
public:
	VertexRef(){
		v[0]=0;
		v[1]=0;
		v[2]=0;
	}

  inline typename T::VertexType *       & V( const int j ) 	     { assert(j>=0 && j<4); return v[j]; }
  inline typename T::VertexType * const & V( const int j ) const { assert(j>=0 && j<4); return v[j]; }
	inline typename T::VertexType * const  cV( const int j ) const { assert(j>=0 && j<4);	return v[j]; }

	// Shortcut per accedere ai punti delle facce
	inline       typename T::CoordType & P( const int j ) 	    {	assert(j>=0 && j<4);		return v[j]->P();	}
	inline const typename T::CoordType & P( const int j ) const	{	assert(j>=0 && j<4);		return v[j]->cP(); }
	inline const typename T::CoordType &cP( const int j ) const	{	assert(j>=0 && j<4);		return v[j]->cP(); }

	/** Return the pointer to the ((j+1)%3)-th vertex of the face.
		@param j Index of the face vertex.
	 */
	inline       typename T::VertexType *       &  V0( const int j )       { return V(j);}
	inline       typename T::VertexType *       &  V1( const int j )       { return V((j+1)%4);}
	inline       typename T::VertexType *       &  V2( const int j )       { return V((j+2)%4);}
	inline const typename T::VertexType * const &  V0( const int j ) const { return V(j);}
	inline const typename T::VertexType * const &  V1( const int j ) const { return V((j+1)%4);}
	inline const typename T::VertexType * const &  V2( const int j ) const { return V((j+2)%4);}
	inline const typename T::VertexType * const & cV0( const int j ) const { return cV(j);}
	inline const typename T::VertexType * const & cV1( const int j ) const { return cV((j+1)%4);}
	inline const typename T::VertexType * const & cV2( const int j ) const { return cV((j+2)%4);}

	/// Shortcut to get vertex values
	inline       typename T::CoordType &  P0( const int j )       { return V(j)->P();}
	inline       typename T::CoordType &  P1( const int j )       { return V((j+1)%4)->P();}
	inline       typename T::CoordType &  P2( const int j )       { return V((j+2)%4)->P();}
	inline const typename T::CoordType &  P0( const int j ) const { return V(j)->P();}
	inline const typename T::CoordType &  P1( const int j ) const { return V((j+1)%4)->P();}
	inline const typename T::CoordType &  P2( const int j ) const { return V((j+2)%4)->P();}
	inline const typename T::CoordType & cP0( const int j ) const { return cV(j)->P();}
	inline const typename T::CoordType & cP1( const int j ) const { return cV((j+1)%4)->P();}
	inline const typename T::CoordType & cP2( const int j ) const { return cV((j+2)%4)->P();}

  static bool HasVertexRef()   { return true; }
	static void Name(std::vector<std::string> & name){name.push_back(std::string("VertexRef"));T::Name(name);}


  private:
  typename T::VertexType *v[4];
};


/*------------------------- FACE NORMAL -----------------------------------------*/
template <class A, class T> class EmptyFaceNormal: public T {
public:
	typedef ::vcg::Point3<A> NormalType;
	/// Return the vector of Flags(), senza effettuare controlli sui bit
	NormalType N(const int & ){ static int dummynormal(0); return dummynormal; }
  const NormalType cN(const int & ) const { return 0; }
  static bool HasFaceNormal()   { return false; }
  static bool HasFaceNormalOcc()   { return false; }
	static void Name(std::vector<std::string> & name){T::Name(name);}

};

template <class A, class T> class FaceNormal:  public T {
public:
	typedef ::vcg::Point3<A> NormalType;

	NormalType N(const int & i){  assert((i>=0)&&(i < 4)); return _facenormals[i]; }
  const NormalType cN(const int & i) const { assert((i>=0)&&(i < 4)); return _facenormals[i]; }
  static bool HasFaceNormals()   { return true; }
  static bool HasFaceNormalOcc()   { return false; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("FaceNormal"));T::Name(name);}
	
private:
  NormalType  _facenormals[4];    
};

template <class T> class FaceNormal3f: public FaceNormal<float,T>{
public:static void Name(std::vector<std::string> & name){name.push_back(std::string("FaceNormal3f"));T::Name(name);} };

template <class T> class FaceNormal3d: public FaceNormal<double,T>{
public:static void Name(std::vector<std::string> & name){name.push_back(std::string("FaceNormal3d"));T::Name(name);} };

/*------------------------- FLAGS -----------------------------------------*/ 
template <class T> class EmptyBitFlags: public T {
public:
	/// Return the vector of Flags(), senza effettuare controlli sui bit
  int &Flags() { static int dummyflags(0); return dummyflags; }
  const int Flags() const { return 0; }
  static bool HasFlags()   { return false; }
  static bool HasFlagsOcc()   { return false; }
	static void Name(std::vector<std::string> & name){T::Name(name);}

};

template <class T> class BitFlags:  public T {
public:
  BitFlags(){_flags=0;}
   int &Flags() {return _flags; }
   const int Flags() const {return _flags; }
  static bool HasFlags()   { return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("BitFlags"));T::Name(name);}


private:
  int  _flags;    
};
/*-------------------------- INCREMENTAL MARK  ----------------------------------------*/ 

template <class T> class EmptyMark: public T {
public:
	typedef int MarkType;
  static bool HasMark()   { return false; }
  static bool HasMarkOcc()   { return false; }
  inline void InitIMark()    {  }
  inline int & IMark()       { assert(0); static int tmp=-1; return tmp;}
  inline const int IMark() const {return 0;}
  static void Name(std::vector<std::string> & name){T::Name(name);}

};
template <class T> class Mark: public T {
public:
  static bool HasMark()      { return true; }
  static bool HasMarkOcc()   { return true; }
  inline void InitIMark()    { _imark = 0; }
  inline int & IMark()       { return _imark;}
  inline const int & IMark() const {return _imark;}
  static void Name(std::vector<std::string> & name){name.push_back(std::string("Mark"));T::Name(name);}
    
 private:
	int _imark;
};


/*----------------------------- VTADJ ------------------------------*/ 

template <class T> class EmptyAdj: public T {
public:
	typedef int VFAdjType;
	typename T::TetraPointer & VTp( const int ) { static typename T::TetraPointer tp=0; return tp; }
	typename T::TetraPointer const cVTp( const int ) const { static typename T::TetraPointer const tp=0; return tp; }
	typename T::TetraPointer & TTp( const int ) { static typename T::TetraPointer tp=0; return tp; }
	typename T::TetraPointer const cTTp( const int ) const { static typename T::TetraPointer const tp=0; return tp; }
	char & VTi( const int j ) { static char z=0; return z; }
	char & TTi( const int j ) { static char z=0; return z; }
	static bool HasVTAdjacency() { return false; }
	static bool HasTTAdjacency() { return false; }
	static bool HasTTAdjacencyOcc() { return false; }
	static bool HasVTAdjacencyOcc() { return false; }
	static void Name( std::vector< std::string > & name ){ T::Name(name); }
};

template <class T> class VTAdj: public T {
public:
	VTAdj() { _vtp[0]=0; _vtp[1]=0; _vtp[2]=0; _vtp[3]=0; }
	typename T::TetraPointer & VTp( const int j ) { assert( j >= 0 && j < 4 ); return _vtp[j]; }
	typename T::TetraPointer const VTp( const int j ) const { assert( j >= 0 && j < 4 ); return _vtp[j]; }
	typename T::TetraPointer const cVTp( const int j ) const { assert( j >= 0 && j < 4 ); return _vtp[j]; }
	char & VTi( const int j ) { return _vti[j]; }
	const char & cVTi( const int j ) const { return _vti[j]; }
	static bool HasVTAdjacency() { return true; }
	static bool HasVTAdjacencyOcc() { return false; }
	static void Name( std::vector< std::string > & name ) { name.push_back( std::string("VTAdj") ); T::Name(name); }

private:
	typename T::TetraPointer _vtp[4];
	char _vti[4];
};

/*----------------------------- TTADJ ------------------------------*/ 

template <class T> class TTAdj: public T {
public:
	TTAdj(){
		_ttp[0]=0;
		_ttp[1]=0;
		_ttp[2]=0;
		_ttp[3]=0;
	}
  typename T::TetraPointer       &TTp(const int j)        { assert(j>=0 && j<4);  return _ttp[j]; }
  typename T::TetraPointer const  TTp(const int j) const  { assert(j>=0 && j<4);  return _ttp[j]; }
  typename T::TetraPointer const cTTp(const int j) const  { assert(j>=0 && j<4);  return _ttp[j]; }
  char        &TTi(const int j)       { return _tti[j]; }
  const char &cTTi(const int j) const { return _tti[j]; }

  typename T::TetraPointer        &TTp1( const int j )       { return TTp((j+1)%4);}
	typename T::TetraPointer        &TTp2( const int j )       { return TTp((j+2)%4);}
	typename T::TetraPointer  const  TTp1( const int j ) const { return TTp((j+1)%4);}
	typename T::TetraPointer  const  TTp2( const int j ) const { return TTp((j+2)%4);}

	bool IsBorderF(const int & i)  const { assert( (i>=0) && (i < 4)); { return TTp(i) == this;}}

  static bool HasTTAdjacency()      {   return true; }
  static bool HasTTAdjacencyOcc()   {   return false; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("TTAdj"));T::Name(name);}

private:
  typename T::TetraPointer _ttp[4] ;    
  char _tti[4] ;    
};

  } // end namespace vert
}// end namespace vcg
#endif
