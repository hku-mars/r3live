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
#include <vcg/complex/complex.h>

#ifndef __VCG_HEDGE_COMPONENT
#define __VCG_HEDGE_COMPONENT

namespace vcg {
	namespace hedge {
/*
Some naming Rules
All the Components that can be added to a vertex should be defined in the namespace hedge:

*/
//
///*-------------------------- VERTEX ----------------------------------------*/
//template <class T> class EmptyVertexRef: public T {
//public:
// // typedef typename T::VertexType VertexType;
// // typedef typename T::CoordType CoordType;
//  inline typename T::VertexType *       & V( const int j ) 	    {	assert(0);		static typename T::VertexType *vp=0; return vp; }
//  inline typename T::VertexType * const & V( const int j ) const {	assert(0);		static typename T::VertexType *vp=0; return vp; }
//        inline typename T::VertexType *  cV( const int j ) const {	assert(0);		static typename T::VertexType *vp=0; return vp;	}
//	inline       typename T::CoordType & P( const int j ) 	    {	assert(0);		static typename T::CoordType coord(0, 0, 0); return coord;	}
//	inline const typename T::CoordType & P( const int j ) const {	assert(0);		static typename T::CoordType coord(0, 0, 0); return coord;	}
//	inline const typename T::CoordType &cP( const int j ) const	{	assert(0);		static typename T::CoordType coord(0, 0, 0); return coord;	}
//	template <class LeftF>
//	void ImportData(const LeftF & leftF) {T::ImportData(leftF);}
//  static bool HasVertexRef()   { return false; }
//	static void Name(std::vector<std::string> & name){T::Name(name);}
//
//};
//template <class T> class VertexRef: public T {
//public:
//	VertexRef(){
//		v[0]=0;
//		v[1]=0;
//	}
//
//  inline typename T::VertexType *       & V( const int j ) 	     { assert(j>=0 && j<2); return v[j]; }
//  inline typename T::VertexType * const & V( const int j ) const { assert(j>=0 && j<2); return v[j]; }
//        inline typename T::VertexType *  cV( const int j ) const { assert(j>=0 && j<2);	return v[j]; }
//
//	// Shortcut per accedere ai punti delle facce
//	inline       typename T::CoordType & P( const int j ) 	    {	assert(j>=0 && j<2);		return v[j]->P();	}
//	inline const typename T::CoordType &cP( const int j ) const	{	assert(j>=0 && j<2);		return v[j]->cP(); }
//
//	/** Return the pointer to the ((j+1)%3)-th vertex of the face.
//		@param j Index of the face vertex.
//	 */
//	inline       typename T::VertexType *       &  V0( const int j )       { return V(j);}
//	inline       typename T::VertexType *       &  V1( const int j )       { return V((j+1)%2);}
//	inline const typename T::VertexType * const &  V0( const int j ) const { return V(j);}
//	inline const typename T::VertexType * const &  V1( const int j ) const { return V((j+1)%2);}
//	inline const typename T::VertexType * const & cV0( const int j ) const { return cV(j);}
//	inline const typename T::VertexType * const & cV1( const int j ) const { return cV((j+1)%2);}
//
//	/// Shortcut per accedere ai punti delle facce
//	inline       typename T::CoordType &  P0( const int j )       { return V(j)->P();}
//	inline       typename T::CoordType &  P1( const int j )       { return V((j+1)%2)->P();}
//	inline const typename T::CoordType &  P0( const int j ) const { return V(j)->P();}
//	inline const typename T::CoordType &  P1( const int j ) const { return V((j+1)%2)->P();}
//	inline const typename T::CoordType & cP0( const int j ) const { return cV(j)->P();}
//	inline const typename T::CoordType & cP1( const int j ) const { return cV((j+1)%2)->P();}
//
//	template <class LeftF>
//	void ImportData(const LeftF & leftF){ V(0) = NULL; V(1) = NULL; V(2) = NULL; T::ImportData(leftF);}
//
//  static bool HasVertexRef()   { return true; }
//	static void Name(std::vector<std::string> & name){name.push_back(std::string("VertexRef"));T::Name(name);}
//
//
//  private:
//  typename T::VertexType *v[2];
//};



/*-------------------------- INCREMENTAL MARK  ----------------------------------------*/ 

template <class T> class EmptyMark: public T {
public:
  static bool HasMark()   { return false; }
  static bool HasMarkOcc()   { return false; }
  inline void InitIMark()    {  }
  inline int & IMark()       { assert(0); static int tmp=-1; return tmp;}
  inline int IMark() const {return 0;}
	template < class LeftV>
	void ImportData(const LeftV  & left ) { T::ImportData( left); }
	static void Name(std::vector<std::string> & name){T::Name(name);}

};
template <class T> class Mark: public T {
public:
  static bool HasMark()      { return true; }
  static bool HasMarkOcc()   { return true; }
  inline void InitIMark()    { _imark = 0; }
  inline int & IMark()       { return _imark;}
  inline const int & IMark() const {return _imark;}
	template < class LeftV>
	void ImportData(const LeftV  & left ) { IMark() = left.IMark(); T::ImportData( left); }
	static void Name(std::vector<std::string> & name){name.push_back(std::string("Mark"));T::Name(name);}
    
 private:
	int _imark;
};

/*------------------------- FLAGS -----------------------------------------*/ 
template <class T> class EmptyBitFlags: public T {
public:
	typedef int FlagType;
  /// Return the vector of Flags(), senza effettuare controlli sui bit
  int &Flags() { static int dummyflags(0);  assert(0); return dummyflags; }
  int Flags() const { return 0; }
	template < class LeftV>
	void ImportData(const LeftV  & left ) { T::ImportData( left); }
  static bool HasFlags()   { return false; }
	static void Name(std::vector<std::string> & name){T::Name(name);}

};

template <class T> class BitFlags:  public T {
public:
	BitFlags(){_flags=0;}
  typedef int FlagType;
  int &Flags() {return _flags; }
  int Flags() const {return _flags; }
	template < class LeftV>
	void ImportData(const LeftV  & left ) { Flags() = left.Flags(); T::ImportData( left); }
  static bool HasFlags()   { return true; }
	static void Name(std::vector<std::string> & name){name.push_back(std::string("BitFlags"));T::Name(name);}

private:
  int  _flags;    
};

/*----------------------------- HVADJ ------------------------------*/
template <class T> class EmptyHVAdj: public T {
public:
	typename T::VertexPointer &HVp() { static typename T::VertexPointer ep=0;  assert(0); return ep; }
	const typename T::VertexPointer cHVp() const { static typename T::VertexPointer ep=0;  assert(0); return ep; }
        int &HVi(){static int z=0; return z;}
	template < class LeftV>
		void ImportData(const LeftV  & left ) { T::ImportData( left); }
	static bool HasHVAdjacency()   {   return false; }
	static bool HasHVAdjacencyOcc()   {   return false; }
	static void Name(std::vector<std::string> & name){ T::Name(name);}
};

template <class T> class HVAdj: public T {
public:
	HVAdj(){_vp =0;}
	typename T::VertexPointer			&	HVp() {return _vp ; }
	const typename T::VertexPointer cHVp() const {return _vp ; }
	template < class LeftV>
	void ImportData(const LeftV  & left ) { this->V() = NULL; T::ImportData( left); }
	static bool HasHVAdjacency()   {   return true; }
	static bool HasHVAdjacencyOcc()   {   return true; }
	static void Name(std::vector<std::string> & name){name.push_back(std::string("HVAdj"));T::Name(name);}
	
private:
   typename T::VertexPointer	 _vp ;    
};

/*----------------------------- HEADJ ------------------------------*/
template <class T> class EmptyHEAdj: public T {
public:
        typename T::EdgePointer &HEp() { static typename T::EdgePointer ep=0;  assert(0); return ep; }
				const typename T::EdgePointer cHEp() const { static typename T::EdgePointer ep=0;  assert(0); return ep; }
        template < class LeftV>
								void ImportData(const LeftV  & left ) { T::ImportData( left); }
        static bool HasHEAdjacency()   {   return false; }
        static bool HasHEAdjacencyOcc()   {   return false; }
        static void Name(std::vector<std::string> & name){ T::Name(name);}
};

template <class T> class HEAdj: public T {
public:
        HEAdj(){_ep =0;}
        typename T::EdgePointer &HEp() {return _ep ; }
        const typename T::EdgePointer cHEp() const {return _ep ; }
        template < class LeftV>
				void ImportData(const LeftV  & left ) { this->V() = NULL; T::ImportData( left); }
        static bool HasHEAdjacency()   {   return true; }
        static bool HasHEAdjacencyOcc()   {   return true; }
        static void Name(std::vector<std::string> & name){name.push_back(std::string("HEAdj"));T::Name(name);}

private:
   typename T::EdgePointer	 _ep ;
};


/*----------------------------- HHADJ ------------------------------*/
template <class T> class EmptyHHAdj: public T {
public:
	typename T::HEdgePointer &HHp(const int &  ) { static typename T::EdgePointer ep=0;  assert(0); return ep; }
	typename T::HEdgePointer cHHp(const int & ) { static typename T::EdgePointer ep=0;  assert(0); return ep; }
        int &HHi(){static int z=0; return z;}
	template < class LeftV>
		void ImportData(const LeftV  & left ) { T::ImportData( left); }
	static bool HasHHAdjacency()   {   return false; }
	static bool HasHHAdjacencyOcc()   {   return false; }
	static void Name(std::vector<std::string> & name){ T::Name(name);}
};

template <class T> class HHAdj: public T {
public:
	HHAdj(){_ep=0;}
	typename T::EdgePointer &HHp(const int & i) {return _ep[i]; }
	typename T::EdgePointer cHHp(const int & i) {return _ep[i]; }
	int &HHi(const int & i) {return _zp[i]; }
	template < class LeftV>
	void ImportData(const LeftV  & left ) { HHp() = NULL; T::ImportData( left); }
	static bool HasHHAdjacency()   {   return true; }
	static bool HasHHAdjacencyOcc()   {   return true; }
	static void Name(std::vector<std::string> & name){name.push_back(std::string("HHAdj"));T::Name(name);}

private:
	typename T::HEdgePointer _ep[2] ;
  int _zp[2] ;    
};




/*----------------------------- HENextADJ ------------------------------*/ 
template <class T> class EmptyHNextAdj: public T {
public:
	typename T::HEdgePointer &HNp( ) { static typename T::HEdgePointer ep=0;  assert(0); return ep; }
	typename T::HEdgePointer const cHNp( ) const  { static typename T::HEdgePointer ep=0;  assert(0); return ep; }
	template < class LeftV>
		void ImportData(const LeftV  & left ) { T::ImportData( left); }
	static bool HasHNextAdjacency()   {   return false; }
	static bool HasHNextAdjacencyOcc()   {   return false; }
	static void Name(std::vector<std::string> & name){ T::Name(name);}
};

template <class T> class HNextAdj: public T {
public:
	HNextAdj(){_nep=0;}
	typename T::HEdgePointer &HNp() {return _nep; }
	typename T::HEdgePointer const cHNp() const {return _nep; }
	template < class LeftV>
	void ImportData(const LeftV  & left ) { this->EEp() = NULL; T::ImportData( left); }
	static bool HasHNextAdjacency()   {   return true; }
	static void Name(std::vector<std::string> & name){name.push_back(std::string("HNextAdj"));T::Name(name);}
	
private:
	typename T::HEdgePointer _nep ;
};

/*----------------------------- HEOppADJ ------------------------------*/ 
template <class T> class EmptyHOppAdj: public T {
public:
	typename T::HEdgePointer &HOp() { static typename T::HEdgePointer ep=0;  assert(0); return ep; }
	typename T::HEdgePointer const cHOp() const { static typename T::HEdgePointer ep=0;  assert(0); return ep; }
  int &EEi(){static int z=0; return z;}
	template < class LeftV>
		void ImportData(const LeftV  & left ) { T::ImportData( left); }
	static bool HasHOppAdjacency()   {   return false; }
	static bool HasHOpptAdjacencyOcc()   {   return false; }
	static void Name(std::vector<std::string> & name){ T::Name(name);}
};

template <class T> class HOppAdj: public T {
public:
	HOppAdj(){_oep=0;}
	typename T::HEdgePointer &HOp() {return _oep; }
	typename T::HEdgePointer cHOp() {return _oep; }
	template < class LeftV>
	void ImportData(const LeftV  & left ) { this->HOp() = NULL; T::ImportData( left); }
	static bool HasHOppAdjacency()   {   return true; }
	static void Name(std::vector<std::string> & name){name.push_back(std::string("HOppAdj"));T::Name(name);}
	
private:
	typename T::HEdgePointer _oep ;
 
};
/*----------------------------- HPrevADJ ------------------------------*/
template <class T> class EmptyHPrevAdj: public T {
public:
	typename T::HEdgePointer &HPp() { static typename T::HEdgePointer ep=0;  assert(0); return ep; }
	typename T::HEdgePointer const cHPp() const { static typename T::HEdgePointer ep=0;  assert(0); return ep; }
  int &EEi(){static int z=0; return z;}
	template < class LeftV>
		void ImportData(const LeftV  & left ) { T::ImportData( left); }
	static bool HasHPrevAdjacency()   {   return false; }
	static bool HasHPrevAdjacencyOcc()   {   return false; }
  static void Name(std::vector<std::string> & name){ T::Name(name);}
};

template <class T> class HPrevAdj: public T {
public:
	HPrevAdj(){_pep=0;}
        typename T::HEdgePointer &HPp() {return _pep; }
        typename T::HEdgePointer cHPp() {return _pep; }
  int &EEi(const int & i) {return this->_nei[i]; }
	template < class LeftV>
	void ImportData(const LeftV  & left ) { this->EEp() = NULL; T::ImportData( left); }
	static bool HasHPrevAdjacency()   {   return true; }
	static void Name(std::vector<std::string> & name){name.push_back(std::string("HPrevAdj"));T::Name(name);}
	
private:
  typename T::HEdgePointer _pep ;
};
/*----------------------------- HFADJ ------------------------------*/

template <class T> class EmptyHFAdj: public T {
public:
	typename T::FacePointer &HFp() { static typename T::FacePointer fp=0;  assert(0); return fp; }
	typename T::FacePointer const cHFp() const  { static typename T::FacePointer fp=0;  assert(0); return fp; }
  int &EFi(){static int z=0; return z;}
	template < class LeftV>
	void ImportData(const LeftV  & left ) { T::ImportData( left); }
	static bool HasHFAdjacency()   {   return false; }
	static bool HasHFAdjacencyOcc()   {   return false; }
	static void Name(std::vector<std::string> & name){ T::Name(name);}
};

template <class T> class HFAdj: public T {
public:
	HFAdj(){_fp=0;}
	typename T::FacePointer &HFp() {return _fp; }
	typename T::FacePointer cHFp() {return _fp; }
  int &EFi() {return _zp; }
	template < class LeftV>
	void ImportData(const LeftV  & left ) { this->EFp() = NULL; T::ImportData( left); }
	static bool HasHFAdjacency()   {   return true; }
	static bool HasHFAdjacencyOcc()   {   return true; }
	static void Name(std::vector<std::string> & name){name.push_back(std::string("HFAdj"));T::Name(name);}

private:
  typename T::FacePointer _fp ;    
  int _zp ;    
};


/*----------------------------- HFADJ ------------------------------*/
/**
 HEdgeData keep all the data for the half edge
*/
template <class T> 
class EmptyHEdgeData : public	EmptyHFAdj<		// pointer to the face
							EmptyHOppAdj <		// pointer to the opposite half edge
							EmptyHNextAdj <	// pointer to the next half edge along the face
							EmptyHVAdj <		// pointer to the vertex
                                                        EmptyHEAdj <		// pointer to the edge
							EmptyHPrevAdj<
                                                        T > > > > > > {};


template <class T> 
class HEdgeData : public	HFAdj<			// pointer to the face
							HOppAdj <		// pointer to the opposite half edge
							HNextAdj <		// pointer to the next half edge along the face
							HVAdj <		// pointer to the vertex
                                                        HEAdj <         // pointer to the edge
                                                        T > > > > > {

    public:
	// functions to make the half edge user confortable
        typename T::VertexPointer & Vertex()                    { return this->HVp();}
        const typename T::VertexPointer &  cVertex()    const   { return this->cHVp();}
        typename T::HEdgePointer Opposite()                     { return this->HOp();}
        const typename T::HEdgePointer & cOpposite()    const   { return this->cHOp();}
        typename T::HEdgePointer & Next()                       { return this->HNp();}
        const typename T::HEdgePointer &  cNext()       const   { return this->HNp();}

};

  } // end namespace edge
}// end namespace vcg
#endif
