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
#ifndef __VCG_EDGE_PLUS_COMPONENT
#define __VCG_EDGE_PLUS_COMPONENT

namespace vcg {
  namespace edge {

  /** \addtogroup EdgeComponentGroup
    @{
  */

/*
Some naming Rules
All the Components that can be added to a vertex should be defined in the namespace edge:

*/
  /*------------------------- EMPTY CORE COMPONENTS -----------------------------------------*/

template <class T> class EmptyCore: public T
  {
public:
	inline       typename T::VertexType *       &  V( const int j )       { (void)j; assert(0);  static typename T::VertexType *vp=0;         return vp;    }
	inline       typename T::VertexType * const &  V( const int j ) const { (void)j; assert(0);  static typename T::VertexType *vp=0;         return vp;    }
	inline       typename T::VertexType *         cV( const int j ) const { (void)j; assert(0);  static typename T::VertexType *vp=0;         return vp;    }
	inline       typename T::CoordType &           P( const int j )       { (void)j; assert(0);  static typename T::CoordType coord(0, 0, 0); return coord; }
	inline const typename T::CoordType &           P( const int j ) const { (void)j; assert(0);  static typename T::CoordType coord(0, 0, 0); return coord; }
	inline const typename T::CoordType &          cP( const int j ) const { (void)j; assert(0);  static typename T::CoordType coord(0, 0, 0); return coord; }
	static bool HasEVAdjacency()   { return false; }
	static bool HasVertexRef()     { return false; }

	typedef vcg::Color4b ColorType;
	ColorType &C() { static ColorType dumcolor(vcg::Color4b::White); assert(0); return dumcolor; }
	ColorType cC() const { static ColorType dumcolor(vcg::Color4b::White);  assert(0); return dumcolor; }
	static bool HasColor()   { return false; }

    typedef float QualityType;
    QualityType &Q() { static QualityType dummyQuality(0);  assert(0); return dummyQuality; }
    QualityType cQ() const { static QualityType dummyQuality(0);  assert(0); return dummyQuality; }
    static bool HasQuality()   { return false; }

    typedef int  MarkType;
    inline void InitIMark()    {  }
    inline int cIMark() const { assert(0); static int tmp=-1; return tmp;}
    inline int &IMark()       { assert(0); static int tmp=-1; return tmp;}
    static bool HasMark()   { return false; }

    typedef int FlagType;
    int &Flags() { static int dummyflags(0);  assert(0); return dummyflags; }
    int Flags() const { return 0; }
    static bool HasFlags()   { return false; }

    typename T::EdgePointer &VEp(const int &  ) { static typename T::EdgePointer ep=0;  assert(0); return ep; }
    typename T::EdgePointer cVEp(const int & ) const { static typename T::EdgePointer ep=0;  assert(0); return ep; }
    int &VEi(const int &){static int z=0; assert(0); return z;}
    int cVEi(const int &) const {static int z=0; assert(0); return z;}
    static bool HasVEAdjacency()   {   return false; }

    typename T::EdgePointer &EEp(const int &  ) { static typename T::EdgePointer ep=0;  assert(0); return ep; }
    typename T::EdgePointer cEEp(const int & ) const { static typename T::EdgePointer ep=0;  assert(0); return ep; }
    int &EEi(const int &){static int z=0; assert(0); return z;}
    int cEEi(const int &) const {static int z=0; assert(0); return z;}
    static bool HasEEAdjacency()   {   return false; }

    typename T::HEdgePointer &EHp(  ) { static typename T::HEdgePointer hp=0;  assert(0); return hp; }
    typename T::HEdgePointer cEHp(  ) const { static typename T::HEdgePointer hp=0;  assert(0); return hp; }
    static bool HasEHAdjacency()   {   return false; }

    typename T::FacePointer &EFp() { static typename T::FacePointer fp=0;  assert(0); return fp; }
    typename T::FacePointer cEFp() const  { static typename T::FacePointer fp=0;  assert(0); return fp; }
    int &EFi()   {static int z=0; return z;}
    int &cEFi() const {static int z=0; return z;}
    static bool HasEFAdjacency()   {   return false; }

    template <class LeftF>
    void ImportData(const LeftF & leftF) {T::ImportData(leftF);}
    static void Name(std::vector<std::string> & name){T::Name(name);}
  };

  /*-------------------------- VertexRef ----------------------------------------*/
  /*! \brief The references to the two vertexes of a edge
   *
   * Stored as pointers to the VertexType
   */

template <class T> class VertexRef: public T {
public:
	VertexRef(){
		v[0]=0;
		v[1]=0;
	}

  inline typename T::VertexType *       & V( const int j ) 	     { assert(j>=0 && j<2); return v[j]; }
  inline typename T::VertexType * const & V( const int j ) const { assert(j>=0 && j<2); return v[j]; }
		inline typename T::VertexType *  cV( const int j ) const { assert(j>=0 && j<2);	return v[j]; }

	// Shortcut per accedere ai punti delle facce
	inline       typename T::CoordType & P( const int j ) 	    {	assert(j>=0 && j<2);		return v[j]->P();	}
	inline const typename T::CoordType &cP( const int j ) const	{	assert(j>=0 && j<2);		return v[j]->cP(); }

	/** Return the pointer to the ((j+1)%3)-th vertex of the face.
		@param j Index of the face vertex.
	 */
	inline       typename T::VertexType *       &  V0( const int j )       { return V(j);}
	inline       typename T::VertexType *       &  V1( const int j )       { return V((j+1)%2);}
	inline const typename T::VertexType * const &  V0( const int j ) const { return V(j);}
	inline const typename T::VertexType * const &  V1( const int j ) const { return V((j+1)%2);}
	inline const typename T::VertexType * const & cV0( const int j ) const { return cV(j);}
	inline const typename T::VertexType * const & cV1( const int j ) const { return cV((j+1)%2);}

	/// Shortcut per accedere ai punti delle facce
	inline       typename T::CoordType &  P0( const int j )       { return V(j)->P();}
	inline       typename T::CoordType &  P1( const int j )       { return V((j+1)%2)->P();}
	inline const typename T::CoordType &  P0( const int j ) const { return V(j)->P();}
	inline const typename T::CoordType &  P1( const int j ) const { return V((j+1)%2)->P();}
	inline const typename T::CoordType & cP0( const int j ) const { return cV(j)->P();}
	inline const typename T::CoordType & cP1( const int j ) const { return cV((j+1)%2)->P();}

	template <class LeftF>
	void ImportData(const LeftF & leftF){ T::ImportData(leftF);}

  static bool HasEVAdjacency()   {   return true; }
  static bool HasVertexRef()   { return true; }
	static void Name(std::vector<std::string> & name){name.push_back(std::string("VertexRef"));T::Name(name);}


  private:
  typename T::VertexType *v[2];
};

template <class T> class EVAdj : public VertexRef<T>{};


/*-------------------------- INCREMENTAL MARK  ----------------------------------------*/

/*! \brief \em Component: Per edge \b Incremental \b Mark
 *
 * An int that allows to efficently un-mark the whole mesh. \sa UnmarkAll
 */
template <class T> class Mark: public T {
public:
    Mark():_imark(0){}
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
  /*! \brief \em Component: Per edge \b Flags
   *
   * This component stores a 32 bit array of bit flags. These bit flags are used for keeping track of selection, deletion, visiting etc. \sa \ref flags for more details on common uses of flags.
   */
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

/*-------------------------- Color  ----------------------------------*/
  /*! \brief \em Component: Per edge \b Color
   *
   * Usually most of the library expects a color stored as 4 unsigned chars (so the component you use is a \c vertex::Color4b)
   * but you can also use float for the color components.
   */
template <class A, class T> class Color: public T {
public:
  Color():_color(vcg::Color4b::White) {}
  typedef A ColorType;
  ColorType &C() { return _color; }
  const ColorType &C() const { return _color; }
  const ColorType &cC() const { return _color; }
	template < class LeftV>
	void ImportData(const LeftV  & left ) { C() = left.cC(); T::ImportData( left); }
  static bool HasColor()   { return true; }
	static void Name(std::vector<std::string> & name){name.push_back(std::string("Color"));T::Name(name);}

private:
  ColorType _color;
};

template <class TT> class Color4b: public edge::Color<vcg::Color4b, TT> {
	public: static void Name(std::vector<std::string> & name){name.push_back(std::string("Color4b"));TT::Name(name);}
};

/*-------------------------- Quality  ----------------------------------*/
  /*! \brief \em Component: Per edge \b quality
   *
   * The Quality Component is a generic place for storing a float. The term 'quality' is a bit misleading and it is due to its original storic meaning. You should intend it as a general purpose container.
   * \sa vcg::tri::UpdateColor for methods transforming quality into colors
   * \sa vcg::tri::UpdateQuality for methods to manage it
   */
template <class A, class TT> class Quality: public TT {
public:
  typedef A QualityType;
  QualityType &Q() { return _quality; }
  const QualityType & cQ() const {return _quality; }
	template < class LeftV>
	void ImportData(const LeftV  & left ) { Q() = left.cQ(); TT::ImportData( left); }
  static bool HasQuality()   { return true; }
	static void Name(std::vector<std::string> & name){name.push_back(std::string("Quality"));TT::Name(name);}

private:
  QualityType _quality;
};

template <class TT> class Qualitys: public Quality<short, TT> {
public: static void Name(std::vector<std::string> & name){name.push_back(std::string("Qualitys"));TT::Name(name);}
};
template <class TT> class Qualityf: public Quality<float, TT> {
public: static void Name(std::vector<std::string> & name){name.push_back(std::string("Qualityf"));TT::Name(name);}
};
template <class TT> class Qualityd: public Quality<double, TT> {
public: static void Name(std::vector<std::string> & name){name.push_back(std::string("Qualityd"));TT::Name(name);}
};

/*----------------------------- VEADJ ------------------------------*/
  /*! \brief \em Component: Per vertex \b Vertex-Edge adjacency relation companion component
  This component implement one element of the list of edges incident on a vertex.
  You must use this component only toghether with the corresponding \ref vcg::vertex::VEAdj component in the vertex type

  \sa vcg::tri::UpdateTopology for functions that compute this relation
  \sa iterators
  */
  template <class T> class VEAdj: public T {
  public:
	VEAdj(){_ep[0]=0;_ep[1]=0;_zp[0]=-1;_zp[1]=-1;}
	typename T::EdgePointer &VEp(const int & i) {return _ep[i]; }
	typename T::EdgePointer cVEp(const int & i) const {return _ep[i]; }
	int &VEi(const int & i){ return _zp[i];}
	int cVEi(const int &i )const {return _zp[i];}

	template < class LeftV>
	void ImportData(const LeftV  & left ) {  T::ImportData( left); }
	static bool HasVEAdjacency()   {   return true; }
	static bool HasVEAdjacencyOcc()   {   return true; }
	static void Name(std::vector<std::string> & name){name.push_back(std::string("VEAdj"));T::Name(name);}

  private:
	typename T::EdgePointer _ep[2] ;
	int _zp[2] ;
  };

/*----------------------------- EEADJ ------------------------------*/
  /*! \brief \em Component: \b Edge-Edge adjacency relation
  This component implement store the pointer (and index) of the adjacent edges.
  If the vertex is 1-manifold (as in a classical polyline)
  it holds that:
  \code
   e->EEp(i)->EEp(e->EEi(i)) == e
  \endcode
  otherwise the edges are connected in a unordered chain (quite similar to how Face-Face adjacency relation is stored);

  \sa vcg::tri::UpdateTopology for functions that compute this relation
  \sa iterators
  */

template <class T> class EEAdj: public T {
public:
  EEAdj(){_ep[0]=0;_ep[1]=0;_zp[0]=-1;_zp[1]=-1;}
  typename T::EdgePointer &EEp(const int & i) {return _ep[i]; }
  typename T::EdgePointer cEEp(const int & i) const {return _ep[i]; }
  int &EEi(const int & i){ return _zp[i];}
  int cEEi(const int &i )const {return _zp[i];}

	template < class LeftV>
	void ImportData(const LeftV  & left ) {  T::ImportData( left); }
  static bool HasEEAdjacency()   {   return true; }
  static bool HasEEAdjacencyOcc()   {   return true; }
	static void Name(std::vector<std::string> & name){name.push_back(std::string("EEAdj"));T::Name(name);}

private:
  typename T::EdgePointer _ep[2] ;
  int _zp[2] ;
};

/*----------------------------- EHADJ ------------------------------*/
template <class T> class EHAdj: public T {
public:
  EHAdj(){_hp=0;}
  typename T::HEdgePointer &EHp( ) {return _hp ; }
	const typename T::HEdgePointer cEHp( ) const {return _hp ; }

	template < class LeftV>
	void ImportData(const LeftV  & left ) { T::ImportData( left); }
  static bool HasEHAdjacency()   {   return true; }
  static bool HasEHAdjacencyOcc()   {   return true; }
	static void Name(std::vector<std::string> & name){name.push_back(std::string("EHAdj"));T::Name(name);}

private:
  typename T::HEdgePointer _hp ;
};

/*----------------------------- EFADJ ------------------------------*/
  /*! \brief \em Component: \b Edge-Face adjacency relation
  This component implement store the pointer to a face sharing this edge.

  \sa vcg::tri::UpdateTopology for functions that compute this relation
  \sa iterators
  */

template <class T> class EFAdj: public T {
public:
  EFAdj(){_fp=0;}
  typename T::FacePointer &EFp() {return _fp; }
  typename T::FacePointer cEFp() const {return _fp; }
  int &EFi()   {static int z=0; return z;}
  int cEFi() const  {return _zp; }
  template < class LeftV>
  void ImportData(const LeftV  & left ) {  T::ImportData( left); }
  static bool HasEFAdjacency()   {   return true; }
  static bool HasEFAdjacencyOcc()   {   return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("EFAdj"));T::Name(name);}

private:
  typename T::FacePointer _fp ;
  int _zp ;
};

  /** @} */   // End Doxygen EdgeComponentGroup
  } // end namespace edge
}// end namespace vcg
#endif
