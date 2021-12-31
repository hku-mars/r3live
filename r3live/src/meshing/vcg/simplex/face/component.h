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
#ifndef __VCG_FACE_PLUS_COMPONENT
#define __VCG_FACE_PLUS_COMPONENT


namespace vcg {
namespace face {
/** \addtogroup FaceComponentGroup
  @{
*/
/*------------------------- EMPTY CORE COMPONENTS -----------------------------------------*/

template <class T> class EmptyCore: public T {
public:
  inline typename T::VertexType * &V( const int )       { assert(0);		static typename T::VertexType *vp=0; return vp; }
  inline typename T::VertexType * cV( const int ) const { assert(0);		static typename T::VertexType *vp=0; return vp;	}
  inline typename T::VertexType * &FVp( const int i )       {	return this->V(i); }
  inline typename T::VertexType * cFVp( const int i ) const {	return this->cV(i); }
  inline typename T::CoordType &P( const int ) 	     { assert(0);		static typename T::CoordType coord(0, 0, 0); return coord;	}
  inline typename T::CoordType cP( const int ) const { assert(0);		static typename T::CoordType coord(0, 0, 0); return coord;	}

  static bool HasVertexRef()   { return false; }
  static bool HasFVAdjacency()   { return false; }

  typedef typename T::VertexType::NormalType NormalType;
  NormalType &N() { static NormalType dummy_normal(0, 0, 0);  assert(0); return dummy_normal; }
  NormalType cN() const { static NormalType dummy_normal(0, 0, 0); return dummy_normal; }
  NormalType &WN(int) { static NormalType dummy_normal(0, 0, 0);  assert(0); return dummy_normal; }
  NormalType cWN(int) const { static NormalType dummy_normal(0, 0, 0); return dummy_normal; }


  typedef int WedgeTexCoordType;
  typedef vcg::TexCoord2<float,1> TexCoordType;
  TexCoordType &WT(const int) { static TexCoordType dummy_texture;  assert(0); return dummy_texture;}
  TexCoordType const &cWT(const int) const { static TexCoordType dummy_texture; return dummy_texture;}


  int &Flags() { static int dummyflags(0);  assert(0); return dummyflags; }
  int cFlags() const { return 0; }
  static bool HasFlags()   { return false; }

  inline void InitIMark()    {  }
  inline int &IMark()       { assert(0); static int tmp=-1; return tmp;}
  inline int cIMark() const { return 0;}

  typedef int MarkType;
  typedef float QualityType;
  typedef Point3f Quality3Type;
  typedef vcg::Color4b ColorType;

  ColorType &C()       { static ColorType dumcolor(vcg::Color4b::White);  assert(0); return dumcolor; }
  ColorType cC() const { static ColorType dumcolor(vcg::Color4b::White);  assert(0); return dumcolor; }
  ColorType &WC(const int)       { static ColorType dumcolor(vcg::Color4b::White);  assert(0); return dumcolor; }
  ColorType cWC(const int) const { static ColorType dumcolor(vcg::Color4b::White);  assert(0); return dumcolor; }
  QualityType &Q()       { static QualityType dummyQuality(0);  assert(0); return dummyQuality; }
  QualityType cQ() const { static QualityType dummyQuality(0);  assert(0); return dummyQuality; }
  Quality3Type &Q3()       { static Quality3Type dummyQuality3(0,0,0);  assert(0); return dummyQuality3; }
  Quality3Type cQ3() const { static Quality3Type dummyQuality3(0,0,0);  assert(0); return dummyQuality3; }

  static bool HasColor()   { return false; }
  static bool HasQuality()   { return false; }
  static bool HasQuality3()   { return false; }
  static bool HasMark()   { return false; }
  static bool HasNormal()    { return false; }

  static bool HasWedgeColor()   { return false; }
  static bool HasWedgeNormal()   { return false; }
  static bool HasWedgeTexCoord()   { return false; }

  // Interfaces for dynamic types
  inline bool IsColorEnabled( )        const { return T::FaceType::HasColor(); }
  inline bool IsCurvatureDirEnabled( ) const { return T::FaceType::HasCurvatureDir(); }
  inline bool IsMarkEnabled( )         const { return T::FaceType::HasMark(); }
  inline bool IsNormalEnabled( )       const { return T::FaceType::HasNormal(); }
  inline bool IsQualityEnabled( )      const { return T::FaceType::HasQuality(); }
  inline bool IsQuality3Enabled( )     const { return T::FaceType::HasQuality3(); }

  inline bool IsWedgeColorEnabled( )    const { return T::FaceType::HasWedgeColor(); }
  inline bool IsWedgeNormalEnabled( )   const { return T::FaceType::HasWedgeNormal(); }
  inline bool IsWedgeTexCoordEnabled( ) const { return T::FaceType::HasWedgeTexCoord(); }

  typedef int VFAdjType;
  typename T::FacePointer &VFp(int)       { static typename T::FacePointer fp=0; assert(0); return fp; }
  typename T::FacePointer cVFp(int) const { static typename T::FacePointer fp=0; assert(0); return fp; }
  typename T::FacePointer &FFp(int)       { static typename T::FacePointer fp=0; assert(0); return fp; }
  typename T::FacePointer cFFp(int) const { static typename T::FacePointer fp=0; assert(0); return fp; }
  typename T::EdgePointer &FEp(int)       { static typename T::EdgePointer fp=0; assert(0); return fp; }
  typename T::EdgePointer cFEp(int) const { static typename T::EdgePointer fp=0; assert(0); return fp; }
  typename T::HEdgePointer &FHp()       { static typename T::HEdgePointer fp=0; assert(0); return fp; }
  typename T::HEdgePointer cFHp() const { static typename T::HEdgePointer fp=0; assert(0); return fp; }
  char &VFi(int)       { static char z=0; assert(0); return z;}
  char &FFi(int)       { static char z=0; assert(0); return z;}
  char cVFi(int) const { static char z=0; assert(0); return z;}
  char cFFi(int) const { static char z=0; assert(0); return z;}
  bool IsVFInitialized(const int j) const {return  static_cast<const typename T::FaceType *>(this)->cVFi(j)!=-1;}
  void VFClear(int j) {
    if(IsVFInitialized(j)) {
      static_cast<typename T::FacePointer>(this)->VFp(j)=0;
      static_cast<typename T::FacePointer>(this)->VFi(j)=-1;
    }
  }
  static bool HasVFAdjacency()   {   return false; }
  static bool HasFFAdjacency()   {   return false; }
  static bool HasFEAdjacency()   {   return false; }
  static bool HasFHAdjacency()   {   return false; }

  typedef int CurvatureDirType;

  Point3f &PD1()       { static Point3f dummy(0,0,0); assert(0); return dummy;}
  Point3f &PD2()       { static Point3f dummy(0,0,0); assert(0); return dummy;}
  Point3f cPD1() const { static Point3f dummy(0,0,0); assert(0); return dummy;}
  Point3f cPD2() const { static Point3f dummy(0,0,0); assert(0); return dummy;}

  float &K1()      { static float dummy(0); assert(0); return dummy;}
  float &K2()      { static float dummy(0); assert(0); return dummy;}
  float cK1() const { static float dummy(0); assert(0); return dummy;}
  float cK2() const { static float dummy(0); assert(0); return dummy;}

  static bool HasCurvatureDir()   { return false; }


  static bool HasPolyInfo()   { return false; }

   template <class RightValueType>
  void ImportData(const RightValueType & rightF) {T::ImportData(rightF);}
  inline void Alloc(const int & ns) {T::Alloc(ns);}
  inline void Dealloc(){T::Dealloc();}
  static void Name(std::vector<std::string> & name){T::Name(name);}
  };

/*-------------------------- VertexRef ----------------------------------------*/
/*! \brief The references to the vertexes of a triangular face
 *
 * Stored as three pointers to the VertexType
 */


template <class T> class VertexRef: public T {
public:
    VertexRef(){
        v[0]=0;
        v[1]=0;
        v[2]=0;
    }

  typedef typename T::VertexType::CoordType CoordType;
  typedef typename T::VertexType::ScalarType ScalarType;

  inline typename T::VertexType * &V( const int j )       { assert(j>=0 && j<3); return v[j]; } /// \brief The pointer to the i-th vertex
  inline typename T::VertexType * cV( const int j ) const { assert(j>=0 && j<3);	return v[j]; }

  inline CoordType &P( const int j ) 	    {	assert(j>=0 && j<3);		return v[j]->P();	} /// \brief Shortcut: the position of the  i-th vertex (equivalent to \c V(i)->P() )
  inline CoordType cP( const int j ) const	{	assert(j>=0 && j<3);		return v[j]->cP(); }

  inline typename T::VertexType * & V0( const int j )       { return V(j);}        /** \brief Return the pointer to the j-th vertex of the face. */
  inline typename T::VertexType * & V1( const int j )       { return V((j+1)%3);}  /** \brief Return the pointer to the ((j+1)%3)-th vertex of the face. */
  inline typename T::VertexType * & V2( const int j )       { return V((j+2)%3);}  /** \brief Return the pointer to the ((j+2)%3)-th vertex of the face. */
  inline typename T::VertexType *  cV0( const int j ) const { return cV(j);}
  inline typename T::VertexType *  cV1( const int j ) const { return cV((j+1)%3);}
  inline typename T::VertexType *  cV2( const int j ) const { return cV((j+2)%3);}

  inline       CoordType &  P0( const int j )       { return V(j)->P();}
  inline       CoordType &  P1( const int j )       { return V((j+1)%3)->P();}
  inline       CoordType &  P2( const int j )       { return V((j+2)%3)->P();}
  inline const CoordType & cP0( const int j ) const { return cV(j)->P();}
  inline const CoordType & cP1( const int j ) const { return cV((j+1)%3)->P();}
  inline const CoordType & cP2( const int j ) const { return cV((j+2)%3)->P();}

  // Small comment about the fact that the pointers are zero filled.
  // The importLocal is meant for copyng stuff between very different meshes, so copying the pointers would be meaningless.
  // if you are using ImportData for copying internally simplex you have to set up all the pointers by hand.
  template <class RightValueType>
  void ImportData(const RightValueType & rightF){  T::ImportData(rightF);}
  inline void Alloc(const int & ns){T::Alloc(ns);}
  inline void Dealloc(){T::Dealloc();}

  static bool HasVertexRef()   { return true; }
  static bool HasFVAdjacency()   { return true; }

  static void Name(std::vector<std::string> & name){name.push_back(std::string("VertexRef"));T::Name(name);}

private:
  typename T::VertexType *v[3];
};

template <class T>
void ComputeNormal(T &f) {	f.N().Import(vcg::Normal<T>(f)); }

template <class T>
void ComputeNormalizedNormal(T &f) {	f.N().Import(vcg::NormalizedNormal<T>(f)); }

template <class A, class T> class NormalAbs: public T {
public:
  typedef A NormalType;
  inline NormalType &N() { return _norm; }
  inline NormalType cN() const { return _norm; }
  template <class RightValueType>
  void ImportData(const RightValueType & rightF)
  {
    if(rightF.IsNormalEnabled()) N().Import(rightF.cN());
    T::ImportData(rightF);
  }

  inline void Alloc(const int & ns){T::Alloc(ns);}
  inline void Dealloc(){T::Dealloc();}
  static bool HasNormal()   { return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("NormalAbs"));T::Name(name);}

private:
  NormalType _norm;
};

template <class T> class WedgeNormal: public T {
public:
  typedef typename T::VertexType::NormalType NormalType;
  inline NormalType &WN(int j)       { return _wnorm[j]; }
  inline NormalType cWN(int j) const { return _wnorm[j]; }
  template <class RightValueType>
  void ImportData(const RightValueType & rightF){ if(rightF.IsWedgeNormalEnabled()) for (int i=0; i<3; ++i) { WN(i) = rightF.cWN(i); } T::ImportData(rightF);}
  inline void Alloc(const int & ns){T::Alloc(ns);}
  inline void Dealloc(){T::Dealloc();}
  static bool HasWedgeNormal()   { return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("WedgeNormal"));T::Name(name);}

private:
  NormalType _wnorm[3];
};

template <class A, class T> class WedgeRealNormal: public T {
public:
  typedef A NormalType;
  inline NormalType &WN(int i)       { return _wn[i]; }
  inline NormalType cWN(int i) const { return _wn[i]; }
  template <class RightValueType>
  void ImportData(const RightValueType & rightF){ if(RightValueType::HasWedgeNormal()) for (int i=0; i<3; ++i) { WN(i) = rightF.cWN(i); } T::ImportData(rightF);}
  inline void Alloc(const int & ns){T::Alloc(ns);}
  inline void Dealloc(){T::Dealloc();}
  static bool HasWedgeNormal()   { return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("WedgeRealNormal"));T::Name(name);}

private:
  NormalType _wn[3];
};

template <class TT> class WedgeRealNormal3s: public WedgeRealNormal<vcg::Point3s, TT> {
public:  static void Name(std::vector<std::string> & name){name.push_back(std::string("WedgeRealNormal2s"));TT::Name(name);}};
template <class TT> class WedgeRealNormal3f: public WedgeRealNormal<vcg::Point3f, TT> {
public:  static void Name(std::vector<std::string> & name){name.push_back(std::string("WedgeRealNormal2f"));TT::Name(name);}};
template <class TT> class WedgeRealNormal3d: public WedgeRealNormal<vcg::Point3d, TT> {
public:  static void Name(std::vector<std::string> & name){name.push_back(std::string("WedgeRealNormal2d"));TT::Name(name);}};

template <class T> class Normal3s: public NormalAbs<vcg::Point3s, T> {
public:static void Name(std::vector<std::string> & name){name.push_back(std::string("Normal3s"));T::Name(name);}
};
template <class T> class Normal3f: public NormalAbs<vcg::Point3f, T> {
public:  static void Name(std::vector<std::string> & name){name.push_back(std::string("Normal3f"));T::Name(name);}
};
template <class T> class Normal3d: public NormalAbs<vcg::Point3d, T> {
public: static void Name(std::vector<std::string> & name){name.push_back(std::string("Normal3d"));T::Name(name);}
};


/*-------------------------- TexCoord ----------------------------------------*/

template <class A, class T> class WedgeTexCoord: public T {
public:
  typedef int WedgeTexCoordType;
  typedef A TexCoordType;
  TexCoordType &WT(const int i)       { return _wt[i]; }
  TexCoordType cWT(const int i) const { return _wt[i]; }
  template <class RightValueType>
  void ImportData(const RightValueType & rightF){
    if(rightF.IsWedgeTexCoordEnabled())
      for (int i=0; i<3; ++i) { WT(i) = rightF.cWT(i); }
    T::ImportData(rightF);
  }
  inline void Alloc(const int & ns){T::Alloc(ns);}
  inline void Dealloc(){T::Dealloc();}
  static bool HasWedgeTexCoord()   { return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("WedgeTexCoord"));T::Name(name);}

private:
  TexCoordType _wt[3];
};

template <class TT> class WedgeTexCoord2s: public WedgeTexCoord<TexCoord2<short,1>, TT> {
public:  static void Name(std::vector<std::string> & name){name.push_back(std::string("WedgeTexCoord2s"));TT::Name(name);}
};
template <class TT> class WedgeTexCoord2f: public WedgeTexCoord<TexCoord2<float,1>, TT> {
public:  static void Name(std::vector<std::string> & name){name.push_back(std::string("WedgeTexCoord2f"));TT::Name(name);}
};
template <class TT> class WedgeTexCoord2d: public WedgeTexCoord<TexCoord2<double,1>, TT> {
public: static void Name(std::vector<std::string> & name){name.push_back(std::string("WedgeTexCoord2d"));TT::Name(name);}
};

/*------------------------- BitFlags -----------------------------------------*/
/*! \brief \em Component: Per face \b Flags

This component stores a 32 bit array of bit flags. These bit flags are used for keeping track of selection, deletion, visiting etc. \sa \ref flags for more details on common uses of flags.
*/
template <class T> class BitFlags:  public T {
public:
  BitFlags():_flags(0) {}
  int &Flags()       {return _flags; }
  int cFlags() const {return _flags; }
  template <class RightValueType>
  void ImportData(const RightValueType & rightF){
    if(RightValueType::HasFlags())
      Flags() = rightF.cFlags();
    T::ImportData(rightF);
  }
  inline void Alloc(const int & ns){T::Alloc(ns);}
  inline void Dealloc(){T::Dealloc();}
  static bool HasFlags()   { return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("BitFlags"));T::Name(name);}

private:
  int  _flags;
};

/*-------------------------- Color ----------------------------------*/
template <class A, class T> class Color: public T {
public:
  typedef A ColorType;
  Color():_color(vcg::Color4b::White) {}
  ColorType &C()       { return _color; }
  ColorType cC() const { return _color; }
  template <class RightValueType>
  void ImportData(const RightValueType & rightF){
    if(rightF.IsColorEnabled()) C() = rightF.cC();
    T::ImportData(rightF);
  }
  inline void Alloc(const int & ns){T::Alloc(ns);}
  inline void Dealloc(){T::Dealloc();}
  static bool HasColor()   { return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("Color"));T::Name(name);}

private:
  ColorType _color;
};

template <class A, class T> class WedgeColor: public T {
public:
  typedef A ColorType;
  ColorType &WC(int i) { return _color[i]; }
  ColorType cWC(int i) const { return _color[i]; }

  template <class RightValueType>
  void ImportData(const RightValueType & rightF){
    if (rightF.IsWedgeColorEnabled())
    {
      for (int i=0; i<3; ++i) { WC(i) = rightF.cWC(i); }
    }
    T::ImportData(rightF);

  }
  static bool HasWedgeColor()   { return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("WedgeColor"));T::Name(name);}

private:
  ColorType _color[3];
};

template <class T> class WedgeColor4b: public WedgeColor<vcg::Color4b, T> {
public: static void Name(std::vector<std::string> & name){name.push_back(std::string("WedgeColor4b"));T::Name(name);}
};
template <class T> class WedgeColor4f: public WedgeColor<vcg::Color4f, T> {
public: static void Name(std::vector<std::string> & name){name.push_back(std::string("WedgeColor4f"));T::Name(name);}
};
template <class T> class Color4b: public Color<vcg::Color4b, T> { public:
public: static void Name(std::vector<std::string> & name){name.push_back(std::string("Color4b"));T::Name(name);}
};

/*-------------------------- Quality  ----------------------------------*/
template <class A, class T> class Quality: public T {
public:
  typedef A QualityType;
  Quality():_quality(0) {}
  QualityType &Q()       { return _quality; }
  QualityType cQ() const { return _quality; }
    template <class RightValueType>
    void ImportData(const RightValueType & rightF){
      if(rightF.IsQualityEnabled())
        Q() = rightF.cQ();
      T::ImportData(rightF);
    }
    inline void Alloc(const int & ns){T::Alloc(ns);}
    inline void Dealloc(){T::Dealloc();}
  static bool HasQuality()   { return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("Quality"));T::Name(name);}
private:
  QualityType _quality;
};

template <class T> class Qualitys: public Quality<short, T> {
public:  static void Name(std::vector<std::string> & name){name.push_back(std::string("Qualitys"));T::Name(name);}
};
template <class T> class Qualityf: public Quality<float, T> {
public:  static void Name(std::vector<std::string> & name){name.push_back(std::string("Qualityf"));T::Name(name);}
};
template <class T> class Qualityd: public Quality<double, T> {
public:  static void Name(std::vector<std::string> & name){name.push_back(std::string("Qualityd"));T::Name(name);}
};

/*-------------------------- Quality3  ----------------------------------*/
template <class A, class T> class Quality3: public T {
public:
  typedef vcg::Point3<A> Quality3Type;
  Quality3Type &Q3()       { return _quality; }
  Quality3Type cQ3() const { return _quality; }
  template <class RightValueType>
  void ImportData(const RightValueType & rightF){
    if(rightF.IsQuality3Enabled()) Q3() = rightF.cQ3();
    T::ImportData(rightF);
  }
  inline void Alloc(const int & ns){T::Alloc(ns);}
  inline void Dealloc(){T::Dealloc();}
  static bool HasQuality3()   { return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("Quality3"));T::Name(name);}
private:
  Quality3Type _quality;
};

template <class T> class Quality3s: public Quality3<short, T> {
public:  static void Name(std::vector<std::string> & name){name.push_back(std::string("Quality3s"));T::Name(name);}
};
template <class T> class Quality3f: public Quality3<float, T> {
public:  static void Name(std::vector<std::string> & name){name.push_back(std::string("Quality3f"));T::Name(name);}
};
template <class T> class Quality3d: public Quality3<double, T> {
public:  static void Name(std::vector<std::string> & name){name.push_back(std::string("Quality3d"));T::Name(name);}
};

/*-------------------------- INCREMENTAL MARK  ----------------------------------------*/
/*! \brief Per vertex \b Incremental \b Mark

    It is just an \c int that allows to efficently (in constant time) un-mark the whole mesh. \sa UnmarkAll
    */

template <class T> class Mark: public T {
public:
  Mark():_imark(0){}
  inline int &IMark()       { return _imark;}
  inline int cIMark() const { return _imark;}
  inline void InitIMark()    { _imark = 0; }
  static bool HasMark()      { return true; }
  template <class RightValueType>
  void ImportData(const RightValueType & rightF){
    if(rightF.IsMarkEnabled())
      IMark() = rightF.cIMark();
    T::ImportData(rightF);
  }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("Mark"));T::Name(name);}

private:
  int _imark;
};

/*-------------------------- Curvature Direction ----------------------------------*/
template <class S>
struct CurvatureDirBaseType{
        typedef Point3<S> VecType;
        typedef  S   ScalarType;
        CurvatureDirBaseType () {}
        Point3<S>max_dir,min_dir; // max and min curvature direction
        S k1,k2;// max and min curvature values
};

template <class A, class TT> class CurvatureDir: public TT {
public:
  typedef A CurvatureDirType;
  typedef typename CurvatureDirType::VecType VecType;
  typedef typename CurvatureDirType::ScalarType ScalarType;

  VecType &PD1()       { return _curv.max_dir;}
  VecType &PD2()       { return _curv.min_dir;}
  VecType cPD1() const { return _curv.max_dir;}
  VecType cPD2() const { return _curv.min_dir;}

  ScalarType &K1()       { return _curv.k1;}
  ScalarType &K2()       { return _curv.k2;}
  ScalarType cK1() const {return _curv.k1;}
  ScalarType cK2() const {return _curv.k2;}
  template < class RightValueType>
  void ImportData(const RightValueType  & rightF ) {
    if(rightF.IsCurvatureDirEnabled()) {
      PD1() = rightF.cPD1(); PD2() = rightF.cPD2();
      K1()  = rightF.cK1();  K2()  = rightF.cK2();
    }
    TT::ImportData(rightF);
  }

  static bool HasCurvatureDir()   { return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("CurvatureDir"));TT::Name(name);}

private:
  CurvatureDirType _curv;
};


template <class T> class CurvatureDirf: public CurvatureDir<CurvatureDirBaseType<float>, T> {
public:	static void Name(std::vector<std::string> & name){name.push_back(std::string("CurvatureDirf"));T::Name(name);}
};
template <class T> class CurvatureDird: public CurvatureDir<CurvatureDirBaseType<double>, T> {
public:	static void Name(std::vector<std::string> & name){name.push_back(std::string("CurvatureDird"));T::Name(name);}
};

/*----------------------------- VFADJ ------------------------------*/
/*! \brief \em Component: Per Face \b Vertex-Face adjacency relation

It stores a pointer to the next face of the list of faces incident on a vertex that is stored in a distributed way on the faces themselves.
Note that if you use this component it is expected that on the Vertex you use also the corresponding vcg::vertex::VFAdj component.
Note that for this component we have three class of values:
- \b valid: a valid pointer in the range of the vector of faces
- \b null: a null pointer, used to indicate the end of the list
- \b uninitialized: a special value that you can test/set with the IsVFInitialized()/VFClear() functions;
     it is used to indicate when the VF Topology is not computed.

\sa vcg::tri::UpdateTopology for functions that compute this relation
\sa vcg::vertex::VFAdj
\sa iterators
*/


template <class T> class VFAdj: public T {
public:
    VFAdj(){
        _vfp[0]=0;
        _vfp[1]=0;
        _vfp[2]=0;
        _vfi[0]=-1;
        _vfi[1]=-1;
        _vfi[2]=-1;
    }
  typename T::FacePointer &VFp(const int j)        { assert(j>=0 && j<3);  return _vfp[j]; }
  typename T::FacePointer cVFp(const int j) const  { assert(j>=0 && j<3);  return _vfp[j]; }
  char &VFi(const int j) {return _vfi[j]; }
  char cVFi(const int j)const {return _vfi[j]; }
    template <class RightValueType>
    void ImportData(const RightValueType & rightF){T::ImportData(rightF);}
    inline void Alloc(const int & ns){T::Alloc(ns);}
    inline void Dealloc(){T::Dealloc();}
  static bool HasVFAdjacency()      {   return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("VFAdj"));T::Name(name);}

private:
  typename T::FacePointer _vfp[3] ;
  char _vfi[3] ;
};

/*----------------------------- EFADJ ------------------------------*/
template <class T> class EFAdj: public T {
public:
    EFAdj(){
        _efp[0]=0;
        _efp[1]=0;
        _efp[2]=0;
        _efi[0]=-1;
        _efi[1]=-1;
        _efi[2]=-1;
    }
  typename T::FacePointer &EFp(const int j)       { assert(j>=0 && j<3);  return _efp[j]; }
  typename T::FacePointer cEFp(const int j) const { assert(j>=0 && j<3);  return _efp[j]; }
  char &VFi(const int j) {return _efi[j]; }
  template <class RightValueType>
  void ImportData(const RightValueType & rightF){T::ImportData(rightF);}
  inline void Alloc(const int & ns){T::Alloc(ns);}
  inline void Dealloc(){T::Dealloc();}
  static bool HasEFAdjacency()      {   return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("EFAdj"));T::Name(name);}

private:
  typename T::FacePointer _efp[3] ;
  char _efi[3] ;
};


/*----------------------------- FFADJ ------------------------------*/
/*! \brief \em Component: Per Face \b Face-Face adjacency relation

It encodes the adjacency of faces through edges; for 2-manifold edges it just point to the other face,
and for non manifold edges (where more than 2 faces share the same edge) it stores a pointer to the next
face of the ring of faces incident on a edge.
 Note that border faces points to themselves.
 NULL pointer is used as a special value to indicate when the FF Topology is not computed.

\sa vcg::tri::UpdateTopology for functions that compute this relation
\sa vcg::vertex::VFAdj
\sa iterators
*/

template <class T> class FFAdj: public T {
public:
  FFAdj(){
    _ffp[0]=0;
    _ffp[1]=0;
    _ffp[2]=0;
  }
  typename T::FacePointer &FFp(const int j)        { assert(j>=0 && j<3);  return _ffp[j]; }
  typename T::FacePointer cFFp(const int j) const  { assert(j>=0 && j<3);  return _ffp[j]; }
  char &FFi(const int j)       { return _ffi[j]; }
  char cFFi(const int j) const { return _ffi[j]; }

  typename T::FacePointer &FFp1( const int j )       { return FFp((j+1)%3);}
  typename T::FacePointer &FFp2( const int j )       { return FFp((j+2)%3);}
  typename T::FacePointer cFFp1( const int j ) const { return FFp((j+1)%3);}
  typename T::FacePointer cFFp2( const int j ) const { return FFp((j+2)%3);}

  template <class RightValueType>
  void ImportData(const RightValueType & rightF){T::ImportData(rightF);}
  inline void Alloc(const int & ns){T::Alloc(ns);}
  inline void Dealloc(){T::Dealloc();}
  static bool HasFFAdjacency()      {   return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("FFAdj"));T::Name(name);}

private:
  typename T::FacePointer _ffp[3] ;
  char _ffi[3] ;
};


/*----------------------------- FEADJ ------------------------------*/

template <class T> class FEAdj: public T {
public:
  FEAdj(){
    _fep[0]=0;
    _fep[1]=0;
    _fep[2]=0;
  }

  typename T::EdgePointer &FEp( int j)        { assert(j>=0 && j<3);  return _fep[j]; }
  typename T::EdgePointer cFEp( int j) const  { assert(j>=0 && j<3);  return _fep[j]; }

  typename T::EdgePointer &FEp1( int j )       { return FEp((j+1)%3);}
  typename T::EdgePointer &FEp2( int j )       { return FEp((j+2)%3);}
  typename T::EdgePointer  FEp1( int j ) const { return FEp((j+1)%3);}
  typename T::EdgePointer  FEp2( int j ) const { return FEp((j+2)%3);}

  template <class RightValueType>
  void ImportData(const RightValueType & rightF){T::ImportData(rightF);}
  inline void Alloc(const int & ns){T::Alloc(ns);}
  inline void Dealloc(){T::Dealloc();}
  static bool HasFEAdjacency()      {   return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("FEAdj"));T::Name(name);}

private:
  typename T::EdgePointer _fep[3] ;
  char _fei[3] ;
};


/*----------------------------- FHADJ ------------------------------*/
template <class T> class FHAdj: public T {
public:
  FHAdj(){_fh=0;}
  typename T::HEdgePointer &FHp( )       { return _fh; }
  typename T::HEdgePointer cFHp( ) const { return _fh; }

  template <class RightValueType>
  void ImportData(const RightValueType & rightF){T::ImportData(rightF);}
  inline void Alloc(const int & ns){T::Alloc(ns);}
  inline void Dealloc(){T::Dealloc();}
  static bool HasFHAdjacency()      {   return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("FHAdj"));T::Name(name);}

private:
  typename T::HEdgePointer _fh ;
};
/** @} */   // End Doxygen FaceComponentGroup
  } // end namespace face
}// end namespace vcg
#endif
