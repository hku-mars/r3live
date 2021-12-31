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
#ifndef __VCG_FACE_PLUS_COMPONENT_OCF
#define __VCG_FACE_PLUS_COMPONENT_OCF
#ifndef __VCG_MESH
#error "This file should not be included alone. It is automatically included by complex.h"
#endif

namespace vcg {
  namespace face {
/*
All the Components that can be added to a faceex should be defined in the namespace face:

*/

template <class VALUE_TYPE>
class vector_ocf: public std::vector<VALUE_TYPE> {
  typedef std::vector<VALUE_TYPE> BaseType;
    typedef typename vector_ocf<VALUE_TYPE>::iterator ThisTypeIterator;

public:
  vector_ocf():std::vector<VALUE_TYPE>()
  {
    _ColorEnabled=false;
    CurvatureDirEnabled = false;
    MarkEnabled=false;
    NormalEnabled=false;
    QualityEnabled=false;
    WedgeTexEnabled=false;
    WedgeColorEnabled=false;
    WedgeNormalEnabled=false;
    VFAdjacencyEnabled=false;
    FFAdjacencyEnabled=false;
  }

// Auxiliary types to build internal vectors
struct AdjTypePack {
  typename VALUE_TYPE::FacePointer _fp[3] ;
  char _zp[3] ;

  // Default constructor.
  // Needed because we need to know if adjacency is initialized or not
  // when resizing vectors and during an allocate face.
  AdjTypePack() {
        _fp[0]=0;
        _fp[1]=0;
        _fp[2]=0;
  }
  };

//template <class TexCoordType>
class WedgeTexTypePack {
public:
  WedgeTexTypePack() {
    wt[0].U()=.5;wt[0].V()=.5;
    wt[1].U()=.5;wt[1].V()=.5;
    wt[2].U()=.5;wt[2].V()=.5;
    wt[0].N()=-1;
    wt[1].N()=-1;
    wt[2].N()=-1;
  }

  typename VALUE_TYPE::TexCoordType wt[3];
};

class WedgeColorTypePack {
public:
  WedgeColorTypePack() {
  typedef typename VALUE_TYPE::ColorType::ScalarType WedgeColorScalarType;
    for (int i=0; i<3; ++i)
    {
    wc[i][0] = WedgeColorScalarType(255);
    wc[i][1] = WedgeColorScalarType(255);
    wc[i][2] = WedgeColorScalarType(255);
    wc[i][3] = WedgeColorScalarType(255);
    }
  }

  typename VALUE_TYPE::ColorType wc[3];
};

class WedgeNormalTypePack {
public:
  WedgeNormalTypePack() {
  typedef typename VALUE_TYPE::NormalType::ScalarType WedgeNormalScalarType;
    for (int i=0; i<3; ++i)
    {
    wn[i][0] = WedgeNormalScalarType(0);
    wn[i][1] = WedgeNormalScalarType(0);
    wn[i][2] = WedgeNormalScalarType(1);
    }
  }

  typename VALUE_TYPE::NormalType wn[3];
};


////////////////////////////////////////
// All the standard methods of std::vector that can change the reallocation are
// redefined in order to manage the additional data.
    void push_back(const VALUE_TYPE & v)
  {
    BaseType::push_back(v);
    BaseType::back()._ovp = this;
    if (QualityEnabled)     QV.push_back(0);
    if (_ColorEnabled)       CV.push_back(vcg::Color4b(vcg::Color4b::White));
    if (MarkEnabled)        MV.push_back(0);
    if (NormalEnabled)      NV.push_back(typename VALUE_TYPE::NormalType());
    if (CurvatureDirEnabled)    CDV.push_back(typename VALUE_TYPE::CurvatureDirType());
    if (VFAdjacencyEnabled) AV.push_back(AdjTypePack());
    if (FFAdjacencyEnabled) AF.push_back(AdjTypePack());
    if (WedgeTexEnabled)    WTV.push_back(WedgeTexTypePack());
    if (WedgeColorEnabled)  WCV.push_back(WedgeColorTypePack());
    if (WedgeNormalEnabled) WNV.push_back(WedgeNormalTypePack());
  }
    void pop_back();
  void resize(const unsigned int & _size)
  {
      unsigned int oldsize = BaseType::size();
    BaseType::resize(_size);
      if(oldsize<_size){
          ThisTypeIterator firstnew = BaseType::begin();
          advance(firstnew,oldsize);
          _updateOVP(firstnew,(*this).end());
      }
    if (QualityEnabled)     QV.resize(_size,0);
    if (_ColorEnabled)       CV.resize(_size);
    if (MarkEnabled)        MV.resize(_size);
    if (NormalEnabled)      NV.resize(_size);
    if (CurvatureDirEnabled)CDV.resize(_size);
    if (VFAdjacencyEnabled) AV.resize(_size);
    if (FFAdjacencyEnabled) AF.resize(_size);
    if (WedgeTexEnabled)    WTV.resize(_size,WedgeTexTypePack());
    if (WedgeColorEnabled)  WCV.resize(_size);
    if (WedgeNormalEnabled) WNV.resize(_size);
   }
  void reserve(const unsigned int & _size)
  {
    BaseType::reserve(_size);

    if (QualityEnabled)     QV.reserve(_size);
    if (_ColorEnabled)      CV.reserve(_size);
    if (MarkEnabled)        MV.reserve(_size);
    if (NormalEnabled)      NV.reserve(_size);
    if (CurvatureDirEnabled)CDV.reserve(_size);
    if (VFAdjacencyEnabled) AV.reserve(_size);
    if (FFAdjacencyEnabled) AF.reserve(_size);
    if (WedgeTexEnabled)    WTV.reserve(_size);
    if (WedgeColorEnabled)  WCV.reserve(_size);
    if (WedgeNormalEnabled) WNV.reserve(_size);

    if( BaseType::empty()) return ;

    ThisTypeIterator oldbegin=(*this).begin();
    if(oldbegin!=(*this).begin()) _updateOVP((*this).begin(),(*this).end());
  }

 void _updateOVP(ThisTypeIterator lbegin, ThisTypeIterator lend)
{
    ThisTypeIterator fi;
    //for(fi=(*this).begin();vi!=(*this).end();++vi)
    for(fi=lbegin;fi!=lend;++fi)
        (*fi)._ovp=this;
 }



// this function is called by the specialized Reorder function, that is called whenever someone call the allocator::CompactVertVector
void ReorderFace(std::vector<size_t> &newFaceIndex )
{
	size_t i=0;
	if (QualityEnabled)     assert( QV.size() == newFaceIndex.size() );
	if (_ColorEnabled)       assert( CV.size() == newFaceIndex.size() );
	if (MarkEnabled)        assert( MV.size() == newFaceIndex.size() );
	if (NormalEnabled)      assert( NV.size() == newFaceIndex.size() );
	if (CurvatureDirEnabled)assert(CDV.size() == newFaceIndex.size() );
	if (VFAdjacencyEnabled) assert( AV.size() == newFaceIndex.size() );
	if (FFAdjacencyEnabled) assert( AF.size() == newFaceIndex.size() );
	if (WedgeTexEnabled)    assert(WTV.size() == newFaceIndex.size() );
	if (WedgeColorEnabled)  assert(WCV.size() == newFaceIndex.size() );
	if (WedgeNormalEnabled) assert(WNV.size() == newFaceIndex.size() );

	for(i=0;i<newFaceIndex.size();++i)
		{
			if(newFaceIndex[i] != std::numeric_limits<size_t>::max() )
				{
					assert(newFaceIndex[i] <= i);
					if (QualityEnabled)      QV[newFaceIndex[i]] =  QV[i];
					if (_ColorEnabled)        CV[newFaceIndex[i]] =  CV[i];
					if (MarkEnabled)         MV[newFaceIndex[i]] =  MV[i];
					if (NormalEnabled)       NV[newFaceIndex[i]] =  NV[i];
					if (CurvatureDirEnabled) CDV[newFaceIndex[i]] =  CDV[i];
					if (VFAdjacencyEnabled)  AV[newFaceIndex[i]] =  AV[i];
					if (FFAdjacencyEnabled)  AF[newFaceIndex[i]] =  AF[i];
					if (WedgeTexEnabled)    WTV[newFaceIndex[i]] = WTV[i];
					if (WedgeColorEnabled)  WCV[newFaceIndex[i]] = WCV[i];
					if (WedgeNormalEnabled) WNV[newFaceIndex[i]] = WNV[i];
				}
		}

	if (QualityEnabled)      QV.resize(BaseType::size(),0);
	if (_ColorEnabled)       CV.resize(BaseType::size());
	if (MarkEnabled)         MV.resize(BaseType::size());
	if (NormalEnabled)       NV.resize(BaseType::size());
	if (CurvatureDirEnabled) CDV.resize(BaseType::size());
	if (VFAdjacencyEnabled)  AV.resize(BaseType::size());
	if (FFAdjacencyEnabled)  AF.resize(BaseType::size());
	if (WedgeTexEnabled)    WTV.resize(BaseType::size());
	if (WedgeColorEnabled)  WCV.resize(BaseType::size());
	if (WedgeNormalEnabled) WNV.resize(BaseType::size());
}

////////////////////////////////////////
// Enabling Functions

bool IsQualityEnabled() const {return QualityEnabled;}
void EnableQuality() {
	assert(VALUE_TYPE::HasQualityOcf());
	QualityEnabled=true;
	QV.resize((*this).size(),0);
}

void DisableQuality() {
	assert(VALUE_TYPE::HasQualityOcf());
	QualityEnabled=false;
	QV.clear();
}

bool IsColorEnabled() const {return _ColorEnabled;}
void EnableColor() {
  assert(VALUE_TYPE::HasColorOcf());
  _ColorEnabled=true;
  CV.resize((*this).size());
}

void DisableColor() {
  assert(VALUE_TYPE::HasColorOcf());
  _ColorEnabled=false;
  CV.clear();
}

bool IsMarkEnabled() const {return MarkEnabled;}
void EnableMark() {
  assert(VALUE_TYPE::HasMarkOcf());
  MarkEnabled=true;
  MV.resize((*this).size(),0);
}

void DisableMark() {
  assert(VALUE_TYPE::HasMarkOcf());
  MarkEnabled=false;
  MV.clear();
}

bool IsNormalEnabled() const {return NormalEnabled;}
void EnableNormal() {
  assert(VALUE_TYPE::HasNormalOcf());
  NormalEnabled=true;
  NV.resize((*this).size());
}

void DisableNormal() {
  assert(VALUE_TYPE::HasNormalOcf());
  NormalEnabled=false;
  NV.clear();
}

bool IsCurvatureDirEnabled() const {return CurvatureDirEnabled;}
void EnableCurvatureDir() {
  assert(VALUE_TYPE::HasCurvatureDirOcf());
  CurvatureDirEnabled=true;
  CDV.resize((*this).size());
}

void DisableCurvatureDir() {
  assert(VALUE_TYPE::HasCurvatureDirOcf());
  CurvatureDirEnabled=false;
  CDV.clear();
}


bool IsVFAdjacencyEnabled() const {return VFAdjacencyEnabled;}
void EnableVFAdjacency() {
  assert(VALUE_TYPE::HasVFAdjacencyOcf());
  VFAdjacencyEnabled=true;
  AV.resize((*this).size());
}

void DisableVFAdjacency() {
  assert(VALUE_TYPE::HasVFAdjacencyOcf());
  VFAdjacencyEnabled=false;
  AV.clear();
}


bool IsFFAdjacencyEnabled() const {return FFAdjacencyEnabled;}
void EnableFFAdjacency() {
  assert(VALUE_TYPE::HasFFAdjacencyOcf());
  FFAdjacencyEnabled=true;
  AF.resize((*this).size());
}

void DisableFFAdjacency() {
  assert(VALUE_TYPE::HasFFAdjacencyOcf());
  FFAdjacencyEnabled=false;
  AF.clear();
}

bool IsWedgeTexCoordEnabled() const {return WedgeTexEnabled;}
void EnableWedgeTexCoord() {
  assert(VALUE_TYPE::HasWedgeTexCoordOcf());
  WedgeTexEnabled=true;
  WTV.resize((*this).size(),WedgeTexTypePack());
}

void DisableWedgeTexCoord() {
  assert(VALUE_TYPE::HasWedgeTexCoordOcf());
  WedgeTexEnabled=false;
  WTV.clear();
}

bool IsWedgeColorEnabled() const {return WedgeColorEnabled;}
void EnableWedgeColor() {
  assert(VALUE_TYPE::HasWedgeColorOcf());
  WedgeColorEnabled=true;
  WCV.resize((*this).size(),WedgeColorTypePack());
}

void DisableWedgeColor() {
  assert(VALUE_TYPE::HasWedgeColorOcf());
  WedgeColorEnabled=false;
  WCV.clear();
}

bool IsWedgeNormalEnabled() const {return WedgeNormalEnabled;}
void EnableWedgeNormal() {
  assert(VALUE_TYPE::HasWedgeNormalOcf());
  WedgeNormalEnabled=true;
  WNV.resize((*this).size(),WedgeNormalTypePack());
}

void DisableWedgeNormal() {
  assert(VALUE_TYPE::HasWedgeNormalOcf());
  WedgeNormalEnabled=false;
  WNV.clear();
}

public:
  std::vector<typename VALUE_TYPE::ColorType> CV;
  std::vector<typename VALUE_TYPE::CurvatureDirType> CDV;
  std::vector<int> MV;
  std::vector<typename VALUE_TYPE::NormalType> NV;
  std::vector<float> QV;
  std::vector<class WedgeColorTypePack> WCV;
  std::vector<class WedgeNormalTypePack> WNV;
  std::vector<class WedgeTexTypePack> WTV;
  std::vector<struct AdjTypePack> AV;
  std::vector<struct AdjTypePack> AF;

  bool _ColorEnabled;
  bool CurvatureDirEnabled;
  bool MarkEnabled;
  bool NormalEnabled;
  bool QualityEnabled;
  bool WedgeColorEnabled;
  bool WedgeNormalEnabled;
  bool WedgeTexEnabled;
  bool VFAdjacencyEnabled;
  bool FFAdjacencyEnabled;
}; // end class vector_ocf


/*----------------------------- VFADJ ------------------------------*/
template <class T> class VFAdjOcf: public T {
public:
  typename T::FacePointer &VFp(const int j) {
    assert((*this).Base().VFAdjacencyEnabled);
    return (*this).Base().AV[(*this).Index()]._fp[j];
  }

  typename T::FacePointer cVFp(const int j) const {
    if(! (*this).Base().VFAdjacencyEnabled ) return 0;
    else return (*this).Base().AV[(*this).Index()]._fp[j];
  }

  char &VFi(const int j) {
    assert((*this).Base().VFAdjacencyEnabled);
    return (*this).Base().AV[(*this).Index()]._zp[j];
  }

  char cVFi(const int j) const {
    assert((*this).Base().VFAdjacencyEnabled);
    return (*this).Base().AV[(*this).Index()]._zp[j];
  }

	template <class RightFaceType>
	void ImportData(const RightFaceType & rightF){
		T::ImportData(rightF);
	}
  static bool HasVFAdjacency()   {   return true; }
  static bool HasVFAdjacencyOcf()   { return true; }

private:
};

/*----------------------------- FFADJ ------------------------------*/
template <class T> class FFAdjOcf: public T {
public:
  typename T::FacePointer &FFp(const int j) {
    assert((*this).Base().FFAdjacencyEnabled);
    return (*this).Base().AF[(*this).Index()]._fp[j];
  }

  typename T::FacePointer cFFp(const int j) const {
    if(! (*this).Base().FFAdjacencyEnabled ) return 0;
    else return (*this).Base().AF[(*this).Index()]._fp[j];
  }

  char &FFi(const int j)       {
    assert((*this).Base().FFAdjacencyEnabled);
    return (*this).Base().AF[(*this).Index()]._zp[j];
  }
  char cFFi(const int j) const {
    assert((*this).Base().FFAdjacencyEnabled);
    return (*this).Base().AF[(*this).Index()]._zp[j];
  }

  typename T::FacePointer  &FFp1( const int j )       { return FFp((j+1)%3);}
  typename T::FacePointer  &FFp2( const int j )       { return FFp((j+2)%3);}
  typename T::FacePointer  cFFp1( const int j ) const { return FFp((j+1)%3);}
  typename T::FacePointer  cFFp2( const int j ) const { return FFp((j+2)%3);}

  typename T::FacePointer  &Neigh( const int j )		{ return FFp(j);}
  typename T::FacePointer  cNeigh( const int j ) const { return cFFp(j);}
  unsigned int SizeNeigh(){return 3;}

  template <class RightFaceType>
  void ImportData(const RightFaceType & rightF){
    T::ImportData(rightF);
  }
  static bool HasFFAdjacency()   {   return true; }
  static bool HasFFAdjacencyOcf()   { return true; }
};

/*------------------------- Normal -----------------------------------------*/
template <class A, class T> class NormalOcf: public T {
public:
  typedef A NormalType;
  inline bool IsNormalEnabled( )        const  { return this->Base().IsNormalEnabled(); }
  static bool HasNormal()   { return true; }
  static bool HasNormalOcf()   { return true; }

  NormalType &N()       {
    // you cannot use Normals before enabling them with: yourmesh.face.EnableNormal()
    assert((*this).Base().NormalEnabled);
    return (*this).Base().NV[(*this).Index()];  }
  NormalType cN() const {
    // you cannot use Normals before enabling them with: yourmesh.face.EnableNormal()
    assert((*this).Base().NormalEnabled);
    return (*this).Base().NV[(*this).Index()];  }

  template <class RightFaceType>
  void ImportData(const RightFaceType & rightF){
    if((*this).IsNormalEnabled() && rightF.IsNormalEnabled())
      N() = rightF.cN();
    T::ImportData(rightF);
  }
};

template <class T> class Normal3sOcf: public NormalOcf<vcg::Point3s, T> {};
template <class T> class Normal3fOcf: public NormalOcf<vcg::Point3f, T> {};
template <class T> class Normal3dOcf: public NormalOcf<vcg::Point3d, T> {};

/*------------------------- CurvatureDir -----------------------------------------*/
template <class S>
struct CurvatureDirOcfBaseType{
        typedef Point3<S> VecType;
        typedef  S   ScalarType;
        CurvatureDirOcfBaseType () {}
        Point3<S>max_dir,min_dir; // max and min curvature direction
        S k1,k2;// max and min curvature values
};

template <class A, class T> class CurvatureDirOcf: public T {
public:
  typedef A CurvatureDirType;
  typedef typename CurvatureDirType::VecType VecType;
  typedef typename CurvatureDirType::ScalarType ScalarType;

  inline bool IsCurvatureDirEnabled( )  const  { return this->Base().IsCurvatureDirEnabled(); }
  static bool HasCurvatureDir()   { return true; }
  static bool HasCurvatureDirOcf()   { return true; }

  VecType &PD1()       {
    assert((*this).Base().CurvatureDirEnabled);
    return (*this).Base().CDV[(*this).Index()].max_dir;
  }

  VecType &PD2()       {
    assert((*this).Base().CurvatureDirEnabled);
    return (*this).Base().CDV[(*this).Index()].min_dir;
  }

  VecType cPD1() const {
    assert((*this).Base().CurvatureDirEnabled);
    return (*this).Base().CDV[(*this).Index()].max_dir;
  }

  VecType cPD2() const {
    assert((*this).Base().CurvatureDirEnabled);
    return (*this).Base().CDV[(*this).Index()].min_dir;
  }

  ScalarType &K1()       {
    // you cannot use Normals before enabling them with: yourmesh.face.EnableNormal()
    assert((*this).Base().NormalEnabled);
    return (*this).Base().CDV[(*this).Index()].k1;
  }
  ScalarType &K2()       {
    // you cannot use Normals before enabling them with: yourmesh.face.EnableNormal()
    assert((*this).Base().NormalEnabled);
    return (*this).Base().CDV[(*this).Index()].k2;
  }
  ScalarType cK1() const {
    // you cannot use Normals before enabling them with: yourmesh.face.EnableNormal()
    assert((*this).Base().NormalEnabled);
    return (*this).Base().CDV[(*this).Index()].k1;
  }
  ScalarType cK2() const {
    // you cannot use Normals before enabling them with: yourmesh.face.EnableNormal()
    assert((*this).Base().NormalEnabled);
    return (*this).Base().CDV[(*this).Index()].k2;
  }


  template <class RightFaceType>
  void ImportData(const RightFaceType & rightF){
    if((*this).IsCurvatureDirEnabled() && rightF.IsCurvatureDirEnabled())
      PD1() = rightF.cPD1();
    PD2() = rightF.cPD2();
    K1() = rightF.cK1();
    K2() = rightF.cK2();
    T::ImportData(rightF);
  }

};

template <class T> class CurvatureDirfOcf: public CurvatureDirOcf<CurvatureDirOcfBaseType<float>, T> {
public:	static void Name(std::vector<std::string> & name){name.push_back(std::string("CurvatureDirfOcf"));T::Name(name);}
};
template <class T> class CurvatureDirdOcf: public CurvatureDirOcf<CurvatureDirOcfBaseType<double>, T> {
public:	static void Name(std::vector<std::string> & name){name.push_back(std::string("CurvatureDirdOcf"));T::Name(name);}
};

///*-------------------------- QUALITY ----------------------------------*/
template <class A, class T> class QualityOcf: public T {
public:
  typedef A QualityType;
  QualityType &Q()        {
    assert((*this).Base().QualityEnabled);
    return (*this).Base().QV[(*this).Index()];
  }
  QualityType cQ() const  {
    assert((*this).Base().QualityEnabled);
    return (*this).Base().QV[(*this).Index()];
  }

  template <class RightFaceType>
  void ImportData(const RightFaceType & rightF){
    if((*this).IsQualityEnabled() && rightF.IsQualityEnabled())
      Q() = rightF.cQ();
    T::ImportData(rightF);
  }
  inline bool IsQualityEnabled( )       const  { return this->Base().IsQualityEnabled(); }
  static bool HasQuality()   { return true; }
  static bool HasQualityOcf()   { return true; }
};

template <class T> class QualityfOcf: public QualityOcf<float, T> {};

///*-------------------------- COLOR ----------------------------------*/
template <class A, class T> class ColorOcf: public T {
public:
  typedef A ColorType;
  ColorType &C()        {
    assert((*this).Base()._ColorEnabled);
    return (*this).Base().CV[(*this).Index()];
  }
  ColorType cC() const  {
    assert((*this).Base()._ColorEnabled);
    return (*this).Base().CV[(*this).Index()];
  }

  template <class RightFaceType>
  void ImportData(const RightFaceType & rightF){
    if((*this).IsColorEnabled() && rightF.IsColorEnabled())
      C() = rightF.cC();
    T::ImportData(rightF);
  }
  inline bool IsColorEnabled()          const  { return this->Base().IsColorEnabled();}
  static bool HasColor()   { return true; }
  static bool HasColorOcf()   { return true; }
};

template <class T> class Color4bOcf: public ColorOcf<vcg::Color4b, T> {};

///*-------------------------- MARK  ----------------------------------*/
template <class T> class MarkOcf: public T {
public:
  inline int &IMark()       {
    assert((*this).Base().MarkEnabled);
    return (*this).Base().MV[(*this).Index()];
  }
  inline int cIMark() const {
    assert((*this).Base().MarkEnabled);
    return (*this).Base().MV[(*this).Index()];
  }

  template <class RightFaceType>
  void ImportData(const RightFaceType & rightF){
    if((*this).IsMarkEnabled() && rightF.IsMarkEnabled())
      IMark() = rightF.cIMark();
    T::ImportData(rightF);
  }
  inline bool IsMarkEnabled( )          const  { return this->Base().IsMarkEnabled(); }
  static bool HasMark()   { return true; }
  static bool HasMarkOcf()   { return true; }
  inline void InitIMark()    { IMark() = 0; }
};

///*-------------------------- WEDGE TEXCOORD  ----------------------------------*/
template <class A, class TT> class WedgeTexCoordOcf: public TT {
public:
  WedgeTexCoordOcf(){ }
  typedef A TexCoordType;
  TexCoordType &WT(const int i)       { assert((*this).Base().WedgeTexEnabled); return (*this).Base().WTV[(*this).Index()].wt[i]; }
  TexCoordType cWT(const int i) const { assert((*this).Base().WedgeTexEnabled); return (*this).Base().WTV[(*this).Index()].wt[i]; }
  template <class RightFaceType>
  void ImportData(const RightFaceType & rightF){
    if(this->IsWedgeTexCoordEnabled() && rightF.IsWedgeTexCoordEnabled())
    { WT(0) = rightF.cWT(0); WT(1) = rightF.cWT(1); WT(2) = rightF.cWT(2); }
    TT::ImportData(rightF);
  }
  inline bool IsWedgeTexCoordEnabled( ) const { return this->Base().IsWedgeTexCoordEnabled(); }
  static bool HasWedgeTexCoord()   { return true; }
  static bool HasWedgeTexCoordOcf()   { return true; }
};

template <class T> class WedgeTexCoordfOcf: public WedgeTexCoordOcf<TexCoord2<float,1>, T> {};

///*-------------------------- WEDGE COLOR  ----------------------------------*/
template <class A, class TT> class WedgeColorOcf: public TT {
public:
  WedgeColorOcf(){ }
  typedef A ColorType;
  ColorType &WC(const int i)              { assert((*this).Base().WedgeColorEnabled); return (*this).Base().WCV[(*this).Index()].wc[i]; }
  const ColorType cWC(const int i) const { assert((*this).Base().WedgeColorEnabled); return (*this).Base().WCV[(*this).Index()].wc[i]; }
  template <class RightFaceType>
  void ImportData(const RightFaceType & rightF){
    if(this->IsWedgeColorEnabled() && rightF.IsWedgeColorEnabled())
    { WC(0) = rightF.cWC(0); WC(1) = rightF.cWC(1); WC(2) = rightF.cWC(2); }
    TT::ImportData(rightF);
  }
  inline bool IsWedgeColorEnabled( )    const { return this->Base().IsWedgeColorEnabled(); }
  static bool HasWedgeColor()   { return true; }
  static bool HasWedgeColorOcf()   { return true; }
};

template <class T> class WedgeColor4bOcf: public WedgeColorOcf<vcg::Color4b, T> {};

///*-------------------------- WEDGE NORMAL ----------------------------------*/
template <class A, class TT> class WedgeNormalOcf: public TT {
public:
  WedgeNormalOcf(){ }
  typedef A NormalType;
  NormalType &WN(const int i)              { assert((*this).Base().WedgeNormalEnabled); return (*this).Base().WNV[(*this).Index()].wn[i]; }
  NormalType const &cWN(const int i) const { assert((*this).Base().WedgeNormalEnabled); return (*this).Base().WNV[(*this).Index()].wn[i]; }
  template <class RightFaceType>
  void ImportData(const RightFaceType & rightF){
    if(this->IsWedgeNormalEnabled() && rightF.IsWedgeNormalEnabled())
    { WN(0) = rightF.cWN(0); WN(1) = rightF.cWN(1); WN(2) = rightF.cWN(2); }
    TT::ImportData(rightF);
  }
  inline bool IsWedgeNormalEnabled( )   const { return this->Base().IsWedgeNormalEnabled(); }
  static bool HasWedgeNormal()   { return true; }
  static bool HasWedgeNormalOcf()   { return true; }
};

template <class T> class WedgeNormal3sOcf: public WedgeNormalOcf<vcg::Point3s, T> {};
template <class T> class WedgeNormal3fOcf: public WedgeNormalOcf<vcg::Point3f, T> {};
template <class T> class WedgeNormal3dOcf: public WedgeNormalOcf<vcg::Point3d, T> {};

///*-------------------------- InfoOpt  ----------------------------------*/
template < class T> class InfoOcf: public T {
public:
  // You should never ever try to copy a vertex that has OCF stuff.
  // use ImportData function.
  inline InfoOcf &operator=(const InfoOcf & /*other*/) {
    assert(0); return *this;
  }


  vector_ocf<typename T::FaceType> &Base() const { return *_ovp;}

  template <class RightFaceType>
  void ImportData(const RightFaceType & rightF){T::ImportData(rightF);}

  static bool HasColorOcf()        { return false; }
  static bool HasCurvatureDirOcf() { return false; }
  static bool HasMarkOcf()         { return false; }
  static bool HasNormalOcf()       { return false; }
  static bool HasQualityOcf()      { return false; }
  static bool HasWedgeTexCoordOcf()    { return false; }
  static bool HasWedgeColorOcf()       { return false; }
  static bool HasWedgeNormalOcf()      { return false; }
  static bool HasFFAdjacencyOcf()      { return false; }
  static bool HasVFAdjacencyOcf()      { return false; }







  inline int Index() const {
    typename T::FaceType const *tp=static_cast<typename T::FaceType const *>(this);
    int tt2=tp- &*(_ovp->begin());
    return tt2;
  }
public:
  // ovp Optional Vector Pointer
  // Pointer to the base vector where each face element is stored.
  // used to access to the vectors of the other optional members.
  vector_ocf<typename T::FaceType> *_ovp;
};

  } // end namespace face

  template < class, class,class,class > class TriMesh;

  namespace tri
  {
  template < class FaceType >
  bool FaceVectorHasVFAdjacency(const face::vector_ocf<FaceType> &fv)
  {
    if(FaceType::HasVFAdjacencyOcf()) return fv.IsVFAdjacencyEnabled();
    else return FaceType::HasVFAdjacency();
  }
  template < class FaceType >
  bool FaceVectorHasFFAdjacency(const face::vector_ocf<FaceType> &fv)
  {
    if(FaceType::HasFFAdjacencyOcf()) return fv.IsFFAdjacencyEnabled();
    else return FaceType::HasFFAdjacency();
  }
  template < class FaceType >
  bool FaceVectorHasPerWedgeTexCoord(const face::vector_ocf<FaceType> &fv)
  {
    if(FaceType::HasWedgeTexCoordOcf()) return fv.IsWedgeTexCoordEnabled();
    else return FaceType::HasWedgeTexCoord();
  }
  template < class FaceType >
  bool FaceVectorHasPerFaceColor(const face::vector_ocf<FaceType> &fv)
  {
    if(FaceType::HasColorOcf()) return fv.IsColorEnabled();
    else return FaceType::HasColor();
  }
  template < class FaceType >
  bool FaceVectorHasPerFaceQuality(const face::vector_ocf<FaceType> &fv)
  {
    if(FaceType::HasQualityOcf()) return fv.IsQualityEnabled();
    else return FaceType::HasQuality();
  }
  template < class FaceType >
  bool FaceVectorHasPerFaceMark(const face::vector_ocf<FaceType> &fv)
  {
    if(FaceType::HasMarkOcf()) return fv.IsMarkEnabled();
    else return FaceType::HasMark();
  }
  template < class FaceType >
  bool FaceVectorHasPerFaceCurvatureDir(const face::vector_ocf<FaceType> &fv)
  {
    if(FaceType::HasCurvatureDirOcf()) return fv.IsCurvatureDirEnabled();
    else return FaceType::HasCurvatureDir();
  }
  template < class FaceType >
  bool FaceVectorHasPerFaceNormal(const face::vector_ocf<FaceType> &fv)
  {
    if(FaceType::HasNormalOcf()) return fv.IsNormalEnabled();
    else return FaceType::HasNormal();
  }
  template < class FaceType >
  void ReorderFace( std::vector<size_t>  &newFaceIndex, face::vector_ocf< FaceType > &faceVec)
  {
    faceVec.ReorderFace(newFaceIndex);
  }

  }
}// end namespace vcg
#endif
