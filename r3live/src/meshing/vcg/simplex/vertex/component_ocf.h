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
OCF = Optional Component Fast (hopefully)
compare with OCC(Optional Component Compact)
*/
#ifndef __VCG_MESH
#error "This file should not be included alone. It is automatically included by complex.h"
#endif
#ifndef __VCG_VERTEX_PLUS_COMPONENT_OCF
#define __VCG_VERTEX_PLUS_COMPONENT_OCF
#ifndef __VCG_MESH
#error "This file should not be included alone. It is automatically included by complex.h"
#endif

namespace vcg {
	namespace vertex {
/*
All the Components that can be added to a vertex should be defined in the namespace vert:

*/
template <class VALUE_TYPE>
class vector_ocf: public std::vector<VALUE_TYPE> {
	typedef std::vector<VALUE_TYPE> BaseType;
	typedef typename vector_ocf<VALUE_TYPE>::iterator ThisTypeIterator;

public:
  vector_ocf():std::vector<VALUE_TYPE>()
  {
    ColorEnabled = false;
    CurvatureEnabled = false;
    CurvatureDirEnabled = false;
    MarkEnabled = false;
    NormalEnabled = false;
    QualityEnabled = false;
    RadiusEnabled = false;
    TexCoordEnabled = false;
    VFAdjacencyEnabled = false;
  }

////////////////////////////////////////
// All the standard methods of std::vector that can change the reallocation are
// redefined in order to manage the additional data.
  void push_back(const VALUE_TYPE & v)
    {
        BaseType::push_back(v);
        BaseType::back()._ovp = this;
        if (ColorEnabled)         CV.push_back(vcg::Color4b(vcg::Color4b::White));
        if (QualityEnabled)       QV.push_back(0);
        if (MarkEnabled)          MV.push_back(0);
        if (NormalEnabled)        NV.push_back(typename VALUE_TYPE::NormalType());
        if (TexCoordEnabled)      TV.push_back(typename VALUE_TYPE::TexCoordType());
        if (VFAdjacencyEnabled)   AV.push_back(VFAdjType());
        if (CurvatureEnabled)     CuV.push_back(typename VALUE_TYPE::CurvatureType());
        if (CurvatureDirEnabled)  CuDV.push_back(typename VALUE_TYPE::CurvatureDirType());
        if (RadiusEnabled)        RadiusV.push_back(typename VALUE_TYPE::RadiusType());
    }

    void pop_back();

	void resize(const unsigned int & _size)
	{
		const unsigned int oldsize = BaseType::size();
			BaseType::resize(_size);
		if(oldsize<_size){
			ThisTypeIterator firstnew = BaseType::begin();
			advance(firstnew,oldsize);
			_updateOVP(firstnew,(*this).end());
		}
		if (ColorEnabled)         CV.resize(_size);
		if (QualityEnabled)       QV.resize(_size,0);
		if (MarkEnabled)          MV.resize(_size);
		if (NormalEnabled)        NV.resize(_size);
		if (TexCoordEnabled)      TV.resize(_size);
		if (VFAdjacencyEnabled)   AV.resize(_size);
		if (CurvatureEnabled)     CuV.resize(_size);
		if (CurvatureDirEnabled)  CuDV.resize(_size);
		if (RadiusEnabled)        RadiusV.resize(_size);
	}

	void reserve(const unsigned int & _size)
	{
		BaseType::reserve(_size);
		if (ColorEnabled)        CV.reserve(_size);
		if (QualityEnabled)      QV.reserve(_size);
		if (MarkEnabled)         MV.reserve(_size);
		if (NormalEnabled)       NV.reserve(_size);
		if (TexCoordEnabled)     TV.reserve(_size);
		if (VFAdjacencyEnabled)  AV.reserve(_size);
		if (CurvatureEnabled)    CuV.reserve(_size);
		if (CurvatureDirEnabled) CuDV.reserve(_size);
		if (RadiusEnabled)       RadiusV.reserve(_size);
	}

	void _updateOVP(ThisTypeIterator lbegin, ThisTypeIterator lend)
	{
		ThisTypeIterator vi;
		for(vi=lbegin;vi!=lend;++vi)
				(*vi)._ovp=this;
	}

////////////////////////////////////////
// Enabling Eunctions

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

bool IsColorEnabled() const {return ColorEnabled;}
void EnableColor() {
	assert(VALUE_TYPE::HasColorOcf());
	ColorEnabled=true;
	CV.resize((*this).size());
}
void DisableColor() {
	assert(VALUE_TYPE::HasColorOcf());
	ColorEnabled=false;
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

bool IsVFAdjacencyEnabled() const {return VFAdjacencyEnabled;}
void EnableVFAdjacency() {
	assert(VALUE_TYPE::HasVFAdjacencyOcf());
	VFAdjacencyEnabled=true;
	VFAdjType zero; zero._fp=0; zero._zp=-1;
	AV.resize((*this).size(),zero);
}
void DisableVFAdjacency() {
	assert(VALUE_TYPE::HasVFAdjacencyOcf());
	VFAdjacencyEnabled=false;
	AV.clear();
}

bool IsCurvatureEnabled() const {return CurvatureEnabled;}
void EnableCurvature() {
	assert(VALUE_TYPE::HasCurvatureOcf());
	CurvatureEnabled=true;
	CuV.resize((*this).size());
}
void DisableCurvature() {
	assert(VALUE_TYPE::HasCurvatureOcf());
	CurvatureEnabled=false;
	CuV.clear();
}

bool IsCurvatureDirEnabled() const {return CurvatureDirEnabled;}
void EnableCurvatureDir() {
	assert(VALUE_TYPE::HasCurvatureDirOcf());
	CurvatureDirEnabled=true;
	CuDV.resize((*this).size());
}
void DisableCurvatureDir() {
	assert(VALUE_TYPE::HasCurvatureDirOcf());
	CurvatureDirEnabled=false;
	CuDV.clear();
}

bool IsRadiusEnabled() const {return RadiusEnabled;}
void EnableRadius() {
	assert(VALUE_TYPE::HasRadiusOcf());
	RadiusEnabled=true;
	RadiusV.resize((*this).size());
}
void DisableRadius() {
	assert(VALUE_TYPE::HasRadiusOcf());
	RadiusEnabled=false;
	RadiusV.clear();
}


bool IsTexCoordEnabled() const {return TexCoordEnabled;}
void EnableTexCoord() {
	assert(VALUE_TYPE::HasTexCoordOcf());
	TexCoordEnabled=true;
	TV.resize((*this).size());
}
void DisableTexCoord() {
	assert(VALUE_TYPE::HasTexCoordOcf());
	TexCoordEnabled=false;
	TV.clear();
}

struct VFAdjType {
	typename VALUE_TYPE::FacePointer _fp ;
	int _zp ;
	};

public:
  std::vector<typename VALUE_TYPE::ColorType> CV;
  std::vector<typename VALUE_TYPE::CurvatureType> CuV;
  std::vector<typename VALUE_TYPE::CurvatureDirType> CuDV;
  std::vector<int> MV;
  std::vector<typename VALUE_TYPE::NormalType> NV;
  std::vector<typename VALUE_TYPE::QualityType> QV;
  std::vector<typename VALUE_TYPE::RadiusType> RadiusV;
  std::vector<typename VALUE_TYPE::TexCoordType> TV;
  std::vector<struct VFAdjType> AV;

  bool ColorEnabled;
  bool CurvatureEnabled;
  bool CurvatureDirEnabled;
  bool MarkEnabled;
  bool NormalEnabled;
  bool QualityEnabled;
  bool RadiusEnabled;
  bool TexCoordEnabled;
  bool VFAdjacencyEnabled;
};


//template<>	void EnableAttribute<typename VALUE_TYPE::NormalType>(){	NormalEnabled=true;}

/*------------------------- COORD -----------------------------------------*/
/*----------------------------- VFADJ ------------------------------*/


template <class T> class VFAdjOcf: public T {
public:
	typename T::FacePointer &VFp()       {
		assert((*this).Base().VFAdjacencyEnabled);
		return (*this).Base().AV[(*this).Index()]._fp;
	}
	typename T::FacePointer cVFp() const {
		if(! (*this).Base().VFAdjacencyEnabled ) return 0;
		else return (*this).Base().AV[(*this).Index()]._fp;
	}

    int &VFi()       {
        assert((*this).Base().VFAdjacencyEnabled);
        return (*this).Base().AV[(*this).Index()]._zp;
    }
    int cVFi() const {
        if(! (*this).Base().VFAdjacencyEnabled ) return -1;
        return (*this).Base().AV[(*this).Index()]._zp;
    }
    template <class RightVertexType>
    void ImportData(const RightVertexType & rightV)
    {
        T::ImportData(rightV);
    }

  static bool HasVFAdjacency()   {   return true; }
  static bool HasVFAdjacencyOcf()   {   return true; }
  bool IsVFAdjacencyEnabled(const typename T::VertexType *vp)   {return vp->Base().VFAdjacencyEnabled;}

   static void Name(std::vector<std::string> & name){name.push_back(std::string("VFAdjOcf"));T::Name(name);}
private:
};

/*------------------------- Normal -----------------------------------------*/

template <class A, class T> class NormalOcf: public T {
public:
  typedef A NormalType;
  inline bool IsNormalEnabled( )       const  { return this->Base().IsNormalEnabled(); }
  static bool HasNormal()   { return true; }
  static bool HasNormalOcf()   { return true; }

  const NormalType &N() const { assert((*this).Base().NormalEnabled); return (*this).Base().NV[(*this).Index()];  }
        NormalType &N()       { assert((*this).Base().NormalEnabled); return (*this).Base().NV[(*this).Index()];  }
        NormalType cN() const { assert((*this).Base().NormalEnabled); return (*this).Base().NV[(*this).Index()];  }

  template <class RightVertexType>
  void ImportData(const RightVertexType & rightV){
    if((*this).IsNormalEnabled() && rightV.IsNormalEnabled() )
      N().Import(rightV.cN());
    T::ImportData(rightV);}
};

template <class T> class Normal3sOcf: public NormalOcf<vcg::Point3s, T> {public: static void Name(std::vector<std::string> & name){name.push_back(std::string("Normal3sOcf"));T::Name(name);}};
template <class T> class Normal3fOcf: public NormalOcf<vcg::Point3f, T> {public: static void Name(std::vector<std::string> & name){name.push_back(std::string("Normal3fOcf"));T::Name(name);}};
template <class T> class Normal3dOcf: public NormalOcf<vcg::Point3d, T> {public: static void Name(std::vector<std::string> & name){name.push_back(std::string("Normal3dOcf"));T::Name(name);}};

///*-------------------------- COLOR ----------------------------------*/

template <class A, class T> class ColorOcf: public T {
public:
  typedef A ColorType;
  const ColorType &C() const { assert((*this).Base().ColorEnabled); return (*this).Base().CV[(*this).Index()]; }
        ColorType &C()       { assert((*this).Base().ColorEnabled); return (*this).Base().CV[(*this).Index()]; }
        ColorType cC() const { assert((*this).Base().ColorEnabled); return (*this).Base().CV[(*this).Index()]; }
  template <class RightVertexType>
  void ImportData(const RightVertexType & rightV)
  {
    if((*this).IsColorEnabled() && rightV.IsColorEnabled() )
      C() = rightV.cC();
    T::ImportData(rightV);
  }

  inline bool IsColorEnabled()         const  { return this->Base().IsColorEnabled();}
  static bool HasColor()   { return true; }
  static bool HasColorOcf()   { assert(!T::HasColorOcf()); return true; }
};

template <class T> class Color4bOcf: public ColorOcf<vcg::Color4b, T> {
public: static void Name(std::vector<std::string> & name){name.push_back(std::string("Color4bOcf"));T::Name(name);}
};

///*-------------------------- QUALITY  ----------------------------------*/

template <class A, class T> class QualityOcf: public T {
public:
  typedef A QualityType;
  const QualityType &Q() const { assert((*this).Base().QualityEnabled); return (*this).Base().QV[(*this).Index()]; }
        QualityType &Q()       { assert((*this).Base().QualityEnabled); return (*this).Base().QV[(*this).Index()]; }
        QualityType cQ() const { assert((*this).Base().QualityEnabled); return (*this).Base().QV[(*this).Index()]; }
  template <class RightVertexType>
  void ImportData(const RightVertexType & rightV)
  {
    if((*this).IsQualityEnabled() && rightV.IsQualityEnabled() ) // copy the data only if they are enabled in both vertices
      Q() = rightV.cQ();
    T::ImportData(rightV);
  }
  inline bool IsQualityEnabled( )      const  { return this->Base().IsQualityEnabled(); }
  static bool HasQuality()   { return true; }
  static bool HasQualityOcf()   { assert(!T::HasQualityOcf()); return true; }
};

template <class T> class QualityfOcf: public QualityOcf<float, T> {
public: static void Name(std::vector<std::string> & name){name.push_back(std::string("QualityfOcf"));T::Name(name);}
};


///*-------------------------- TEXTURE  ----------------------------------*/

template <class A, class TT> class TexCoordOcf: public TT {
public:
  typedef A TexCoordType;
  const TexCoordType &T() const { assert((*this).Base().TexCoordEnabled); return (*this).Base().TV[(*this).Index()]; }
        TexCoordType &T()       { assert((*this).Base().TexCoordEnabled); return (*this).Base().TV[(*this).Index()]; }
        TexCoordType cT() const { assert((*this).Base().TexCoordEnabled); return (*this).Base().TV[(*this).Index()]; }
  template < class RightVertexType>
  void ImportData(const RightVertexType & rightV)
  {
    if((*this).IsTexCoordEnabled() && rightV.IsTexCoordEnabled()) // copy the data only if they are enabled in both vertices
      T() = rightV.cT();
    TT::ImportData(rightV);
  }
  inline bool IsTexCoordEnabled( )     const  { return this->Base().IsTexCoordEnabled(); }
  static bool HasTexCoord()   { return true; }
  static bool HasTexCoordOcf()   { assert(!TT::HasTexCoordOcf()); return true; }
};

template <class T> class TexCoordfOcf: public TexCoordOcf<TexCoord2<float,1>, T> {
public: static void Name(std::vector<std::string> & name){name.push_back(std::string("TexCoordfOcf"));T::Name(name);}
};

///*-------------------------- MARK  ----------------------------------*/

template <class T> class MarkOcf: public T {
public:
  typedef int MarkType;
  inline const int &IMark() const { assert((*this).Base().MarkEnabled);  return (*this).Base().MV[(*this).Index()];   }
  inline       int &IMark()       { assert((*this).Base().MarkEnabled);  return (*this).Base().MV[(*this).Index()];   }
  inline       int cIMark() const {  assert((*this).Base().MarkEnabled); return (*this).Base().MV[(*this).Index()]; }

  template <class RightVertexType>
  void ImportData(const RightVertexType & rightV)
  {
    if((*this).IsMarkEnabled() && rightV.IsMarkEnabled()) // copy the data only if they are enabled in both vertices
      IMark() = rightV.cIMark();
    T::ImportData(rightV);
  }
  inline bool IsMarkEnabled( )         const  { return this->Base().IsMarkEnabled(); }
  static bool HasMark()   { return true; }
  static bool HasMarkOcf()   { return true; }
  inline void InitIMark()    { IMark() = 0; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("IMark"));T::Name(name);}

};


///*-------------------------- CURVATURE  ----------------------------------*/

template <class A, class TT> class CurvatureOcf: public TT {
public:
  typedef Point2<A> CurvatureType;
  typedef typename CurvatureType::ScalarType ScalarType;

  ScalarType &Kh(){  assert((*this).Base().CurvatureEnabled); return (*this).Base().CuV[(*this).Index()][0];}
  ScalarType &Kg(){  assert((*this).Base().CurvatureEnabled); return (*this).Base().CuV[(*this).Index()][1];}
  ScalarType cKh() const { assert((*this).Base().CurvatureEnabled); return (*this).Base().CuV[(*this).Index()][0];}
  ScalarType cKg() const { assert((*this).Base().CurvatureEnabled); return (*this).Base().CuV[(*this).Index()][1];}

  template <class RightVertexType>
  void ImportData(const RightVertexType & rightV){
    if((*this).IsCurvatureEnabled() && rightV.IsCurvatureEnabled())
    {
      (*this).Base().CuV[(*this).Index()][0] = rightV.cKh();
      (*this).Base().CuV[(*this).Index()][1] = rightV.cKg();
    }
    TT::ImportData(rightV);
  }

  inline bool IsCurvatureEnabled( )    const  { return this->Base().IsCurvatureDirEnabled(); }
  static bool HasCurvature() { return true; }
  static bool HasCurvatureOcf()   { return true; }
};

template <class T> class CurvaturefOcf: public CurvatureOcf<float, T> {public: static void Name(std::vector<std::string> & name){name.push_back(std::string("CurvaturefOcf"));T::Name(name);} };
template <class T> class CurvaturedOcf: public CurvatureOcf<double, T> {public: static void Name(std::vector<std::string> & name){name.push_back(std::string("CurvaturedOcf"));T::Name(name);} };


///*-------------------------- CURVATURE DIR ----------------------------------*/

template <class S>
struct CurvatureDirTypeOcf{
	typedef Point3<S> VecType;
	typedef  S   ScalarType;
	CurvatureDirTypeOcf () {}
	Point3<S>max_dir,min_dir;
	S k1,k2;
};


template <class A, class TT> class CurvatureDirOcf: public TT {
public:
  typedef A CurvatureDirType;
  typedef typename CurvatureDirType::VecType VecType;
  typedef typename CurvatureDirType::ScalarType ScalarType;

  VecType &PD1()       { assert((*this).Base().CurvatureDirEnabled); return (*this).Base().CuDV[(*this).Index()].max_dir;}
  VecType &PD2()       { assert((*this).Base().CurvatureDirEnabled); return (*this).Base().CuDV[(*this).Index()].min_dir;}
  VecType cPD1() const { assert((*this).Base().CurvatureDirEnabled); return (*this).Base().CuDV[(*this).Index()].max_dir;}
  VecType cPD2() const { assert((*this).Base().CurvatureDirEnabled); return (*this).Base().CuDV[(*this).Index()].min_dir;}

  ScalarType &K1()       { assert((*this).Base().CurvatureDirEnabled); return (*this).Base().CuDV[(*this).Index()].k1;}
  ScalarType &K2()       { assert((*this).Base().CurvatureDirEnabled); return (*this).Base().CuDV[(*this).Index()].k2;}
  ScalarType cK1() const { assert((*this).Base().CurvatureDirEnabled); return (*this).Base().CuDV[(*this).Index()].k1;}
  ScalarType cK2() const { assert((*this).Base().CurvatureDirEnabled); return (*this).Base().CuDV[(*this).Index()].k2;}

  template <class RightVertexType>
  void ImportData(const RightVertexType & rightV){
    if((*this).IsCurvatureDirEnabled() && rightV.IsCurvatureDirEnabled())
    {
      (*this).PD1() = rightV.cPD1();
      (*this).PD2() = rightV.cPD2();
      (*this).K1() = rightV.cK1();
      (*this).K2() = rightV.cK2();
    }
    TT::ImportData(rightV);
  }

  inline bool IsCurvatureDirEnabled( ) const  { return this->Base().IsCurvatureDirEnabled(); }
  static bool HasCurvatureDir()  { return true; }
  static bool HasCurvatureDirOcf()  { return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("CurvatureDirOcf"));TT::Name(name);}
  };

template <class T> class CurvatureDirfOcf: public CurvatureDirOcf<CurvatureDirTypeOcf<float>, T> {
public:	static void Name(std::vector<std::string> & name){name.push_back(std::string("CurvatureDirfOcf"));T::Name(name);}
};
template <class T> class CurvatureDirdOcf: public CurvatureDirOcf<CurvatureDirTypeOcf<double>, T> {
public:	static void Name(std::vector<std::string> & name){name.push_back(std::string("CurvatureDirdOcf"));T::Name(name);}
};


///*-------------------------- RADIUS  ----------------------------------*/

template <class A, class TT> class RadiusOcf: public TT {
public:
  typedef A RadiusType;
  typedef RadiusType ScalarType;

  const RadiusType &R() const { assert((*this).Base().RadiusEnabled); return (*this).Base().RadiusV[(*this).Index()];}
        RadiusType &R()       { assert((*this).Base().RadiusEnabled); return (*this).Base().RadiusV[(*this).Index()];}
        RadiusType cR() const { assert((*this).Base().RadiusEnabled); return (*this).Base().RadiusV[(*this).Index()];}

  template <class RightVertexType>
  void ImportData(const RightVertexType & rightV)
  {
    if ((*this).IsRadiusEnabled() && rightV.IsRadiusEnabled())
      (*this).Base().RadiusV[(*this).Index()] = rightV.cR();
    TT::ImportData(rightV);
  }

  inline bool IsRadiusEnabled( )       const  { return this->Base().IsRadiusEnabled(); }
  static bool HasRadius()     { return true; }
  static bool HasRadiusOcf()  { return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("RadiusOcf")); TT::Name(name);}
};

template <class T> class RadiusfOcf: public RadiusOcf<float, T> {};
template <class T> class RadiusdOcf: public RadiusOcf<double, T> {};


///*-------------------------- InfoOpt  ----------------------------------*/

template < class T> class InfoOcf: public T {
public:
	// You should never ever try to copy a vertex that has OCF stuff.
		// use ImportData function.
	inline InfoOcf &operator=(const InfoOcf & /*other*/) {
		assert(0); return *this;
	}

		vector_ocf<typename T::VertexType> &Base() const { return *_ovp;}

	inline int Index() const {
		typename  T::VertexType const *tp=static_cast<typename T::VertexType const*>(this);
		int tt2=tp- &*(_ovp->begin());
		return tt2;
	}
public:
	vector_ocf<typename T::VertexType> *_ovp;

	static bool HasColorOcf()   { return false; }
	static bool HasCurvatureOcf()   { return false; }
	static bool HasCurvatureDirOcf()   { return false; }
	static bool HasNormalOcf()   { return false; }
	static bool HasMarkOcf()   { return false; }
	static bool HasQualityOcf()   { return false; }
	static bool HasRadiusOcf()   { return false; }
	static bool HasTexCoordOcf()   { return false; }
	static bool HasVFAdjacencyOcf()   { return false; }
};


} // end namespace vert


namespace tri
{
template < class VertexType >
bool VertexVectorHasVFAdjacency(const vertex::vector_ocf<VertexType> &fv)
{
  if(VertexType::HasVFAdjacencyOcf()) return fv.IsVFAdjacencyEnabled();
  else return VertexType::HasVFAdjacency();
}
template < class VertexType >
bool VertexVectorHasPerVertexRadius(const vertex::vector_ocf<VertexType> &fv)
{
	if(VertexType::HasRadiusOcf()) return fv.IsRadiusEnabled();
	else return VertexType::HasRadius();
}
template < class VertexType >
bool VertexVectorHasPerVertexQuality(const vertex::vector_ocf<VertexType> &fv)
{
	if(VertexType::HasQualityOcf()) return fv.IsQualityEnabled();
	else return VertexType::HasQuality();
}
template < class VertexType >
bool VertexVectorHasPerVertexNormal(const vertex::vector_ocf<VertexType> &fv)
{
	if(VertexType::HasNormalOcf()) return fv.IsNormalEnabled();
	else return VertexType::HasNormal();
}
template < class VertexType >
bool VertexVectorHasPerVertexColor(const vertex::vector_ocf<VertexType> &fv)
{
	if(VertexType::HasColorOcf()) return fv.IsColorEnabled();
	else return VertexType::HasColor();
}
template < class VertexType >
bool VertexVectorHasPerVertexCurvature(const vertex::vector_ocf<VertexType> &fv)
{
	if(VertexType::HasCurvatureOcf()) return fv.IsCurvatureEnabled();
	else return VertexType::HasCurvature();
}
template < class VertexType >
bool VertexVectorHasPerVertexCurvatureDir(const vertex::vector_ocf<VertexType> &fv)
{
	if(VertexType::HasCurvatureDirOcf()) return fv.IsCurvatureDirEnabled();
	else return VertexType::HasCurvatureDir();
}

template < class VertexType >
bool VertexVectorHasPerVertexTexCoord(const vertex::vector_ocf<VertexType> &fv)
{
	if(VertexType::HasTexCoordOcf()) return fv.IsTexCoordEnabled();
	else return VertexType::HasTexCoord();
}
}
}// end namespace vcg
#endif
