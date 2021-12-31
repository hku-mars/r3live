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
Revision 1.2  2007/03/12 15:37:21  tarini
Texture coord name change!  "TCoord" and "Texture" are BAD. "TexCoord" is GOOD.

Revision 1.1  2005/10/15 16:24:10  ganovelli
Working release (compilata solo su MSVC), component_occ � migrato da component_opt



****************************************************************************/
#ifndef __VCG_MESH
#error "This file should not be included alone. It is automatically included by complex.h"
#endif
#ifndef __VCG_VERTEX_PLUS_COMPONENT_OCC
#define __VCG_VERTEX_PLUS_COMPONENT_OCC

namespace vcg {
  namespace vertex {
/*
Some naming Rules
All the Components that can be added to a vertex should be defined in the namespace vert:

*/

/*------------------------- COORD -----------------------------------------*/ 

template <class A, class T> class CoordOcc: public T {
public:
  typedef A CoordType;
  typedef typename CoordType::ScalarType      ScalarType;
	typedef typename T::VertType VertType;
	CoordType &P() { return CAT< vector_occ<VertType>,CoordType>::Instance()->Get((VertType*)this); }
};
template <class T> class Coord3fOcc: public CoordOcc<vcg::Point3f, T> {};
template <class T> class Coord3dOcc: public CoordOcc<vcg::Point3d, T> {};


/*-------------------------- NORMAL ----------------------------------------*/ 

template <class A, class T> class NormalOcc: public T {
public:
  typedef A NormalType;
	typedef typename T::VertType VertType;
  NormalType &N() {return CAT< vector_occ<VertType>,NormalType>::Instance()->Get((VertType*)this); }
/*private:
  NormalType _norm;   */ 
};

template <class T> class Normal3sOcc: public NormalOcc<vcg::Point3s, T> {};
template <class T> class Normal3fOcc: public NormalOcc<vcg::Point3f, T> {};
template <class T> class Normal3dOcc: public NormalOcc<vcg::Point3d, T> {};

/*-------------------------- TEXCOORD ----------------------------------------*/ 

template <class A, class TT> class TexCoordOcc: public TT {
public:
  typedef A TexCoordType;
	typedef typename TT::VertType VertType;
  TexCoordType &T() {return CAT< vector_occ<VertType>,TexCoordType>::Instance()->Get((VertType*)this); }
  static bool HasTexCoord()   { return true; }
  static bool HasTexCoordOcc()   { return true; }

/* private:
  TexCoordType _t;   */ 
};

template <class T> class TexCoord2sOcc: public TexCoordOcc<TexCoord2<short,1>, T> {};
template <class T> class TexCoord2fOcc: public TexCoordOcc<TexCoord2<float,1>, T> {};
template <class T> class TexCoord2dOcc: public TexCoordOcc<TexCoord2<double,1>, T> {};

///*------------------------- FLAGS -----------------------------------------*/ 

template <class T> class FlagOcc:  public T {
public:
	typedef typename T::VertType VertType;
   int &Flags() {return CAT< vector_occ<VertType>,int>::Instance()->Get((VertType*)this); }
   const int Flags() const {return CAT< vector_occ<VertType>,int>::Instance()->Get((VertType*)this); }
	static bool HasFlags() {return true;}	
	static bool HasFlagsOcc() {return true;}	


};

///*-------------------------- COLOR ----------------------------------*/ 

template <class A, class T> class ColorOcc: public T {
public:
  typedef A ColorType;
	typedef typename T::VertType VertType;
  ColorType &C() { return CAT< vector_occ<VertType>,ColorType>::Instance()->Get((VertType*)this); }
  static bool HasColor()   { return true; }
/*private:
  ColorType _color;   */ 
};

template <class T> class Color4bOcc: public ColorOcc<vcg::Color4b, T> {};

///*-------------------------- Quality  ----------------------------------*/ 

template <class A, class T> class QualityOcc: public T {
public:
  typedef A QualityType;
	typedef typename T::VertType VertType;
  QualityType &Q() { return CAT< vector_occ<VertType>,QualityType>::Instance()->Get((VertType*)this);}
  static bool HasQuality()   { return true; }

/*private:
  QualityType _quality;  */  
};

template <class T> class QualitysOcc: public QualityOcc<short, T> {};
template <class T> class QualityfOcc: public QualityOcc<float, T> {};
template <class T> class QualitydOcc: public QualityOcc<double, T> {};
//

///*-------------------------- Curvature  ----------------------------------*/ 

template <class A, class TT> class CurvatureOcc: public TT {
public:
  typedef Point2<A> CurvatureTypeOcc;
	typedef typename TT::VertType VertType;
	typedef typename CurvatureTypeOcc::ScalarType ScalarType;

	ScalarType  &H(){  return CAT< vector_occ<VertType>,CurvatureTypeOcc>::Instance()->Get((VertType*)this)[0];}
	ScalarType  &K(){  return CAT< vector_occ<VertType>,CurvatureTypeOcc>::Instance()->Get((VertType*)this)[1];}
	const ScalarType &cH() const { return CAT< vector_occ<VertType>,CurvatureTypeOcc>::Instance()->Get((VertType*)this)[0];}
	const ScalarType &cK() const { return CAT< vector_occ<VertType>,CurvatureTypeOcc>::Instance()->Get((VertType*)this)[1];}

 	template <class LeftV>
	void ImportData(const LeftV & leftV){
			CAT< vector_occ<VertType>,CurvatureTypeOcc>::Instance()->Get((VertType*)this)[0] = leftV.cH();
			CAT< vector_occ<VertType>,CurvatureTypeOcc>::Instance()->Get((VertType*)this)[1] = leftV.cK();
			TT::ImporLocal(leftV);
	}

	static bool HasCurvature()   { return true; }
	static bool HasCurvatureOcc()   { return true; }
	static void Name(std::vector<std::string> & name){name.push_back(std::string("CurvatureOcc"));TT::Name(name);}

private:   
};

template <class T> class CurvaturefOcc: public CurvatureOcc<float, T> {
	static void Name(std::vector<std::string> & name){name.push_back(std::string("CurvaturefOcc"));T::Name(name);}
};
template <class T> class CurvaturedOcc: public CurvatureOcc<double, T> {
	static void Name(std::vector<std::string> & name){name.push_back(std::string("CurvaturedOcc"));T::Name(name);}
};


/*-------------------------- Curvature Direction ----------------------------------*/ 

template <class S>
struct CurvatureDirTypeOcc{
	typedef Point3<S> VecType;
	typedef  S   ScalarType;
	CurvatureDirTypeOcc () {}
	Point3<S>max_dir,min_dir; // max and min curvature direction 
	S k1,k2;// max and min curvature values
};


template <class A, class TT> class CurvatureDirOcc: public TT {
public:
  typedef A CurvatureDirTypeOcc;
	typedef typename CurvatureDirTypeOcc::VecType VecType;
	typedef typename CurvatureDirTypeOcc::ScalarType ScalarType;
	typedef typename TT::VertType VertType;

	VecType &PD1(){ return CAT< vector_occ<VertType>,CurvatureDirTypeOcc>::Instance()->Get((VertType*)this).max_dir;}
	VecType &PD2(){ return CAT< vector_occ<VertType>,CurvatureDirTypeOcc>::Instance()->Get((VertType*)this).min_dir;}
	const VecType &cPD1() const {return CAT< vector_occ<VertType>,CurvatureDirTypeOcc>::Instance()->Get((VertType*)this).max_dir;}
	const VecType &cPD2() const {return CAT< vector_occ<VertType>,CurvatureDirTypeOcc>::Instance()->Get((VertType*)this).min_dir;}

	ScalarType &K1(){ return CAT< vector_occ<VertType>,CurvatureDirTypeOcc>::Instance()->Get((VertType*)this).k1;}
	ScalarType &K2(){ return CAT< vector_occ<VertType>,CurvatureDirTypeOcc>::Instance()->Get((VertType*)this).k2;}
	const ScalarType &cK1() const {return CAT< vector_occ<VertType>,CurvatureDirTypeOcc>::Instance()->Get((VertType*)this).k1;}
	const ScalarType &cK2()const  {return CAT< vector_occ<VertType>,CurvatureDirTypeOcc>::Instance()->Get((VertType*)this).k2;}

  static bool HasCurvatureDir()   { return true; }
  static bool HasCurvatureDirOcc()   { return true; }
	static void Name(std::vector<std::string> & name){name.push_back(std::string("CurvatureDir"));TT::Name(name);}

};


template <class T> class CurvatureDirfOcc: public CurvatureDirOcc<CurvatureDirTypeOcc<float>, T> {
public:	static void Name(std::vector<std::string> & name){name.push_back(std::string("CurvatureDirf"));T::Name(name);}
};
template <class T> class CurvatureDirdOcc: public CurvatureDirOcc<CurvatureDirTypeOcc<double>, T> {
public:	static void Name(std::vector<std::string> & name){name.push_back(std::string("CurvatureDird"));T::Name(name);}
};

/*-------------------------- RADIUS ----------------------------------*/

template <class A, class TT> class RadiusOcc: public TT {
public:
  typedef A RadiusType;
  typedef A ScalarType;
  typedef typename TT::VertType VertType;

  RadiusType  &R(){  return CAT< vector_occ<VertType>,RadiusType>::Instance()->Get((VertType*)this);}
  const RadiusType &cR() const { return CAT< vector_occ<VertType>,RadiusType>::Instance()->Get((VertType*)this);}

  template <class LeftV>
	void ImportData(const LeftV & leftV){
    CAT< vector_occ<VertType>,RadiusType>::Instance()->Get((VertType*)this) = leftV.cR();
    TT::ImporLocal(leftV);
  }

  static bool HasRadius()     { return true; }
  static bool HasRadiusOcc()  { return true; }
  static void Name(std::vector<std::string> & name){name.push_back(std::string("RadiusOcc"));TT::Name(name);}

private:   
};

template <class T> class RadiusfOcc: public RadiusOcc<float, T> {
  static void Name(std::vector<std::string> & name){name.push_back(std::string("RadiusfOcc"));T::Name(name);}
};
template <class T> class RadiusdOcc: public RadiusOcc<double, T> {
  static void Name(std::vector<std::string> & name){name.push_back(std::string("RadiusdOcc"));T::Name(name);}
};

///*----------------------------- VFADJ ------------------------------*/ 

template <class T> class VFAdjOcc: public T {
public:
	typedef typename T::VertType VertType;
	typedef typename T::FacePointer FacePointer;	
 FacePointer &Fp() {return CAT< vector_occ<VertType>,FacePointer>::Instance()->Get((VertType*)this); }
  int &Zp() {return _zp; }
  static bool HasVFAdjacency()   {   return true; }
private:
  typename T::FacePointer _fp ;    
  int _zp ;    
};

  } // end namespace vert
}// end namespace vcg
#endif
