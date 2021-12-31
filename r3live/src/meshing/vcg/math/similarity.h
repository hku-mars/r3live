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
Revision 1.16  2008/02/22 17:40:27  ponchio
Fixed determinantt problem and quaternion problem.

Revision 1.15  2008/02/21 11:34:08  ponchio
refixed bug in FromMatrix

Revision 1.14  2008/02/21 10:57:59  ponchio
fixed bug in FromMatrix

Revision 1.13  2008/02/21 10:30:18  benedetti
corrected bug in FromMatrix

Revision 1.12  2007/02/05 14:17:48  corsini
add new ctor (build similarity from euler angles)

Revision 1.11  2004/12/15 18:45:50  tommyfranken
*** empty log message ***

Revision 1.10  2004/10/07 13:55:47  ganovelli
templated on the kind of class used to implement rotation
(default is QUternion but it can be Matrix44 as well)

Revision 1.9  2004/06/04 13:35:07  cignoni
added InverseMatrix,

Revision 1.8  2004/05/07 10:09:13  cignoni
missing final newline

Revision 1.7  2004/05/04 23:23:45  cignoni
unified to the gl stlyle matix*vector. removed vector*matrix operator

Revision 1.6  2004/03/25 14:57:49  ponchio
Microerror. ($LOG$ -> $Log: not supported by cvs2svn $
Microerror. ($LOG$ -> Revision 1.16  2008/02/22 17:40:27  ponchio
Microerror. ($LOG$ -> Fixed determinantt problem and quaternion problem.
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.15  2008/02/21 11:34:08  ponchio
Microerror. ($LOG$ -> refixed bug in FromMatrix
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.14  2008/02/21 10:57:59  ponchio
Microerror. ($LOG$ -> fixed bug in FromMatrix
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.13  2008/02/21 10:30:18  benedetti
Microerror. ($LOG$ -> corrected bug in FromMatrix
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.12  2007/02/05 14:17:48  corsini
Microerror. ($LOG$ -> add new ctor (build similarity from euler angles)
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.11  2004/12/15 18:45:50  tommyfranken
Microerror. ($LOG$ -> *** empty log message ***
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.10  2004/10/07 13:55:47  ganovelli
Microerror. ($LOG$ -> templated on the kind of class used to implement rotation
Microerror. ($LOG$ -> (default is QUternion but it can be Matrix44 as well)
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.9  2004/06/04 13:35:07  cignoni
Microerror. ($LOG$ -> added InverseMatrix,
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.8  2004/05/07 10:09:13  cignoni
Microerror. ($LOG$ -> missing final newline
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.7  2004/05/04 23:23:45  cignoni
Microerror. ($LOG$ -> unified to the gl stlyle matix*vector. removed vector*matrix operator
Microerror. ($LOG$ ->


****************************************************************************/

#ifndef SIMILARITY_H
#define SIMILARITY_H

#include <vcg/math/quaternion.h>
#include <vcg/math/matrix44.h>

namespace vcg {

template <class S,class RotationType = Quaternion<S> > class Similarity {
public:
  Similarity() {}
  Similarity(const RotationType &q) { SetRotate(q); }
  Similarity(const Point3<S> &p) { SetTranslate(p); }
  Similarity(S s) { SetScale(s); }
	Similarity(S alpha, S beta, S gamma)
	{
		rot.FromEulerAngles(alpha, beta, gamma);
		tra = Point3<S>(0, 0, 0);
		sca = 1;
	}
  
  Similarity operator*(const Similarity &affine) const;
  Similarity &operator*=(const Similarity &affine);
  //Point3<S> operator*(const Point3<S> &p) const;
  
  
  Similarity &SetIdentity();
  Similarity &SetScale(const S s);
	Similarity &SetTranslate(const Point3<S> &t);	
  ///use radiants for angle.
  Similarity &SetRotate(S angle, const Point3<S> & axis); 
  Similarity &SetRotate(const RotationType &q);

  Matrix44<S> Matrix() const;
  Matrix44<S> InverseMatrix() const;
  void FromMatrix(const Matrix44<S> &m);

  RotationType rot;
  Point3<S> tra;
  S sca;  
};



template <class S,class RotationType> Similarity<S,RotationType> &Invert(Similarity<S,RotationType> &m);
template <class S,class RotationType> Similarity<S,RotationType> Inverse(const Similarity<S,RotationType> &m);
template <class S,class RotationType> Point3<S> operator*(const Similarity<S,RotationType> &m, const Point3<S> &p);


template <class S,class RotationType> Similarity<S,RotationType> Similarity<S,RotationType>::operator*(const Similarity &a) const {
  Similarity<S,RotationType> r;
  r.rot = rot * a.rot;
  r.sca = sca * a.sca;
  r.tra = (rot.Rotate(a.tra)) * sca + tra;
  return r;
}

template <class S,class RotationType> Similarity<S,RotationType> &Similarity<S,RotationType>::operator*=(const Similarity &a) {  
  rot = rot * a.rot;
  sca = sca * a.sca;
  tra = (rot.Rotate(a.tra)) * sca + tra;
  return *this;
}
  
template <class S,class RotationType> Similarity<S,RotationType> &Similarity<S,RotationType>::SetIdentity() {
  rot.SetIdentity();
  tra = Point3<S>(0, 0, 0);
  sca = 1;
  return *this;
}

template <class S,class RotationType> Similarity<S,RotationType> &Similarity<S,RotationType>::SetScale(const S s) {
  SetIdentity();
  sca = s;
  return *this;
}

template <class S,class RotationType> Similarity<S,RotationType> &Similarity<S,RotationType>::SetTranslate(const Point3<S> &t) {
  SetIdentity();
  tra = t;
  return *this;
}

template <class S,class RotationType> Similarity<S,RotationType> &Similarity<S,RotationType>::SetRotate(S angle, const Point3<S> &axis) {
  SetIdentity();
  rot.FromAxis(angle, axis);
  return *this;
}

template <class S,class RotationType> Similarity<S,RotationType> &Similarity<S,RotationType>::SetRotate(const RotationType &q) {
  SetIdentity();
  rot = q;  
  return *this;
}


template <class S,class RotationType> Matrix44<S> Similarity<S,RotationType>::Matrix() const {
  Matrix44<S> r;
  rot.ToMatrix(r);
  Matrix44<S> s = Matrix44<S>().SetScale(sca, sca, sca);
  Matrix44<S> t = Matrix44<S>().SetTranslate(tra[0], tra[1], tra[2]);
  return Matrix44<S>(s*r*t);  // trans * scale * rot;
}

template <class S,class RotationType> Matrix44<S> Similarity<S,RotationType>::InverseMatrix() const {
  return Inverse(Matrix());
}


template <class S,class RotationType> void Similarity<S,RotationType>::FromMatrix(const Matrix44<S> &m) {
 //Computes a t*s*r decomposition
  S det = m.Determinant();
  assert(det > 0);
  sca = (S)pow((S)det, (S)(1/3.0));  
  Matrix44<S> t = m*Matrix44<S>().SetScale(1/sca, 1/sca, 1/sca);
  tra[0] = t.ElementAt(0, 3);t[0][3] = 0.0;
  tra[1] = t.ElementAt(1, 3);t[1][3] = 0.0;
  tra[2] = t.ElementAt(2, 3);t[2][3] = 0.0;
  rot.FromMatrix(t);

	Invert(t);	
	tra = t * tra;
	tra/= sca;
}


template <class S,class RotationType> Similarity<S,RotationType> &Invert(Similarity<S,RotationType> &a) {  
  a.rot.Invert();
  a.sca = 1/a.sca;
  a.tra = a.rot.Rotate(-a.tra)*a.sca;
  return a;
}

template <class S,class RotationType> Similarity<S,RotationType> Inverse(const Similarity<S,RotationType> &m) {
  Similarity<S,RotationType> a = m;
  return Invert(a);
}


template <class S,class RotationType> Similarity<S,RotationType> Interpolate(const Similarity<S,RotationType> &a, const Similarity<S,RotationType> &b, const S t) {
  Similarity<S,RotationType> r;
  r.rot = Interpolate(a.rot, b.rot, t);
  r.tra = a.tra * t + b.tra * (1-t);
  r.sca = a.sca * t + b.sca * (1-t);
  return r;
}

template <class S,class RotationType> Point3<S> operator*(const Similarity<S,RotationType> &m, const Point3<S> &p) {
  Matrix44<S> t;
  m.rot.ToMatrix(t);  
  Point3<S> r = t*p;
  r *= m.sca;
  r += m.tra;
  return r;
}


//typedef Similarity<float> Similarityf;
//typedef Similarity<double>Similarityd;

class Similarityf:public Similarity<float>{};

class Similarityd:public Similarity<double>{};

} //namespace

#endif
