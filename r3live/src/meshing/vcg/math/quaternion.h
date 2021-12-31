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
Revision 1.18  2007/07/03 16:07:09  corsini
add DCM to Euler Angles method (to implement)

Revision 1.17  2007/02/06 12:24:07  tarini
added a missing "Quaternion<S>::" in "FromEulerAngles"

Revision 1.16  2007/02/05 13:55:21  corsini
add euler angle to quaternion conversion

Revision 1.15  2006/06/22 08:00:26  ganovelli
toMatrix with matrix33 added

Revision 1.14  2005/04/17 21:57:03  ganovelli
tolto il const a interpolate

Revision 1.13  2005/04/15 09:19:50  ponchio
Typo: Point3 -> Point4

Revision 1.12  2005/04/14 17:22:34  ponchio
*** empty log message ***

Revision 1.11  2005/04/14 11:35:09  ponchio
*** empty log message ***

Revision 1.10  2004/12/15 18:45:50  tommyfranken
*** empty log message ***

Revision 1.9  2004/10/22 14:35:42  ponchio
m.element(x, y) -> m[x][y]

Revision 1.8  2004/10/07 13:54:03  ganovelli
added SetIdentity

Revision 1.7  2004/04/07 10:48:37  cignoni
updated access to matrix44 elements through V() instead simple []

Revision 1.6  2004/03/25 14:57:49  ponchio
Microerror. ($LOG$ -> $Log: not supported by cvs2svn $
Microerror. ($LOG$ -> Revision 1.18  2007/07/03 16:07:09  corsini
Microerror. ($LOG$ -> add DCM to Euler Angles method (to implement)
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.17  2007/02/06 12:24:07  tarini
Microerror. ($LOG$ -> added a missing "Quaternion<S>::" in "FromEulerAngles"
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.16  2007/02/05 13:55:21  corsini
Microerror. ($LOG$ -> add euler angle to quaternion conversion
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.15  2006/06/22 08:00:26  ganovelli
Microerror. ($LOG$ -> toMatrix with matrix33 added
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.14  2005/04/17 21:57:03  ganovelli
Microerror. ($LOG$ -> tolto il const a interpolate
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.13  2005/04/15 09:19:50  ponchio
Microerror. ($LOG$ -> Typo: Point3 -> Point4
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.12  2005/04/14 17:22:34  ponchio
Microerror. ($LOG$ -> *** empty log message ***
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.11  2005/04/14 11:35:09  ponchio
Microerror. ($LOG$ -> *** empty log message ***
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.10  2004/12/15 18:45:50  tommyfranken
Microerror. ($LOG$ -> *** empty log message ***
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.9  2004/10/22 14:35:42  ponchio
Microerror. ($LOG$ -> m.element(x, y) -> m[x][y]
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.8  2004/10/07 13:54:03  ganovelli
Microerror. ($LOG$ -> added SetIdentity
Microerror. ($LOG$ ->
Microerror. ($LOG$ -> Revision 1.7  2004/04/07 10:48:37  cignoni
Microerror. ($LOG$ -> updated access to matrix44 elements through V() instead simple []
Microerror. ($LOG$ ->


****************************************************************************/


#ifndef QUATERNION_H
#define QUATERNION_H

#include <vcg/space/point3.h>
#include <vcg/space/point4.h>
#include <vcg/math/base.h>
#include <vcg/math/matrix44.h>
#include <vcg/math/matrix33.h>

namespace vcg {

	/** Class quaternion.
	    A quaternion is a point in the unit sphere in four dimension: all
			rotations in three-dimensional space can be represented by a quaternion.
		*/
template<class S> class Quaternion: public Point4<S> {
public:

	Quaternion() {}
	Quaternion(const S v0, const S v1, const S v2, const S v3): Point4<S>(v0,v1,v2,v3){}	
	Quaternion(const Point4<S> p) : Point4<S>(p)	{}
  Quaternion(const S phi, const Point3<S> &a);

  Quaternion operator*(const S &s) const;
  //Quaternion &operator*=(S d);
  Quaternion operator*(const Quaternion &q) const;
  Quaternion &operator*=(const Quaternion &q);
  void Invert();
	Quaternion<S> Inverse() const;
  
	
	void SetIdentity();

  void FromAxis(const S phi, const Point3<S> &a);
  void ToAxis(S &phi, Point3<S> &a ) const;

  ///warning m must be a rotation matrix, result is unpredictable otherwise
  void FromMatrix(const Matrix44<S> &m);
  void FromMatrix(const Matrix33<S> &m);

  void ToMatrix(Matrix44<S> &m) const;
  void ToMatrix(Matrix33<S> &m) const;

	void ToEulerAngles(S &alpha, S &beta, S &gamma) const;
	void FromEulerAngles(S alpha, S beta, S gamma);
  
  Point3<S> Rotate(const Point3<S> vec) const; 
  
  //duplicated ... because of gcc new confoming to ISO template derived classes
  //do no 'see' parent members (unless explicitly specified) 
  const S & V ( const int i ) const	{ assert(i>=0 && i<4); return Point4<S>::V(i); }
  S & V ( const int i )	{ assert(i>=0 && i<4); return Point4<S>::V(i); }

  /// constuctor that imports from different Quaternion types
  template <class Q>
  static inline Quaternion Construct( const Quaternion<Q> & b )
  {
    return Quaternion(S(b[0]),S(b[1]),S(b[2]),S(b[3]));
  }

private:
};

/*template<classS, class M> void QuaternionToMatrix(Quaternion<S> &s, M &m);
template<classS, class M> void MatrixtoQuaternion(M &m, Quaternion<S> &s);*/

template <class S> Quaternion<S> Interpolate(  Quaternion<S>   a,   Quaternion<S>   b, double t);
template <class S> Quaternion<S> &Invert(Quaternion<S> &q);
template <class S> Quaternion<S> Inverse(const Quaternion<S> &q);


//Implementation
template <class S> 
void Quaternion<S>::SetIdentity(){
	FromAxis(0, Point3<S>(1, 0, 0));
}
	

template <class S> Quaternion<S>::Quaternion(const S phi, const Point3<S> &a) {
  FromAxis(phi, a);
}
 	 

template <class S> Quaternion<S> Quaternion<S>::operator*(const S &s) const {
		return (Quaternion(V(0)*s,V(1)*s,V(2)*s,V(3)*s));
}
 
template <class S> Quaternion<S> Quaternion<S>::operator*(const Quaternion &q) const {		
	Point3<S> t1(V(1), V(2), V(3));
  Point3<S> t2(q.V(1), q.V(2), q.V(3));
		
  S d  = t2.dot(t1);
	Point3<S> t3 = t1 ^ t2;
		
  t1 *= q.V(0);
	t2 *= V(0);

	Point3<S> tf = t1 + t2 +t3;

   Quaternion<S> t;
	t.V(0) = V(0) * q.V(0) - d;
	t.V(1) = tf[0];
	t.V(2) = tf[1];
	t.V(3) = tf[2];
	return t;
}

template <class S> Quaternion<S> &Quaternion<S>::operator*=(const Quaternion &q) {
  S ww = V(0) * q.V(0) - V(1) * q.V(1) - V(2) * q.V(2) - V(3) * q.V(3);
	S xx = V(0) * q.V(1) + V(1) * q.V(0) + V(2) * q.V(3) - V(3) * q.V(2);
	S yy = V(0) * q.V(2) - V(1) * q.V(3) + V(2) * q.V(0) + V(3) * q.V(1);

	V(0) = ww; 
  V(1) = xx; 
  V(2) = yy;
  V(3) = V(0) * q.V(3) + V(1) * q.V(2) - V(2) * q.V(1) + V(3) * q.V(0);
	return *this;
}

template <class S> void Quaternion<S>::Invert() {
	V(1)*=-1;
	V(2)*=-1;
	V(3)*=-1;
}

template <class S> Quaternion<S> Quaternion<S>::Inverse() const{
	return Quaternion<S>( V(0), -V(1), -V(2), -V(3) );
}

template <class S> void Quaternion<S>::FromAxis(const S phi, const Point3<S> &a) {
  Point3<S> b = a;
  b.Normalize();
  S s = math::Sin(phi/(S(2.0)));

  V(0) = math::Cos(phi/(S(2.0)));
	V(1) = b[0]*s;
	V(2) = b[1]*s;
	V(3) = b[2]*s;
}

template <class S> void Quaternion<S>::ToAxis(S &phi, Point3<S> &a) const {
  S s = math::Asin(V(0))*S(2.0);
  phi = math::Acos(V(0))*S(2.0);

	if(s < 0) 
		phi = - phi;

  a.V(0) = V(1);
	a.V(1) = V(2);
	a.V(2) = V(3);
  a.Normalize();
}


template <class S> Point3<S> Quaternion<S>::Rotate(const Point3<S> p) const {
		Quaternion<S> co = *this;
		co.Invert();

    Quaternion<S> tmp(0, p.V(0), p.V(1), p.V(2));

		tmp = (*this) * tmp * co;
		return 	Point3<S>(tmp.V(1), tmp.V(2), tmp.V(3));
	}


template<class S, class M> void QuaternionToMatrix(const Quaternion<S> &q, M &m) {
  float x2 = q.V(1) + q.V(1);
  float y2 = q.V(2) + q.V(2);
  float z2 = q.V(3) + q.V(3);
  {
    float xx2 = q.V(1) * x2;
    float yy2 = q.V(2) * y2;
    float zz2 = q.V(3) * z2;
    m[0][0] = 1.0f - yy2 - zz2;
    m[1][1] = 1.0f - xx2 - zz2;
    m[2][2] = 1.0f - xx2 - yy2;
  }
  {
    float yz2 = q.V(2) * z2;
    float wx2 = q.V(0) * x2;
    m[1][2] = yz2 - wx2;
    m[2][1] = yz2 + wx2;
  }
  {
    float xy2 = q.V(1) * y2;
    float wz2 = q.V(0) * z2;
    m[0][1] = xy2 - wz2;
    m[1][0] = xy2 + wz2;
  }
  {
    float xz2 = q.V(1) * z2;
    float wy2 = q.V(0) * y2;
    m[2][0] = xz2 - wy2;
    m[0][2] = xz2 + wy2;
  }
}

template <class S> void Quaternion<S>::ToMatrix(Matrix44<S> &m) const	{
  QuaternionToMatrix<S, Matrix44<S> >(*this, m);
  m[0][3] = (S)0.0;
  m[1][3] = (S)0.0;
  m[2][3] = (S)0.0;
  m[3][0] = (S)0.0;
  m[3][1] = (S)0.0;
  m[3][2] = (S)0.0;
  m[3][3] = (S)1.0;
}

template <class S> void Quaternion<S>::ToMatrix(Matrix33<S> &m) const	{
  QuaternionToMatrix<S, Matrix33<S> >(*this, m);

 
}


template<class S, class M> void MatrixToQuaternion(const M &m, Quaternion<S> &q) {

  if ( m[0][0] + m[1][1] + m[2][2] > 0.0f ) {
    S t =  m[0][0] + m[1][1] + m[2][2] + 1.0f;
    S s = (S)0.5 / math::Sqrt(t);
    q.V(0) = s * t;
    q.V(3) = ( m[1][0] - m[0][1] ) * s;
    q.V(2) = ( m[0][2] - m[2][0] ) * s;
    q.V(1) = ( m[2][1] - m[1][2] ) * s;
  } else if ( m[0][0] > m[1][1] && m[0][0] > m[2][2] ) {
    S t = m[0][0] - m[1][1] - m[2][2] + 1.0f;
    S s = (S)0.5 / math::Sqrt(t);
    q.V(1) = s * t;
    q.V(2) = ( m[1][0] + m[0][1] ) * s;
    q.V(3) = ( m[0][2] + m[2][0] ) * s;
    q.V(0) = ( m[2][1] - m[1][2] ) * s;
  } else if ( m[1][1] > m[2][2] ) {
    S t = - m[0][0] + m[1][1] - m[2][2] + 1.0f;
    S s = (S)0.5 / math::Sqrt(t);
    q.V(2) = s * t;
    q.V(1) = ( m[1][0] + m[0][1] ) * s;
    q.V(0) = ( m[0][2] - m[2][0] ) * s;
    q.V(3) = ( m[2][1] + m[1][2] ) * s; 
  } else {
    S t = - m[0][0] - m[1][1] + m[2][2] + 1.0f;
    S s = (S)0.5 / math::Sqrt(t);
    q.V(3) = s * t;
    q.V(0) = ( m[1][0] - m[0][1] ) * s;
    q.V(1) = ( m[0][2] + m[2][0] ) * s;
    q.V(2) = ( m[2][1] + m[1][2] ) * s;
  }
}


template <class S> void Quaternion<S>::FromMatrix(const Matrix44<S> &m) {	
  MatrixToQuaternion<S, Matrix44<S> >(m, *this);
}
template <class S> void Quaternion<S>::FromMatrix(const Matrix33<S> &m) {	
  MatrixToQuaternion<S, Matrix33<S> >(m, *this);
}


template<class S>
void Quaternion<S>::ToEulerAngles(S &alpha, S &beta, S &gamma) const
{
#define P(a,b,c,d) (2*(V(a)*V(b)+V(c)*V(d)))
#define M(a,b,c,d) (2*(V(a)*V(b)-V(c)*V(d)))
	alpha = math::Atan2( P(0,1,2,3) , 1-P(1,1,2,2) );
	beta  = math::Asin ( M(0,2,3,1) );
	gamma = math::Atan2( P(0,3,1,2) , 1-P(2,2,3,3) );
#undef P
#undef M
}

template<class S>
void Quaternion<S>::FromEulerAngles(S alpha, S beta, S gamma)
{
	S cosalpha = math::Cos(alpha / 2.0);
	S cosbeta = math::Cos(beta / 2.0);
	S cosgamma = math::Cos(gamma / 2.0);
	S sinalpha = math::Sin(alpha / 2.0);
	S sinbeta = math::Sin(beta / 2.0);
	S singamma = math::Sin(gamma / 2.0);

	V(0) = cosalpha * cosbeta * cosgamma + sinalpha * sinbeta * singamma;
	V(1) = sinalpha * cosbeta * cosgamma - cosalpha * sinbeta * singamma;
	V(2) = cosalpha * sinbeta * cosgamma + sinalpha * cosbeta * singamma;
	V(3) = cosalpha * cosbeta * singamma - sinalpha * sinbeta * cosgamma;
}

template <class S> Quaternion<S> &Invert(Quaternion<S> &m) {
  m.Invert();
  return m;
}

template <class S> Quaternion<S> Inverse(const Quaternion<S> &m) {
  Quaternion<S> a = m;
  a.Invert();
  return a;
}

template <class S> Quaternion<S> Interpolate(   Quaternion<S>   a ,    Quaternion<S>   b , double t) {
 	 
		double v = a.V(0) * b.V(0) + a.V(1) * b.V(1) + a.V(2) * b.V(2) + a.V(3) * b.V(3);
		double phi = math::Acos(v);
		if(phi > 0.01) {
			a = a  * (math::Sin(phi *(1-t))/math::Sin(phi));
			b = b  * (math::Sin(phi * t)/math::Sin(phi));	
		}
		
		Quaternion<S> c;
		c.V(0) = a.V(0) + b.V(0);
		c.V(1) = a.V(1) + b.V(1);
		c.V(2) = a.V(2) + b.V(2);
		c.V(3) = a.V(3) + b.V(3);
		
		if(v < -0.999) { //almost opposite
			double d = t * (1 - t);
			if(c.V(0) == 0)
				c.V(0) += d;
			else
				c.V(1) += d;		
		}
		c.Normalize();
		return c;
	}
		
	

typedef Quaternion<float>  Quaternionf;
typedef Quaternion<double> Quaterniond;

} // end namespace


#endif
