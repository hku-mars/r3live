#ifndef AUXMATH_H
#define AUXMATH_H
#pragma once

#include <complex>
#include <cmath>
//#include "Claussen.h"

static const double pi = 3.1415926535897932384626433;

const double SQRT2_2 = 0.7071067811865; // sqrt(2)/2 = 1/sqrt(2)

const double EPSILON = 1.0e-8;

template <class T> T sqr( const T& a ) { return a*a; }

inline double l2s( const double angle ){
  const double s = 2*fabs(sin(angle));
  return s < 1e-40 ? -92.103403719761827360719658 : log(s);
}
//
//inline double Lob( const double angle ){
//  return 0 <= angle && angle <= pi ? claussen(2*angle)/2 : -1e10;
//}

inline double PropLength( const double l, const double an, const double ad ){
  return l*(sin(an)/sin(ad));
}

inline double cot( const double x ){
  const double a = fabs(fmod(x+pi,2*pi)-pi)<1e-40 ? 1e-40 : x;
  return 1./tan(a);
}

typedef std::complex<double> Cmplx;

#endif // AUXMATH_H

