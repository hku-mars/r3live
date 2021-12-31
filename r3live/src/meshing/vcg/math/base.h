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
Revision 1.21  2007/01/08 09:23:49  pietroni
added explicit cast in function inline float Sqrt(const int v) in order to avoid warnings

Revision 1.20  2006/10/13 13:14:50  cignoni
Added two sqrt templates for resolving ambiguity of sqrt(int)

Revision 1.19  2005/12/01 01:03:37  cignoni
Removed excess ';' from end of template functions, for gcc compiling

Revision 1.18  2004/08/31 15:42:59  fasano
Aggiunte macro sin/cos/atan per C++ Builder

Revision 1.17  2004/05/10 13:00:14  ganovelli
limits function cancelled

Revision 1.16  2004/05/03 08:38:08  ganovelli
correction on templates

Revision 1.15  2004/04/15 09:36:59  ganovelli
 Min and Max changed from const members to static class function
Use: Value<float>::Min()

Revision 1.14  2004/03/31 12:41:55  tarini
debugged Max and Min const values (to make them linkable)

Revision 1.13  2004/03/31 10:09:19  cignoni
int64 -> long long for GCC compatibility

Revision 1.12  2004/03/16 00:23:50  tarini
- added VoidType    - added "static_assert"

Revision 1.11  2004/03/10 17:37:54  tarini
Added Atan2.
Added common utilities: Max, Min, Swap, Sort(a,b), Sort(a,b,c).
Changed Max values syntax. example:  Value<float>::Max

Revision 1.10  2004/03/10 16:54:57  tarini
Added Atan2.
Added common utilities: Max, Min, Swap, Sort(a,b), Sort(a,b,c).
Changed Max values syntax. example:  Value<float>::Max

Revision 1.7  2004/03/08 19:38:29  tarini
Added Min e Max. usage: Min<float>::Value (tarini)

Revision 1.6  2004/03/08 14:49:37  ponchio
Aggiunti un po di inline davanti alle funzioni

Revision 1.5  2004/03/04 00:21:00  ponchio
added Acos e Asin

Revision 1.4  2004/03/03 22:51:49  cignoni
changed math from class to template

Revision 1.2  2004/02/13 02:18:57  cignoni
Edited Comments and GPL license


****************************************************************************/

#ifndef __VCGLIB_MATH_BASE
#define __VCGLIB_MATH_BASE

#include <float.h>
#include <math.h>
#include <assert.h>
#include <limits>
#include <algorithm>


  #ifdef __BORLANDC__
    float sqrtf (float v) {return sqrt(v);}
    float fabsf (float v) {return fabs(v);}
    float cosf  (float v) {return cos(v);}
    float sinf  (float v) {return sin(v);}
    float acosf  (float v) {return acos(v);}
    float asinf  (float v) {return asin(v);}
    float atan2f (float v0, float v1) {return atan2(v0,v1);}
  #endif

namespace vcg {

namespace math {

 template <class SCALAR>
 class MagnitudoComparer
    {
      public:
        inline bool operator() ( const SCALAR a, const SCALAR b ) { return fabs(a)>fabs(b);  }
    };

  inline float Sqrt(const short v)   { return sqrtf(v); }
  inline float Sqrt(const int v)   { return sqrtf((float)v); }

  inline float Sqrt(const float v)   { return sqrtf(v); }
  inline float Abs(const float v)   { return fabsf(v); }
  inline float Cos(const float v)   { return cosf(v); }
  inline float Sin(const float v)   { return sinf(v); }
  inline float Acos(const float v)   { return acosf(v); }
  inline float Asin(const float v)   { return asinf(v); }
  inline float Atan2(const float v0,const float v1)   { return atan2f(v0,v1); }

  inline double Sqrt(const double v)   { return sqrt(v); }
  inline double Abs(const double v)   { return fabs(v); }
  inline double Cos(const double v)   { return cos(v); }
  inline double Sin(const double v)   { return sin(v); }
  inline double Acos(const double v)   { return acos(v); }
  inline double Asin(const double v)   { return asin(v); }
  inline double Atan2(const double v0,const double v1)   { return atan2(v0,v1); }

    template <typename T> inline static T Sqr(T a) { return a*a; }

  template<class T> inline const T & Min(const T &a, const T &b,const T &c){
    if (a<b) {
      if(a<c) return a;
         else return c;
      } else {
      if(b<c) return b;
      else return c;
      }
    }
  template<class T> inline const T & Max(const T &a, const T &b, const T &c){
    if (a>b) {
      if(a>c) return a;
         else return c; // if c<a then c is smaller than b...
      } else {
      if(b>c) return b;
      else return c;
      }
  }

	template<class T> inline void Sort(T &a, T &b){
		if (a>b) std::swap(a,b);
	}
	template<class T> inline void Sort(T &a, T &b, T &c){
		if (a>b) std::swap(a,b);
		if (b>c) {std::swap(b,c); if (a>b) std::swap(a,b);}
	}

/* Some <math.h> files do not define M_PI... */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef SQRT_TWO
#define SQRT_TWO 1.4142135623730950488
#endif

template <class SCALAR>
inline SCALAR  Clamp( const SCALAR & val, const SCALAR& minval, const SCALAR& maxval)
{
	if(val < minval) return minval;
	if(val > maxval) return maxval;
	return val;
}



inline float   ToDeg(const float &a){return a*180.0f/float(M_PI);}
inline float   ToRad(const float &a){return float(M_PI)*a/180.0f;}
inline double  ToDeg(const double &a){return a*180.0/M_PI;}
inline double  ToRad(const double &a){return M_PI*a/180.0;}


#if defined(_MSC_VER) // Microsoft Visual C++
template<class T> int IsNAN(T t) {    return _isnan(t) || (!_finite(t)); }
#elif defined(__GNUC__) // GCC
template<class T> int IsNAN(T t) {    return std::isnan(t) || std::isinf(t); }
#else // generic

template<class T> int IsNAN(T t)
{
     if(std::numeric_limits<T>::has_infinity)
         return !(t <= std::numeric_limits<T>::infinity());
     else
         return t != t;
}

#endif
}	// End math namespace

/// a type that stands for "void". Useful for Parameter type of a point.
class VoidType{ public:
    VoidType(){};
};

}	// End vcg namespace


#endif
