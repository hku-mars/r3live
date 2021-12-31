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
* GNU General Public License (http://www.gnu.o/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
****************************************************************************/

#ifndef __VCGLIB_COLOR4
#define __VCGLIB_COLOR4

#include <vcg/space/point3.h>
#include <vcg/space/point4.h>

namespace vcg {

/** \addtogroup space */
/*@{*/
    /**
        The templated class for representing 4 entity color.
        The class is templated over the ScalarType.  class that is used to represent color with float or with unsigned chars. All the usual
        operator overloading (* + - ...) is present.
     */
template <class T>
class Color4 : public Point4<T>
{
	typedef Point4<T> Base;
public:
  /// Constant for storing standard colors.
  /// Each color is stored in a simple in so that the bit pattern match with the one of Color4b.
	enum ColorConstant  {
	  Black   = 0xff000000,
	  Gray    = 0xff808080,
	  White   = 0xffffffff,

	  Red     = 0xff0000ff,
	  Green   = 0xff00ff00,
	  Blue    = 0xffff0000,

	  Cyan    = 0xffffff00,
	  Yellow  = 0xff00ffff,
	  Magenta = 0xffff00ff,

	  LightGray   =0xffc0c0c0,
	  LightRed    =0xff8080ff,
	  LightGreen  =0xff80ff80,
	  LightBlue   =0xffff8080,

	  DarkGray    =0xff404040,
	  DarkRed     =0xff000040,
	  DarkGreen   =0xff004000,
	  DarkBlue    =0xff400000
	};

  inline Color4 ( const T nx, const T ny, const T nz , const T nw ) :Point4<T>(nx,ny,nz,nw) {}
  inline Color4 ( const Point4<T> &c) :Point4<T>(c) {}
  inline Color4 (){}
  inline Color4 (ColorConstant cc);

  template <class Q>
	inline void Import(const Color4<Q> & b )
  {
	  (*this)[0] = T(b[0]);
	  (*this)[1] = T(b[1]);
	  (*this)[2] = T(b[2]);
	  (*this)[3] = T(b[3]);
  }

 template <class Q>
	inline void Import(const Point4<Q> & b )
  {
	  (*this)[0] = T(b[0]);
	  (*this)[1] = T(b[1]);
	  (*this)[2] = T(b[2]);
	  (*this)[3] = T(b[3]);
  }

 template <class Q>
  static inline Color4 Construct( const Color4<Q> & b )
  {
    return Color4(T(b[0]),T(b[1]),T(b[2]),T(b[3]));
  }

  //inline void Import(const Color4<float> &b);
  //inline void Import(const Color4<unsigned char> &b);

 inline Color4 operator + ( const Color4 & p) const
	{
		return Color4( (*this)[0]+p.V()[0], (*this)[1]+p.V()[1], (*this)[2]+p.V()[2], (*this)[3]+p.V()[3] );
	}


  inline void lerp(const Color4 &c0, const Color4 &c1, const float x)
  {
      assert(x>=0);
      assert(x<=1);

      (*this)[0]=(T)(c1.V()[0]*x + c0.V()[0]*(1.0f-x));
      (*this)[1]=(T)(c1.V()[1]*x + c0.V()[1]*(1.0f-x));
      (*this)[2]=(T)(c1.V()[2]*x + c0.V()[2]*(1.0f-x));
      (*this)[3]=(T)(c1.V()[3]*x + c0.V()[3]*(1.0f-x));
  }

  inline void lerp(const Color4 &c0, const Color4 &c1, const Color4 &c2, const Point3f &ip)
  {
    assert(fabs(ip[0]+ip[1]+ip[2]-1)<0.00001);

    (*this)[0]=(T)(c0[0]*ip[0] + c1[0]*ip[1]+ c2[0]*ip[2]);
    (*this)[1]=(T)(c0[1]*ip[0] + c1[1]*ip[1]+ c2[1]*ip[2]);
    (*this)[2]=(T)(c0[2]*ip[0] + c1[2]*ip[1]+ c2[2]*ip[2]);
    (*this)[3]=(T)(c0[3]*ip[0] + c1[3]*ip[1]+ c2[3]*ip[2]);
  }


  /// given a float and a range set the corresponding color in the well known red->green->blue color ramp. To reverse the direction of the ramp just swap minf and maxf.
    inline void SetColorRamp(const float &minf,const float  &maxf ,float v )
    {
        if(minf>maxf) { SetColorRamp(maxf,minf,maxf+(minf-v)); return; }

        float step=(maxf-minf)/4;
        if(v <  minf ) { *this=Color4<T>(Color4<T>::Red); return; }
        v-=minf;
        if(v<step) { lerp(Color4<T>(Color4<T>::Red),   Color4<T>(Color4<T>::Yellow),v/step); return;}
        v-=step;
        if(v<step) { lerp(Color4<T>(Color4<T>::Yellow),Color4<T>(Color4<T>::Green), v/step); return;}
        v-=step;
        if(v<step) { lerp(Color4<T>(Color4<T>::Green), Color4<T>(Color4<T>::Cyan),  v/step); return;}
        v-=step;
        if(v<step) { lerp(Color4<T>(Color4<T>::Cyan),  Color4<T>(Color4<T>::Blue),  v/step); return;}

        *this= Color4<T>(Color4<T>::Blue);
    }

    void SetHSVColor( float h, float s, float v)
    {
      float r,g,b;
      if(s==0.0){	// gray color
        r = g = b = v;
        (*this)[0]=(unsigned char)(255*r);
        (*this)[1]=(unsigned char)(255*g);
        (*this)[2]=(unsigned char)(255*b);
        (*this)[3]=255;
        return;
      }
      if(h==1.0) h = 0.0;

      int i   = int( floor(h*6.0) );
      float f = float(h*6.0f - floor(h*6.0f));

      float p = v*(1.0f-s);
      float q = v*(1.0f-s*f);
      float t = v*(1.0f-s*(1.0f-f));

      switch(i)
      {
        case 0: r=v; g=t; b=p; break;
        case 1: r=q; g=v; b=p; break;
        case 2: r=p; g=v; b=t; break;
        case 3: r=p; g=q; b=v; break;
        case 4: r=t; g=p; b=v; break;
        case 5: r=v; g=p; b=q; break;
        default: r=0;g=0;b=0; assert(0);break;
      }
      (*this)[0]=(unsigned char)(255*r);
      (*this)[1]=(unsigned char)(255*g);
      (*this)[2]=(unsigned char)(255*b);
      (*this)[3]=255;
    }

inline static Color4 GrayShade(float f)
{
 return Color4(f,f,f,1);
}

inline void SetGrayShade(float f)
{
 Import(Color4<float>(f,f,f,1));
}


/** Given an integer returns a well ordering of colors
// so that every color differs as much as possible form the previous one
// params:
//		range is the maximum expected value (max of the range)
//		value is the requested position (it must be <range);
*/
inline static Color4 Scatter(int range, int value,float Sat=.3f,float Val=.9f)
{
  int b, k, m=range;
  int r =range;

	for (b=0, k=1; k<range; k<<=1)
			if (value<<1>=m) {
				if (b==0) r = k;
				b += k;
				value -= (m+1)>>1;
				m >>= 1;
			}
	else m = (m+1)>>1;
	if (r>range-b) r = range-b;

	//TRACE("Scatter range 0..%i, in %i out %i\n",n,a,b);
	Color4 rc;
	rc.SetHSVColor(float(b)/float(range),Sat,Val);
	return rc;
}

inline static Color4 ColorRamp(const float &minf,const float  &maxf ,float v )
{
  Color4 rc;
  rc.SetColorRamp(minf,maxf,v);
  return rc;
}

inline static unsigned short ToUnsignedB5G5R5(Color4 &) { return 0;}
inline static unsigned short ToUnsignedR5G5B5(Color4 &) { return 0;}

inline static Color4 FromUnsignedB5G5R5(unsigned short)
{
  return Color4(Color4::White);
}
inline static Color4 FromUnsignedR5G5B5(unsigned short)
{
  return Color4(Color4::White);
}

}; /// END CLASS ///////////////////

template <> template <>
inline void Color4<float>::Import(const Color4<unsigned char> &b)
{
  (*this)[0]=b[0]/255.0f;
  (*this)[1]=b[1]/255.0f;
  (*this)[2]=b[2]/255.0f;
  (*this)[3]=b[3]/255.0f;
}

template <> template <>
inline void Color4<unsigned char>::Import(const Color4<float> &b)
{
  (*this)[0]=(unsigned char)(b[0]*255.0f);
  (*this)[1]=(unsigned char)(b[1]*255.0f);
  (*this)[2]=(unsigned char)(b[2]*255.0f);
  (*this)[3]=(unsigned char)(b[3]*255.0f);
}

template <> template <>
inline void Color4<unsigned char>::Import(const Point4<float> &b)
{
  (*this)[0]=(unsigned char)(b[0]*255.0f);
  (*this)[1]=(unsigned char)(b[1]*255.0f);
  (*this)[2]=(unsigned char)(b[2]*255.0f);
  (*this)[3]=(unsigned char)(b[3]*255.0f);
}

template <> template <>
inline Color4<unsigned char> Color4<unsigned char>::Construct( const Color4<float> & b )
{
    return Color4<unsigned char>(
									(unsigned char)(b[0]*255.0f),
									(unsigned char)(b[1]*255.0f),
									(unsigned char)(b[2]*255.0f),
									(unsigned char)(b[3]*255.0f));
}

template <> template <>
inline Color4<float> Color4<float>::Construct( const Color4<unsigned char> & b )
{
    return Color4<float>(
									(float)(b[0])/255.0f,
									(float)(b[1])/255.0f,
									(float)(b[2])/255.0f,
									(float)(b[3])/255.0f);
}

template<>
inline Color4<unsigned char>::Color4(Color4<unsigned char>::ColorConstant cc)
{
  *((int *)this )= cc;
}

template<>
inline Color4<float>::Color4(Color4<float>::ColorConstant cc)
{
  Import(Color4<unsigned char>((Color4<unsigned char>::ColorConstant)cc));
}

inline Color4<float> Clamp(Color4<float> &c)
{
	c[0]=math::Clamp(c[0],0.0f,1.0f);
	c[1]=math::Clamp(c[1],0.0f,1.0f);
	c[2]=math::Clamp(c[2],0.0f,1.0f);
	c[3]=math::Clamp(c[3],0.0f,1.0f);
	return c;
}

template<>
inline Color4<unsigned char> Color4<unsigned char>::operator + ( const Color4<unsigned char>  & p) const
{
		return Color4<unsigned char>(
									 (unsigned char)(math::Clamp(int((*this)[0])+int(p[0]),0,255)),
									 (unsigned char)(math::Clamp(int((*this)[1])+int(p[1]),0,255)),
									 (unsigned char)(math::Clamp(int((*this)[2])+int(p[2]),0,255)),
									 (unsigned char)(math::Clamp(int((*this)[3])+int(p[3]),0,255))
									 );
}


typedef Color4<unsigned char>  Color4b;
typedef Color4<float>          Color4f;
typedef Color4<double>         Color4d;


template<>
inline unsigned short Color4<unsigned char>::ToUnsignedB5G5R5(Color4<unsigned char> &cc)
{
  unsigned short r = cc[0]/8;
  unsigned short g = cc[1]/8;
  unsigned short b = cc[2]/8;
  unsigned short res = b + g*32 + r*1024;
  return res;
}

template<>
inline unsigned short Color4<unsigned char>::ToUnsignedR5G5B5(Color4<unsigned char> &cc)
{
  unsigned short r = cc[0]/8;
  unsigned short g = cc[1]/8;
  unsigned short b = cc[2]/8;
  unsigned short res = r + g*32 + b*1024;
  return res;
}


template<>
inline Color4<unsigned char> Color4<unsigned char>::FromUnsignedR5G5B5(unsigned short val)
{
  unsigned short r = val % 32 *8;
  unsigned short g = ((val/32)%32)*8;
  unsigned short b = ((val/1024)%32)*8;
      Color4b cc((unsigned char)r,(unsigned char)g,(unsigned char)b,(unsigned char)255);
  return cc;
}

template<>
inline Color4<unsigned char> Color4<unsigned char>::FromUnsignedB5G5R5(unsigned short val)
{
  unsigned short b = val % 32 *8;
  unsigned short g = ((val/32)%32)*8;
  unsigned short r = ((val/1024)%32)*8;
      Color4b cc((unsigned char)r,(unsigned char)g,(unsigned char)b,(unsigned char)255);
  return cc;
}

/*@}*/


} // end of NameSpace

#endif
