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

#ifndef __VCGLIB_TEXCOORD2__
#define __VCGLIB_TEXCOORD2__

#include <vcg/space/point2.h>

namespace vcg {
/** \addtogroup space */
/*@{*/

/**
    Templated class for a set of 2D texture coord. It for each is templated over two parameters:
  the type of the tex coord and the number of texcoord to be stored. This class is intended to be used when many textures
  id are shared over the same surface, so for each coord the id of the texture is stored. If no id is needed see the vcg::TexCoord2Simple class.
*/

template<class T = float, int NMAX = 1>
class TexCoord2
{
public:
  typedef Point2<T>  PointType;
  typedef T ScalarType;


private:
    PointType _t[NMAX];
    short     _n[NMAX];
public:

  TexCoord2(T u, T v) { if(NMAX>0) _n[0]=0; _t[0][0]=u; _t[0][1]=v; };
  TexCoord2() {  };

  inline const PointType &P() const { return _t[0]; };
  inline PointType &P() { return _t[0]; };

  inline const PointType &P(const int i) const { assert(i>0 && i<NMAX); return _t[i]; };
  inline PointType &P(const int i) { assert(i>0 && i<NMAX); return _t[i]; };

    inline T & U() { return _t[0][0]; }
    inline T & V() { return _t[0][1]; }
    inline const T & U() const { return _t[0][0]; }
    inline const T & V() const { return _t[0][1]; }
    inline T & U(const int i) { assert(i>0 && i<NMAX);  return _t[i][0]; }
    inline T & V(const int i) { assert(i>0 && i<NMAX);  return _t[i][1]; }
    inline const T & U(const int i) const { assert(i>0 && i<NMAX); return _t[i][0]; }
    inline const T & V(const int i) const { assert(i>0 && i<NMAX); return _t[i][1]; }

    inline short     & N() { return _n[0]; }
        inline short N() const { return _n[0]; }

    inline short     & N(const int i) { assert(i>0 && i<NMAX); return _n[i]; }
        inline short N(const int i) const { assert(i>0 && i<NMAX); return _n[i]; }


  /* <OLD_METHODS> (lowercase ones). DEPRECATED. TO BE REMOVED SOON.*/
    /**/inline T & u() { return _t[0][0]; }
    /**/inline T & v() { return _t[0][1]; }
    /**/inline const T & u() const { return _t[0][0]; }
    /**/inline const T & v() const { return _t[0][1]; }
    /**/inline T & u(const int i) { return _t[i][0]; }
    /**/inline T & v(const int i) { return _t[i][1]; }
    /**/inline const T & u(const int i) const { return _t[i][0]; }
    /**/inline const T & v(const int i) const { return _t[i][1]; }
  /**/
    /**/inline short     & n() { return _n[0]; }
        /**/inline short n() const { return _n[0]; }
  /**/
    /**/inline short     & n(const int i) { return _n[i]; }
        /**/inline short n(const int i) const { return _n[i]; }
    /**/
    /**/inline Point2<T> & t(const int i) { return _t[i]; }
    /**/inline Point2<T> t(const int i) const { return _t[i]; }
  /**/
    /**/inline Point2<T> & t() { return _t[0]; }
    /**/inline Point2<T> t() const { return _t[0]; }
  /* </OLD_METHODS> */

    inline bool operator == ( TexCoord2 const & p ) const
        {
         for(int i=0;i<NMAX;++i)
             if(p._t[i] != _t[i] || p._n[i] != _n[i]) return false;
         return true;
        }

  inline bool operator != ( TexCoord2 const & p ) const
        {
         for(int i=0;i<NMAX;++i)
             if(p._t[i] != _t[i] || p._n[i] != _n[i]) return true;
         return false;
        }

    inline bool operator < ( TexCoord2 const & p ) const
        {
         for(int i=0;i<NMAX;++i)
             if(p._t[i] != _t[i]) return p._t[i] < _t[i];
         return false;
        }

    enum { n_coords=NMAX };
};

/**
    Templated class for a single 2D texture coord.
*/
template<class T = float >
class TexCoord2Simple
{
public:
  typedef Point2<T>  PointType;
  typedef T ScalarType;

private:
    Point2<T> _t;

    inline short & static_n() const
    {
        static short _n = 0;
        return _n;
    }

public:

    inline T & U() { return _t[0]; }
    inline T & V() { return _t[1]; }
    inline const T & U() const { return _t[0]; }
    inline const T & V() const { return _t[1]; }
    inline T & U(const int i) { (void)i; assert(i==0); return _t[0]; }
    inline T & V(const int i) { (void)i; assert(i==0); return _t[1]; }
    inline const T & U(const int i) const { (void)i; assert(i==0); return _t[0]; }
    inline const T & V(const int i) const { (void)i; assert(i==0); return _t[1]; }

    inline Point2<T> & P(const int i)       { (void)i; assert(i==0); return _t; }
    inline Point2<T>   P(const int i) const { (void)i; assert(i==0); return _t; }

    inline Point2<T> & P()       { return _t; }
    inline Point2<T>   P() const { return _t; }

    inline short & N()       { assert(static_n()==0); return static_n(); }
    inline short   N() const { assert(static_n()==0); return 0; }

    inline short & N(const int i)       { (void)i; assert(i==0); return static_n(); }
    inline short   N(const int i) const { (void)i; assert(i==0); return 0; }


/* <OLD_METHODS> (lowercase ones). DEPRECATED. TO BE REMOVED SOON.*/
    inline T & u() { return _t[0]; }
    inline T & v() { return _t[1]; }
    inline const T & u() const { return _t[0]; }
    inline const T & v() const { return _t[1]; }
    inline T & u(const int i) { (void)i; assert(i==0); return _t[0]; }
    inline T & v(const int i) { (void)i; assert(i==0); return _t[1]; }
    inline const T & u(const int i) const { (void)i; assert(i==0); return _t[0]; }
    inline const T & v(const int i) const { (void)i; assert(i==0); return _t[1]; }

    inline Point2<T> & t(const int i)       { (void)i; assert(i==0); return _t; }
    inline Point2<T>   t(const int i) const { (void)i; assert(i==0); return _t; }

    inline Point2<T> & t()       { return _t; }
    inline Point2<T>   t() const { return _t; }

    inline short & n()       { assert(static_n()==0); return static_n(); }
    inline short   n() const { assert(static_n()==0); return 0; }

    inline short & n(const int i)       { (void)i; assert(i==0); return static_n(); }
    inline short   n(const int i) const { (void)i; assert(i==0); return 0; }

/* </OLD_METHODS> */

    inline bool operator == ( TexCoord2Simple const & p ) const
        {
            return _t==p._t;
        }

    enum { n_coords=1};

};
typedef TexCoord2<float>  TexCoord2f;
typedef TexCoord2<double> TexCoord2d;


/*@}*/

}

#endif
