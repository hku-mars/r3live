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
Revision 1.26  2006/11/13 13:03:45  ponchio
Added GetBBox in Point3 (declaration) the body of the function is in box3.h

Revision 1.25  2006/10/13 12:59:24  cignoni
Added **explicit** constructor from three coords of a different scalartype

Revision 1.24  2006/09/28 13:37:35  m_di_benedetto
added non const * V()

Revision 1.23  2005/11/09 16:11:55  cignoni
Added Abs and LowClampToZero

Revision 1.22  2005/09/14 14:09:21  m_di_benedetto
Added specialized Convert() for the same scalar type.

Revision 1.21  2005/05/06 14:45:33  spinelli
cambiato parentesi nel costruttore di GetUV per rendere compatibile tale costruttore con MVC e borland

Revision 1.20  2005/04/27 16:05:19  callieri
line 466, added parentesis on default value creator getUV [borland]

Revision 1.19  2004/11/09 15:49:07  ganovelli
added GetUV

Revision 1.18  2004/10/13 12:45:51  cignoni
Better Doxygen documentation

Revision 1.17  2004/09/10 14:01:40  cignoni
Added polar to cartesian

Revision 1.16  2004/03/21 17:14:56  ponchio
Added a math::

Revision 1.15  2004/03/07 22:45:32  cignoni
Moved quality and normal functions to the triangle class.

Revision 1.14  2004/03/05 17:55:01  tarini
errorino: upper case in Zero()

Revision 1.13  2004/03/03 14:22:48  cignoni
Yet against cr lf mismatch

Revision 1.12  2004/02/23 23:42:26  cignoni
Translated comments, removed unusued stuff. corrected linefeed/cr

Revision 1.11  2004/02/19 16:12:28  cignoni
cr lf mismatch 2

Revision 1.10  2004/02/19 16:06:24  cignoni
cr lf mismatch

Revision 1.8  2004/02/19 15:13:40  cignoni
corrected sqrt and added doxygen groups

Revision 1.7  2004/02/17 02:08:47  cignoni
Di prova...

Revision 1.6  2004/02/15 23:35:47  cignoni
Cambiato nome type template in accordo alla styleguide

Revision 1.5  2004/02/10 01:07:15  cignoni
Edited Comments and GPL license

Revision 1.4  2004/02/09 13:48:02  cignoni
Edited doxygen comments
****************************************************************************/

#ifndef __VCGLIB_POINT3
#define __VCGLIB_POINT3

#include <assert.h>
#include <algorithm>
#include <vcg/math/base.h>

namespace vcg {

/** \addtogroup space */
/*@{*/
    /**
        The templated class for representing a point in 3D space.
        The class is templated over the ScalarType class that is used to represent coordinates. All the usual
        operator overloading (* + - ...) is present.
     */
template <class T> class Box3;

template <class P3ScalarType> class Point3
{
protected:
  /// The only data member. Hidden to user.
    P3ScalarType _v[3];

public:
    typedef P3ScalarType ScalarType;
    enum {Dimension = 3};


//@{

  /** @name Standard Constructors and Initializers
   No casting operators have been introduced to avoid automatic unattended (and costly) conversion between different point types
   **/

  inline Point3 () { }
    inline Point3 ( const P3ScalarType nx, const P3ScalarType ny, const P3ScalarType nz )
    {
        _v[0] = nx;
        _v[1] = ny;
        _v[2] = nz;
    }
    inline Point3 ( Point3 const & p )
    {
        _v[0]= p._v[0];
        _v[1]= p._v[1];
        _v[2]= p._v[2];
    }
    inline Point3 ( const P3ScalarType nv[3] )
    {
        _v[0] = nv[0];
        _v[1] = nv[1];
        _v[2] = nv[2];
    }
    inline Point3 & operator =( Point3 const & p )
    {
            _v[0]= p._v[0]; _v[1]= p._v[1]; _v[2]= p._v[2];
            return *this;
    }
    inline void SetZero()
    {
        _v[0] = 0;
        _v[1] = 0;
        _v[2] = 0;
    }

  /// Padding function: give a default 0 value to all the elements that are not in the [0..2] range.
  /// Useful for managing in a consistent way object that could have point2 / point3 / point4
    inline P3ScalarType Ext( const int i ) const
    {
        if(i>=0 && i<=2) return _v[i];
        else             return 0;
    }

    template <class Q>
    inline void Import( const Point3<Q> & b )
    {
        _v[0] = P3ScalarType(b[0]);
        _v[1] = P3ScalarType(b[1]);
        _v[2] = P3ScalarType(b[2]);
    }
    template <class EigenVector>
    inline void FromEigenVector( const EigenVector & b )
    {
        _v[0] = P3ScalarType(b[0]);
        _v[1] = P3ScalarType(b[1]);
        _v[2] = P3ScalarType(b[2]);
    }
    template <class EigenVector>
    inline void ToEigenVector( EigenVector & b ) const
    {
        b[0]=_v[0] ;
        b[1]=_v[1] ;
        b[2]=_v[2] ;
    }
  template <class Q>
  static inline Point3 Construct( const Point3<Q> & b )
  {
    return Point3(P3ScalarType(b[0]),P3ScalarType(b[1]),P3ScalarType(b[2]));
  }

  template <class Q>
  static inline Point3 Construct( const Q & P0, const Q & P1, const Q & P2)
  {
    return Point3(P3ScalarType(P0),P3ScalarType(P1),P3ScalarType(P2));
  }

  static inline Point3 Construct( const Point3<ScalarType> & b )
  {
    return b;
  }

//@}

//@{

  /** @name Data Access.
   access to data is done by overloading of [] or explicit naming of coords (x,y,z)**/

    inline P3ScalarType & operator [] ( const int i )
    {
        assert(i>=0 && i<3);
        return _v[i];
    }
    inline const P3ScalarType & operator [] ( const int i ) const
    {
        assert(i>=0 && i<3);
        return _v[i];
    }
  inline const P3ScalarType &X() const { return _v[0]; }
    inline const P3ScalarType &Y() const { return _v[1]; }
    inline const P3ScalarType &Z() const { return _v[2]; }
    inline P3ScalarType &X() { return _v[0]; }
    inline P3ScalarType &Y() { return _v[1]; }
    inline P3ScalarType &Z() { return _v[2]; }
    inline const P3ScalarType * V() const
    {
        return _v;
    }
    inline P3ScalarType * V()
    {
        return _v;
    }
    inline P3ScalarType & V( const int i )
    {
        assert(i>=0 && i<3);
        return _v[i];
    }
    inline const P3ScalarType & V( const int i ) const
    {
        assert(i>=0 && i<3);
        return _v[i];
    }
//@}
//@{

  /** @name Classical overloading of operators
  Note
  **/

    inline Point3 operator + ( Point3 const & p) const
    {
        return Point3<P3ScalarType>( _v[0]+p._v[0], _v[1]+p._v[1], _v[2]+p._v[2] );
    }
    inline Point3 operator - ( Point3 const & p) const
    {
        return Point3<P3ScalarType>( _v[0]-p._v[0], _v[1]-p._v[1], _v[2]-p._v[2] );
    }
    inline Point3 operator * ( const P3ScalarType s ) const
    {
        return Point3<P3ScalarType>( _v[0]*s, _v[1]*s, _v[2]*s );
    }
    inline Point3 operator / ( const P3ScalarType s ) const
    {
        return Point3<P3ScalarType>( _v[0]/s, _v[1]/s, _v[2]/s );
    }
        /// Dot product
    inline P3ScalarType operator * ( Point3 const & p ) const
    {
        return ( _v[0]*p._v[0] + _v[1]*p._v[1] + _v[2]*p._v[2] );
    }
    inline P3ScalarType dot( const Point3 & p ) const { return (*this) * p; }
        /// Cross product
    inline Point3 operator ^ ( Point3 const & p ) const
    {
        return Point3 <P3ScalarType>
        (
            _v[1]*p._v[2] - _v[2]*p._v[1],
            _v[2]*p._v[0] - _v[0]*p._v[2],
            _v[0]*p._v[1] - _v[1]*p._v[0]
        );
    }

    inline Point3 & operator += ( Point3 const & p)
    {
        _v[0] += p._v[0];
        _v[1] += p._v[1];
        _v[2] += p._v[2];
        return *this;
    }
    inline Point3 & operator -= ( Point3 const & p)
    {
        _v[0] -= p._v[0];
        _v[1] -= p._v[1];
        _v[2] -= p._v[2];
        return *this;
    }
    inline Point3 & operator *= ( const P3ScalarType s )
    {
        _v[0] *= s;
        _v[1] *= s;
        _v[2] *= s;
        return *this;
    }
    inline Point3 & operator /= ( const P3ScalarType s )
    {
        _v[0] /= s;
        _v[1] /= s;
        _v[2] /= s;
        return *this;
    }
        // Norme
    inline P3ScalarType Norm() const
    {
    return math::Sqrt( _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2] );
    }
    inline P3ScalarType SquaredNorm() const
    {
        return ( _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2] );
    }
        // Scalatura differenziata
    inline Point3 & Scale( const P3ScalarType sx, const P3ScalarType sy, const P3ScalarType sz )
    {
        _v[0] *= sx;
        _v[1] *= sy;
        _v[2] *= sz;
        return *this;
    }
    inline Point3 & Scale( const Point3 & p )
    {
        _v[0] *= p._v[0];
        _v[1] *= p._v[1];
        _v[2] *= p._v[2];
        return *this;
    }

    // Normalizzazione
    inline Point3 & Normalize()
    {
        P3ScalarType n = P3ScalarType(math::Sqrt(_v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2]));
        if (n > P3ScalarType(0)) { _v[0] /= n; _v[1] /= n; _v[2] /= n; }
        return *this;
    }

    // for compatibility with eigen port
    inline Point3 & normalized() { return Normalize(); }

    /**
     * Convert to polar coordinates from cartesian coordinates.
     *
     * Theta is the azimuth angle and ranges between [0, 2PI) degrees.
     * Phi is the elevation angle (not the polar angle) and ranges between [-PI/2, PI/2] degrees.
     *
     * /note Note that instead of the classical polar angle, which ranges between
     *       0 and PI degrees we opt for the elevation angle to obtain a more
     *       intuitive spherical coordinate system.
     */
    void ToPolarRad(P3ScalarType &ro, P3ScalarType &theta, P3ScalarType &phi) const
    {
        ro = Norm();
        theta = (P3ScalarType)atan2(_v[2], _v[0]);
        phi   = (P3ScalarType)asin(_v[1]/ro);
    }

    /**
     * Convert from polar coordinates to cartesian coordinates.
     *
     * Theta is the azimuth angle and ranges between [0, 2PI) radians.
     * Phi is the elevation angle (not the polar angle) and ranges between [-PI/2, PI/2] radians.
     *
     * \note Note that instead of the classical polar angle, which ranges between
     *       0 and PI degrees, we opt for the elevation angle to obtain a more
     *       intuitive spherical coordinate system.
     */
  void FromPolarRad(const P3ScalarType &ro, const P3ScalarType &theta, const P3ScalarType &phi)
    {
    _v[0]= ro*cos(theta)*cos(phi);
    _v[1]= ro*sin(phi);
    _v[2]= ro*sin(theta)*cos(phi);
    }

  Box3<P3ScalarType> GetBBox(Box3<P3ScalarType> &bb) const;
//@}
//@{

  /** @name Comparison Operators.
   Note that the reverse z prioritized ordering, useful in many situations.
   **/

inline bool operator == ( Point3 const & p ) const
    {
        return _v[0]==p._v[0] && _v[1]==p._v[1] && _v[2]==p._v[2];
    }
    inline bool operator != ( Point3 const & p ) const
    {
        return _v[0]!=p._v[0] || _v[1]!=p._v[1] || _v[2]!=p._v[2];
    }
    inline bool operator <  ( Point3 const & p ) const
    {
        return	(_v[2]!=p._v[2])?(_v[2]<p._v[2]):
                (_v[1]!=p._v[1])?(_v[1]<p._v[1]):
                               (_v[0]<p._v[0]);
    }
    inline bool operator >  ( Point3 const & p ) const
    {
        return	(_v[2]!=p._v[2])?(_v[2]>p._v[2]):
                (_v[1]!=p._v[1])?(_v[1]>p._v[1]):
                               (_v[0]>p._v[0]);
    }
    inline bool operator <= ( Point3 const & p ) const
    {
        return	(_v[2]!=p._v[2])?(_v[2]< p._v[2]):
                (_v[1]!=p._v[1])?(_v[1]< p._v[1]):
                               (_v[0]<=p._v[0]);
    }
    inline bool operator >= ( Point3 const & p ) const
    {
        return	(_v[2]!=p._v[2])?(_v[2]> p._v[2]):
                (_v[1]!=p._v[1])?(_v[1]> p._v[1]):
                               (_v[0]>=p._v[0]);
    }


    inline Point3 operator - () const
    {
        return Point3<P3ScalarType> ( -_v[0], -_v[1], -_v[2] );
    }
 //@}

}; // end class definition


template <class P3ScalarType>
inline P3ScalarType Angle( Point3<P3ScalarType> const & p1, Point3<P3ScalarType> const & p2 )
{
    P3ScalarType w = p1.Norm()*p2.Norm();
    if(w==0) return -1;
    P3ScalarType t = (p1*p2)/w;
    if(t>1) t = 1;
    else if(t<-1) t = -1;
    return (P3ScalarType) acos(t);
}

// versione uguale alla precedente ma che assume che i due vettori sono unitari
template <class P3ScalarType>
inline P3ScalarType AngleN( Point3<P3ScalarType> const & p1, Point3<P3ScalarType> const & p2 )
{
    P3ScalarType w = p1*p2;
    if(w>1)
        w = 1;
    else if(w<-1)
        w=-1;
  return (P3ScalarType) acos(w);
}


template <class P3ScalarType>
inline P3ScalarType Norm( Point3<P3ScalarType> const & p )
{
    return p.Norm();
}

template <class P3ScalarType>
inline P3ScalarType SquaredNorm( Point3<P3ScalarType> const & p )
{
    return p.SquaredNorm();
}

template <class P3ScalarType>
inline Point3<P3ScalarType> & Normalize( Point3<P3ScalarType> & p )
{
    p.Normalize();
    return p;
}

template <class P3ScalarType>
inline P3ScalarType Distance( Point3<P3ScalarType> const & p1,Point3<P3ScalarType> const & p2 )
{
    return (p1-p2).Norm();
}

template <class P3ScalarType>
inline P3ScalarType SquaredDistance( Point3<P3ScalarType> const & p1,Point3<P3ScalarType> const & p2 )
{
    return (p1-p2).SquaredNorm();
}

template <class P3ScalarType>
P3ScalarType ApproximateGeodesicDistance(const Point3<P3ScalarType>& p0, const Point3<P3ScalarType>& n0,
                                       const Point3<P3ScalarType>& p1, const Point3<P3ScalarType>& n1)
{
    Point3<P3ScalarType> V(p0-p1);
    V.Normalize();
    const P3ScalarType C0 = V*n0;
    const P3ScalarType C1 = V*n1;
    const P3ScalarType De = Distance(p0,p1);
    if(fabs(C0-C1)<0.0001) return De/sqrt(1-C0*C1);
    const P3ScalarType Dg = ((asin(C0) - asin(C1))/(C0-C1));
    return Dg*De;
}


    // Dot product preciso numericamente (solo double!!)
    // Implementazione: si sommano i prodotti per ordine di esponente
    // (prima le piu' grandi)
template<class P3ScalarType>
double stable_dot ( Point3<P3ScalarType> const & p0, Point3<P3ScalarType> const & p1 )
{
    P3ScalarType k0 = p0._v[0]*p1._v[0];
    P3ScalarType k1 = p0._v[1]*p1._v[1];
    P3ScalarType k2 = p0._v[2]*p1._v[2];

    int exp0,exp1,exp2;

    frexp( double(k0), &exp0 );
    frexp( double(k1), &exp1 );
    frexp( double(k2), &exp2 );

    if( exp0<exp1 )
    {
        if(exp0<exp2)
            return (k1+k2)+k0;
        else
            return (k0+k1)+k2;
    }
    else
    {
        if(exp1<exp2)
            return(k0+k2)+k1;
        else
            return (k0+k1)+k2;
    }
}



/// Point(p) Edge(v1-v2) dist, q is the point in v1-v2 with min dist
template<class P3ScalarType>
P3ScalarType PSDist( const Point3<P3ScalarType> & p,
                     const Point3<P3ScalarType> & v1,
                     const Point3<P3ScalarType> & v2,
                     Point3<P3ScalarType> & q )
{
    Point3<P3ScalarType> e = v2-v1;
    P3ScalarType  t = ((p-v1)*e)/e.SquaredNorm();
    if(t<0)      t = 0;
    else if(t>1) t = 1;
    q = v1+e*t;
    return Distance(p,q);
}


template <class P3ScalarType>
void GetUV( Point3<P3ScalarType> &n,Point3<P3ScalarType> &u, Point3<P3ScalarType> &v, Point3<P3ScalarType> up=(Point3<P3ScalarType>(0,1,0)) )
{
    n.Normalize();
    const double LocEps=double(1e-7);
    u=n^up;
  double len = u.Norm();
    if(len < LocEps)
    {
        if(fabs(n[0])<fabs(n[1])){
            if(fabs(n[0])<fabs(n[2])) up=Point3<P3ScalarType>(1,0,0); // x is the min
                                     else up=Point3<P3ScalarType>(0,0,1); // z is the min
        }else {
            if(fabs(n[1])<fabs(n[2])) up=Point3<P3ScalarType>(0,1,0); // y is the min
                                     else up=Point3<P3ScalarType>(0,0,1); // z is the min
        }
        u=n^up;
    }
    u.Normalize();
    v=n^u;
    v.Normalize();
}


template <class SCALARTYPE>
inline Point3<SCALARTYPE> Abs(const Point3<SCALARTYPE> & p) {
    return (Point3<SCALARTYPE>(math::Abs(p[0]), math::Abs(p[1]), math::Abs(p[2])));
}
// probably a more uniform naming should be defined...
template <class SCALARTYPE>
inline Point3<SCALARTYPE> LowClampToZero(const Point3<SCALARTYPE> & p) {
  return (Point3<SCALARTYPE>(std::max(p[0], (SCALARTYPE)0), std::max(p[1], (SCALARTYPE)0), std::max(p[2], (SCALARTYPE)0)));
}

typedef Point3<short>  Point3s;
typedef Point3<int>	   Point3i;
typedef Point3<float>  Point3f;
typedef Point3<double> Point3d;

/*@}*/

} // end namespace

#endif

