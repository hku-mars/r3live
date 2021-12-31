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
Revision 1.8  2004/05/14 03:14:04  ponchio
Added Distance

Revision 1.7  2004/05/10 10:58:35  ganovelli
name of the constructor changed from LineType to Line3

Revision 1.6  2004/03/11 11:47:20  tarini
minor updates, corrections, added documentations, etc.

Revision 1.5  2004/03/10 15:27:48  tarini
added Normalized flag


Revision 1.1  2004/03/08 16:15:48  tarini
first version (tarini)

****************************************************************************/



#ifndef __VCGLIB_LINE3
#define __VCGLIB_LINE3

#include <vcg/space/point3.h>

namespace vcg {

/** \addtogroup space */
/*@{*/
/** 
Templated class for 3D lines.
  This is the class for infinite lines in 3D space. A Line is stored just as two Point3:
	an origin and a direction (not necessarily normalized).
	@param LineScalarType (template parameter) Specifies the type of scalar used to represent coords.
	@param NORM: if on, the direction is always Normalized
*/
template <class LineScalarType, bool NORM=false>
class Line3
{
public: 

	/// The scalar type
	typedef LineScalarType ScalarType;

	/// The point type
	typedef Point3<LineScalarType> PointType;

	/// The line type
	typedef Line3<LineScalarType,NORM> LineType;

private:

	/// Origin
	PointType _ori;

	/// Direction (not necessarily normalized, unless so specified by NORM)
	PointType _dir;

public:

//@{
	 /** @name Members to access the origin or direction
	   Direction() cannot be assigned directly.
		 Use SetDirection() or Set() instead.
	**/
		/// 
  inline const PointType &Origin() const { return _ori; } 
  inline PointType &Origin() { return _ori; } 
  inline const PointType &Direction() const { return _dir; } 
		/// sets the origin
	inline void SetOrigin( const PointType & ori )
	{	_ori=ori; }
		/// sets the direction
	inline void SetDirection( const PointType & dir)
	{	_dir=dir; if (NORM) _dir.Normalize();  }
		/// sets origin and direction.
	inline void Set( const PointType & ori, const PointType & dir )
	{	SetOrigin(ori); SetDirection(dir); }
//@}

//@{
	 /** @name Constructors 
	**/
 		/// The empty constructor
	Line3() {};
		/// The (origin, direction) constructor
	Line3(const PointType &ori, const PointType &dir) {SetOrigin(ori); SetDirection(dir);};
//@}

		/// Operator to compare two lines
	inline bool operator == ( LineType const & p ) const
	{	return _ori==p._ori && _dir==p._dir; }
		/// Operator to dispare two lines
	inline bool operator != ( LineType const & p ) const
	{	return _ori!=p._ori || _dir!=p._dir; }
		/// Projects a point on the line
	inline ScalarType Projection( const  PointType &p ) const
	{ if (NORM) return ScalarType((p-_ori).dot(_dir));
		else      return ScalarType((p-_ori).dot(_dir)/_dir.SquaredNorm());
	}
	  /// returns wheter this type is normalized or not
	static bool IsNormalized() {return NORM;};
	  /// calculates the point of parameter t on the line.
	inline PointType P( const ScalarType t ) const
	{ return _ori + _dir * t; }
		/// normalizes direction field (returns a Normalized Line)
	inline Line3<ScalarType,true> &Normalize()
	{ if (!NORM) _dir.Normalize(); return *((Line3<ScalarType,true>*)this);}
		/// normalizes direction field (returns a Normalized Line) - static version
	static Line3<ScalarType,true> &Normalize(LineType &p)
	{ p.Normalize(); return *((Line3<ScalarType,true>*)(&p));}
	  /// importer for different line types (with any scalar type or normalization beaviour)
	template <class Q, bool K>
	inline void Import( const Line3<Q,K> & b )
	{ _ori.Import( b.Origin() );	_dir.Import( b.Direction() ); 
	  if ((NORM) && (!K)) _dir.Normalize(); 
		//printf("(=)%c->%c ",(!NORM)?'N':'n', NORM?'N':'n');
	}
		/// constructs a new line importing it from an existing one
	template <class Q, bool K>
	static LineType Construct( const Line3<Q,K> & b )
	{ LineType res; res.Import(b);  return res;
	}	
	PointType ClosestPoint(const PointType & p) const{
	return P(Projection(p));
	}
	  /// flips the line
	inline void Flip(){
		_dir=-_dir;
	};

//@{
	 /** @name Linearity for 3d lines 
   (operators +, -, *, /) so a line can be set as a linear combination
	 of several lines. Note that the result of any operation returns 
	 a non-normalized line; however, the command r0 = r1*a + r2*b is licit 
	 even if r0,r1,r2 are normalized lines, as the normalization will
	 take place within the final assignement operation. 
	**/
	inline Line3<ScalarType,false> operator + ( LineType const & p) const
	{return Line3<ScalarType,false> ( _ori+p.Origin(), _dir+p.Direction() );}
	inline Line3<ScalarType,false> operator - ( LineType const & p) const
	{return Line3<ScalarType,false> ( _ori-p.Origin(), _dir-p.Direction() );}
	inline Line3<ScalarType,false> operator * ( const ScalarType s ) const
	{return Line3<ScalarType,false> ( _ori*s, _dir*s );}
	inline Line3<ScalarType,false> operator / ( const ScalarType s ) const
	{ScalarType s0=((ScalarType)1.0)/s; return LineType( _ori*s0, _dir*s0 );}
//@}


//@{
	 /** @name Automatic normalized to non-normalized
	 "Line3dN r0 = r1" is equivalent to
	 "Line3dN r0 = r1.Normalize()" if r1 is a Line3d
	**/
		/// copy constructor that takes opposite beaviour
	Line3 (const Line3<ScalarType,!NORM > &r) 
	{ Import(r); };
		/// assignment
	inline LineType & operator = ( Line3<ScalarType,!NORM> const &r) 
	{ Import(r); return *this; };
//@}

}; // end class definition

typedef Line3<short>  Line3s;
typedef Line3<int>	  Line3i;
typedef Line3<float>  Line3f;
typedef Line3<double> Line3d;

typedef Line3<short ,true> Line3sN;
typedef Line3<int   ,true> Line3iN;
typedef Line3<float ,true> Line3fN;
typedef Line3<double,true> Line3dN;

	  /// returns closest point
template <class ScalarType, bool NORM> 
Point3<ScalarType> ClosestPoint( Line3<ScalarType,NORM> l, const Point3<ScalarType> & p) 
{
	return l.P(l.Projection(p)); 
}

template <class ScalarType, bool NORM> 
ScalarType Distance(const Line3<ScalarType, NORM> &l, 
		    const Point3<ScalarType> &p) {
  Point3<ScalarType> o = l.ClosestPoint(p);
  return (o - p).Norm();
}

/*@}*/

} // end namespace
#endif
