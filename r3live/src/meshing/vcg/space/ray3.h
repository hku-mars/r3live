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
Revision 1.4  2004/03/11 11:47:20  tarini
minor updates, corrections, added documentations, etc.


Revision 1.3  2004/03/10 15:27:18  tarini
first version

****************************************************************************/



#ifndef __VCGLIB_RAY3
#define __VCGLIB_RAY3

#include <vcg/space/point3.h>

namespace vcg {

/** \addtogroup space */
/*@{*/
/** 
Templated class for 3D rays.
  This is the class for infinite rays in 3D space. A Ray is stored just as two Point3:
	an origin and a direction (not necessarily normalized).
	@param RayScalarType (template parameter) Specifies the type of scalar used to represent coords.
	@param NORM: if on, the direction is always Normalized
*/
template <class RayScalarType, bool NORM=false>
class Ray3
{
public: 

	/// The scalar type
	typedef RayScalarType ScalarType;

	/// The point type
	typedef Point3<RayScalarType> PointType;

	/// The ray type
	typedef Ray3<RayScalarType,NORM> RayType;

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
	Ray3() {};
		/// The (origin, direction) constructor
	Ray3(const PointType &ori, const PointType &dir) {SetOrigin(ori); SetDirection(dir);};
//@}

		/// Operator to compare two rays
	inline bool operator == ( RayType const & p ) const
	{	return _ori==p._ori && _dir==p._dir; }
		/// Operator to dispare two rays
	inline bool operator != ( RayType const & p ) const
	{	return _ori!=p._ori || _dir!=p._dir; }
		/// Projects a point on the ray
	inline ScalarType Projection( const  PointType &p ) const
	{ if (NORM) return ScalarType((p-_ori).dot(_dir));
		else      return ScalarType((p-_ori).dot(_dir)/_dir.SquaredNorm());
	}
	  /// returns wheter this type is normalized or not
	static bool IsNormalized() {return NORM;};
	  /// calculates the point of parameter t on the ray.
	inline PointType P( const ScalarType t ) const
	{ return _ori + _dir * t; }
		/// normalizes direction field (returns a Normalized Ray)
	inline Ray3<ScalarType,true> &Normalize()
	{ if (!NORM) _dir.Normalize(); return *((Ray3<ScalarType,true>*)this);}
		/// normalizes direction field (returns a Normalized Ray) - static version
	static Ray3<ScalarType,true> &Normalize(RayType &p)
	{ p.Normalize(); return *((Ray3<ScalarType,true>*)(&p));}
	  /// importer for different ray types (with any scalar type or normalization beaviour)
	template <class Q, bool K>
	inline void Import( const Ray3<Q,K> & b )
	{ _ori.Import( b.Origin() );	_dir.Import( b.Direction() ); 
	  if ((NORM) && (!K)) _dir.Normalize(); 
		//printf("(=)%c->%c ",(!NORM)?'N':'n', NORM?'N':'n');
	}
		/// constructs a new ray importing it from an existing one
	template <class Q, bool K>
	static RayType Construct( const Ray3<Q,K> & b )
	{ RayType res; res.Import(b);  return res;
	}	
	PointType ClosestPoint(const PointType & p) const{
	return P(Projection(p));
	}
	  /// flips the ray
	inline void Flip(){
		_dir=-_dir;
	};

//@{
	 /** @name Linearity for 3d rays 
   (operators +, -, *, /) so a ray can be set as a linear combination
	 of several rays. Note that the result of any operation returns 
	 a non-normalized ray; however, the command r0 = r1*a + r2*b is licit 
	 even if r0,r1,r2 are normalized rays, as the normalization will
	 take place within the final assignement operation. 
	**/
	inline Ray3<ScalarType,false> operator + ( RayType const & p) const
	{return Ray3<ScalarType,false> ( _ori+p.Origin(), _dir+p.Direction() );}
	inline Ray3<ScalarType,false> operator - ( RayType const & p) const
	{return Ray3<ScalarType,false> ( _ori-p.Origin(), _dir-p.Direction() );}
	inline Ray3<ScalarType,false> operator * ( const ScalarType s ) const
	{return Ray3<ScalarType,false> ( _ori*s, _dir*s );}
	inline Ray3<ScalarType,false> operator / ( const ScalarType s ) const
	{ScalarType s0=((ScalarType)1.0)/s; return RayType( _ori*s0, _dir*s0 );}
//@}


//@{
	 /** @name Automatic normalized to non-normalized
	 "Ray3dN r0 = r1" is equivalent to
	 "Ray3dN r0 = r1.Normalize()" if r1 is a Ray3d
	**/
		/// copy constructor that takes opposite beaviour
	Ray3(const Ray3<ScalarType,!NORM > &r) 
	{ Import(r); };
		/// assignment
	inline RayType & operator = ( Ray3<ScalarType,!NORM> const &r) 
	{ Import(r); return *this; };
//@}

}; // end class definition

typedef Ray3<short>  Ray3s;
typedef Ray3<int>    Ray3i;
typedef Ray3<float>  Ray3f;
typedef Ray3<double> Ray3d;

typedef Ray3<short ,true> Ray3sN;
typedef Ray3<int   ,true> Ray3iN;
typedef Ray3<float ,true> Ray3fN;
typedef Ray3<double,true> Ray3dN;

	  /// returns closest point
template <class ScalarType, bool NORM> 
Point3<ScalarType> ClosestPoint( Ray3<ScalarType,NORM> r, const Point3<ScalarType> & p) 
{
	ScalarType t = r.Projection(p); 
	if (t<0) return r.Origin();
	return r.P(t); 
}

/*@}*/

} // end namespace
#endif
