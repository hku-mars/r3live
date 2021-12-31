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
Revision 1.1  2004/03/16 03:07:38  tarini
"dimensionally unified" version: first commit

Revision 1.5  2004/03/05 17:51:28  tarini
Errorino "ScalarType" -> "S"

Revision 1.4  2004/03/03 14:32:13  cignoni
Yet another cr lf mismatch

Revision 1.3  2004/02/23 23:44:21  cignoni
cr lf mismatch

Revision 1.2  2004/02/19 15:40:56  cignoni
Added doxygen groups

Revision 1.1  2004/02/13 02:16:22  cignoni
First working release.


****************************************************************************/


#ifndef __VCGLIB_BOX
#define __VCGLIB_BOX

#include <vcg/space/point.h>
#include <vcg/space/space.h>
#include <vcg/math/linear.h>

namespace vcg {

/** \addtogroup space */
/*@{*/
/** 
Templated class for 3D boxes.
  This is the class for definition of a axis aligned box in 2D or 3D space. 
	Typically used as bounding boxes.
	It is stored just as two Points (at the opposite vertices).
	@param S (template parameter) Specifies the type of scalar used to represent coords.
*/
template <int N, class S>
class Box : public Space<N,S> , Linear<Box>
{
public:
	typedef S          ScalarType;
	typedef Point<N,S> ParamType;
	typedef Point<N,S> PointType;
	enum {Dimension=N};

		/// The scalar type
protected:
	/// _min coordinate point
  Point3<S> _min;
	/// _max coordinate point
	Point3<S> _max;

public:

  inline const PointType &Max() const { return _max; } 
  inline PointType &Max() { return _max; } 
  inline const PointType &Min() const { return _min; } 
  inline PointType &Min() { return _min; } 

		/// The box constructor
	inline  Box() { 
		_min.X()= 1;_max.X()= -1;
		_min.Y()= 1;_max.Y()= -1;
		if (N>2) {_min.Z()= 1;_max.Z()= -1;}
	}
		/// Min Max constructor
	inline  Box( const PointType & mi, const PointType & ma ) { _min = mi; _max = ma; }
		/// The box distructor
	inline ~Box() { }
		/// Operator to compare two  boxes
	inline bool operator == ( Box const & p ) const
	{
		return _min==p._min && _max==p._max;
	}
		/// Operator to dispare two boxes
	inline bool operator != ( Box const & p ) const
	{
		return _min!=p._min || _max!=p._max;
	}
		/** Infaltes the box of a percentage..
			@param s Scalar value. E.g if s=0.1 the box enlarges of 10% in every direction 
			       if S==0.5 box doubles (+50% in every direction)
			       if S < 0 box shrinks 
			       if S==0.5 box reduces to a point
		*/
	void Inflate( const S s )
	{
		Inflate( (_max-_min)*s );
	}
		/** Enlarges the box dimensions by k in every direction, with k = bbox.diag*s
		*/
	void InflateFix( const S s )
	{
		S k = Diag()*s;
		 if (N==2) Inflate( PointType (k,k));
		 if (N==3) Inflate( PointType (k,k,k));
	}
		/** Enlarges the box dimensions by a fixed delta.
			@param delta Point in D space. If delta > 0 box enlarges. If delta < 0 box reduces.
		*/
	void Inflate( const PointType & delta )
	{
		_min -= delta;
		_max += delta;
	}
		/// Initializing the  box
	void Set( const PointType & p )
	{
		_min = _max = p;
	}
		/// Set the  box to a null value
	void SetNull()
	{
		_min.X()= 1; _max.X()= -1;
		_min.Y()= 1; _max.Y()= -1;
		_min.Z()= 1; _max.Z()= -1;
	}
		/** Add two  boxex: 
		    Returns minimal box that contains both operands.
			@param b The  box to add
		*/
	void Add( Box const & b )
	{
		if(IsNull()) *this=b;
		else
			Add(_min); Add(_max);
	}
		/** Add a point to a  box. 
		  The box is modified is the added point is aoutside it.
			@param p The point to add
		*/
	void Add( const PointType & p )
	{
		if(IsNull()) Set(p);
		else 
		{
			if(_min.X() > p.X()) _min.X() = p.X();
			else if(_max.X() < p.X()) _max.X() = p.X();
			if(_min.Y() > p.Y()) _min.Y() = p.Y();
			else if(_max.Y() < p.Y()) _max.Y() = p.Y();
			if (N>2) {
				if(_min.Z() > p.Z()) _min.Z() = p.Z();
				else if(_max.Z() < p.Z()) _max.Z() = p.Z();
			};
		}
	}
		/** Coputes intersection of Boxes: the minimal box containing both operands.
			@param b The other operand
		*/
	void Intersect( const Box & b )
	{
		if(_min.X() < b._min.X()) _min.X() = b._min.X();
		if(_min.Y() < b._min.Y()) _min.Y() = b._min.Y();
		if (N>2) if(_min.Z() < b._min.Z()) _min.Z() = b._min.Z();

		if(_max.X() > b._max.X()) _max.X() = b._max.X();
		if(_max.Y() > b._max.Y()) _max.Y() = b._max.Y();
		if (N>2) if(_max.Z() > b._max.Z()) _max.Z() = b._max.Z();

		if(_min.X()>_max.X() || _min.Y()>_max.Y() ) SetNull();
		else if (N>2) if (_min.Z()>_max.Z()) SetNull();
	}
		/** Traslalate the  box.
			@param p: the translation vector
		*/
	void Translate( const PointType & p )
	{
		_min += p;
		_max += p;
	}
		/** Check wheter a point is inside box.
			@param p The point
			@returns True if inside, false otherwise
		*/
	bool IsIn( PointType const & p ) const
	{
		if (N==2) return (
			_min.X() <= p.X() && p.X() <= _max.X() &&
			_min.Y() <= p.Y() && p.Y() <= _max.Y() 
		); 
		if (N==3) return (
			_min.X() <= p.X() && p.X() <= _max.X() &&
			_min.Y() <= p.Y() && p.Y() <= _max.Y() &&
			_min.Z() <= p.Z() && p.Z() <= _max.Z()
		); 	
	}
		/** Check wheter a point is inside box, open at left and closed at right [min..max)
			@param p The point 3D
			@returns True if inside, false otherwise
		*/
	bool IsInEx( PointType const & p ) const
	{
		if (N==2) return (
			_min.X() <= p.X() && p.X() < _max.X() &&
			_min.Y() <= p.Y() && p.Y() < _max.Y() 
		); 
		if (N==3) return (
			_min.X() <= p.X() && p.X() < _max.X() &&
			_min.Y() <= p.Y() && p.Y() < _max.Y() &&
			_min.Z() <= p.Z() && p.Z() < _max.Z()
		); 	
	}
		/** 
		  TODO: Move TO COLLIDE!!!
		  Verifica se due  box collidono cioe' se hanno una intersezione non vuota. Per esempio
			due  box adiacenti non collidono.
			@param b A  box
			@return True se collidoo, false altrimenti
		*/

	bool Collide(Box const &b)
	{
		return b._min.X()<_max.X() && b._max.X()>_min.X() &&
			   b._min.Y()<_max.Y() && b._max.Y()>_min.Y() &&
			   b._min.Z()<_max.Z() && b._max.Z()>_min.Z() ;
	}
		/** Controlla se il  box e' nullo.
			@return True se il  box e' nullo, false altrimenti
		*/
	bool IsNull() const { return _min.X()>_max.X() || _min.Y()>_max.Y() || _min.Z()>_max.Z(); }
		/** Controlla se il  box e' vuoto.
			@return True se il  box e' vuoto, false altrimenti
		*/
	bool IsEmpty() const { return _min==_max; }
		/// Restituisce la lunghezza della diagonale del  box.
	S Diag() const
	{
		return Distance(_min,_max);
	}
		/// Calcola il quadrato della diagonale del  box.
	S SquaredDiag() const
	{
		return SquaredDistance(_min,_max);
	}
		/// Calcola il centro del  box.
	PointType Center() const
	{
		return (_min+_max)/2;
	}

	  /// Returns global coords of a local point expressed in [0..1]^3
	PointType LocalToGlobal(PointType const & p) const{
		return PointType( 
			_min[0] + p[0]*(_max[0]-_min[0]), 
			_min[1] + p[1]*(_max[1]-_min[1]),
			_min[2] + p[2]*(_max[2]-_min[2]));
	}
	  /// Returns local coords expressed in [0..1]^3 of a point in 3D
	PointType GlobalToLocal(PointType const & p) const{
		return PointType( 
		  (p[0]-_min[0])/(_max[0]-_min[0]), 
		  (p[1]-_min[1])/(_max[1]-_min[1]), 
		  (p[2]-_min[2])/(_max[2]-_min[2])
			);
	}
		/// Computes the Volume for the  box.
	inline S Volume() const
	{
		if (N==2) return (_max.X()-_min.X())*(_max.Y()-_min.Y());
		if (N==3) return (_max.X()-_min.X())*(_max.Y()-_min.Y())*(_max.Z()-_min.Z());
	}
	  /// Area() and Volume() are sinonims (a
	inline S Area() const {
		return Volume();
	};
		/// Compute box size.
	PointType Size() const
	{
		return (_max-_min);
	}		
	  /// Compute box size X.
	inline S SizeX() const { return _max.X()-_min.X();}
		/// Compute box size Y.
	inline S SizeY() const { return _max.Y()-_min.Y();}
		/// Compute box size Z.
	inline S SizeZ() const { static_assert(N>2); return _max.Z()-_min.Z();}

	/** @name Linearity for boxes 
  **/

	/// sets a point to Zero
	inline void Zero()
	{
		_min.SetZero();
		_max.SetZero();
	}
	inline Box operator + ( Box const & p) const
	{
		return Box(_min+p._min,_max+p._max);
	}
	inline Box operator - ( Box const & p) const
	{
		return Box(_min-p._min,_max-p._max);
	}
	inline Box operator * ( const S s ) const
	{
		return Box(_min*s,_max*s);
	}
	inline Box operator / ( const S s ) const
	{
		S inv=S(1.0)/s;
		return Box(_min*inv,_max*inv);
	}
	inline Box & operator += ( Box const & p)
	{
		_min+=p._min; _max+=p._max;	return *this;
	}
	inline Box & operator -= ( Box const & p)
	{
		_min-=p._min; _max-=p._max;	return *this;
	}
	inline Box & operator *= ( const S s )
	{
		_min*=s; _max*=s;	return *this;
	}
	inline Box & operator /= ( const S s )
	{
		S inv=S(1.0)/s;
		_min*=s; _max*=s;	return *this;
		return *this;
	}
	inline Box operator - () const
	{
		return Box(-_min,-_max);
	}
//@}

//@{
 /** @name Iporters (for boxes in different spaces and with different scalar types)
	**/

	/// imports the box
	template <int N0, class S0>
	inline void Import( const Box<N0,S0> & b )
	{ _max.Import( b._max );_min.Import( b._min );
	}
	template <int N0, class S0>
		/// constructs a new ray importing it from an existing one
	static Box Construct( const Box<N0,S0> & b )
	{ 
		return Box(PointType::Construct(b._min),PointType::Construct(b._max));
	}	


}; // end class definition




typedef Box<3,short>  Box3s;
typedef Box<3,int>	  Box3i;
typedef Box<3,float>  Box3f;
typedef Box<3,double> Box3d;

typedef Box<2,short>  Box2s;
typedef Box<2,int>	  Box2i;
typedef Box<2,float>  Box2f;
typedef Box<2,double> Box2d;


/*@}*/

} // end namespace
#endif
