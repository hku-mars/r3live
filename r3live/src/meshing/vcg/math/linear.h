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
Revision 1.2  2005/12/12 11:24:09  ganovelli
missing type added

Revision 1.1  2004/03/16 03:08:17  tarini
first version


****************************************************************************/

#ifndef __VCGLIB_LINEAR
#define __VCGLIB_LINEAR

namespace vcg {
/*@{*/
    /**
        This class represents the common interface for any linear objects.
				It consists (the declaration of) a set of functions and types that
				each such object mush have.
				Linear have the Zero element (neutral element for sums)
				moltiplication (for a scalar), and two linear elements of
				a given type can be summed.
				In this way it is possible to interpolate between two different linear entities.
				For example:
				LinearType a,b,c,d,e,f;
				...
				d = a * 0.1 + b * 0.9;
				e = a + (b - a) * 0.9;
     */
	template <class T>
	class Linear{
	public:
		typedef T ScalarType;
		inline void SetZero();
		T operator + ( T const & p) const;
		T operator - ( T const & p) const;
		T operator * ( const ScalarType );
		T operator / ( const ScalarType ) const;
		T & operator += ( T const & );
		T & operator -= ( T const & );
		T & operator *= ( const ScalarType );
		T & operator /= ( const ScalarType );
		T operator - () const;
	};
};

#endif
