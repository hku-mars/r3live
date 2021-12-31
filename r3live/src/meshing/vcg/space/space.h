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
Revision 1.2  2006/01/03 12:44:58  spinelli
fix some bugs.

Revision 1.1  2004/03/16 03:08:02  tarini
first commit


****************************************************************************/

#ifndef __VCGLIB_SPACE
#define __VCGLIB_SPACE


#include <vcg/math/linear.h>

namespace vcg {
/*@{*/
    /**
        This class represents the interface for any spatial objects.
				(points, lines, rays, segments, planes, triangles, axis aligned box).
				It consists in (the declaration of) a set of functions and types that
				each such object mush have. 
     */

template <int N, class S> class Point;
template <int N, class S> class Box;

class ParamType;

template <int N, class S> 
class Space {
public:
	/// Dimension is a constant determines the dimension of the space.
	enum {Dimension=N};
	/// the type used as scalar. Typically, float or double, but char or int are possible too.
	typedef S          ScalarType;
	/// type used as point Type
	typedef Point<N,S> PointType;
	/// the ...
	//typedef ParamType;

	/// returns the bounding box of the object
	Box<N,S> const BBox() const; 

	/// given a point, return the closest point
	PointType ClosestPoint(PointType const &p) const;

	PointType LocalToGlobal(ParamType);

	ParamType GlobalToLocal(ParamType);
}; // end class definition

/*@}*/



} // end namespace
#endif
