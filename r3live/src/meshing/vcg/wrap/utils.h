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
Revision 1.3  2005/11/12 18:10:35  cignoni
Removed Abs and LowClampToZero and added comments

Revision 1.2  2005/09/29 22:22:59  m_di_benedetto
Added classes GetBox3Functor and GetBarycenter3Functor.

Revision 1.1  2005/09/28 20:01:35  m_di_benedetto
First Commit.


****************************************************************************/

#ifndef __VCGLIB_WRAPUTILS_H
#define __VCGLIB_WRAPUTILS_H

// vcg headers
#include <vcg/math/base.h>
#include <vcg/space/point3.h>
#include <vcg/space/box3.h>

namespace vcg {

class EmptyClass {
public:
	typedef EmptyClass ClassType;
};

class GetPointerFunctor {
public:
	typedef GetPointerFunctor ClassType;

	template <class T>
	inline T * operator () (T & t) {
		return (&t);
	}

	template <class T>
	inline T * operator () (T * & t) {
		return (t);
	}
};
/// Helper class used to build in a easy way a functor that gives the bbox of a face
/// used mainly in the aabbtree that require such a functor as a parameter 

class GetBox3Functor {
public:
	template <class OBJTYPE, class SCALARTYPE>
	void operator () (const OBJTYPE & obj, Box3<SCALARTYPE> & box) {
		Box3<typename OBJTYPE::ScalarType> tb;
		obj.GetBBox(tb);
		box.Import(tb);
	}
};

class GetBarycenter3Functor {
public:
	template <class OBJTYPE, class SCALARTYPE>
	void operator () (const OBJTYPE & obj, Point3<SCALARTYPE> & bar) {
		bar.Import(Barycenter<OBJTYPE>(obj));
	}
};

} // end namespace vcg

#endif // #ifndef __VCGLIB_WRAPUTILS_H
