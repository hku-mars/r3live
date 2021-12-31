/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2006                                                \/)\/    *
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

#ifndef __VCG_FACE_PLUS_COMPONENT_RT
#define __VCG_FACE_PLUS_COMPONENT_RT

#include <vcg/space/plane3.h>

namespace vcg {
  namespace face {

template <class CoordType>
struct EdgePlaneInfo{
	CoordType edge[3];
	::vcg::Plane3<typename CoordType::ScalarType> plane;
	typename CoordType::ScalarType edgescale;
};
/** \addtogroup face
  @{
*/

/*! \brief Per Face Precomputed Edge/Plane

  This component is used to speed up some geometric queries like the ray-triangle intersection or the Point-Triangle distance.
  Before using it you have to initialize it using \ref UpdateComponentEP class
    */

template <class T> class EdgePlane: public T {
public:
	typedef EdgePlaneInfo<typename T::VertexType::CoordType> EdgePlaneType;

  typename T::VertexType::CoordType &Edge(const int j) {
		return _ep.edge[j];
	}
  typename T::VertexType::CoordType  cEdge(const int j)const {
		return _ep.edge[j];
	}

	typename vcg::Plane3<typename T::VertexType::CoordType::ScalarType> &Plane() {
		return _ep.plane;
	}
  typename vcg::Plane3<typename T::VertexType::CoordType::ScalarType>  cPlane()const {
		return _ep.plane;
	}

  static bool HasEdgePlane()   {   return true; }

	static void Name(std::vector<std::string> & name){name.push_back(std::string("EdgePlane"));T::Name(name);}

private:

EdgePlaneType _ep;
};


// This empty class is rarely useful but you need it if you have code
// where you eventually decide (not at compile time) what closest algorithm you need.
// (for example in unit testing...)

template <class T> class EmptyEdgePlane: public T {
public:
typedef EdgePlaneInfo<typename T::VertexType::CoordType> EdgePlaneType;

typename T::VertexType::CoordType &Edge(const int ) { assert(0);  static typename T::VertexType::CoordType dum; return dum;}
typename T::VertexType::CoordType &cEdge(const int ) const { assert(0);  static typename T::VertexType::CoordType dum; return dum;}

typename vcg::Plane3<typename T::VertexType::CoordType::ScalarType> &Plane() {assert(0);  static typename vcg::Plane3<typename T::VertexType::CoordType::ScalarType> dum; return dum;}
typename vcg::Plane3<typename T::VertexType::CoordType::ScalarType> &cPlane() const {assert(0);  static typename vcg::Plane3<typename T::VertexType::CoordType::ScalarType> dum; return dum;}
static bool HasEdgePlane()   {   return false; }

static void Name(std::vector<std::string> & name){name.push_back(std::string(""));T::Name(name);}
};
/**
  @}
*/

  } // end namespace face
}// end namespace vcg
#endif
