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


****************************************************************************/

#ifndef __VCGLIB_EDGE_DISTANCE
#define __VCGLIB_EDGE_DISTANCE

#include <vcg/math/base.h>
#include <vcg/space/point3.h>
#include <vcg/space/segment3.h>


namespace vcg {
	namespace edge{
	/*Point edge distance*/

	template <class EdgeType>
	bool PointDistance(	const EdgeType &e, 
							const vcg::Point3<typename EdgeType::ScalarType> & q, 
							typename EdgeType::ScalarType & dist, 
							vcg::Point3<typename EdgeType::ScalarType> & p )
	{
		vcg::Segment3<typename EdgeType::ScalarType> s;
		s.P0()=e.V(0)->P();
		s.P1()=e.V(1)->P();
		typename EdgeType::CoordType nearest;
		nearest=vcg::ClosestPoint<typename EdgeType::ScalarType>(s,q);
		typename EdgeType::ScalarType d=(q-nearest).Norm();
		if (d<dist){
			dist=d;
			p=nearest;
			return true;
		}
		else 
			return false;
	}

	template <class S>
	class PointDistanceFunctor {
	public:
		typedef S ScalarType;
		typedef Point3<ScalarType> QueryType;
		static inline const Point3<ScalarType> &  Pos(const QueryType & qt)  {return qt;}
	
		template <class EDGETYPE, class SCALARTYPE>
		inline bool operator () (const EDGETYPE & e, const Point3<SCALARTYPE> & p, SCALARTYPE & minDist, Point3<SCALARTYPE> & q) {
			const Point3<typename EDGETYPE::ScalarType> fp = Point3<typename EDGETYPE::ScalarType>::Construct(p);
			Point3<typename EDGETYPE::ScalarType> fq;
			typename EDGETYPE::ScalarType md = (typename EDGETYPE::ScalarType)(minDist);
			const bool ret = vcg::edge::PointDistance(e, fp, md, fq);
			minDist = (SCALARTYPE)(md);
			q = Point3<SCALARTYPE>::Construct(fq);
			return (ret);
		}
	};

}	 // end namespace edge
	
}	 // end namespace vcg


#endif

