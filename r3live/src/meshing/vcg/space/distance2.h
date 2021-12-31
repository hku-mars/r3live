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
/****************************************************************************/

#ifndef __VCG_DISTANCE2
#define __VCG_DISTANCE2

#include <limits>
#include <vcg/space/segment2.h>
#include <vcg/space/intersection2.h>

namespace vcg {

	/*
	* Computes the minimum distance between a two segments
	* @param[in] S0				The input segment0
	* @param[in] S1				The input segment1
	* return the distance between the two segments
	*/
	template<class ScalarType>
	ScalarType Segment2DSegment2DDistance(const vcg::Segment2<ScalarType> &S0,
									const vcg::Segment2<ScalarType> &S1,
									vcg::Point2<ScalarType> &p_clos)
	{
		///first test if they intersect
		if (vcg:: SegmentSegmentIntersection(S0,S1,p_clos))return 0;
		vcg::Point2<ScalarType> Pclos0=ClosestPoint(S0,S1.P0());
		vcg::Point2<ScalarType> Pclos1=ClosestPoint(S0,S1.P1());
		vcg::Point2<ScalarType> Pclos2=ClosestPoint(S1,S0.P0());
		vcg::Point2<ScalarType> Pclos3=ClosestPoint(S1,S0.P1());
		ScalarType d0=(Pclos0-S1.P0()).Norm();
		ScalarType d1=(Pclos1-S1.P1()).Norm();
		ScalarType d2=(Pclos2-S0.P0()).Norm();
		ScalarType d3=(Pclos3-S0.P1()).Norm();
		///then return the minimuim distance
		return (std::min(d0,std::min(d1,std::min(d2,d3))));
	}


}///end namespace vcg

#endif
