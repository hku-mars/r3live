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

#ifndef __VCGLIB_PLANAR_POLYGON_TESSELLATOR
#define __VCGLIB_PLANAR_POLYGON_TESSELLATOR

#include <assert.h>
#include <vcg/space/segment2.h>
#include <vcg/math/random_generator.h>

namespace vcg {

/** \addtogroup space */
/*@{*/
    /**
	A very simple earcut tessellation of planar 2D polygon.
	Input: a vector or Point2<>
	Output: a vector of faces as a triple of indices to the input vector
	*/
	template <class ScalarType> 
	bool Cross(	 const Point2<ScalarType> & p00,
				 const Point2<ScalarType> & p01,
				 const Point2<ScalarType> & p10,
				 const Point2<ScalarType> & p11)
	{
		Point2<ScalarType> vec0 = p01-p00;
		Point2<ScalarType> vec1 = p11-p10;
		if ( ( vec0^ (p11-p00)) *  ( vec0^ (p10 - p00)) >=0) return false;
		if ( ( vec1^ (p01-p10)) *  ( vec1^ (p00 - p10)) >=0) return false;
		return true;
	}

	template <class S>
	bool Intersect(size_t cur , int v2, std::vector<int> & next, std::vector<Point2<S> > & points2){
		for(size_t i  = 0; i < points2.size();++i)
			if( (next[i]!=-1) && (i!=cur))
				if( Cross(points2[cur], points2[v2],points2[i],points2[next[i]]))
					return true;
		return false;
	}


	template <class POINT_CONTAINER>
	void TessellatePlanarPolygon2( POINT_CONTAINER &  points2, std::vector<int> & output){
		typedef typename POINT_CONTAINER::value_type Point2x;
		typedef typename Point2x::ScalarType S;
		// tessellate
		//  first very inefficient implementation
		std::vector<int> next,prev;
		for(size_t i = 0; i < points2.size(); ++i) next.push_back((i+1)%points2.size());
		for(size_t i = 0; i < points2.size(); ++i) prev.push_back((i+points2.size()-1)%points2.size());
		int v1,v2;
		// check orientation
		S orient = 0.0;
		for(size_t i = 0 ; i < points2.size(); ++i){
			v1 =  next[i];
			v2 =  next[v1];
			orient+= (points2[v1] - points2[0]) ^ (points2[v2] - points2[0]);
		}
		orient = (orient>0)? 1.0:-1.0;

		int cur = 0;
		while(output.size()<3*(points2.size()-2)){
			v1 =  next[cur];
			v2 =  next[v1];
			if( ( (orient*((points2[v1] - points2[cur]) ^ (points2[v2] - points2[cur]))) >= 0.0) && 
				  !Intersect(cur, v2,next,points2))
			{
				// output the face
				output.push_back(cur);
				output.push_back(v1);
				output.push_back(v2);

				// readjust the topology
				next[cur] = v2;
				prev[v2] = cur;
				prev[v1] = -1;//unnecessary
				next[v1] = -1;//unnecessary
			} 
			else
				do{cur = (cur+1)%points2.size();} while(next[cur]==-1);
		}
	}

    /**
	A very simple earcut tessellation of planar 2D polygon.
	Input: a vector or Point3<>
	Output: a vector of faces as a triple of indices to the input vector

	*/

	template <class POINT_CONTAINER>
	void TessellatePlanarPolygon3( POINT_CONTAINER &  points, std::vector<int> & output){
		typedef typename POINT_CONTAINER::value_type Point3x;
		typedef typename Point3x::ScalarType S;
		Point3x n;

		math::SubtractiveRingRNG rg;
		size_t i12[2];
		S bestsn = -1.0;
		Point3x bestn,u,v;
		for(size_t i  =0; i < points.size();++i){
			for(size_t j = 0; j < 2; ++j){ i12[j] = i; while(i12[j]==i) i12[j] = rg.generate(points.size()-1);}
			n = (points[i12[0]]-points[i])^(points[i12[1]]-points[i]);
			S sn = n.SquaredNorm();
			if(sn > bestsn){ bestsn = sn; bestn = n;} 
		}
		
		GetUV(n,u,v);
		// project the coordinates
		std::vector<Point2<S> > points2;
		for(size_t i = 0; i < points.size(); ++i){
			Point3x & p = points[i];
			points2.push_back(Point2<S>(p*u,p*v));
		}
		TessellatePlanarPolygon2( points2,output);
	}

/*@}*/
} // end namespace
#endif
