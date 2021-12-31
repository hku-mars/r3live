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

#ifndef __VCG_DISTANCE3
#define __VCG_DISTANCE3

#include <limits>
#include <vcg/space/intersection3.h>

namespace vcg {


/*
* Computes the minimum distance between a 3D box and a point
* @param[in] p				The input point
* @param[in] b				The input bounding box
* return			The distance
* This function returns 0 for points Inside the bbox while the next one return the distance from the surface
*/
template<class Scalar>
Scalar PointFilledBoxDistance(const Point3<Scalar> &p, const Box3<Scalar> &bbox)
{
	Scalar dist2 = 0.;
	Scalar aux;
	for (int k=0 ; k<3 ; ++k)
	{
		if ( (aux = (p[k]-bbox.min[k]))<0. )
			dist2 += aux*aux;
		else if ( (aux = (bbox.max[k]-p[k]))<0. )
			dist2 += aux*aux;
	}
	return sqrt(dist2);
}

/*
* Computes the minimum distance between a 3D box and a point
* @param[in] p				The input point
* @param[in] b				The input bounding box
* @param[out] dist			The distance
* Note that this function with respect to the previous one compute the distance between a point
* and the 'surface' of a Box3. 
* 
*/
template <class ScalarType> 
void PointBoxDistance(const Point3<ScalarType> &p,
										const Box3<ScalarType> &b,
										ScalarType& dist)
{
	///if fall inside return distance to a face
	if (b.IsIn(p))
	{
		const ScalarType dx = std::min<ScalarType>(b.max.X()-p.X(), p.X()-b.min.X());
		const ScalarType dy = std::min<ScalarType>(b.max.Y()-p.Y(), p.Y()-b.min.Y());
		const ScalarType dz = std::min<ScalarType>(b.max.Z()-p.Z(), p.Z()-b.min.Z());

		dist= std::min<ScalarType>(dx, std::min<ScalarType>(dy, dz));
		return;
	}
    
	{
		ScalarType sq_dist = ScalarType(0);
		for (int i=0; i<3; ++i)
		{
			ScalarType delta = ScalarType(0);
			if       (p[i] < b.min[i])  delta = p[i] - b.min[i];
			else if  (p[i] > b.max[i])  delta = p[i] - b.max[i];
			sq_dist += delta * delta;
		}

    dist= math::Sqrt(sq_dist);
	}
}

/*
* Computes the minimum distance between a sphere and a point
* @param[in] p				The input point
* @param[in] sphere		The input sphere
* @param[out] dist			The distance
*/
template <class ScalarType> 
void SpherePointDistance(const Sphere3<ScalarType> &sphere, 
															 const Point3<ScalarType> &p,
															 ScalarType& dist) 
{
  dist = Distance(p, sphere.Center()) - sphere.Radius();
  if(dist < 0) dist = 0;
}

/*
* Computes the minimum distance between two spheres
* @param[in] sphere0				The input sphere
* @param[in] sphere1				The input sphere
* @param[out] dist			The distance
*/
template <class ScalarType> 
void SphereSphereDistance(const Sphere3<ScalarType> &sphere0, 
													const Sphere3<ScalarType> &sphere1,
													ScalarType& dist) 
{
  dist = (sphere1.Center()-sphere0.Center()).Norm()
                    - sphere0.Radius() - sphere1.Radius();
  if(dist < 0) dist = 0;
  return dist;
}


/*
* Computes the minimum squared distance between a between a point and a line
* @param[in] l				The input line
* @param[in] p				The input point
* @param[out] closest	The closest point
* @param[out] dist		The squared distance
*/
template <class ScalarType> 
void  LinePointSquaredDistance(const Line3<ScalarType> &l,const Point3<ScalarType> &p,
												Point3<ScalarType> &closest,ScalarType &dist) 
{
	closest=l.P(l.Projection(p));
  dist= (closest - p).SquaredNorm();
}

/*
* Computes the minimum  distance between a between a point and a line
* @param[in] l				The input line
* @param[in] p				The input point
* @param[out] closest				The closest point
* @param[out] dist			The distance
*/
template <class ScalarType> 
void  LinePointDistance(const Line3<ScalarType> &l,const Point3<ScalarType> &p,
												Point3<ScalarType> &closest,ScalarType &dist) 
{
	LinePointSquaredDistance(l,p,closest,dist);
	dist=sqrt(dist);
}

/*
* Computes the minimum  distance between two lines
* @param[in] mLine0				The input line0
* @param[in] mLine1				The input line1
* @param[out] parallel		true if the two lines are parallel
* @param[mClosestPoint0]  the closest point on line0
* @param[mClosestPoint1]  the closest point on line1
*/
template <class ScalarType>
void LineLineDistance(const vcg::Line3<ScalarType> &mLine0,
											const vcg::Line3<ScalarType> &mLine1,
											bool &parallel,
											ScalarType &dist,
											vcg::Point3<ScalarType> &mClosestPoint0,
											vcg::Point3<ScalarType> &mClosestPoint1)
{
  const ScalarType loc_EPSILON=ScalarType(0.000000001);
	typedef typename vcg::Point3<ScalarType> CoordType;
  CoordType diff = mLine0.Origin() - mLine1.Origin();
  ScalarType a01 = -mLine0.Direction()* mLine1.Direction();
  ScalarType b0 = diff *(mLine0.Direction());
  ScalarType c = diff.SquaredNorm();
  ScalarType det = fabs((ScalarType)1 - a01*a01);
  ScalarType b1, s0, s1, sqrDist;

    if (det >=loc_EPSILON)
    {
        // Lines are not parallel.
        b1 = -diff*(mLine1.Direction());
        ScalarType invDet = ((ScalarType)1)/det;
        s0 = (a01*b1 - b0)*invDet;
        s1 = (a01*b0 - b1)*invDet;
        sqrDist = s0*(s0 + a01*s1 + ((ScalarType)2)*b0) +
            s1*(a01*s0 + s1 + ((ScalarType)2)*b1) + c;
				parallel=false;
    }
    else
    {
        // Lines are parallel, select any closest pair of points.
        s0 = -b0;
        s1 = (ScalarType)0;
        sqrDist = b0*s0 + c;
				parallel=true;
    }
		///find the two closest points
    mClosestPoint0 = mLine0.Origin() + mLine0.Direction()*s0;
    mClosestPoint1 = mLine1.Origin() + mLine1.Direction()*s1;
    /*mLine0Parameter = s0;
    mLine1Parameter = s1;*/

    // Account for numerical round-off errors.
    if (sqrDist < (ScalarType)0)
    {
        sqrDist = (ScalarType)0;
    }
    dist=sqrt(sqrDist);
}


/*
* Computes the minimum distance between a segment and a point
* @param[in] segment	The input segment
* @param[in] p				The input point
* @param[in] clos			The closest point
* @param[in] sqr_dist The squared distance
*/
template <class ScalarType> 
void SegmentPointSquaredDistance( const Segment3<ScalarType> &s,
																							 const Point3<ScalarType> & p,
																							 Point3< ScalarType > &closest,
																							 ScalarType &sqr_dist) 
{
	Point3<ScalarType> e = s.P1()-s.P0();
	ScalarType eSquaredNorm = e.SquaredNorm();
	if (eSquaredNorm < std::numeric_limits<ScalarType>::min())
	{
		closest=s.MidPoint();
		sqr_dist=SquaredDistance(closest,p);
	}
	else
	{
		ScalarType  t = ((p-s.P0())*e)/eSquaredNorm;
		if(t<0)      t = 0;
		else if(t>1) t = 1;
		closest = s.P0()+e*t;
		sqr_dist = SquaredDistance(p,closest);
		assert(!math::IsNAN(sqr_dist));
	}
}

/*
* Computes the minimum distance between a segment and a point
* @param[in] segment	The input segment
* @param[in] p				The input point
* @param[in] clos			The closest point
* @param[in] dist The distance
*/
template <class ScalarType> 
void SegmentPointDistance( Segment3<ScalarType> s, 
													const Point3<ScalarType> & p,
													Point3< ScalarType > &clos,
													ScalarType &dist) 
{
	SegmentPointSquaredDistance(s,p,clos,dist);
	dist=sqrt(dist);
}

/*
* Computes the minimum distance between two segments
* @param[in] s0				The input segment0
* @param[in] s1				The input segment1
* @param[out] parallel		true if the two segments are parallel
* @param[out] dist		the distance
* @param[closest0]  the closest point on segment0
* @param[closest1]  the closest point on segment1
*/
template <class ScalarType>
void SegmentSegmentDistance(const vcg::Segment3<ScalarType> &s0,
														const vcg::Segment3<ScalarType> &s1,
														ScalarType &dist,
														bool &parallel,
														vcg::Point3<ScalarType> &closest0,
														vcg::Point3<ScalarType> &closest1)

{
	typedef typename vcg::Point3<ScalarType> CoordType;

	vcg::Line3<ScalarType> l0,l1;

	///construct two lines
	l0.SetOrigin(s0.P0());
	l0.SetDirection((s0.P1()-s0.P0()).Normalize());

	l1.SetOrigin(s1.P0());
	l1.SetDirection((s1.P1()-s1.P0()).Normalize());

	///then find closest point
	ScalarType line_dist;
	CoordType closest_test0,closest_test1;
	LineLineDistance(l0,l1,parallel,line_dist,closest_test0,closest_test1);
	///special case if the two lines are parallel
	if (parallel)
	{
		///find the minimum distance between extremes to segments
		ScalarType dist_test;
		CoordType clos_test;
    //CoordType to_test[4]={s1.P0(),s1.P1(),s0.P0(),s1.P1()};

		///find combination of distances between all extremes and segments
		SegmentPointSquaredDistance(s0,s1.P0(),clos_test,dist);
		closest0=clos_test;
		closest1=s1.P0();
		///and find the minimum updating coherently the closest points
		SegmentPointSquaredDistance(s0,s1.P1(),clos_test,dist_test);
		if (dist_test<dist)
		{
			dist=dist_test;
			closest0=clos_test;
			closest1=s1.P1();
		}
		SegmentPointSquaredDistance(s1,s0.P0(),clos_test,dist_test);
		if (dist_test<dist)
		{
			dist=dist_test;
			closest0=s0.P0();
			closest1=clos_test;
		}
		SegmentPointSquaredDistance(s1,s0.P1(),clos_test,dist_test);
		if (dist_test<dist)
		{
			dist=dist_test;
			closest0=s0.P1();
			closest1=clos_test;
		}
		dist=sqrt(dist);
		return;
	}

	///then ffind the closest segments points... 
	///means that if it is not an extreme then take 
	///the closest extreme
	ScalarType sqr_dist0;
	SegmentPointSquaredDistance(s0,closest_test0,closest0,sqr_dist0);
	ScalarType sqr_dist1;
	SegmentPointSquaredDistance(s1,closest_test1,closest1,sqr_dist1);

	///then return the distance
	dist=(closest0-closest1).Norm();
}

 /* @brief Computes the distance between a triangle and a point.
 *
 * @param t         reference to the triangle
 * @param q         point location
 * @param dist      distance from p to t
 * @param closest   perpendicular projection of p onto t
 */
template<class ScalarType>
void TrianglePointDistance(const vcg::Triangle3<ScalarType> &t,
                           const typename vcg::Point3<ScalarType> & q,
                           ScalarType & dist,
                           typename vcg::Point3<ScalarType> & closest )
{
	typedef typename vcg::Point3<ScalarType> CoordType;

	CoordType clos[3];
	ScalarType distv[3];
	CoordType clos_proj;
	ScalarType distproj;

	///find distance on the plane
	vcg::Plane3<ScalarType> plane;
	plane.Init(t.P(0),t.P(1),t.P(2));
	clos_proj=plane.Projection(q);

	///control if inside/outside
	CoordType n=(t.P(1)-t.P(0))^(t.P(2)-t.P(0));
	CoordType n0=(t.P(0)-clos_proj)^(t.P(1)-clos_proj);
	CoordType n1=(t.P(1)-clos_proj)^(t.P(2)-clos_proj);
	CoordType n2=(t.P(2)-clos_proj)^(t.P(0)-clos_proj);
	distproj=(clos_proj-q).Norm();
	if (((n*n0)>=0)&&((n*n1)>=0)&&((n*n2)>=0))
	{
		closest=clos_proj;
		dist=distproj;
		return;
	}
	

	//distance from the edges
	vcg::Segment3<ScalarType> e0=vcg::Segment3<ScalarType>(t.P(0),t.P(1));
	vcg::Segment3<ScalarType> e1=vcg::Segment3<ScalarType>(t.P(1),t.P(2));
	vcg::Segment3<ScalarType> e2=vcg::Segment3<ScalarType>(t.P(2),t.P(0));
	SegmentPointDistance(e0,q,clos[0],distv[0]);
	SegmentPointDistance(e1,q,clos[1],distv[1]);
	SegmentPointDistance(e2,q,clos[2],distv[2]);
	/*clos[0]=ClosestPoint<ScalarType>( e0, q);
	clos[1]=ClosestPoint<ScalarType>( e1, q);
	clos[2]=ClosestPoint<ScalarType>( e2, q);
	*/
	//distv[0]=(clos[0]-q).Norm();
	//distv[1]=(clos[1]-q).Norm();
	//distv[2]=(clos[2]-q).Norm();
	int min=0;

	///find minimum distance
	for (int i=1;i<3;i++)
	{
		if (distv[i]<distv[min])
			min=i;
	}

	closest=clos[min];
	dist=distv[min];
}


/*
* return the distance between a triangle and a segment
* @param[in] t				The input triangle
* @param[in] s				The input segment
* @param[out] dist		the distance
*/
template<class ScalarType>
void TriangleSegmentDistance(const vcg::Triangle3<ScalarType> &t,
														 const vcg::Segment3<ScalarType> &s,
														 ScalarType & dist)
{
	dist=std::numeric_limits<ScalarType>::max();
	///test the intersection
	ScalarType a,b;
	typedef typename vcg::Point3<ScalarType> CoordType;

	bool intersect=IntersectionSegmentTriangle<vcg::Triangle3<ScalarType> >(s,t,a,b);
	if (intersect)
	{
		dist=0;
		return;
	}
	///project endpoints and see if they fall into the triangle
	vcg::Plane3<ScalarType> pl3;
	pl3.Init(t.P(0),t.P(1),t.P(2));
	CoordType pj0=pl3.Projection(s.P(0));
	CoordType pj1=pl3.Projection(s.P(1));
	///take distances
	ScalarType dpj0=(pj0-s.P(0)).Norm();
	ScalarType dpj1=(pj1-s.P(1)).Norm();

	///test if they fall inside the triangle
	CoordType bary0,bary1;
	bool Inside0=vcg::InterpolationParameters(t,pj0,bary0);
	bool Inside1=vcg::InterpolationParameters(t,pj1,bary1);
	if (Inside0&&Inside1)
	{
		dist=std::min(dpj0,dpj1);
		return;
	}
	///initialize with the sdistance if only once falls into
	if (Inside0)
		dist=dpj0;
	if (Inside1)
		dist=dpj1;

	///then test segment-to segment distance with edges of the triangle
	for (int i=0;i<3;i++)
 {
	 vcg::Segment3<ScalarType> edge=vcg::Segment3<ScalarType>(t.P0(i),t.P0((i+1)%3));
	 ScalarType test_dist;
	 CoordType clos1,clos2;
	 bool parallel;
	 vcg::SegmentSegmentDistance<ScalarType>(s,edge,test_dist,parallel,clos1,clos2);
	 if (test_dist<dist)
		 dist=test_dist;
 }
}

/*
* return the minimum distance between two triangles
* @param[in] t0				The input triangle0
* @param[in] t1				The input triangle1
* @param[out] dist		the distance
*/
template<class ScalarType>
void TriangleTriangleDistance(const  vcg::Triangle3<ScalarType> &t0,
															const  vcg::Triangle3<ScalarType> &t1,
															ScalarType &dist)
{
  const ScalarType loc_EPSILON=(vcg::DoubleArea(t0)+vcg::DoubleArea(t1))*(ScalarType)0.0000001;
 dist=std::numeric_limits<ScalarType>::max();

 ///test each segment of t1 with t0 
 ///keeping the minimum distance
 for (int i=0;i<3;i++)
 {
	 vcg::Segment3<ScalarType> edge=vcg::Segment3<ScalarType>(t0.P0(i),t0.P0((i+1)%3));
	 ScalarType test_dist;
	 vcg::TriangleSegmentDistance<ScalarType>(t1,edge,test_dist);
   if (test_dist<loc_EPSILON)
	 {
		 dist=0;
		 return;
	 }
	 if (test_dist<dist)
		 dist=test_dist;
 }
 ///then viceversa
 for (int i=0;i<3;i++)
 {
	 vcg::Segment3<ScalarType> edge=vcg::Segment3<ScalarType>(t1.P0(i),t1.P0((i+1)%3));
	 ScalarType test_dist;
	 vcg::TriangleSegmentDistance<ScalarType>(t0,edge,test_dist);
   if (test_dist<loc_EPSILON)
	 {
		 dist=0;
		 return;
	 }
	 if (test_dist<dist)
		 dist=test_dist;
 }
}

}///end namespace vcg

#endif
