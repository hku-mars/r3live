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

#ifndef __VCGLIB_INTERSECTION_3
#define __VCGLIB_INTERSECTION_3

#include <vcg/math/base.h>
#include <vcg/space/point3.h>
#include <vcg/space/line3.h>
#include <vcg/space/ray3.h>
#include <vcg/space/plane3.h>
#include <vcg/space/segment3.h>
#include <vcg/space/sphere3.h>
#include <vcg/space/triangle3.h>
#include <vcg/space/intersection/triangle_triangle3.h>




namespace vcg {
/** \addtogroup space */
/*@{*/
/** 
    Function computing the intersection between couple of geometric primitives in
    3 dimension
*/
  /// interseciton between sphere and line
  template<class T>
    inline bool IntersectionLineSphere( const Sphere3<T> & sp, const Line3<T> & li, Point3<T> & p0,Point3<T> & p1 ){

    // Per prima cosa si sposta il sistema di riferimento 
    // fino a portare il centro della sfera nell'origine
    Point3<T> neworig=li.Origin()-sp.Center();
    // poi si risolve il sistema di secondo grado (con maple...)
    T t1 = li.Direction().X()*li.Direction().X();
    T t2 = li.Direction().Y()*li.Direction().Y();
    T t3 = li.Direction().Z()*li.Direction().Z();
    T t6 = neworig.Y()*li.Direction().Y();
    T t7 = neworig.X()*li.Direction().X();
    T t8 = neworig.Z()*li.Direction().Z();
    T t15 = sp.Radius()*sp.Radius();
    T t17 = neworig.Z()*neworig.Z();
    T t19 = neworig.Y()*neworig.Y();
    T t21 = neworig.X()*neworig.X();
    T t28 = T(2.0*t7*t6+2.0*t6*t8+2.0*t7*t8+t1*t15-t1*t17-t1*t19-t2*t21+t2*t15-t2*t17-t3*t21+t3*t15-t3*t19);
    if(t28<0) return false;
    T t29 = sqrt(t28);      
    T val0 = 1/(t1+t2+t3)*(-t6-t7-t8+t29); 
    T val1 = 1/(t1+t2+t3)*(-t6-t7-t8-t29);

    p0=li.P(val0);
    p1=li.P(val1);
    return true;
  }

	/*
	* Function computing the intersection between a sphere and a segment.
	* @param[in]	sphere				the sphere
	* @param[in]	segment				the segment
	* @param[out]	intersection  the intersection point, meaningful only if the segment intersects the sphere
	* \return			(0, 1 or 2)		the number of intersections between the segment and the sphere. 
	*														t1 is a valid intersection only if the returned value is at least 1; 
	*														similarly t2 is valid iff the returned value is 2.
	*/
	template < class SCALAR_TYPE >
	inline int IntersectionSegmentSphere(const Sphere3<SCALAR_TYPE>& sphere, const Segment3<SCALAR_TYPE>& segment, Point3<SCALAR_TYPE> & t0, Point3<SCALAR_TYPE> & t1)
	{
		typedef SCALAR_TYPE													ScalarType;
		typedef typename vcg::Point3< ScalarType >	Point3t;

		Point3t s = segment.P0() - sphere.Center();
		Point3t r = segment.P1() - segment.P0();

		ScalarType rho2 = sphere.Radius()*sphere.Radius();

		ScalarType sr = s*r;
		ScalarType r_squared_norm = r.SquaredNorm(); 
		ScalarType s_squared_norm = s.SquaredNorm(); 
		ScalarType sigma = sr*sr - r_squared_norm*(s_squared_norm-rho2);

		if (sigma<ScalarType(0.0)) // the line containing the edge doesn't intersect the sphere
			return 0;

		ScalarType sqrt_sigma = ScalarType(sqrt( ScalarType(sigma) ));
		ScalarType lambda1 = (-sr - sqrt_sigma)/r_squared_norm;
		ScalarType lambda2 = (-sr + sqrt_sigma)/r_squared_norm;

		int solution_count = 0;
		if (ScalarType(0.0)<=lambda1 && lambda1<=ScalarType(1.0))
		{
      ScalarType t_enter = std::max< ScalarType >(lambda1, ScalarType(0.0));
			t0 = segment.P0() + r*t_enter;
			solution_count++;
		}

		if (ScalarType(0.0)<=lambda2 && lambda2<=ScalarType(1.0))
		{
			Point3t *pt = (solution_count>0) ? &t1 : &t0;
      ScalarType t_exit  = std::min< ScalarType >(lambda2, ScalarType(1.0));
			*pt = segment.P0() + r*t_exit;
			solution_count++;
		}
		return solution_count;
	}; // end of IntersectionSegmentSphere

	
	/*!
	* Compute the intersection between a sphere and a triangle.
	* \param[in]	sphere		the input sphere
	* \param[in]	triangle	the input triangle
	* \param[out]	witness		it is the point on the triangle nearest to the center of the sphere (even when there isn't intersection)
	* \param[out] res				if not null, in the first item is stored the minimum distance between the triangle and the sphere,
	*                       while in the second item is stored the penetration depth
	* \return			true			iff there is an intersection between the sphere and the triangle
	*/
	template < class SCALAR_TYPE, class TRIANGLETYPE >
	bool IntersectionSphereTriangle(const vcg::Sphere3	< SCALAR_TYPE >		& sphere  ,
																	TRIANGLETYPE														triangle, 
																	vcg::Point3					< SCALAR_TYPE >		& witness ,
																	std::pair< SCALAR_TYPE, SCALAR_TYPE > * res=NULL)
	{
		typedef SCALAR_TYPE														ScalarType;
		typedef typename vcg::Point3< ScalarType >		Point3t;
		typedef TRIANGLETYPE Triangle3t;

		bool penetration_detected = false;

		ScalarType radius = sphere.Radius();
		Point3t	center = sphere.Center();
		Point3t p0 = triangle.P(0)-center;
		Point3t p1 = triangle.P(1)-center;
		Point3t p2 = triangle.P(2)-center;

		Point3t p10 = p1-p0;
		Point3t p21 = p2-p1;
		Point3t p20 = p2-p0;

		ScalarType delta0_p01 =  p10.dot(p1);
		ScalarType delta1_p01 = -p10.dot(p0);
		ScalarType delta0_p02 =  p20.dot(p2);
		ScalarType delta2_p02 = -p20.dot(p0);
		ScalarType delta1_p12 =  p21.dot(p2);
		ScalarType delta2_p12 = -p21.dot(p1);

		// the closest point can be one of the vertices of the triangle
		if			(delta1_p01<=ScalarType(0.0) && delta2_p02<=ScalarType(0.0)) { witness = p0; }
		else if (delta0_p01<=ScalarType(0.0) && delta2_p12<=ScalarType(0.0)) { witness = p1; }
		else if (delta0_p02<=ScalarType(0.0) && delta1_p12<=ScalarType(0.0)) { witness = p2; }
		else
		{
			ScalarType temp = p10.dot(p2);
			ScalarType delta0_p012 = delta0_p01*delta1_p12 + delta2_p12*temp;
			ScalarType delta1_p012 = delta1_p01*delta0_p02 - delta2_p02*temp;
			ScalarType delta2_p012 = delta2_p02*delta0_p01 - delta1_p01*(p20.dot(p1));

			// otherwise, can be a point lying on same edge of the triangle
			if (delta0_p012<=ScalarType(0.0)) 
			{
				ScalarType denominator = delta1_p12+delta2_p12;
				ScalarType mu1 = delta1_p12/denominator;
				ScalarType mu2 = delta2_p12/denominator;
				witness = (p1*mu1 + p2*mu2);
			}
			else if (delta1_p012<=ScalarType(0.0))
			{
				ScalarType denominator = delta0_p02+delta2_p02;
				ScalarType mu0 = delta0_p02/denominator;
				ScalarType mu2 = delta2_p02/denominator;
				witness = (p0*mu0 + p2*mu2);
			}
			else if (delta2_p012<=ScalarType(0.0))
			{
				ScalarType denominator = delta0_p01+delta1_p01;
				ScalarType mu0 = delta0_p01/denominator;
				ScalarType mu1 = delta1_p01/denominator;
				witness = (p0*mu0 + p1*mu1);
			}
			else
			{
				// or else can be an point internal to the triangle
				ScalarType denominator =  delta0_p012 + delta1_p012 + delta2_p012;
				ScalarType lambda0 = delta0_p012/denominator;
				ScalarType lambda1 = delta1_p012/denominator;
				ScalarType lambda2 = delta2_p012/denominator;
				witness = p0*lambda0 + p1*lambda1 + p2*lambda2;
			}
		}

		if (res!=NULL)
		{
			ScalarType witness_norm = witness.Norm();
      res->first  = std::max< ScalarType >( witness_norm-radius, ScalarType(0.0) );
      res->second = std::max< ScalarType >( radius-witness_norm, ScalarType(0.0) );
		}
		penetration_detected = (witness.SquaredNorm() <= (radius*radius));
		witness += center;
		return penetration_detected;
	}; //end of IntersectionSphereTriangle

  /// intersection between line and plane
  template<class T>
    inline bool IntersectionPlaneLine( const Plane3<T> & pl, const Line3<T> & li, Point3<T> & po){
    const T epsilon = T(1e-8);

    T k = pl.Direction().dot(li.Direction());						// Compute 'k' factor
    if( (k > -epsilon) && (k < epsilon))
      return false;
    T r = (pl.Offset() - pl.Direction().dot(li.Origin()))/k;	// Compute ray distance
    po = li.Origin() + li.Direction()*r;
    return true;
  }
    /// intersection between line and plane
    template<class T>
    inline bool IntersectionLinePlane(const Line3<T> & li, const Plane3<T> & pl, Point3<T> & po){

      return IntersectionPlaneLine(pl,li,po);
    }

   /// intersection between segment and plane
  template<class T>
    inline bool IntersectionPlaneSegment( const Plane3<T> & pl, const Segment3<T> & s, Point3<T> & p0){
		T p1_proj = s.P1()*pl.Direction()-pl.Offset();
		T p0_proj = s.P0()*pl.Direction()-pl.Offset();
    if ( (p1_proj>0)-(p0_proj<0)) return false;

    if(p0_proj == p1_proj) return false;

    // check that we perform the computation in a way that is independent with v0 v1 swaps
    if(p0_proj < p1_proj)
           p0 =  s.P0() + (s.P1()-s.P0()) * fabs(p0_proj/(p1_proj-p0_proj));
    if(p0_proj > p1_proj)
           p0 =  s.P1() + (s.P0()-s.P1()) * fabs(p1_proj/(p0_proj-p1_proj));

		return true;
  }

  /// intersection between segment and plane
  template<class ScalarType>
    inline bool IntersectionPlaneSegmentEpsilon(const Plane3<ScalarType> & pl,
                                                const Segment3<ScalarType> & sg,
                                                Point3<ScalarType> & po,
                                                const ScalarType epsilon = ScalarType(1e-8)){

    typedef ScalarType T;
    T k = pl.Direction().dot((sg.P1()-sg.P0()));
    if( (k > -epsilon) && (k < epsilon))
      return false;
    T r = (pl.Offset() - pl.Direction().dot(sg.P0()))/k;	// Compute ray distance
    if( (r<0) || (r > 1.0))
      return false;
    po = sg.P0()*(1-r)+sg.P1() * r;
    return true;
  }

  /// intersection between plane and triangle 
  // not optimal: uses plane-segment intersection (and the fact the two or none edges can be intersected)
  // its use is rather dangerous because it can return inconsistent stuff on degenerate cases.
  // added assert to underline this danger.
  template<typename TRIANGLETYPE> 
    inline bool IntersectionPlaneTriangle( const Plane3<typename TRIANGLETYPE::ScalarType> & pl, 
			      const  TRIANGLETYPE & tr, 
            Segment3<typename TRIANGLETYPE::ScalarType> & sg)
    {
      typedef typename TRIANGLETYPE::ScalarType T;
      if(IntersectionPlaneSegment(pl,Segment3<T>(tr.cP(0),tr.cP(1)),sg.P0()))
      {
        if(IntersectionPlaneSegment(pl,Segment3<T>(tr.cP(0),tr.cP(2)),sg.P1()))
          return true;
        else
        {
          if(IntersectionPlaneSegment(pl,Segment3<T>(tr.cP(1),tr.cP(2)),sg.P1()))
            return true;
          else assert(0);
              return true;
        }
      }
      else
      {
        if(IntersectionPlaneSegment(pl,Segment3<T>(tr.cP(1),tr.cP(2)),sg.P0()))
        {
          if(IntersectionPlaneSegment(pl,Segment3<T>(tr.cP(0),tr.cP(2)),sg.P1()))return true;
          assert(0);
          return true;
        }
      }
      return false;
    }

    /// intersection between two triangles
    template<typename TRIANGLETYPE>
    inline bool IntersectionTriangleTriangle(const TRIANGLETYPE & t0,const TRIANGLETYPE & t1){
    	typedef typename TRIANGLETYPE::ScalarType ScalarType;
#ifndef USE_MOLLER
		return GuigueTriTri(t0.cP(0), t0.cP(1), t0.cP(2),
							t1.cP(0), t1.cP(1), t1.cP(2));
#else
       return NoDivTriTriIsect(t0.cP(0),t0.cP(1),t0.cP(2),
                               t1.cP(0),t1.cP(1),t1.cP(2));
#endif
    }

  template<class T>
    inline bool IntersectionTriangleTriangle(Point3<T> V0,Point3<T> V1,Point3<T> V2,
			     Point3<T> U0,Point3<T> U1,Point3<T> U2){
#ifndef USE_MOLLER
    return GuigueTriTri(V0,V1,V2,U0,U1,U2);
#else
    return NoDivTriTriIsect(V0,V1,V2,U0,U1,U2);
#endif
  }
#if 0
  template<class T>
    inline bool Intersection(Point3<T> V0,Point3<T> V1,Point3<T> V2,
			     Point3<T> U0,Point3<T> U1,Point3<T> U2,int *coplanar,
			     Point3<T> &isectpt1,Point3<T> &isectpt2){

    return tri_tri_intersect_with_isectline(V0,V1,V2,U0,U1,U2,
					    coplanar,isectpt1,isectpt2);
  }

  template<typename TRIANGLETYPE,typename SEGMENTTYPE >
    inline bool Intersection(const TRIANGLETYPE & t0,const TRIANGLETYPE & t1,bool &coplanar,
			     SEGMENTTYPE  & sg){
    Point3<typename SEGMENTTYPE::PointType> ip0,ip1; 
    return  tri_tri_intersect_with_isectline(t0.P0(0),t0.P0(1),t0.P0(2),
					     t1.P0(0),t1.P0(1),t1.P0(2),
					     coplanar,sg.P0(),sg.P1()
					     );              
  }

#endif	
	 
	/*
	* Function computing the intersection between a line and a triangle.
	* from: 
	* Tomas Moller and Ben Trumbore,  
	* ``Fast, Minimum Storage Ray-Triangle Intersection'',	
	* journal of graphics tools, vol. 2, no. 1, pp. 21-28, 1997
	* @param[in]	line				 
	* @param[in]	triangle vertices			 
	* @param[out]=(t,u,v)	the intersection point, meaningful only if the line intersects the triangle
	*            t is the line parameter and
	*            (u,v) are the baricentric coords of the intersection point
	*
	*                 Line.Orig + t * Line.Dir = (1-u-v) * Vert0 + u * Vert1 +v * Vert2 
	*
	*/

template<class T>
bool IntersectionLineTriangle( const Line3<T> & line, const Point3<T> & vert0, 
				  const Point3<T> & vert1, const Point3<T> & vert2,
				  T & t ,T & u, T & v)
{
	#define EPSIL 0.000001

	vcg::Point3<T> edge1, edge2, tvec, pvec, qvec;
	T det,inv_det;

   /* find vectors for two edges sharing vert0 */
   edge1 = vert1 - vert0;
   edge2 = vert2 - vert0;

   /* begin calculating determinant - also used to calculate U parameter */
   pvec =  line.Direction() ^ edge2;

   /* if determinant is near zero, line lies in plane of triangle */
   det =  edge1 *  pvec;

   /* calculate distance from vert0 to line origin */
   tvec = line.Origin() - vert0;
   inv_det = 1.0 / det;

   qvec = tvec ^ edge1;

   if (det > EPSIL)
   {
      u =  tvec * pvec ;
      if ( u < 0.0 ||  u > det)
        return 0;
            
      /* calculate V parameter and test bounds */
       v =  line.Direction() * qvec;
      if ( v < 0.0 ||  u +  v > det)
        return 0;
      
   }
   else if(det < -EPSIL)
   {
      /* calculate U parameter and test bounds */
       u =  tvec * pvec ;
      if ( u > 0.0 ||  u < det)
        return 0;
      
      /* calculate V parameter and test bounds */
       v =  line.Direction() * qvec  ;
      if ( v > 0.0 ||  u +  v < det)
        return 0;
   }
   else return 0;  /* line is parallell to the plane of the triangle */

    t = edge2 *  qvec  * inv_det;
   ( u) *= inv_det;
   ( v) *= inv_det;

   return 1;
}

template<class T>
bool IntersectionRayTriangle( const Ray3<T> & ray, const Point3<T> & vert0, 
				  const Point3<T> & vert1, const Point3<T> & vert2,
				  T & t ,T & u, T & v)
{
	Line3<T> line(ray.Origin(), ray.Direction());
	if (IntersectionLineTriangle(line, vert0, vert1, vert2, t, u, v))
	{
		if (t < 0) return 0;
		else return 1; 
	}else return 0;
}

// line-box
template<class T>
bool IntersectionLineBox( const Box3<T> & box, const Line3<T> & r, Point3<T> & coord )
{
	const int NUMDIM = 3;
	const int RIGHT  = 0;
	const int LEFT	 = 1;
	const int MIDDLE = 2;

	int inside = 1;
	char quadrant[NUMDIM];
    int i;
    int whichPlane;
    Point3<T> maxT,candidatePlane;
    
	// Find candidate planes; this loop can be avoided if
   	// rays cast all from the eye(assume perpsective view)
    for (i=0; i<NUMDIM; i++)
    {
        if(r.Origin()[i] < box.min[i])
		{
			quadrant[i] = LEFT;
			candidatePlane[i] = box.min[i];
			inside = 0;
		}
		else if (r.Origin()[i] > box.max[i])
		{
			quadrant[i] = RIGHT;
			candidatePlane[i] = box.max[i];
			inside = 0;
		}
		else
		{
			quadrant[i] = MIDDLE;
		}
    }

		// Ray origin inside bounding box
	if(inside){
	    coord = r.Origin();
	    return true;
	}

	// Calculate T distances to candidate planes 
    for (i = 0; i < NUMDIM; i++)
    {
		if (quadrant[i] != MIDDLE && r.Direction()[i] !=0.)
			maxT[i] = (candidatePlane[i]-r.Origin()[i]) / r.Direction()[i];
		else
			maxT[i] = -1.;
    }

	// Get largest of the maxT's for final choice of intersection
    whichPlane = 0;
    for (i = 1; i < NUMDIM; i++)
	    if (maxT[whichPlane] < maxT[i])
			whichPlane = i;

	// Check final candidate actually inside box 
    if (maxT[whichPlane] < 0.) return false;
    for (i = 0; i < NUMDIM; i++)
		if (whichPlane != i)
		{
			coord[i] = r.Origin()[i] + maxT[whichPlane] *r.Direction()[i];
			if (coord[i] < box.min[i] || coord[i] > box.max[i])
				return false;
		}
		else
		{
			coord[i] = candidatePlane[i];
		}
    return true;			// ray hits box
}	

// ray-box
template<class T>
bool IntersectionRayBox( const Box3<T> & box, const Ray3<T> & r, Point3<T> & coord )
{
	Line3<T> l;
	l.SetOrigin(r.Origin());
	l.SetDirection(r.Direction());
  return(IntersectionLineBox<T>(box,l,coord));
}	

// segment-box return fist intersection found  from p0 to p1
template<class ScalarType>
bool IntersectionSegmentBox( const Box3<ScalarType> & box,
							  const Segment3<ScalarType> & s, 
							  Point3<ScalarType> & coord )
{
	//as first perform box-box intersection
	Box3<ScalarType> test;
	test.Add(s.P0());
	test.Add(s.P1());
	if (!test.Collide(box))
		return false;
	else
	{
		Line3<ScalarType> l;
		Point3<ScalarType> dir=s.P1()-s.P0();
		dir.Normalize();
		l.SetOrigin(s.P0());
		l.SetDirection(dir);
    if(IntersectionLineBox<ScalarType>(box,l,coord))
			return (test.IsIn(coord));
		return false;
	}
}

// segment-box intersection , return number of intersections and intersection points
template<class ScalarType>
int IntersectionSegmentBox( const Box3<ScalarType> & box,
							 const Segment3<ScalarType> & s,
							 Point3<ScalarType> & coord0,
							 Point3<ScalarType> & coord1 )
{
	int num=0;
	Segment3<ScalarType> test= s;
  if (IntersectionSegmentBox(box,test,coord0 ))
	{
		num++;
		Point3<ScalarType> swap=test.P0();
		test.P0()=test.P1();
		test.P1()=swap;
    if (IntersectionSegmentBox(box,test,coord1 ))
			num++;
	}
	return num;
}	

/**
* Compute the intersection between a segment and a triangle.
* It relies on the lineTriangle Intersection
* @param[in]	segment
* @param[in]	triangle vertices
* @param[out]=(t,u,v)	the intersection point, meaningful only if the line of segment intersects the triangle
*            t     is the baricentric coord of the point on the segment
*            (u,v) are the baricentric coords of the intersection point in the segment
*
*/
template<class ScalarType>
bool IntersectionSegmentTriangle( const vcg::Segment3<ScalarType> & seg,
								   const Point3<ScalarType> & vert0, 
									const Point3<ScalarType> & vert1, const
									Point3<ScalarType> & vert2,
									ScalarType & a ,ScalarType & b)
{
	//control intersection of bounding boxes
	vcg::Box3<ScalarType> bb0,bb1;
	bb0.Add(seg.P0());
	bb0.Add(seg.P1());
	bb1.Add(vert0);
	bb1.Add(vert1);
	bb1.Add(vert2);
	Point3<ScalarType> inter;
	if (!bb0.Collide(bb1))
		return false;
  if (!vcg::IntersectionSegmentBox(bb1,seg,inter))
		return false;

	//first set both directions of ray
  vcg::Line3<ScalarType> line;
	vcg::Point3<ScalarType> dir;
	ScalarType length=seg.Length();
	dir=(seg.P1()-seg.P0());
	dir.Normalize();
  line.Set(seg.P0(),dir);
	ScalarType orig_dist;
  if(IntersectionLineTriangle<ScalarType>(line,vert0,vert1,vert2,orig_dist,a,b))
    return (orig_dist>=0 && orig_dist<=length);
  return false;
}
/**
* Compute the intersection between a segment and a triangle.
* Wrapper of the above function
*/
template<class TriangleType>
bool IntersectionSegmentTriangle( const vcg::Segment3<typename TriangleType::ScalarType> & seg,
                  const TriangleType &t,
                  typename TriangleType::ScalarType & a ,typename TriangleType::ScalarType & b)
{
  return IntersectionSegmentTriangle(seg,t.cP(0),t.cP(1),t.cP(2),a,b);
}

template<class ScalarType>
bool IntersectionPlaneBox(const vcg::Plane3<ScalarType> &pl,
							vcg::Box3<ScalarType> &bbox)
{
	ScalarType dist,dist1;
	if(bbox.IsNull()) return false; // intersection with a  null bbox is empty
	dist = SignedDistancePlanePoint(pl,bbox.P(0)) ;
	for (int i=1;i<8;i++)  if(  SignedDistancePlanePoint(pl,bbox.P(i))*dist<0) return true;
	return true;
}

template<class ScalarType>
bool IntersectionTriangleBox(const vcg::Box3<ScalarType>   &bbox,
							   const vcg::Point3<ScalarType> &p0,
							   const vcg::Point3<ScalarType> &p1,
							   const vcg::Point3<ScalarType> &p2)
{
	typedef typename vcg::Point3<ScalarType> CoordType;
	CoordType intersection;
	/// control bounding box collision
	vcg::Box3<ScalarType> test;
	test.SetNull();
	test.Add(p0);
	test.Add(p1);
	test.Add(p2);
	if (!test.Collide(bbox))
		return false;
	/// control if each point is inside the bouding box
	if ((bbox.IsIn(p0))||(bbox.IsIn(p1))||(bbox.IsIn(p2)))
		return true;

	/////control plane of the triangle with bbox
	//vcg::Plane3<ScalarType> plTri=vcg::Plane3<ScalarType>();
	//plTri.Init(p0,p1,p2);
  //if (!IntersectionPlaneBox<ScalarType>(plTri,bbox))
	//	return false;

	///then control intersection of segments with box
  if (IntersectionSegmentBox<ScalarType>(bbox,vcg::Segment3<ScalarType>(p0,p1),intersection)||
    IntersectionSegmentBox<ScalarType>(bbox,vcg::Segment3<ScalarType>(p1,p2),intersection)||
    IntersectionSegmentBox<ScalarType>(bbox,vcg::Segment3<ScalarType>(p2,p0),intersection))
		return true;
	///control intersection of diagonal of the cube with triangle

	Segment3<ScalarType> diag[4];

	diag[0]=Segment3<ScalarType>(bbox.P(0),bbox.P(7));
	diag[1]=Segment3<ScalarType>(bbox.P(1),bbox.P(6));
	diag[2]=Segment3<ScalarType>(bbox.P(2),bbox.P(5));
	diag[3]=Segment3<ScalarType>(bbox.P(3),bbox.P(4));
	ScalarType a,b,dist;
	for (int i=0;i<3;i++)
    if (IntersectionSegmentTriangle<ScalarType>(diag[i],p0,p1,p2,a,b,dist))
			return true;

	return false;
}

template <class SphereType>
bool IntersectionSphereSphere( const SphereType & s0,const SphereType & s1){
	return (s0.Center()-s1.Center()).SquaredNorm() < (s0.Radius()+s1.Radius())*(s0.Radius()+s1.Radius());
}

template<class T>
bool IntersectionPlanePlane (const Plane3<T> & plane0, const Plane3<T> & plane1,
                             Line3<T> & line)
{
    // If Cross(N0,N1) is zero, then either planes are parallel and separated
    // or the same plane.  In both cases, 'false' is returned.  Otherwise,
    // the intersection line is
    //
    //   L(t) = t*Cross(N0,N1) + c0*N0 + c1*N1
    //
    // for some coefficients c0 and c1 and for t any real number (the line
    // parameter).  Taking dot products with the normals,
    //
    //   d0 = Dot(N0,L) = c0*Dot(N0,N0) + c1*Dot(N0,N1)
    //   d1 = Dot(N1,L) = c0*Dot(N0,N1) + c1*Dot(N1,N1)
    //
    // which are two equations in two unknowns.  The solution is
    //
    //   c0 = (Dot(N1,N1)*d0 - Dot(N0,N1)*d1)/det
    //   c1 = (Dot(N0,N0)*d1 - Dot(N0,N1)*d0)/det
    //
    // where det = Dot(N0,N0)*Dot(N1,N1)-Dot(N0,N1)^2.

    T n00 = plane0.Direction()*plane0.Direction();
    T n01 = plane0.Direction()*plane1.Direction();
    T n11 = plane1.Direction()*plane1.Direction();
    T det = n00*n11-n01*n01;

    const T tolerance = (T)(1e-06f);
		if ( math::Abs(det) < tolerance )
        return false;

    T invDet = 1.0f/det;
    T c0 = (n11*plane0.Offset() - n01*plane1.Offset())*invDet;
    T c1 = (n00*plane1.Offset() - n01*plane0.Offset())*invDet;

    line.SetDirection(plane0.Direction()^plane1.Direction());
    line.SetOrigin(plane0.Direction()*c0+ plane1.Direction()*c1);

    return true;
}


// Ray-Triangle Functor
template <bool BACKFACETEST = true>
class RayTriangleIntersectionFunctor {
public:
	template <class TRIANGLETYPE, class SCALARTYPE>
	inline bool operator () (const TRIANGLETYPE & f, const Ray3<SCALARTYPE> & ray, SCALARTYPE & t) {
		typedef SCALARTYPE ScalarType;
		ScalarType u;
		ScalarType v;

		bool bret = IntersectionRayTriangle(ray, Point3<SCALARTYPE>::Construct(f.cP(0)), Point3<SCALARTYPE>::Construct(f.cP(1)), Point3<SCALARTYPE>::Construct(f.cP(2)), t, u, v);
		if (BACKFACETEST) {
			if (!bret) {
				bret = IntersectionRayTriangle(ray, Point3<SCALARTYPE>::Construct(f.cP(0)), Point3<SCALARTYPE>::Construct(f.cP(2)), Point3<SCALARTYPE>::Construct(f.cP(1)), t, u, v);
			}
		}
		return (bret);
	}
};
	

/*@}*/


} // end namespace
#endif
