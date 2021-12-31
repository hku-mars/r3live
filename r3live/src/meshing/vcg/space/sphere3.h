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
Revision 1.9  2006/07/06 12:40:34  ganovelli
typdef ..ScalarType  added

Revision 1.8  2005/02/22 14:18:15  ponchio
assert addded.

Revision 1.7  2005/02/21 17:03:03  ponchio
Added Tight creation.

Revision 1.6  2004/12/01 16:06:59  ponchio
Distance

Revision 1.5  2004/09/29 13:55:33  ponchio
Added Distance shpere - point.

Revision 1.4  2004/04/02 09:49:01  ponchio
Ehm... a couople of small errors.

Revision 1.3  2004/04/02 09:44:13  ponchio
Sphere ->Sphere3

Revision 1.2  2004/03/25 17:25:46  ponchio
#include sbagliato.

Revision 1.1  2004/03/21 17:51:57  ponchio
First version.


****************************************************************************/

#ifndef VCG_SPHERE_H
#define VCG_SPHERE_H

#include <assert.h>
#include <vcg/space/point3.h>
#include <vector>

namespace vcg {

/** \addtogroup space */
/*@{*/
/** 
Templated class for 3D sphere.
  This is the class for definition of a sphere in 3D space. It is stored just as a Point3 and a radius
	@param T (template parameter) Specifies the type of scalar used to represent coords.
  Various policy could be added to improve efficience (keeping square of radius for instance).
*/

template <class T> class Sphere3 {
protected:
  Point3<T> _center;
  T _radius;
 public:
	typedef T ScalarType;
  Sphere3(): _radius(-1) {}
  Sphere3(const Point3<T> &center, T radius): _center(center), _radius(radius) {}
  
  T &Radius() { return _radius; }
  const T &Radius() const { return _radius; }
  Point3<T> &Center() { return _center; }
  const Point3<T> &Center() const { return _center; }

	bool IsEmpty() const { return _radius < 0; }
  ///return true if @param p - Center() <= Radius() 
  bool IsIn(const Point3<T> &p) const;
  bool IsIn(const Sphere3<T> &p) const;

  void Add(const Point3<T> &p);
  void Add(const Sphere3 &sphere);
  void Intersect(const Sphere3 &sphere);
  	

  int CreateFromBox(int n, const Point3<T> *points);
  //makes 36 iterations over the data... but get good results.
	int CreateTight(int n, const Point3<T> *points, 
		  T threshold = 1.01, T speed = 0.6);
	int CreateTight(const std::vector<Point3< T> > & points, 
		  T threshold = 1.01, T speed = 0.6);
};

//template <class T> T Distance(const Sphere3<T> &sphere, 
//			      const Point3<T> &point) {
//  T dist = Distance(point, sphere.Center()) - sphere.Radius();
//  if(dist < 0) dist = 0;
//  return dist;
//}
//
//template <class T> T Distance(const Sphere3<T> &sphere, 
//			      const Sphere3<T> &s) {
//  T dist = Distance(s.Center(), sphere.Center()) 
//                    - sphere.Radius() - s.Radius();
//  if(dist < 0) dist = 0;
//  return dist;
//}

typedef Sphere3<float> Sphere3f;
typedef Sphere3<double> Sphere3d;

template <class T> void Sphere3<T>::Add(const Sphere3<T> &sphere) {
  if(IsEmpty()) {
			*this = sphere;
			return;
		}
		Point3<T> dist = sphere.Center() - _center;
    float distance = dist.Norm();
    float fartest = sphere.Radius() + distance;
		if(fartest <= _radius) 
      return;

    float nearest = sphere.Radius() - distance;
    if(nearest >= _radius) {
      *this = sphere;
      return;
    }

    if(distance < 0.001*(_radius + sphere.Radius())) {
      _radius += distance;
      return;
    }

    _center += dist * ((fartest - _radius) / (distance * 2));
    _radius = (_radius + fartest)/2;
}

template <class T> void Sphere3<T>::Add(const Point3<T> &p) {
	if(IsEmpty()) {
		_center = p;
		_radius = 0;
	}
	Point3<T> dist = p - _center;
	float fartest = dist.Norm();
	if(fartest <= _radius) return;
	_center += dist * ((fartest - _radius) / (fartest*2));
	_radius = (_radius + fartest)/2;
}  

template <class T> bool Sphere3<T>::IsIn(const Point3<T> &p) const {
  if(IsEmpty()) return false;
  Point3<T> dist = p - _center;
  return dist.SquaredNorm() <= _radius*_radius;
}

template <class T> bool Sphere3<T>::IsIn(const Sphere3<T> &p) const {
  if(IsEmpty()) return false;
  Point3<T> dist = p.Center() - _center;
  float distance = dist.Norm();
  return distance + p.Radius() < _radius;
}

template <class T> void Sphere3<T>::Intersect(const Sphere3<T> &s) {
  float dist = Distance(_center, s.Center());
  float r = 0.5 * (_radius + s.Radius() - dist);
  if(r < 0) {
    _radius = -1;
    return;
  }
  _center = (s.Center()*(_radius - r) + _center*(s.Radius() - r))/dist;
  _radius = r;
}

 template <class T> int Sphere3<T>::CreateFromBox(int n,  const Point3<T> *points) {
   Point3f hi(-1e100, -1e100, -1e100);
   Point3f low(1e100, 1e100, 1e100);
   for(int i = 0; i < n; i++) {
     for(int k = 0; k < 3; k++) {
       if(hi[k] < points[i][k]) hi[k] = points[i][k];
       if(low[k] > points[i][k]) low[k] = points[i][k];
     }
   }
   Center() = (low + hi)/2;
   Radius() = (low - hi).Norm()/2;
   return 0;
 }
 template <class T> int Sphere3<T>::CreateTight(int n, const Point3<T> *points, 
						T threshold, T speed) { 
   //This is quantized gradient descent... really ugly. But simple :P
   //TODO step should adapt to terrain...
   for(int i = 0; i < n; i++)
     Add(points[i]);
   Radius() *= 1.0001;

   Point3<T> center;
   //Test with 6 directions

   Point3f pert[6];
   T step = Radius()/8;
   int count = 0;
   while(1) {
     count++;
     T radius = Radius();
     pert[0] = Point3f(step, 0, 0);
     pert[1] = -pert[0];
     pert[2] = Point3f(0, step, 0);
     pert[3] = -pert[2];
     pert[4] = Point3f(0, 0, step);
     pert[5] = -pert[4];

     int best = 6;
     T best_radius = Radius()/threshold;

     for(int k = 0; k < 6; k++) {
       center = Center() + pert[k];
       radius = 0;
       for(int i = 0; i < n; i++) {
         float r = Distance(center, points[i]);
         if(r > radius)
           radius = r;
       }
       if(radius < best_radius) {
         best = k;
         best_radius = radius;
       }
     }
     if(best != 6) {
       Center() = Center() + pert[best];
       Radius() = best_radius;
     }
     step *= speed; 
     if(step <= Radius() * (threshold - 1))
       break;
   }
   Radius() *= 1.01;
   
   //Test we did it correctly.
   for(int i = 0; i < n; i++) 
     assert(IsIn(points[i]));
   
   return count;
 }

template <class T> int Sphere3<T>::CreateTight(const std::vector<Point3<T> > & points,
																							 T threshold, T speed){
		return (points.empty())? -1 :CreateTight(points.size(),&(*points.begin()),threshold,speed);
}

} //namespace


#endif
