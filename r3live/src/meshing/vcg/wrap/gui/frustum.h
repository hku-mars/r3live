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
Revision 1.9  2005/03/02 15:13:45  ponchio
Minimal fix in remoteness (Bugged anyway)

Revision 1.8  2005/02/22 14:33:04  ponchio
small bugs

Revision 1.7  2005/01/21 18:06:05  ponchio
Added remoteness ("distance" from frustum)

Revision 1.6  2004/10/04 12:33:02  ponchio
Cleaning up and planes init more stable.

Revision 1.5  2004/09/28 10:23:28  ponchio
Various generic changes.

Revision 1.4  2004/05/12 20:55:18  ponchio
*** empty log message ***

Revision 1.3  2004/03/31 15:06:41  ponchio
#include <camera> -> #include <view>

Revision 1.2  2004/03/25 14:55:25  ponchio
Adding copyright.


****************************************************************************/

#ifndef FRUSTUM_H
#define FRUSTUM_H

#include <wrap/gui/view.h>
#include <vcg/space/plane3.h>
#include <vcg/space/line3.h>

#include <iostream>
using namespace std;

namespace vcg {

template <class T> class Frustum: public View<T> {
public:                                            
  void GetView();
  void SetView(const float *_proj, const float *_modelview, const int *_viewport);
  Point3<T> ViewPoint();
  T Resolution(float dist = 1);
  bool IsOutside(Point3<T> &point);
  T Remoteness(Point3<T> &point, T radius);
  T IsOutside(Point3<T> &point, T radius);
  T Distance(Point3<T> &point, int plane);  
  T range(Point3<T> &point, T radius, T &closest, T &farthest);
  
protected:
  T resolution;  
  Plane3<T> planes[6];  
  Point3<T> view_point;
  void UpdateView();
};


typedef Frustum<float> Frustumf;
typedef Frustum<double> Frustumd;


//Implementation
template <class T> Point3<T> Frustum<T>::ViewPoint() {
  return view_point;  
}

template <class T> T Frustum<T>::Resolution(float dist) {
  return resolution * dist;  
}

template <class T> bool Frustum<T>::IsOutside(Point3<T> &point) {
  Point3<T> r = Project(point);
  if(r[0] < View<T>::viewport[0] || r[0] > View<T>::viewport[0]+View<T>::viewport[2] ||
     r[1] < View<T>::viewport[1] || r[1] > View<T>::viewport[1]+View<T>::viewport[3]) 
    return true;
  return false;
}

template <class T> T Frustum<T>::Remoteness(Point3<T> &point, T radius) {
  Point3<T> r = Project(point);
  T dist = (point - view_point).Norm();
  if(dist < radius) return 0;
  T rad =  1 + radius / (resolution * dist);
  T mindist = 0;
  T tmp;
  tmp = View<T>::viewport[0] - r[0] - rad;
  if(tmp > mindist) mindist = tmp;
  tmp = r[0] - rad - (View<T>::viewport[0] + View<T>::viewport[2]);
  if(tmp > mindist) mindist = tmp;
  
  tmp = View<T>::viewport[1] - r[1] - rad;
  if(tmp > mindist) mindist = tmp;
  tmp = r[1] - rad - (View<T>::viewport[1] + View<T>::viewport[3]);
  if(tmp > mindist) mindist = tmp;
  
  if(mindist == 0) return 0;
  return 1 + (mindist / (View<T>::viewport[0] + View<T>::viewport[2]));
}

template <class T> T Frustum<T>::IsOutside(Point3<T> &point, T radius) {
  T dist = 0;
  for(int i = 0; i < 4; i++) {
    T d = -Distance(point, i) - radius;
    if(d > dist) dist = d;
  }
  return dist;
}

template <class T> T Frustum<T>::range(Point3<T> &point, T radius, T &closest, T &farthest) {
  //4 near 5 far
  T dist = (view_point - point).Norm();
  closest = dist - radius;
  farthest = dist + radius;
}

template <class T> T Frustum<T>::Distance(Point3<T> &point, int plane) {    
  return vcg::SignedDistancePlanePoint(planes[plane], point);
}

template <class T> void Frustum<T>::GetView() {
  View<T>::GetView();
  UpdateView();
}
template <class T> void Frustum<T>::SetView(const float *_proj = NULL,
                                            const float *_modelview = NULL,
                                            const int *_viewport = NULL) {
  View<T>::SetView(_proj, _modelview, _viewport);
  UpdateView();
}

template <class T> void Frustum<T>::UpdateView() {
  float t = (float)(View<T>::viewport[1] +View<T>:: viewport[3]);
  float b = (float)View<T>::viewport[1];
  float r = (float)(View<T>::viewport[0] + View<T>::viewport[2]);
  float l = (float)View<T>::viewport[0];
  
  Point3<T> nw = UnProject(Point3<T>(l, b, 0.0f));
  Point3<T> sw = UnProject(Point3<T>(l, t, 0.0f));
  Point3<T> ne = UnProject(Point3<T>(r, b, 0.0f));
  Point3<T> se = UnProject(Point3<T>(r, t, 0.0f));
  Point3<T> NW = UnProject(Point3<T>(l, b, 1.0f));
  Point3<T> SW = UnProject(Point3<T>(l, t, 1.0f));
  Point3<T> NE = UnProject(Point3<T>(r, b, 1.0f));
  Point3<T> SE = UnProject(Point3<T>(r, t, 1.0f));

  view_point = View<T>::ViewPoint();  	

  planes[0].Init(nw, NW, NE);  
  planes[1].Init(ne, NE, SE);
  planes[2].Init(se, SE, SW);
  planes[3].Init(sw, SW, NW);
  planes[4].Init(se, sw, nw);
  planes[5].Init(SW, SE, NE);   

  //compute resolution: sizeo of a pixel unitary distance from view_point
  resolution = ((ne + NE)/2 - (nw + NW)/2).Norm() /
               (View<T>::viewport[2] * ((ne + NE + nw + NW)/4 - view_point).Norm());
}

}//namespace

#endif

  
	

  
