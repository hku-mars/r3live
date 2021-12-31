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
#ifndef CYLINDER_CLIP_H
#define CYLINDER_CLIP_H
#include <vcg/space/segment3.h>
#include <vcg/complex/algorithms/refine.h>
namespace vcg
{

// Taken a cylinder and a line calculates whether there is an intersection between these.
// In output are provided, if they exist, any two points of intersection (p0, p1)
// and the parameters t (t0, t1) on the line.
// Returns false if the distance of the line from the axis of the cylinder is greater than
// the radius of the cylinder or, if the calculation of t parameters - obtained by solving the
// quadratic equation - gives a delta less than zero.
// To find the intersection of a line p1+td1 with the axis p+td of the cylinder:
// (p1-p+td1-<v,p1-p+td1>d)^2 -r^2=0, becomes At^2+Bt+C=0.
//
// tmpA = d1 - (<d1,d>/<d,d>)*d.
// tmpB = (p1-p) - (<p1-p,d>/<d,d>)*d.
// A = <tmpA,tmpA>.
// B = 2*<tmpA,tmpB>.
// C = <tmpB,tmpB> - r^2.
// Input:  Cylinder<T> & cyl, Line3<T> & line.
// Output: CoordType & p0,CoordType & p1, T & t0, T &t1.

template<class T>
static bool IntersectionLineCylinder(const Segment3<T> & cylSeg, T radius, const Line3<T> & line, Point3<T> & p0, Point3<T> & p1, T & t0, T &t1)
{
      T dist;
      Point3<T> mClosestPoint0,mClosestPoint1;
      bool parallel=false;

      Line3<T> tmp;
      tmp.Set(cylSeg.P0(), (cylSeg.P1()-cylSeg.P0()).Normalize());
      LineLineDistance(tmp,line,parallel,dist,mClosestPoint0,mClosestPoint1);
      if(dist>radius)
          return false;
      if(parallel) return false;
      Point3<T> cyl_origin=tmp.Origin();
      Point3<T> line_origin=line.Origin();
      Point3<T> cyl_direction=tmp.Direction();
      Point3<T> line_direction=line.Direction();

      Point3<T> deltaP=line_origin-cyl_origin;

      T dotDirCyl=cyl_direction.SquaredNorm();    //<d,d>

      T scalar=line_direction.dot(cyl_direction);
      Point3<T> tmpA=line_direction-(cyl_direction/dotDirCyl)*scalar;
      T A=tmpA.SquaredNorm();

      T scalar2=deltaP.dot(cyl_direction);
      Point3<T> tmpB=(deltaP-(cyl_direction/dotDirCyl)*scalar2);

      T B=2.0*tmpA.dot(tmpB);

      T C=tmpB.SquaredNorm()-pow(radius,2);

      T delta=pow(B,2)-4*A*C;

      if(delta<0)
          return false;

      t0=(-B-sqrt(delta))/(2*A);
      t1=(-B+sqrt(delta))/(2*A);

      p0=line.P(t0);
      p1=line.P(t1);

      return true;
}

// Taken a cylinder and a segment calculates the intersection possible using the
// IntersectionLineCylinder() and checking the output of this.
// Whether the t0 and t1 scalars are between 0 and the length of the segment, then the point
// belongs to it and returns true.
// In output are given two points of intersection (p0, p1) and the parameters t (t0, t1) on the line.
// If p1 belongs to the segment and p0 no, it swaps the points (p0, p1) because operator() in the
// MidPointCylinder always takes the first.
// Otherwise, it means that there is no point between the extremes of the segment that intersects
// the cylinder, in this case it returns false.
//
// Input:  Cylinder<MESH_TYPE, Type, T> & cyl, Segment3<T> & seg.
// Output: CoordType & p0,CoordType & p1, T & t0, T &t1.

template<class T>
static bool IntersectionSegmentCylinder(const Segment3<T> & cylSeg , T radius,  const Segment3<T> & seg, Point3<T> & p0, Point3<T> & p1)
{
  const float eps = 10e-5;

  Line3<T> line;
  line.SetOrigin(seg.P0());
  line.SetDirection((seg.P1()-seg.P0()).Normalize());

  T t0,t1;
  if(IntersectionLineCylinder(cylSeg,radius,line,p0,p1,t0,t1)){
      bool inters0 = (t0>=0) && (t0<=seg.Length());
      bool inters1 = (t1>=0) && (t1<=seg.Length());
      if( inters0 && !inters1) p1=p0;  // if only one of the line intersections belong to the segment
      if(!inters0 &&  inters1) p0=p1;  // make both of them the same value.
      return inters0 || inters1;
  }
  return false;
}

template<class T>
static bool PointIsInSegment(const Point3<T> &point, const Segment3<T> &seg){
    const float eps = 10e-5;
    Line3<T> line;
    line.SetOrigin(seg.P0());
    line.SetDirection((seg.P1()-seg.P0()));
    T t=line.Projection(point);
    // Remembers, two points are different if their distance is >=eps
    if(t>-eps && t<1+eps)
        return true;
    return false;
}

namespace tri
{

template <class MeshType>
class CylinderClipping
{
public:
  typedef typename MeshType::ScalarType          ScalarType;
  typedef typename MeshType::VertexType          VertexType;
  typedef typename MeshType::VertexPointer       VertexPointer;
  typedef typename MeshType::VertexIterator      VertexIterator;
  typedef typename MeshType::CoordType           CoordType;
  typedef typename MeshType::FaceType            FaceType;
  typedef typename MeshType::FacePointer         FacePointer;
  typedef typename MeshType::FaceIterator        FaceIterator;
  typedef typename face::Pos<FaceType>        PosType;
  typedef Segment3<ScalarType> Segment3x;
  typedef Plane3<ScalarType> Plane3x;

  // This predicate
  class CylPred
  {
    public:
    CylPred(CoordType &_origin, CoordType &_end, ScalarType _radius, ScalarType _maxDist, ScalarType _minEdgeLen):
      origin(_origin),end(_end),radius(_radius),maxDist(_maxDist),minEdgeLen(_minEdgeLen){
      seg.Set(origin,end);
      pl0.Init(origin,(end-origin).Normalize());
      pl1.Init(end,(end-origin).Normalize());
    }
    void Init() { newPtMap.clear(); }
    ScalarType radius;
    CoordType origin,end;
    ScalarType minEdgeLen;
    ScalarType maxDist;
  private:
    Segment3x seg;
    Plane3x pl0,pl1;
  public:
    // This map store for each edge the new position.
    // it is intializaed by the predicate itself.
    // and it is shared with the midpoint functor.
    std::map< std::pair<CoordType,CoordType>,CoordType > newPtMap;

    // Return true if the given edge intersect the cylinder.

    // Verify if exist a point in an edge that intersects the cylinder. Then calculate
    // this point and store it for later use.
    // The cases possible are:
    // 1. Both extremes have distance greater than or equal to the radius, in this case it
    //    calculates the point of this segment closest to the axis of the cylinder. If this
    //    has distance less than or equal to the radius and is different from the extremes
    //    returns true and this point, otherwise false;
    // 2. If there is an extreme inside and one outside it returns true because exist the point
    //    of intersection that is calculated using the IntersectionSegmentCylinder();
    // 3. Otherwise false.
    // So a point is inside of the cylinder if its distance from his axis is <radius-eps??,
    // is external if the distance is > radius+eps and it is on the circumference if the
    // distance is in the range [radius-eps??, radius+eps].
    //
    // Input:  face::Pos<typename MESH_TYPE::FaceType>  ep, Cylinder<typename MESH_TYPE::ScalarType> cyl,

    bool operator()(PosType ep)
    {
      VertexType *v0 = ep.V();
      VertexType *v1 = ep.VFlip();
      ScalarType eps = minEdgeLen/100.0f;

      if(v0>v1) std::swap(v0,v1);
      CoordType p0=v0->P();
      CoordType p1=v1->P();

      // CASE 0 - For very short edges --- DO NOTHING
      if(Distance(p0,p1)< minEdgeLen) return false;

      Segment3x edgeSeg(p0,p1);
      CoordType closest0,closest1; // points on the cyl axis
      ScalarType dist0,dist1,dist2;

      SegmentPointDistance(this->seg,p0,closest0,dist0);
      SegmentPointDistance(this->seg,p1,closest1,dist1);

      // Case 0.5
      if(fabs(dist0-radius)<maxDist && fabs(dist1-radius)<maxDist)
      {
        newPtMap[std::make_pair(p0,p1)] = (p0+p1)*0.5;
        return true;
      }

      // ************ Case 1;
      if((dist0>radius) && (dist1>radius))
      {
        bool parallel;
        SegmentSegmentDistance(edgeSeg,this->seg, dist2, parallel, closest0,closest1);
        if((dist2<radius) &&
           (Distance(closest0,p0)>minEdgeLen) &&
           (Distance(closest0,p1)>minEdgeLen))
        {
          newPtMap[std::make_pair(p0,p1)] = closest0;
          return true;
        }
      }
      else if(((dist0<radius) && (dist1>radius))||((dist0>radius) && (dist1<radius))){
          CoordType int0,int1;
          // If there is an intersection point between segment and cylinder,
          // this must be different from the extremes of the segment and
          // his projection must be in the segment.
          if(IntersectionSegmentCylinder(this->seg, this->radius,edgeSeg,int0,int1)){
              if(PointIsInSegment(int0,this->seg) && (Distance(p0,int0)>eps) && (Distance(p1,int0)>eps))
              {
                if(Distance(int0,p0)<maxDist) return false;
                if(Distance(int0,p1)<maxDist) return false;
                newPtMap[std::make_pair(p0,p1)] = int0;
                  return true;
              }
          }
      }
      // Now check also against the caps
      CoordType pt;
      if(IntersectionPlaneSegment(pl0,edgeSeg,pt)){
        if((Distance(pt,origin)<radius+2.0*minEdgeLen) &&
           (Distance(pt,p0)>eps) && (Distance(pt,p1)>eps) )
        {
              newPtMap[std::make_pair(p0,p1)] = pt;
              return true;
        }
      }
      if(IntersectionPlaneSegment(pl1,edgeSeg,pt)){
        if( (Distance(pt,end)<radius+2.0*minEdgeLen) &&
            (Distance(pt,p0)>eps) && (Distance(pt,p1)>eps) )
        {
              newPtMap[std::make_pair(p0,p1)] = pt;
              return true;
        }
      }
      return false;
      //
    }
  };

  class CylMidPoint : public   std::unary_function<PosType, CoordType>
  {
  private:
    CylMidPoint() {assert(0);}
  public:
    CylMidPoint(CylPred &ep) : newPtMap(&(ep.newPtMap)) {
      assert(newPtMap);
    }
    std::map< std::pair<CoordType,CoordType>, CoordType > *newPtMap;
    void operator()(VertexType &nv, PosType ep)
    {
      typename std::map< std::pair<CoordType,CoordType>,CoordType >::iterator mi;
      VertexType *v0 = ep.V();
      VertexType *v1 = ep.VFlip();
      assert(newPtMap);
      if(v0>v1) std::swap(v0,v1);

      CoordType p0=v0->P();
      CoordType p1=v1->P();
      mi=newPtMap->find(std::make_pair(v0->P(),v1->P()));
      assert(mi!=newPtMap->end());
      nv.P()=(*mi).second;
    }

    Color4<ScalarType> WedgeInterp(Color4<ScalarType> &c0, Color4<ScalarType> &c1)
    {
        Color4<ScalarType> cc;
        return cc.lerp(c0,c1,0.5f);
    }

    TexCoord2<ScalarType,1> WedgeInterp(TexCoord2<ScalarType,1> &t0, TexCoord2<ScalarType,1> &t1)
    {
        TexCoord2<ScalarType,1> tmp;
        assert(t0.n()== t1.n());
        tmp.n()=t0.n();
        tmp.t()=(t0.t()+t1.t())/2.0;
        return tmp;
    }
  };

  static void Apply(MeshType &m, CoordType &origin, CoordType &end, ScalarType radius)
  {
    CylPred cylep(origin,end,radius,radius/100.0,m.bbox.Diag()/50.0f);
    CylMidPoint cylmp(cylep);
    int i=0;
    while((tri::RefineE<MeshType, CylMidPoint >(m, cylmp,cylep))&&(i<50)){
      cylep.Init();
      printf("Refine %d Vertici: %d, Facce: %d\n",i,m.VN(),m.FN());
      i++;
    }
  }
};
} // end namespace tri
} // end namespace vcg
#endif // CYLINDER_CLIP_H
