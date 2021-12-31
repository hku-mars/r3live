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

#ifndef __VCGLIB_FACE_DISTANCE
#define __VCGLIB_FACE_DISTANCE

#include <vcg/math/base.h>
#include <vcg/space/point3.h>
#include <vcg/space/segment3.h>
#include <vcg/space/distance3.h>

namespace vcg {
    namespace face{
/*
  Basic Wrapper for getting point-triangular face distance
  distance is unsigned;

  return true if the closest point <q> on <f> is nearer than the passed <dist>;
  return false otherwiswe (and q is not valid)

  This wrapper requires that your face has
  - Per Face Flags well initialized
  - Per Face EdgePlane component initialized.
  Initialization must be done with:

      tri::UpdateEdges<MeshType>::Set(yourMesh);
 */
    template <class FaceType>
    bool PointDistanceEP(	const FaceType &f,
                        const vcg::Point3<typename FaceType::ScalarType> & q,
                        typename FaceType::ScalarType & dist,
                        vcg::Point3<typename FaceType::ScalarType> & p )
    {
      typedef typename FaceType::ScalarType ScalarType;
      const ScalarType EPS = ScalarType( 0.000001);

        ScalarType b,b0,b1,b2;

        ScalarType d = SignedDistancePlanePoint( f.cPlane(), q );
        if( d>dist || d<-dist ) return false;

        Point3<ScalarType> t = f.cPlane().Direction();
        p = q - t*d; // p is the projection of q on the face plane
        // Now Choose the best plane and test to see if p is inside the triangle

        switch( f.cFlags() & (FaceType::NORMX|FaceType::NORMY|FaceType::NORMZ) )
        {
        case FaceType::NORMX:
            b0 = f.cEdge(1)[1]*(p[2] - f.cP(1)[2]) - f.cEdge(1)[2]*(p[1] - f.cP(1)[1]);
            if(b0<=0)
            {
                b0 = PSDist(q,f.cV(1)->cP(),f.cV(2)->cP(),p);
                if(dist>b0) { dist = b0; return true; }
                else return false;
            }
            b1 = f.cEdge(2)[1]*(p[2] - f.cP(2)[2]) - f.cEdge(2)[2]*(p[1] - f.cP(2)[1]);
            if(b1<=0)
            {
                b1 = PSDist(q,f.cV(2)->cP(),f.cV(0)->cP(),p);
                if(dist>b1) { dist = b1; return true; }
                else return false;
            }
            b2 = f.cEdge(0)[1]*(p[2] - f.cP(0)[2]) - f.cEdge(0)[2]*(p[1] - f.cP(0)[1]);
            if(b2<=0)
            {
                b2 = PSDist(q,f.cV(0)->cP(),f.cV(1)->cP(),p);
                if(dist>b2) { dist = b2; return true; }
                else return false;
            }
            // if all these tests failed the projection p should be inside.
            // Some further tests for more robustness...
            if( (b=std::min(b0,std::min(b1,b2)) ) < EPS*DoubleArea(f))
            {
              ScalarType bt;
              if(b==b0) 	    bt = PSDist(q,f.cV(1)->cP(),f.cV(2)->cP(),p);
              else if(b==b1) 	bt = PSDist(q,f.cV(2)->cP(),f.cV(0)->cP(),p);
              else {          assert(b==b2);
                bt = PSDist(q,f.cV(0)->cP(),f.cV(1)->cP(),p);
              }
              if(dist>bt) { dist = bt; return true; }
              else return false;
            }
            break;

          case FaceType::NORMY:
            b0 = f.cEdge(1)[2]*(p[0] - f.cP(1)[0]) - f.cEdge(1)[0]*(p[2] - f.cP(1)[2]);
            if(b0<=0)
            {
                b0 = PSDist(q,f.cV(1)->cP(),f.cV(2)->cP(),p);
                if(dist>b0) { dist = b0; return true; }
                else return false;
            }
            b1 = f.cEdge(2)[2]*(p[0] - f.cP(2)[0]) - f.cEdge(2)[0]*(p[2] - f.cP(2)[2]);
            if(b1<=0)
            {
                b1 = PSDist(q,f.cV(2)->cP(),f.cV(0)->cP(),p);
                if(dist>b1) { dist = b1; return true; }
                else return false;
            }
            b2 = f.cEdge(0)[2]*(p[0] - f.cP(0)[0]) - f.cEdge(0)[0]*(p[2] - f.cP(0)[2]);
            if(b2<=0)
            {
                b2 = PSDist(q,f.cV(0)->cP(),f.cV(1)->cP(),p);
                if(dist>b2) { dist = b2; return true; }
                else return false;
            }
            if( (b=math::Min<ScalarType>(b0,b1,b2)) < EPS*DoubleArea(f))
            {
              ScalarType bt;
              if(b==b0) 	    bt = PSDist(q,f.cV(1)->cP(),f.cV(2)->cP(),p);
              else if(b==b1) 	bt = PSDist(q,f.cV(2)->cP(),f.cV(0)->cP(),p);
              else { assert(b==b2);
                bt = PSDist(q,f.cV(0)->cP(),f.cV(1)->cP(),p);
              }
              if(dist>bt) { dist = bt; return true; }
                else return false;
            }
            break;

          case FaceType::NORMZ:
            b0 = f.cEdge(1)[0]*(p[1] - f.cP(1)[1]) - f.cEdge(1)[1]*(p[0] - f.cP(1)[0]);
            if(b0<=0)
            {
                b0 = PSDist(q,f.cV(1)->cP(),f.cV(2)->cP(),p);
                if(dist>b0) { dist = b0; return true; }
                else return false;
            }
            b1 = f.cEdge(2)[0]*(p[1] - f.cP(2)[1]) - f.cEdge(2)[1]*(p[0] - f.cP(2)[0]);
            if(b1<=0)
            {
                b1 = PSDist(q,f.cV(2)->cP(),f.cV(0)->cP(),p);
                if(dist>b1) { dist = b1; return true; }
                else return false;
            }
            b2 = f.cEdge(0)[0]*(p[1] - f.cP(0)[1]) - f.cEdge(0)[1]*(p[0] - f.cP(0)[0]);
            if(b2<=0)
            {
                b2 = PSDist(q,f.cV(0)->cP(),f.cV(1)->cP(),p);
                if(dist>b2) { dist = b2; return true; }
                else return false;
            }
            if( (b=math::Min<ScalarType>(b0,b1,b2)) < EPS*DoubleArea(f))
            {
              ScalarType bt;
              if(b==b0) 	    bt = PSDist(q,f.cV(1)->cP(),f.cV(2)->cP(),p);
              else if(b==b1) 	bt = PSDist(q,f.cV(2)->cP(),f.cV(0)->cP(),p);
              else { assert(b==b2);
                bt = PSDist(q,f.cV(0)->cP(),f.cV(1)->cP(),p);
              }

              if(dist>bt) { dist = bt; return true; }
                else return false;
            }
            break;

        } // end switch

        dist = ScalarType(fabs(d));
        return true;
    }

    template <class S>
    class PointDistanceEPFunctor {
    public:
        typedef S ScalarType;
        typedef Point3<ScalarType> QueryType;
        static inline const Point3<ScalarType> &  Pos(const QueryType & qt)  {return qt;}

        template <class FACETYPE, class SCALARTYPE>
        inline bool operator () (const FACETYPE & f, const Point3<SCALARTYPE> & p, SCALARTYPE & minDist, Point3<SCALARTYPE> & q) {
            const Point3<typename FACETYPE::ScalarType> fp = Point3<typename FACETYPE::ScalarType>::Construct(p);
            Point3<typename FACETYPE::ScalarType> fq;
            typename FACETYPE::ScalarType md = (typename FACETYPE::ScalarType)(minDist);
            const bool ret = PointDistanceEP(f, fp, md, fq);
            minDist = (SCALARTYPE)(md);
            q = Point3<SCALARTYPE>::Construct(fq);
            return (ret);
        }
    };



    template <class S>
    class PointNormalDistanceFunctor {
    public:
        typedef typename S::ScalarType ScalarType;
        typedef S QueryType;
        static inline const Point3<ScalarType> &  Pos(const QueryType & qt)  {return qt.P();}


        static ScalarType & Alpha(){static ScalarType alpha = 1.0; return alpha;}
        static ScalarType & Beta (){static ScalarType beta  = 1.0; return beta;}
        static ScalarType & Gamma(){static ScalarType gamma = 1.0; return gamma;}
        static ScalarType & InterPoint(){static ScalarType interpoint= 1.0; return interpoint;}


        template <class FACETYPE, class SCALARTYPE>
        inline bool operator () (const FACETYPE &f, const typename FACETYPE::VertexType &p,
            SCALARTYPE & minDist,Point3<SCALARTYPE> & q)
        {
            const Point3<typename FACETYPE::ScalarType> fp = Point3<typename FACETYPE::ScalarType>::Construct(p.cP());
            const Point3<typename FACETYPE::ScalarType> fn = Point3<typename FACETYPE::ScalarType>::Construct(p.cN());
            Point3<typename FACETYPE::ScalarType> fq;
            typename FACETYPE::ScalarType md = (typename FACETYPE::ScalarType)(minDist);
            const bool ret=PointDistance(f,fp,md,fq);

            SCALARTYPE  dev=InterPoint()*(pow((ScalarType)(1-f.cN().dot(fn)),(ScalarType)Beta())/(Gamma()*md+0.1));

            if (md+dev < minDist){
                minDist = (SCALARTYPE)(md+dev);
                q = Point3<SCALARTYPE>::Construct(fq);
                //q.N() = f.N();
                return (ret);
            }
            return false;
        }
    };

        /// BASIC VERSION of the Point-face distance that does not require the EdgePlane Additional data.
        /// Given a face and a point, returns the closest point of the face to p.

        template <class FaceType>
            bool PointDistanceBase(
                                                    const FaceType &f,																		/// the face to be tested
                                                    const vcg::Point3<typename FaceType::ScalarType> & q, /// the point tested
                                                    typename FaceType::ScalarType & dist,                 /// bailout distance. It must be initialized with the max admittable value.
                                                    vcg::Point3<typename FaceType::ScalarType> & p )
        {
                typedef typename FaceType::ScalarType ScalarType;

                if(f.cN()==Point3<ScalarType>(0,0,0)) // to correctly manage the case of degenerate triangles we consider them as segments.
                {
                  Box3<ScalarType> bb;
                  f.GetBBox(bb);
                  Segment3<ScalarType> degenTri(bb.min,bb.max);
                  Point3<ScalarType> closest;
                  ScalarType d;
                  if(bb.Diag()>0)
                    vcg::SegmentPointDistance<ScalarType>(degenTri,q,closest,d);
                  else // very degenerate triangle (just a point)
                  {
                    closest = bb.min;
                    d=Distance(q,closest);
                  }
                  if( d>dist) return false;
                  dist=d;
                  p=closest;
                  assert(!math::IsNAN(dist));
                  return true;
                }

                Plane3<ScalarType,true> fPlane;
                fPlane.Init(f.cP(0),f.cN());
                const ScalarType EPS = ScalarType( 0.000001);
                ScalarType b,b0,b1,b2;
                // Calcolo distanza punto piano
                ScalarType d = SignedDistancePlanePoint( fPlane, q );
                if( d>dist || d<-dist )			// Risultato peggiore: niente di fatto
                    return false;

                // Projection of query point onto the triangle plane
                p = q - fPlane.Direction()*d;

                Point3<ScalarType> fEdge[3];
                fEdge[0] = f.cP(1); fEdge[0] -= f.cP(0);
                fEdge[1] = f.cP(2); fEdge[1] -= f.cP(1);
                fEdge[2] = f.cP(0); fEdge[2] -= f.cP(2);

                /*
                This piece of code is part of the EdgePlane initialization structure: note that the edges are scaled!.

                if(nx>ny && nx>nz) { f.Flags() |= FaceType::NORMX; d = 1/f.Plane().Direction()[0]; }
                else if(ny>nz)     { f.Flags() |= FaceType::NORMY; d = 1/f.Plane().Direction()[1]; }
                else               { f.Flags() |= FaceType::NORMZ; d = 1/f.Plane().Direction()[2]; }
                f.Edge(0)*=d; f.Edge(1)*=d;f.Edge(2)*=d;

                So we must apply the same scaling according to the plane orientation, eg in the case of NORMX
                  scaleFactor= 1/fPlane.Direction()[0];
                  fEdge[0]*=d; fEdge[1]*=d;fEdge[2]*=d;
                */

                int bestAxis;
                if(fabs(f.cN()[0])>fabs(f.cN()[1]))
                {
                  if(fabs(f.cN()[0])>fabs(f.cN()[2])) bestAxis = 0;
                  else bestAxis = 2;
                } else {
                  if(fabs(f.cN()[1])>fabs(f.cN()[2])) bestAxis=1; /* 1 > 0 ? 2 */
                  else bestAxis=2; /* 2 > 1 ? 2 */
                }

                ScalarType scaleFactor;

                switch( bestAxis )
                {
                    case 0:  /************* X AXIS **************/
                        scaleFactor= 1/fPlane.Direction()[0];
                        fEdge[0]*=scaleFactor; fEdge[1]*=scaleFactor; fEdge[2]*=scaleFactor;

                        b0 = fEdge[1][1]*(p[2] - f.cP(1)[2]) - fEdge[1][2]*(p[1] - f.cP(1)[1]);
                        if(b0<=0)
                        {
                            b0 = PSDist(q,f.cV(1)->cP(),f.cV(2)->cP(),p);
                            if(dist>b0) { dist = b0; return true; }
                            else return false;
                        }
                            b1 = fEdge[2][1]*(p[2] - f.cP(2)[2]) - fEdge[2][2]*(p[1] - f.cP(2)[1]);
                        if(b1<=0)
                        {
                            b1 = PSDist(q,f.cV(2)->cP(),f.cV(0)->cP(),p);
                            if(dist>b1) { dist = b1; return true; }
                            else return false;
                        }
                            b2 = fEdge[0][1]*(p[2] - f.cP(0)[2]) - fEdge[0][2]*(p[1] - f.cP(0)[1]);
                        if(b2<=0)
                        {
                            b2 = PSDist(q,f.cV(0)->cP(),f.cV(1)->cP(),p);
                            if(dist>b2) { dist = b2; return true; }
                            else return false;
                        }
                            // sono tutti e tre > 0 quindi dovrebbe essere dentro;
                            // per sicurezza se il piu' piccolo dei tre e' < epsilon (scalato rispetto all'area della faccia
                            // per renderlo dimension independent.) allora si usa ancora la distanza punto
                            // segmento che e' piu robusta della punto piano, e si fa dalla parte a cui siamo piu'
                            // vicini (come prodotto vettore)
                            // Nota: si potrebbe rendere un pochino piu' veloce sostituendo Area()
                            // con il prodotto vettore dei due edge in 2d lungo il piano migliore.
                        if( (b=vcg::math::Min<ScalarType>(b0,b1,b2)) < EPS*DoubleArea(f))
                            {
                                ScalarType bt;
                                if(b==b0) 	    bt = PSDist(q,f.cV(1)->cP(),f.cV(2)->cP(),p);
                                else if(b==b1) 	bt = PSDist(q,f.cV(2)->cP(),f.cV(0)->cP(),p);
                                else {assert(b==b2); bt = PSDist(q,f.cV(0)->cP(),f.cV(1)->cP(),p);}
                                //printf("Warning area:%g %g %g %g thr:%g bt:%g\n",Area(), b0,b1,b2,EPSILON*Area(),bt);
                                if(dist>bt) { dist = bt; return true; }
                                else return false;
                            }
                            break;

                    case 1:  /************* Y AXIS **************/
                        scaleFactor= 1/fPlane.Direction()[1];
                        fEdge[0]*=scaleFactor; fEdge[1]*=scaleFactor; fEdge[2]*=scaleFactor;

                        b0 = fEdge[1][2]*(p[0] - f.cP(1)[0]) - fEdge[1][0]*(p[2] - f.cP(1)[2]);
                        if(b0<=0)
                        {
                            b0 = PSDist(q,f.cV(1)->cP(),f.cV(2)->cP(),p);
                            if(dist>b0) { dist = b0; return true; }
                            else return false;
                        }
                            b1 = fEdge[2][2]*(p[0] - f.cP(2)[0]) - fEdge[2][0]*(p[2] - f.cP(2)[2]);
                        if(b1<=0)
                        {
                            b1 = PSDist(q,f.cV(2)->cP(),f.cV(0)->cP(),p);
                            if(dist>b1) { dist = b1; return true; }
                            else return false;
                        }
                            b2 = fEdge[0][2]*(p[0] - f.cP(0)[0]) - fEdge[0][0]*(p[2] - f.cP(0)[2]);
                        if(b2<=0)
                        {
                            b2 = PSDist(q,f.cV(0)->cP(),f.cV(1)->cP(),p);
                            if(dist>b2) { dist = b2; return true; }
                            else return false;
                        }
            if( (b=vcg::math::Min<ScalarType>(b0,b1,b2)) < EPS*DoubleArea(f))
                            {
                                ScalarType bt;
                                if(b==b0) 	    bt = PSDist(q,f.cV(1)->cP(),f.cV(2)->cP(),p);
                                else if(b==b1) 	bt = PSDist(q,f.cV(2)->cP(),f.cV(0)->cP(),p);
                                else{ assert(b==b2); bt = PSDist(q,f.cV(0)->cP(),f.cV(1)->cP(),p);}
                                //printf("Warning area:%g %g %g %g thr:%g bt:%g\n",Area(), b0,b1,b2,EPSILON*Area(),bt);
                                if(dist>bt) { dist = bt; return true; }
                                else return false;
                            }
                            break;

                    case 2:  /************* Z AXIS **************/
                        scaleFactor= 1/fPlane.Direction()[2];
                        fEdge[0]*=scaleFactor; fEdge[1]*=scaleFactor; fEdge[2]*=scaleFactor;

                        b0 = fEdge[1][0]*(p[1] - f.cP(1)[1]) - fEdge[1][1]*(p[0] - f.cP(1)[0]);
                        if(b0<=0)
                        {
                            b0 = PSDist(q,f.cV(1)->cP(),f.cV(2)->cP(),p);
                            if(dist>b0) { dist = b0; return true; }
                            else return false;
                        }
                            b1 = fEdge[2][0]*(p[1] - f.cP(2)[1]) - fEdge[2][1]*(p[0] - f.cP(2)[0]);
                        if(b1<=0)
                        {
                            b1 = PSDist(q,f.cV(2)->cP(),f.cV(0)->cP(),p);
                            if(dist>b1) { dist = b1; return true; }
                            else return false;
                        }
                            b2 = fEdge[0][0]*(p[1] - f.cP(0)[1]) - fEdge[0][1]*(p[0] - f.cP(0)[0]);
                        if(b2<=0)
                        {
                            b2 = PSDist(q,f.cV(0)->cP(),f.cV(1)->cP(),p);
                            if(dist>b2) { dist = b2; return true; }
                            else return false;
                        }
            if( (b=vcg::math::Min<ScalarType>(b0,b1,b2)) < EPS*DoubleArea(f))
                            {
                                ScalarType bt;
                                if(b==b0) 	    bt = PSDist(q,f.cV(1)->cP(),f.cV(2)->cP(),p);
                                else if(b==b1) 	bt = PSDist(q,f.cV(2)->cP(),f.cV(0)->cP(),p);
                                else { assert(b==b2); bt = PSDist(q,f.cV(0)->cP(),f.cV(1)->cP(),p); }
                                //printf("Warning area:%g %g %g %g thr:%g bt:%g\n",Area(), b0,b1,b2,EPSILON*Area(),bt);

                                if(dist>bt) { dist = bt; return true; }
                                else return false;
                            }
                            break;
                        default: assert(0); // if you get this assert it means that you forgot to set the required UpdateFlags<MeshType>::FaceProjection(m);

                }

                dist = ScalarType(fabs(d));
                //dist = Distance(p,q);
                return true;
        }

    template <class S>
    class PointDistanceBaseFunctor {
public:
            typedef S ScalarType;
            typedef Point3<ScalarType> QueryType;

          static inline const Point3<ScalarType> & Pos(const Point3<ScalarType> & qt)  {return qt;}
            template <class FACETYPE, class SCALARTYPE>
            inline bool operator () (const FACETYPE & f, const Point3<SCALARTYPE> & p, SCALARTYPE & minDist, Point3<SCALARTYPE> & q) {
                const Point3<typename FACETYPE::ScalarType> fp = Point3<typename FACETYPE::ScalarType>::Construct(p);
                Point3<typename FACETYPE::ScalarType> fq;
                typename FACETYPE::ScalarType md = (typename FACETYPE::ScalarType)(minDist);
                const bool ret = PointDistanceBase(f, fp, md, fq);
                minDist = (SCALARTYPE)(md);
                q = Point3<SCALARTYPE>::Construct(fq);
                return (ret);
            }
        };



}	 // end namespace face

}	 // end namespace vcg


#endif

