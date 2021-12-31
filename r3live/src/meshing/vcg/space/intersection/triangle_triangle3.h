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
Revision 1.2  2005/10/24 09:19:33  ponchio
Added newline at end of file (tired of stupid warnings...)

Revision 1.1  2004/04/26 12:33:59  ganovelli
first version

****************************************************************************/
#ifndef __VCGLIB_INTERSECTIONTRITRI3
#define __VCGLIB_INTERSECTIONTRITRI3

#include <vcg/space/point3.h>
#ifndef FAST
#include "predicates.h"
#endif
#include <math.h>


namespace vcg {

/** \addtogroup space */
/*@{*/
/** 
		Triangle/triangle intersection ,based on the algorithm presented in "A Fast Triangle-Triangle Intersection Test",
		Journal of Graphics Tools, 2(2), 1997
 */
#ifndef FABS
#define FABS(x) (T(fabs(x)))  
#endif
#define USE_EPSILON_TEST
#define TRI_TRI_INT_EPSILON 0.000001



#define CROSS(dest,v1,v2){                     \
              dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
              dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
              dest[2]=v1[0]*v2[1]-v1[1]*v2[0];}

#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])

#define SUB(dest,v1,v2){         \
            dest[0]=v1[0]-v2[0]; \
            dest[1]=v1[1]-v2[1]; \
            dest[2]=v1[2]-v2[2];}


#define SORT(a,b)       \
             if(a>b)    \
             {          \
               T c; \
               c=a;     \
               a=b;     \
               b=c;     \
             }


#define EDGE_EDGE_TEST(V0,U0,U1)                      \
  Bx=U0[i0]-U1[i0];                                   \
  By=U0[i1]-U1[i1];                                   \
  Cx=V0[i0]-U0[i0];                                   \
  Cy=V0[i1]-U0[i1];                                   \
  f=Ay*Bx-Ax*By;                                      \
  d=By*Cx-Bx*Cy;                                      \
  if((f>0 && d>=0 && d<=f) || (f<0 && d<=0 && d>=f))  \
  {                                                   \
    e=Ax*Cy-Ay*Cx;                                    \
    if(f>0)                                           \
    {                                                 \
      if(e>=0 && e<=f) return 1;                      \
    }                                                 \
    else                                              \
    {                                                 \
      if(e<=0 && e>=f) return 1;                      \
    }                                                 \
  }

#define EDGE_AGAINST_TRI_EDGES(V0,V1,U0,U1,U2) \
{                                              \
  T Ax,Ay,Bx,By,Cx,Cy,e,d,f;               \
  Ax=V1[i0]-V0[i0];                            \
  Ay=V1[i1]-V0[i1];                            \
  /* test edge U0,U1 against V0,V1 */          \
  EDGE_EDGE_TEST(V0,U0,U1);                    \
  /* test edge U1,U2 against V0,V1 */          \
  EDGE_EDGE_TEST(V0,U1,U2);                    \
  /* test edge U2,U1 against V0,V1 */          \
  EDGE_EDGE_TEST(V0,U2,U0);                    \
}

#define POINT_IN_TRI(V0,U0,U1,U2)           \
{                                           \
  T a,b,c,d0,d1,d2;                     \
  /* is T1 completly inside T2? */          \
  /* check if V0 is inside tri(U0,U1,U2) */ \
  a=U1[i1]-U0[i1];                          \
  b=-(U1[i0]-U0[i0]);                       \
  c=-a*U0[i0]-b*U0[i1];                     \
  d0=a*V0[i0]+b*V0[i1]+c;                   \
                                            \
  a=U2[i1]-U1[i1];                          \
  b=-(U2[i0]-U1[i0]);                       \
  c=-a*U1[i0]-b*U1[i1];                     \
  d1=a*V0[i0]+b*V0[i1]+c;                   \
                                            \
  a=U0[i1]-U2[i1];                          \
  b=-(U0[i0]-U2[i0]);                       \
  c=-a*U2[i0]-b*U2[i1];                     \
  d2=a*V0[i0]+b*V0[i1]+c;                   \
  if(d0*d1>0.0)                             \
  {                                         \
    if(d0*d2>0.0) return 1;                 \
  }                                         \
}

template<class T>
/** CHeck two triangles for coplanarity
	@param N
	@param V0 A vertex of the first triangle
	@param V1 A vertex of the first triangle
	@param V2 A vertex of the first triangle
	@param U0 A vertex of the second triangle
	@param U1 A vertex of the second triangle
	@param U2 A vertex of the second triangle
	@return true se due triangoli sono cooplanari, false altrimenti

*/
bool coplanar_tri_tri(const Point3<T> N, const Point3<T> V0, const Point3<T> V1,const Point3<T> V2,
										 const Point3<T> U0, const Point3<T> U1,const Point3<T> U2)
{
   T A[3];
   short i0,i1;
   /* first project onto an axis-aligned plane, that maximizes the area */
   /* of the triangles, compute indices: i0,i1. */
   A[0]=FABS(N[0]);
   A[1]=FABS(N[1]);
   A[2]=FABS(N[2]);
   if(A[0]>A[1])
   {
      if(A[0]>A[2])
      {
          i0=1;      /* A[0] is greatest */
          i1=2;
      }
      else
      {
          i0=0;      /* A[2] is greatest */
          i1=1;
      }
   }
   else   /* A[0]<=A[1] */
   {
      if(A[2]>A[1])
      {
          i0=0;      /* A[2] is greatest */
          i1=1;
      }
      else
      {
          i0=0;      /* A[1] is greatest */
          i1=2;
      }
    }

    /* test all edges of triangle 1 against the edges of triangle 2 */
    EDGE_AGAINST_TRI_EDGES(V0,V1,U0,U1,U2);
    EDGE_AGAINST_TRI_EDGES(V1,V2,U0,U1,U2);
    EDGE_AGAINST_TRI_EDGES(V2,V0,U0,U1,U2);

    /* finally, test if tri1 is totally contained in tri2 or vice versa */
    POINT_IN_TRI(V0,U0,U1,U2);
    POINT_IN_TRI(U0,V0,V1,V2);

    return 0;
}



#define NEWCOMPUTE_INTERVALS(VV0,VV1,VV2,D0,D1,D2,D0D1,D0D2,A,B,C,X0,X1) \
{ \
        if(D0D1>0.0f) \
        { \
                /* here we know that D0D2<=0.0 */ \
            /* that is D0, D1 are on the same side, D2 on the other or on the plane */ \
                A=VV2; B=(VV0-VV2)*D2; C=(VV1-VV2)*D2; X0=D2-D0; X1=D2-D1; \
        } \
        else if(D0D2>0.0f)\
        { \
                /* here we know that d0d1<=0.0 */ \
            A=VV1; B=(VV0-VV1)*D1; C=(VV2-VV1)*D1; X0=D1-D0; X1=D1-D2; \
        } \
        else if(D1*D2>0.0f || D0!=0.0f) \
        { \
                /* here we know that d0d1<=0.0 or that D0!=0.0 */ \
                A=VV0; B=(VV1-VV0)*D0; C=(VV2-VV0)*D0; X0=D0-D1; X1=D0-D2; \
        } \
        else if(D1!=0.0f) \
        { \
                A=VV1; B=(VV0-VV1)*D1; C=(VV2-VV1)*D1; X0=D1-D0; X1=D1-D2; \
        } \
        else if(D2!=0.0f) \
        { \
                A=VV2; B=(VV0-VV2)*D2; C=(VV1-VV2)*D2; X0=D2-D0; X1=D2-D1; \
        } \
        else \
        { \
                /* triangles are coplanar */ \
                return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2); \
        } \
}


template<class T>
/* 
	@param V0 A vertex of the first triangle
	@param V1 A vertex of the first triangle
	@param V2 A vertex of the first triangle
	@param U0 A vertex of the second triangle
	@param U1 A vertex of the second triangle
	@param U2 A vertex of the second triangle
	@return true if the two triangles interesect
*/
bool NoDivTriTriIsect(const Point3<T> V0,const Point3<T> V1,const Point3<T> V2,
                      const Point3<T> U0,const Point3<T> U1,const Point3<T> U2)
{
  Point3<T> E1,E2;
  Point3<T> N1,N2;
	T d1,d2;
  T du0,du1,du2,dv0,dv1,dv2;
  Point3<T> D;
  T isect1[2], isect2[2];
  T du0du1,du0du2,dv0dv1,dv0dv2;
  short index;
  T vp0,vp1,vp2;
  T up0,up1,up2;
  T bb,cc,max;

  /* compute plane equation of triangle(V0,V1,V2) */
  SUB(E1,V1,V0);
  SUB(E2,V2,V0);
  CROSS(N1,E1,E2);
	N1.Normalize(); // aggiunto rispetto al codice orig.
  d1=-DOT(N1,V0);
  /* plane equation 1: N1.X+d1=0 */

  /* put U0,U1,U2 into plane equation 1 to compute signed distances to the plane*/
  du0=DOT(N1,U0)+d1;
  du1=DOT(N1,U1)+d1;
  du2=DOT(N1,U2)+d1;

  /* coplanarity robustness check */
#ifdef USE_TRI_TRI_INT_EPSILON_TEST
  if(FABS(du0)<TRI_TRI_INT_EPSILON) du0=0.0;
  if(FABS(du1)<TRI_TRI_INT_EPSILON) du1=0.0;
  if(FABS(du2)<TRI_TRI_INT_EPSILON) du2=0.0;
#endif
  du0du1=du0*du1;
  du0du2=du0*du2;

  if(du0du1>0.0f && du0du2>0.0f) /* same sign on all of them + not equal 0 ? */
    return 0;                    /* no intersection occurs */

  /* compute plane of triangle (U0,U1,U2) */
  SUB(E1,U1,U0);
  SUB(E2,U2,U0);
  CROSS(N2,E1,E2);
  d2=-DOT(N2,U0);
  /* plane equation 2: N2.X+d2=0 */

  /* put V0,V1,V2 into plane equation 2 */
  dv0=DOT(N2,V0)+d2;
  dv1=DOT(N2,V1)+d2;
  dv2=DOT(N2,V2)+d2;

#ifdef USE_TRI_TRI_INT_EPSILON_TEST
  if(FABS(dv0)<TRI_TRI_INT_EPSILON) dv0=0.0;
  if(FABS(dv1)<TRI_TRI_INT_EPSILON) dv1=0.0;
  if(FABS(dv2)<TRI_TRI_INT_EPSILON) dv2=0.0;
#endif

  dv0dv1=dv0*dv1;
  dv0dv2=dv0*dv2;

  if(dv0dv1>0.0f && dv0dv2>0.0f) /* same sign on all of them + not equal 0 ? */
    return 0;                    /* no intersection occurs */

  /* compute direction of intersection line */
  CROSS(D,N1,N2);

  /* compute and index to the largest component of D */
  max=(T)FABS(D[0]);
  index=0;
  bb=(T)FABS(D[1]);
  cc=(T)FABS(D[2]);
  if(bb>max) max=bb,index=1;
  if(cc>max) max=cc,index=2;

  /* this is the simplified projection onto L*/
  vp0=V0[index];
  vp1=V1[index];
  vp2=V2[index];

  up0=U0[index];
  up1=U1[index];
  up2=U2[index];

  /* compute interval for triangle 1 */
  T a,b,c,x0,x1;
  NEWCOMPUTE_INTERVALS(vp0,vp1,vp2,dv0,dv1,dv2,dv0dv1,dv0dv2,a,b,c,x0,x1);

  /* compute interval for triangle 2 */
  T d,e,f,y0,y1;
  NEWCOMPUTE_INTERVALS(up0,up1,up2,du0,du1,du2,du0du1,du0du2,d,e,f,y0,y1);

  T xx,yy,xxyy,tmp;
  xx=x0*x1;
  yy=y0*y1;
  xxyy=xx*yy;

  tmp=a*xxyy;
  isect1[0]=tmp+b*x1*yy;
  isect1[1]=tmp+c*x0*yy;

  tmp=d*xxyy;
  isect2[0]=tmp+e*xx*y1;
  isect2[1]=tmp+f*xx*y0;

  SORT(isect1[0],isect1[1]);
  SORT(isect2[0],isect2[1]);

  if(isect1[1]<isect2[0] || isect2[1]<isect1[0]) return 0;
  return 1;
}



#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])
#define ADD(dest,v1,v2) dest[0]=v1[0]+v2[0]; dest[1]=v1[1]+v2[1]; dest[2]=v1[2]+v2[2]; 
#define MULT(dest,v,factor) dest[0]=factor*v[0]; dest[1]=factor*v[1]; dest[2]=factor*v[2];
#define SET(dest,src) dest[0]=src[0]; dest[1]=src[1]; dest[2]=src[2]; 
/* sort so that a<=b */
#define SORT2(a,b,smallest)       \
             if(a>b)       \
             {             \
               float c;    \
               c=a;        \
               a=b;        \
               b=c;        \
               smallest=1; \
             }             \
             else smallest=0;

template <class T>
inline void isect2(Point3<T> VTX0,Point3<T> VTX1,Point3<T> VTX2,float VV0,float VV1,float VV2,
	    float D0,float D1,float D2,float *isect0,float *isect1,Point3<T> &isectpoint0,Point3<T> &isectpoint1) 
{
  float tmp=D0/(D0-D1);          
  float diff[3];
  *isect0=VV0+(VV1-VV0)*tmp;         
  SUB(diff,VTX1,VTX0);              
  MULT(diff,diff,tmp);               
  ADD(isectpoint0,diff,VTX0);        
  tmp=D0/(D0-D2);                    
  *isect1=VV0+(VV2-VV0)*tmp;          
  SUB(diff,VTX2,VTX0);                   
  MULT(diff,diff,tmp);                 
  ADD(isectpoint1,VTX0,diff);          
}

template <class T>
inline int compute_intervals_isectline(Point3<T> VERT0,Point3<T> VERT1,Point3<T> VERT2,
				       float VV0,float VV1,float VV2,float D0,float D1,float D2,
				       float D0D1,float D0D2,float *isect0,float *isect1,
				       Point3<T> & isectpoint0, Point3<T> & isectpoint1)
{
  if(D0D1>0.0f)                                        
  {                                                    
    /* here we know that D0D2<=0.0 */                  
    /* that is D0, D1 are on the same side, D2 on the other or on the plane */
    isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,isect0,isect1,isectpoint0,isectpoint1);
  } 
  else if(D0D2>0.0f)                                   
    {                                                   
    /* here we know that d0d1<=0.0 */             
    isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,isect0,isect1,isectpoint0,isectpoint1);
  }                                                  
  else if(D1*D2>0.0f || D0!=0.0f)   
  {                                   
    /* here we know that d0d1<=0.0 or that D0!=0.0 */
    isect2(VERT0,VERT1,VERT2,VV0,VV1,VV2,D0,D1,D2,isect0,isect1,isectpoint0,isectpoint1);   
  }                                                  
  else if(D1!=0.0f)                                  
  {                                               
    isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,isect0,isect1,isectpoint0,isectpoint1); 
  }                                         
  else if(D2!=0.0f)                                  
  {                                                   
    isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,isect0,isect1,isectpoint0,isectpoint1);     
  }                                                 
  else                                               
  {                                                   
    /* triangles are coplanar */    
    return 1;
  }
  return 0;
}

#define COMPUTE_INTERVALS_ISECTLINE(VERT0,VERT1,VERT2,VV0,VV1,VV2,D0,D1,D2,D0D1,D0D2,isect0,isect1,isectpoint0,isectpoint1) \
  if(D0D1>0.0f)                                         \
  {                                                     \
    /* here we know that D0D2<=0.0 */                   \
    /* that is D0, D1 are on the same side, D2 on the other or on the plane */ \
    isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,&isect0,&isect1,isectpoint0,isectpoint1);          \
  }                                                     
#if 0
  else if(D0D2>0.0f)                                    \
  {                                                     \
    /* here we know that d0d1<=0.0 */                   \
    isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,&isect0,&isect1,isectpoint0,isectpoint1);          \
  }                                                     \
  else if(D1*D2>0.0f || D0!=0.0f)                       \
  {                                                     \
    /* here we know that d0d1<=0.0 or that D0!=0.0 */   \
    isect2(VERT0,VERT1,VERT2,VV0,VV1,VV2,D0,D1,D2,&isect0,&isect1,isectpoint0,isectpoint1);          \
  }                                                     \
  else if(D1!=0.0f)                                     \
  {                                                     \
    isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,&isect0,&isect1,isectpoint0,isectpoint1);          \
  }                                                     \
  else if(D2!=0.0f)                                     \
  {                                                     \
    isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,&isect0,&isect1,isectpoint0,isectpoint1);          \
  }                                                     \
  else                                                  \
  {                                                     \
    /* triangles are coplanar */                        \
    coplanar=1;                                         \
    return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2);      \
  }
#endif

template <class T>
bool tri_tri_intersect_with_isectline(	Point3<T> V0,Point3<T> V1,Point3<T> V2,
										Point3<T> U0,Point3<T> U1,Point3<T> U2,bool &coplanar,
										Point3<T> &isectpt1,Point3<T> &isectpt2)
{
  Point3<T> E1,E2;
  Point3<T> N1,N2;
  T d1,d2;
  float du0,du1,du2,dv0,dv1,dv2;
  Point3<T> D;
  float isect1[2], isect2[2];
  Point3<T> isectpointA1,isectpointA2;
  Point3<T> isectpointB1,isectpointB2;
  float du0du1,du0du2,dv0dv1,dv0dv2;
  short index;
  float vp0,vp1,vp2;
  float up0,up1,up2;
  float b,c,max;

  Point3<T> diff;
  int smallest1,smallest2;
  
  /* compute plane equation of triangle(V0,V1,V2) */
  SUB(E1,V1,V0);
  SUB(E2,V2,V0);
  CROSS(N1,E1,E2);
  d1=-DOT(N1,V0);
  /* plane equation 1: N1.X+d1=0 */

  /* put U0,U1,U2 into plane equation 1 to compute signed distances to the plane*/
  du0=DOT(N1,U0)+d1;
  du1=DOT(N1,U1)+d1;
  du2=DOT(N1,U2)+d1;

  /* coplanarity robustness check */
#ifdef USE_EPSILON_TEST
  if(fabs(du0)<TRI_TRI_INT_EPSILON) du0=0.0;
  if(fabs(du1)<TRI_TRI_INT_EPSILON) du1=0.0;
  if(fabs(du2)<TRI_TRI_INT_EPSILON) du2=0.0;
#endif
  du0du1=du0*du1;
  du0du2=du0*du2;

  if(du0du1>0.0f && du0du2>0.0f) /* same sign on all of them + not equal 0 ? */
    return 0;                    /* no intersection occurs */

  /* compute plane of triangle (U0,U1,U2) */
  SUB(E1,U1,U0);
  SUB(E2,U2,U0);
  CROSS(N2,E1,E2);
  d2=-DOT(N2,U0);
  /* plane equation 2: N2.X+d2=0 */

  /* put V0,V1,V2 into plane equation 2 */
  dv0=DOT(N2,V0)+d2;
  dv1=DOT(N2,V1)+d2;
  dv2=DOT(N2,V2)+d2;

#ifdef USE_EPSILON_TEST
  if(fabs(dv0)<TRI_TRI_INT_EPSILON) dv0=0.0;
  if(fabs(dv1)<TRI_TRI_INT_EPSILON) dv1=0.0;
  if(fabs(dv2)<TRI_TRI_INT_EPSILON) dv2=0.0;
#endif

  dv0dv1=dv0*dv1;
  dv0dv2=dv0*dv2;
        
  if(dv0dv1>0.0f && dv0dv2>0.0f) /* same sign on all of them + not equal 0 ? */
    return 0;                    /* no intersection occurs */

  /* compute direction of intersection line */
  CROSS(D,N1,N2);

  /* compute and index to the largest component of D */
  max=fabs(D[0]);
  index=0;
  b=fabs(D[1]);
  c=fabs(D[2]);
  if(b>max) max=b,index=1;
  if(c>max) max=c,index=2;

  /* this is the simplified projection onto L*/
  vp0=V0[index];
  vp1=V1[index];
  vp2=V2[index];
  
  up0=U0[index];
  up1=U1[index];
  up2=U2[index];

  /* compute interval for triangle 1 */
  coplanar=compute_intervals_isectline(V0,V1,V2,vp0,vp1,vp2,dv0,dv1,dv2,
				       dv0dv1,dv0dv2,&isect1[0],&isect1[1],isectpointA1,isectpointA2);
  if(coplanar) return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2);     


  /* compute interval for triangle 2 */
  compute_intervals_isectline(U0,U1,U2,up0,up1,up2,du0,du1,du2,
			      du0du1,du0du2,&isect2[0],&isect2[1],isectpointB1,isectpointB2);

  SORT2(isect1[0],isect1[1],smallest1);
  SORT2(isect2[0],isect2[1],smallest2);

  if(isect1[1]<isect2[0] || isect2[1]<isect1[0]) return 0;

  /* at this point, we know that the triangles intersect */

  if(isect2[0]<isect1[0])
  {
    if(smallest1==0) { SET(isectpt1,isectpointA1); }
    else { SET(isectpt1,isectpointA2); }

    if(isect2[1]<isect1[1])
    {
      if(smallest2==0) { SET(isectpt2,isectpointB2); }
      else { SET(isectpt2,isectpointB1); }
    }
    else
    {
      if(smallest1==0) { SET(isectpt2,isectpointA2); }
      else { SET(isectpt2,isectpointA1); }
    }
  }
  else
  {
    if(smallest2==0) { SET(isectpt1,isectpointB1); }
    else { SET(isectpt1,isectpointB2); }

    if(isect2[1]>isect1[1])
    {
      if(smallest1==0) { SET(isectpt2,isectpointA2); }
      else { SET(isectpt2,isectpointA1); }      
    }
    else
    {
      if(smallest2==0) { SET(isectpt2,isectpointB2); }
      else { SET(isectpt2,isectpointB1); } 
    }
  }
  return 1;
}

/** \addtogroup space */
/*@{*/
/**
		Triangle/triangle intersection ,based on the algorithm presented in "Fast and Robust Triangle-Triangle Overlap Test using Orientation Predicates",
		Journal of Graphics Tools, 8(1), 2003
 */
#ifdef FAST
#define ORIENT_2D(a, b, c)  ((a[0]-c[0])*(b[1]-c[1])-(a[1]-c[1])*(b[0]-c[0]))
#else
#define ORIENT_2D(a, b, c) orient2d(a, b, c)
#endif

#define INTERSECTION_TEST_VERTEX(P1, Q1, R1, P2, Q2, R2) {\
  if (ORIENT_2D(R2,P2,Q1) >= 0)\
    if (ORIENT_2D(R2,Q2,Q1) <= 0)\
      if (ORIENT_2D(P1,P2,Q1) > 0) {\
  if (ORIENT_2D(P1,Q2,Q1) <= 0) return 1; \
  else return 0;} else {\
  if (ORIENT_2D(P1,P2,R1) >= 0)\
    if (ORIENT_2D(Q1,R1,P2) >= 0) return 1; \
    else return 0;\
  else return 0;}\
    else \
      if (ORIENT_2D(P1,Q2,Q1) <= 0)\
  if (ORIENT_2D(R2,Q2,R1) <= 0)\
    if (ORIENT_2D(Q1,R1,Q2) >= 0) return 1; \
    else return 0;\
  else return 0;\
      else return 0;\
  else\
    if (ORIENT_2D(R2,P2,R1) >= 0) \
      if (ORIENT_2D(Q1,R1,R2) >= 0)\
  if (ORIENT_2D(P1,P2,R1) >= 0) return 1;\
  else return 0;\
      else \
  if (ORIENT_2D(Q1,R1,Q2) >= 0) {\
    if (ORIENT_2D(R2,R1,Q2) >= 0) return 1; \
    else return 0; }\
  else return 0; \
    else  return 0; \
 };

#define INTERSECTION_TEST_EDGE(P1, Q1, R1, P2, Q2, R2) { \
  if (ORIENT_2D(R2,P2,Q1) >= 0) {\
    if (ORIENT_2D(P1,P2,Q1) >= 0) { \
        if (ORIENT_2D(P1,Q1,R2) >= 0) return 1; \
        else return 0;} else { \
      if (ORIENT_2D(Q1,R1,P2) >= 0){ \
  if (ORIENT_2D(R1,P1,P2) >= 0) return 1; else return 0;} \
      else return 0; } \
  } else {\
    if (ORIENT_2D(R2,P2,R1) >= 0) {\
      if (ORIENT_2D(P1,P2,R1) >= 0) {\
  if (ORIENT_2D(P1,R1,R2) >= 0) return 1;  \
  else {\
    if (ORIENT_2D(Q1,R1,R2) >= 0) return 1; else return 0;}}\
      else  return 0; }\
    else return 0; }}

template <typename T>
int ccw_tri_tri_intersection_2d(const Point2<T> &p1, const Point2<T> &q1, const Point2<T> &r1,
        						const Point2<T> &p2, const Point2<T> &q2, const Point2<T> &r2) {
  if ( ORIENT_2D(p2,q2,p1) >= 0 ) {
    if ( ORIENT_2D(q2,r2,p1) >= 0 ) {
      if ( ORIENT_2D(r2,p2,p1) >= 0 ) return 1;
      else INTERSECTION_TEST_EDGE(p1,q1,r1,p2,q2,r2)
    } else {
      if ( ORIENT_2D(r2,p2,p1) >= 0 )
  INTERSECTION_TEST_EDGE(p1,q1,r1,r2,p2,q2)
      else INTERSECTION_TEST_VERTEX(p1,q1,r1,p2,q2,r2)}}
  else {
    if ( ORIENT_2D(q2,r2,p1) >= 0 ) {
      if ( ORIENT_2D(r2,p2,p1) >= 0 )
  INTERSECTION_TEST_EDGE(p1,q1,r1,q2,r2,p2)
      else  INTERSECTION_TEST_VERTEX(p1,q1,r1,q2,r2,p2)}
    else INTERSECTION_TEST_VERTEX(p1,q1,r1,r2,p2,q2)}
};


template <typename T>
int tri_tri_overlap_test_2d(const Point2<T> &p1, const Point2<T> &q1, const Point2<T> &r1,
          	  	  	  	  	const Point2<T> &p2, const Point2<T> &q2, const Point2<T> &r2) {
  if ( ORIENT_2D(p1,q1,r1) < 0 )
    if ( ORIENT_2D(p2,q2,r2) < 0 )
      return ccw_tri_tri_intersection_2d(p1,r1,q1,p2,r2,q2);
    else
      return ccw_tri_tri_intersection_2d(p1,r1,q1,p2,q2,r2);
  else
    if ( ORIENT_2D(p2,q2,r2) < 0 )
      return ccw_tri_tri_intersection_2d(p1,q1,r1,p2,r2,q2);
    else
      return ccw_tri_tri_intersection_2d(p1,q1,r1,p2,q2,r2);

};

template <typename T>
int coplanar_tri_tri3d(const Point3<T> &p1, const Point3<T> &q1, const Point3<T> &r1,
                       const Point3<T> &p2, const Point3<T> &q2, const Point3<T> &r2,
                       const Point3<T> &normal_1){

  Point2<T> P1,Q1,R1;
  Point2<T> P2,Q2,R2;

  T n_x, n_y, n_z;

  n_x = ((normal_1[0]<0)?-normal_1[0]:normal_1[0]);
  n_y = ((normal_1[1]<0)?-normal_1[1]:normal_1[1]);
  n_z = ((normal_1[2]<0)?-normal_1[2]:normal_1[2]);


  // Projection of the triangles in 3D onto 2D such that the area of
  //   the projection is maximized.
  if (( n_x > n_z ) && ( n_x >= n_y )) {
    // Project onto plane YZ

      P1[0] = q1[2]; P1[1] = q1[1];
      Q1[0] = p1[2]; Q1[1] = p1[1];
      R1[0] = r1[2]; R1[1] = r1[1];

      P2[0] = q2[2]; P2[1] = q2[1];
      Q2[0] = p2[2]; Q2[1] = p2[1];
      R2[0] = r2[2]; R2[1] = r2[1];

  } else if (( n_y > n_z ) && ( n_y >= n_x )) {
    // Project onto plane XZ

    P1[0] = q1[0]; P1[1] = q1[2];
    Q1[0] = p1[0]; Q1[1] = p1[2];
    R1[0] = r1[0]; R1[1] = r1[2];

    P2[0] = q2[0]; P2[1] = q2[2];
    Q2[0] = p2[0]; Q2[1] = p2[2];
    R2[0] = r2[0]; R2[1] = r2[2];

  } else {
    // Project onto plane XY

    P1[0] = p1[0]; P1[1] = p1[1];
    Q1[0] = q1[0]; Q1[1] = q1[1];
    R1[0] = r1[0]; R1[1] = r1[1];

    P2[0] = p2[0]; P2[1] = p2[1];
    Q2[0] = q2[0]; Q2[1] = q2[1];
    R2[0] = r2[0]; R2[1] = r2[1];
  }

  return tri_tri_overlap_test_2d(P1,Q1,R1,P2,Q2,R2);
}

#ifdef FAST
#define CHECK_MIN_MAX(p1,q1,r1,p2,q2,r2) {\
  SUB(v1,p2,q1)\
  SUB(v2,p1,q1)\
  CROSS(N1,v1,v2)\
  SUB(v1,q2,q1)\
  if (DOT(v1,N1) > 0.0f) return 0;\
  SUB(v1,p2,p1)\
  SUB(v2,r1,p1)\
  CROSS(N1,v1,v2)\
  SUB(v1,r2,p1) \
  if (DOT(v1,N1) > 0.0f) return 0;\
  else return 1; }
#else
#define CHECK_MIN_MAX(p1,q1,r1,p2,q2,r2) {\
	const double t1 = orient3d(p2, p1, q1, q2); \
	if (t1 > 0.0) return 0;\
	const double t2 = orient3d(p2, r1, p1, r2); \
	if (t2 > 0.0) return 0;\
	else return 1; }
#endif

// Permutation in a canonical form of T2's vertices

#define TRI_TRI_3D(p1,q1,r1,p2,q2,r2,dp2,dq2,dr2) { \
  if (dp2 > 0) { \
     if (dq2 > 0) CHECK_MIN_MAX(p1,r1,q1,r2,p2,q2) \
     else if (dr2 > 0) CHECK_MIN_MAX(p1,r1,q1,q2,r2,p2)\
     else CHECK_MIN_MAX(p1,q1,r1,p2,q2,r2) }\
  else if (dp2 < 0) { \
    if (dq2 < 0) CHECK_MIN_MAX(p1,q1,r1,r2,p2,q2)\
    else if (dr2 < 0) CHECK_MIN_MAX(p1,q1,r1,q2,r2,p2)\
    else CHECK_MIN_MAX(p1,r1,q1,p2,q2,r2)\
  } else { \
    if (dq2 < 0) { \
      if (dr2 >= 0)  CHECK_MIN_MAX(p1,r1,q1,q2,r2,p2)\
      else CHECK_MIN_MAX(p1,q1,r1,p2,q2,r2)\
    } \
    else if (dq2 > 0) { \
      if (dr2 > 0) CHECK_MIN_MAX(p1,r1,q1,p2,q2,r2)\
      else  CHECK_MIN_MAX(p1,q1,r1,q2,r2,p2)\
    } \
    else  { \
      if (dr2 > 0) CHECK_MIN_MAX(p1,q1,r1,r2,p2,q2)\
      else if (dr2 < 0) CHECK_MIN_MAX(p1,r1,q1,r2,p2,q2)\
      else return coplanar_tri_tri3d(p1,q1,r1,p2,q2,r2,N1);\
     }}}

template <typename T>
#ifdef FAST
bool GuigueTriTri(const Point3<T> p1,const Point3<T> q1,const Point3<T> r1,
                  const Point3<T> p2,const Point3<T> q2,const Point3<T> r2)
{
  T dp1, dq1, dr1, dp2, dq2, dr2;
  Point3<T> v1, v2;
  Point3<T> N1, N2;

  // Compute distance signs  of p1, q1 and r1 to the plane of triangle(p2,q2,r2)
  SUB(v1,p2,r2)
  SUB(v2,q2,r2)
  CROSS(N2,v1,v2)

  SUB(v1,p1,r2)
  dp1 = DOT(v1,N2);
  SUB(v1,q1,r2)
  dq1 = DOT(v1,N2);
  SUB(v1,r1,r2)
  dr1 = DOT(v1,N2);

  if (((dp1 * dq1) > 0.0f) && ((dp1 * dr1) > 0.0f))  return false;

  // Compute distance signs  of p2, q2 and r2 to the plane of triangle(p1,q1,r1)
  SUB(v1,q1,p1)
  SUB(v2,r1,p1)
  CROSS(N1,v1,v2)

  SUB(v1,p2,r1)
  dp2 = DOT(v1,N1);
  SUB(v1,q2,r1)
  dq2 = DOT(v1,N1);
  SUB(v1,r2,r1)
  dr2 = DOT(v1,N1);

  if (((dp1 * dq1) > 0.0f) && ((dp1 * dr1) > 0.0f)) return false;

#else
bool GuigueTriTri(const Point3<T> &pi1, const Point3<T> &qi1, const Point3<T> &ri1,
                  const Point3<T> &pi2, const Point3<T> &qi2, const Point3<T> &ri2)
{
  Point3<double> v1, v2;
  Point3<double> N1;

  const Point3<double> p1 = {pi1[0], pi1[1], pi1[2]};
  const Point3<double> q1 = {qi1[0], qi1[1], qi1[2]};
  const Point3<double> r1 = {ri1[0], ri1[1], ri1[2]};
  const Point3<double> p2 = {pi2[0], pi2[1], pi2[2]};
  const Point3<double> q2 = {qi2[0], qi2[1], qi2[2]};
  const Point3<double> r2 = {ri2[0], ri2[1], ri2[2]};

  SUB(v1,q1,p1)
  SUB(v2,r1,p1)
  CROSS(N1,v1,v2)
  N1.Normalize();

  // Compute distance signs  of p1, q1 and r1 to the plane of triangle(p2,q2,r2)
  const double dp1 = orient3d(p2, q2, r2, p1);
  const double dq1 = orient3d(p2, q2, r2, q1);
  const double dr1 = orient3d(p2, q2, r2, r1);

  // point vertices on same side of plane no intersection
  if (((dp1 * dq1) > 0.0) && ((dp1 * dr1) > 0.0))  return false;

  // Compute distance signs  of p2, q2 and r2 to the plane of triangle(p1,q1,r1)
  const double dp2 = orient3d(p1, q1, r1, p2);
  const double dq2 = orient3d(p1, q1, r1, q2);
  const double dr2 = orient3d(p1, q1, r1, r2);

  // point vertices on same side of plane no intersection
  if (((dp2 * dq2) > 0.0) && ((dp2 * dr2) > 0.0))  return false;
#endif


  // Permutation in a canonical form of T1's vertices
  if (dp1 > 0) {
    if (dq1 > 0) TRI_TRI_3D(r1,p1,q1,p2,r2,q2,dp2,dr2,dq2)
    else if (dr1 > 0) TRI_TRI_3D(q1,r1,p1,p2,r2,q2,dp2,dr2,dq2)
    else TRI_TRI_3D(p1,q1,r1,p2,q2,r2,dp2,dq2,dr2)
  } else if (dp1 < 0) {
    if (dq1 < 0) TRI_TRI_3D(r1,p1,q1,p2,q2,r2,dp2,dq2,dr2)
    else if (dr1 < 0) TRI_TRI_3D(q1,r1,p1,p2,q2,r2,dp2,dq2,dr2)
    else TRI_TRI_3D(p1,q1,r1,p2,r2,q2,dp2,dr2,dq2)
  } else {
    if (dq1 < 0) {
      if (dr1 >= 0) TRI_TRI_3D(q1,r1,p1,p2,r2,q2,dp2,dr2,dq2)
      else TRI_TRI_3D(p1,q1,r1,p2,q2,r2,dp2,dq2,dr2)
    }
    else if (dq1 > 0) {
      if (dr1 > 0) TRI_TRI_3D(p1,q1,r1,p2,r2,q2,dp2,dr2,dq2)
      else TRI_TRI_3D(q1,r1,p1,p2,q2,r2,dp2,dq2,dr2)
    }
    else  {
      if (dr1 > 0) TRI_TRI_3D(r1,p1,q1,p2,q2,r2,dp2,dq2,dr2)
      else if (dr1 < 0) TRI_TRI_3D(r1,p1,q1,p2,r2,q2,dp2,dr2,dq2)
      else return coplanar_tri_tri3d(p1,q1,r1,p2,q2,r2,N1);
    }
  }
}

} // end namespace

#undef FABS
#undef USE_EPSILON_TEST
#undef TRI_TRI_INT_EPSILON 
#undef CROSS
#undef DOT
#undef SUB
#undef SORT 
#undef SORT2 
#undef ADD
#undef MULT
#undef SET
#undef EDGE_EDGE_TEST
#undef EDGE_AGAINST_TRI_EDGE
#undef POINT_IN_TRI
#undef COMPUTE_INTERVALS_ISECTLINE
#undef NEWCOMPUTE_INTERVALS

#endif 
