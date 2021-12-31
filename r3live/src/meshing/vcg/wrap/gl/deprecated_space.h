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
Revision 1.10  2007/07/31 12:21:50  ganovelli
added gltetra, added normal gltriangle

Revision 1.9  2007/05/08 18:55:38  ganovelli
glTriangle added

Revision 1.8  2007/01/18 01:26:23  cignoni
Added cast for mac compiling

Revision 1.7  2005/10/13 08:32:26  cignoni
Added glscale(scalar) and corrected bug in glscale(point2)

Revision 1.6  2005/06/30 10:17:04  ganovelli
added draw plane

Revision 1.5  2005/05/05 12:28:13  cignoni
added glboxwire

Revision 1.4  2004/07/07 23:30:28  cignoni
Added box3 drawing functions

Revision 1.3  2004/05/26 15:13:01  cignoni
Removed inclusion of gl extension stuff and added glcolor stuff

Revision 1.2  2004/05/13 23:44:47  ponchio
<GL/GL.h>  -->   <GL/gl.h>

Revision 1.1  2004/04/05 11:56:14  cignoni
First working version!


****************************************************************************/

#ifndef VCG_GL_SPACE_H
#define VCG_GL_SPACE_H

// Please note that this file assume that you have already included your
// gl-extension wrapping utility, and that therefore all the extension symbol are already defined.

#include <vcg/space/triangle3.h>
#include <vcg/space/plane3.h>
#include <vcg/space/point2.h>
#include <vcg/space/point3.h>
#include <vcg/space/color4.h>
#include <vcg/space/box2.h>
#include <vcg/space/box3.h>

namespace vcg {

	inline void glScale(float const & p){ glScalef(p,p,p);}
	inline void glScale(double const & p){ glScaled(p,p,p);}

	inline void glVertex(Point2<int> const & p)   { glVertex2iv((const GLint*)p.V());}
	inline void glVertex(Point2<short> const & p) { glVertex2sv(p.V());}
	inline void glVertex(Point2<float> const & p) { glVertex2fv(p.V());}
	inline void glVertex(Point2<double> const & p){ glVertex2dv(p.V());}
	inline void glTexCoord(Point2<int> const & p)   { glTexCoord2iv((const GLint*)p.V());}
	inline void glTexCoord(Point2<short> const & p) { glTexCoord2sv(p.V());}
	inline void glTexCoord(Point2<float> const & p) { glTexCoord2fv(p.V());}
	inline void glTexCoord(Point2<double> const & p){ glTexCoord2dv(p.V());}
	inline void glTranslate(Point2<float> const & p) { glTranslatef(p[0],p[1],0);}
	inline void glTranslate(Point2<double> const & p){ glTranslated(p[0],p[1],0);}
	inline void glScale(Point2<float> const & p) { glScalef(p[0],p[1],1.0);}
	inline void glScale(Point2<double> const & p){ glScaled(p[0],p[1],1.0);}

  inline void glVertex(Point3<int> const & p)   { glVertex3iv((const GLint*)p.V());}
	inline void glVertex(Point3<short> const & p) { glVertex3sv(p.V());}
	inline void glVertex(Point3<float> const & p) { glVertex3fv(p.V());}
	inline void glVertex(Point3<double> const & p){ glVertex3dv(p.V());}
	inline void glNormal(Point3<int> const & p)   { glNormal3iv((const GLint*)p.V());}
	inline void glNormal(Point3<short> const & p) { glNormal3sv(p.V());}
	inline void glNormal(Point3<float> const & p) { glNormal3fv(p.V());}
	inline void glNormal(Point3<double> const & p){ glNormal3dv(p.V());}
	inline void glTexCoord(Point3<int> const & p)   { glTexCoord3iv((const GLint*)p.V());}
	inline void glTexCoord(Point3<short> const & p) { glTexCoord3sv(p.V());}
	inline void glTexCoord(Point3<float> const & p) { glTexCoord3fv(p.V());}
	inline void glTexCoord(Point3<double> const & p){ glTexCoord3dv(p.V());}
	inline void glTranslate(Point3<float> const & p) { glTranslatef(p[0],p[1],p[2]);}
	inline void glTranslate(Point3<double> const & p){ glTranslated(p[0],p[1],p[2]);}
	inline void glScale(Point3<float> const & p) { glScalef(p[0],p[1],p[2]);}
	inline void glScale(Point3<double> const & p){ glScaled(p[0],p[1],p[2]);}

  inline void glColor(Color4b const & c)   { glColor4ubv(c.V());}
  inline void glColor(Color4f const & c)   { glColor4fv (c.V());}
  inline void glColor(Color4d const & c)   { glColor4dv (c.V());}
  inline void glClearColor(Color4b const &c) { ::glClearColor(float(c[0])/255.0f,float(c[1])/255.0f,float(c[2])/255.0f,1.0f);}
  inline void glClearColor(Color4f const &c) { ::glClearColor(c[0],c[1],c[2],c[3]); }
  inline void glClearColor(Color4d const &c) { ::glClearColor(float(c[0]),float(c[1]),float(c[2]),float(c[3])); }
  inline void glLight(GLenum light, GLenum pname,  Color4b const & c) {
    static float cf[4];
    cf[0]=float(cf[0]/255.0); cf[1]=float(c[1]/255.0); cf[2]=float(c[2]/255.0); cf[3]=float(c[3]/255.0);
    glLightfv(light,pname,cf);
  }


 template <class T>
   inline void glBoxWire(Box3<T> const & b)
{
	glPushAttrib(GL_ENABLE_BIT);
	glDisable(GL_LIGHTING);
	glBegin(GL_LINE_STRIP);
	glVertex3f((float)b.min[0],(float)b.min[1],(float)b.min[2]);
	glVertex3f((float)b.max[0],(float)b.min[1],(float)b.min[2]);
	glVertex3f((float)b.max[0],(float)b.max[1],(float)b.min[2]);
	glVertex3f((float)b.min[0],(float)b.max[1],(float)b.min[2]);
	glVertex3f((float)b.min[0],(float)b.min[1],(float)b.min[2]);
	glEnd();
	glBegin(GL_LINE_STRIP);
	glVertex3f((float)b.min[0],(float)b.min[1],(float)b.max[2]);
	glVertex3f((float)b.max[0],(float)b.min[1],(float)b.max[2]);
	glVertex3f((float)b.max[0],(float)b.max[1],(float)b.max[2]);
	glVertex3f((float)b.min[0],(float)b.max[1],(float)b.max[2]);
	glVertex3f((float)b.min[0],(float)b.min[1],(float)b.max[2]);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f((float)b.min[0],(float)b.min[1],(float)b.min[2]);
	glVertex3f((float)b.min[0],(float)b.min[1],(float)b.max[2]);

	glVertex3f((float)b.max[0],(float)b.min[1],(float)b.min[2]);
	glVertex3f((float)b.max[0],(float)b.min[1],(float)b.max[2]);

	glVertex3f((float)b.max[0],(float)b.max[1],(float)b.min[2]);
	glVertex3f((float)b.max[0],(float)b.max[1],(float)b.max[2]);

	glVertex3f((float)b.min[0],(float)b.max[1],(float)b.min[2]);
	glVertex3f((float)b.min[0],(float)b.max[1],(float)b.max[2]);
	glEnd();
	glPopAttrib();
};
template <class T>
	/// Funzione di utilita' per la visualizzazione in OpenGL (flat shaded)
inline void glBoxFlat(Box3<T> const & b)
{
	glPushAttrib(GL_SHADE_MODEL);
	glShadeModel(GL_FLAT);
	glBegin(GL_QUAD_STRIP);
	glNormal3f(.0f,.0f,1.0f);
	glVertex3f(b.min[0], b.max[1], b.max[2]);
  glVertex3f(b.min[0], b.min[1], b.max[2]);
	glVertex3f(b.max[0], b.max[1], b.max[2]);
  glVertex3f(b.max[0], b.min[1], b.max[2]);
	glNormal3f(1.0f,.0f,.0f);
	glVertex3f(b.max[0], b.max[1], b.min[2]);
	glVertex3f(b.max[0], b.min[1], b.min[2]);
  glNormal3f(.0f,.0f,-1.0f);
	glVertex3f(b.min[0], b.max[1], b.min[2]);
	glVertex3f(b.min[0], b.min[1], b.min[2]);
	glNormal3f(-1.0f,.0f,.0f);
	glVertex3f(b.min[0], b.max[1], b.max[2]);
	glVertex3f(b.min[0], b.min[1], b.max[2]);
	glEnd();

  glBegin(GL_QUADS);
	glNormal3f(.0f,1.0f,.0f);
	glVertex3f(b.min[0], b.max[1], b.max[2]);
	glVertex3f(b.max[0], b.max[1], b.max[2]);
	glVertex3f(b.max[0], b.max[1], b.min[2]);
  glVertex3f(b.min[0], b.max[1], b.min[2]);

	glNormal3f(.0f,-1.0f,.0f);
	glVertex3f(b.min[0], b.min[1], b.min[2]);
  glVertex3f(b.max[0], b.min[1], b.min[2]);
	glVertex3f(b.max[0], b.min[1], b.max[2]);
  glVertex3f(b.min[0], b.min[1], b.max[2]);
  glEnd();
	glPopAttrib();
};


template <class T>
	/// Setta i sei clip planes di opengl a far vedere solo l'interno del box
inline void glBoxClip(const Box3<T>  & b)
{
	double eq[4];
	eq[0]= 1; eq[1]= 0; eq[2]= 0; eq[3]=(double)-b.min[0];
	glClipPlane(GL_CLIP_PLANE0,eq);
	eq[0]=-1; eq[1]= 0; eq[2]= 0; eq[3]=(double) b.max[0];
	glClipPlane(GL_CLIP_PLANE1,eq);

	eq[0]= 0; eq[1]= 1; eq[2]= 0; eq[3]=(double)-b.min[1];
	glClipPlane(GL_CLIP_PLANE2,eq);
	eq[0]= 0; eq[1]=-1; eq[2]= 0; eq[3]=(double) b.max[1];
	glClipPlane(GL_CLIP_PLANE3,eq);


	eq[0]= 0; eq[1]= 0; eq[2]= 1; eq[3]=(double)-b.min[2];
	glClipPlane(GL_CLIP_PLANE4,eq);
	eq[0]= 0; eq[1]= 0; eq[2]=-1; eq[3]=(double) b.max[2];
	glClipPlane(GL_CLIP_PLANE5,eq);
}
 template <class T>
   inline void glBoxWire(const Box2<T>  & b)
{
	glPushAttrib(GL_ENABLE_BIT);
	glDisable(GL_LIGHTING);
	glBegin(GL_LINE_LOOP);

  glVertex2f((float)b.min[0],(float)b.min[1]);
	  glVertex2f((float)b.max[0],(float)b.min[1]);
	  glVertex2f((float)b.max[0],(float)b.max[1]);
	  glVertex2f((float)b.min[0],(float)b.max[1]);
  glEnd();

	glPopAttrib();
};
 template <class T>
	inline void glPlane3( Plane3<T>   p, Point3<T>  c, T size )  {
		Point3<T> w = p.Direction();
		Point3<T> u,v,c1;
		GetUV<T>(w,u,v);

		c1 = p.Projection(c);

		u.Normalize();
		w.Normalize();
		v.Normalize();

		Matrix44<T> m;
	  *(Point3<T>*)&m[0][0] = *(Point3<T>*)&u[0];m[0][3]=0;
		*(Point3<T>*)&m[1][0] = *(Point3<T>*)&w[0];m[1][3]=0;
		*(Point3<T>*)&m[2][0] = *(Point3<T>*)&v[0];m[2][3]=0;
		*(Point3<T>*)&m[3][0] = *(Point3<T>*)&c1[0];m[3][3]=1;

		glPushMatrix();
		glMultMatrix(m.transpose());

		glBegin(GL_QUADS);
		glNormal(Point3<T>(0,1,0));
		glVertex(Point3<T>(-size,0,-size));
		glVertex(Point3<T>(size ,0,-size));
		glVertex(Point3<T>(size ,0, size));
		glVertex(Point3<T>(-size,0, size));
		glEnd();


		glPopMatrix();
	}


template <class TriangleType>
	inline void glTriangle3(  TriangleType & c )  {
		vcg::Point3<typename TriangleType::ScalarType> n =  vcg::Normal(c);
		glBegin(GL_TRIANGLES);
		glNormal(n);
		glVertex(c.P(0));
		glVertex(c.P(1));
		glVertex(c.P(2));
		glEnd();
	}

template <class TetraType>
	inline void glTetra3(  TetraType & c )  {
		glTriangle3(Triangle3<typename TetraType::ScalarType>(c.P(0),c.P(1),c.P(2)));
		glTriangle3(Triangle3<typename TetraType::ScalarType>(c.P(1),c.P(3),c.P(2)));
		glTriangle3(Triangle3<typename TetraType::ScalarType>(c.P(0),c.P(2),c.P(3)));
		glTriangle3(Triangle3<typename TetraType::ScalarType>(c.P(1),c.P(0),c.P(3)));
	}

#ifdef VCG_USE_EIGEN

template<typename Derived, int Rows=Derived::RowsAtCompileTime, int Cols=Derived::ColsAtCompileTime>
struct EvalToKnownPointType;

template<typename Derived> struct EvalToKnownPointType<Derived,2,1>
{ typedef Point2<typename Derived::Scalar> Type; };

template<typename Derived> struct EvalToKnownPointType<Derived,3,1>
{ typedef Point3<typename Derived::Scalar> Type; };

template<typename Derived> struct EvalToKnownPointType<Derived,4,1>
{ typedef Point4<typename Derived::Scalar> Type; };

#define _WRAP_EIGEN_XPR(FUNC) template<typename Derived>  \
	inline void FUNC(const Eigen::MatrixBase<Derived>& p) { \
		FUNC(typename EvalToKnownPointType<Derived>::Type(p)); }

_WRAP_EIGEN_XPR(glVertex)
_WRAP_EIGEN_XPR(glNormal)
_WRAP_EIGEN_XPR(glTexCoord)
_WRAP_EIGEN_XPR(glTranslate)
_WRAP_EIGEN_XPR(glScale)

#endif

}//namespace
#endif
