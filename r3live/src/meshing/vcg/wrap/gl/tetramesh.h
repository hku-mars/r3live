/****************************************************************************
* MeshLab                                                           o o     *
* An extendible mesh processor                                    o     o   *
*                                                                _   O  _   *
* Copyright(C) 2005, 2009                                          \/)\/    *
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

#ifndef __GLWRAPTETRA__
#define __GLWRAPTETRA__
#include <GL/glew.h>
#include <GL/GL.h>
#include <vcg/space/color4.h>
#include <vcg/space/tetra3.h>
#include <wrap/gui/view.h>
#include <wrap/gl/space.h>
#include <wrap/gl/math.h>

namespace vcg {

namespace tetra {

class GLW {
public:
  enum DrawMode  {DMNone, DMSmallTetra,DMFlat,DMWire, DMHidden,DMTransparent,DMFlatWire} ;
	enum NormalMode{NMFlat,NMSmooth, NMUser, NMPerMesh};
	enum ColorMode {CMNone, CMPerMesh,CMUser,CMPerTetraF,CMPerVertexF,CMPerVertex};
	enum Hint {HShrinkFactor};
};

template <typename CONT_TETRA>
class GlTetramesh:public GLW{


public:
  
	typedef typename CONT_TETRA::value_type TetraType; 
  typedef typename TetraType::VertexType VertexType;
  typedef typename VertexType::ScalarType ScalarType;
  typedef typename VertexType::CoordType Point3x;

  //subclass for clipping planes
		class ClipPlane
		{
			private:
			Point3x D;
			Point3x D0;
			GLdouble eqn[4];
			vcg::Matrix44<float> TR;

			Point3x pp0;
			Point3x pp1;
			Point3x pp2;
			Point3x	pp3;

			public:

			bool active;

			Point3x P;

			ClipPlane (){active=false;}

			~ClipPlane (){}

			ClipPlane(Point3x p0, Point3x p1,Point3x p2)
			{
				Point3x N=((p1-p0)^(p2-p0)).Normalize();
				N.Normalize();
				D=N;
				D0=D;
				P=(p0+p1+p2)/3.f;

				Point3x v0=N;
				Point3x v1=(P-p0);
				v1.Normalize();
				Point3x v2=(v0^v1);
				v2.Normalize();

				v0=v0*2;
				v1=v1*2;
				v2=v2*2;

				pp0=-v1-v2;
				pp1=-v1+v2;
				pp2=v1+v2;
				pp3=v1-v2;

			}
			//set normal of the clipping plane
			void SetD(Point3x d)
			{
				D=d;
			}
			//set the point of the clipping plane
			void SetP(Point3x p)
			{
				P=p;
			}

			void GlClip()
			{
				if (active){
					GLdouble d=-(D.V(0)*P.V(0)+D.V(1)*P.V(1)+D.V(2)*P.V(2));
					eqn[0]=-D.V(0);
					eqn[1]=-D.V(1);
					eqn[2]=-D.V(2);
					eqn[3]=-d;
					glClipPlane(GL_CLIP_PLANE0, eqn); 
					glEnable(GL_CLIP_PLANE0);
				}
			}

			void GlDraw()
			{
				glPushMatrix();
				glPushAttrib(0xffffffff);
				glDisable(GL_CLIP_PLANE0);
				
				glEnable(GL_LIGHTING);
				glEnable(GL_NORMALIZE);

				glTranslate(P);
				glMultMatrix(TR);
				glLineWidth(0.5);
				glColor3d(0.7,0,0.7);
				glBegin(GL_LINE_LOOP);
					glVertex(pp0);
					glVertex(pp1);
					glVertex(pp2);
					glVertex(pp3);
				glEnd();

				glPopAttrib();
				glPopMatrix();
				
			}

			void Transform(vcg::Matrix44<float> Tr)
			{
				//thath's for casting in case of trackball using
				//float to double and vice-versa
				Point3f p=Point3f((float)D0.V(0),(float)D0.V(1),(float)D0.V(2));
				TR=Tr;
				p=TR*p;
				D=Point3x((ScalarType) p.V(0),(ScalarType) p.V(1),(ScalarType) p.V(2));
			}

			void Translate(float L)
			{
				Point3x D1=D*L;
				P+=D1;
			}

			
		};

	GlTetramesh(CONT_TETRA * _t):tetra(_t){}
	GlTetramesh( )  {}

	CONT_TETRA	* tetra;	
	ClipPlane section;

	private:
	ScalarType shrink_factor;
	

	public:
		
		void SetHint(Hint h, double value){
			switch(h){
				case HShrinkFactor: shrink_factor = value; break;
				}
			}

		void AddClipSection(Point3x p0,Point3x p1,Point3x p2)
		{
			section=ClipPlane(p0,p1,p2);
			section.active=true;
		}

		void ClearClipSection()
		{
			section.active=false;
		}

  typedef Color4b (*color_func_vertex)(VertexType&v);
	color_func_vertex  color_vertex;

	typedef Color4b (*color_func_tetra)(TetraType&v);
	color_func_tetra  color_tetra;
	

	template <DrawMode dm,NormalMode nm,ColorMode cm >

    void	Draw(){
	switch (dm){
	    case DMNone: break;
        case DMSmallTetra:_DrawSmallTetra<cm>();break;
        case DMFlat:_DrawSurface<dm,nm,cm>();break;	
        case DMWire:_DrawSurface<dm,nm,cm>();break;
		case DMHidden:_DrawSurface<dm,nm,cm>();break;
		case DMFlatWire:_DrawFlatWire<nm,cm>(); break;
        case DMTransparent:break;
				}
			}

private:
template <ColorMode cm >
 void _DrawSmallTetra(){
		Point3x p[4],br;
		typename CONT_TETRA::iterator it;
		glPushAttrib(0xffffffff);
		glEnable(GL_COLOR_MATERIAL);
		glEnable(GL_LIGHT0);
		glEnable(GL_LIGHTING);
		glEnable(GL_NORMALIZE);
		glPolygonMode(GL_FRONT,GL_FILL);
		if (section.active)
		{
			section.GlClip();
			section.GlDraw();	
		}
		/*glBegin(GL_TRIANGLES);*/
		for( it = tetra->begin(); it != tetra->end(); ++it)
			if((!it->IsD())&&(!(it->IsS()))) //draw as normal
			{
					_DrawSmallTetra<cm>(*it);
			}
			else 
			if((!it->IsD())&&((it->IsS())))//draw in selection mode
			{
					_DrawSelectedTetra(*it);
			}
		//glEnd();
		glPopAttrib();
		}

template <NormalMode nm,ColorMode cm >
void 	_DrawFlatWire(){
		glPushAttrib(0xffff);
		glEnable(GL_COLOR_MATERIAL);
		glEnable(GL_DEPTH);
		glDepthRange(0.001,1.0);
		Draw<DMFlat,nm,cm>();
		glDisable(GL_LIGHTING);
		glColor3f(0.0,0.0,0.0);
		glDepthRange(0.0,0.999);
		Draw<DMHidden,nm,cm>();
		glPopAttrib();
}


template <DrawMode dm,NormalMode nm,ColorMode cm >
void _DrawSurface(){
		typename CONT_TETRA::iterator it;

		glPushAttrib(0xffff);
		glEnable(GL_COLOR_MATERIAL);
		if((dm == DMWire)||(dm ==DMHidden))
		{
			glDisable(GL_LIGHTING);
			glDisable(GL_NORMALIZE);
			glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
		}
		else
		{
			glEnable(GL_LIGHTING);
			glEnable(GL_NORMALIZE);
			glPolygonMode(GL_FRONT,GL_FILL);
		}
		//glBegin(GL_TRIANGLES);
		for( it = tetra->begin(); it != tetra->end(); ++it)
			_DrawTetra<dm,nm,cm>((*it));
	  //glEnd();
	  glPopAttrib();
}


void _DrawSelectedTetra(TetraType &t)
{
	glPushMatrix();
	glPushAttrib(0xffff);
	glDisable(GL_CLIP_PLANE0);
	glDisable(GL_BLEND);
	glDisable(GL_LIGHTING);
	glDisable(GL_NORMALIZE);
	glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);

	glColor3d(1,0,0);

	glBegin(GL_TRIANGLES);
	for (int face=0;face<4;face++)
	{
		glVertex(t.V(Tetra::VofF(face,0))->P());
		glVertex(t.V(Tetra::VofF(face,1))->P());
		glVertex(t.V(Tetra::VofF(face,2))->P());
	}
	glEnd();

	//end drawing
	glPopAttrib();
	glPopMatrix();
}

template <DrawMode dm,NormalMode nm,ColorMode cm >
void _DrawTetra(TetraType &t)
{
  if((!t.IsD())&&(!t.IsS()))
  {
       if ((dm!=DMWire)&&(dm!=DMHidden))
          _ChooseColorTetra<cm>(t);
       for(int i = 0; i < 4; ++i){
         if (dm == DMWire)
           _DrawFace<cm>(t,i);
         else
         {
           if (t.IsBorderF(i))
           {
              if(nm==NMSmooth)
			     _DrawFaceSmooth<cm>(t,i);
              else
			  if(nm==NMFlat)
			     _DrawFace<cm>(t,i);
           }
         }
       }
      }
  else 
	if((!t.IsD())&&(t.IsS()))
		_DrawSelectedTetra(t);			
}

template <ColorMode cm >
void _ChooseColorTetra(TetraType &t)
{
  if (cm==CMNone)
  {
	  if (t.IsS())
		glColor3d(1,0,0);
	  else
		glColor3d(0.8f,0.8f,0.8f);
  }
  else
  if(cm == CMPerTetraF)
      {
				 Color4b c;
				 c = color_tetra(t);
				 GLint ic[4]; ic[0] = c[0];ic[1] = c[1];ic[2] = c[2];ic[3] = c[3];
				 glMaterialiv(GL_FRONT,GL_DIFFUSE ,ic);
	  }
}

template <ColorMode cm >
void _ChooseColorVertex(VertexType &v)
{
  if (cm!=CMNone)
  {
  if(cm == CMPerVertexF)
      {
				 Color4b c;
				 c = color_vertex(v);
				 GLint ic[4]; ic[0] = c[0];ic[1] = c[1];ic[2] = c[2];ic[3] = c[3];
				 glMaterialiv(GL_FRONT,GL_DIFFUSE ,ic);
	 }
  else
  if(cm == CMPerVertex)
						glColor3f(v.C()[0],v.C()[1],v.C()[2]);
  }
}

template <ColorMode cm >
void _DrawFaceSmooth(TetraType &t,int face)
{

  VertexType *v0=t.V(Tetra::VofF(face,0));
  VertexType *v1=t.V(Tetra::VofF(face,1));
  VertexType *v2=t.V(Tetra::VofF(face,2));

  glBegin(GL_TRIANGLES);
	_ChooseColorVertex<cm>(*v0);
	glNormal(v0->N());
	glVertex(v0->P());
	_ChooseColorVertex<cm>(*v1);
	glNormal(v1->N());
	glVertex(v1->P());
	_ChooseColorVertex<cm>(*v2);
	glNormal(v2->N());
	glVertex(v2->P());
  glEnd();
}

template <ColorMode cm >
void _DrawFace(TetraType &t,int face)
{
glBegin(GL_TRIANGLES);
  glNormal(t.N(face));
  VertexType *v0=t.V(Tetra::VofF(face,0));
  VertexType *v1=t.V(Tetra::VofF(face,1));
  VertexType *v2=t.V(Tetra::VofF(face,2));
  _ChooseColorVertex<cm>(*v0);
  glVertex(v0->P());
  _ChooseColorVertex<cm>(*v1);
  glVertex(v1->P());
  _ChooseColorVertex<cm>(*v2);
  glVertex(v2->P());
  glEnd();
}

template <ColorMode cm >
void _DrawSmallTetra(TetraType &t)
{
  Tetra3<ScalarType> T=Tetra3<ScalarType>();
  T.P0(0)=t.V(0)->cP();
  T.P1(0)=t.V(1)->cP();
  T.P2(0)=t.V(2)->cP();
  T.P3(0)=t.V(3)->cP();
  Point3x p[4], br;
  br=T.ComputeBarycenter();
  for(int i = 0; i < 4; ++i)
					p[i] = t.V(i)->P()* shrink_factor + br *(1- shrink_factor);
  _ChooseColorTetra<cm>(t);

  glBegin(GL_TRIANGLES);
  for(int i = 0; i < 4; ++i)
    {
		glNormal(t.N(i));
        VertexType *v0=t.V(Tetra::VofF(i,0));
        VertexType *v1=t.V(Tetra::VofF(i,1));
        VertexType *v2=t.V(Tetra::VofF(i,2));
        _ChooseColorVertex<cm>(*v0);
        glVertex(p[Tetra::VofF(i,0)]);
        _ChooseColorVertex<cm>(*v1);
		glVertex(p[Tetra::VofF(i,1)]);
        _ChooseColorVertex<cm>(*v2);
		glVertex(p[Tetra::VofF(i,2)]);
	}
	glEnd();
}


};

} // end namespace tetra	
} // end nemaspace tri
#endif
