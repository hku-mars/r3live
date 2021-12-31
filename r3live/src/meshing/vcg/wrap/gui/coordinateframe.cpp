/****************************************************************************
 * MeshLab                                                           o o     *
 * A versatile mesh processing toolbox                             o     o   *
 *                                                                _   O  _   *
 * Copyright(C) 2008                                                \/)\/    *
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
Revision 1.8  2008/03/02 16:44:18  benedetti
moved ActiveCoordinateFrame to its own files

Revision 1.7  2008/02/26 18:22:42  benedetti
corrected after quaternion/similarity/trackball changes

Revision 1.6  2008/02/22 20:34:35  benedetti
corrected typo

Revision 1.5  2008/02/22 20:04:02  benedetti
many user interface improvements, cleaned up a little

Revision 1.4  2008/02/17 20:52:53  benedetti
some generalization made

Revision 1.3  2008/02/16 14:12:30  benedetti
first version


****************************************************************************/

#include <GL/glew.h>
#include <wrap/gl/math.h>
#include <wrap/gl/space.h>
#include <wrap/gl/addons.h>
#include <wrap/qt/gl_label.h>

#include "coordinateframe.h"

using namespace vcg;

CoordinateFrame::CoordinateFrame(float s)
:basecolor(Color4b::White),xcolor(Color4b::Red)
,ycolor(Color4b::Green),zcolor(Color4b::Blue),size(s),linewidth(2.0)
,font(),drawaxis(true),drawlabels(true),drawvalues(false)
{
  font.setFamily("Helvetica");
}

void CoordinateFrame::Render(QGLWidget* glw,QPainter* p)
{
  assert( glw!= NULL);
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_POINT_SMOOTH);
  glLineWidth(linewidth);
  glPointSize(linewidth*1.5);
  glLabel::Mode md;
  Point3d o(0,0,0);
  Point3d a(size,0,0);
  Point3d b(0,size,0);
  Point3d c(0,0,size);
  // Get gl state values
  double mm[16],mp[16];
  GLint vp[4];
  glGetDoublev(GL_MODELVIEW_MATRIX,mm);
  glGetDoublev(GL_PROJECTION_MATRIX,mp);
  glGetIntegerv(GL_VIEWPORT,vp);
  float slope_a=calcSlope(-a,a,2*size,10,mm,mp,vp);
  float slope_b=calcSlope(-b,b,2*size,10,mm,mp,vp);
  float slope_c=calcSlope(-c,c,2*size,10,mm,mp,vp);
  float scalefactor = size*0.02f;
  if(drawaxis){
    glBegin(GL_LINES);
      glColor(xcolor);
      glVertex(-a); glVertex(a);
      glColor(ycolor);
      glVertex(-b); glVertex(b);
      glColor(zcolor);
      glVertex(-c); glVertex(c);
    glEnd();
    glColor(basecolor);
    // positive axes
    drawTickedLine(o,a,size,slope_a,linewidth);  // Draws x axis
    drawTickedLine(o,b,size,slope_b,linewidth);  // Draws y axis
    drawTickedLine(o,c,size,slope_c,linewidth);  // Draws z axis
    // negative axes
    drawTickedLine(o,-a,size,slope_a,linewidth);  // Draws x axis
    drawTickedLine(o,-b,size,slope_b,linewidth);  // Draws y axis
    drawTickedLine(o,-c,size,slope_c,linewidth);  // Draws z axis
    glPushMatrix();
      glTranslate(a);
      glScalef(scalefactor,scalefactor,scalefactor);
      Add_Ons::Cone(10,linewidth*1.5,linewidth*0.5,true);
    glPopMatrix();
    glPushMatrix();
      glTranslate(b);
      glRotatef(90,0,0,1);
      glScalef(scalefactor,scalefactor,scalefactor);
      Add_Ons::Cone(10,linewidth*1.5,linewidth*0.5,true);
    glPopMatrix();
    glPushMatrix();
      glTranslate(c);
      glRotatef(-90,0,1,0);
      glScalef(scalefactor,scalefactor,scalefactor);
      Add_Ons::Cone(10,linewidth*1.5,linewidth*0.5,true);
    glPopMatrix();
  }
  if(drawlabels){
    md.qFont.setBold(true);
    md.qFont.setPixelSize(12);
    float d=size+scalefactor*linewidth*1.5;
    if (p) {
      vcg::glLabel::render(p,vcg::Point3f(d,0,0),QString("X"),md);
      vcg::glLabel::render(p,vcg::Point3f(0,d,0),QString("Y"),md);
      vcg::glLabel::render(p,vcg::Point3f(0,0,d),QString("Z"),md);
    }
  }
  if(drawvalues){
    md.qFont.setBold(false);
    md.qFont.setPixelSize(8);
    md.color=Color4b(Color4b::LightGray);
    float i;
    glColor(Color4b::LightGray);
    for(i=slope_a;i<size;i+=slope_a){
      vcg::glLabel::render(p,vcg::Point3f( i,0,0),QString(" %1").arg(i,3,'f',1),md);
      vcg::glLabel::render(p,vcg::Point3f(-i,0,0),QString(" %1").arg(i,3,'f',1),md);
    }
    for(i=slope_b;i<size;i+=slope_b){
      vcg::glLabel::render(p,vcg::Point3f(0, i,0),QString(" %1").arg(i,3,'f',1),md);
      vcg::glLabel::render(p,vcg::Point3f(0,-i,0),QString(" %1").arg(i,3,'f',1),md);
    }
    for(i=slope_c;i<size;i+=slope_c){
      vcg::glLabel::render(p,vcg::Point3f(0,0, i),QString(" %1").arg(i,3,'f',1),md);
      vcg::glLabel::render(p,vcg::Point3f(0,0,-i),QString(" %1").arg(i,3,'f',1),md);
    }
  }
  glGetError(); // Patch to buggy qt rendertext;
  glPopAttrib();
  assert(!glGetError());  
}

void CoordinateFrame::drawTickedLine(const Point3d &a,const Point3d &b, float dim,float tickDist,float linewidth)
{
  Point3d v(b-a);
  v = v /dim; // normalize without computing square roots and powers

  glBegin(GL_POINTS);
  float i;
  for(i=tickDist;i<dim;i+=tickDist)
    glVertex3f(a[0] + i*v[0],a[1] + i*v[1],a[2] + i*v[2]);
  glEnd();

  glPushAttrib(GL_POINT_BIT);
  glPointSize(linewidth*3);  
  glBegin(GL_POINTS);
       glVertex3f(a[0] + dim*v[0],a[1] + dim*v[1],a[2] + dim*v[2]);
  glEnd();

  glPopAttrib();
}

float CoordinateFrame::calcSlope(const Point3d &a,const Point3d &b,float dim,int spacing,double *mm,double *mp,GLint *vp)
{
   Point3d p1,p2;

  gluProject(a[0],a[1],a[2],mm,mp,vp,&p1[0],&p1[1],&p1[2]);
  gluProject(b[0],b[1],b[2],mm,mp,vp,&p2[0],&p2[1],&p2[2]);
  p1[2]=p2[2]=0;

  float tickNum = spacing/Distance(p2,p1);// pxl spacing
  float slope = dim*tickNum;
  float nslope = math::Min(niceRound(slope), 0.5f*niceRound(2.0f*slope), 0.2f*niceRound(5.0f*slope));
  nslope = std::max(niceRound(dim*.001f),nslope); // prevent too small slope
  return nslope;
}

float CoordinateFrame::niceRound(float val)
{
  return powf(10.f,ceil(log10(val)));
}

MovableCoordinateFrame::MovableCoordinateFrame(float size)
:CoordinateFrame(size),position(0,0,0),rotation(0,Point3f(1,0,0))
{
  // nothing here
}

void MovableCoordinateFrame::Render(QGLWidget* gla)
{
  glPushMatrix();
  
  glTranslate(position);  
  Matrix44f mrot; 
  rotation.ToMatrix(mrot);

  glMultMatrix(Inverse(mrot));
  
  CoordinateFrame::Render(gla);
  
  glPopMatrix();
}

void MovableCoordinateFrame::GetTransform(Matrix44f & transform)
{
  // build the matrix that moves points in world coordinates

  // clean transform
  transform.SetIdentity();

  // apply rotation
  Matrix44f rot;
  rotation.ToMatrix(rot);

  transform = Inverse(rot) * transform ;
  
  // apply translation
  Matrix44f pos;
  pos.SetTranslate(position);
  
  transform = pos * transform;
  
}

void MovableCoordinateFrame::Reset(bool reset_position,bool reset_alignment)
{
  if(reset_position)
    position = Point3f(0,0,0);
  if(reset_alignment)
    rotation = Quaternionf(0,Point3f(1,0,0));
}

void MovableCoordinateFrame::SetPosition(const Point3f newpos)
{
  position = newpos;
}

void MovableCoordinateFrame::SetRotation(const Quaternionf newrot)
{
  rotation = newrot;
}

Point3f MovableCoordinateFrame::GetPosition()
{
  return position;
}

Quaternionf MovableCoordinateFrame::GetRotation()
{
  return rotation;
}

void MovableCoordinateFrame::Rot(float angle_deg,const Point3f axis)
{
  Similarityf s;
	s.SetRotate(math::ToRad(angle_deg),(rotation).Rotate(axis));
  Move(s);
}

void MovableCoordinateFrame::AlignWith(const Point3f pri,const Point3f secondary,const char c1, const char c2)
{
  const float EPSILON=1e-6f;
  Point3f primary=pri;

  if( primary.Norm() < EPSILON*size )
    return;

  primary.Normalize();
  Plane3f plane(0,primary); // projection plane for the second rotation
   
  Point3f x(1,0,0),y(0,1,0),z(0,0,1);
  Point3f first(0,0,0),second(0,0,0),third(0,0,0);

  if(c1=='X'){ first = x;
    if((c2=='Y')||(c2==' ')){ second = y; third = z; } 
    else if(c2=='Z'){ second = z; third = y; } 
    else assert (0);
  } else if(c1=='Y'){ first = y;
    if((c2=='Z')||(c2==' ')){ second = z; third = x; } 
    else if(c2=='X'){ second = x; third = z; }
    else assert (0);
  } else if(c1=='Z'){ first = z;
    if((c2=='X')||(c2==' ')){ second = x; third = y; }
    else if(c2=='Y'){ second = y; third = x; }
    else assert (0);   
  } else assert (0);

  Point3f old_first = Inverse(rotation).Rotate(first); // axis 1
  Point3f old_second_pro = plane.Projection(Inverse(rotation).Rotate(second)); // axis 2 projection
  Point3f old_third_pro = plane.Projection(Inverse(rotation).Rotate(third)); // axis 3 projection

  // align axis 1 to primary
  RotateToAlign(old_first,primary);

  Point3f secondary_pro = plane.Projection(secondary); // secondary's projection
  Point3f new_second_pro = plane.Projection(Inverse(rotation).Rotate(second)); // axis 2 projection after the first rotation

  if( secondary.Norm() > EPSILON*size && secondary_pro.Norm() > EPSILON ){ // secondary is not null nor parallel to primary
    // align axis 2 projection after the first rotation to secondary's projection
    secondary_pro.Normalize();
    RotateToAlign(new_second_pro,secondary_pro);
    return;
  }

  if ( old_second_pro.Norm() > EPSILON ) { // can realign axis 2
    // align axis 2 projection after the first rotation to old axis 2 projection
    old_second_pro.Normalize();
    RotateToAlign(new_second_pro,old_second_pro);
    return;
  }

  // realign axis 3
  Point3f new_third_pro = plane.Projection(Inverse(rotation).Rotate(third));// axis 3 projection after the first rotation
  assert(old_third_pro.Norm() > EPSILON ); // old axis 3 projection should not be null
  // align axis 3 projection after the first rotation to old axis 3 projection
  old_third_pro.Normalize();
  RotateToAlign(new_third_pro,old_third_pro);
}

void MovableCoordinateFrame::Move(const Similarityf track)
{
  position = position + track.tra;
  rotation = rotation * Inverse(track.rot);
}

void MovableCoordinateFrame::RotateToAlign(const Point3f source, const Point3f dest)
{
  const float EPSILON=1e-6f;
  // source and dest must be versors
  assert( math::Abs(source.Norm() - 1) < EPSILON);
  assert( math::Abs(dest.Norm() - 1) < EPSILON);

  Point3f axis = dest ^ source;
  float sinangle = axis.Norm();
  float cosangle = dest.dot(source);
  float angle = math::Atan2(sinangle,cosangle);  

  if( math::Abs(angle) < EPSILON )    
    return; // angle ~ 0, aborting

  if( math::Abs(math::Abs(angle)-M_PI) < EPSILON){
    // must find a axis to flip on
    Plane3f plane(0,source);
    axis=plane.Projection(Point3f(1,0,0)); // project a "random" point on source's normal plane
  	if(axis.Norm() < EPSILON){ // source was ~ [1,0,0]...
  	  axis=plane.Projection(Point3f(0,1,0)); 
      assert(axis.Norm() > EPSILON); // this point must be good
  	}
  }
  rotation = rotation * Quaternionf(angle,axis);
}
