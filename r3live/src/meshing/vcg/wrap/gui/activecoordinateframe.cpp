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
Revision 1.1  2008/03/02 16:44:18  benedetti
moved ActiveCoordinateFrame to its own files


****************************************************************************/

#include <GL/glew.h>
#include <wrap/gl/math.h>
#include <wrap/gl/space.h>
#include <wrap/gl/addons.h>

#include "activecoordinateframe.h"

using namespace vcg;

ActiveCoordinateFrame::ActiveCoordinateFrame(float size)
:MovableCoordinateFrame(size),manipulator(NULL),drawmoves(true),
drawrotations(true),move_button(Trackball::BUTTON_RIGHT),
rotate_button(Trackball::BUTTON_LEFT),x_modifier(Trackball::BUTTON_NONE),
y_modifier(Trackball::KEY_CTRL),z_modifier(Trackball::KEY_SHIFT),
x_axis(1,0,0),y_axis(0,1,0),z_axis(0,0,1),rot_snap_rad(0.0f),mov_snap(0.0f),
movx(move_button | x_modifier),movy(move_button | y_modifier),
movz(move_button | z_modifier),rotx(rotate_button | x_modifier),
roty(rotate_button | y_modifier),rotz(rotate_button | z_modifier)
{
  manipulator=new Trackball();
  Update();
 }

ActiveCoordinateFrame::~ActiveCoordinateFrame()
{
   if(manipulator!=NULL) {
     delete manipulator;
     manipulator=NULL;
  }
}

void ActiveCoordinateFrame::Render(QGLWidget* glw)
{
  glPushMatrix();

  manipulator->radius=size;
  manipulator->center=position;
  manipulator->GetView();
  manipulator->Apply();
   
  MovableCoordinateFrame::Render(glw);
  
  // got nothing to draw
  if(!drawmoves && !drawrotations){
    glPopMatrix();
    return;  
  }

  int current_mode=manipulator->current_button;  
  bool rotating=(current_mode==rotx)||(current_mode==roty)||(current_mode==rotz);
  bool moving=(current_mode==movx)||(current_mode==movy)||(current_mode==movz);

  // maybe got something to draw
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_POINT_SMOOTH);
  
  QString message("this should never be seen");
  char axis_name;
  float verse;
  
  if(current_mode==x_modifier){
    glColor(xcolor); message = QString("move or rotate on X axis");
  } else if(current_mode==y_modifier){
    glColor(ycolor); message = QString("move or rotate on Y axis");
  } else if(current_mode==z_modifier){
    glColor(zcolor); message = QString("move or rotate on Z axis");
  } else 
  if(rotating && drawrotations){ // draw a rotation
    Point3f axis, arc_point;
    float angle;
    manipulator->track.rot.ToAxis(angle,axis);
    angle = -angle;
    if(current_mode==rotx){
      verse=((axis+x_axis).Norm()<1?-1:1);
      glColor(xcolor); axis_name='x'; arc_point=y_axis*(size*0.8);
    } else if(current_mode==roty) {
      verse=((axis+y_axis).Norm()<1?-1:1);
      glColor(ycolor); axis_name='y'; arc_point=z_axis*(size*0.8);
    } else if(current_mode==rotz) {
      verse=((axis+z_axis).Norm()<1?-1:1);
      glColor(zcolor); axis_name='z'; arc_point=x_axis*(size*0.8);
    } else assert(0);
    // normalizing rotation between -180 and 180 degrees
    float sign = ((angle*verse)<0) ? -1 : 1;
    float abs_angle = (angle<0) ? -angle : angle;
    angle = sign * ( (abs_angle>M_PI) ? 2*M_PI-abs_angle : abs_angle );
    axis = axis * verse;
    message = QString("rotated %1 deg around %2")
                      .arg(((angle*180.0)/M_PI),5,'f',3)
                      .arg(axis_name);
    Quaternionf arc_rot;
    arc_rot.FromAxis(angle/18.0,axis);
    glBegin(GL_POLYGON);   
      glVertex(position);
      glVertex(position+arc_point);
      for(int i=0;i<18;i++){
      	 arc_point = arc_rot.Rotate(arc_point);
         glVertex(position+arc_point);
      }
    glEnd(); 
  } else if(moving && drawmoves){ // draw a traslation
    Point3f ntra=manipulator->track.tra;
    ntra.Normalize();
    if(current_mode==movx){
      verse=((ntra+x_axis).Norm()<1?-1:1);
      glColor(xcolor); axis_name='x';
    }else if(current_mode==movy){
      verse=((ntra+y_axis).Norm()<1?-1:1);
      glColor(ycolor); axis_name='y';
    }else if(current_mode==movz){
      verse=((ntra+z_axis).Norm()<1?-1:1);
      glColor(zcolor); axis_name='z';
    }else assert(0);
    message = QString("moved %1 units along %2")
                      .arg(verse*manipulator->track.tra.Norm(),5,'f',3)
                      .arg(axis_name);
    Point3f old_pos = position-manipulator->track.tra;
    glLineWidth(2*linewidth);
    glPointSize(4*linewidth);
    glBegin(GL_LINES);
      glVertex(position);
      glVertex(old_pos);
    glEnd();
    glBegin(GL_POINTS);
      glVertex(old_pos);
    glEnd();    
  } else { // got nothing to draw
    glPopAttrib();
    glPopMatrix();
    return;  
  }
  // draw message below cursor
  font.setBold(true);
  font.setPixelSize(12);
  QPoint cursor=glw->mapFromGlobal(glw->cursor().pos());
  glw->renderText(cursor.x()+16,cursor.y()+16,message,font);

  glPopAttrib();
  glPopMatrix();
}

void ActiveCoordinateFrame::Reset(bool reset_position,bool reset_alignment)
{
  MovableCoordinateFrame::Reset(reset_position, reset_alignment);
  Update();
  manipulator->Reset();  
}

void ActiveCoordinateFrame::SetPosition(const Point3f newpos)
{
  MovableCoordinateFrame::SetPosition(newpos);
  Update();
  manipulator->Reset();
}

void ActiveCoordinateFrame::SetRotation(const Quaternionf newrot)
{
  MovableCoordinateFrame::SetRotation(newrot);
  Update();
  manipulator->Reset();
}

void ActiveCoordinateFrame::AlignWith(const Point3f primary,const Point3f secondary,const char c1,const char c2)
{
  MovableCoordinateFrame::AlignWith(primary,secondary,c1,c2);
  Update();
  manipulator->Reset();
}

void ActiveCoordinateFrame::MouseDown(int x, int y, /*Button*/ int button)
{
  Move(manipulator->track);
  manipulator->Reset();
  manipulator->MouseDown(x,y,button);
}

void ActiveCoordinateFrame::MouseMove(int x, int y)
{
  manipulator->MouseMove(x,y);
}

void ActiveCoordinateFrame::MouseUp(int x, int y, /*Button */ int button) 
{
  Move(manipulator->track);
  manipulator->Reset();
  manipulator->MouseUp(x, y, button);
}

void ActiveCoordinateFrame::ButtonUp(int button)
{
  Move(manipulator->track);
  manipulator->Reset();
  manipulator->ButtonUp((Trackball::Button) button);
}

void ActiveCoordinateFrame::ButtonDown(int button)
{
  Move(manipulator->track);
  manipulator->Reset();
  manipulator->ButtonDown((Trackball::Button) button);
}

void ActiveCoordinateFrame::SetSnap(float rot_deg)
{
  assert((rot_deg>=0.0)&&(rot_deg<=180));
  rot_snap_rad=rot_deg*M_PI/180.0;
  Update();
}

void ActiveCoordinateFrame::Move(const Similarityf track)
{
  MovableCoordinateFrame::Move(track);
  Update();
}

void ActiveCoordinateFrame::Update()
{
  movx=(move_button | x_modifier);
  movy=(move_button | y_modifier);
  movz=(move_button | z_modifier);
  rotx=(rotate_button | x_modifier);
  roty=(rotate_button | y_modifier);
  rotz=(rotate_button | z_modifier);

  Point3f p=position;
  Quaternionf r=Inverse(rotation);
  x_axis=r.Rotate(Point3f(1,0,0));
  y_axis=r.Rotate(Point3f(0,1,0));
  z_axis=r.Rotate(Point3f(0,0,1));
  
  manipulator->ClearModes();
  manipulator->modes[0] = NULL;    
  manipulator->modes[movx] = new AxisMode(p,x_axis);
  manipulator->modes[movy] = new AxisMode(p,y_axis);
  manipulator->modes[movz] = new AxisMode(p,z_axis);
  manipulator->modes[rotx] = new CylinderMode(p,x_axis,rot_snap_rad);
  manipulator->modes[roty] = new CylinderMode(p,y_axis,rot_snap_rad);
  manipulator->modes[rotz] = new CylinderMode(p,z_axis,rot_snap_rad);
  manipulator->SetCurrentAction();
}
