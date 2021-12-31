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

****************************************************************************/

#include <GL/glew.h>
#include <wrap/gl/space.h>
#include <wrap/gl/picking.h>
#include <wrap/qt/device_to_logical.h>

#include "rubberband.h"

using namespace vcg;

Rubberband::Rubberband(Color4b c)
:color(c)
{
  this->Reset();
}

void Rubberband::Render(QGLWidget* gla)
{
  if(have_to_pick){
    assert(currentphase!=RUBBER_DRAGGED);
    Point3f pick_point;
    bool picked = Pick(QTLogicalToDevice(gla, qt_cursor.x()), QTLogicalToDevice(gla, gla->height() - qt_cursor.y()), pick_point);
    if(picked){ // we have not picked the background
      have_to_pick=false;
      switch(currentphase){
        case RUBBER_BEGIN:
          start = pick_point;
          gla->setMouseTracking(true);
          currentphase = RUBBER_DRAGGING;
          break;
        case RUBBER_DRAGGING:
          if(pick_point==start){
              have_to_pick=true;
            break;
          }
          end = pick_point;
          gla->setMouseTracking(false);
          currentphase = RUBBER_DRAGGED;
          break;
        default:
          assert(0);
      }
    }
  }

  if(currentphase==RUBBER_BEGIN) return;

  // Drawing of the current line
  glPushAttrib(GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_LINE_BIT | GL_POINT_BIT | GL_CURRENT_BIT | GL_LIGHTING_BIT | GL_COLOR_BUFFER_BIT);
  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
  glDepthMask(false);
  glLineWidth(2.5);
  glPointSize(5.0);
  if(currentphase==RUBBER_DRAGGING ) {
    Point2f qt_start_point = DevicePixelConvert(start);
    glColor(color);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0,QTLogicalToDevice(gla,gla->width()),QTLogicalToDevice(gla,gla->height()),0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
    glBegin(GL_LINES);
      glVertex(qt_start_point);
      glVertex2f(QTLogicalToDevice(gla, qt_cursor.x()),    QTLogicalToDevice(gla, qt_cursor.y()));
    glEnd();
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
  } else {
    assert(currentphase == RUBBER_DRAGGED);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POINT_SMOOTH);
    glColor(color);
    glBegin(GL_LINES);
      glVertex(start);
      glVertex(end);
    glEnd();
    glBegin(GL_POINTS);
      glVertex(start);
      glVertex(end);
    glEnd();
    glDisable(GL_DEPTH_TEST);
    glLineWidth(0.7);
    glPointSize(1.4);
    glBegin(GL_LINES);
      glVertex(start);
      glVertex(end);
    glEnd();
    glBegin(GL_POINTS);
      glVertex(start);
      glVertex(end);
    glEnd();
  }
  glPopAttrib();
  assert(!glGetError());
}

void Rubberband::Drag(QPoint p)
{
  if(currentphase==RUBBER_DRAGGING)
    qt_cursor=p;
}

void Rubberband::Pin(QPoint p)
{
  if(IsReady())
    return;
  qt_cursor=p;
  have_to_pick=true;
}

void Rubberband::Reset()
{
  currentphase = RUBBER_BEGIN;
  qt_cursor = QPoint();
  start = Point3f(0,0,0);
  end = Point3f(0,0,0);
  have_to_pick = false;
}

bool Rubberband::IsReady()
{
  return currentphase==RUBBER_DRAGGED;
}

void Rubberband::GetPoints(Point3f &s,Point3f &e)
{
  assert(IsReady());
  s=start;
  e=end;
}

Point2f Rubberband::DevicePixelConvert(const Point3f p)
{
  GLint vm[4];
  GLdouble mm[16];
  GLdouble pm[16];
  glGetIntegerv(GL_VIEWPORT, vm);
  glGetDoublev(GL_MODELVIEW_MATRIX, mm);
  glGetDoublev(GL_PROJECTION_MATRIX, pm);
  GLdouble wx,wy,wz;
  gluProject(p[0], p[1], p[2], mm, pm, vm, &wx, &wy, &wz);
  return Point2f(wx,vm[3]-wy);
}
