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
#ifndef VCG_WRAP_QT_GLLABEL_H
#define VCG_WRAP_QT_GLLABEL_H

#include <QMessageBox>
#include <wrap/qt/col_qt_convert.h>
#include <wrap/qt/device_to_logical.h>
#include <wrap/qt/checkGLError.h>
#include <QPainter>
namespace vcg
{
/*
 */
  class glLabel
  {

    public:
    class Mode
    {
    public:
      Mode()
      {
        color=vcg::Color4b(vcg::Color4b::White);
        angle=0;
        rightAlign = false;
        qFont.setStyleStrategy(QFont::NoAntialias);
        qFont.setFamily("Helvetica");
        qFont.setPixelSize(12);
      }

      Mode(QFont &_qFont, vcg::Color4b _color, float _angle,bool _rightAlign)
      {
        qFont=_qFont;
        color=_color;
        angle=_angle;
        rightAlign=_rightAlign;
      }

      float angle;
      bool rightAlign;
      vcg::Color4b color;
      QFont qFont;
    };

  private:
    static void enter2D(QPainter *painter)
    {
      glPushAttrib(GL_ENABLE_BIT|GL_VIEWPORT_BIT);
      glDisable(GL_DEPTH_TEST);
      glMatrixMode(GL_PROJECTION);
      glPushMatrix();
      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
      painter->endNativePainting();
      painter->save();
    }

    static void exit2D(QPainter *painter)
    {
      //checkGLError::qDebug("glLabel");
      painter->restore();
      painter->beginNativePainting();
      glMatrixMode(GL_PROJECTION);
      glPopMatrix();
      glMatrixMode(GL_MODELVIEW);
      glPopMatrix();
      glPopAttrib();
    }

  public:
    enum LabelPosition { TOP_LEFT, TOP_RIGHT, BOTTOM_LEFT, BOTTOM_RIGHT} ;

    static void render2D(QPainter *painter, const LabelPosition pos, const QString &text, const Mode &m=Mode())
    {
      render2D(painter,pos,0,text,m);
    }
    static void render2D(QPainter *painter, const LabelPosition pos, int linePos, const QString &text, const Mode &m=Mode())
    {
      Mode lm = m;
      if(pos == TOP_RIGHT || pos == BOTTOM_RIGHT)
        lm.rightAlign=true;

      GLint view[4];
      glGetIntegerv(GL_VIEWPORT, view);
      QFontMetrics qfm(m.qFont);
      float delta =  qfm.ascent()/2;
      switch(pos)
      {
        case TOP_LEFT     : render2D(painter,vcg::Point2f(delta,         view[3]-3*delta - delta*3*linePos),text,lm); break;
        case TOP_RIGHT    : render2D(painter,vcg::Point2f(view[2]-delta, view[3]-3*delta - delta*3*linePos),text,lm); break;
        case BOTTOM_LEFT  : render2D(painter,vcg::Point2f(delta,         3*delta         + delta*3*linePos),text,lm); break;
        case BOTTOM_RIGHT : render2D(painter,vcg::Point2f(view[2]-delta, 3*delta         + delta*3*linePos),text,lm); break;
      }
    }

    /*

     */
    static void render2D(QPainter *painter, const vcg::Point2f &p, const QString &text, const Mode &m)
    {
      GLint view[4];
      glGetIntegerv(GL_VIEWPORT, view);
      QFontMetrics qfm(m.qFont);
      QRect textBox = qfm.boundingRect(text);
      glLabel::enter2D(painter);
      painter->setRenderHint(QPainter::TextAntialiasing);
      painter->setPen(vcg::ColorConverter::ToQColor(m.color));
      painter->setFont(m.qFont);

      painter->translate(QPointF(p[0],view[3]-p[1]));
      painter->rotate(m.angle);
      QPoint base(0,qfm.ascent()/2);
      if(m.rightAlign)
        base.setX(-textBox.width() -qfm.maxWidth());
      painter->drawText(base,text);
      glLabel::exit2D(painter);
      glViewport(view[0],view[1],view[2],view[3]);
    }

    static void render(QPainter *painter, const vcg::Point3f &p, const QString &text)
    {
      Mode m;
      render(painter,p,text,m);
    }

    static void render(QPainter *painter, const vcg::Point3f &p, const QString &text, Mode &m)
    {
      GLdouble model[16];
      GLdouble proj[16];
      GLint view[4];

      glGetDoublev(GL_MODELVIEW_MATRIX, model);
      glGetDoublev(GL_PROJECTION_MATRIX, proj);
      glGetIntegerv(GL_VIEWPORT, view);
      GLdouble winx,winy,winz;

      gluProject(p[0],p[1],p[2],model,proj,view,&winx,&winy,&winz);

      QFontMetrics qfm(m.qFont);
      QRect textBox = qfm.boundingRect(text);
      glLabel::enter2D(painter);

      painter->setRenderHint(QPainter::TextAntialiasing);
      painter->setPen(vcg::ColorConverter::ToQColor(m.color));
      painter->setFont(m.qFont);
      painter->translate(QPointF(QTDeviceToLogical(painter,winx),
                                 QTDeviceToLogical(painter,view[3]-winy)));
      painter->rotate(m.angle);
      QPoint base(0,qfm.ascent()/2);
      if(m.rightAlign)
        base.setX(-textBox.width() -qfm.maxWidth());
      painter->drawText(base,text);

      glLabel::exit2D(painter);
    }



    static void render(QPainter *painter, const vcg::Point3d &p, const QString &text)
    { render(painter,Point3f::Construct(p),text); }
    static void render(QPainter *painter, const vcg::Point3d &p, const QString &text, Mode &m)
    { render(painter,Point3f::Construct(p),text,m); }

  };
} // end namespace

#endif
