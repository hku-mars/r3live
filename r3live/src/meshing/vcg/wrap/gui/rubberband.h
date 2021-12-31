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
Revision 1.1  2008/02/16 12:00:34  benedetti
first version, adapted from meshlab's editmeasure plugin


****************************************************************************/
#ifndef RUBBERBAND_H
#define RUBBERBAND_H

#include <vcg/space/color4.h>
#include <QGLWidget>

namespace vcg {

/*!
  @brief The Rubberband class.

  This class is useful for interactively draw a straight line between 2 pickable points in a GL widget.
*/
class Rubberband
{
public:
  //data:

  /// The color of the rubberband
  Color4b color;

  // functions:

  /*!
    @brief The constructor.

    Initialize the rubberband data.
  */
  Rubberband(Color4b);

  /*!
    @brief The destructor.

    The destructor.
  */
  virtual ~Rubberband() {}

  /*!
    @brief Render the rubberband and do the picking.

    Is important that this function is called in order to apply the Drag and Pin commands.

    @param glw the GL widget.
  */
  void Render(QGLWidget* glw);

  /*!
    @brief Set the current rubberband endpoint.

    This function should be called after MouseMove events.

    @param cursor the cursor position.
  */
  void Drag(QPoint cursor);

  /*!
    @brief Ask for picking.

    This function should be called after MouseRelease events.

    The first time is called, if the picking is successful, sets the startpoint.
    The second time sets, if the picking is successful, the endpoint.
    After the second time this has no effect.

    @param cursor the cursor position.
  */
  void Pin(QPoint cursor);

  /*!
    @brief Reset the rubberband.
  */
  void Reset();

  /*!
    @brief Return true if the rubberband has been drawn.

    @return true if the line has been drawn, false otherwise.
  */
  bool IsReady();

  /*!
    @brief Get the rubberband start and end points.

    @param startpoint is set to the rubberband start point.
    @param endpoint  is set to the rubberband end point.
    @warning Don't call this function until IsReady() returns true!
  */
  void GetPoints(Point3f &startpoint,Point3f &endpoint);

  /*!
    @brief Render a text label near the endpoint (if it exists).

    @param text the text to render.
    @param glw the GL widget.
  */
private:
  // types:
  typedef enum { RUBBER_BEGIN = 0,
                 RUBBER_DRAGGING = 1,
                 RUBBER_DRAGGED = 2,
               } RubberPhase;
  // data:
  RubberPhase currentphase;
  QPoint qt_cursor;
  Point3f start, end;
  bool have_to_pick;
//  QFont font;
  // functions:
  vcg::Point2f DevicePixelConvert(const Point3f);

};

}//namespace

#endif /*RUBBERBAND_H*/
