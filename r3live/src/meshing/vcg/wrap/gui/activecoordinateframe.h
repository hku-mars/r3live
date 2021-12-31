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
#ifndef ACTIVECOORDINATEFRAME_H
#define ACTIVECOORDINATEFRAME_H

#include "coordinateframe.h"
#include <wrap/gui/trackball.h>
#include <QGLWidget>

namespace vcg {

/*!
  @brief The ActiveCoordinateFrame class.

  This class implements an eulerian trackball over a Movable Coordinate Frame.
*/
class ActiveCoordinateFrame: public MovableCoordinateFrame
{
public:
  /*!
    @brief The constructor.

    Initialize the ActiveCoordinateFrame data.
    @param size the distance from the origin to the endpoint of the arrows.
  */
  ActiveCoordinateFrame(float size);

  /*!
    @brief The destructor.

    The destructor.
  */
  virtual ~ActiveCoordinateFrame();

  /*!
    @brief Render the active coordinate frame in its position.

    @param glw the GL widget.
  */
  virtual void Render(QGLWidget* glw);

  /*!
    @brief Reset the position and/or the rotation of the coordinate frame.

    @param reset_position set to true to reset the position.
    @param reset_alignment set to true to reset the rotation.
  */
  virtual void Reset(bool reset_position,bool reset_alignment);
  /*!
    @brief Set the position of the coordinate frame.

    @param new_position the new position of the coordinate frame.
  */
  virtual void SetPosition(const Point3f new_position);

  /*!
    @brief Set the rotation of the coordinate frame.

    @param new_rotation the new rotation of the coordinate frame.
  */
  virtual void SetRotation(const Quaternionf rotation);
  
  /*!
    @brief Align the coordinate frame to one or two directions.

    If the primary direction of alignment is null this function does nothing, otherwise two rotations are performed: the first rotation aligns the axis named axis_1 to the primary direction of alignment, the second rotation never moves axis_1 away from the primary direction.

    If the secondary direction of alignment is not null and is not parallel to the primary direction the axis named axis_2 is rotated as much as possible to be aligned to secondary direction.

    If the secondary direction of alignment is null the axis named axis_2 is rotated as much as possible to be realigned to its old direction, if this is impossible the remaining axis is used.

    @param primary the primary direction of alignment.
    @param secondary the secondary direction of alignment.
    @param axis_1 the name of the axis to align to the primary direction, must be a char choosen from 'X', 'Y' and 'Z'
    @param axis_2 the name of the axis to align to the secondary direction, must be different from axis_1 and must be a char choosen from 'X', 'Y', 'Z' and ' '; if the char is ' ' the axis is choosen automatically.
  */
  virtual void AlignWith(const Point3f primary, const Point3f secondary, const char axis_1, const char axis_2);

  /*!
    @brief Interface function relative to mouse down event in QT.

    @param x the x coordinate of the cursor.
    @param y the y coordinate of the cursor.
    @param button the keyboard modifiers state.
  */
  void MouseDown(int x, int y, int button);

  /*!
    @brief Interface function relative to mouse move event in QT.

    @param x the x coordinate of the cursor.
    @param y the y coordinate of the cursor.
  */
  void MouseMove(int x, int y); 

  /*!
    @brief Interface function relative to mouse up event in QT.

    @param x the x coordinate of the cursor.
    @param y the y coordinate of the cursor.
    @param button the keyboard modifiers state.
  */
  void MouseUp(int x, int y, int button);

  /*!
    @brief Interface function relative to keyboard up event in QT.

    @param button the keyboard modifiers state.
  */
  void ButtonUp(int button);

  /*!
    @brief Interface function relative to keyboard down event in QT.

    @param button the keyboard modifiers state.
  */
  void ButtonDown(int button);
  
  /*!
    @brief Set rotational snap value.
  
    @param value the new rotational snap value, in degrees.
  */
  void SetSnap(float value);

  /// The eulerian trackball.
  Trackball *manipulator;
  
  /// The flag that enables moves feedback rendering
  bool drawmoves;
  
  /// The flag that enables rotations feedback rendering
  bool drawrotations;
protected:
  // data:
  const int move_button,rotate_button;
  const int x_modifier,y_modifier,z_modifier;
  Point3f x_axis,y_axis,z_axis;
  float rot_snap_rad,mov_snap;
  // functions: 
  virtual void Move(const Similarityf);
  void Update();
private:
  int movx,movy,movz,rotx,roty,rotz;
};

}//namespace
#endif /*ACTIVECOORDINATEFRAME_H*/
