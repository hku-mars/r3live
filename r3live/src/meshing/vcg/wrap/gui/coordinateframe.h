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
Revision 1.7  2008/03/14 16:54:34  benedetti
Added doxygen documentation

Revision 1.6  2008/03/02 16:44:18  benedetti
moved ActiveCoordinateFrame to its own files

Revision 1.5  2008/02/22 20:04:02  benedetti
many user interface improvements, cleaned up a little

Revision 1.4  2008/02/17 20:52:53  benedetti
some generalization made

Revision 1.3  2008/02/16 14:12:30  benedetti
first version


****************************************************************************/
#ifndef COORDINATEFRAME_H
#define COORDINATEFRAME_H


#include <vcg/math/similarity.h>
#include <vcg/space/color4.h>

#include <QGLWidget>

namespace vcg {

/*!
  @brief The CoordinateFrame class.

  This class can draw the standard icon for a 3D coordinate frame.
*/
class CoordinateFrame
{
public:
  // functions:
  /*!
    @brief The constructor.

    Initialize the CoordinateFrame data.
    @param size the distance from the origin to the endpoint of the arrows.
  */
  CoordinateFrame(float size);

  /*!
    @brief The destructor.

    The destructor.
  */
  virtual ~CoordinateFrame() {}

  /*!
    @brief Render the coordinate frame.

    @param glw the GL widget.
  */
  virtual void Render(QGLWidget* glw,QPainter* p = NULL);
  // data

  /// The color used for the ticks, the ticks values and the head of the arrows.
  Color4b basecolor;

  /// The color of the X axis and label.
  Color4b xcolor;

  /// The color of the Y axis and label.
  Color4b ycolor;

  /// The color of the Z axis and label.
  Color4b zcolor;

  /// The distance from the origin to the endpoint of the arrows.
  float size;

  /// The width of the lines.
  float linewidth;

  /// The font used for the labels and the ticks values.
  QFont font;

  /// The flag that enables axes rendering.
  bool drawaxis;

  /// The flag that enables lablels rendering.
  bool drawlabels;

  /// The flag that enables ticks values rendering.
  bool drawvalues;

  // useful functions:
  static void drawTickedLine(const Point3d &, const Point3d &, float, float,float);
  static float calcSlope(const Point3d &, const Point3d &, float, int , double *, double *, GLint *);
  static float niceRound(float);
};

/*!
  @brief The MovableCoordinateFrame class.

  This class extends the coordinate frame with the ability of being programmatically rototranslated.
*/
class MovableCoordinateFrame: public CoordinateFrame
{

public:
  /*!
    @brief The constructor.

    Initialize the MovableCoordinateFrame data.
    @param size the distance from the origin to the endpoint of the arrows.
  */
  MovableCoordinateFrame(float);

  /*!
    @brief The destructor.

    The destructor.
  */
  virtual ~MovableCoordinateFrame(){}

  /*!
    @brief Render the movable coordinate frame in its position.

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
    @brief Get the position of the coordinate frame.

    @return the position of the coordinate frame.
  */
  virtual Point3f GetPosition();

  /*!
    @brief Get the rotation of the coordinate frame.

    @return the rotation of the coordinate frame.
  */
  virtual Quaternionf GetRotation();

  /*!
    @brief Computes the transformation matrix relative to the current rototranslation.

    @param m is set to the transformation matrix.
  */
  virtual void GetTransform(Matrix44f &m);

  /*!
    @brief Rotates the coordinate frame wrt itself.

    @param angle the angle of the rotation, in degrees.
    @param axis the axis of the rotation.
  */
  virtual void Rot(float angle,const Point3f axis);

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

protected:
  // data:
  Point3f position;
  Quaternionf rotation;

  // functions: 
  virtual void Move(const Similarityf);
  void RotateToAlign(const Point3f, const Point3f);

};

}//namespace
#endif /*COORDINATEFRAME_H*/
