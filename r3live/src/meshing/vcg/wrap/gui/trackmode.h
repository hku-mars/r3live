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
Revision 1.15  2007/07/14 12:43:44  benedetti
Added Doxygen documentation.

Revision 1.14  2007/07/09 22:47:18  benedetti
Removed using namespace std and modified accordingly.

Revision 1.13  2007/06/25 10:21:38  fiorin
Added some std:: here and there

Revision 1.12  2007/06/13 17:15:09  benedetti
Added one-level undo system and sticky trackmodes.

Revision 1.11  2007/05/15 14:59:10  benedetti
Main restructuring. added many new modes

Revision 1.10  2007/02/26 01:30:02  cignoni
Added reflection Name

Revision 1.9  2006/02/13 13:10:27  cignoni
Added Zmode for moving objects along the perpendicular to the viewplane

Revision 1.8  2004/07/18 06:54:08  cignoni
Added Scaling

Revision 1.7  2004/07/11 22:06:56  cignoni
Added scaling by wheel

Revision 1.6  2004/06/09 14:01:13  cignoni
Heavily restructured. To be completed only rotation works...

Revision 1.5  2004/05/14 03:15:09  ponchio
Redesigned partial version.

Revision 1.4  2004/05/07 12:46:08  cignoni
Restructured and adapted in a better way to opengl

Revision 1.3  2004/04/07 10:54:11  cignoni
Commented out unused parameter names and other minor warning related issues

Revision 1.2  2004/03/25 14:55:25  ponchio
Adding copyright.


****************************************************************************/

#ifndef TRACKMODE_H
#define TRACKMODE_H

#include <vcg/space/line3.h>
#include <vcg/space/plane3.h>
#include <vcg/space/segment3.h>
#include <vcg/space/ray3.h>
#include <wrap/gui/view.h>

namespace vcg {

class Trackball;

/*! 
  @brief Base class for all the manipulators.

  Functions in this class implements the default
  behaviour of a manipulator: <em>doing nothing</em>.

  Every manipulator must be subclass of this class.
*/
class TrackMode {
public:
  /*!
    @brief The default virtual destructor
  */
  virtual ~TrackMode () {
  }
  /*!
    @brief The default manipulator application for mouse drags.

    This default application does nothing.

    @param trackball the manipulator manager.
    @param new_point the new mouse pointer coordinate.
  */
  virtual void Apply (Trackball * trackball, Point3f new_point);
  /*!
    @brief The default manipulator application for mouse scrolls.

    This default application does nothing.

    @param trackball the manipulator manager.
    @param WheelNotch the mouse wheel notch.
  */
  virtual void Apply (Trackball * trackball, float WheelNotch);
  /*!
    @brief The default manipulator's begin action function.

    This default implementation does nothing.
  */
  virtual void SetAction ();
  /*!
    @brief The default manipulator's reset function.

    If a manipulator has a state, it can be reset to the inital state calling this function.
  */
  virtual void Reset ();
  /*!
    @brief The default manipulator's name.

    @return the constant string "TrackMode"
  */
  virtual const char *Name (){
    return "TrackMode";
  };
  /*!
    @brief The default manipulator's render function.

    @param trackball the manipulator manager.
   */
  virtual void Draw (Trackball * trackball);
  /*!
    @brief The default avaibility to manipulator changes inside an action.

    Every manipulator class can choose if the manipulator manager can switch
    between it and another manipulator in the middle of an user action,
    e.g. switching Trackball's current_mode without releasing the mouse button.

    The default behaviour is to allow the switch.

    Blocking switches is useful for stateful manipulators, regarding state
    consistency respect to Trackball's %Undo() calls.
    @return false if manipulator permits the switch.
  */
  virtual bool isSticky();
  /*!
    @brief The default manipulator's undo function.

    If a manipulator has a state, it must be undoable with a call of this function.
    The undo must recreate the state present before the last Apply() call.

    This default implementation does nothing.
  */
  virtual void Undo();

	virtual bool IsAnimating(const Trackball *tb);
	virtual void Animate(unsigned int msec, Trackball *tb);
}; 

/*!
  @brief An inactive manipulator.

  This manipulator is useful only for drawing the inactive trackball
  and for feeding occasional Trackball's modes with inactive manipulators.
*/
class InactiveMode:public TrackMode {
public:
  /*!
    @brief Return this manipulator's name.

    @return the constant string "InactiveMode"
  */
  const char *Name () {
    return "InactiveMode";
  };
 /*!
    @brief Render this manipulator.

    @param trackball the manipulator manager.
 */
 void Draw (Trackball * trackball);
};

/* View space modes */

// old interfaces
/*
class SphereMode: public TrackMode {
}
class CylinderMode: public TrackMode {
}
class PlaneMode: public TrackMode {
}
class ZMode: public TrackMode {
}
class LineMode: public TrackMode {
}
class LineMode: public TrackMode {
}
class ScaleMode: public TrackMode {

*/

// Sphere mode.
/*!
  @brief The classic \e arcball manipulator.

  This class implements the classic free rotation manipulator,
  called \e arcball or \e trackball.

  This is a stateless manipulator, result of the Apply function is
  determined only by the mouse coordinates.
*/
class SphereMode:public TrackMode {
public:
  /*!
    @brief Apply a rotation, function of the user mouse drag action.

    Map a mouse drag action on a rotation, and apply the rotation to the manipulated objects.

    If the user does not hit the sphere that surrounds the manipulated object(s),
    a rotational hyperboloid is used to compute the rotation.
    @param trackball the manipulator manager.
    @param new_point the new mouse pointer coordinate.
  */
  void Apply (Trackball * trackball, Point3f new_point);
  /*!
    @brief Return this manipulator's name.

    @return the constant string "SphereMode"
  */
  const char *Name () {
    return "SphereMode";
  };
 /*!
    @brief Render this manipulator.

    @param trackball the manipulator manager.
 */
  void Draw (Trackball * trackball);
};

// Panning mode.
/*!
  @brief The panning manipulator.

  This manipulator implements a bidimensional translation
  on the view plane.

  This is a stateless manipulator, result of the Apply function is
  determined only by the mouse coordinates.
*/
class PanMode:public TrackMode {
public:
  /*!
    @brief Apply a translation, function of the user mouse drag action.

    The manipulated object is dragged in the plane parallel to the screen.

    @param trackball the manipulator manager.
    @param new_point the new mouse pointer coordinate.
  */
  void Apply (Trackball * trackball, Point3f new_point);
  /*!
    @brief Return this manipulator's name.

    @return the constant string "PanMode"
  */
  const char *Name () {
    return "PanMode";
  };
  /*!
    @brief Render this manipulator.

    @param trackball the manipulator manager.
 */
 void Draw (Trackball * trackball);
};

// Z mode.
/*!
  @brief The Z-directional manipulator.

  This manipulator implements a monodimensional translation
  on the axis normal to the view plane.

  Dragging the mouse up and down or scrolling the 
  mouse wheel will move the object along the Z of the camera.

  This is a stateless manipulator, result of the Apply functions is
  determined only either by the mouse coordinates or by the mouse wheel notch.
*/
class ZMode:public TrackMode {
public:
  /*!
    @brief Return this manipulator's name.

    @return the constant string "ZMode"
  */
  const char *Name () {
    return "ZMode";
  };
  /*!
    @brief Apply a translation, function of the user mouse drag action.

    The manipulated object is moved along the Z of the camera:
    - Dragging the mouse down will move the object nearer to the camera.
    - Dragging the mouse up will move the object farther from the camera.

    @param trackball the manipulator manager.
    @param new_point the new mouse pointer coordinate.
  */
  void Apply (Trackball * trackball, Point3f new_point);
  /*!
    @brief Apply a translation, function of the user mouse wheel action.

    The manipulated object is moved along the Z of the camera:
    - Scrolling the mouse wheel down will move the object nearer to the camera.
    - Scrolling the mouse wheel up will move the object farther from the camera.

    @param trackball the manipulator manager.
    @param WheelNotch the mouse wheel notch.
  */
  void Apply (Trackball * trackball, float WheelNotch);
  /*!
    @brief Render this manipulator.

    @param trackball the manipulator manager.
  */
  void Draw (Trackball * trackball);
};

// Scale Mode.
/*!
  @brief The scale manipulator.

  This manipulator implements a scaling transformation.

  Dragging the mouse up and down or scrolling the 
  mouse wheel will scale the object.

  This is a stateless manipulator, result of the Apply functions is
  determined only either by the mouse coordinates or by the mouse wheel notch.
*/
class ScaleMode:public TrackMode {
public:
  /*!
    @brief Return this manipulator's name.

    @return the constant string "ScaleMode"
  */
  const char *Name () {
    return "ScaleMode";
  };
  /*!
    @brief Apply a scaling, function of the user mouse drag action.

    The manipulated object is scaled in this way:
    - Dragging the mouse up will scale the object to a smaller dimension.
    - Dragging the mouse down will scale the object to a greater dimension.

    @param trackball the manipulator manager.
    @param new_point the new mouse pointer coordinate.
  */
  void Apply (Trackball * trackball, Point3f new_point);
  /*!
    @brief Apply a scaling, function of the user mouse wheel action.

    The manipulated object is scaled in this way:
    - Scrolling the mouse wheel up will scale the object to a smaller dimension.
    - Scrolling the mouse wheel down will scale the object to a greater dimension.

    @param trackball the manipulator manager.
    @param WheelNotch the mouse wheel notch.
  */
  void Apply (Trackball * trackball, float WheelNotch);
  /*!
    @brief Render this manipulator.

    @param trackball the manipulator manager.
  */
  void Draw (Trackball * trackball);
};

// Axis mode.
/*!
  @brief The one-directional manipulator.

  This manipulator implements a monodimensional translation
  on a constrained direction.

  Dragging the mouse up and down or scrolling the 
  mouse wheel will move the object along the direction.

  This is a stateless manipulator, result of the Apply functions is
  determined only either by the mouse coordinates or by the mouse wheel notch.
*/
class AxisMode:public TrackMode {
public:
  /*!
    @brief The line constructor.

    This manipulator needs to be initialized with a direction.
    This constructor can initialize it with a Line3f.

    The line will be normalized.
    @param ln the line that represent the direction.
  */
  AxisMode (const Line3f & ln)
    : axis (ln) {
  }
  /*!
    @brief The origin-direction constructor.

    This manipulator needs to be initialized with a direction.
    This constructor can initialize it with two Point3f,
    representing a point and a vector.

    @param origin a point on the line.
    @param direction the line direction.
  */
  AxisMode (const Point3f & origin, const Point3f & direction)
    : axis(Line3fN (origin, direction)) {
  }
  /*!
    @brief Return this manipulator's name.

    @return the constant string "AxisMode"
  */
  const char *Name () {
    return "AxisMode";
  };
  /*!
    @brief Apply a translation, function of the user mouse drag action.

    The manipulated object is moved along the direction.

    If the pointer ray is divergent from the direction the
    object is not moved.

    @param trackball the manipulator manager.
    @param new_point the new mouse pointer coordinate.
  */
  void Apply (Trackball * trackball, Point3f new_point);
  /*!
    @brief Apply a translation, function of the user mouse wheel action.

    The manipulated object is moved along the direction.

    @param trackball the manipulator manager.
    @param WheelNotch the mouse wheel notch.
  */
  void Apply (Trackball * trackball, float WheelNotch);
  /*!
    @brief Render this manipulator.

    @param trackball the manipulator manager.
  */
  void Draw (Trackball * trackball);
private:
  /// The direction, stored as a normalized line.
  Line3fN axis;
};

// Plane mode.
/*!
  @brief The planar manipulator.

  This manipulator implements a bidimensional translation
  on a constrained plane.

  This is a stateless manipulator, result of the Apply function is
  determined only by the mouse coordinates.
*/
class PlaneMode:public TrackMode {
public:
  /*!
    @brief The plane costants constructor.

    This manipulator needs to be initialized with a plane.
    This constructor can initialize it with the four coefficients
    of the plane equation \f$ ax + by + cz + d = 0 \f$.

    @param a the first coefficient of the plane equation.
    @param b the second coefficient of the plane equation.
    @param c the third coefficient of the plane equation.
    @param d the fourth coefficient of the plane equation.
  */
  PlaneMode (float a, float b, float c, float d)
    : plane(Plane3f(d,Point3f(a,b,c))){
  }
  /*!
    @brief The plane constructor.

    This manipulator needs to be initialized with a plane.
    This constructor can initialize it with a Plane3f.

    @param pl the plane.
  */
  PlaneMode (Plane3f & pl)
    : plane(pl) {
  }
  /*!
    @brief Return this manipulator's name.

    @return the constant string "PlaneMode"
 */
 const char *Name () {
    return "PlaneMode";
  };
  /*!
    @brief Apply a translation, function of the user mouse drag action.

    The manipulated object is dragged in the plane.

    If the pointer ray is divergent from the plane the
    object is not moved.

    @param trackball the manipulator manager.
    @param new_point the new mouse pointer coordinate.
  */
  void Apply (Trackball * trackball, Point3f new_point);
  /*!
    @brief Render this manipulator.

    @param trackball the manipulator manager.
  */
  void Draw (Trackball * trackball);
private:
  /// The plane.
  Plane3f plane;
};

// Cylinder mode.
/*!
  @brief The constrained rotation manipulator.

  This manipulator implements a rotation manipulator, that
  make the rotation constrained around a given axis.

  The user can either drag the mouse or scroll the wheel,
  in either cases the rotation's angle is influenced by 
  the radius of the trackball.

  This is a stateless manipulator, result of the Apply functions is
  determined only either by the mouse coordinates or by the mouse wheel notch.
*/
class CylinderMode:public TrackMode {
public:
  /*!
    @brief The line constructor.

    This manipulator needs to be initialized with an axis.
    This constructor can initialize it with a Line3f.

    The line will be normalized.
    @param ln the line that represent the axis.
    @param s a rotational snap angle non negative
  */
  CylinderMode (Line3fN & ln,float s=0.0f)
    : axis (ln), snap(s){
    assert(snap>=0.0);
  }
  /*!
    @brief The origin-direction constructor.

    This manipulator needs to be initialized with a axis.
    This constructor can initialize it with two Point3f,
    representing a point and a vector.

    @param origin a point on the axis.
    @param direction the axis direction.
    @param s a rotational snap angle (non negative)
  */
  CylinderMode (const Point3f & origin, const Point3f & direction,float s=0.0f)
    : axis (Line3fN(origin,direction)), snap(s){
    assert(snap>=0.0);
  }
  /*!
    @brief Return this manipulator's name.

    @return the constant string "CylinderMode"
  */
  const char *Name () {
    return "CylinderMode";
  };
 /*!
    @brief Apply a rotation, function of the user mouse drag action.

    The manipulated object is rotated around the axis.

    if the axis is too perpendicular to view plane, the angle is specified
    only by the vertical component of the mouse drag and the radius.

    @param trackball the manipulator manager.
    @param new_point the new mouse pointer coordinate.
  */
  void Apply (Trackball * trackball, Point3f new_point);
  /*!
    @brief Apply a rotation, function of the user mouse wheel action.

   The manipulated object is rotated around the axis.

    @param trackball the manipulator manager.
    @param WheelNotch the mouse wheel notch.
  */
  void Apply (Trackball * trackball, float WheelNotch);
  /*!
    @brief Render this manipulator.

    @param trackball the manipulator manager.
  */
  void Draw (Trackball * trackball);
private:
  /// The axis, stored as a normalized line.
  Line3fN axis;
  /// The rotational snap value
  float snap;
};

// Path mode.
/*!
  @brief The path constrained manipulator.

  This manipulator moves the object along an eventually closed path.

  The user can either drag the mouse or scroll the wheel,
  when the user drags the mouse, the object tries to slide toward it.

  The object is assumed to initially be on the same position
  of the first point on the path.

  This is a \b stateful manipulator, result of the Apply functions is
  determined by the objects's position along the path and
  by either the mouse coordinates or the mouse wheel notch.
*/
class PathMode:public TrackMode {
public:
/*!
  @brief The vector-boolean constructor.

  The vector passed to build the path is copied locally.
  The boolean value specifies if the path is to be closed.
  If the boolean value is not specified, the path is not closed.

  @warning the path is \b not assumed to have 0-length segments, so, if you want to close the path, please <b>do not</b> add  a copy of the first point on the end of the vector.

  @param pts the path nodes.
  @param w a boolean value that closes the path.
*/
  PathMode ( const std::vector < Point3f > &pts, bool w = false)
    : points(), wrap(w), current_state(0), initial_state(0), old_hitpoint()
  {
    Init(pts);
    assert(min_seg_length > 0.0f);
  }
/*!
  @brief The segment constructor.

  If the path is a simple segment, it can be specified just with the endpoints.

  @param start the starting point.
  @param end the ending point.
*/
  PathMode ( const Point3f &start, const Point3f &end )
    : points(), wrap(false), current_state(0), initial_state(0), old_hitpoint()
  {
    points.push_back(start); 	
    points.push_back(end); 
    path_length=Distance(start,end);
    min_seg_length=path_length;
    assert(min_seg_length > 0.0f);
  }
  /*!
    @brief Return this manipulator's name.

    @return the constant string "PathMode"
  */
  const char *Name () {
    return "PathMode";
  };
  /*!
    @brief Apply a translation, function of the user mouse drag action.

    The manipulated object is moved along the path.
    This function implements an algorithm that makes
    the object try to slide on the path towards the
    mouse pointer.

    @param trackball the manipulator manager.
    @param new_point the new mouse pointer coordinate.
  */
  void Apply (Trackball * trackball, Point3f new_point);
  /*!
    @brief Apply a translation, function of the user mouse wheel action.

    The manipulated object is moved along the path.
    A step of the mouse wheel makes the object slide
    by a distance equal to the half of the shortest
    segment on the path.

    @param trackball the manipulator manager.
    @param WheelNotch the mouse wheel notch.
  */
  void Apply (Trackball * trackball, float WheelNotch);
  /*!
    @brief Render this manipulator.

    @param trackball the manipulator manager.
  */
  void Draw (Trackball * trackball);
  /*!
    @brief The begin action function.

    This function is to be called at the begin of an user action.
  */
  void SetAction ();
  /*!
    @brief The reset function.

    This function reset the object position to the initial point.
  */
  void Reset (); 
  /*!
    @brief Try to set the inital point.

    This function try to set the starting point in the point
    passed as parameter, if the point passed does not reside
    on the path, the start is put on the closest point on it.

    @param p the point wished for the start.
    @return the starting point on the path.
  */
  Point3f SetStartNear(Point3f p);
  /*!
    @brief The (non) avaibility to manipulator changes inside an action.

    This manipulator has an internal state and does not allow a 
    switch in the middle of a function.

    @return the costant boolean true.
  */
  bool isSticky();
  /*!
    @brief The undo function.

    This function recreates the state present before the last Apply() call.
  */
  void Undo();
private:
  /*!
    @brief The data initializer.

    Initialize the internal state and checks params validity.

    @param points the path nodes.
  */
  void Init(const std::vector < Point3f > &points);
  /*!
    @brief The state interpreter.

    Given the state, return the current point, the previous node and
    the next node on the path.
    The algoritm is linear in the node paths.

    @param state the given state.
    @param point is set to the current point.
    @param prev_point is set to the point of current point's previous node.
    @param next_point is set to the point of current point's next node.
  */
  void GetPoints(float state, Point3f & point, Point3f & prev_point, Point3f & next_point);
  /*!
    @brief The state normalizer.

    Normalize a given state in the right interval:
    - \f$ [0 \ldots 1] \f$ if the path is open.
    - \f$ [0 \ldots 1) \f$ if the path is closed (because it wraps).

    @param state the given state.
  */
  float Normalize(float state);
  /*!
    @brief Compute the new point and the \f$\Delta\f$-state.

    Given a state and the mouse coords ray, computes the new 
    state point and return the \f$\Delta\f$-state.
    The algoritm is linear in the node paths.

    @param state the given state.
    @param ray the ray relative to mouse coords.
    @param hit_point is set to the new state point.
    @return the \f$\Delta\f$-state.
  */
  float HitPoint(float state, Ray3fN ray, Point3f &hit_point);
  /*!
    @brief Compute the verse to follow for slide nearer to a given point.

    Given the current state point, the point of previus and
    next node and a reference point, compute the verse to
    follow for the object to come closer to the reference point.

    @param reference_point
    @param current_point
    @param prev_point
    @param next_point
    @return -1, 0 or 1 if the verse is respectively towars the startpoint, null or towards the endpoint.
  */
  int Verse(Point3f reference_point,Point3f current_point,Point3f prev_point,Point3f next_point);

  /// The node vector.
  std::vector < Point3f > points;
  /// True if the path is closed, false otherwise.
  bool wrap;
  /// The current state.
  float current_state;
  /// The initial state.
  float initial_state;
  /// The path length.
  float path_length;
  /// The length of the shostest path segment
  float min_seg_length;
  /// The point relative to the old state.
  Point3f old_hitpoint;
  /// current_state after an Undo() call.
  float undo_current_state;
  /// old_hitpoint after an Undo() call.
  Point3f undo_old_hitpoint;
};

// Area mode.
/*!
  @brief The area constrained manipulator.

  This manipulator moves the object inside a poligonal area.

  The user can drag the object inside a planar area, defined by a polygon.
  The polygon can be non convex, and is specified with a vector of vertexes.

  If the object's trajectory intersects some poligon side, it tries to slide
  around it, in a <em>"rubber band flavoured"</em> way.

  The object is assumed to initially be on the same position of the
  first vertex.

  This is a \b stateful manipulator, result of the Apply function is
  determined by the objects's position inside the area and
  by the mouse coordinates.
*/
class AreaMode:public TrackMode {
public:
/*!
  @brief The constructor.

  From the given vector, is calculated the plane of the polygon, then
  every point in the vector is projected on this plane.

  The vector passed to build the polygon is copied locally.

  @warning The vector is assumed to be formed of \b non collinear points.
  @warning The polygon is \b not assumed to have 0-length sides, so please <b>do not</b> add a copy of the first point on the end of the vector.

  @param pts the vertexes vector.
*/
  AreaMode (const std::vector < Point3f > &pts)
  {
    Init(pts);
    assert(min_side_length > 0.0f);
  }
  /*!
    @brief Return this manipulator's name.

    @return the constant string "AreaMode"
  */
  const char *Name () {
    return "AreaMode";
  };
  /*!
    @brief Apply a translation, function of the user mouse drag action.

    The manipulated object is moved inside the poligon.
    This function implements an algorithm that makes
    the object try to slide around the polygon borders.

    @param trackball the manipulator manager.
    @param new_point the new mouse pointer coordinate.
  */
  void Apply (Trackball * trackball, Point3f new_point);
  /*!
    @brief Render this manipulator.

    @param trackball the manipulator manager.
 */
  void Draw (Trackball * trackball);
  /*!
    @brief The begin action function.

    This function is to be called at the begin of an user action.
  */
  void SetAction ();
  /*!
    @brief The reset function.

    This function reset the object position to the initial point.
  */
  void Reset (); 
  /*!
    @brief Try to set the inital point.

    This function try to set the starting point in the point
    passed as parameter, if the point passed does not reside
    in the area, the start is put in the closest point on it.

    @param p the point wished for the start.
    @return the starting point in the area.
  */
  Point3f SetStartNear(Point3f p);
  /*!
    @brief The (non) avaibility to manipulator changes inside an action.

    This manipulator has an internal state and does not allow a 
    switch in the middle of a function.

    @return The costant boolean true.
  */
  bool isSticky();
  /*!
    @brief The undo function.

    This function recreates the state present before the last Apply() call.
  */
  void Undo();
private:
  /*!
    @brief The data initializer.

    Initialize the internal state and checks params validity.

    @param pts The polygon vertexes.
  */
  void Init(const std::vector < Point3f > &pts);
  /*!
    @brief Point in Polygon test.

    Checks if a given point relies inside the poligon area,
    using the ray tracing algorithm, linear in the number
    of vertexes.

    @param point The point to test.
    @return true if the point is inside the polygon, false otherwise.
  */
  bool Inside(Point3f point);
  /*!
    @brief Try to move the object inside the polygon

    Given a point inside the area and a destination in the
    poligon plane, try to move the point toward the destination,
    sliding on any evenual border of the polygon.
    The object can be imagined tied with a rubber attached to
    destination, so it won't go back from it to slide around a border.

    The algorithm is quadratic in the number of vertexes 
    (worst case, really really unlikely).

    @param start the starting point <b>assumed inside</b>.
    @param end the destination <b>assumed in the plane</b>.
    @return the final move vector.
  */
  Point3f Move(Point3f start,Point3f end);

  /// The vertexes vector.
  std::vector < Point3f > points;
  /// True in time inteval between a call to SetAction () and a call to Apply()
  bool begin_action;
  /// One of the two dimensions used during the point in polygon test.
  int first_coord_kept;
  /// One of the two dimensions used during the point in polygon test.
  int second_coord_kept;
  /// The length of the shortest border
  float min_side_length;
  /// The current status.
  Point3f status;
  /// The screen space differenve between the object and the cursor during an action.
  Point3f delta_mouse;
  /// The old status.
  Point3f old_status;
  /// The initial status.
  Point3f initial_status;
  /// The polygon plane
  Plane3f plane;
  /// The rubberband handle (current destination in Move())
  Point3f rubberband_handle ;
  /// Current action's object trace
  std::vector < Point3f > path;
  /// begin_action after an Undo() call.
  bool undo_begin_action;
  /// status after an Undo() call.
  Point3f undo_status;
  /// delta_mouse after an Undo() call.
  Point3f undo_delta_mouse;
  /// old_status after an Undo() call.
  Point3f undo_old_status;
  /// rubberband_handle after an Undo() call.
  Point3f undo_rubberband_handle;
  /// path endpoint after an Undo() call.
  unsigned int undo_path_index;

};

// Polar mode.
/* WARNING this mode is not compatible with the other rotation modes */

class PolarMode:public TrackMode {
public:
  PolarMode(): alpha(0), beta(0), enda(0), endb(0) {}
   void Apply (Trackball * trackball, Point3f new_point);

  const char *Name () {
    return "PolarMode";
  };
  void SetAction();
  void Reset(); 
 void Draw (Trackball * trackball);
private:
 float alpha, beta; //rotation in y and x axis
 float enda, endb;  //store intermediate values of alpha and beta
};

class NavigatorWasdMode:public TrackMode {
public:
  NavigatorWasdMode();
  
	void Apply (Trackball * trackball, Point3f new_point);

  const char *Name () {
    return "NavigatorWasdMode";
  };
  //void SetAction();
  void Reset(); 
  //void Draw (Trackball * trackball);  
	
	bool isSticky();
	bool IsAnimating(const Trackball *tb);
	void Animate(unsigned int msec, Trackball *tb);
	void SetAction ();
	
	/// specific option setup methods for this mode
	void FlipH(), FlipV(); // flips mouse controls
	
	void SetTopSpeedsAndAcc(float speed_h, float speed_v, float acc=0.0); // (top) speed is in units on sec
                             // Acc is in units on sec^2, if 0 then no-inertia
                             
	void SetStepOnWalk(float width, float height); // optionally, set step-on-walk effects

  void Apply (Trackball * trackball, float WheelNotch);

private:
  float alpha, beta; //rotation in y and x axis
	Point3f current_speed;
	float step_current, step_last, step_x;

	int _flipH, _flipV;
	
	float accX, accY, accZ, dumping, topSpeedH, topSpeedV;
	float step_height, step_length; // height of steps
};

}//namespace 

#endif
