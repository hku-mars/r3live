/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2004                                               \/)\/     *
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
Revision 1.16  2007/07/14 12:43:44  benedetti
Added Doxygen documentation.

Revision 1.15  2007/06/13 17:15:08  benedetti
Added one-level undo system and sticky trackmodes.

Revision 1.14  2007/05/15 15:00:47  benedetti
Moved the drawing code to trackmodes, some other minor changes

Revision 1.13  2007/02/26 01:30:02  cignoni
Added reflection Name

Revision 1.12  2007/01/15 15:04:15  tarini
added "ToAscii" and "SetFromAscii" methods to load/store current trackball status from/to ascii strings
(intended uses: clipboard operations and comments inside png snapshots!)

Revision 1.11  2006/08/23 15:40:57  marfr960
*** empty log message ***

Revision 1.10  2006/02/13 13:15:52  cignoni
Added Scale and Translate methods.
Added many drawing hints and raised the default num. of steps when drawing circles.
Added MouseDown without coords (for remembering changes of keys modifiers)
Added ZMode to the default modes under Alt+left
Added DrawPostApply (to be completed)

Revision 1.9  2005/10/17 01:29:46  cignoni
Main restructuring. Removed the Draw function and slightly changed the meaning of the trackball itself.
See the notes at the beginning of trackball.h

Revision 1.8  2004/07/11 22:06:56  cignoni
Added scaling by wheel

Revision 1.7  2004/06/09 14:01:13  cignoni
Heavily restructured. To be completed only rotation works...

Revision 1.6  2004/05/14 03:15:09  ponchio
Redesigned partial version.

Revision 1.5  2004/05/12 20:55:18  ponchio
*** empty log message ***

Revision 1.4  2004/05/07 12:46:08  cignoni
Restructured and adapted in a better way to opengl

Revision 1.3  2004/04/07 10:54:10  cignoni
Commented out unused parameter names and other minor warning related issues

Revision 1.2  2004/03/25 14:55:25  ponchio
Adding copyright.


****************************************************************************/
#ifndef TRACKBALL_H
#define TRACKBALL_H

#include <time.h>
#include <vcg/math/similarity.h>
#include <vcg/space/color4.h>
#include <wrap/gui/view.h>
#include <wrap/gui/trackmode.h>
#include <list>
#include <vector>
#include <map>

namespace vcg {
/*!
  @brief The base class for Trackball.

  This class is useful for using a Trackball instance in a scene graph,
  as a sort of interactive transform.
*/
class Transform {
public:
  /*!
    @brief The constructor.

    Initialize:
    - track to the identity transform.
    - center to origin 0,0,0.
    - radius to unit size.
  */
  Transform();
  /// A trackball stores a transformation called 'track' that effectively rototranslate the object.
  Similarityf track;
  /// track position in model space.
  Point3f center; 
  /// size of the widget in model space.
  float radius; 
};

/*!
  @brief Computes the linear interpolation between 2 transforms.

  @param a The first transform.
  @param b The second transform.
  @param t The interpolation value (0: just a, 0.5: middle from a to b, 1: just b).
  @return The linear interpolation.
*/
Transform interpolate(const Transform &a, const Transform &b, float t);

class TrackMode;

/*!
  @brief The manipulator manager system.

  <em>Short usage note:</em>

  - Center specify the center of rotation and scaling of the trackball and usually is set by the program and do not interactively change
  - Radius specify the radius of the interactive ball shaped icon to specify rotation. It is in absolute unit.
  - Like the previous one it is not changed during interaction.

  When you specify a translation with the trackball the trackball center remain \b unchanged, in other words it means that the object move out of the trackball icon. Similarly when you apply a scaling the size of the manipulator icon do not change.

  Typical use:
<pre>
glMatrixMode(GL_PROJECTION);
glLoadIdentity();
gluPerspective(60, float(width())/float(height()), 1, 100);
glMatrixMode(GL_MODELVIEW);
glLoadIdentity();
gluLookAt(0,0,3,   0,0,0,   0,1,0)

trackball.center=Point3f(0, 0, 0);
trackball.radius= 1;

trackball.GetView();
trackball.Apply(true); //false if you want an invisible trackball

float d=1.0f/mesh.bbox.Diag();
glScale(d);
glTranslate(-mesh.bbox.Center());
mesh->Render();
</pre>

  Note on the typical use:
  - Perspective and glulookat are choosed to frame the origin centered 1-radius trackball.
  - The final scale and translate are just to fit a generic mesh to the 1sized origin centered where the trackball stays box.
  - The trackball works also on Orthographic projections \b but that are not centered around origin (just move it back along the Z)
*/
class Trackball: public Transform {
public:

  /// The componibile states of the manipulator system.
  enum Button { BUTTON_NONE   = 0x0000, ///< No button or key pressed.
                BUTTON_LEFT   = 0x0001, ///< Left mouse button pressed.
                BUTTON_MIDDLE = 0x0002, ///< Middle mouse button pressed.
                BUTTON_RIGHT  = 0x0004, ///< Right mouse button pressed.
                WHEEL         = 0x0008, ///< Mouse wheel activated.
                KEY_SHIFT     = 0x0010, ///< Shift key pressed.
                KEY_CTRL      = 0x0020, ///< Ctrl key pressed.
                KEY_ALT       = 0x0040, ///< Alt key pressed.
                HANDLE        = 0x0080, ///< Application-defined state activated.
								MODIFIER_MASK = 0x00FF, ///< (mask to get modifiers only)
								KEY_UP        = 0x0100, ///< Up directional key
								KEY_DOWN      = 0x0200, ///< Down directional key
								KEY_LEFT      = 0x0400, ///< Left directional key
								KEY_RIGHT     = 0x0800, ///< Right directional key
								KEY_PGUP      = 0x1000, ///< PageUp directional key
								KEY_PGDOWN    = 0x2000, ///< PageDown directional key
              };

  /*!
    @brief The constructor.

   Initialize the internal state with default values
   and call setDefaultMapping().
  */
  Trackball();
  /*!
    @brief The destructor.

    @warning The destructor <b>does not</b> deallocate the memory allocated by setDefaultMapping(), because the application can change the modes map. This can lead to small memory leaks, so please explicitally delete any manipulator in the modes map if you are going to repeatly allocate and deallocate Trackball instances.
  */
  ~Trackball();
	
	private:
	// Trackball must not be copied. Use Append (see vcg/complex/trimesh/append.h)
	Trackball operator =(const Trackball &  /*m*/){ assert(0); return *this; }
	public:
  /*!
    @brief Reset the trackball.

    Equivalent to Reset().
  */
  void SetIdentity();
  /*!
    @brief Set the position of the trackball.

    @param c The new position of the trackball.
    @param millisec Currently not in use.
  */
  void SetPosition(const Point3f &c, int millisec = 0);
  /*!
    @brief Currently not in use.

    @param s Currently not in use.
  */
  void SetScale(const float s) {radius=s;};
  /*!
    @brief Currently not in use.

    @param transform Currently not in use.
    @param millisec Currently not in use.
  */
  void SetTransform(const Transform &transform, int millisec = 0);
  /*!
    @brief Apply a translation on the current transformation.

    @param tr The translation vector.
  */
  void Translate(Point3f tr);
  /*!
    @brief Apply a scaling on the current transformation.

    @param f The scale factor.
  */
  void Scale(const float f);

  //operating
  /*!
    @brief Initialize the camera instance.
  */
  void GetView();

  /*!
    @brief Application of the transformation.

    @warning This function does \b not draw anything. You have to call DrawPostApply() after drawing everything.

  */
  void Apply ();

  /*!
    @brief Draw the current manipulator.

    Call the draw function of the current manipulator.
    If no manipulator is selected call the draw function of the manipulator associated to inactive_mode.

    @warning This function assumes that the OpenGL modelview matrix has been initialized with Apply ().
  */
  void DrawPostApply();

  /*!
    @brief Apply the \b inverse of current transformation on the OpenGL modelview matrix.
  */
  void ApplyInverse();
  // DrawIcon() has been moved to trackutils.h
  //void DrawIcon();
  
  // T(c) S R T(t) T(-c) => S R T(S^(-1) R^(-1)(c) + t - c)
  Matrix44f Matrix() const;
  Matrix44f InverseMatrix() const;

  /*!
    @brief Reset the transformation and every mapped manipulator.
  */
  void Reset();

  /*!
    @brief clear the modes map. Taking the right care of not doubledeleting anything.
  */
  void ClearModes();

  // DrawCircle (), DrawPlane(), DrawPlaneHandle() has been moved to trackutils.h
  // the drawing code has been moved to the trackmodes
  // void DrawCircle ();
  // void DrawPlane();
  // void DrawPlaneHandle();

  //interface
  /*!
    @brief Interface function relative to mouse down event in QT/SDL.

    @param button The new state.
  */
  void MouseDown(/*Button*/ int button);
  /*!
    @brief Interface function relative to mouse down event in QT/SDL.

    @param x The horizontal coordinate of the mouse pointer.
    @param y The vertical coordinate of the mouse pointer.
    @param button The new state.
  */
  void MouseDown(int x, int y, /*Button*/ int button);
  /*!
    @brief Interface function relative to mouse down event in QT/SDL.

    @param x The horizontal coordinate of the mouse pointer.
    @param y The vertical coordinate of the mouse pointer.
  */
  void MouseMove(int x, int y); 
  /*!
    @brief Interface function relative to mouse down event in QT/SDL.

    @param x The horizontal coordinate of the mouse pointer.
    @param y The vertical coordinate of the mouse pointer.
    @param button The new state.
  */
  void MouseUp(int x, int y, /*Button */ int button); 
  /*!
    @brief Old interface function relative to mouse down event in QT/SDL.

    @param notch The mouse wheel notch (1: one forward step, -1: one backward step).
  */
  void MouseWheel(float notch);
  /*!
    @brief Interface function relative to mouse down event in QT/SDL.

    @param notch The mouse wheel notch (1: one forward step, -1: one backward step).
    @param button The new state.
  */
  void MouseWheel (float notch, /*Button */ int button);
  /*!
    @brief Interface function relative to key down event in QT/SDL.

    @param button the new state.
  */
  void ButtonUp(Button button);
  /*!
    @brief Interface function relative to key up event in QT/SDL.

    @param button the new state.
  */
  void ButtonDown(Button button, unsigned int msec=0);
  /*!
    @brief Undo function for manipulator system.

    A call of this function restores the state before last user action.
    This function calls %Undo() on every mapped manipulator.
  */
  void Undo();

  //default sensitivity 1
  /*!
    @brief Currently not in use.

    @param s Currently not in use.
  */
  void SetSensitivity(float s);

  void SetSpinnable(bool on);

	
	// returns if it is animating or not
	// 
	bool IsAnimating(unsigned int msec=0);

	// Animate: either takes an absolute time (if default not specified, then it is automeasured)
	// or a fixed delta
	void Animate(unsigned int msec=0);

  /*!
    @brief Currently not in use.

    @return A meaningless boolean value.
  */
	bool IsSpinnable();
  /*!
    @brief Currently not in use.

    @param spin Currently not in use.
  */
  void SetSpinning(Quaternionf &spin);
  /*!
    @brief Currently not in use.
  */
  void StopSpinning();
  /*!
    @brief Currently not in use.

    @return A meaningless boolean value.
  */
  bool IsSpinning();

  //interfaccia navigation:
  /*!
    @brief Currently not in use.
  */
  void Back();
  /*!
    @brief Currently not in use.
  */
  void Forward();
  /*!
    @brief Currently not in use.
  */
  void Home();
   /*!
    @brief Currently not in use.
  */
  void Store();
  /*!
    @brief Currently not in use.
  */
  void HistorySize(int lenght);

/*    //internals  // commented out no more used this stuff!
    enum Action { NONE = 0,
		  VIEW_ROTATE = 1,
		  // Axis Constrained Rotation 
		  TRACK_ROTATE_X = 3, TRACK_ROTATE_Y = 4, TRACK_ROTATE_Z = 5,
		  // Drag constrained to an axis (trackball axis)
		  DRAG_X = 6,   DRAG_Y = 7,   DRAG_Z = 8,
		  // Drag constrained to a plane
		  DRAG_XY = 9,  DRAG_YZ = 10,  DRAG_XZ = 11,
		  //scale model respect to center of trackball
		  VIEW_SCALE = 12,
		  //scale trackball and model
		  TRACK_SCALE = 13
    };
*/
  // loads stores current status from/to ascii stings
  /*!
    @brief Stores current status into an ascii stings

    Stores current status into an ascii stings. This is useful for example to implement cut-and-paste operations of trackball status, or to embed used trackball into a comment inside a screenshot, etc.
    @param st The string where to export (must be allocated 256bytes should be enough).
  */
  void ToAscii(char * st);
  /*!
    @brief Loads current status from an ascii stings

    Loads current status from an ascii stings. This is useful for example to implement cut-and-paste operations of trackball status, or to embed used trackball into a comment inside a screenshot, etc.
    @param st The string where to read from (must be allocated). Use ToAscii() method to set it.
    @return True iff the trackball was successfully recovered.
  */
  bool SetFromAscii(const char * st);

  //protected:
  /// The reference for point projection and unprojection from screen space to modelspace.
  View<float> camera;
  /*!
    @brief Prepare Trackball and every mapped TrackMode for an user action.

    This function is called automatically when an user action begins.
  */
  void SetCurrentAction();
	/// Current state composition. Note: mask with MODIFIERS to get modifier buttons only
  int current_button;
  /// The selected manipulator.
  TrackMode *current_mode;

  /// The inactive manipulator. It is drawn when Trackball is inactive.
  TrackMode *inactive_mode;
  
  // The manipulator to deal with timer events and key events
  TrackMode *idle_and_keys_mode; 
  /*!
    @brief Reset modes to default mapping.

    Set the default modes mapping.
    The default mapping is:
    - \b LEFT : SphereMode.
    - \b LEFT+CTRL or \b MIDDLE : PanMode.
    - \b LEFT+SHIFT or \b WHEEL : ScaleMode.
    - \b LEFT+ALT : ZMode.

    @warning The memory allocated by this function <b>is not</b> automatically deallocated. see ~Trackball().
  */
  void setDefaultMapping ();

  /// The manipulator mapping. Needs to be explicitally managed for custom mappings.
  std::map<int, TrackMode *> modes;

  // undo_track and last_track have different meanings..
  /// Transformation before current user action.
  Similarityf last_track;
  /// track after an Undo() call.
  Similarityf undo_track; 
  /// Currently not in use.
  Similarityf last_view;
  /// Mouse cursor coordinates before current action.
  Point3f last_point;
  /// Currently not in use.
  std::vector<Point3f> Hits;
  /// Currently not in use.
  bool dragging;
  /// Currently not in use.
  int button_mask;

	unsigned int last_time;

  /// Currently not in use.
  Quaternionf spin;
  /// Currently not in use.
  bool spinnable;
  /// Currently not in use.
  bool spinning;

  /// Currently not in use.
  std::list<Transform> history;
  /// Currently not in use.
  int history_size;


	void SetFixedTimesteps(bool mode){
		fixedTimestepMode=mode;
	}

  /// Manipulators needs full access to this class.
  friend class TrackMode;
private:
	void Sync(unsigned int msec);
	bool fixedTimestepMode; // if true, animations occurs at fixed time steps

};


}//namespace

#endif
