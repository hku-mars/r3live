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


#ifndef VCG_GL_MATH_H
#define VCG_GL_MATH_H

// Please note that this file assume that you have already included your 
// gl-extension wrapping utility, and that therefore all the extension symbol are already defined.

#include <vcg/math/matrix44.h>
#include <vcg/math/similarity.h>
//#include <GL/glew.h> // please do not include it!


namespace vcg {

inline void glMultMatrixE(const Matrix44f &matrix) {

    glMultMatrixf((const GLfloat *)(matrix.transpose().V()));
  
}

inline void glMultMatrix(const Matrix44f &matrix) {
    glMultMatrixf((const GLfloat *)(matrix.transpose().V()));
}

inline void glMultMatrixE(const Matrix44d &matrix) {
 
    glMultMatrixd((const GLdouble *)(matrix.transpose().V()));

}
inline void glMultMatrix(const Matrix44d &matrix) {
    glMultMatrixd((const GLdouble *)(matrix.transpose().V()));
}

inline void glLoadMatrix(const Matrix44d &matrix) {
    glLoadMatrixd((const GLdouble *)(matrix.transpose().V()));
}
inline void glLoadMatrix(const Matrix44f &matrix) {
    glLoadMatrixf((const GLfloat *)(matrix.transpose().V()));
}


inline void glMultMatrixDirect(const Matrix44f &matrix) {
   glMultMatrixf((const GLfloat *)(matrix.V()));
}

inline void glMultMatrixDirect(const Matrix44d &matrix) {
   glMultMatrixd((const GLdouble *)(matrix.V()));
}

inline void glMultMatrix(const Similarityf &s) {
  glTranslatef(s.tra[0], s.tra[1], s.tra[2]);
  glScalef(s.sca, s.sca, s.sca);
  float alpha;
  Point3f axis;
  s.rot.ToAxis(alpha, axis);    
  glRotatef(math::ToDeg(alpha), axis[0], axis[1], axis[2]);    
  
}

inline void glMultMatrix(const Similarityd &s) {
  glTranslated(s.tra[0], s.tra[1], s.tra[2]);
  double alpha;
  Point3d axis;
  s.rot.ToAxis(alpha, axis);
  glRotated(math::ToDeg(alpha), axis[0], axis[1], axis[2]);
  glScaled(s.sca, s.sca, s.sca);
}

inline void glGetv(const GLenum  pname, Matrix44f  & m){
	Matrix44f tmp;
	glGetFloatv(pname,tmp.V());
  m = tmp.transpose();
}

inline void glGetv(const GLenum  pname, Matrix44d  & m){
  Matrix44d tmp;
	glGetDoublev(pname,tmp.V());
  m = tmp.transpose();
}

inline void glGetDirectv(const GLenum  pname, Matrix44f  & m){
	glGetFloatv(pname,m.V());
}

inline void glGetDirecv(const GLenum  pname, Matrix44d  & m){
	glGetDoublev(pname,m.V());
}


}//namespace
#endif
