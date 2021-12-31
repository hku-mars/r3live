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
Revision 1.10  2006/02/13 13:05:05  cignoni
Removed glew inclusion

Revision 1.9  2004/09/30 00:48:07  ponchio
<gl/glew.h> -> <GL/glew.h>

Revision 1.8  2004/09/28 14:04:36  ganovelli
glGet added

Revision 1.7  2004/07/13 15:55:57  cignoni
Added test on presence of glTranspose extension (for old hw support)

Revision 1.6  2004/05/26 15:12:39  cignoni
Removed inclusion of gl extension stuff

Revision 1.5  2004/05/12 20:54:55  ponchio
*** empty log message ***

Revision 1.4  2004/05/12 13:07:47  ponchio
Added #include <glew.h>

Revision 1.3  2004/05/04 23:36:23  cignoni
remove include of gl and added glextgension exploiting,

Revision 1.2  2004/04/07 10:47:03  cignoni
inlined functions for avoid multiple linking errors

Revision 1.1  2004/03/31 15:27:17  ponchio
*** empty log message ***


****************************************************************************/

#ifndef VCG_USE_EIGEN
#include "deprecated_math.h"
#else

#ifndef VCG_GL_MATH_H
#define VCG_GL_MATH_H

// Please note that this file assume that you have already included your
// gl-extension wrapping utility, and that therefore all the extension symbol are already defined.

#include <vcg/math/matrix44.h>
#include <vcg/math/similarity.h>
//#include <GL/glew.h> // please do not include it!


namespace vcg {

template<typename T,int StorageOrder>
inline void glLoadMatrix(const Eigen::Matrix<T,4,4,StorageOrder>& matrix) { assert(0); }

template<> inline void glLoadMatrix(const Eigen::Matrix<float,4,4>& matrix) { glLoadMatrixf(matrix.data()); }
template<> inline void glLoadMatrix(const Eigen::Matrix<float,4,4,Eigen::RowMajor>& matrix) {
	Eigen::Matrix4f tmp(matrix);
	glLoadMatrixf(tmp.data());
}
inline void glLoadMatrix(const Eigen::Matrix<double,4,4>& matrix) { glLoadMatrixd(matrix.data()); }
inline void glLoadMatrix(const Eigen::Matrix<double,4,4,Eigen::RowMajor>& matrix) {
	Eigen::Matrix4d tmp(matrix);
	glLoadMatrixd(tmp.data());
}
template<typename Scalar>
inline void glLoadMatrix(const Eigen::Transform<Scalar,3>& t) { glLoadMatrix(t.matrix()); }


template<typename T,int StorageOrder>
inline void glMultMatrix(const Eigen::Matrix<T,4,4,StorageOrder>& matrix) { assert(0); }

template<> inline void glMultMatrix(const Eigen::Matrix<float,4,4>& matrix) { glMultMatrixf(matrix.data()); }
inline void glMultMatrix(const Eigen::Matrix<float,4,4,Eigen::RowMajor>& matrix) {
	Eigen::Matrix<float,4,4> tmp(matrix);
	glMultMatrixf(tmp.data());
}
template<> inline void glMultMatrix(const Eigen::Matrix<double,4,4>& matrix) { glMultMatrixd(matrix.data()); }
template<> inline void glMultMatrix(const Eigen::Matrix<double,4,4,Eigen::RowMajor>& matrix) {
	Eigen::Matrix<double,4,4> tmp(matrix);
	glMultMatrixd(tmp.data());
}
template<typename Scalar>
inline void glMultMatrix(const Eigen::Transform<Scalar,3>& t) { glMultMatrix(t.matrix()); }


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

inline void glGetv(const GLenum  pname, Eigen::Matrix<float,4,4>& matrix){
	glGetFloatv(pname,matrix.data());
}
inline void glGetv(const GLenum pname, Eigen::Matrix<double,4,4>& matrix){
	glGetDoublev(pname,matrix.data());
}
inline void glGetv(const GLenum pname, Eigen::Matrix<float,4,4,Eigen::RowMajor>& matrix){
	glGetFloatv(pname,matrix.data());
	matrix.transposeInPlace();
}
inline void glGetv(const GLenum pname, Eigen::Matrix<double,4,4,Eigen::RowMajor>& matrix){
	glGetDoublev(pname,matrix.data());
	matrix.transposeInPlace();
}
template<typename Scalar>
inline void glGetv(const GLenum pname, const Eigen::Transform<Scalar,3>& t)
{ glGetv(pname,t.matrix()); }

}//namespace
#endif

#endif
