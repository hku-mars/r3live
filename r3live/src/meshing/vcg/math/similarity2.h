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
#ifndef __VCG_SIMILARITY2_H
#define __VCG_SIMILARITY2_H
namespace vcg
{

/*
  This class codify a similarity transformation in 2D

  The applied transformation is exactly the same of the Similarity class
  Tra(Sca(Rot(P)))
*/

template <class SCALAR_TYPE>
class Similarity2
{
public:
  Similarity2():rotRad(0),tra(0,0),sca(1)  {}

  SCALAR_TYPE rotRad;
  Point2<SCALAR_TYPE> tra;
  SCALAR_TYPE sca;
};

template <class SCALAR_TYPE>
Point2<SCALAR_TYPE> operator*(const Similarity2<SCALAR_TYPE> &m, const Point2<SCALAR_TYPE> &p) {
  Point2<SCALAR_TYPE> r = p;
  // Apply Rotation to point
  r.Rotate(m.rotRad);
  r *= m.sca;
  r += m.tra;
  return r;
}

typedef Similarity2<float> Similarity2f;
typedef Similarity2<double> Similarity2d;

} // end namespace vcg
#endif // __VCG_SIMILARITY2_H
