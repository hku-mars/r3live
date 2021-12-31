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

****************************************************************************/

#ifndef VCG_STRIP_MESH_H
#define VCG_STRIP_MESH_H

#include <vector>

namespace vcg {


class StripMesh {
public:
  enum Signature { NORMAL = 1, COLOR = 2, STRIP = 4 };

  StripMesh(char *s);

private:
  unsigned short _vert_size;
  unsigned short _norm_size;
  unsigned short _color_size;
  unsigned short _strip_size;

  Point3f        *_vert_start;  
  short          *_norm_start;  
  unsigned char  *_color_start;  
  unsigned short *_strip_start;  
};  

class StripMeshBuilder {
public:
  std::vector<Point3f> vert;
  std::vector<short> norm;
  std::vector<unsigned char> color;
  std::vector<unsigned short> strip;

  unsigned int Signature();
  ///required size;
  unsigned int Size(); 
  void Write(char *buffer, unsigned int size);
};

}//namespace
#endif
