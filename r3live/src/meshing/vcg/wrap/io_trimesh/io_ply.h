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
Revision 1.3  2004/05/12 10:19:30  ganovelli
new line added at the end of file

Revision 1.2  2004/03/09 21:26:47  cignoni
cr lf mismatch

Revision 1.1  2004/03/08 09:21:34  cignoni
Initial commit

Revision 1.1  2004/03/03 15:00:51  cignoni
Initial commit

****************************************************************************/
#ifndef __VCGLIB_IOTRIMESH_IO_PLY
#define __VCGLIB_IOTRIMESH_IO_PLY


/**
@name Load and Save in Ply format
*/
//@{
#include<wrap/callback.h>
#include<wrap/ply/plylib.h>

namespace vcg {
namespace tri {
namespace io {

  
/** Additional data needed or useful for parsing a ply mesh.
This class can be passed to the ImporterPLY::Open() function for 
- retrieving additional per-vertex per-face data
- specifying a callback for long ply parsing
- knowing what data is  contained in a ply file
*/
class PlyInfo
{
public:
  typedef ::vcg::ply::PropDescriptor PropDescriptor ;

  PlyInfo()
  {
    status=0;
    mask=0;
    cb=0;
    vdn=fdn=0;
    VertexData=FaceData=0;
  }
  /// Store the error codes enconutered when parsing a ply
  int status;
  /// It returns a bit mask describing the field preesnt in the ply file
  int mask;  

  /// a Simple callback that can be used for long ply parsing. 
  // it returns the current position, and formats a string with a description of what th efunction is doing (loading vertexes, faces...)
  CallBackPos *cb;

  /// the number of per-vertex descriptor (usually 0)
  int vdn;
  /// The additional vertex descriptor that a user can specify to load additional per-vertex non-standard data stored in a ply
  PropDescriptor *VertexData;
  /// the number of per-face descriptor (usually 0)
  int fdn;
  
  /// The additional vertex descriptor that a user can specify to load additional per-face non-standard data stored in a ply
  PropDescriptor *FaceData;

  /// a string containing the current ply header. Useful for showing it to the user.
  std::string header;

enum Error
{
		// Funzioni superiori
  E_NO_VERTEX       = ply::E_MAXPLYERRORS+1,			// 14
	E_NO_FACE         = ply::E_MAXPLYERRORS+2,				// 15
	E_SHORTFILE       = ply::E_MAXPLYERRORS+3,			// 16
	E_NO_3VERTINFACE  = ply::E_MAXPLYERRORS+4,		// 17
	E_BAD_VERT_INDEX  = ply::E_MAXPLYERRORS+5,		// 18
	E_NO_6TCOORD      = ply::E_MAXPLYERRORS+6,			// 19
	E_DIFFER_COLORS   = ply::E_MAXPLYERRORS+7,	
	E_BAD_VERT_INDEX_EDGE  = ply::E_MAXPLYERRORS+8,		// 18
  E_MAXPLYINFOERRORS= ply::E_MAXPLYERRORS+9// 20
};

}; // end class
} // end namespace tri
} // end namespace io
} // end namespace vcg
#endif
