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
Revision 1.6  2006/05/21 06:58:55  cignoni
Added ClampMask function

Revision 1.5  2006/01/10 13:20:42  cignoni
Changed ply::PlyMask to io::Mask

Revision 1.4  2004/10/28 00:52:45  cignoni
Better Doxygen documentation

Revision 1.3  2004/05/12 10:19:30  ganovelli
new line added at the end of file

Revision 1.2  2004/03/09 21:26:47  cignoni
cr lf mismatch

Revision 1.1  2004/03/03 15:00:51  cignoni
Initial commit

****************************************************************************/
#ifndef __VCGLIB_IOTRIMESH_IO_MASK
#define __VCGLIB_IOTRIMESH_IO_MASK

namespace vcg {
namespace tri {
namespace io {

/**
@name Input/output data mask
*/
//@{
  
class Mask
{
public:

  /*
  Bitmask for specifying what data has to be loaded or saved or it is present in a given plyfile;
*/

enum {
	IOM_NONE         = 0x00000,

  IOM_VERTCOORD    = 0x00001,
	IOM_VERTFLAGS    = 0x00002,
	IOM_VERTCOLOR    = 0x00004,
	IOM_VERTQUALITY  = 0x00008,
	IOM_VERTNORMAL   = 0x00010,
	IOM_VERTTEXCOORD = 0x00020,
	IOM_VERTRADIUS   = 0x10000,

	IOM_EDGEINDEX    = 0x80000,

	IOM_FACEINDEX    = 0x00040,
	IOM_FACEFLAGS    = 0x00080,
	IOM_FACECOLOR    = 0x00100,
	IOM_FACEQUALITY  = 0x00200,
	IOM_FACENORMAL   = 0x00400,
	
	IOM_WEDGCOLOR    = 0x00800,
	IOM_WEDGTEXCOORD = 0x01000,
	IOM_WEDGTEXMULTI = 0x02000, // when textrue index is explicit
	IOM_WEDGNORMAL   = 0x04000,

	IOM_BITPOLYGONAL = 0x20000, // loads explicit polygonal mesh

	IOM_CAMERA       = 0x08000,

	IOM_FLAGS        = IOM_VERTFLAGS + IOM_FACEFLAGS,

	IOM_ALL          = 0xFFFFF
};
//
//
//static void IOMask2String( int mask, char str[] )
//{
//	str[0] = 0;
//
//	strcat(str,"V:");
//	if( mask & IOM_VERTFLAGS    ) strcat(str,"flag,");
//	if( mask & IOM_VERTCOLOR    ) strcat(str,"color,");
//	if( mask & IOM_VERTQUALITY  ) strcat(str,"quality,");
//	if( mask & IOM_VERTTEXCOORD ) strcat(str,"texcoord,");
//	if( mask & IOM_VERTNORMAL ) strcat(str,"normal,");
//
//	strcat(str," F:");
//	if( mask & IOM_FACEFLAGS    ) strcat(str,"mask,");
//	if( mask & IOM_FACECOLOR    ) strcat(str,"color,");
//	if( mask & IOM_FACEQUALITY  ) strcat(str,"quality,");
//	if( mask & IOM_FACENORMAL   ) strcat(str,"normal,");
//
//	strcat(str," W:");
//	if( mask & IOM_WEDGCOLOR    ) strcat(str,"color,");
//	if( mask & IOM_WEDGTEXCOORD ) strcat(str,"texcoord,");
//	if( mask & IOM_WEDGNORMAL  ) strcat(str,"normal,");
//
//	if( mask & IOM_CAMERA ) strcat(str," camera");
//}
template <class MeshType>
static void ClampMask(MeshType &m, int &mask)
{
  if( (mask & IOM_FACECOLOR)    && !HasPerFaceColor(m) )      mask = mask & (~IOM_FACECOLOR);
  if( (mask & IOM_WEDGTEXCOORD) && !HasPerWedgeTexCoord(m) )  mask = mask & (~IOM_WEDGTEXCOORD);
  if( (mask & IOM_WEDGNORMAL)   && !HasPerWedgeNormal(m) )   mask = mask & (~IOM_WEDGNORMAL);
  if( (mask & IOM_VERTCOLOR)    && !HasPerVertexColor(m) )   mask = mask & (~IOM_VERTCOLOR);
}

}; // end class
//@}

} // end namespace tri
} // end namespace io
} // end namespace vcg
#endif
