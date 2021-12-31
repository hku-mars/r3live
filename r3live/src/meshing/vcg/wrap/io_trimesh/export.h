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
Revision 1.6  2006/11/04 14:01:00  granzuglia
fixed bug: &(*filename.end()) ---> &(*filename.rbegin())

Revision 1.5  2006/07/09 05:32:15  cignoni
Uncommented obj export. Now obj saving is enabled by default

Revision 1.4  2006/03/29 08:14:20  corsini
Add LoadMask to OFF importer

Revision 1.3  2006/02/16 19:28:36  fmazzant
transfer of Export_3ds.h, Export_obj.h, Io_3ds_obj_material.h from Meshlab to vcg

Revision 1.2  2006/01/13 15:47:42  cignoni
Uniformed return type to the style of Open. Now every export function returns 0 in case of success.

Revision 1.1  2005/11/12 18:34:17  cignoni
Initial Commit


****************************************************************************/

#ifndef __VCGLIB_TRIMESH_GENERIC_EXPORT
#define __VCGLIB_TRIMESH_GENERIC_EXPORT

#include <wrap/io_trimesh/export_ply.h>
#include <wrap/io_trimesh/export_stl.h>
#include <wrap/io_trimesh/export_off.h>
#include <wrap/io_trimesh/export_dxf.h>
#include <wrap/io_trimesh/export_obj.h>

#include <locale>

namespace vcg {
namespace tri {
namespace io {

/** 
This class encapsulate a filter for automatically importing meshes by guessing 
the right filter according to the extension
*/

template <class OpenMeshType>
class Exporter
{
private:
  enum KnownTypes { KT_UNKNOWN, KT_PLY, KT_STL, KT_DXF, KT_OFF, KT_OBJ};
static int &LastType()
{
  static int lastType= KT_UNKNOWN;
return lastType;
}

public:
// simple aux function that returns true if a given file has a given extesnion
static bool FileExtension(std::string filename,  std::string extension)
{
  std::locale loc1 ;
  std::use_facet<std::ctype<char> > ( loc1 ).tolower(&*filename.begin(),&(*filename.rbegin()));
  std::use_facet<std::ctype<char> > ( loc1 ).tolower(&*extension.begin(),&(*extension.rbegin()));
  std::string end=filename.substr(filename.length()-extension.length(),extension.length());
  return end==extension;
}
// Open Mesh
static int Save(OpenMeshType &m, const char *filename, CallBackPos *cb=0)
{
 return Save(m,filename,0,cb);
}

// Open Mesh
static int Save(OpenMeshType &m, const char *filename, const int mask, CallBackPos *cb=0)
{
  int err;
  if(FileExtension(filename,"ply"))
  {
    err = ExporterPLY<OpenMeshType>::Save(m,filename,mask);
    LastType()=KT_PLY;
  }
  else if(FileExtension(filename,"stl"))
  {
    err = ExporterSTL<OpenMeshType>::Save(m,filename);
    LastType()=KT_STL;
  }
  else if(FileExtension(filename,"off"))
  {
    err = ExporterOFF<OpenMeshType>::Save(m,filename,mask);
    LastType()=KT_OFF;
  }
  else if(FileExtension(filename,"dxf"))
  {
    err = ExporterDXF<OpenMeshType>::Save(m,filename);
    LastType()=KT_DXF;
  }
  else if(FileExtension(filename,"obj"))
  {
	  err = ExporterOBJ<OpenMeshType>::Save(m,filename,mask,cb);
	  LastType()=KT_OBJ;
  }
 else {
    err=1;
    LastType()=KT_UNKNOWN;
  }

  return err;
}

static const char *ErrorMsg(int error)
{
  switch(LastType())
  {
    case KT_PLY : return ExporterPLY<OpenMeshType>::ErrorMsg(error); break;
    case KT_STL : return ExporterSTL<OpenMeshType>::ErrorMsg(error); break;
    case KT_OFF : return ExporterOFF<OpenMeshType>::ErrorMsg(error); break;
    case KT_DXF : return ExporterDXF<OpenMeshType>::ErrorMsg(error); break;
  	case KT_OBJ : return ExporterOBJ<OpenMeshType>::ErrorMsg(error); break;
  }
  return "Unknown type";  
}

}; // end class
} // end Namespace tri
} // end Namespace io
} // end Namespace vcg

#endif
