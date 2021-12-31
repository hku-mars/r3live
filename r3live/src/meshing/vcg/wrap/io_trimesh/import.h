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
Revision 1.11  2006/03/29 09:27:07  cignoni
Added managemnt of non critical errors

Revision 1.10  2006/03/29 08:16:31  corsini
Minor change in LoadMask

Revision 1.9  2006/03/27 07:17:49  cignoni
Added generic LoadMask

Revision 1.8  2006/03/07 13:19:29  cignoni
First Release with OBJ import support

Revision 1.7  2006/02/28 14:50:00  corsini
Fix comments

Revision 1.6  2006/02/10 16:14:53  corsini
Fix typo

Revision 1.5  2006/02/10 08:14:32  cignoni
Refactored import. No more duplicated code

Revision 1.4  2006/02/09 16:04:45  corsini
Expose load mask

Revision 1.3  2006/01/11 10:37:45  cignoni
Added use of Callback

Revision 1.2  2005/01/26 22:43:19  cignoni
Add std:: to stl containers

Revision 1.1  2004/11/29 08:12:10  cignoni
Initial Update


****************************************************************************/

#ifndef __VCGLIB_IMPORT
#define __VCGLIB_IMPORT

#include <wrap/io_trimesh/import_obj.h>
#include <wrap/io_trimesh/import_ply.h>
#include <wrap/io_trimesh/import_stl.h>
#include <wrap/io_trimesh/import_off.h>
#include <wrap/io_trimesh/import_vmi.h>

#include <locale>

namespace vcg {
namespace tri {
namespace io {

/**
This class encapsulate a filter for automatically importing meshes by guessing
the right filter according to the extension
*/

template <class OpenMeshType>
class Importer
{
private:
  enum KnownTypes { KT_UNKNOWN, KT_PLY, KT_STL, KT_OFF, KT_OBJ, KT_VMI };
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
  std::use_facet<std::ctype<char> > ( loc1 ).tolower(&*filename.begin(),&*filename.rbegin());
  std::use_facet<std::ctype<char> > ( loc1 ).tolower(&*extension.begin(),&*extension.rbegin());
  std::string end=filename.substr(filename.length()-extension.length(),extension.length());
  return end==extension;
}

// Open Mesh, returns 0 on success.
static int Open(OpenMeshType &m, const char *filename, CallBackPos *cb=0)
{
  int dummymask = 0;
  return Open(m,filename,dummymask,cb);
}

/// Open Mesh and fills the load mask (the load mask must be initialized first); returns 0 on success.
static int Open(OpenMeshType &m, const char *filename, int &loadmask, CallBackPos *cb=0)
{
	int err;
	if(FileExtension(filename,"ply"))
	{
		err = ImporterPLY<OpenMeshType>::Open(m, filename, loadmask, cb);
		LastType()=KT_PLY;
	}
	else if(FileExtension(filename,"stl"))
	{
		err = ImporterSTL<OpenMeshType>::Open(m, filename, loadmask, cb);
		LastType()=KT_STL;
	}
	else if(FileExtension(filename,"off"))
	{
		err = ImporterOFF<OpenMeshType>::Open(m, filename, loadmask, cb);
		LastType()=KT_OFF;
	}
	else if(FileExtension(filename,"obj"))
	{
		err = ImporterOBJ<OpenMeshType>::Open(m, filename, loadmask, cb);
		LastType()=KT_OBJ;
	}
    else if(FileExtension(filename,"vmi"))
    {
        err = ImporterVMI<OpenMeshType>::Open(m, filename, loadmask, cb);
        LastType()=KT_VMI;
    }
  else {
		err=1;
		LastType()=KT_UNKNOWN;
	}

	return err;
}

static bool ErrorCritical(int error)
{
  switch(LastType())
  {
    case KT_PLY : return (error>0); break;
    case KT_STL : return (error>0); break;
    case KT_OFF : return (error>0); break;
    case KT_OBJ : return ImporterOBJ<OpenMeshType>::ErrorCritical(error); break;
  }

  return true;
}

static const char *ErrorMsg(int error)
{
  switch(LastType())
  {
    case KT_PLY : return ImporterPLY<OpenMeshType>::ErrorMsg(error); break;
    case KT_STL : return ImporterSTL<OpenMeshType>::ErrorMsg(error); break;
    case KT_OFF : return ImporterOFF<OpenMeshType>::ErrorMsg(error); break;
    case KT_OBJ : return ImporterOBJ<OpenMeshType>::ErrorMsg(error); break;
    case KT_VMI : return ImporterVMI<OpenMeshType>::ErrorMsg(error); break;
  }
  return "Unknown type";
}

static bool LoadMask(const char * filename, int &mask)
{
	bool err;

	if(FileExtension(filename,"ply"))
	{
		err = ImporterPLY<OpenMeshType>::LoadMask(filename, mask);
		LastType()=KT_PLY;
	}
	else if(FileExtension(filename,"stl"))
	{
		mask = Mask::IOM_VERTCOORD | Mask::IOM_FACEINDEX;
		err = true;
		LastType()=KT_STL;
	}
	else if(FileExtension(filename,"off"))
	{
		mask = Mask::IOM_VERTCOORD | Mask::IOM_FACEINDEX;
		err = ImporterOFF<OpenMeshType>::LoadMask(filename, mask);
		LastType()=KT_OFF;
	}
	else if(FileExtension(filename,"obj"))
	{
		err = ImporterOBJ<OpenMeshType>::LoadMask(filename, mask);
		LastType()=KT_OBJ;
	}
	else
	{
		err = false;
		LastType()=KT_UNKNOWN;
	}

	return err;
}
}; // end class
} // end Namespace tri
} // end Namespace io
} // end Namespace vcg

#endif
