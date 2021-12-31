/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2011                                                \/)\/    *
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

#ifndef __VCGLIB_IMPORT_CTM
#define __VCGLIB_IMPORT_CTM
#include <openctm.h>
#include <wrap/callback.h>
#include <wrap/io_trimesh/io_mask.h>

// lib3ds headers



namespace vcg {
namespace tri {
namespace io {

/** 
This class encapsulate a filter for importing 3ds meshes.
It uses the lib3ds library.
*/
template <class OpenMeshType>
class ImporterCTM
{
public:

typedef typename OpenMeshType::VertexPointer VertexPointer;
typedef typename OpenMeshType::ScalarType ScalarType;
typedef typename OpenMeshType::VertexType VertexType;
typedef typename OpenMeshType::FaceType FaceType;
typedef typename OpenMeshType::VertexIterator VertexIterator;
typedef typename OpenMeshType::FaceIterator FaceIterator;

enum CTMError {
    // Successfull opening
  E_NOERROR,								// 0
    // Opening Errors
  E_CANTOPEN,								// 1
  E_UNESPECTEDEOF,					// 2
  E_ABORTED,								// 3

  E_NO_VERTEX,							// 4
  E_NO_FACE,								// 5
  E_LESS_THAN_3VERTINFACE,	// 6
  E_BAD_VERT_INDEX,					// 7
  E_BAD_TEX_VERT_INDEX 			// 8
};

static const char* ErrorMsg(int error)
{
  static const char* _3ds_error_msg[] =
  {
    "No errors",												// 0
    "Can't open file",									// 1
    "Premature End of file",						// 2
    "File opening aborted",							// 3
    "No vertex field found",						// 4
    "No face field found",							// 5
    "Face with less than 3 vertices",		// 6
    "Bad vertex index in face",					// 7
    "Bad texture index in face"					// 8
  };

  if(error>8 || error<0) return "Unknown error";
  else return _3ds_error_msg[error];
};


static int Open( OpenMeshType &m, const char * filename, int &loadmask, CallBackPos */*cb*/=0)
{
    CTMcontext context;

    // Create a new importer context
    context = ctmNewContext(CTM_IMPORT);
    // Load the OpenCTM file
    ctmLoad(context, filename);
    if(ctmGetError(context) == CTM_NONE)
    {
    // Access the mesh data
    CTMuint vertCount = ctmGetInteger(context, CTM_VERTEX_COUNT);
    const CTMfloat *vertices = ctmGetFloatArray(context, CTM_VERTICES);
    CTMuint triCount = ctmGetInteger(context, CTM_TRIANGLE_COUNT);
    const CTMuint *indices = ctmGetIntegerArray(context, CTM_INDICES);
    // Deal with the mesh (e.g. transcode it to our // internal representation) // ...

    // Extract colors
    m.Clear();
    Allocator<OpenMeshType>::AddVertices(m, vertCount);
    for(unsigned int i=0;i<vertCount;++i)
        m.vert[i].P()=Point3f(vertices[i*3+0],vertices[i*3+1],vertices[i*3+2]);

    CTMenum colorAttrib = ctmGetNamedAttribMap(context,"Color");
    if(colorAttrib != CTM_NONE)
    {
      const CTMfloat *colors = ctmGetFloatArray(context,colorAttrib);
      for(unsigned int i=0;i<vertCount;++i)
          m.vert[i].C()=Color4b(colors[i*4+0]*255,colors[i*4+1]*255,colors[i*4+2]*255,colors[i*4+3]*255);
      loadmask |= Mask::IOM_VERTCOLOR;
    }

    CTMenum qualityAttrib = ctmGetNamedAttribMap(context,"Quality");
    if(qualityAttrib != CTM_NONE)
    {
      const CTMfloat *qualities = ctmGetFloatArray(context,colorAttrib);
      for(unsigned int i=0;i<vertCount;++i)
          m.vert[i].Q()=qualities[i*4+0];
      loadmask |= Mask::IOM_VERTQUALITY;
    }

    if(triCount==1)
    {
      if(indices[0]==0 && indices[1]==0 && indices[2]==0)
        triCount=0;
    }
    Allocator<OpenMeshType>::AddFaces(m, triCount);
    for(unsigned int i=0;i<triCount;++i)
    {
        m.face[i].V(0)=&m.vert[indices[i*3+0]];
        m.face[i].V(1)=&m.vert[indices[i*3+1]];
        m.face[i].V(2)=&m.vert[indices[i*3+2]];
    }
    // Free the context
    ctmFreeContext(context);
    }

    int result = E_NOERROR;
    return result;
} // end of Open


}; // end class
} // end Namespace tri
} // end Namespace io
} // end Namespace vcg

#endif  // ndef __VCGLIB_IMPORT_3DS
