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
Revision 1.1  2005/07/06 08:02:27  cignoni
Initial commit


****************************************************************************/
#ifndef __VCG_TRI_UPDATE_POSITION
#define __VCG_TRI_UPDATE_POSITION

#include "normal.h"

namespace vcg {
namespace tri {

/// \ingroup trimesh 

/// \headerfile position.h vcg/complex/algorithms/update/position.h

/// \brief This class is used to update vertex position according to a transformation matrix.
template <class ComputeMeshType>
class UpdatePosition
{

public:
typedef ComputeMeshType MeshType; 
typedef typename MeshType::ScalarType     ScalarType;
typedef typename MeshType::VertexType     VertexType;
typedef typename MeshType::VertexPointer  VertexPointer;
typedef typename MeshType::VertexIterator VertexIterator;
typedef typename MeshType::FaceType       FaceType;
typedef typename MeshType::FacePointer    FacePointer;
typedef typename MeshType::FaceIterator   FaceIterator;

/// \brief Multiply 
static void Matrix(ComputeMeshType &m, const Matrix44<ScalarType> &M, bool update_also_normals = true)
{
	VertexIterator vi;
	for(vi=m.vert.begin();vi!=m.vert.end();++vi)
	        if(!(*vi).IsD()) (*vi).P()=M*(*vi).cP();

	if(update_also_normals){
		if(HasPerVertexNormal(m)){
			UpdateNormal<ComputeMeshType>::PerVertexMatrix(m,M);
		}
		if(HasPerFaceNormal(m)){
			UpdateNormal<ComputeMeshType>::PerFaceMatrix(m,M);
		}
	}
}

static void Translate(ComputeMeshType &m, const Point3<ScalarType> &t)
{
  VertexIterator vi;
  for(vi=m.vert.begin();vi!=m.vert.end();++vi)
          if(!(*vi).IsD()) (*vi).P()+=t;
}

static void Scale(ComputeMeshType &m, const ScalarType s)
{
  Scale(m,Point3<ScalarType>(s,s,s));
}

static void Scale(ComputeMeshType &m, const Point3<ScalarType> &s)
{
  VertexIterator vi;
  for(vi=m.vert.begin();vi!=m.vert.end();++vi)
          if(!(*vi).IsD()) {
            (*vi).P()[0]*=s[0];
            (*vi).P()[1]*=s[1];
            (*vi).P()[2]*=s[2];
          }
}

}; // end class

}	// End namespace
}	// End namespace


#endif
