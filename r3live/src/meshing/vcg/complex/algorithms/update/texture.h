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


#ifndef __VCG_TRI_UPDATE_TEXTURE
#define __VCG_TRI_UPDATE_TEXTURE

//#include <vcg/space/plane.h>

namespace vcg {
namespace tri {

/// \ingroup trimesh 

/// \headerfile texture.h vcg/complex/algorithms/update/texture.h

/// \brief This class is used to update/generate texcoord position according to various critera. .
template <class ComputeMeshType>
class UpdateTexture
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

static void WedgeTexFromPlane(ComputeMeshType &m, const Point3<ScalarType> &uVec, const Point3<ScalarType> &vVec, bool aspectRatio)
{
  // First just project

	FaceIterator fi;
	for(fi=m.face.begin();fi!=m.face.end();++fi)
	        if(!(*fi).IsD()) 
							{
               for(int i=0;i<3;++i)
               {
                 (*fi).WT(i).U()= (*fi).V(i)->cP() * uVec;
                 (*fi).WT(i).V()= (*fi).V(i)->cP() * vVec;
               }
							}											
  // second Loop normalize to
  Box2f bb;
  for(fi=m.face.begin();fi!=m.face.end();++fi)
          if(!(*fi).IsD())
              {
               for(int i=0;i<3;++i)
               {
                 bb.Add((*fi).WT(i).P());
               }
              }


  ScalarType wideU =  bb.max[0]- bb.min[0];
  ScalarType wideV =  bb.max[1]- bb.min[1];
  if(aspectRatio) {
    wideU = std::max(wideU,wideV);
    wideV = wideU;
  }

  for(fi=m.face.begin();fi!=m.face.end();++fi)
          if(!(*fi).IsD())
              {
               for(int i=0;i<3;++i)
               {
                 (*fi).WT(i).U() = ((*fi).WT(i).U() - bb.min[0]) / wideU;
                 (*fi).WT(i).V() = ((*fi).WT(i).V() - bb.min[1]) / wideV;
               }
              }
}

static void WedgeTexFromCamera(ComputeMeshType &m, Plane3<ScalarType> &pl)
{
	
}

static void WedgeTexFromVertexTex(ComputeMeshType &m)
{
  for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi)
          if(!(*fi).IsD())
              {
               for(int i=0;i<3;++i)
               {
                 (*fi).WT(i).U() = (*fi).V(i)->T().U();
                 (*fi).WT(i).V() = (*fi).V(i)->T().V();
               }
              }
}


/// Currently texture coords are kept for ALL the triangles of a mesh. The texture id is stored with each face. 
/// if a given face should not have tex coord it has the default -1 value for texture ID.
/// This function will add an new fake texture, add that to the list of textures and change all the -1 id to that value.
static void WedgeTexRemoveNull(ComputeMeshType &m, const std::string &texturename)
{
	bool found=false;
	
	FaceIterator fi;
	// first loop lets check that there are -1 indexed textured face
	for(fi=m.face.begin();fi!=m.face.end();++fi)
		if(!(*fi).IsD()) if((*fi).WT(0).N()==-1) found = true;
	
	if(!found) return;
	m.textures.push_back(texturename);
	
	int nullId=m.textures.size()-1;
	
	for(fi=m.face.begin();fi!=m.face.end();++fi)
		if(!(*fi).IsD()) if((*fi).WT(0).N()==-1)		
		{
			(*fi).WT(0).N() = nullId;
			(*fi).WT(1).N() = nullId;
			(*fi).WT(2).N() = nullId;			
		}											
			
}

}; // end class

}	// End namespace
}	// End namespace


#endif
