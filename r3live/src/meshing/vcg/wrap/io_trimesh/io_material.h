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
 Revision 1.5  2008/02/27 00:34:43  cignoni
 corrected various bugs in the export of texture coords

 Revision 1.4  2007/06/20 10:28:04  tarini
 "newline at end of file" and  "endif" warnings fixed

 Revision 1.3  2006/11/09 07:51:44  cignoni
 bug due to wrong access to eventually unexistent FaceColor

 Revision 1.2  2006/10/09 19:58:08  cignoni
 Added casts to remove warnings

 Revision 1.1  2006/03/07 13:19:29  cignoni
 First Release with OBJ import support

 Revision 1.1  2006/02/16 19:28:36  fmazzant
 transfer of Export_3ds.h, Export_obj.h, Io_3ds_obj_material.h from Meshlab to vcg

 Revision 1.1  2006/02/06 11:04:40  fmazzant
 added file material.h. it include struct Material, CreateNewMaterial(...) and MaterialsCompare(...)


 ****************************************************************************/

#ifndef __VCGLIB_MATERIAL
#define __VCGLIB_MATERIAL

#include <string>
#include <vector>
#include <vcg/space/point3.h>

namespace vcg {
namespace tri {
namespace io {
	
	/*
		structures material
	*/
	struct Material
	{
		unsigned int index;//index of material
		std::string materialName;

		Point3f Ka;//ambient
		Point3f Kd;//diffuse
		Point3f Ks;//specular
		
		float d;//alpha
		float Tr;//alpha
		
		int illum;//specular illumination
		float Ns;

		std::string map_Kd; //filename texture
	};

	
	template <class SaveMeshType>
	class Materials
	{
	public:	
		typedef typename SaveMeshType::FaceIterator FaceIterator;
		typedef typename SaveMeshType::VertexIterator VertexIterator;
		typedef typename SaveMeshType::VertexType VertexType;
	
		/*
			creates a new meterial
		*/
		inline static int CreateNewMaterial(SaveMeshType &m, std::vector<Material> &materials, unsigned int index, FaceIterator &fi)
		{			
			Point3f diffuse(1,1,1);
      float Transp = 1;
      if(HasPerFaceColor(m)){
        diffuse = Point3f((float)((*fi).C()[0])/255.0f,(float)((*fi).C()[1])/255.0f,(float)((*fi).C()[2])/255.0f);//diffuse
			  Transp = (float)((*fi).C()[3])/255.0f;//alpha
      }
			
			int illum = 2; //default not use Ks!
			float ns = 0.0; //default

			Material mtl;

			mtl.index = index;//index of materials
			mtl.Ka = Point3f(0.2f,0.2f,0.2f);//ambient
			mtl.Kd = diffuse;//diffuse
			mtl.Ks = Point3f(1.0f,1.0f,1.0f);//specular
			mtl.Tr = Transp;//alpha
			mtl.Ns = ns;
			mtl.illum = illum;//illumination
			
			if(m.textures.size() && (*fi).WT(0).n() >=0 ) 
				mtl.map_Kd = m.textures[(*fi).WT(0).n()];
			else
				mtl.map_Kd = "";
			
			int i = -1;
			if((i = MaterialsCompare(materials,mtl)) == -1)
			{
				materials.push_back(mtl);
				return materials.size();
			}
			return i;
		}

		/*
			returns the index of the material if it exists inside the list of the materials, 
			otherwise it returns -1.
		*/
		inline static int MaterialsCompare(std::vector<Material> &materials, Material mtl)
		{
			for(unsigned int i=0;i<materials.size();i++)
			{
				if(materials[i].Kd     != mtl.Kd    ) continue;
				if(materials[i].Ka     != mtl.Ka    ) continue;
				if(materials[i].Ks     != mtl.Ks    ) continue;
				if(materials[i].Tr     != mtl.Tr    ) continue;
				if(materials[i].illum  != mtl.illum ) continue;
				if(materials[i].Ns     != mtl.Ns    ) continue;
				if(materials[i].map_Kd != mtl.map_Kd) continue;
				return i;
			}
			return -1;
		}
	};
}
}
}
#endif //__VCGLIB_MATERIAL
