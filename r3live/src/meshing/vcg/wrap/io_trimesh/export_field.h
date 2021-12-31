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
#ifndef __VCGLIB_EXPORTERFIELD
#define __VCGLIB_EXPORTERFIELD

namespace vcg {
namespace tri {
namespace io {

/** 
This class encapsulate a filter for saving field formats
*/
template <class MeshType>
class ExporterFIELD
{
    typedef typename MeshType::ScalarType ScalarType;
    typedef typename MeshType::FaceType FaceType;
    typedef typename MeshType::VertexType VertexType;
    typedef typename MeshType::CoordType CoordType;

public:

    ///Save a 4 rosy format file as used by
    ///Interactive Visualization of Rotational Symmetry Fields on Surfaces
    ///Jonathan Palacios and Eugene Zhang
    static void Save4ROSY(MeshType &mesh,
                        const char *path)
    {
        FILE *f = fopen(path,"wt");
        fprintf(f,"%d\n",mesh.vn);
        fprintf(f,"4\n");
        for (unsigned int i=0;i<mesh.vert.size();i++)
        {
            float dirX=(float)mesh.vert[i].PD1().X();
            float dirY=(float)mesh.vert[i].PD1().Y();
            float dirZ=(float)mesh.vert[i].PD1().Z();
            fprintf(f,"%f %f %f \n",dirX,dirY,dirZ);

        }
        fclose(f);
    }

    ///Save a 4 rosy format file as used by
    ///Interactive Visualization of Rotational Symmetry Fields on Surfaces
    ///Jonathan Palacios and Eugene Zhang
    static void Save4ROSYFace(MeshType &mesh,
                        const char *path)
    {
        FILE *f = fopen(path,"wt");
        fprintf(f,"%d\n",mesh.vn);
        fprintf(f,"4\n");
        for (unsigned int i=0;i<mesh.face.size();i++)
        {
            float dirX=(float)mesh.face[i].PD1().X();
            float dirY=(float)mesh.face[i].PD1().Y();
            float dirZ=(float)mesh.face[i].PD1().Z();
            fprintf(f,"%f %f %f \n",dirX,dirY,dirZ);

        }
        fclose(f);
    }

}; // end class



} // end namespace tri
} // end namespace io
} // end namespace vcg

#endif

