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
#ifndef __VCGLIB_IMPORTERFIELD
#define __VCGLIB_IMPORTERFIELD
#include <vcg/complex/algorithms/parametrization/tangent_field_operators.h>

namespace vcg {
namespace tri {
namespace io {

/** 
This class encapsulate a filter for opening field formats
*/
template <class MeshType>
class ImporterFIELD
{
    typedef typename MeshType::ScalarType ScalarType;
    typedef typename MeshType::FaceType FaceType;
    typedef typename MeshType::VertexType VertexType;
    typedef typename MeshType::CoordType CoordType;

public:

    static bool LoadGrad(MeshType &mesh,
                         const char *path)
    {
        FILE *f = fopen(path,"rt");
        if (!f)
        {
            return false;
        }
        int numF;
        fscanf(f,"%d\n",&numF);
        assert(numF==mesh.fn);
        char skipstr[200];
        //int readed0;
        for (int i=0;i<9;i++)
            fscanf(f,"%s",&skipstr[0]);

        for (int i=0;i<mesh.fn;i++)
        {
            int i0=-1;
            int i1=-1;
            int i2=-1;
            double u0,v0,u1,v1,u2,v2;
            int readed1=fscanf(f,"%d %d %d %lf %lf %lf %lf %lf %lf",&i0,&i1,&i2,&u0,&v0,&u1,&v1,&u2,&v2);
            assert(readed1==9);
            vcg::Point2<ScalarType> UV[3];
            UV[0]= vcg::Point2<ScalarType>(u0,v0);
            UV[1]= vcg::Point2<ScalarType>(u1,v1);
            UV[2]= vcg::Point2<ScalarType>(u2,v2);
            CoordType dir1;
            CoordType dir2;
            vcg::tri::CrossField<MeshType>::GradientToCross(mesh.face[i],UV[0],UV[1],UV[2],dir1,dir2);
            dir1.Normalize();
            dir2.Normalize();
            mesh.face[i].PD1()=dir1;
            mesh.face[i].PD2()=dir2;
        }
        fclose(f);
        return true;
    }

    ///load a field on the mesh, it could be a vfield file (per vertex)
    ///or an ffield file (per face)
    static bool LoadFFIELD(MeshType &mesh,
        const char *path,
        bool per_vertex=false)
    {

        FILE *f = fopen(path,"rt");
        if (!f)
        {
            return false;
        }
        {
            char word[512]; word[0]=0;
            fscanf(f,"%s",word);
            char c=0;
            if (word[0]=='#') {
                // skip comment line
                while (fscanf(f,"%c",&c)!=EOF) if (c=='\n') break;
            }
            else
            {
                return false;
            }
            int nnv = -1;
            if (fscanf(f,"%d",&nnv)!=1)
            {
                while (fscanf(f,"%c",&c)!=EOF) if (c=='\n') break; // skip
                fscanf(f,"%d",&nnv);
            }
            int targetnum=mesh.fn;
            if (per_vertex)
                targetnum=mesh.vn;
            if (nnv != (int)targetnum)
            {
                //if (errorMsg) sprintf(errorMsg,"Wrong element number. Found: %d. Expected: %d.",nnv,mesh->vn);
                return false;
            }

            if( per_vertex && !HasPerVertexCurvatureDir(mesh)) throw vcg::MissingComponentException("PerVertexCurvatureDir");
            if(!per_vertex && !HasPerFaceCurvatureDir(mesh))   throw vcg::MissingComponentException("PerFaceCurvatureDir");

            while (fscanf(f,"%c",&c)!=EOF) if (c=='\n') break; // skip
            // skip strange string line
            while (fscanf(f,"%c",&c)!=EOF) if (c=='\n') break;
            for (int i=0; i<nnv; i++){
                vcg::Point3<float> u,v;
                float a,b;
                if (fscanf(f,
                    "%f %f %f %f %f %f %f %f",
                    &a,&b,
                    &(v.X()),&(v.Y()),&(v.Z()),
                    &(u.X()),&(u.Y()),&(u.Z())
                    )!=8) {
                        //if (errorMsg) sprintf(errorMsg,"Format error reading vertex n. %d",i);
                        return false;
                }

                u.Normalize();
                v.Normalize();

                if (per_vertex)
                {
                  mesh.vert[i].PD1().Import(u);
                  mesh.vert[i].PD2().Import(v);
                }
                else
                {
                  mesh.face[i].PD1().Import(u);
                  mesh.face[i].PD2().Import(v);
                }
            }
        }
        fclose(f);
        return true;
    }

    ///Load a 4 rosy format file as used by
    ///Interactive Visualization of Rotational Symmetry Fields on Surfaces
    ///Jonathan Palacios and Eugene Zhang
    static void Load4ROSY(MeshType &mesh,
                        const char *path)
    {
         FILE *f = fopen(path,"rt");
        int num,symm;
        fscanf(f,"%d",&num);
        assert(num==mesh.vn);
        fscanf(f,"%d\n",&symm);
        assert(symm==4);
        for (unsigned int i=0;i<num;i++)
        {
            float dirX,dirY,dirZ;
            fscanf(f,"%f %f %f \n",&dirX,&dirY,&dirZ);
            mesh.vert[i].PD1()=CoordType(dirX,dirY,dirZ);
            mesh.vert[i].PD2()=mesh.vert[i].PD1()^mesh.vert[i].N();
            mesh.vert[i].PD1().Normalize();
            mesh.vert[i].PD2().Normalize();
        }
        fclose(f);
    }


    static bool LoadSeamsMMFromOBJ(MeshType &mesh,std::string PathOBJ)
    {
        ///per face per edge of mmatch in the solver
        typename  MeshType::template PerFaceAttributeHandle<vcg::Point3i> Handle_MMatch;
        ///seam per face
        typename  MeshType::template PerFaceAttributeHandle<vcg::Point3<bool> > Handle_Seams;

        bool HasHandleMMatch=vcg::tri::HasPerFaceAttribute(mesh,std::string("MissMatch"));
        if (!HasHandleMMatch)
            Handle_MMatch = vcg::tri::Allocator<MeshType>::template AddPerFaceAttribute<vcg::Point3i>(mesh,std::string("MissMatch"));
        else
            Handle_MMatch = vcg::tri::Allocator<MeshType>::template FindPerFaceAttribute<vcg::Point3i>(mesh,std::string("MissMatch"));

        bool HasHandleSeams=vcg::tri::HasPerFaceAttribute(mesh,std::string("Seams"));
        if (!HasHandleSeams)
            Handle_Seams=vcg::tri::Allocator<MeshType>::template AddPerFaceAttribute<vcg::Point3<bool> >(mesh,std::string("Seams"));
        else
            Handle_Seams=vcg::tri::Allocator<MeshType>::template FindPerFaceAttribute<vcg::Point3<bool> >(mesh,std::string("Seams"));

        FILE *f = fopen(PathOBJ.c_str(),"rt");
        if (!f)
            return false;

        for (unsigned int i=0;i<mesh.face.size();i++)
        {
            for (int j=0;j<3;j++)
            {
                Handle_Seams[i][j]=false;
                Handle_MMatch[i][j]=0;
            }
        }

        while (!feof(f))
        {

            int f_int,v_int,rot;
            int readed=fscanf(f,"sm %d %d %d\n",&f_int,&v_int,&rot);
            ///skip lines
            if (readed==0)
            {
                char buff[200];
                fscanf(f,"%s\n",&buff[0]);
            }
            else ///add the actual seams
            {
                VertexType *v=&mesh.vert[v_int-1];
                FaceType *f0=&mesh.face[f_int-1];
                int e0=-1;
                if (f0->V(0)==v)e0=0;
                if (f0->V(1)==v)e0=1;
                if (f0->V(2)==v)e0=2;
                e0=(e0+2)%3;
                assert(e0!=-1);
                FaceType *f1;
                int e1;
                f1=f0->FFp(e0);
                e1=f0->FFi(e0);
                Handle_Seams[f0][e0]=true;
                Handle_Seams[f1][e1]=true;

                Handle_MMatch[f0][e0]=rot;
                int rot1;
                if (rot==0)rot1=0;
                if (rot==1)rot1=3;
                if (rot==2)rot1=2;
                if (rot==3)rot1=1;
                Handle_MMatch[f1][e1]=rot1;
            }
        }
        //printf("NEED  %d LINES\n",i);
        return true;
    }


}; // end class



} // end namespace tri
} // end namespace io
} // end namespace vcg

#endif

