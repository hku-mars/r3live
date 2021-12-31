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

#ifndef __VCGLIB_EXPORT_VMI
#define __VCGLIB_EXPORT_VMI


/*
    VMI VCG Mesh Image.
    The vmi image file consists of a header containing the description of the vertex and face type,
    the length of vectors containing vertices of faces and the memory image of the object mesh as it is when
    passed to the function Save(SaveMeshType m).
    NOTE: THIS IS NOT A FILE FORMAT. IT IS ONLY USEFUL FOR DUMPING MESH IMAGES FOR DEBUG PURPOSE.
    Example of use: say you are running a time consuming mesh processing and you want to save intermediate
    state, but no file format support all the attributes you need in your vertex/face type.
    NOTE2: At the present if you add members to your TriMesh these will NOT be saved. More precisely, this file and
    import_vmi must be updated to reflect changes in vcg/complex/trimesh/base.h
    */

#include <vcg/complex/complex.h>

namespace vcg {
namespace tri {
namespace io {

    template <int N> struct PlaceHolderType{ char A[N];};


    template <class SaveMeshType>
    class ExporterVMI
    {


        static char * & Out_mem(){static char *    out_mem; return out_mem;}
        static unsigned int & Out_mode(){static unsigned int  out_mode = 0; return out_mode;}


        static unsigned int & pos(){static unsigned int  p = 0; return p;}
        static int fwrite_sim(const void * , size_t size, size_t count){ pos() += size * count;return size * count; }
        static int fwrite_mem(const void *src , size_t size, size_t count ){ memcpy(&Out_mem()[pos()],src,size*count); pos() += size * count;return size * count; }


        static int WriteOut(const void * src,  size_t size, size_t count){
            switch(Out_mode()){
            case 0: return fwrite_sim(src, size,count);  break;
            case 1: return fwrite_mem(src, size,count);  break;
            case 2: return fwrite(src, size,count, F() ); break;
             }
         assert(0);
         return 0;
        }


        static void WriteString( const char * in)		{ unsigned int l = strlen(in); WriteOut(&l,4,1 ); WriteOut(in,1,l );}
        static void WriteInt (const unsigned int i)	{ WriteOut(&i,1,4 );}

        static void WriteFloat( const float v)	{ WriteOut(&v,1,sizeof(float) );}

        /* save Ocf Vertex Components */
        template <typename OpenMeshType,typename CONT>
        struct SaveVertexOcf{
            SaveVertexOcf(  const CONT & /*vert*/, bool only_header){
                // do nothing, it is a std::vector
                if(only_header){
                    WriteString( "NOT_HAS_VERTEX_QUALITY_OCF");
                    WriteString( "NOT_HAS_VERTEX_COLOR_OCF");
                    WriteString( "NOT_HAS_VERTEX_NORMAL_OCF");
                    WriteString( "NOT_HAS_VERTEX_MARK_OCF");
                    WriteString( "NOT_HAS_VERTEX_TEXCOORD_OCF");
                    WriteString( "NOT_HAS_VERTEX_VFADJACENCY_OCF");
                    WriteString( "NOT_HAS_VERTEX_CURVATURE_OCF");
                    WriteString( "NOT_HAS_VERTEX_CURVATUREDIR_OCF");
                    WriteString( "NOT_HAS_VERTEX_RADIUS_OCF");
                }
            }
        };

        /* partial specialization for vector_ocf */
        template <typename MeshType>
        struct SaveVertexOcf<MeshType, vertex::vector_ocf<typename MeshType::VertexType> >{
            typedef typename MeshType::VertexType VertexType;
            SaveVertexOcf(  const vertex::vector_ocf<VertexType> & vert, bool only_header){

                if( VertexType::HasQualityOcf() && vert.IsQualityEnabled()){
                    WriteString( "HAS_VERTEX_QUALITY_OCF");
                    if(!only_header) WriteOut(&vert.QV[0],sizeof(typename VertexType::QualityType),vert.size() );
                }else WriteString( "NOT_HAS_VERTEX_QUALITY_OCF");

                if( VertexType::HasColorOcf() && vert.IsColorEnabled()){
                    WriteString( "HAS_VERTEX_COLOR_OCF");
                    if(!only_header) WriteOut(&vert.CV[0],sizeof(typename VertexType::ColorType),vert.size() );
                }else WriteString( "NOT_HAS_VERTEX_COLOR_OCF");

                if( VertexType::HasNormalOcf() && vert.IsNormalEnabled()){
                    WriteString( "HAS_VERTEX_NORMAL_OCF");
                    if(!only_header) WriteOut(&vert.NV[0],sizeof(typename VertexType::NormalType),vert.size() );
                }else WriteString( "NOT_HAS_VERTEX_NORMAL_OCF");

                if( VertexType::HasMarkOcf() && vert.IsMarkEnabled()){
                    WriteString( "HAS_VERTEX_MARK_OCF");
                    if(!only_header) WriteOut(&vert.MV[0],sizeof(typename VertexType::MarkType),vert.size() );
                }else WriteString( "NOT_HAS_VERTEX_MARK_OCF");

                if( VertexType::HasTexCoordOcf() && vert.IsTexCoordEnabled()){
                    WriteString( "HAS_VERTEX_TEXCOORD_OCF");
                    if(!only_header) WriteOut(&vert.TV[0],sizeof(typename VertexType::TexCoordType),vert.size() );
                }else WriteString( "NOT_HAS_VERTEX_TEXCOORD_OCF");

                if( VertexType::HasVFAdjacencyOcf() && vert.IsVFAdjacencyEnabled()){
                    WriteString( "HAS_VERTEX_VFADJACENCY_OCF");
                    if(!only_header) WriteOut(&vert.AV[0],sizeof(typename vertex::vector_ocf<VertexType>::VFAdjType),vert.size() );
                }else WriteString( "NOT_HAS_VERTEX_VFADJACENCY_OCF");

                if( VertexType::HasCurvatureOcf() && vert.IsCurvatureEnabled()){
                    WriteString( "HAS_VERTEX_CURVATURE_OCF");
                    if(!only_header) WriteOut(&vert.CuV[0],sizeof(typename VertexType::CurvatureType),vert.size() );
                }else WriteString( "NOT_HAS_VERTEX_CURVATURE_OCF");

                if( VertexType::HasCurvatureDirOcf() && vert.IsCurvatureDirEnabled()){
                    WriteString( "HAS_VERTEX_CURVATUREDIR_OCF");
                    if(!only_header) WriteOut(&vert.CuDV[0],sizeof(typename VertexType::CurvatureDirType),vert.size() );
                }else WriteString( "NOT_HAS_VERTEX_CURVATUREDIR_OCF");

                if( VertexType::HasRadiusOcf() && vert.IsRadiusEnabled()){
                    WriteString( "HAS_VERTEX_RADIUS_OCF");
                    if(!only_header) WriteOut(&vert.RadiusV[0],sizeof(typename VertexType::RadiusType),vert.size() );
                }else WriteString( "NOT_HAS_VERTEX_RADIUS_OCF");

            }
        };


        /* save Ocf Face Components */
        template <typename MeshType,typename CONT>
        struct SaveFaceOcf{
            SaveFaceOcf(  const CONT & /*face*/, bool only_header){
                // it is a std::vector
                if(only_header){
                    WriteString( "NOT_HAS_FACE_QUALITY_OCF");
                    WriteString( "NOT_HAS_FACE_COLOR_OCF");
                    WriteString( "NOT_HAS_FACE_NORMAL_OCF");
                    WriteString( "NOT_HAS_FACE_MARK_OCF");
                    WriteString( "NOT_HAS_FACE_WEDGETEXCOORD_OCF");
                    WriteString( "NOT_HAS_FACE_FFADJACENCY_OCF");
                    WriteString( "NOT_HAS_FACE_VFADJACENCY_OCF");
                    WriteString( "NOT_HAS_FACE_WEDGECOLOR_OCF");
                    WriteString( "NOT_HAS_FACE_WEDGENORMAL_OCF");
                }
            }
        };

        /* partial specialization for vector_ocf */
        template <typename MeshType>
        struct SaveFaceOcf<  MeshType, face::vector_ocf<typename MeshType::FaceType> >{
            typedef typename MeshType::FaceType FaceType;
            SaveFaceOcf( const face::vector_ocf<FaceType> & face, bool only_header){

                if( FaceType::HasQualityOcf() && face.IsQualityEnabled()){
                    WriteString( "HAS_FACE_QUALITY_OCF");
                    if(!only_header) WriteOut(&face.QV[0],sizeof(typename FaceType::QualityType),face.size() );
                }else WriteString( "NOT_HAS_FACE_QUALITY_OCF");

                if( FaceType::HasColorOcf() && face.IsColorEnabled()){
                    WriteString( "HAS_FACE_COLOR_OCF");
                    if(!only_header) WriteOut(&face.CV[0],sizeof(typename FaceType::ColorType),face.size() );
                }else WriteString( "NOT_HAS_FACE_COLOR_OCF");

                if( FaceType::HasNormalOcf() && face.IsNormalEnabled()){
                    WriteString( "HAS_FACE_NORMAL_OCF");
                    if(!only_header) WriteOut(&face.NV[0],sizeof(typename FaceType::NormalType),face.size() );
                }else WriteString( "NOT_HAS_FACE_NORMAL_OCF");

                if( FaceType::HasMarkOcf() && face.IsMarkEnabled()){
                    WriteString( "HAS_FACE_MARK_OCF");
                    if(!only_header) WriteOut(&face.MV[0],sizeof(typename FaceType::MarkType),face.size() );
                }else WriteString( "NOT_HAS_FACE_MARK_OCF");

                if( FaceType::HasWedgeTexCoordOcf() && face.IsWedgeTexCoordEnabled()){
                    WriteString( "HAS_FACE_WEDGETEXCOORD_OCF");
                    if(!only_header) WriteOut(&face.WTV[0],sizeof(typename FaceType::WedgeTexCoordType),face.size() );
                }else WriteString( "NOT_HAS_FACE_WEDGETEXCOORD_OCF");

                if( FaceType::HasFFAdjacencyOcf() && face.IsFFAdjacencyEnabled()){
                    WriteString( "HAS_FACE_FFADJACENCY_OCF");
                    if(!only_header) WriteOut(&face.AF[0],sizeof(typename face::vector_ocf<FaceType>::AdjTypePack),face.size() );
                }else WriteString( "NOT_HAS_FACE_FFADJACENCY_OCF");

                if( FaceType::HasVFAdjacencyOcf() && face.IsVFAdjacencyEnabled()){
                    WriteString( "HAS_FACE_VFADJACENCY_OCF");
                    if(!only_header) WriteOut(&face.AV[0],sizeof(typename face::vector_ocf<FaceType>::AdjTypePack),face.size() );
                }else WriteString( "NOT_HAS_FACE_VFADJACENCY_OCF");

                if( FaceType::HasWedgeColorOcf() && face.IsWedgeColorEnabled()){
                    WriteString( "HAS_FACE_WEDGECOLOR_OCF");
                    if(!only_header) WriteOut(&face.WCV[0],sizeof(typename face::vector_ocf<FaceType>::WedgeColorTypePack),face.size() );
                }else WriteString( "NOT_HAS_FACE_WEDGECOLOR_OCF");

                if( FaceType::HasWedgeNormalOcf() && face.IsWedgeNormalEnabled()){
                    WriteString( "HAS_FACE_WEDGENORMAL_OCF");
                    if(!only_header) WriteOut(&face.WNV[0],sizeof(typename face::vector_ocf<FaceType>::WedgeNormalTypePack),face.size() );
                }else WriteString( "NOT_HAS_FACE_WEDGENORMAL_OCF");
            }
        };



        static FILE *& F(){static FILE * f; return f;}

        typedef typename SaveMeshType::FaceContainer FaceContainer;
        typedef typename SaveMeshType::FaceIterator FaceIterator;
        typedef typename SaveMeshType::VertContainer VertContainer;
        typedef typename SaveMeshType::VertexIterator VertexIterator;
        typedef typename SaveMeshType::VertexType VertexType;
        typedef typename SaveMeshType::FaceType FaceType;
    typedef SimpleTempDataBase STDBv;
    typedef SimpleTempDataBase STDBf;
    //	typedef typename SaveMeshType::Attribute <SaveMeshType::FaceContainer> STDBm;

        /* save Ocf Components */


    public:

        static int Save(const SaveMeshType &m,const char * filename){
            Out_mode() = 2;
            F() = fopen(filename,"wb");
            if(F()==NULL)	return 1; // 1 is the error code for cant'open, see the ErrorMsg function
            int res = Serialize(m);
            fclose(F());
            return res;
        }
        static int DumpToMem(const SaveMeshType &m,char * ptr){
            Out_mode() = 1;
            pos() = 0;
            Out_mem() = ptr;
            return Serialize(m);
        }
        static int BufferSize(const SaveMeshType &m){
            Out_mode() = 0;
            pos() = 0 ;
            Serialize(m);
            return pos();
        }


        static int Serialize(const SaveMeshType &m){
            unsigned int i;
            unsigned int vertSize,faceSize;
            std::vector<std::string> nameF,nameV;
            SaveMeshType::FaceType::Name(nameF);
            SaveMeshType::VertexType::Name(nameV);
            vertSize = m.vert.size();
            faceSize = m.face.size();

            /* write header */
            WriteString( "FACE_TYPE");
            WriteInt( nameF.size());

            for(i=0; i < nameF.size(); ++i) WriteString( nameF[i].c_str());
            SaveFaceOcf<SaveMeshType,FaceContainer>( m.face,true);
            WriteString( "SIZE_VECTOR_FACES");
            WriteInt(  faceSize );

            WriteString( "VERTEX_TYPE");
            WriteInt( nameV.size());

            for(i=0; i < nameV.size(); ++i) WriteString( nameV[i].c_str());
            SaveVertexOcf<SaveMeshType,VertContainer>( m.vert,true);

            WriteString( "SIZE_VECTOR_VERTS");
            WriteInt( vertSize);

            WriteString( "BOUNDING_BOX");
            float float_value;
            for(unsigned int i =0; i < 2; ++i){float_value = m.bbox.min[i]; WriteFloat( float_value);}
            for(unsigned int i =0; i < 2; ++i){float_value = m.bbox.max[i]; WriteFloat( float_value);}

            WriteString( "end_header");
            /* end header */

            if(vertSize!=0){
                                void * offsetV =  (void*) &m.vert[0];
                /* write the address of the first vertex */
                                WriteOut(&offsetV,sizeof(void *),1 );
            }

            if(faceSize!=0){
                                 void * offsetF= (void*)&m.face[0];
                /* write the address of the first face */
                                WriteOut(&offsetF,sizeof( void *),1 );
            }

            /* save the object mesh */
            WriteOut(&m.shot,sizeof(Shot<typename SaveMeshType::ScalarType>),1 );
            WriteOut(&m.vn,sizeof(int),1 );
            WriteOut(&m.fn,sizeof(int),1 );
            WriteOut(&m.imark,sizeof(int),1 );
            WriteOut(&m.bbox,sizeof(Box3<typename SaveMeshType::ScalarType>),1 );
            WriteOut(&m.C(),sizeof(Color4b),1 );

            unsigned int written;


            if(vertSize!=0){
                /* save the vertices */
                written = WriteOut((void*)&m.vert[0],sizeof(typename SaveMeshType::VertexType),m.vert.size() );
                SaveVertexOcf<SaveMeshType,VertContainer>( m.vert,false);
            }


            if(faceSize!=0){
                /* save the faces */
                written = WriteOut((void*)&m.face[0],sizeof(typename SaveMeshType::FaceType),faceSize );
                SaveFaceOcf<SaveMeshType,FaceContainer>( m.face,false);

            }



            /* save the attributes */
            typename std::set< typename SaveMeshType::PointerToAttribute>::const_iterator ai;


            /* save the per vertex attributes */
            {
                typename std::set< typename SaveMeshType::PointerToAttribute>::const_iterator ai;
                unsigned int n_named_attr = 0;
                for(ai = m.vert_attr.begin(); ai != m.vert_attr.end(); ++ai) n_named_attr+=!(*ai)._name.empty();

                WriteString( "N_PER_VERTEX_ATTRIBUTES"); WriteInt ( n_named_attr);
                for(ai = m.vert_attr.begin(); ai != m.vert_attr.end(); ++ai)
                    if(!(*ai)._name.empty())
                        {
                            STDBv * stdb = (STDBv *) (*ai)._handle;

                            WriteString( "PER_VERTEX_ATTR_NAME");
                            WriteString( (*ai)._name.c_str() );

                            WriteString( "PER_VERTEX_ATTR_SIZE");
                            WriteInt( stdb->SizeOf());

                            WriteOut(stdb->DataBegin(),m.vert.size(),stdb->SizeOf() );
                        }
            }

            /* save the per face attributes */
            {
                typename std::set< typename SaveMeshType::PointerToAttribute>::const_iterator ai;
                unsigned int n_named_attr = 0;
                for(ai = m.face_attr.begin(); ai != m.face_attr.end(); ++ai) n_named_attr+=!(*ai)._name.empty();

                WriteString( "N_PER_FACE_ATTRIBUTES");
                WriteInt ( n_named_attr);

                for(ai = m.face_attr.begin(); ai != m.face_attr.end(); ++ai)
                    if(!(*ai)._name.empty())
                        {
                            STDBf * stdb = (STDBf *) (*ai)._handle;

                            WriteString( "PER_FACE_ATTR_NAME");
                            WriteString( (*ai)._name.c_str());

                            WriteString( "PER_FACE_ATTR_SIZE");
                            WriteInt( stdb->SizeOf());

                            WriteOut(stdb->DataBegin(),m.face.size(),stdb->SizeOf() );
                        }
            }

            ///* save the per mesh attributes */
            {
                typename std::set< typename SaveMeshType::PointerToAttribute>::const_iterator ai;
                unsigned int n_named_attr = 0;
                for(ai = m.mesh_attr.begin(); ai != m.mesh_attr.end(); ++ai) n_named_attr+=!(*ai)._name.empty();
                WriteString( "N_PER_MESH_ATTRIBUTES"); WriteInt( n_named_attr);
                for(ai = m.mesh_attr.begin(); ai != m.mesh_attr.end(); ++ai)
                    if(!(*ai)._name.empty())
                        {
              SimpleTempDataBase  *    handle =  (SimpleTempDataBase  *)   (*ai)._handle ;

                            WriteString( "PER_MESH_ATTR_NAME");
                            WriteString( (*ai)._name.c_str());

                            WriteString( "PER_MESH_ATTR_SIZE");
                            WriteInt( handle->SizeOf());

                            WriteOut(handle->DataBegin(),1,handle->SizeOf() );
                        }
            }

            //	fflush(F());
            return 0;
        }
        static const char *ErrorMsg(int error)
        {
          static std::vector<std::string> off_error_msg;
          if(off_error_msg.empty())
          {
            off_error_msg.resize(2 );
            off_error_msg[0]="No errors";
              off_error_msg[1]="Can't open file";
            }

          if(error>1 || error<0) return "Unknown error";
          else return off_error_msg[error].c_str();
        }
    }; // end class

} // end Namespace tri
} // end Namespace io
} // end Namespace vcg

#endif
