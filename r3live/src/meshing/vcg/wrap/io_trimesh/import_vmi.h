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
* This program is Free software; you can redistribute it and/or modify      *
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
 Revision 1.1  2007/02/14 01:20:37  ganovelli
 working draft of VCG Mesh Image importer and exporter. Does not consider optional attributes. The mesh atributes are only vn and fn (no bbox, texture coordiantes)



****************************************************************************/

#ifndef __VCGLIB_IMPORT_VMI
#define __VCGLIB_IMPORT_VMI

#include <wrap/io_trimesh/io_mask.h>
#include <wrap/callback.h>
/*
    VMI VCG Mesh Image.
    The vmi image file consists of a header containing the description of the vertex and face type,
    the length of vectors containing vertices of faces and the memory image of the object mesh as it is when
    passed to the function Save(SaveMeshType m)
    NOTE: THIS IS NOT A FILE FORMAT. IT IS ONLY USEFUL FOR DUMPING MESH IMAGES FOR DEBUG PURPOSE.
    Example of use: say you are running a time consuming mesh processing and you want to save intermediate
    state, but no file format support all the attributes you need in your vertex/face type.
    NOTE2: At the present if you add members to your TriMesh these will NOT be saved. More precisely, this file and
    import_vmi must be updated to reflect changes in vcg/complex/trimesh/base.h

*/

namespace vcg {
namespace tri {
namespace io {

    template <int N> struct DummyType{ char placeholder[N]; };

    /* ------------------------- derivation chain for the vertex attribute ---------------------------*/

    /** this class is for testing only the equality with the type optionally provided by the user when calling Open
    */
    template <class MeshType, class A, class T>
    struct Der:public T{
        typedef typename std::set<typename MeshType::PointerToAttribute >::iterator HWIte;

        template <int VoF>
        static void AddAttrib(MeshType &m, const char * name, unsigned int s, void * data){
            switch(VoF)
            {
            case 0: if(s == sizeof(A)){
                    typename MeshType::template PerVertexAttributeHandle<A> h = vcg::tri::Allocator<MeshType>:: template AddPerVertexAttribute<A>(m,name);
                    for(unsigned int i  = 0; i < m.vert.size(); ++i)
                        memcpy(&h[i], (void*) &((A*)data)[i],sizeof(A)); // we don't want the type conversion
                        }
                    else
                        T::template AddAttrib<0>(m,name,s,data);
                break;
            case 1: if(s == sizeof(A)){
                    typename MeshType::template PerFaceAttributeHandle<A> h = vcg::tri::Allocator<MeshType>:: template AddPerFaceAttribute<A>(m,name);
                    for(unsigned int i  = 0; i < m.face.size(); ++i)
                        memcpy(&h[i], (void*) &((A*)data)[i],sizeof(A)); // we don't want the type conversion
                    }
                    else
                        T::template AddAttrib<0>(m,name,s,data);
                break;
            case 2:
                if(s == sizeof(A)){
                    typename MeshType::template PerMeshAttributeHandle<A> h = vcg::tri::Allocator<MeshType>:: template AddPerMeshAttribute<A>(m,name);
                        memcpy(&h(), (void*) ((A*)data),sizeof(A)); // we don't want the type conversion
                }
                    else
                        T::template AddAttrib<2>(m,name,s,data);
                break;

            default:break;
            }
        }
    };

    /** this class is for testing the list of increasing size types until one is larger than the size of the unknown type
    */
    template <class MeshType, class A, class T>
    struct DerK:public T{
        typedef typename std::set<typename MeshType::PointerToAttribute >::iterator HWIte;
        template <int VoF>
        static void AddAttrib(MeshType &m, const char * name, unsigned int s, void * data){
            switch(VoF){
                    case 0:
                    if(s == sizeof(A)){
                        typename MeshType::template PerVertexAttributeHandle<A> h = vcg::tri::Allocator<MeshType>::template AddPerVertexAttribute<A>(m,name);
                        for(unsigned int i  = 0; i < m.vert.size(); ++i)
                            memcpy((void*) &(h[i]), (void*) &((A*)data)[i],sizeof(A)); // we don't want the type conversion
                    }
                    else
                        if(s < sizeof(A)){
                            // padding
                            int padd = sizeof(A) - s;
                            typename MeshType::template PerVertexAttributeHandle<A> h = vcg::tri::Allocator<MeshType>::template AddPerVertexAttribute<A>(m,name);
                            for(unsigned int i  = 0; i < m.vert.size(); ++i){
                                char * dest =  &((char*)(&h[i]))[0];
                                memcpy( (void *)dest , (void*) &((A*)data)[i],s); // we don't want the type conversion
                            }
                            typename MeshType::PointerToAttribute pa;
                            pa._name = std::string(name);
                            HWIte res = m.vert_attr.find(pa);
                            pa = *res;
                            m.vert_attr.erase(res);
                            pa._padding = padd;
                            std::pair<HWIte,bool > new_pa = m.vert_attr.insert(pa);
                            (void)new_pa;
                            assert(new_pa.second);
                        }
                        else
                            T::template AddAttrib<0>(m,name,s,data);
                    break;
                    case 1:
                        if(s == sizeof(A)){
                            typename MeshType::template PerVertexAttributeHandle<A> h = vcg::tri::Allocator<MeshType>::template AddPerVertexAttribute<A>(m,name);
                            for(unsigned int i  = 0; i < m.vert.size(); ++i)
                            memcpy((void*) &(h[i]), (void*) &((A*)data)[i],sizeof(A)); // we don't want the type conversion
                            }
                        else
                            if(s < sizeof(A)){
                                // padding
                                int padd = sizeof(A) - s;
                                typename MeshType::template PerFaceAttributeHandle<A> h = vcg::tri::Allocator<MeshType>::template AddPerFaceAttribute<A>(m,name);
                                for(unsigned int i  = 0; i < m.face.size(); ++i){
                                    char * dest =  &((char*)(&h[i]))[0];
                                    memcpy( (void *)dest , (void*) &((A*)data)[i],s); // we don't want the type conversion
                                }
                                typename MeshType::PointerToAttribute pa;
                                pa._name = std::string(name);
                                HWIte res = m.face_attr.find(pa);
                                pa = *res;
                                m.face_attr.erase(res);
                                pa._padding = padd;
                                std::pair<HWIte,bool > new_pa = m.face_attr.insert(pa);
                                (void)new_pa;
                                assert(new_pa.second);
                            }
                        else
                            T::template AddAttrib<1>(m,name,s,data);
                        break;
                    case 2:
                        if(s == sizeof(A)){
                            typename MeshType::template PerMeshAttributeHandle<A> h = vcg::tri::Allocator<MeshType>::template AddPerMeshAttribute<A>(m,name);
                                memcpy((void*)&h(), (void*)((A*)data),sizeof(A)); // we don't want the type conversion
                            }
                        else
                            if(s < sizeof(A)){
                                // padding
                                int padd = sizeof(A) - s;
                                typename MeshType::template PerMeshAttributeHandle<A> h = vcg::tri::Allocator<MeshType>::template AddPerMeshAttribute<A>(m,name);
                                char * dest =  & ((char*)(&h()))[0];
                                memcpy( (void *)dest , (void*)((A*)data),s); // we don't want the type conversion

                                typename MeshType::PointerToAttribute pa;
                                pa._name = std::string(name);
                                HWIte res = m.mesh_attr.find(pa);
                                pa = *res;
                                m.mesh_attr.erase(res);
                                pa._padding = padd;
                                std::pair<HWIte,bool > new_pa = m.mesh_attr.insert(pa);
                                (void)new_pa;
                                assert(new_pa.second);
                            }
                        else
                            T::template AddAttrib<2>(m,name,s,data);
                        break;
                    default: assert(0);break;
            }
        }
    };

    /**
    This is the templated derivation  chain
    */
    template <class MeshType>	struct K	{
        template <int VoF>
        static void AddAttrib(MeshType &/*m*/, const char * /*name*/, unsigned int /*s*/, void * /*data*/){
            // if yohu got this your attribute is larger than	1048576. Honestly...
            assert(0);
        }
    };

    template <class MeshType, class B0 >																												struct K0	: public DerK<  MeshType, B0,	K<MeshType> > {};
    template <class MeshType, class B0, class B1 >																										struct K1	: public DerK<  MeshType, B1,	K0<MeshType, B0> > {};
    template <class MeshType, class B0, class B1, class B2 >																							struct K2	: public DerK<  MeshType, B2,	K1<MeshType, B0, B1> > {};
    template <class MeshType, class B0, class B1, class B2,class B3>																					struct K3	: public DerK<  MeshType, B3,	K2<MeshType, B0, B1, B2> > {};
    template <class MeshType, class B0, class B1, class B2,class B3,class B4>																			struct K4	: public DerK<  MeshType, B4,	K3<MeshType, B0, B1, B2, B3> > {};
    template <class MeshType, class B0, class B1, class B2,class B3,class B4,class B5>																	struct K5	: public DerK<  MeshType, B5,	K4<MeshType, B0, B1, B2, B3, B4> > {};
    template <class MeshType, class B0, class B1, class B2,class B3,class B4,class B5,class B6>															struct K6	: public DerK<  MeshType, B6,	K5<MeshType, B0, B1, B2, B3, B4, B5> > {};
    template <class MeshType, class B0, class B1, class B2,class B3,class B4,class B5,class B6,class B7>												struct K7	: public DerK<  MeshType, B7,	K6<MeshType, B0, B1, B2, B3, B4, B5, B6> > {};
    template <class MeshType, class B0, class B1, class B2,class B3,class B4,class B5,class B6,class B7,class B8>										struct K8	: public DerK<  MeshType, B8,	K7<MeshType, B0, B1, B2, B3, B4, B5, B6, B7> > {};
    template <class MeshType, class B0, class B1, class B2,class B3,class B4,class B5,class B6,class B7,class B8,class B9>								struct K9	: public DerK<  MeshType, B9,	K8<MeshType, B0, B1, B2, B3, B4, B5, B6, B7, B8> > {};
    template <class MeshType, class B0, class B1, class B2,class B3,class B4,class B5,class B6,class B7,class B8,class B9,class B10>					struct K10	: public DerK<  MeshType, B10,	K9<MeshType, B0, B1, B2, B3, B4, B5, B6, B7, B8, B9> > {};
    template <class MeshType, class B0, class B1, class B2,class B3,class B4,class B5,class B6,class B7,class B8,class B9,class B10,class B11>			struct K11	: public DerK<  MeshType, B11,	K10<MeshType, B0, B1, B2, B3, B4, B5, B6, B7, B8, B9, B11 > > {};
    template <class MeshType, class B0, class B1, class B2,class B3,class B4,class B5,class B6,class B7,class B8,class B9,class B10,class B11,class B12>struct K12	: public DerK<  MeshType, B12,	K11<MeshType, B0, B1, B2, B3, B4, B5, B6, B7, B8, B9, B11, B12 > > {};

    template <class MeshType, class A0,
        class B0  = DummyType<1048576>,
        class B1  = DummyType<2048>,
        class B2  = DummyType<1024>,
        class B3  = DummyType<512>,
        class B4  = DummyType<256>,
        class B5  = DummyType<128>,
        class B6  = DummyType<64>,
        class B7  = DummyType<32>,
        class B8  = DummyType<16>,
        class B9  = DummyType<8>,
        class B10 = DummyType<4>,
        class B11 = DummyType<2>,
        class B12 = DummyType<1>
    >	struct C0		: public DerK<  MeshType, A0,    K12<MeshType, B0, B1, B2, B3, B4,B5,B6,B7,B8,B9,B10,B11,B12> > {};

    template <class MeshType, class A0, class A1>											struct C1		: public Der<  MeshType, A1,	C0<MeshType, A0> > {};
    template <class MeshType, class A0, class A1, class A2>	 								struct C2		: public Der<  MeshType, A2,	C1<MeshType, A0, A1> > {};
    template <class MeshType, class A0, class A1, class A2,class A3>	 					struct C3		: public Der<  MeshType, A3,	C2<MeshType, A0, A1, A2> > {};
    template <class MeshType, class A0, class A1, class A2,class A3,class A4>				struct AttrAll	: public Der<  MeshType, A4,	C3<MeshType, A0, A1, A2, A3> > {};



    template <class OpenMeshType,class A0 = long, class A1 = double, class A2 = int,class A3 = short, class A4 = char >
    class ImporterVMI: public AttrAll<OpenMeshType,A0,A1,A2,A3,A4>
    {

        static void ReadString(std::string & out){
            unsigned int l; Read(&l,4,1);
            char * buf = new char[l+1];
            Read(buf,1,l);buf[l]='\0';
            out = std::string(buf);
            delete [] buf;
            }

        static void ReadInt( unsigned int & i){ Read(&i,1,4);}
        static void ReadFloat( float & v){ Read(&v,1,sizeof(float));}


         static int   LoadVertexOcfMask( ){
            int mask  =0;
            std::string s;

            // vertex quality
            ReadString( s);
            if( s == std::string("HAS_VERTEX_QUALITY_OCF")) mask |= Mask::IOM_VERTQUALITY;

            // vertex color
            ReadString( s);
            if( s == std::string("HAS_VERTEX_COLOR_OCF"))  mask |= Mask::IOM_VERTCOLOR;

            // vertex normal
            ReadString( s);
            if( s == std::string("HAS_VERTEX_NORMAL_OCF")) mask |= Mask::IOM_VERTNORMAL;

            // vertex mark
            ReadString( s);
            //if( s == std::string("HAS_VERTEX_MARK_OCF"))  mask |=

            // vertex texcoord
            ReadString( s);
            if( s == std::string("HAS_VERTEX_TEXCOORD_OCF"))  mask |= Mask::IOM_VERTTEXCOORD;

            // vertex-face adjacency
            ReadString( s);
            //if( s == std::string("HAS_VERTEX_VFADJACENCY_OCF")) mask |=

            // vertex curvature
            ReadString( s);
            //if( s == std::string("HAS_VERTEX_CURVATURE_OCF"))  mask |=

            //// vertex curvature dir
            ReadString( s);
            //if( s == std::string("HAS_VERTEX_CURVATUREDIR_OCF"))  mask |=

            // vertex radius
            ReadString( s);
            if( s == std::string("HAS_VERTEX_RADIUS_OCF"))  mask |= Mask::IOM_VERTRADIUS;

            return mask;
       }

        template <typename MeshType, typename CONT>
                struct LoadVertexOcf{
                        LoadVertexOcf(FILE* /*f*/,const CONT & /*vert*/){
                                // do nothing, it is a std::vector
                        }
                };



        template <typename MeshType>
        struct
        LoadVertexOcf<MeshType,vertex::vector_ocf<typename OpenMeshType::VertexType> >{
                        typedef typename OpenMeshType::VertexType VertexType;
            LoadVertexOcf( FILE * /*f*/, vertex::vector_ocf<typename OpenMeshType::VertexType> & vert){
            std::string s;

                // vertex quality
                ReadString( s);
                if( s == std::string("HAS_VERTEX_QUALITY_OCF")) {
                        vert.EnableQuality();
                        Read((void*)&vert.QV[0],sizeof(typename VertexType::QualityType),vert.size() );
                }

                // vertex color
                ReadString( s);
                if( s == std::string("HAS_VERTEX_COLOR_OCF")) {
                        vert.EnableColor();
                        Read((void*)&vert.CV[0],sizeof(typename VertexType::ColorType),vert.size() );
                }

                // vertex normal
                ReadString( s);
                if( s == std::string("HAS_VERTEX_NORMAL_OCF")) {
                        vert.EnableNormal();
                        Read((void*)&vert.NV[0],sizeof(typename VertexType::NormalType),vert.size() );
                }

                 // vertex mark
                ReadString( s);
                if( s == std::string("HAS_VERTEX_MARK_OCF")) {
                        vert.EnableMark();
                        Read((void*)&vert.MV[0],sizeof(typename VertexType::MarkType),vert.size() );
                }

                // vertex texcoord
                ReadString( s);
                if( s == std::string("HAS_VERTEX_TEXCOORD_OCF")) {
                        vert.EnableTexCoord();
                        Read((void*)&vert.TV[0],sizeof(typename VertexType::TexCoordType),vert.size() );
                }

                // vertex-face adjacency
                ReadString( s);
                if( s == std::string("HAS_VERTEX_VFADJACENCY_OCF")) {
                        vert.EnableVFAdjacency();
                        Read((void*)&vert.AV[0],sizeof(typename vertex::vector_ocf<VertexType>::VFAdjType),vert.size() );
                }

                // vertex curvature
                ReadString( s);
                if( s == std::string("HAS_VERTEX_CURVATURE_OCF")) {
                        vert.EnableCurvature();
                        Read((void*)&vert.CuV[0],sizeof(typename VertexType::CurvatureType),vert.size() );
                }

                // vertex curvature dir
                ReadString( s);
                if( s == std::string("HAS_VERTEX_CURVATUREDIR_OCF")) {
                        vert.EnableCurvatureDir();
                        Read((void*)&vert.CuDV[0],sizeof(typename VertexType::CurvatureDirType),vert.size() );
                }

                // vertex radius
                ReadString( s);
                if( s == std::string("HAS_VERTEX_RADIUS_OCF")) {
                        vert.EnableRadius();
                        Read((void*)&vert.RadiusV[0],sizeof(typename VertexType::RadiusType),vert.size() );
                }

                        }
                };

                template <typename MeshType, typename CONT>
                struct LoadFaceOcf{
                        LoadFaceOcf(const CONT & /* face */){
                                // do nothing, it is a std::vector
                        }
                };


                 static int  LoadFaceOcfMask( ){
                int mask=0;
                    std::string s;

                // face quality
                    ReadString( s);
                    if( s == std::string("HAS_FACE_QUALITY_OCF"))	mask	|=  Mask::IOM_FACEQUALITY;

                    // face color
                    ReadString( s);
                         if( s == std::string("HAS_FACE_COLOR_OCF"))		mask	|=   Mask::IOM_FACECOLOR;

                    // face normal
                    ReadString( s);
                    if( s == std::string("HAS_FACE_NORMAL_OCF"))  mask	|=  Mask::IOM_FACENORMAL;

                    //// face mark
                    ReadString( s);
                    //if( s == std::string("HAS_FACE_MARK_OCF")) mask	|=

                    // face wedgetexcoord
                    ReadString( s);
                    if( s == std::string("HAS_FACE_WEDGETEXCOORD_OCF")) mask	|= Mask::IOM_WEDGTEXCOORD;


                    // face-face adjacency
                    ReadString( s);
//                    if( s == std::string("HAS_FACE_FFADJACENCY_OCF")) mask	|= */

                    // vertex-face adjacency
                    ReadString( s);
                    //if( s == std::string("HAS_FACE_VFADJACENCY_OCF")) mask	|=

                    // face WedgeColor
                    ReadString( s);
                    if( s == std::string("HAS_FACE_WEDGECOLOR_OCF")) mask	|=  Mask::IOM_WEDGCOLOR;

                    // face WedgeNormal
                    ReadString( s);
                    if( s == std::string("HAS_FACE_WEDGENORMAL_OCF")) mask	|=  Mask::IOM_WEDGNORMAL;
                    return mask;
            }


                /* partial specialization for vector_ocf */
                template <typename MeshType>
                                                                struct LoadFaceOcf< MeshType, face::vector_ocf<typename OpenMeshType::FaceType> >{
                                                typedef typename OpenMeshType::FaceType FaceType;
                        LoadFaceOcf( face::vector_ocf<FaceType> & face){
                                std::string s;

                                // face quality
                                ReadString( s);
                                if( s == std::string("HAS_FACE_QUALITY_OCF")) {
                                        face.EnableQuality();
                                        Read((void*)&face.QV[0],sizeof(typename FaceType::QualityType),face.size() );
                                }

                                // face color
                                ReadString( s);
                                if( s == std::string("HAS_FACE_COLOR_OCF")) {
                                        face.EnableColor();
                                        Read((void*)&face.CV[0],sizeof(typename FaceType::ColorType),face.size() );
                                }

                                // face normal
                                ReadString( s);
                                if( s == std::string("HAS_FACE_NORMAL_OCF")) {
                                        face.EnableNormal();
                                        Read((void*)&face.NV[0],sizeof(typename FaceType::NormalType),face.size() );
                                }

                                // face mark
                                ReadString( s);
                                if( s == std::string("HAS_FACE_MARK_OCF")) {
                                        face.EnableMark();
                                        Read((void*)&face.MV[0],sizeof(typename FaceType::MarkType),face.size() );
                                }

                                // face wedgetexcoord
                                ReadString( s);
                                if( s == std::string("HAS_FACE_WEDGETEXCOORD_OCF")) {
                                        face.EnableWedgeTexCoord();
                                        Read((void*)&face.WTV[0],sizeof(typename FaceType::WedgeTexCoordType),face.size() );
                                }


                                // face-face adjacency
                                ReadString( s);
                                if( s == std::string("HAS_FACE_FFADJACENCY_OCF")) {
                                        face.EnableFFAdjacency();
                                        Read((void*)&face.AF[0],sizeof(typename  face::vector_ocf<FaceType>::AdjTypePack),face.size() );
                                }

                                // vertex-face adjacency
                                ReadString( s);
                                if( s == std::string("HAS_FACE_VFADJACENCY_OCF")) {
                                        face.EnableVFAdjacency();
                                        Read((void*)&face.AV[0],sizeof(typename  face::vector_ocf<FaceType>::AdjTypePack),face.size() );
                                }

                                // face WedgeColor
                                ReadString( s);
                                if( s == std::string("HAS_FACE_WEDGECOLOR_OCF")) {
                                        face.EnableWedgeColor();
                                        Read((void*)&face.WCV[0],sizeof(typename  face::vector_ocf<FaceType>::WedgeColorTypePack),face.size() );
                                }

                                // face WedgeNormal
                                ReadString( s);
                                if( s == std::string("HAS_FACE_WEDGENORMAL_OCF")) {
                                        face.EnableWedgeNormal();
                                        Read((void*)&face.WNV[0],sizeof(typename  face::vector_ocf<FaceType>::WedgeNormalTypePack),face.size() );
                                }
                        }
                };

    static int  FaceMaskBitFromString(std::string s){
        if( s.find("Color",0) != std::string::npos )			return Mask::IOM_FACECOLOR;			else
        if( s.find("BitFlags",0) != std::string::npos )			return Mask::IOM_FACEFLAGS;			else
        if( s.find("VertexRef",0) != std::string::npos )		return Mask::IOM_FACEINDEX;			else
        if( s.find("Normal",0) != std::string::npos )			return Mask::IOM_FACENORMAL;		else
        if( s.find("Quality",0) != std::string::npos )			return Mask::IOM_FACEQUALITY;		else
        if( s.find("Quality",0) != std::string::npos )			return Mask::IOM_FACEQUALITY;		else
        if( s.find("WedgeColor",0) != std::string::npos )		return Mask::IOM_WEDGCOLOR;			else
        if( s.find("WedgeNormal",0) != std::string::npos )		return Mask::IOM_WEDGNORMAL;		else
        if( s.find("WedgeTexCoord",0) != std::string::npos)		return Mask::IOM_WEDGTEXCOORD;		else
        return 0;
    }
    static int  VertexMaskBitFromString(std::string s){
        if( s.find("Color",0) != std::string::npos )		return Mask::IOM_VERTCOLOR;		else
        if( s.find("Coord",0) != std::string::npos )		return Mask::IOM_VERTCOORD;		else
        if( s.find("BitFlags",0) != std::string::npos )		return Mask::IOM_VERTFLAGS;		else
        if( s.find("Quality",0) != std::string::npos )		return Mask::IOM_VERTQUALITY;	else
        if( s.find("Normal",0) != std::string::npos )		return Mask::IOM_VERTNORMAL;	else
        if( s.find("TexCoord",0) != std::string::npos )		return Mask::IOM_VERTTEXCOORD;	else
        if( s.find("Radius",0) != std::string::npos )		return Mask::IOM_VERTRADIUS;	else
            return 0;
    }


        static FILE *& F(){static FILE * f; return f;}


        static void * Malloc(unsigned int n){ return (n)?malloc(n):0;}
        static void Free(void * ptr){ if(ptr) free (ptr);}


        typedef typename OpenMeshType::FaceType FaceType;
        typedef typename OpenMeshType::FaceContainer FaceContainer;
        typedef typename OpenMeshType::FaceIterator FaceIterator;

        typedef typename OpenMeshType::VertContainer VertContainer;
        typedef typename OpenMeshType::VertexIterator VertexIterator;
        typedef typename OpenMeshType::VertexType VertexType;

    public:
       enum VMIErrorCodes {
                VMI_NO_ERROR = 0,
                VMI_INCOMPATIBLE_VERTEX_TYPE,
                VMI_INCOMPATIBLE_FACE_TYPE,
                VMI_FAILED_OPEN
       };

        /*!
         *	Standard call for knowing the meaning of an error code
         * \param message_code	The code returned by <CODE>Open</CODE>
         *	\return							The string describing the error code
         */
        static const char* ErrorMsg(int message_code)
        {
            static const char* error_msg[] =
            {
                "No errors",
                "The file has a incompatible vertex signature",
                "The file has a incompatible Face signature",
                "General failure of the file opening"
            };

            if(message_code>4 || message_code<0)
                return "Unknown error";
            else
                return error_msg[message_code];
        };

        /* Read the info about the mesh. Note: in the header the bounding box is always written/readed
           as a vcg::Box3f, even if the scalar type is not float. The bounding box of the mesh will
           be set properly on loading.
          */
        static bool GetHeader(	std::vector<std::string>& fnameV,
                                std::vector<std::string>& fnameF,
                                unsigned int & vertSize,
                                unsigned int &faceSize,
                                vcg::Box3f & bbox,
                                int & mask){
            std::string name;
            unsigned int nameFsize,nameVsize,i;

            ReadString( name); ReadInt( nameFsize);

            for(i=0; i < nameFsize; ++i)
                {ReadString(  name);fnameF.push_back( name );mask |= FaceMaskBitFromString(name);}
            mask |= LoadFaceOcfMask();

            ReadString( name); ReadInt( faceSize);
            ReadString(  name); ReadInt( nameVsize);

            for(i=0; i < nameVsize; ++i)
                {ReadString(  name) ;fnameV.push_back( name);mask |= VertexMaskBitFromString(name);}
            mask |= LoadVertexOcfMask();

            ReadString( name);
            ReadInt( vertSize);

            ReadString( name);
            float float_value;
            for(unsigned int i =0; i < 2; ++i){ReadFloat( float_value); bbox.min[i]=float_value;}
            for(unsigned int i =0; i < 2; ++i){ReadFloat( float_value); bbox.max[i]=float_value;}

            ReadString( name);
            assert(strstr( name.c_str(),"end_header")!=NULL);
            return true;
        }


        static bool GetHeader(const char * filename,std::vector<std::string>& nameV, std::vector<std::string>& nameF, unsigned int & vertSize, unsigned int &faceSize,vcg::Box3f & bbox,int & mask){
                F() = fopen(filename,"rb");
                bool res =  GetHeader(nameV, nameF, vertSize, faceSize,bbox,mask);
                fclose(F());
                return res;
    }

    public:
        static const char * & In_mem(){static const char *    in_mem; return in_mem;}
        static unsigned int & In_mode(){static unsigned int  in_mode = 0; return in_mode;}


        static unsigned int & pos(){static unsigned int  p = 0; return p;}
        static int Read_sim(const void * , size_t size, size_t count ){ pos() += size * count;return size * count; }
        static int Read_mem( void *dst , size_t size, size_t count ){ memcpy(dst,&In_mem()[pos()],size*count); pos() += size * count;return size * count; }


        static int Read( void * dst,  size_t size, size_t count){
            switch(In_mode()){
            case 0: return Read_mem(dst, size,count );  break;
            case 1: return fread(dst, size,count, F() ); break;
             }
            assert(0);
            return 0;
        }


        static bool LoadMask(const char * f, int & mask){
            std::vector<std::string>  nameV;
            std::vector<std::string>  nameF;
            unsigned int   vertSize, faceSize;
            vcg::Box3f bbox;
            F() = fopen(f,"rb");
            In_mode() = 1;
            GetHeader(nameV,nameF,vertSize, faceSize, bbox, mask);
            return true;
        }

        static bool LoadMaskFromMem(  const char * ptr, int & mask){
            std::vector<std::string>  nameV;
            std::vector<std::string>  nameF;
            unsigned int   vertSize, faceSize;
            vcg::Box3f bbox;
            In_mode() = 0;
            pos() = 0;
            In_mem() = ptr;
            GetHeader(nameV,nameF,vertSize, faceSize, bbox, mask);
            return true;
        }

        static int Open(OpenMeshType &m, const char * filename, int & mask,CallBackPos  * /*cb*/ = 0 )       {
            In_mode() = 1;
            F() = fopen(filename,"rb");
            if(!F()) return VMI_FAILED_OPEN;
            if(F()==NULL)	return 1; // 1 is the error code for cant'open, see the ErrorMsg function
            int res = Deserialize(m,mask);
            fclose(F());
            return  res;
        }
        static int ReadFromMem(  OpenMeshType &m, int & mask,char * ptr){
            In_mode() = 0;
            pos() = 0;
            In_mem() = ptr;
            return Deserialize(m,mask);
        }

        static int Deserialize(OpenMeshType &m, int & mask)
        {
            typedef typename OpenMeshType::VertexType VertexType;
            typedef typename OpenMeshType::FaceType FaceType;
            typename OpenMeshType::FaceIterator fi;
            typename OpenMeshType::VertexIterator vi;

            std::vector<std::string> nameF,nameV,fnameF,fnameV;
            unsigned int vertSize,faceSize;

            /* read the header */
      vcg::Box3f lbbox;
      GetHeader(fnameV, fnameF, vertSize, faceSize,lbbox,mask);
      m.bbox.Import(lbbox);
            /* read the mesh type */
            OpenMeshType::FaceType::Name(nameF);
            OpenMeshType::VertexType::Name(nameV);

            /* check if the type is the very same, otherwise return */
            if(fnameV != nameV) return VMI_INCOMPATIBLE_VERTEX_TYPE;
            if(fnameF != nameF) return VMI_INCOMPATIBLE_FACE_TYPE;

             void *  offsetV = 0,*offsetF = 0;

             if(vertSize!=0)
                /* read the address of the first vertex */
                Read(&offsetV,sizeof( void *),1 );

             if(faceSize!=0)
                /* read the address of the first face */
                Read(&offsetF,sizeof( void *),1 );

            /* read the object mesh */
            Read(&m.shot,sizeof(Shot<typename OpenMeshType::ScalarType>),1 );
            Read(&m.vn,sizeof(int),1 );
            Read(&m.fn,sizeof(int),1 );
            Read(&m.imark,sizeof(int),1 );
            Read(&m.bbox,sizeof(Box3<typename OpenMeshType::ScalarType>),1 );
            Read(&m.C(),sizeof(Color4b),1 );


            /* resize the vector of vertices */
            m.vert.resize(vertSize);


            size_t read = 0;
            /* load the vertices */
            if(vertSize>0){
                read=Read((void*)& m.vert[0],sizeof(VertexType),vertSize );
                 LoadVertexOcf<OpenMeshType,VertContainer>(F(),m.vert);
            }

            read = 0;
            m.face.resize(faceSize);
            if(faceSize>0){
                /* load the faces */
                read = Read((void*)& m.face[0],sizeof(FaceType),faceSize );
                LoadFaceOcf<OpenMeshType,FaceContainer>(m.face);
            }


            /* load the per vertex attributes */
            std::string _string,_trash;
            unsigned int n,sz;

            ReadString( _trash); ReadInt( n);

            for(size_t ia = 0 ; ia < n; ++ia){
                ReadString(_trash); ReadString(_string);
                ReadString(_trash); ReadInt(sz);

                void * data = Malloc(sz*m.vert.size());
                Read(data,sz,m.vert.size());
                AttrAll<OpenMeshType,A0,A1,A2,A3,A4>::template AddAttrib<0>(m,_string.c_str(),sz,data);
                Free(data);
            }

            /* load the per face attributes */
            ReadString(_trash); ReadInt( n);
            for(size_t ia = 0 ; ia < n; ++ia){
                ReadString(_trash); ReadString( _string);
                ReadString(_trash); ReadInt( sz);
                void * data = Malloc(sz*m.face.size());
                Read(data,sz,m.face.size() );
                AttrAll<OpenMeshType,A0,A1,A2,A3,A4>::template AddAttrib<1>(m,_string.c_str(),sz,data);
                Free(data);
            }

            /* load the per mesh attributes */
            ReadString( _trash); ReadInt( n);
            for(unsigned int ia = 0 ; ia < n; ++ia){
                ReadString( _trash); ReadString( _string);
                ReadString( _trash); ReadInt( sz);
                void * data = Malloc(sz);
                Read(data,1,sz );
                AttrAll<OpenMeshType,A0,A1,A2,A3,A4>::template AddAttrib<2>(m,_string.c_str(),sz,data);
                Free(data);
            }

            if(!m.face.empty()){
            if(FaceVectorHasVFAdjacency(m.face))
                for(vi = m.vert.begin(); vi != m.vert.end(); ++vi){
                    (*vi).VFp() = (*vi).VFp()-(FaceType*)offsetF+ &m.face[0];
                    (*vi).VFp() = (*vi).VFp()-(FaceType*)offsetF+ &m.face[0];
                    (*vi).VFp() = (*vi).VFp()-(FaceType*)offsetF+ &m.face[0];
                }

            if(FaceVectorHasFVAdjacency(m.face))
                for(fi = m.face.begin(); fi != m.face.end(); ++fi){
                    (*fi).V(0) = (*fi).V(0)-(VertexType*)offsetV+ &m.vert[0];
                    (*fi).V(1) = (*fi).V(1)-(VertexType*)offsetV+ &m.vert[0];
                    (*fi).V(2) = (*fi).V(2)-(VertexType*)offsetV+ &m.vert[0];
                }

            if(FaceVectorHasFFAdjacency(m.face))
                for(fi = m.face.begin(); fi != m.face.end(); ++fi){
                    (*fi).FFp(0) = (*fi).FFp(0)-(FaceType*)offsetF+ &m.face[0];
                    (*fi).FFp(1) = (*fi).FFp(1)-(FaceType*)offsetF+ &m.face[0];
                    (*fi).FFp(2) = (*fi).FFp(2)-(FaceType*)offsetF+ &m.face[0];
                }

            }

            return VMI_NO_ERROR; // zero is the standard (!) code of success
        }

    }; // end class


} // end Namespace tri
} // end Namespace io
} // end Namespace vcg

#endif
