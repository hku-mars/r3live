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
#ifndef EXPORT_CTM_H
#define EXPORT_CTM_H

namespace vcg {
    namespace tri {
        namespace io {
            template <class SaveMeshType>
            class ExporterCTM
            {

            public:
                typedef typename SaveMeshType::VertexPointer VertexPointer;
                typedef typename SaveMeshType::FaceIterator FaceIterator;

                static int Save(SaveMeshType &m, const char * filename, int mask=0, bool lossLessFlag=false, float relativePrecision=0.0001)
                {
                    tri::Allocator<SaveMeshType>::CompactVertexVector(m);
                    tri::Allocator<SaveMeshType>::CompactFaceVector(m);
                    CTMuint aVertCount=m.vn;
                    CTMuint aTriCount=m.fn;
                    std::vector<CTMfloat> aVertices(aVertCount*3);
                    std::vector<CTMfloat> aColors(aVertCount*4);
                    std::vector<CTMfloat> aQuality(aVertCount*4);
                    std::vector<CTMuint> aIndices(aTriCount*3);
                    int err;
                    CTMcontext context;
                    // Create a new exporter context
                    context = ctmNewContext(CTM_EXPORT);
                    if(lossLessFlag) ctmCompressionMethod(context, CTM_METHOD_MG1);
                    else {
                      ctmCompressionMethod(context, CTM_METHOD_MG2);
                      ctmVertexPrecision(context, relativePrecision);
                    }
                    // Prepare vertex and faces
                    for(unsigned int i=0;i<aVertCount;++i)
                    {
                        aVertices[i*3+0]=m.vert[i].P()[0];
                        aVertices[i*3+1]=m.vert[i].P()[1];
                        aVertices[i*3+2]=m.vert[i].P()[2];
                    }
                    for(unsigned int i=0;i<aTriCount;++i)
                    {
                        aIndices[i*3+0]=m.face[i].V(0)-&*m.vert.begin();
                        aIndices[i*3+1]=m.face[i].V(1)-&*m.vert.begin();
                        aIndices[i*3+2]=m.face[i].V(2)-&*m.vert.begin();
                    }
                    if(aTriCount==0)
                    {
                      aIndices.resize(3,0);
                      aTriCount=1;
                    }

                    // Define our mesh representation to OpenCTM
                    ctmDefineMesh(context, &*aVertices.begin(), aVertCount, &*aIndices.begin(), aTriCount, NULL);
                    err=ctmGetError(context);
                    if(err) return err;
                    if( tri::HasPerVertexColor(m)   && (mask & io::Mask::IOM_VERTCOLOR))
                    {
                        aColors.resize(aVertCount*4);
                        for(unsigned int i=0;i<aVertCount;++i)
                        {
                            aColors[i*4+0]=(float)(m.vert[i].C()[0])/255.0f;
                            aColors[i*4+1]=(float)(m.vert[i].C()[1])/255.0f;
                            aColors[i*4+2]=(float)(m.vert[i].C()[2])/255.0f;
                            aColors[i*4+3]=(float)(m.vert[i].C()[3])/255.0f;
                        }
                        ctmAddAttribMap(context,&aColors[0], "Color");
                    }
                    if( tri::HasPerVertexQuality(m)   && (mask & io::Mask::IOM_VERTQUALITY))
                    {
                        aQuality.resize(aVertCount*4,0);
                        for(unsigned int i=0;i<aVertCount;++i)
                        {
                            aQuality[i*4+0]=m.vert[i].Q();
                        }
                        ctmAddAttribMap(context,&aQuality[0], "Quality");
                    }

                    // Save the OpenCTM file
                    ctmSave(context, filename);
                    err=ctmGetError(context);
                    if(err) return err;
                    // Free the context
                    ctmFreeContext(context);
                    return err;
                }
                /*
            returns mask of capability one define with what are the saveable information of the format.
        */
                static int GetExportMaskCapability()
                {
                    int capability = 0;

                    //vert
                    capability |= vcg::tri::io::Mask::IOM_VERTCOORD;
                    capability |= vcg::tri::io::Mask::IOM_VERTQUALITY;
                    capability |= vcg::tri::io::Mask::IOM_VERTCOLOR;
                    return capability;
                }

                static const char *ErrorMsg(int error)
                {
                    if(error==0) return "Ok, no errors";
                    else return ctmErrorString((CTMenum)error);
                }
            };
        } // end namespace io
    } // end namespace tri
} // end namespace vcg
#endif // EXPORT_CTM_H
