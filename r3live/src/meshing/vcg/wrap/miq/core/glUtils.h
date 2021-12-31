#ifndef MIQ_GL_UTILS
#define MIQ_GL_UTILS
//#include <wrap/gl/space.h>
#include "vertex_indexing.h"

class Miq_Gl_Utils
{
public:

    ///singular vertices should be selected
    template <class MeshType>
    static void GLDrawSingularities(MeshType &mesh,
                                    float size=8,
                                    bool DrawUV=false)
    {
        bool hasSingular = vcg::tri::HasPerVertexAttribute(mesh,std::string("Singular"));
        bool hasSingularDegree = vcg::tri::HasPerVertexAttribute(mesh,std::string("SingularityDegree"));
        if (!hasSingular)return;

        typename MeshType::template PerVertexAttributeHandle<bool> Handle_Singular;
        typename MeshType::template PerVertexAttributeHandle<int> Handle_SingularDegree;

        Handle_Singular=vcg::tri::Allocator<MeshType>::template GetPerVertexAttribute<bool>(mesh,std::string("Singular"));

        Handle_SingularDegree=vcg::tri::Allocator<MeshType>::template GetPerVertexAttribute<int>(mesh,std::string("SingularityDegree"));

        glPushAttrib(GL_ALL_ATTRIB_BITS);
        glEnable(GL_COLOR_MATERIAL);
        glDisable(GL_LIGHTING);
        glDepthRange(0,0.999);
        glPointSize(size);
        glBegin(GL_POINTS);
        glColor4d(0,1,1,0.7);
        for (unsigned int j=0;j<mesh.face.size();j++)
        {
            CFace *f=&mesh.face[j];
            if (f->IsD())continue;
            for (int i=0;i<3;i++)
            {
                CVertex *v=f->V(i);
                if (!Handle_Singular[v])continue;
                //int mmatch=3;
                if (hasSingularDegree)
                {
                    int mmatch=Handle_SingularDegree[v];
                    if (mmatch==1)vcg::glColor(vcg::Color4b(255,0,0,255));///valence 5
                    if (mmatch==2)vcg::glColor(vcg::Color4b(0,255,0,255));///valence 6
                    if (mmatch==3)vcg::glColor(vcg::Color4b(0,0,255,255));///valence 3
                }
                if (!DrawUV)
                    vcg::glVertex(v->P());
                else
                    glVertex3d(f->WT(i).P().X(),f->WT(i).P().Y(),0);
            }
        }

        glEnd();
        glPopAttrib();
    }

    template <class FaceType>
    static void GLDrawFaceSeams(const FaceType &f,
                                vcg::Point3<bool> seams,
                                vcg::Color4b seamCol[3],
                                float size=3,
                                bool UV=false)
    {
        typedef typename FaceType::CoordType CoordType;
        typedef typename FaceType::ScalarType ScalarType;
        glLineWidth(size);
        glBegin(GL_LINES);
        for (int i=0;i<3;i++)
        {
            if (!seams[i])continue;
            vcg::glColor(seamCol[i]);
            if (!UV)
            {
                glVertex(f.cV0(i)->P());
                glVertex(f.cV1(i)->P());
            }
            else
            {
                int i0=i;
                int i1=(i0+1)%3;
                CoordType p0=CoordType(f.cWT(i0).P().X(),f.cWT(i0).P().Y(),0);
                CoordType p1=CoordType(f.cWT(i1).P().X(),f.cWT(i1).P().Y(),0);
                glVertex(p0);
                glVertex(p1);
            }
        }
        glEnd();
    }

    template <class MeshType>
    static void GLDrawSeams(MeshType &mesh,
                            float size=3,
                            bool UV=false,
                            int numCuts=400)
    {
        bool hasSeam = vcg::tri::HasPerFaceAttribute(mesh,std::string("Seams"));
        if(!hasSeam)return;
        bool HasSeamIndex=vcg::tri::HasPerFaceAttribute(mesh,std::string("SeamsIndex"));

        typedef typename MeshType::template PerFaceAttributeHandle<vcg::Point3<bool> > SeamsHandleType;
        typedef typename MeshType::template PerFaceAttributeHandle<vcg::Point3i > SeamsIndexHandleType;

        typedef typename vcg::tri::Allocator<MeshType> SeamsAllocator;

        SeamsHandleType Handle_Seam;
        Handle_Seam=SeamsAllocator::template GetPerFaceAttribute<vcg::Point3<bool> >(mesh,std::string("Seams"));

        SeamsIndexHandleType Handle_SeamIndex;
        if (HasSeamIndex)
        Handle_SeamIndex=SeamsAllocator::template GetPerFaceAttribute<vcg::Point3i >(mesh,std::string("SeamsIndex"));

        glPushAttrib(GL_ALL_ATTRIB_BITS);
        glEnable(GL_COLOR_MATERIAL);
        glDisable(GL_LIGHTING);

        glDepthRange(0,0.999);
        for (unsigned int i=0;i<mesh.face.size();i++)
        {
            if (mesh.face[i].IsD())continue;
            vcg::Point3<bool> seams=Handle_Seam[i];
            vcg::Color4b seamCol[3];
            for (int j=0;j<3;j++)
            {
                seamCol[j]=vcg::Color4b(0,255,0,255);
                if (HasSeamIndex)
                {
                    int index=Handle_SeamIndex[i][j];
                    //assert(index>0);
                    if (index>=0)
                        seamCol[j]=vcg::Color4b(255,0,0,255);
                        //seamCol[j]=vcg::Color4b::Scatter(numCuts,index);
                }
            }

            GLDrawFaceSeams(mesh.face[i],seams,seamCol,size,UV);

        }
        glPopAttrib();
    }

   ///this is useful to debug the output
   ///of the vertex indexing class
   template <class VertexIndexingType>
   static void GLDrawVertexIndexing(VertexIndexingType &VI)
   {
       typedef typename VertexIndexingType::ScalarType ScalarType;

       glPushAttrib(GL_ALL_ATTRIB_BITS);
       glEnable(GL_COLOR_MATERIAL);
       glDisable(GL_LIGHTING);
       glDepthRange(0,0.999);
       typename VertexIndexingType::ScalarType size=5;
       glPointSize(size);
       vcg::glColor(vcg::Color4b(0,255,0,255));
       glBegin(GL_POINTS);
       for (unsigned int i=0;i<VI.duplicated.size();i++)
       {
           assert(!VI.duplicated[i]->IsD());
           vcg::glVertex(VI.duplicated[i]->P());
       }
       glEnd();
       glPopAttrib();
   }


   template <class QuadrangulatorType>
   static void QuadGLDrawIntegerVertices(QuadrangulatorType &Quadr)
   {
       glPushAttrib(GL_ALL_ATTRIB_BITS);
       glDisable(GL_LIGHTING);
       glEnable(GL_COLOR_MATERIAL);
       glDisable(GL_TEXTURE_2D);
       glPointSize(8);

       glDepthRange(0,0.997);
       /*glColor3d(1,0,0);*/
       glBegin(GL_POINTS);
       for (int i=0;i<Quadr.IntegerVertex.size();i++)
       {
           typename QuadrangulatorType::TriVertexType* v=Quadr.IntegerVertex[i];
           typename QuadrangulatorType::CoordType pos=v->P();
           if (v->IsV())
               glColor3d(1,0,0);
           else
               glColor3d(1,1,0);
           glVertex(pos);
       }
       glEnd();

       glPopAttrib();
   }

   template <class QuadrangulatorType>
   static void GLDrawIntegerLines(QuadrangulatorType &Quadr)
   {
       glPushAttrib(GL_ALL_ATTRIB_BITS);
       glDisable(GL_LIGHTING);
       glEnable(GL_COLOR_MATERIAL);
       glDisable(GL_TEXTURE_2D);
       glLineWidth(2);

       glColor3d(0,1,0);
       glDepthRange(0,0.998);

       for (int i=0;i<Quadr.IntegerLines.size();i++)
       {
           typename QuadrangulatorType::TriFaceType *f=Quadr.IntegerLines[i].first;
           int edge=Quadr.IntegerLines[i].second;
           typename QuadrangulatorType::TriVertexType* v0=f->V0(edge);
           typename QuadrangulatorType::TriVertexType* v1=f->V1(edge);
           glBegin(GL_LINES);
           glVertex(v0->P());
           glVertex(v1->P());
           glEnd();
       }

       glPopAttrib();
   }

   template <class QuadrangulatorType>
   static void GLDrawPolygons(QuadrangulatorType &Quadr)
   {
       glPushAttrib(GL_ALL_ATTRIB_BITS);
       glEnable(GL_LIGHTING);
       glEnable(GL_COLOR_MATERIAL);
       glDisable(GL_TEXTURE_2D);
       glColor3d(0.7,0.8,0.9);
       //glFrontFace(GL_CW);
       glDepthRange(0,0.998);
       for (unsigned int i=0;i<Quadr.polygons.size();i++)
       {
           glBegin(GL_POLYGON);
           for (unsigned int j=0;j<Quadr.polygons[i].size();j++)
           {
               typename QuadrangulatorType::TriVertexType* v=Quadr.polygons[i][j];
               glNormal(v->N());
               glVertex(v->P());
           }
           glEnd();
       }

       glDepthRange(0,0.997);
       glDisable(GL_LIGHTING);
       glEnable(GL_COLOR_MATERIAL);
       glColor3d(0,0,0);
       for (unsigned int i=0;i<Quadr.polygons.size();i++)
       {
           glBegin(GL_LINE_LOOP);
           for (unsigned int j=0;j<Quadr.polygons[i].size();j++)
           {
               typename QuadrangulatorType::TriVertexType* v=Quadr.polygons[i][j];
               glVertex(v->P());
           }
           glEnd();
       }

       glPopAttrib();
   }
};
#endif
