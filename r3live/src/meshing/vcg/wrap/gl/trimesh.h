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

#ifndef __VCG_GLTRIMESH
#define __VCG_GLTRIMESH

#include <queue>
#include <vector>

//#include <GL/glew.h>
#include <wrap/gl/space.h>
#include <wrap/gl/math.h>
#include <vcg/space/color4.h>

namespace vcg {


// classe base di glwrap usata solo per poter usare i vari drawmode, normalmode senza dover
// specificare tutto il tipo (a volte lunghissimo)
// della particolare classe glwrap usata.
class GLW
{
public:
    enum DrawMode	{DMNone, DMBox, DMPoints, DMWire, DMHidden, DMFlat, DMSmooth, DMFlatWire, DMRadar, DMLast} ;
    enum NormalMode	{NMNone, NMPerVert, NMPerFace, NMPerWedge, NMLast};
    enum ColorMode	{CMNone, CMPerMesh, CMPerFace, CMPerVert, CMLast};
    enum TextureMode{TMNone, TMPerVert, TMPerWedge, TMPerWedgeMulti};
    enum Hint {
        HNUseTriStrip		  = 0x0001,				// ha bisogno che ci sia la fftopology gia calcolata!
//		HNUseEdgeStrip		  = 0x0002,			//
        HNUseDisplayList	  = 0x0004,
        HNCacheDisplayList	  = 0x0008,		// Each mode has its dl;
        HNLazyDisplayList	  = 0x0010,			// Display list are generated only when requested
        HNIsTwoManifold		  = 0x0020,			// There is no need to make DetachComplex before .
        HNUsePerWedgeNormal	  = 0x0040,		//
        HNHasFFTopology       = 0x0080,		// E' l'utente che si preoccupa di tenere aggiornata la topologia ff
        HNHasVFTopology       = 0x0100,		// E' l'utente che si preoccupa di tenere aggiornata la topologia vf
        HNHasVertNormal       = 0x0200,		// E' l'utente che si preoccupa di tenere aggiornata le normali per faccia
        HNHasFaceNormal       = 0x0400,		// E' l'utente che si preoccupa di tenere aggiornata le normali per vertice
        HNUseVArray           = 0x0800,
        HNUseLazyEdgeStrip	  = 0x1000,		// Edge Strip are generated only when requested
        HNUseVBO              = 0x2000,		// Use Vertex Buffer Object
        HNIsPolygonal         = 0x4000    // In wireframe modes, hide faux edges
    };

    enum Change {
        CHVertex		= 0x01,
        CHNormal		= 0x02,
        CHColor			= 0x04,
        CHFace			= 0x08,
        CHFaceNormal	= 0x10,
        CHRender        = 0x20,
        CHAll			= 0xff
    };
    enum HintParami {
    HNPDisplayListSize =0,
    HNPPointDistanceAttenuation =1,
    HNPPointSmooth = 2
    };
    enum HintParamf {
        HNPCreaseAngle =0,	// crease angle in radians
        HNPZTwist = 1,				// Z offset used in Flatwire and hiddenline modality
        HNPPointSize = 2		// the point size used in point rendering
    };

    template<class MESH_TYPE>
    class VertToSplit
    {
    public:
        typename MESH_TYPE::face_base_pointer f;
        char z;
        char edge;
        bool newp;
        typename MESH_TYPE::vertex_pointer v;
    };

    // GL Array Elemet
    class GLAElem {
    public :
        int glmode;
        int len;
        int start;
    };


};

template <class MESH_TYPE,  bool partial = false , class FACE_POINTER_CONTAINER = std::vector<typename MESH_TYPE::FacePointer> >
class GlTrimesh : public GLW
{
public:

    typedef MESH_TYPE mesh_type;
    FACE_POINTER_CONTAINER face_pointers;


    std::vector<unsigned int> TMId;
    unsigned int array_buffers[3];

    int curr_hints;      // the current hints

    // The parameters of hints
    int   HNParami[8];
    float HNParamf[8];

    MESH_TYPE *m;
    GlTrimesh()
    {
        m=0;
        dl=0xffffffff;
        curr_hints=HNUseLazyEdgeStrip;
        cdm=DMNone;
        ccm=CMNone;
        cnm=NMNone;

        SetHintParamf(HNPCreaseAngle,float(M_PI/5));
        SetHintParamf(HNPZTwist,0.00005f);
        SetHintParamf(HNPPointSize,1.0f);
    SetHintParami(HNPPointDistanceAttenuation, 1);
    SetHintParami(HNPPointSmooth, 0);
  }

    ~GlTrimesh()
    {
        //Delete the VBOs
        if(curr_hints&HNUseVBO)
        {
            for(int i=0;i<3;++i)
                if(glIsBuffer(GLuint(array_buffers[i])))
                    glDeleteBuffersARB(1, (GLuint *)(array_buffers+i));
        }
    }

    void SetHintParami(const HintParami hip, const int value)
    {
        HNParami[hip]=value;
    }
    int GetHintParami(const HintParami hip) const
    {
        return HNParami[hip];
    }
    void SetHintParamf(const HintParamf hip, const float value)
    {
        HNParamf[hip]=value;
    }
    float GetHintParamf(const HintParamf hip) const
    {
        return HNParamf[hip];
    }
    void SetHint(Hint hn)
    {
        curr_hints |= hn;
    }
    void ClearHint(Hint hn)
    {
        curr_hints&=(~hn);
    }

    unsigned int dl;
    std::vector<unsigned int> indices;

    DrawMode cdm; // Current DrawMode
    NormalMode cnm; // Current NormalMode
    ColorMode ccm; // Current ColorMode

void Update(/*Change c=CHAll*/)
{
    if(m==0) return;

    if(curr_hints&HNUseVArray || curr_hints&HNUseVBO)
    {
        typename MESH_TYPE::FaceIterator fi;
        indices.clear();
        for(fi = m->face.begin(); fi != m->face.end(); ++fi)
        {
            indices.push_back((unsigned int)((*fi).V(0) - &(*m->vert.begin())));
            indices.push_back((unsigned int)((*fi).V(1) - &(*m->vert.begin())));
            indices.push_back((unsigned int)((*fi).V(2) - &(*m->vert.begin())));
        }

        if(curr_hints&HNUseVBO)
        {
            if(!glIsBuffer(array_buffers[1]))
                glGenBuffers(2,(GLuint*)array_buffers);
            glBindBuffer(GL_ARRAY_BUFFER,array_buffers[0]);
            glBufferData(GL_ARRAY_BUFFER_ARB, m->vn * sizeof(typename MESH_TYPE::VertexType),
                (char *)&(m->vert[0].P()), GL_STATIC_DRAW_ARB);

            glBindBuffer(GL_ARRAY_BUFFER,array_buffers[1]);
            glBufferData(GL_ARRAY_BUFFER_ARB, m->vn * sizeof(typename MESH_TYPE::VertexType),
                (char *)&(m->vert[0].N()), GL_STATIC_DRAW_ARB);
        }

        glVertexPointer(3,GL_FLOAT,sizeof(typename MESH_TYPE::VertexType),0);
        glNormalPointer(GL_FLOAT,sizeof(typename MESH_TYPE::VertexType),0);
    }

    //int C=c;
    //if((C&CHVertex) || (C&CHFace)) {
    //	ComputeBBox(*m);
    //	if(!(curr_hints&HNHasFaceNormal)) m->ComputeFaceNormal();
    //	if(!(curr_hints&HNHasVertNormal)) m->ComputeVertexNormal();
    //	C= (C | CHFaceNormal);
    //}
    //if((C&CHFace) && (curr_hints&HNUseEdgeStrip)) 		 ComputeEdges();
    //if((C&CHFace) && (curr_hints&HNUseLazyEdgeStrip)) ClearEdges();
    //if(MESH_TYPE::HasFFTopology())
    //	if((C&CHFace) && (curr_hints&HNUseTriStrip)) 		{
    //			if(!(curr_hints&HNHasFFTopology)) m->FFTopology();
    //			ComputeTriStrip();
    //		}
    //if((C&CHFaceNormal) && (curr_hints&HNUsePerWedgeNormal))		{
    //	  if(!(curr_hints&HNHasVFTopology)) m->VFTopology();
    //		CreaseWN(*m,MESH_TYPE::scalar_type(GetHintParamf(HNPCreaseAngle)));
    //}
    //if(C!=0) { // force the recomputation of display list
    //	cdm=DMNone;
    //	ccm=CMNone;
    //	cnm=NMNone;
    //}
    //if((curr_hints&HNUseVArray) && (curr_hints&HNUseTriStrip))
    //	{
    //	 ConvertTriStrip<MESH_TYPE>(*m,TStrip,TStripF,TStripVED,TStripVEI);
    //	}
}

void Draw(DrawMode dm ,ColorMode cm, TextureMode tm)
{
    switch(dm)
    {
        case	DMNone    : Draw<DMNone    >(cm,tm); break;
        case	DMBox     : Draw<DMBox     >(cm,tm); break;
        case	DMPoints  : Draw<DMPoints  >(cm,tm); break;
        case	DMWire    : Draw<DMWire    >(cm,tm); break;
        case	DMHidden  : Draw<DMHidden  >(cm,tm); break;
        case	DMFlat    : Draw<DMFlat    >(cm,tm); break;
        case	DMSmooth  : Draw<DMSmooth  >(cm,tm); break;
        case	DMFlatWire: Draw<DMFlatWire>(cm,tm); break;
        default : break;
    }
}

template< DrawMode dm >
void Draw(ColorMode cm, TextureMode tm)
{
    switch(cm)
    {
        case	CMNone    : Draw<dm,CMNone   >(tm); break;
        case	CMPerMesh : Draw<dm,CMPerMesh>(tm); break;
        case	CMPerFace : Draw<dm,CMPerFace>(tm); break;
        case	CMPerVert : Draw<dm,CMPerVert>(tm); break;
        default : break;
    }
}

template< DrawMode dm, ColorMode cm >
void Draw(TextureMode tm)
{
    switch(tm)
    {
        case	TMNone          : Draw<dm,cm,TMNone          >(); break;
        case	TMPerVert       : Draw<dm,cm,TMPerVert       >(); break;
        case	TMPerWedge      : Draw<dm,cm,TMPerWedge      >(); break;
        case	TMPerWedgeMulti : Draw<dm,cm,TMPerWedgeMulti >(); break;
        default : break;
    }
}



template< DrawMode dm, ColorMode cm, TextureMode tm>
void Draw()
{
    if(!m) return;
    if((curr_hints & HNUseDisplayList)){
                if (cdm==dm && ccm==cm){
                        glCallList(dl);
                        return;
                }
                else {
                    if(dl==0xffffffff) dl=glGenLists(1);
                    glNewList(dl,GL_COMPILE);
                }
    }

    glPushMatrix();
    switch(dm)
        {
            case DMNone		  : break;
            case DMBox		  : DrawBBox(cm);break;
            case DMPoints   : DrawPoints<NMPerVert,cm>();break;
            case DMHidden		:	DrawHidden();break;
            case DMFlat			:	DrawFill<NMPerFace,cm,tm>();break;
            case DMFlatWire :	DrawFlatWire<NMPerFace,cm,tm>();break;
            case DMRadar		:	DrawRadar<NMPerFace,cm>();break;
            case DMWire		  :	DrawWire<NMPerVert,cm>();break;
            case DMSmooth   : DrawFill<NMPerVert,cm,tm>();break;
            default : break;
        }
    glPopMatrix();

    if((curr_hints & HNUseDisplayList)){
        cdm=dm;
        ccm=cm;
        glEndList();
        glCallList(dl);
    }
}


/*********************************************************************************************/
/*********************************************************************************************/


template <NormalMode nm, ColorMode cm, TextureMode tm>
void DrawFill()
{
  if(m->fn==0) return;
    typename FACE_POINTER_CONTAINER::iterator fp;

    typename MESH_TYPE::FaceIterator fi;

    typename std::vector<typename MESH_TYPE::FaceType*>::iterator fip;

    if(cm == CMPerMesh)
        glColor(m->C());

    if(tm == TMPerWedge || tm == TMPerWedgeMulti )
        glDisable(GL_TEXTURE_2D);

    if(curr_hints&HNUseVBO)
    {
        if( (cm==CMNone) || (cm==CMPerMesh) )
        {
            if (nm==NMPerVert)
                glEnableClientState (GL_NORMAL_ARRAY);
            glEnableClientState (GL_VERTEX_ARRAY);

            if (nm==NMPerVert)
            {
                glBindBuffer(GL_ARRAY_BUFFER,array_buffers[1]);
                glNormalPointer(GL_FLOAT,sizeof(typename MESH_TYPE::VertexType),0);
            }
            glBindBuffer(GL_ARRAY_BUFFER,array_buffers[0]);
            glVertexPointer(3,GL_FLOAT,sizeof(typename MESH_TYPE::VertexType),0);

            glDrawElements(GL_TRIANGLES ,m->fn*3,GL_UNSIGNED_INT, &(*indices.begin()) );
            glDisableClientState (GL_VERTEX_ARRAY);
            if (nm==NMPerVert)
                glDisableClientState (GL_NORMAL_ARRAY);

            glBindBuffer(GL_ARRAY_BUFFER, 0);

            return;

        }
    }

    if(curr_hints&HNUseVArray)
    {
        if( (cm==CMNone) || (cm==CMPerMesh) )
        {
            if (nm==NMPerVert)
                glEnableClientState (GL_NORMAL_ARRAY);
            glEnableClientState (GL_VERTEX_ARRAY);

            if (nm==NMPerVert)
                glNormalPointer(GL_FLOAT,sizeof(typename MESH_TYPE::VertexType),&(m->vert.begin()->N()[0]));
            glVertexPointer(3,GL_FLOAT,sizeof(typename MESH_TYPE::VertexType),&(m->vert.begin()->P()[0]));

            glDrawElements(GL_TRIANGLES ,m->fn*3,GL_UNSIGNED_INT, &(*indices.begin()) );
            glDisableClientState (GL_VERTEX_ARRAY);
            if (nm==NMPerVert)
                glDisableClientState (GL_NORMAL_ARRAY);

            return;
        }
    }
    else

    if(curr_hints&HNUseTriStrip)
    {
        //if( (nm==NMPerVert) && ((cm==CMNone) || (cm==CMPerMesh)))
        //	if(curr_hints&HNUseVArray){
        //		glEnableClientState (GL_NORMAL_ARRAY  );
        //		glNormalPointer(GL_FLOAT,sizeof(MESH_TYPE::VertexType),&(m->vert[0].cN()));
        //		glEnableClientState (GL_VERTEX_ARRAY);
        //		glVertexPointer(3,GL_FLOAT,sizeof(MESH_TYPE::VertexType),&(m->vert[0].cP()));
        //		std::vector<GLAElem>::iterator vi;
        //		for(vi=TStripVED.begin();vi!=TStripVED.end();++vi)
        //					glDrawElements(vi->glmode ,vi->len,GL_UNSIGNED_SHORT,&TStripVEI[vi->start] );
        //
        //		glDisableClientState (GL_NORMAL_ARRAY  );
        //		glDisableClientState (GL_VERTEX_ARRAY);
        //		return;
        //	}

        //std::vector< MESH_TYPE::VertexType *>::iterator vi;
        //glBegin(GL_TRIANGLE_STRIP);
        //if(nm == NMPerFace) fip=TStripF.begin();

        //for(vi=TStrip.begin();vi!=TStrip.end(); ++vi){
        //	if((*vi)){
        //		if(nm==NMPerVert) glNormal((*vi)->cN());
        //		if(nm==NMPerFace) glNormal((*fip)->cN());
        //		glVertex((*vi)->P());
        //		}
        //	else
        //		{
        //			glEnd();
        //			glBegin(GL_TRIANGLE_STRIP);
        //		}
        //	if(nm == NMPerFace) ++fip;
        //	}
        //glEnd();
    }
    else
    {
      short curtexname=-1;
        if(partial)
            fp = face_pointers.begin();
        else
            fi = m->face.begin();

        if(tm==TMPerWedgeMulti)
        {
            curtexname=(*fi).WT(0).n();
            if ((curtexname >= 0) && (curtexname < (int)TMId.size()))
            {
                glEnable(GL_TEXTURE_2D);
                glBindTexture(GL_TEXTURE_2D,TMId[curtexname]);
            }
            else
            {
                glDisable(GL_TEXTURE_2D);
            }
        }

        if(tm==TMPerWedge)
            glEnable(GL_TEXTURE_2D);

    if(tm==TMPerVert && !TMId.empty()) // in the case of per vertex tex coord we assume that we have a SINGLE texture.
    {
        curtexname = 0;
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D,TMId[curtexname]);
    }
        glBegin(GL_TRIANGLES);

        while( (partial)?(fp!=face_pointers.end()):(fi!=m->face.end()))
        {
            typename MESH_TYPE::FaceType & f = (partial)?(*(*fp)): *fi;

            if(!f.IsD())
            {
                if(tm==TMPerWedgeMulti)
                if(f.WT(0).n() != curtexname)
                {
                    curtexname=(*fi).WT(0).n();
                    glEnd();

                    if (curtexname >= 0)
                    {
                        glEnable(GL_TEXTURE_2D);
                        if(!TMId.empty())
                          glBindTexture(GL_TEXTURE_2D,TMId[curtexname]);
                    }
                    else
                    {
                        glDisable(GL_TEXTURE_2D);
                    }

                    glBegin(GL_TRIANGLES);
                }

                if(nm == NMPerFace)	glNormal(f.cN());
                if(nm == NMPerVert)	glNormal(f.V(0)->cN());
                if(nm == NMPerWedge)glNormal(f.WN(0));

                if(cm == CMPerFace)	glColor(f.C());
                if(cm == CMPerVert)	glColor(f.V(0)->C());
                if(tm==TMPerVert) glTexCoord(f.V(0)->T().P());
                if( (tm==TMPerWedge)||(tm==TMPerWedgeMulti) )glTexCoord(f.WT(0).t(0));
                glVertex(f.V(0)->P());

                if(nm == NMPerVert)	glNormal(f.V(1)->cN());
                if(nm == NMPerWedge)glNormal(f.WN(1));
                if(cm == CMPerVert)	glColor(f.V(1)->C());
                if(tm==TMPerVert) glTexCoord(f.V(1)->T().P());
                if( (tm==TMPerWedge)|| (tm==TMPerWedgeMulti)) glTexCoord(f.WT(1).t(0));
                glVertex(f.V(1)->P());

                if(nm == NMPerVert)	glNormal(f.V(2)->cN());
                if(nm == NMPerWedge)glNormal(f.WN(2));
                if(cm == CMPerVert) glColor(f.V(2)->C());
                if(tm==TMPerVert) glTexCoord(f.V(2)->T().P());
                if( (tm==TMPerWedge)|| (tm==TMPerWedgeMulti)) glTexCoord(f.WT(2).t(0));
                glVertex(f.V(2)->P());
            }

            if(partial)
                ++fp;
            else
                ++fi;
        }

        glEnd();

    }
}

// A draw wireframe that hides faux edges
template <NormalMode nm, ColorMode cm>
void DrawWirePolygonal()
{

    typename MESH_TYPE::FaceIterator fi;


  typename FACE_POINTER_CONTAINER::iterator fp;

    typename std::vector<typename MESH_TYPE::FaceType*>::iterator fip;

    if(cm == CMPerMesh)
        glColor(m->C());

    {
        if(partial)
            fp = face_pointers.begin();
        else
            fi = m->face.begin();

        glBegin(GL_LINES);

        while( (partial)?(fp!=face_pointers.end()):(fi!=m->face.end()))
        {
            typename MESH_TYPE::FaceType & f = (partial)?(*(*fp)): *fi;

            if(!f.IsD())
            {

                if(nm == NMPerFace)	glNormal(f.cN());
                if(cm == CMPerFace)	glColor(f.C());

                if (!f.IsF(0)) {
                  if(nm == NMPerVert)	glNormal(f.V(0)->cN());
          if(nm == NMPerWedge)glNormal(f.WN(0));
                  if(cm == CMPerVert)	glColor(f.V(0)->C());
                  glVertex(f.V(0)->P());

                  if(nm == NMPerVert)	glNormal(f.V(1)->cN());
                  if(nm == NMPerWedge)glNormal(f.WN(1));
                  if(cm == CMPerVert)	glColor(f.V(1)->C());
                  glVertex(f.V(1)->P());
        }

                if (!f.IsF(1)) {
                  if(nm == NMPerVert)	glNormal(f.V(1)->cN());
          if(nm == NMPerWedge)glNormal(f.WN(1));
                  if(cm == CMPerVert)	glColor(f.V(1)->C());
                  glVertex(f.V(1)->P());

                  if(nm == NMPerVert)	glNormal(f.V(2)->cN());
                  if(nm == NMPerWedge)glNormal(f.WN(2));
                  if(cm == CMPerVert)	glColor(f.V(2)->C());
                  glVertex(f.V(2)->P());
        }

                if (!f.IsF(2)) {
                  if(nm == NMPerVert)	glNormal(f.V(2)->cN());
          if(nm == NMPerWedge)glNormal(f.WN(2));
                  if(cm == CMPerVert)	glColor(f.V(2)->C());
                  glVertex(f.V(2)->P());

                  if(nm == NMPerVert)	glNormal(f.V(0)->cN());
                  if(nm == NMPerWedge)glNormal(f.WN(0));
                  if(cm == CMPerVert)	glColor(f.V(0)->C());
                  glVertex(f.V(0)->P());
        }

            }

            if(partial)
                ++fp;
            else
                ++fi;
        }

        glEnd();

    }
}

/// Basic Point drawing fucntion
// works also for mesh with deleted vertices
template<NormalMode nm, ColorMode cm>
void DrawPointsBase()
{
    typename MESH_TYPE::VertexIterator vi;
    glBegin(GL_POINTS);
    if(cm==CMPerMesh) glColor(m->C());

    for(vi=m->vert.begin();vi!=m->vert.end();++vi)if(!(*vi).IsD())
    {
            if(nm==NMPerVert) glNormal((*vi).cN());
            if(cm==CMPerVert) glColor((*vi).C());
            glVertex((*vi).P());
    }
    glEnd();
}

/// Utility function that computes in eyespace the current distance between the camera and the center of the bbox of the mesh
double CameraDistance(){
    Point3<typename MESH_TYPE::ScalarType> res;
    Matrix44<typename MESH_TYPE::ScalarType> mm;
    glGetv(GL_MODELVIEW_MATRIX,mm);
    Point3<typename MESH_TYPE::ScalarType>  c=m->bbox.Center();
    res=mm*c;
    return Norm(res);
}
template<NormalMode nm, ColorMode cm>
void DrawPoints()
{
  glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT);
  if(GetHintParami(HNPPointSmooth)>0) glEnable(GL_POINT_SMOOTH);
  else glDisable(GL_POINT_SMOOTH);
  glPointSize(GetHintParamf(HNPPointSize));
  if(GetHintParami(HNPPointDistanceAttenuation)>0)
    {
      float camDist = (float)CameraDistance();
      float quadratic[] = { 0.0f, 0.0f, 1.0f/(camDist*camDist) , 0.0f };
      glPointParameterfv( GL_POINT_DISTANCE_ATTENUATION, quadratic );
      glPointParameterf( GL_POINT_SIZE_MAX, 16.0f );
      glPointParameterf( GL_POINT_SIZE_MIN, 1.0f );
    }
    else
    {
      float quadratic[] = { 1.0f, 0.0f, 0.0f};
      glPointParameterfv( GL_POINT_DISTANCE_ATTENUATION, quadratic );
      glPointSize(GetHintParamf(HNPPointSize));
    }

    if(m->vn!=(int)m->vert.size())
        {
            DrawPointsBase<nm,cm>();
        }
  else
  {
    if(cm==CMPerMesh)
      glColor(m->C());

    // Perfect case, no deleted stuff,
    // draw the vertices using vertex arrays
    if (nm==NMPerVert)
      {
        glEnableClientState (GL_NORMAL_ARRAY);
        if (m->vert.size() != 0)
            glNormalPointer(GL_FLOAT,sizeof(typename MESH_TYPE::VertexType),&(m->vert.begin()->N()[0]));
      }
    if (cm==CMPerVert)
      {
        glEnableClientState (GL_COLOR_ARRAY);
        if (m->vert.size() != 0)
            glColorPointer(4,GL_UNSIGNED_BYTE,sizeof(typename MESH_TYPE::VertexType),&(m->vert.begin()->C()[0]));
      }

    glEnableClientState (GL_VERTEX_ARRAY);
    if (m->vert.size() != 0)
        glVertexPointer(3,GL_FLOAT,sizeof(typename MESH_TYPE::VertexType),&(m->vert.begin()->P()[0]));

    glDrawArrays(GL_POINTS,0,m->vn);

    glDisableClientState (GL_VERTEX_ARRAY);
    if (nm==NMPerVert)  glDisableClientState (GL_NORMAL_ARRAY);
    if (cm==CMPerVert)  glDisableClientState (GL_COLOR_ARRAY);
  }
  glPopAttrib();
    return;
}

void DrawHidden()
{
    //const float ZTWIST=HNParamf[HNPZTwist];
  glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT | GL_LIGHTING_BIT );
    glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1.0, 1);
    //glDepthRange(ZTWIST,1.0f);
    glDisable(GL_LIGHTING);
    glColorMask(GL_FALSE,GL_FALSE,GL_FALSE,GL_FALSE);
    DrawFill<NMNone,CMNone,TMNone>();
    glDisable(GL_POLYGON_OFFSET_FILL);
    glEnable(GL_LIGHTING);
    glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
    //glDepthRange(0.0f,1.0f-ZTWIST);
    DrawWire<NMPerVert,CMNone>();
    glPopAttrib();
//	glDepthRange(0,1.0f);
}

template <NormalMode nm, ColorMode cm, TextureMode tm>
void DrawFlatWire()
{
    //const float ZTWIST=HNParamf[HNPZTwist];
    //glDepthRange(ZTWIST,1.0f);
  glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT | GL_LIGHTING_BIT );
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1.0, 1);
    DrawFill<nm,cm,tm>();
  glDisable(GL_POLYGON_OFFSET_FILL);
    //glDepthRange(0.0f,1.0f-ZTWIST);
  glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
    //glColorMaterial(GL_FRONT,GL_DIFFUSE);
  glColor3f(.3f,.3f,.3f);
    DrawWire<nm,CMNone>();
    glPopAttrib();
    //glDepthRange(0,1.0f);
}

template <NormalMode nm, ColorMode cm>
void DrawRadar()
{
        const float ZTWIST=HNParamf[HNPZTwist];
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(0);
    glDepthRange(ZTWIST,1.0f);

    if (cm == CMNone)
        glColor4f(0.2f, 1.0f, 0.4f, 0.2f);
//	DrawFill<nm,cm,TMNone>();
    Draw<DMFlat,CMNone,TMNone>();

    glDepthMask(1);
    glColorMask(GL_FALSE,GL_FALSE,GL_FALSE,GL_FALSE);
//	DrawFill<nm,cm,TMNone>();
    Draw<DMFlat,CMNone,TMNone>();

    glDepthRange(0.0f,1.0f-ZTWIST);
    glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
    glColor4f(0.1f, 1.0f, 0.2f, 0.6f);
    Draw<DMWire,CMNone,TMNone>();
    glDisable(GL_BLEND);
    glDepthRange(0,1.0f);

}



#ifdef GL_TEXTURE0_ARB
// Multitexturing nel caso voglia usare due texture unit.
void DrawTexture_NPV_TPW2()
{
    unsigned int texname=(*(m->face.begin())).WT(0).n(0);
    glBindTexture(GL_TEXTURE_2D,TMId[texname]);
    typename MESH_TYPE::FaceIterator fi;
    glBegin(GL_TRIANGLES);
    for(fi=m->face.begin();fi!=m->face.end();++fi)if(!(*fi).IsD()){
          if(texname!=(*fi).WT(0).n(0))	{
                texname=(*fi).WT(0).n(0);
                glEnd();
                glBindTexture(GL_TEXTURE_2D,TMId[texname]);
                glBegin(GL_TRIANGLES);
            }
            glMultiTexCoordARB(GL_TEXTURE0_ARB, (*fi).WT(0).t(0));
            glMultiTexCoordARB(GL_TEXTURE1_ARB, (*fi).WT(0).t(0));
            glNormal((*fi).V(0)->N());
            glVertex((*fi).V(0)->P());

            glMultiTexCoordARB(GL_TEXTURE0_ARB, (*fi).WT(1).t(0));
            glMultiTexCoordARB(GL_TEXTURE1_ARB, (*fi).WT(1).t(0));
            glNormal((*fi).V(1)->N());
            glVertex((*fi).V(1)->P());

            glMultiTexCoordARB(GL_TEXTURE0_ARB, (*fi).WT(2).t(0));
            glMultiTexCoordARB(GL_TEXTURE1_ARB, (*fi).WT(2).t(0));
            glNormal((*fi).V(2)->N());
            glVertex((*fi).V(2)->P());
    }
    glEnd();
}

#endif


/*int MemUsed()
{
    int tot=sizeof(GlTrimesh);
    tot+=sizeof(mesh_type::edge_type)*edge.size();
    tot+=sizeof(MESH_TYPE::VertexType *) * EStrip.size();
    tot+=sizeof(MESH_TYPE::VertexType *) * TStrip.size();
    tot+=sizeof(MESH_TYPE::FaceType *)   * TStripF.size();
    return tot;
}*/

private:

template <NormalMode nm, ColorMode cm>
void DrawWire()
{
    //if(!(curr_hints & (HNUseEdgeStrip | HNUseLazyEdgeStrip) ) )
  if ( (curr_hints & HNIsPolygonal) )
      {
      DrawWirePolygonal<nm,cm>();
    }
    else
    {
            glPushAttrib(GL_POLYGON_BIT);
            glPolygonMode(GL_FRONT_AND_BACK ,GL_LINE);
            DrawFill<nm,cm,TMNone>();
            glPopAttrib();
    }
  if(m->fn==0 && m->en>0)
  {
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    for(typename mesh_type::EdgeIterator ei=m->edge.begin();ei!=m->edge.end(); ++ei)
    {
      glVertex((*ei).V(0)->P());
      glVertex((*ei).V(1)->P());
    }
    glEnd();
    glPopAttrib();
  }
    //	{
//			if(!HasEdges()) ComputeEdges();

            //if(cm==CMPerMesh)	glColor(m->C());
            //std::vector< MESH_TYPE::VertexType *>::iterator vi;
            //glBegin(GL_LINE_STRIP);
            //for(vi=EStrip.begin();vi!=EStrip.end(); ++vi){
            //	if((*vi)){
            //			glNormal((*vi)->N());
            //			glVertex((*vi)->P());
            //		}
            //	else
            //		{
            //			glEnd();
            //			glBegin(GL_LINE_STRIP);
            //		}
            //}
            //glEnd();
    //	}
}

void DrawBBox(ColorMode cm)
{
    if(cm==CMPerMesh) glColor(m->C());
    glBoxWire(m->bbox);
}


};// end class

} // end namespace

 #endif
