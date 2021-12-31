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
Revision 1.10  2006/12/07 00:39:22  cignoni
Added a class prototype for avoiding the inclusion of tetra.h

Revision 1.9  2006/12/04 09:27:13  cignoni
Removed useless include <tetra.h>

****************************************************************************/
#ifndef __PICK______H
#define __PICK______H

#include <vector>
#include <algorithm>
namespace vcg{

template <class MESH_TYPE>
class GLPickTri
{
	typedef typename MESH_TYPE::FaceIterator FaceIterator;
  typedef typename MESH_TYPE::VertexIterator VertexIterator;
  typedef typename MESH_TYPE::FacePointer  FacePointer;
  typedef typename MESH_TYPE::VertexPointer  VertexPointer;
  typedef typename MESH_TYPE::VertexType  VertexType;

public:

	static bool PickNearestFace(int x, int y, MESH_TYPE &m, FacePointer &fi,int width=4, int height=4)
	{
		std::vector<FacePointer> result;
		int val=PickFace(x,y,m,result,width,height);
		if(val!=0)
		{
			fi=result[0];
			return true;
		}
		fi=NULL;
		return false; 
	}
  static int PickVert(int x, int y, MESH_TYPE &m, std::vector<VertexPointer> &result, int width=4, int height=4,bool sorted=true)
  {
      result.clear();
      if(width==0 ||height==0) return 0;
      long hits;
      int sz=m.vert.size()*5;
      GLuint *selectBuf =new GLuint[sz];
      glSelectBuffer(sz, selectBuf);
      glRenderMode(GL_SELECT);
      glInitNames();

      /* Because LoadName() won't work with no names on the stack */
      glPushName(-1);
      double mp[16];

      GLint viewport[4];
      glGetIntegerv(GL_VIEWPORT,viewport);
      glMatrixMode(GL_PROJECTION);
      glGetDoublev(GL_PROJECTION_MATRIX ,mp);
      glPushMatrix();
      glLoadIdentity();
      gluPickMatrix(x, y, width, height, viewport);
      glMultMatrixd(mp);

      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
      int vcnt=0;
      VertexIterator vi;
      for(vi=m.vert.begin();vi!=m.vert.end();++vi)
      {
        if(!(*vi).IsD())
        {
          glLoadName(vcnt);
          glBegin(GL_POINTS);
            glVertex( (*vi).P() );
          glEnd();
        }
        vcnt++; // the counter should advance even for deleted faces!
      }

      glPopMatrix();
      glMatrixMode(GL_PROJECTION);
      glPopMatrix();
      glMatrixMode(GL_MODELVIEW);
      hits = glRenderMode(GL_RENDER);
      std::vector< std::pair<double,unsigned int> > H;
      for(long ii=0;ii<hits;ii++){
        H.push_back( std::pair<double,unsigned int>(selectBuf[ii*4+1]/4294967295.0,selectBuf[ii*4+3]));
      }
      if(sorted)
        std::sort(H.begin(),H.end());
      result.resize(H.size());
      for(long ii=0;ii<hits;ii++){
        VertexIterator vi=m.vert.begin();
        advance(vi ,H[ii].second);
        result[ii]=&*vi;
      }

      delete [] selectBuf;
      return result.size();
  }
  
	static int PickFace(int x, int y, MESH_TYPE &m, std::vector<FacePointer> &result, int width=4, int height=4,bool sorted=true)
	{
		result.clear();
    if(width==0 ||height==0) return 0; 
		long hits;	
		int sz=m.face.size()*5;
		GLuint *selectBuf =new GLuint[sz];
		//  static unsigned int selectBuf[16384];
		glSelectBuffer(sz, selectBuf);
		glRenderMode(GL_SELECT);
		glInitNames();

		/* Because LoadName() won't work with no names on the stack */
		glPushName(-1);
		double mp[16];

		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT,viewport);
		glMatrixMode(GL_PROJECTION);
		glGetDoublev(GL_PROJECTION_MATRIX ,mp);
		glPushMatrix();
		glLoadIdentity();
		//gluPickMatrix(x, viewport[3]-y, 4, 4, viewport);
		gluPickMatrix(x, y, width, height, viewport);
		glMultMatrixd(mp);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		int fcnt=0; 
		FaceIterator fi;
		for(fi=m.face.begin();fi!=m.face.end();++fi)
		{
			if(!(*fi).IsD())
			{
				glLoadName(fcnt);
				glBegin(GL_TRIANGLES);
					glVertex( (*fi).V(0)->P() );
					glVertex( (*fi).V(1)->P() );
					glVertex( (*fi).V(2)->P() );
				glEnd();
			}
      fcnt++; // the counter should advance even for deleted faces!      
		}

		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		hits = glRenderMode(GL_RENDER);
		//xstring buf;
		//if (hits <= 0)     return 0;
		std::vector< std::pair<double,unsigned int> > H;
		for(long ii=0;ii<hits;ii++){
			//TRACE("%ui %ui %ui %ui\n",selectBuf[ii*4],selectBuf[ii*4+1],selectBuf[ii*4+2],selectBuf[ii*4+3]);
			H.push_back( std::pair<double,unsigned int>(selectBuf[ii*4+1]/4294967295.0,selectBuf[ii*4+3]));
		}
		if(sorted) 
      std::sort(H.begin(),H.end());
		//  if(H.size()>0) TRACE("\n Closest is %i\n",H[0].second);
		result.resize(H.size());
		for(long ii=0;ii<hits;ii++){
			FaceIterator fi=m.face.begin();
			advance(fi ,H[ii].second);
			result[ii]=&*fi;
		}

		delete [] selectBuf;
		return result.size();
	}

 // Same of above but it also assumes that you want only visible faces.
 // Visibility is computed according to the current depth buffer. 
	static int PickFaceVisible(int x, int y, MESH_TYPE &m, std::vector<FacePointer> &resultZ, int width=4, int height=4, bool sorted=true)
	{
		// First step 
		
		double mm[16];
		double mp[16];
		GLint vp[4];
		glGetIntegerv(GL_VIEWPORT,vp);
		glGetDoublev(GL_MODELVIEW_MATRIX ,mm);
		glGetDoublev(GL_PROJECTION_MATRIX ,mp);
		int screenW = 		vp[2]-vp[0];
		int screenH = 		vp[3]-vp[1];
		
		GLfloat *buffer = new GLfloat[screenW*screenH];
		glReadPixels(vp[0],vp[1],vp[2],vp[3],GL_DEPTH_COMPONENT,GL_FLOAT,buffer);
		
		std::vector<FacePointer> result;
		PickFace(x,y,m,result,width,height,sorted);
		float LocalEpsilon = 0.001f;
    for(size_t i =0;i<result.size();++i)
		{	
			typename VertexType::CoordType v=Barycenter(*(result[i]));
			GLdouble tx,ty,tz;
			gluProject(v.X(),v.Y(),v.Z(), mm,mp,vp, &tx,&ty,&tz);
			if(tx >=0 && tx<screenW && ty >=0 && ty<screenH)
			{
					float bufZ = buffer[int(tx)+int(ty)*screenW]; 
					//qDebug("face %i txyz (%f %f %f)  bufz %f",i,tx,ty,tz,bufZ);
					if(bufZ + LocalEpsilon >= tz)
								resultZ.push_back(result[i]);
			}
		}
		
		delete [] buffer;
		return resultZ.size();
	}

};


//////////////////////////////////////////////////////////////////////////
template <class TETRA_MESH_TYPE>
class GLPickTetra
{
	typedef typename TETRA_MESH_TYPE::TetraIterator TetraIterator;
	typedef typename TETRA_MESH_TYPE::TetraPointer  TetraPointer;
	typedef typename TETRA_MESH_TYPE::VertexType  VertexType;

public:
static bool PickNearestTetra(int x, int y,TETRA_MESH_TYPE &m, TetraIterator &ti,int width=4, int height=4)
{
 std::vector<TetraPointer> result;
 int val=PickTetra(x,y,m,result,width,height);
 if(val!=0)
 {
  ti=result[0];
	return true;
 }
 ti=0;
 return false; 
}


// class prototype needed for avoid the inclusion of tetra.h
class Tetra;

static int PickTetra(int x, int y, TETRA_MESH_TYPE &m, std::vector<TetraPointer> &result, int width=4, int height=4)
{
	result.clear();
	long hits;	
  int sz=m.tetra.size()*5;
	unsigned int *selectBuf =new unsigned int[sz];
	//  static unsigned int selectBuf[16384];
  glSelectBuffer(sz, selectBuf);
  glRenderMode(GL_SELECT);
  glInitNames();

  /* Because LoadName() won't work with no names on the stack */
  glPushName(-1);
	double mp[16];
	
  int viewport[4];
  glGetIntegerv(GL_VIEWPORT,viewport);
	glMatrixMode(GL_PROJECTION);
	glGetDoublev(GL_PROJECTION_MATRIX ,mp);
	glPushMatrix();
  glLoadIdentity();
  //gluPickMatrix(x, viewport[3]-y, 4, 4, viewport);
  gluPickMatrix(x, y, width, height, viewport);
	glMultMatrixd(mp);

	glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  int tetracnt=0; 
	TetraIterator ti;
	for(ti=m.tetra.begin();ti!=m.tetra.end();++ti)
	{
		if(!(*ti).IsD())
		{
			glLoadName(tetracnt);
			glBegin(GL_TRIANGLES);
			for (int face=0;face<4;face++)
			{
				//glLoadName(tetracnt);
				VertexType *v0=ti->V(Tetra::VofF(face,0));
				VertexType *v1=ti->V(Tetra::VofF(face,1));
				VertexType *v2=ti->V(Tetra::VofF(face,2));
				glVertex(v0->P());
				glVertex(v1->P());
				glVertex(v2->P());
				
			}
			glEnd();
			tetracnt++;
		}
		
	}

  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
	glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  hits = glRenderMode(GL_RENDER);
	//xstring buf;
	//if (hits <= 0)     return 0;
	std::vector< std::pair<double,unsigned int> > H;
	int ii;
	for(ii=0;ii<hits;ii++){
		//TRACE("%ui %ui %ui %ui\n",selectBuf[ii*4],selectBuf[ii*4+1],selectBuf[ii*4+2],selectBuf[ii*4+3]);
		H.push_back( std::pair<double,unsigned int>(selectBuf[ii*4+1]/4294967295.0,selectBuf[ii*4+3]));
	}
	std::sort(H.begin(),H.end());
//  if(H.size()>0) TRACE("\n Closest is %i\n",H[0].second);
  result.resize(H.size());
	for(ii=0;ii<hits;ii++){
		TetraIterator ti=m.tetra.begin();
		advance(ti ,H[ii].second);
		result[ii]=&*ti;
	}
	
	delete [] selectBuf;
  return result.size();
}

static bool PickNearestTetraFace(int x, int y,TETRA_MESH_TYPE &m, TetraIterator &ti,int &face,int width=4, int height=4)
{
 std::vector<std::pair<TetraPointer,int> > result;
 int val=PickTetraFace(x,y,m,result,width,height);
 if(val!=0)
 {
  ti=result[0].first;
  face=result[0].second;
  return true;
 }
 ti=0;
 face=-1;
 return false; 
}

static int PickTetraFace(int x, int y, TETRA_MESH_TYPE &m, std::vector<std::pair<TetraPointer,int> > &result, int width=4, int height=4)
{
	result.clear();
	long hits;	
  int sz=(m.tetra.size()*4)*5;
	unsigned int *selectBuf =new unsigned int[sz];
	//  static unsigned int selectBuf[16384];
  glSelectBuffer(sz, selectBuf);
  glRenderMode(GL_SELECT);
  glInitNames();

  /* Because LoadName() won't work with no names on the stack */
  glPushName(-1);
	double mp[16];
	
  int viewport[4];
  glGetIntegerv(GL_VIEWPORT,viewport);
	glMatrixMode(GL_PROJECTION);
	glGetDoublev(GL_PROJECTION_MATRIX ,mp);
	glPushMatrix();
  glLoadIdentity();
  //gluPickMatrix(x, viewport[3]-y, 4, 4, viewport);
  gluPickMatrix(x, y, width, height, viewport);
	glMultMatrixd(mp);
	glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  int tetracnt=0; 
	TetraIterator ti;
	VertexType *v0;
	VertexType *v1;
	VertexType *v2;
	int face;
	for(ti=m.tetra.begin();ti!=m.tetra.end();++ti)
	{
		if(!(*ti).IsD())
		{
			for (face=0;face<4;face++){
				v0=ti->V(Tetra::VofF(face,0));
				v1=ti->V(Tetra::VofF(face,1));
				v2=ti->V(Tetra::VofF(face,2));
				glLoadName(tetracnt);
				glBegin(GL_TRIANGLES);
					glVertex(v0->P());
					glVertex(v1->P());
					glVertex(v2->P());
				glEnd();
				tetracnt++;
			}
		}
	}

  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
	glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  hits = glRenderMode(GL_RENDER);
	//xstring buf;
	//if (hits <= 0)     return 0;
	std::vector< std::pair<double,unsigned int> > H;
	int ii;
	for(ii=0;ii<hits;ii++){
		//TRACE("%ui %ui %ui %ui\n",selectBuf[ii*4],selectBuf[ii*4+1],selectBuf[ii*4+2],selectBuf[ii*4+3]);
		H.push_back( std::pair<double,unsigned int>(selectBuf[ii*4+1]/4294967295.0,selectBuf[ii*4+3]));
	}
	std::sort(H.begin(),H.end());
//  if(H.size()>0) TRACE("\n Closest is %i\n",H[0].second);
  result.resize(H.size());
	for(ii=0;ii<hits;ii++){
		TetraIterator ti=m.tetra.begin();
		int index=H[ii].second;
		advance(ti ,(int)(index/4));
		result[ii]=std::pair<TetraPointer,int>(&*ti,index%4);
	}
	
	delete [] selectBuf;
  return result.size();
}

};


}

#endif
