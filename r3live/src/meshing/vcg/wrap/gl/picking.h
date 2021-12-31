#pragma once
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
This file contains two function providing the standard way to do picking using opendgl:
- using the SELECT mode (first function)
- using the depth buffer and gluUnproject (second function)

  History
$Log: not supported by cvs2svn $
Revision 1.5  2007/05/21 13:22:40  cignoni
Corrected gcc compiling issues

Revision 1.4  2006/10/27 08:55:15  fiorin
Added type cast (in order to remove warnings)

Revision 1.3  2006/02/28 13:25:48  ponchio
for(ii... -> for(int ii

Revision 1.2  2006/02/13 13:06:34  cignoni
Removed glut. Added ifdef guards and namespace.
Added bool return value to the pick function

Revision 1.1  2005/12/03 09:36:28  ganovelli
*** empty log message ***

****************************************************************************/

#ifndef WRAP_GL_PICKING_H
#define WRAP_GL_PICKING_H

#include <algorithm>

#ifndef GLU_VERSIONS
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#ifdef _WIN32
  #include <windows.h>
#endif
#include <GL/glu.h>
#endif
#endif

namespace vcg
{

template <class TO_PICK_CONT_TYPE>
int Pick(	const int & x, const int &y,
                    TO_PICK_CONT_TYPE &m,
                    std::vector<typename TO_PICK_CONT_TYPE::value_type*> &result,
                    void (draw_func)(typename TO_PICK_CONT_TYPE::value_type &),
                    int width=4,
                    int height=4)
    {
        result.clear();
        long hits;
        int sz = int(m.size())*5;
        GLuint *selectBuf =new GLuint[sz];
        glSelectBuffer(sz, selectBuf);
        glRenderMode(GL_SELECT);
        glInitNames();

        /* Because LoadName() won't work with no names on the stack */
        glPushName(-1);
        double mp[16];

        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT,viewport);
        glPushAttrib(GL_TRANSFORM_BIT);
        glMatrixMode(GL_PROJECTION);
        glGetDoublev(GL_PROJECTION_MATRIX ,mp);
        glPushMatrix();
        glLoadIdentity();
        //gluPickMatrix(x, viewport[3]-y, 4, 4, viewport);
        gluPickMatrix(x, y, width, height, viewport);
        glMultMatrixd(mp);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        int cnt=0;
        typename TO_PICK_CONT_TYPE::iterator  ei;
        for(ei=m.begin();ei!=m.end();++ei)
        {

                glLoadName(cnt);
                draw_func(*ei);
                cnt++;
        }

        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
        hits = glRenderMode(GL_RENDER);

        if (hits <= 0)     return 0;
        std::vector< std::pair<double,unsigned int> > H;
        for(int ii=0;ii<hits;ii++){
            H.push_back( std::pair<double,unsigned int>(selectBuf[ii*4+1]/4294967295.0,selectBuf[ii*4+3]));
        }
        std::sort(H.begin(),H.end());

        result.resize(H.size());
        for(int ii=0;ii<hits;ii++){
            typename TO_PICK_CONT_TYPE::iterator ei=m.begin();
            std::advance(ei ,H[ii].second);
            result[ii]=&*ei;
        }
        glPopAttrib();
        delete [] selectBuf;
        return int(result.size());
    }

// 10/2/06 Slightly changed the interface.
// Return value is used to determine if the picked point was against the far plane
// (and therefore nothing was picked)
template <class PointType>
bool Pick(const int & x, const int &y, PointType &pp){
    GLdouble res[3];
    GLdouble mm[16],pm[16]; GLint vp[4];
    glGetDoublev(GL_MODELVIEW_MATRIX,mm);
    glGetDoublev(GL_PROJECTION_MATRIX,pm);
    glGetIntegerv(GL_VIEWPORT,vp);

    GLfloat   pix;
    glReadPixels(x,y,1,1,GL_DEPTH_COMPONENT,GL_FLOAT,&pix);
  GLfloat depthrange[2]={0,0};
  glGetFloatv(GL_DEPTH_RANGE,depthrange);
  if(pix==depthrange[1]) return false;
    gluUnProject(x,y,pix,mm,pm,vp,&res[0],&res[1],&res[2]);
    pp=PointType (res[0],res[1],res[2]);
  return true;
    }

} // end namespace

#endif
