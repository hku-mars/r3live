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
Revision 1.1  2006/12/10 19:59:28  ganovelli
first draft of the class the draw a Pos



****************************************************************************/

#ifndef VCG_GL_POS_H
#define VCG_GL_POS_H

// Please note that this file assume that you have already included your 
// gl-extension wrapping utility, and that therefore all the extension symbol are already defined.

#include <vcg/simplex/face/pos.h>
#include <vcg/space/triangle3.h>

namespace vcg {

	template<class PosType>
		struct GlPos{
			typedef typename PosType::ScalarType S;
			static void Draw(PosType & p){
				Point3<S> bc = Barycenter(*(p.f));
				Point3<S> mid = (p.f->P(p.z)+p.f->P(((p.z+1)%3)))*0.5f;
				Point3<S> up = ((bc- p.v->P()) ^ (mid- p.f->P(p.z) )).Normalize();
				Point3<S> ax = (  up  ^ (mid-p.f->P(p.z)) ) .Normalize();
				S proj = (bc-mid)*(ax)*0.5;
				Point3<S> bc1 =  mid+ (ax)*proj;

		
				glBegin(GL_TRIANGLES);
				glVertex(p.v->P());
				if( p.v == p.f->V(p.z))
					{ glVertex(mid); glVertex(bc1);} 
				else
					{glVertex(bc1); glVertex(mid);}
				glEnd();
			}
		};
		
	template<class VfIteType>
		struct GlVfIterator{
			typedef typename VfIteType::ScalarType S;
			static void Draw(VfIteType & v){
				Point3<S>	p = v.F()->P(v.I());// anchor
				Point3<S> c[3];
				for(int i = 0; i < 3;++i)
				{
					c[i] = v.F()->P(i)*0.9 + v.F()->P1(i)*0.05 + v.F()->P2(i)*0.05;
					if(v.F()->VFp(i) != NULL)
					{
						glBegin(GL_LINES);
						glVertex(c[i]);
						glVertex( vcg::Barycenter(*(v.F()->VFp(i))));
						glEnd();
					}
				}
			}
		};

}//namespace
#endif
