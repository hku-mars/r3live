/****************************************************************************
* MeshLab                                                           o o     *
* An extendible mesh processor                                    o     o   *
*                                                                _   O  _   *
* Copyright(C) 2005, 2009                                          \/)\/    *
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

#ifndef __VCGLIB_IMPORTERTS
#define __VCGLIB_IMPORTERTS
#define NULL 0
#include <vcg/space/point3.h>
#include <vcg/space/point4.h>

namespace vcg {
namespace tetra {
namespace io {

template <typename  MESHTYPE>
class ImporterTS{
	typedef MESHTYPE Tetramesh;
	typedef typename Tetramesh::VertexPointer VertexPointer;
	typedef typename Tetramesh::VertexType VertexType;
	typedef typename Tetramesh::TetraType FaceType;
	typedef typename Tetramesh::VertexIterator VertexIterator;
	typedef typename Tetramesh::TetraIterator FaceIterator;
	typedef typename Tetramesh::ScalarType ScalarType;
	typedef Point3<ScalarType> Point3x;

	static FILE *& F(){static FILE * f; return f;}

	inline static void ReadPos( Point3<double> &p){
		fscanf(F(),"%lg %lg %lg",&p[0],&p[1],&p[2]);
	}
	inline static void  ReadPos( Point3<float> &p){
		fscanf(F(),"%f %f %f",&p[0],&p[1],&p[2]);
	}
	inline static void  ReadPos( Point4<ScalarType> &p){
		fscanf(F(),"%g %g %g %g",&p[0],&p[1],&p[2],&p[3]);
	}
public:
static int Open( Tetramesh & m, const char * filename )
{	
	int nvertex;
	int ntetra;
	int tp0;
	int tp1;
	int tp2;
	int tp3;
	typename Tetramesh::VertexType p1;
	F() = fopen(filename,"r");
	if(F() == NULL ) 
		{
			printf( "The file was not opened\n" );
			return -1;
		}
   else
   {
		fscanf(F(), "%i", &nvertex );
		fscanf(F(), "%i", &ntetra );
		m.tetra.reserve(ntetra);
    m.vert.reserve(nvertex);
		int j;
		for (j=0;j<nvertex;j++)
		{
			m.vert.push_back(VertexType());
			ReadPos(m.vert.back().P());
      m.vert.back().ClearFlags();
		}
		for (j=0;j<ntetra;j++)
		{
			fscanf(F(), "%i", &tp0 );
			fscanf(F(), "%i", &tp1 );
			fscanf(F(), "%i", &tp2 );
			fscanf(F(), "%i", &tp3 );
			
			m.tetra.push_back(typename Tetramesh::TetraType());
			m.tetra.back().V(0) = &m.vert[tp0];
			m.tetra.back().V(1) = &m.vert[tp1];
			m.tetra.back().V(2) = &m.vert[tp2];
			m.tetra.back().V(3) = &m.vert[tp3];
			m.tetra.back().Flags() = 0;
		}
	 }
	 m.vn = nvertex;
	 m.tn = ntetra;

		return 0;
	 }
	};// end class
}// end of io
}// end of tetra
}// end of vcg
#endif
