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
Revision 1.1  2004/10/11 17:41:41  ganovelli
added

Revision 1.1  2004/06/03 13:16:32  ganovelli
created

Revision 1.3  2004/05/10 13:14:28  ganovelli
converted to library style (namespaces etc..)


****************************************************************************/
#ifndef __VCGLIB_EXPORTERSMF
#define __VCGLIB_EXPORTERSMF

#include <vcg/space/point3.h>

namespace vcg {
namespace tetra {
namespace io {

template <typename  MESHTYPE>
struct ExporterTS{

	typedef typename MESHTYPE::VertexPointer VertexPointer;
	typedef typename MESHTYPE::VertexType VertexType;
	typedef typename MESHTYPE::TetraType FaceType;
	typedef typename MESHTYPE::VertexIterator VertexIterator;
	typedef typename MESHTYPE::TetraIterator TetraIterator;
	typedef typename MESHTYPE::ScalarType ScalarType;
	typedef Point3<ScalarType> Point3x;
	
	static FILE *& F(){static FILE * f; return f;}

	inline static void WritePos(const Point3<ScalarType> &p){
		fprintf(F(),"%g %g %g\n",p[0],p[1],p[2]);
	}
	inline static void WritePos(const Point4<ScalarType> &p){
		fprintf(F(),"%g %g %g %g\n",p[0],p[1],p[2],p[3]);
	}



static int Save( MESHTYPE & m, const char * filename )
{	
	
	F() = fopen(filename,"w");
	if(F() == NULL ) 
		{
			printf( "The file could not be opened\n" );
			return -1;
		}
   else
   {
		fprintf(F(), "%i\n", m.vn );
		fprintf(F(), "%i\n", m.tn );
		VertexIterator vi;
		for (vi = m.vert.begin(); vi != m.vert.end();++vi)
			//fprintf(F(), "%f %f %f \n", (*vi).P()[0],(*vi).P()[1],(*vi).P()[2] );
			WritePos((*vi).P());

		TetraIterator ti;
		for( ti = m.tetra.begin(); ti != m.tetra.end(); ++ti)
			fprintf(F(), "%d %d %d %d \n",
							(*ti).V(0)-&*m.vert.begin(),
							(*ti).V(1)-&*m.vert.begin(),
							(*ti).V(2)-&*m.vert.begin(),
							(*ti).V(3)-&*m.vert.begin());
	 }
 return 0;
}
	};// end class
		};// end of io
	};// end of tri
};// end of vcg

#endif

