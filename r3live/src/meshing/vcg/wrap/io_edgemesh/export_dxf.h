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
Revision 1.4  2005/02/03 11:22:34  spinelli
ricorretti i metodi save per rendere compatibile il formato dxf con il formato di autocad specificato nel dxf reference 2005

Revision 1.3  2004/07/02 17:08:12  ganovelli
created

Revision 1.2  2004/06/10 15:15:16  ganovelli
changes to comply dxf specs

Revision 1.1  2004/05/27 13:24:08  ganovelli
export_dxf created

****************************************************************************/
#ifndef __VCG_LIB_EXPORTER_DXF
#define __VCG_LIB_EXPORTER_DXF


namespace vcg {
	namespace edg {
		namespace io {


template <class EdgeMeshType>
class ExporterDXF
{
public:

	ExporterDXF(void){}

	

	static bool Save(EdgeMeshType  *mp, const char * filename)
	{
		FILE * o = fopen(filename,"w");
		if(o==NULL)	return false;
		fprintf(o,"0\n");
		fprintf(o,"SECTION\n");
		fprintf(o,"2\n");
		fprintf(o,"ENTITIES\n");

		Save(mp,o);

		fprintf(o,"0\n");
		fprintf(o,"ENDSEC\n");
		fprintf(o,"0\n");
		fprintf(o,"EOF\n");
		fclose(o);
		return true;
	}



	static void Save(EdgeMeshType *mp, FILE* o )
	{
		typename EdgeMeshType::EdgeIterator i;
		for(i=mp->edges.begin(); i!=mp->edges.end();++i)
		{
			Point3f p1 = (*i).V(0)->P();
			Point3f p2 = (*i).V(1)->P();


			fprintf(o,"0\n");  
			fprintf(o,"LINE\n");
			fprintf(o,"8\n");  
			fprintf(o,"0\n");       
			fprintf(o,"10\n"); 

			fprintf(o,"%f\n", p1[0]);     //X
			fprintf(o,"20\n"); 
			fprintf(o,"%f\n", p1[1]);     //Y
			fprintf(o,"30\n");  
			fprintf(o,"%f\n", p1[2]);     //Z

			fprintf(o,"11\n");
			fprintf(o,"%f\n", p2[0]);     //X
			fprintf(o,"21\n"); 
			fprintf(o,"%f\n", p2[1]);     //Y
			fprintf(o,"31\n"); 
			fprintf(o,"%f\n", p2[2]);     //Z

		}



	}


};

		};
	};
};
#endif
