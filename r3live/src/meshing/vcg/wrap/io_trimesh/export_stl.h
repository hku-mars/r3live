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
Revision 1.8  2006/09/18 08:55:33  cignoni
Corrected return value of save function (zero is no error)

Revision 1.7  2006/01/30 13:43:59  cignoni
Added GetExportMaskCapability

Revision 1.6  2006/01/13 15:47:43  cignoni
Uniformed return type to the style of Open. Now every export function returns 0 in case of success.

Revision 1.5  2005/12/01 00:58:56  cignoni
Added and removed typenames for gcc compiling...

Revision 1.4  2004/10/28 00:52:45  cignoni
Better Doxygen documentation

Revision 1.3  2004/03/09 21:26:47  cignoni
cr lf mismatch

Revision 1.2  2004/03/03 15:35:53  cignoni
Yet another cr lf mismatch

Revision 1.3  2004/02/19 15:28:01  ponchio
*** empty log message ***

Revision 1.2  2004/02/13 02:18:57  cignoni
Edited Comments and GPL license


****************************************************************************/

#ifndef __VCGLIB_EXPORT_STL
#define __VCGLIB_EXPORT_STL

#include <stdio.h>

namespace vcg {
namespace tri {
namespace io {

/** 
This class encapsulate a filter for opening stl (sterolitograpy) meshes.
The stl format is quite simple and rather un-flexible. It just stores, in ascii or binary the, unindexed, geometry of the faces.
*/
template <class SaveMeshType>
class ExporterSTL
{
public:    
typedef typename SaveMeshType::FaceType FaceType;
typedef unsigned short CallBackSTLFaceAttribute(const SaveMeshType &m, const FaceType &f);

static int Save(SaveMeshType &m, const char * filename, const int &mask, CallBackPos *)
{
 return Save(m,filename,true,mask);
}

static int Save(SaveMeshType &m, const char * filename , bool binary =true, int mask=0, const char *objectname=0, bool magicsMode=0)
{
  typedef typename SaveMeshType::FaceIterator FaceIterator;
	FILE *fp;

	fp = fopen(filename,"wb");
	if(fp==0)
		return 1;

	if(binary)
	{
		// Write Header
		char header[128]="VCG                                                                                                  ";
		if(objectname)	strncpy(header,objectname,80);
		if(magicsMode)
		{
		  strncpy(header,"COLOR=XXX MATERIAL=AAA BBB CCC                                                                       ",80);
		  for(int i=0;i<3;++i)
		  {
			header[0x06+i]=0x7f;
			header[0x13+i]=0x7f;
			header[0x17+i]=0x7f;
			header[0x1b+i]=0x7f;
		  }
		}
		fwrite(header,80,1,fp);
		// write number of facets
		fwrite(&m.fn,1,sizeof(int),fp); 
		Point3f p;
		unsigned short attributes=0;
		for(FaceIterator fi=m.face.begin(); fi!=m.face.end(); ++fi) if( !(*fi).IsD() )
		{
			// For each triangle write the normal, the three coords and a short set to zero
			p.Import(vcg::NormalizedNormal(*fi));
			fwrite(p.V(),3,sizeof(float),fp);
 
			for(int k=0;k<3;++k){
				p.Import((*fi).V(k)->P());
				fwrite(p.V(),3,sizeof(float),fp);
			}
			if ((mask & Mask::IOM_FACECOLOR) && tri::HasPerFaceColor(m))
			{
			  if(magicsMode) attributes = 32768 | vcg::Color4b::ToUnsignedR5G5B5(fi->C());
						else attributes = 32768 | vcg::Color4b::ToUnsignedB5G5R5(fi->C());
			}
			fwrite(&attributes,1,sizeof(short),fp);
		}
	}
	else
	{
		if(objectname) fprintf(fp,"solid %s\n",objectname);
		else fprintf(fp,"solid vcg\n");

		Point3f p;
		FaceIterator fi;	
		for(fi=m.face.begin(); fi!=m.face.end(); ++fi) if( !(*fi).IsD() )
		{
	  	// For each triangle write the normal, the three coords and a short set to zero
			p.Import(vcg::NormalizedNormal(*fi));
			fprintf(fp,"  facet normal %13e %13e %13e\n",p[0],p[1],p[2]);
			fprintf(fp,"    outer loop\n");
			for(int k=0;k<3;++k){
				p.Import((*fi).V(k)->P());
				fprintf(fp,"      vertex  %13e %13e %13e\n",p[0],p[1],p[2]);			
			}
			fprintf(fp,"    endloop\n");
			fprintf(fp,"  endfacet\n");
		}
		fprintf(fp,"endsolid vcg\n");
	}
	fclose(fp);
	return 0;
}
static const char *ErrorMsg(int error)
{
  static std::vector<std::string> stl_error_msg;
  if(stl_error_msg.empty())
  {
    stl_error_msg.resize(2 );
    stl_error_msg[0]="No errors";
	  stl_error_msg[1]="Can't open file";
    }

  if(error>1 || error<0) return "Unknown error";
  else return stl_error_msg[error].c_str();
};

/*
	returns mask of capability one define with what are the saveable information of the format.
*/
static int GetExportMaskCapability()
{
	int capability = 0;			
	capability |= vcg::tri::io::Mask::IOM_VERTCOORD;
	capability |= vcg::tri::io::Mask::IOM_FACEINDEX;
	capability |= vcg::tri::io::Mask::IOM_FACECOLOR;
	return capability;
}


}; // end class

} // end Namespace tri
} // end Namespace io
} // end Namespace vcg



#endif
