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
****************************************************************************/

#ifndef __VCGLIB_IMPORTER_SMF
#define __VCGLIB_IMPORTER_SMF

#include <stdio.h>
#include <vcg/space/point3.h>

namespace vcg {
namespace tri {
namespace io {
	template<class OpenMeshType>
	/** 
	This class encapsulate a filter for opening smf meshes.
	*/
	class ImporterSMF
	{
	public:
		typedef typename OpenMeshType::VertexPointer VertexPointer;
		typedef typename OpenMeshType::ScalarType ScalarType;
		typedef typename OpenMeshType::VertexType VertexType;
		typedef typename OpenMeshType::FaceType FaceType;
		typedef typename OpenMeshType::VertexIterator VertexIterator;
		typedef typename OpenMeshType::FaceIterator FaceIterator;
		
		// if it is binary there are 80 char of comment, the number fn of faces and then exactly fn*4*3 bytes.
		enum {SMF_LABEL_SIZE=80};
		
		class SMFFacet
		{
		public:
		vcg::Point3f n;
		vcg::Point3f v[3];
		};
		
		enum SMFError {
			E_NOERROR,				// 0
			// Errori di open
			E_CANTOPEN,				// 1
			E_UNESPECTEDEOF       	// 2
		};
		
		/// Standard call for knowing the meaning of an error code
		static const char *ErrorMsg(int error)
		{
			static const char * smf_error_msg[] =
			{
				"No errors",
				"Can't open file",
				"Premature End of file",
			};
			if(error>2 || error<0) return "Unknown error";
			else return smf_error_msg[error];
		};
		///Standard call for reading a mesh
		static int Open(OpenMeshType &m, const char * filename )
		{
			VertexType v;
			FaceType f;
			FILE *fp;
			float x,y,z;	
			bool one = true;			
			std::map<int,VertexPointer> mv;
			fp = fopen(filename,"r");
			if(!fp) return -1;
			char buf[1024];
			while( fgets(buf,1024,fp) )	
			{		
				char *vf, *comm_pt;		
				if((comm_pt = strstr(buf,"#")) != NULL)
					*comm_pt = '\0';
				if( (vf = strstr(buf,"v")) != NULL )
				{
					sscanf(vf+1,"%f %f %f", &x, &y, &z);			
					v.P()[0] = x;			
					v.P()[1] = y;			
					v.P()[2] = z;			
					m.vert.push_back(v);		
				}		
				else if( (vf = strstr(buf,"f")) != NULL)		
				{			
					if(one)			
					{			
						VertexIterator vi;				
						int ind;				
						for(ind=1,vi=m.vert.begin(); vi!=m.vert.end(); ++vi,++ind)				
							mv[ind]=&*vi;				
						one = false;			
					}			
					int v1,v2,v3;			
					sscanf(vf+1,"%d %d %d", &v1, &v2, &v3);			
					f.V(0) = mv[v1];			
					f.V(1) = mv[v2];			
					f.V(2) = mv[v3];			
					m.face.push_back(f);			
				}	
			}	
			m.vn = m.vert.size();	
			m.fn = m.face.size();	
			return 0;
		}
	};
}// end of io
}// end of tri
}// end of vcg

#endif

