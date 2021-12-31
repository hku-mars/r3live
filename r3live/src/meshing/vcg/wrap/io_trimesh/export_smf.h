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
Revision 1.3  2006/11/21 19:23:50  e_cerisoli
Added comments for documentation

Revision 1.2  2006/11/16 11:24:44 
****************************************************************************/

#ifndef __VCGLIB_EXPORT_SMF
#define __VCGLIB_EXPORT_SMF

#include <stdio.h>

namespace vcg {
namespace tri {
namespace io {
	
	template <class SaveMeshType>
	/** 
	This class encapsulate a filter for save smf meshes.
	*/
	class ExporterSMF
	{
	public:
		typedef typename SaveMeshType::VertexPointer VertexPointer;
		typedef typename SaveMeshType::ScalarType ScalarType;
		typedef typename SaveMeshType::VertexType VertexType;
		typedef typename SaveMeshType::FaceType FaceType;
		typedef typename SaveMeshType::VertexIterator VertexIterator;
		typedef typename SaveMeshType::FaceIterator FaceIterator;
		
		///Standard call for saving a mesh
		static int Save(SaveMeshType &m, const char * filename, const int &mask, CallBackPos *cb=0)
		{
			VertexIterator vi;
			FaceIterator fi;
			FILE *fp;
			fp = fopen(filename,"wb");
			fprintf(fp,"#SMF \n" );
			
			std::map<VertexPointer,int> index;
			int ind;
	
			for(ind=1,vi=m.vert.begin(); vi!=m.vert.end(); ++vi,++ind)
			{
				fprintf(fp,"v " );
				fprintf(fp,"%f%s",(*vi).P()[0]," " );
				fprintf(fp,"%f%s",(*vi).P()[1]," " );
				fprintf(fp,"%f%s",(*vi).P()[2],"\n");
	
				index[&*vi] = ind;
			}
			for (fi=m.face.begin(); fi!=m.face.end(); ++fi)
			{
				fprintf(fp,"%s","f ");
				for (int j = 0; j < 3; j++)
					fprintf(fp,"%i%s",index[(*fi).V(j)]," ");
				fprintf(fp,"%s","\n");
			}
			fclose(fp);
			return 0;
		}

		/// Standard call for knowing the meaning of an error code
		static const char *ErrorMsg(int error)
		{
			static std::vector<std::string> smf_error_msg;
			if(smf_error_msg.empty())
			{
				smf_error_msg.resize(2 );
				smf_error_msg[0]="No errors";
				smf_error_msg[1]="Can't open file";
			}

			if(error>1 || error<0) return "Unknown error";
			else return smf_error_msg[error].c_str();
		}
	}; // end class
} // end Namespace tri
} // end Namespace io
} // end Namespace vcg

#endif

