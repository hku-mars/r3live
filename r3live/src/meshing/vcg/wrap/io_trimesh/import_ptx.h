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

#ifndef __VCGLIB_IMPORT_PTX
#define __VCGLIB_IMPORT_PTX

#include <stdio.h>
#include <wrap/callback.h>
#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/position.h>
#include <vcg/complex/algorithms/update/bounding.h>

namespace vcg {
namespace tri {
namespace io {
	/** 
	This class encapsulate a filter for importing ptx meshes.
	*/
	template <class OpenMeshType>
	class ImporterPTX
	{
	public:
		typedef typename OpenMeshType::VertexPointer VertexPointer;
		typedef typename OpenMeshType::ScalarType ScalarType;
		typedef typename OpenMeshType::VertexType VertexType;
		typedef typename OpenMeshType::FaceType FaceType;
		typedef typename OpenMeshType::VertexIterator VertexIterator;
		typedef typename OpenMeshType::FaceIterator FaceIterator;
		
		class Info		//ptx file info
		{
		public:

			Info()
			{
				mask				= 0;
				meshnum			= 0;
				anglecull		= true;
				angle				= 89;
				savecolor		= true;
				pointcull		= true;
				pointsonly	= false;
				switchside	= false;
				flipfaces		= false;
			}

			/// a bit mask describing the field preesnt in the ply file
			int mask;  

			/// index of mesh to be imported
			int meshnum;

			/// if true use angle cull
			bool anglecull;
			/// culling angle, if angle culling is selected
			float angle;

			/// if true, remove invalid points
			bool pointcull;

			/// if true, only keeps points
			bool pointsonly;

			/// if true, color if saved. if no color is present, reflectancy is used instead
			bool savecolor;

			/// switch row-columns
			bool switchside;
			/// flip faces
			bool flipfaces;

		}; // end ptx file info class


		/// Standard call for knowing the meaning of an error code
		static const char *ErrorMsg(int error)
		{
			static const char * ptx_error_msg[] =
			{
				"No errors",
				"Can't open file",
				"Header not found",
				"Eof in header",
				"Format not found",
				"Syntax error on header",
			};
			if(error>6 || error<0) return "Unknown error";
			else return ptx_error_msg[error];
		};

		/// skip ONE range map inside the ptx file, starting from current position
		/// returns true if skipped, false if failed/eof
		static bool skipmesh(FILE* fp, CallBackPos *cb=NULL)
		{
			int colnum;
			int rownum;
			int skiplines;
			char linebuf;

			if(feof(fp))	return false;

			// getting mesh size;
			fscanf(fp,"%i\n",&colnum);
			fscanf(fp,"%i\n",&rownum);

			if ( ( colnum <=0 ) || ( rownum <=0 ) ) return false;
			if(feof(fp))	return false;

			// have to skip (col * row) lines plus 8 lines for the header
			skiplines = (colnum * rownum) + 8; 
			for(int ii=0; ii<skiplines; ii++)
			{
				fread(&linebuf,1,1,fp);
				while(linebuf != '\n')  fread(&linebuf,1,1,fp);
			} 

			if(cb) cb( 100, "Skipped preamble");
			return true;
		}

		///Standard call that reading a mesh
		static int Open( OpenMeshType &m, const char * filename, Info importparams, CallBackPos *cb=NULL)
		{
			FILE *fp;
			fp = fopen(filename, "rb");
			if(fp == NULL) return false;
			m.Clear();
			m.vn=0;
			m.fn=0;

			// if not exporting first one, skip meshes until desired one
			if (importparams.meshnum>0) 
				for (int i=0; i!=importparams.meshnum; ++i)  
					if(!skipmesh(fp, cb))
						return 1;

			if (!readPTX( m, fp, importparams, cb))
			{
				m.Clear();
				return 1;
			}

			return 0;
		}

		///Call that load a mesh
		static bool readPTX( OpenMeshType &m, FILE *fp, Info importparams, CallBackPos *cb=NULL)
		{
			int numtokens;
			int colnum;
			int rownum;
			float xx,yy,zz;		// position
			float rr,gg,bb;		// color
			float rf;					// reflectance
			Matrix44f currtrasf;

			bool hascolor;
			bool savecolor   =  importparams.savecolor &&  VertexType::HasColor();
      bool switchside  =  importparams.switchside;

			int total = 50;
      if (importparams.pointsonly) total = 100;
			char linebuf[256];
			
			fscanf(fp,"%i\n",&colnum);					
			fscanf(fp,"%i\n",&rownum);
			
			if ( ( colnum <=0 ) || ( rownum <=0 ) ) return false;
			// initial 4 lines [still don't know what is this :) :)]
			if ( !fscanf(fp,"%f %f %f\n", &xx, &yy, &zz) ) return false;
			if ( !fscanf(fp,"%f %f %f\n", &xx, &yy, &zz) ) return false;
			if ( !fscanf(fp,"%f %f %f\n", &xx, &yy, &zz) ) return false;
			if ( !fscanf(fp,"%f %f %f\n", &xx, &yy, &zz) ) return false;
									// now the transformation matrix
			if ( !fscanf(fp,"%f %f %f %f\n", &(currtrasf.ElementAt(0,0)), &(currtrasf.ElementAt(0,1)), &(currtrasf.ElementAt(0,2)), &(currtrasf.ElementAt(0,3))) )return false;
			if ( !fscanf(fp,"%f %f %f %f\n", &(currtrasf.ElementAt(1,0)), &(currtrasf.ElementAt(1,1)), &(currtrasf.ElementAt(1,2)), &(currtrasf.ElementAt(1,3))) )return false;
			if ( !fscanf(fp,"%f %f %f %f\n", &(currtrasf.ElementAt(2,0)), &(currtrasf.ElementAt(2,1)), &(currtrasf.ElementAt(2,2)), &(currtrasf.ElementAt(2,3))) )return false;
			if ( !fscanf(fp,"%f %f %f %f\n", &(currtrasf.ElementAt(3,0)), &(currtrasf.ElementAt(3,1)), &(currtrasf.ElementAt(3,2)), &(currtrasf.ElementAt(3,3))) )return false;
			    	
			//now the real data begins
			// first line, we should know if the format is
			// XX YY ZZ RF
			// or it is
			// XX YY ZZ RF RR GG BB
			// read the entire first line and then count the spaces. it's rude but it works :)
			int ii=0;
			fread(&(linebuf[ii++]),1,1,fp);
			while(linebuf[ii-1] != '\n')  if ( fread(&(linebuf[ii++]),1,1,fp)==0 ) return false;
			linebuf[ii-1] = '\0'; // terminate the string
			numtokens=1;
			for(ii=0; ii<(int)strlen(linebuf); ii++) if(linebuf[ii] == ' ') numtokens++;
			if(numtokens == 4)  hascolor = false;
			else if(numtokens == 7)  hascolor = true;
			else  return false;
			
			// PTX transformation matrix is transposed
			currtrasf.transposeInPlace();
			
			// allocating vertex space
			int vn = rownum*colnum;
			VertexIterator vi = Allocator<OpenMeshType>::AddVertices(m,vn); 
			m.vn = vn;
			m.bbox.SetNull();

			// parse the first line....
			if(hascolor)
			{
				printf("\n hascolor ");
				sscanf(linebuf,"%f %f %f %f %f %f %f", &xx, &yy, &zz, &rf, &rr, &gg, &bb);
			}
			else
			{
				printf("\n no color ");
				sscanf(linebuf,"%f %f %f %f", &xx, &yy, &zz, &rf);
			}
			
			//addthefirstpoint
			(*vi).P()[0]=xx;
			(*vi).P()[1]=yy;
			(*vi).P()[2]=zz;

			if(VertexType::HasQuality())
			{
				(*vi).Q()=rf;
			}

      if(savecolor)
      {
        if(hascolor)
          {
            (*vi).C()[0]=rr;
            (*vi).C()[1]=gg;
            (*vi).C()[2]=bb;
            } else {
            (*vi).C()[0]=rf*255;
            (*vi).C()[1]=rf*255;
            (*vi).C()[2]=rf*255;
          }
       }
			vi++;

      if(switchside) std::swap(rownum,colnum);

			// now for each line until end of mesh (row*col)-1
			for(ii=0; ii<((rownum*colnum)-1); ii++)
			{
        if(cb && (ii%100)==0) cb((ii*total)/vn, "Vertex Loading");

				// read the stream
				if(hascolor)   
					fscanf(fp,"%f %f %f %f %f %f %f", &xx, &yy, &zz, &rf, &rr, &gg, &bb);
				else  
					fscanf(fp,"%f %f %f %f", &xx, &yy, &zz, &rf);

				// add the point
				(*vi).P()[0]=xx;
				(*vi).P()[1]=yy;
				(*vi).P()[2]=zz;
					

        if(tri::HasPerVertexQuality(m)) (*vi).Q()=rf;

				if(hascolor && savecolor)
				{
					(*vi).C()[0]=rr;
					(*vi).C()[1]=gg;
					(*vi).C()[2]=bb;
				}
				else if(!hascolor && savecolor)
				{
					(*vi).C()[0]=rf*255;
					(*vi).C()[1]=rf*255;
					(*vi).C()[2]=rf*255;
				}

				vi++;
			}

      if(! importparams.pointsonly)
			{
				// now i can triangulate
				int trinum = (rownum-1) * (colnum-1) * 2;
				typename OpenMeshType::FaceIterator fi= Allocator<OpenMeshType>::AddFaces(m,trinum);
        int v0i,v1i,v2i, t;
				t=0;
				for(int rit=0; rit<rownum-1; rit++)
					for(int cit=0; cit<colnum-1; cit++)
					{
						t++;
						if(cb) cb(50 + (t*50)/(rownum*colnum),"PTX Mesh Loading");	

							v0i = (rit  ) + ((cit  ) * rownum);
							v1i = (rit+1) + ((cit  ) * rownum);
							v2i = (rit  ) + ((cit+1) * rownum);
						
						// upper tri
						(*fi).V(2) = &(m.vert[v0i]);
						(*fi).V(1) = &(m.vert[v1i]);
						(*fi).V(0) = &(m.vert[v2i]);

						fi++;

							v0i = (rit+1) + ((cit  ) * rownum);
							v1i = (rit+1) + ((cit+1) * rownum);
							v2i = (rit  ) + ((cit+1) * rownum);

            // lower tri
						(*fi).V(2) = &(m.vert[v0i]);
						(*fi).V(1) = &(m.vert[v1i]);
						(*fi).V(0) = &(m.vert[v2i]);

						fi++;
					}
			}	
     printf("Loaded %i vert\n",m.vn);
			// remove unsampled points
			if(importparams.pointcull)
			{
				if(cb) cb(40,"PTX Mesh Loading - remove invalid vertices");	
				for(VertexIterator vi = m.vert.begin(); vi != m.vert.end(); vi++)
				{
					if((*vi).P() == Point3f(0.0, 0.0, 0.0)) 
						Allocator<OpenMeshType>::DeleteVertex(m,*vi);						
				}

				if(! importparams.pointsonly)
				{
					if(cb) cb(60,"PTX Mesh Loading - remove invalid faces");	
					for(typename OpenMeshType::FaceIterator fi = m.face.begin(); fi != m.face.end(); fi++)
					{
						if( ((*fi).V(0)->IsD()) || ((*fi).V(1)->IsD()) || ((*fi).V(2)->IsD()) )
								Allocator<OpenMeshType>::DeleteFace(m,*fi);						
					}
				}
			}

      float limitCos = cos( math::ToRad(importparams.angle) );
      printf("Loaded %i vert\n",m.vn);
      if(importparams.pointsonly)
      { // Compute Normals and radius for points
        // Compute the four edges around each point
        // Some edges can be null (boundary and invalid samples)
        if(cb) cb(85,"PTX Mesh Loading - computing vert normals");
        for(int rit=0; rit<rownum; rit++)
        {
          int ritL = std::max(rit-1,0);
          int ritR = std::min(rit+1,rownum-1);
          for(int cit=0; cit<colnum; cit++)
          {
            int citT = std::max(cit-1,0);
            int citB = std::min(cit+1,colnum-1);
            int v0 = (rit ) + ((cit ) * rownum);

            if(m.vert[v0].IsD()) continue;

            int vL = (ritL) + ((cit ) * rownum);
            int vR = (ritR) + ((cit) * rownum);
            int vT = (rit ) + ((citT ) * rownum);
            int vB = (rit ) + ((citB) * rownum);

            Point3f v0p=m.vert[v0].P();
            Point3f vLp(0,0,0),vRp(0,0,0),vTp(0,0,0),vBp(0,0,0);  // Compute the 4 edges around the vertex.
            if(!m.vert[vL].IsD()) vLp=(m.vert[vL].P()-v0p).Normalize();
            if(!m.vert[vR].IsD()) vRp=(m.vert[vR].P()-v0p).Normalize();
            if(!m.vert[vT].IsD()) vTp=(m.vert[vT].P()-v0p).Normalize();
            if(!m.vert[vB].IsD()) vBp=(m.vert[vB].P()-v0p).Normalize();
            float r=0;
            int rc=0; Point3f v0pn = Normalize(v0p);
            // Skip edges that are too steep
            // Compute the four normalized vector orthogonal to each pair of consecutive edges.
            Point3f vLTn =  (vLp ^ vTp).Normalize();
            Point3f vTRn =  (vTp ^ vRp).Normalize();
            Point3f vRBn =  (vRp ^ vBp).Normalize();
            Point3f vBLn =  (vBp ^ vLp).Normalize();

            // Compute an average Normal skipping null normals and normals that are too steep.
            // Compute also the sum of non null edge lenght to compute the radius
            Point3f N(0,0,0);
            if((vLTn*v0pn)>limitCos)  { N+=vLTn; r += Distance(m.vert[vL].P(),v0p)+Distance(m.vert[vT].P(),v0p); rc++; }
            if((vTRn*v0pn)>limitCos)  { N+=vTRn; r += Distance(m.vert[vT].P(),v0p)+Distance(m.vert[vR].P(),v0p); rc++; }
            if((vRBn*v0pn)>limitCos)  { N+=vRBn; r += Distance(m.vert[vR].P(),v0p)+Distance(m.vert[vB].P(),v0p); rc++; }
            if((vBLn*v0pn)>limitCos)  { N+=vBLn; r += Distance(m.vert[vB].P(),v0p)+Distance(m.vert[vL].P(),v0p); rc++; }

            m.vert[v0].N()=-N;

            if(tri::HasPerVertexRadius(m)) m.vert[v0].R() = r/(rc*2.0f);
            // Isolated points has null normal. Delete them please.
            if(m.vert[v0].N() == Point3f(0,0,0)) Allocator<OpenMeshType>::DeleteVertex(m,m.vert[v0]);
          }
        }
      }
      else
      // eliminate high angle triangles
      {
        if(importparams.flipfaces)
          tri::Clean<OpenMeshType>::FlipMesh(m);
        if(importparams.anglecull)
        {
          if(cb) cb(85,"PTX Mesh Loading - remove steep faces");
          tri::UpdateNormal<OpenMeshType>::PerFaceNormalized(m);
          for(FaceIterator fi = m.face.begin(); fi != m.face.end(); fi++)
            if(!(*fi).IsD())
            {
            Point3f raggio = -((*fi).P(0) + (*fi).P(1) + (*fi).P(2)) / 3.0;
            raggio.Normalize();
            if((raggio.dot((*fi).N())) < limitCos)
              Allocator<OpenMeshType>::DeleteFace(m,*fi);
          }
          // deleting unreferenced vertices only if we are interested in faces...
          tri::Clean<OpenMeshType>::RemoveUnreferencedVertex(m);
        }
      }

      tri::UpdatePosition<OpenMeshType>::Matrix(m,currtrasf,true);
      tri::Allocator<OpenMeshType>::CompactVertexVector(m);
      tri::UpdateBounding<OpenMeshType>::Box(m);
			if(cb) cb(100,"PTX Mesh Loading finish!");
			return true;
		}
	
	}; // end class

} // end Namespace tri
} // end Namespace io
} // end Namespace vcg
#endif
