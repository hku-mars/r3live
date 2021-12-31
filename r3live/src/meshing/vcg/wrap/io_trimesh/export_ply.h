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
Revision 1.30  2008/04/29 11:51:28  corsini
set defaut callback (in save) to null

Revision 1.29  2008/01/24 11:54:23  cignoni
passed the callback in the save

Revision 1.28  2007/12/13 00:20:34  cignoni
removed harmless printf cast warnings

Revision 1.27  2007/11/06 10:59:41  ponchio
Typo

Revision 1.26  2007/11/06 10:51:55  ponchio
Fixed wrong 'return false' in Save.

Revision 1.25  2007/10/22 14:47:19  cignoni
Added saving of per vertex normals

Revision 1.24  2007/07/23 13:27:50  tarini
fixed bug on saving flags-per-face

Revision 1.23  2007/03/12 16:40:17  tarini
Texture coord name change!  "TCoord" and "Texture" are BAD. "TexCoord" is GOOD.

Revision 1.22  2007/02/18 08:01:07  cignoni
Added missing typename

Revision 1.21  2007/02/14 16:07:41  ganovelli
added HasPerFaceFlag

Revision 1.20  2007/02/14 15:40:20  ganovelli
a wrong "!" corrected

Revision 1.19  2007/02/14 15:30:13  ganovelli
added treatment of HasPerVertexFlags absent

Revision 1.18  2006/12/18 09:46:39  callieri
camera+shot revamp: changed field names to something with more sense, cleaning of various functions, correction of minor bugs/incongruences, removal of the infamous reference in shot.

Revision 1.17  2006/11/30 22:49:32  cignoni
Added save with (unused) callback

Revision 1.16  2006/10/14 00:39:22  cignoni
Added a comment on an assert

Revision 1.15  2006/01/30 13:43:59  cignoni
Added GetExportMaskCapability

Revision 1.14  2006/01/27 09:11:48  corsini
fix signed/unsigned mismatch

Revision 1.13  2006/01/13 15:47:43  cignoni
Uniformed return type to the style of Open. Now every export function returns 0 in case of success.

Revision 1.12  2006/01/10 13:20:42  cignoni
Changed ply::PlyMask to io::Mask

Revision 1.11  2005/11/23 15:48:25  pietroni
changed shot::similarity to shot::Similarity() and shot::camera to shot::Camera()

Revision 1.10  2004/10/28 00:52:45  cignoni
Better Doxygen documentation

Revision 1.9  2004/10/27 09:33:10  ganovelli
cast from scalar type to float added

Revision 1.8  2004/10/07 14:19:06  ganovelli
shot/camera io added

Revision 1.7  2004/07/15 10:54:48  ganovelli
std added

Revision 1.6  2004/05/28 14:11:13  ganovelli
changes to comply io_mask moving in vcg::ply namesp

Revision 1.5  2004/05/12 14:43:36  cignoni
removed warning of unused variables

Revision 1.4  2004/05/12 10:19:30  ganovelli
new line added at the end of file

Revision 1.3  2004/03/18 15:30:46  cignoni
Removed float/double warning

Revision 1.2  2004/03/09 21:26:47  cignoni
cr lf mismatch

Revision 1.1  2004/03/08 09:21:33  cignoni
Initial commit

Revision 1.1  2004/03/03 15:00:51  cignoni
Initial commit

****************************************************************************/

/**
@name Load and Save in Ply format
*/
//@{

#ifndef __VCGLIB_EXPORT_PLY
#define __VCGLIB_EXPORT_PLY

//#include<wrap/ply/io_mask.h>
#include<wrap/io_trimesh/io_mask.h>
#include<wrap/io_trimesh/io_ply.h>
#include<vcg/container/simple_temporary_data.h>


#include <stdio.h>

namespace vcg {
namespace tri {
namespace io {


template <class SaveMeshType>
class ExporterPLY
{
  // Si occupa di convertire da un tipo all'altro.
// usata nella saveply per matchare i tipi tra stotype e memtype.
// Ad es se in memoria c'e' un int e voglio salvare un float
// src sara in effetti un puntatore a int il cui valore deve 
// essere convertito al tipo di ritorno desiderato (stotype)

template <class StoType> 
static void PlyConv(int mem_type, void *src, StoType &dest)
{
		switch (mem_type){
				case ply::T_FLOAT	:		dest = (StoType) (*  ((float  *) src)); break;
				case ply::T_DOUBLE:		dest = (StoType) (*  ((double *) src)); break;
				case ply::T_INT		:		dest = (StoType) (*  ((int    *) src)); break;
				case ply::T_SHORT	:		dest = (StoType) (*  ((short  *) src)); break;
				case ply::T_CHAR	:		dest = (StoType) (*  ((char   *) src)); break;
				case ply::T_UCHAR	:		dest = (StoType) (*  ((unsigned char *)src)); break;
			 	default : assert(0);
		}
}

public:
typedef ::vcg::ply::PropDescriptor PropDescriptor ;
typedef typename SaveMeshType::VertexPointer VertexPointer;
typedef typename SaveMeshType::ScalarType ScalarType;
typedef typename SaveMeshType::VertexType VertexType;
typedef typename SaveMeshType::FaceType FaceType;
typedef typename SaveMeshType::FacePointer FacePointer;
typedef typename SaveMeshType::VertexIterator VertexIterator;
typedef typename SaveMeshType::FaceIterator FaceIterator;
typedef typename SaveMeshType::EdgeIterator EdgeIterator;

static int Save(SaveMeshType &m, const char * filename, bool binary=true)
{
  PlyInfo pi;
  return Save(m,filename,binary,pi);
}

static int Save(SaveMeshType &m,  const char * filename, int savemask, bool binary = true, CallBackPos *cb=0 )
{
	PlyInfo pi;
  pi.mask=savemask;
  return Save(m,filename,binary,pi,cb);
}

static int Save(SaveMeshType &m,  const char * filename, bool binary, PlyInfo &pi, CallBackPos *cb=0)	// V1.0
{
	FILE * fpout;
	int i;
  
	const char * hbin = "binary_little_endian";
	const char * hasc = "ascii";
	const char * h;

	bool multit = false;

	if(binary) h=hbin;
	else       h=hasc;

	fpout = fopen(filename,"wb");
	if(fpout==NULL)	{
		pi.status=::vcg::ply::E_CANTOPEN;
		return ::vcg::ply::E_CANTOPEN;
	}
	fprintf(fpout,
		"ply\n"
		"format %s 1.0\n"
		"comment VCGLIB generated\n"
		,h
	);

	if (((pi.mask & Mask::IOM_WEDGTEXCOORD) != 0) || ((pi.mask & Mask::IOM_VERTTEXCOORD) != 0))
	{
		const char * TFILE = "TextureFile";

		for(i=0; i < static_cast<int>(m.textures.size()); ++i)
			fprintf(fpout,"comment %s %s\n", TFILE, (const char *)(m.textures[i].c_str()) );

    if(m.textures.size()>1 && (HasPerWedgeTexCoord(m) || HasPerVertexTexCoord(m))) multit = true;
	}

	if((pi.mask & Mask::IOM_CAMERA))
	 {
		fprintf(fpout,
			"element camera 1\n"
			"property float view_px\n"
			"property float view_py\n"
			"property float view_pz\n"
			"property float x_axisx\n"
			"property float x_axisy\n"
			"property float x_axisz\n"
			"property float y_axisx\n"
			"property float y_axisy\n"
			"property float y_axisz\n"
			"property float z_axisx\n"
			"property float z_axisy\n"
			"property float z_axisz\n"
			"property float focal\n"
			"property float scalex\n"
			"property float scaley\n"
			"property float centerx\n"
			"property float centery\n"
			"property int viewportx\n"
			"property int viewporty\n"
			"property float k1\n"
			"property float k2\n"
			"property float k3\n"
			"property float k4\n"
		);
	} 

	fprintf(fpout,
		"element vertex %d\n"
		"property float x\n"
		"property float y\n"
		"property float z\n"
		,m.vn
	);

  if( HasPerVertexNormal(m) &&( pi.mask & Mask::IOM_VERTNORMAL) )
	{
		fprintf(fpout,
						"property float nx\n"
						"property float ny\n"
						"property float nz\n"
						);
	}


  if( HasPerVertexFlags(m) &&( pi.mask & Mask::IOM_VERTFLAGS) )
	{
		fprintf(fpout,
			"property int flags\n"
		);
	}
	
  if( HasPerVertexColor(m)  && (pi.mask & Mask::IOM_VERTCOLOR) )
	{
		fprintf(fpout,
			"property uchar red\n"
			"property uchar green\n"
			"property uchar blue\n"
			"property uchar alpha\n"
		);
	}

  if( HasPerVertexQuality(m) && (pi.mask & Mask::IOM_VERTQUALITY) )
	{
		fprintf(fpout,
			"property float quality\n"
		);
	}

	if( tri::HasPerVertexRadius(m) && (pi.mask & Mask::IOM_VERTRADIUS) )
	{
		fprintf(fpout,
			"property float radius\n"
		);
	}
  if( ( HasPerVertexTexCoord(m) && pi.mask & Mask::IOM_VERTTEXCOORD ) )
  {
      fprintf(fpout,
              "property float texture_u\n"
              "property float texture_v\n"
              );
  }
	for(i=0;i<pi.vdn;i++)
			fprintf(fpout,"property %s %s\n",pi.VertexData[i].stotypename(),pi.VertexData[i].propname);
	
	fprintf(fpout,
		"element face %d\n"
		"property list uchar int vertex_indices\n"
		,m.fn
	);

  if(HasPerFaceFlags(m)   && (pi.mask & Mask::IOM_FACEFLAGS) )
	{
		fprintf(fpout,
			"property int flags\n"
		);
	}

  if( (HasPerWedgeTexCoord(m) || HasPerVertexTexCoord(m) ) && pi.mask & Mask::IOM_WEDGTEXCOORD ) // Note that you can save VT as WT if you really want it...
	{
		fprintf(fpout,
			"property list uchar float texcoord\n"
		);

		if(multit)
			fprintf(fpout,
				"property int texnumber\n"
			);
	}

  if( HasPerFaceColor(m) && (pi.mask & Mask::IOM_FACECOLOR) )
	{
		fprintf(fpout,
			"property uchar red\n"
			"property uchar green\n"
			"property uchar blue\n"
			"property uchar alpha\n"
		);
	}
	
  if ( HasPerWedgeColor(m) && (pi.mask & Mask::IOM_WEDGCOLOR)  )
	{
		fprintf(fpout,
			"property list uchar float color\n"
		);
	}

  if( HasPerFaceQuality(m) && (pi.mask & Mask::IOM_FACEQUALITY) )
	{
		fprintf(fpout,
			"property float quality\n"
		);
	}

	for(i=0;i<pi.fdn;i++)
			fprintf(fpout,"property %s %s\n",pi.FaceData[i].stotypename(),pi.FaceData[i].propname);
	// Saving of edges is enabled if requested
	if( m.en>0 && (pi.mask & Mask::IOM_EDGEINDEX) )
	  fprintf(fpout,
			  "element edge %d\n"
			  "property int vertex1\n"
			  "property int vertex2\n"
			  ,m.en
			  );
	fprintf(fpout, "end_header\n"	);

		// Salvataggio camera
	 if((pi.mask & Mask::IOM_CAMERA))
	 {
		 if(binary)
		 {
				float t[17];

				t[ 0] = (float)m.shot.Extrinsics.Tra()[0];
				t[ 1] = (float)m.shot.Extrinsics.Tra()[1];
				t[ 2] = (float)m.shot.Extrinsics.Tra()[2];
				t[ 3] = (float)m.shot.Extrinsics.Rot()[0][0];
				t[ 4] = (float)m.shot.Extrinsics.Rot()[0][1];
				t[ 5] = (float)m.shot.Extrinsics.Rot()[0][2];
				t[ 6] = (float)m.shot.Extrinsics.Rot()[1][0];
				t[ 7] = (float)m.shot.Extrinsics.Rot()[1][1];
				t[ 8] = (float)m.shot.Extrinsics.Rot()[1][2];
				t[ 9] = (float)m.shot.Extrinsics.Rot()[2][0];
				t[10] = (float)m.shot.Extrinsics.Rot()[2][1];
				t[11] = (float)m.shot.Extrinsics.Rot()[2][2];
				t[12] = (float)m.shot.Intrinsics.FocalMm;
				t[13] = (float)m.shot.Intrinsics.PixelSizeMm[0];
				t[14] = (float)m.shot.Intrinsics.PixelSizeMm[1];
				t[15] = (float)m.shot.Intrinsics.CenterPx[0];
				t[16] = (float)m.shot.Intrinsics.CenterPx[1];
				fwrite(t,sizeof(float),17,fpout);

				fwrite( &m.shot.Intrinsics.ViewportPx[0],sizeof(int),2,fpout );

				t[ 0] = (float)m.shot.Intrinsics.k[0];
				t[ 1] = (float)m.shot.Intrinsics.k[1];
				t[ 2] = (float)m.shot.Intrinsics.k[2];
				t[ 3] = (float)m.shot.Intrinsics.k[3];
				fwrite(t,sizeof(float),4,fpout);
		}
		else
		{
			fprintf(fpout,"%g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %d %d %g %g %g %g\n"
				,-m.shot.Extrinsics.Tra()[0]
				,-m.shot.Extrinsics.Tra()[1]
				,-m.shot.Extrinsics.Tra()[2]
				,m.shot.Extrinsics.Rot()[0][0]
				,m.shot.Extrinsics.Rot()[0][1]
				,m.shot.Extrinsics.Rot()[0][2]
				,m.shot.Extrinsics.Rot()[1][0]
				,m.shot.Extrinsics.Rot()[1][1]
				,m.shot.Extrinsics.Rot()[1][2]
				,m.shot.Extrinsics.Rot()[2][0]
				,m.shot.Extrinsics.Rot()[2][1]
				,m.shot.Extrinsics.Rot()[2][2]
				,m.shot.Intrinsics.FocalMm
				,m.shot.Intrinsics.PixelSizeMm[0]
				,m.shot.Intrinsics.PixelSizeMm[1]
				,m.shot.Intrinsics.CenterPx[0]
				,m.shot.Intrinsics.CenterPx[1]
				,m.shot.Intrinsics.ViewportPx[0]
				,m.shot.Intrinsics.ViewportPx[1]
				,m.shot.Intrinsics.k[0]
				,m.shot.Intrinsics.k[1]
				,m.shot.Intrinsics.k[2]
				,m.shot.Intrinsics.k[3]
			);
		}		
	}


	int j;
	std::vector<int> FlagV;
	VertexPointer  vp;
	VertexIterator vi;
	SimpleTempData<typename SaveMeshType::VertContainer,int> indices(m.vert);

	for(j=0,vi=m.vert.begin();vi!=m.vert.end();++vi){
		vp=&(*vi);
		indices[vi] = j;
	//((m.vn+m.fn) != 0) all vertices and faces have been marked as deleted but the are still in the vert/face vectors  
    if(cb && ((j%1000)==0) && ((m.vn+m.fn) != 0) )(*cb)( (100*j)/(m.vn+m.fn), "Saving Vertices");

    if( !HasPerVertexFlags(m) || !vp->IsD() )
		{
			if(binary)
			{
				float t;

        t = float(vp->P()[0]); fwrite(&t,sizeof(float),1,fpout);
        t = float(vp->P()[1]); fwrite(&t,sizeof(float),1,fpout);
        t = float(vp->P()[2]); fwrite(&t,sizeof(float),1,fpout);
				
        if( HasPerVertexNormal(m) && (pi.mask & Mask::IOM_VERTNORMAL) )
				{
					t = float(vp->N()[0]); fwrite(&t,sizeof(float),1,fpout);
					t = float(vp->N()[1]); fwrite(&t,sizeof(float),1,fpout);
					t = float(vp->N()[2]); fwrite(&t,sizeof(float),1,fpout);
				}
        if( HasPerVertexFlags(m) && (pi.mask & Mask::IOM_VERTFLAGS) )
                    fwrite(&(vp->Flags()),sizeof(int),1,fpout);

        if( HasPerVertexColor(m) && (pi.mask & Mask::IOM_VERTCOLOR) )
					fwrite(&( vp->C() ),sizeof(char),4,fpout);

        if( HasPerVertexQuality(m) && (pi.mask & Mask::IOM_VERTQUALITY) )
					fwrite(&( vp->Q() ),sizeof(float),1,fpout);

				if( HasPerVertexRadius(m) && (pi.mask & Mask::IOM_VERTRADIUS) )
					fwrite(&( vp->R() ),sizeof(float),1,fpout);

        if( HasPerVertexTexCoord(m) && (pi.mask & Mask::IOM_VERTTEXCOORD) )
        {
          t = float(vp->T().u()); fwrite(&t,sizeof(float),1,fpout);
          t = float(vp->T().v()); fwrite(&t,sizeof(float),1,fpout);
        }

				for(i=0;i<pi.vdn;i++)
				{
					double td(0); float tf(0);int ti;short ts; char tc; unsigned char tuc;
					switch (pi.VertexData[i].stotype1)
					{
          case ply::T_FLOAT	 :		PlyConv(pi.VertexData[i].memtype1,  ((char *)vp)+pi.VertexData[i].offset1, tf );	fwrite(&tf, sizeof(float),1,fpout); break;
					case ply::T_DOUBLE :		PlyConv(pi.VertexData[i].memtype1,  ((char *)vp)+pi.VertexData[i].offset1, td );	fwrite(&td, sizeof(double),1,fpout); break;
					case ply::T_INT		 :		PlyConv(pi.VertexData[i].memtype1,  ((char *)vp)+pi.VertexData[i].offset1, ti );	fwrite(&ti, sizeof(int),1,fpout); break;
					case ply::T_SHORT	 :		PlyConv(pi.VertexData[i].memtype1,  ((char *)vp)+pi.VertexData[i].offset1, ts );	fwrite(&ts, sizeof(short),1,fpout); break;
					case ply::T_CHAR	 :		PlyConv(pi.VertexData[i].memtype1,  ((char *)vp)+pi.VertexData[i].offset1, tc );	fwrite(&tc, sizeof(char),1,fpout); break;
					case ply::T_UCHAR	 :		PlyConv(pi.VertexData[i].memtype1,  ((char *)vp)+pi.VertexData[i].offset1, tuc);	fwrite(&tuc,sizeof(unsigned char),1,fpout); break;
					default : assert(0);
					}
				}
			}
			else 	// ***** ASCII *****
			{
				fprintf(fpout,"%g %g %g " ,vp->P()[0],vp->P()[1],vp->P()[2]);

        if( HasPerVertexNormal(m) && (pi.mask & Mask::IOM_VERTNORMAL) )
					fprintf(fpout,"%g %g %g " ,double(vp->N()[0]),double(vp->N()[1]),double(vp->N()[2]));

        if( HasPerVertexFlags(m) && (pi.mask & Mask::IOM_VERTFLAGS))
                    fprintf(fpout,"%d ",vp->Flags());

        if( HasPerVertexColor(m) && (pi.mask & Mask::IOM_VERTCOLOR) )
					fprintf(fpout,"%d %d %d %d ",vp->C()[0],vp->C()[1],vp->C()[2],vp->C()[3] );

        if( HasPerVertexQuality(m) && (pi.mask & Mask::IOM_VERTQUALITY) )
					fprintf(fpout,"%g ",vp->Q());

				if( HasPerVertexRadius(m) && (pi.mask & Mask::IOM_VERTRADIUS) )
					fprintf(fpout,"%g ",vp->R());

        if( HasPerVertexTexCoord(m) && (pi.mask & Mask::IOM_VERTTEXCOORD) )
          fprintf(fpout,"%g %g",vp->T().u(),vp->T().v());

				for(i=0;i<pi.vdn;i++)
				{
					float tf(0); double td(0);
					int ti;
					switch (pi.VertexData[i].memtype1)
					{
					case ply::T_FLOAT	 :		tf=*( (float  *)        (((char *)vp)+pi.VertexData[i].offset1));	fprintf(fpout,"%g ",tf); break;
					case ply::T_DOUBLE :    td=*( (double *)        (((char *)vp)+pi.VertexData[i].offset1));	fprintf(fpout,"%g ",tf); break;
					case ply::T_INT		 :		ti=*( (int    *)        (((char *)vp)+pi.VertexData[i].offset1));	fprintf(fpout,"%i ",ti); break;
					case ply::T_SHORT	 :		ti=*( (short  *)        (((char *)vp)+pi.VertexData[i].offset1)); fprintf(fpout,"%i ",ti); break;
					case ply::T_CHAR	 :		ti=*( (char   *)        (((char *)vp)+pi.VertexData[i].offset1));	fprintf(fpout,"%i ",ti); break;
					case ply::T_UCHAR	 :		ti=*( (unsigned char *) (((char *)vp)+pi.VertexData[i].offset1));	fprintf(fpout,"%i ",ti); break;
					default : assert(0);
					}
				}

				fprintf(fpout,"\n");
			}
			j++;
		}
	}
	/*vcg::tri::*/
	// this assert triggers when the vn != number of vertexes in vert that are not deleted.
  assert(j==m.vn); 

	char c = 3;
	unsigned char b9 = 9;
	unsigned char b6 = 6;
	FacePointer fp;
	int vv[3];
	FaceIterator fi;
	int fcnt=0;
	for(j=0,fi=m.face.begin();fi!=m.face.end();++fi)
		{
			//((m.vn+m.fn) != 0) all vertices and faces have been marked as deleted but the are still in the vert/face vectors  
		  if(cb && ((j%1000)==0) && ((m.vn+m.fn) != 0)) 
			  (*cb)( 100*(m.vn+j)/(m.vn+m.fn), "Saving Vertices");

			fp=&(*fi);
			if( ! fp->IsD() )
			{ fcnt++;
				if(binary)
				{
						vv[0]=indices[fp->cV(0)];
						vv[1]=indices[fp->cV(1)];
						vv[2]=indices[fp->cV(2)];
						fwrite(&c,1,1,fpout);
						fwrite(vv,sizeof(int),3,fpout);

          if(HasPerFaceFlags(m)&&( pi.mask & Mask::IOM_FACEFLAGS) )
						fwrite(&(fp->Flags()),sizeof(int),1,fpout);

          if( HasPerVertexTexCoord(m) && (pi.mask & Mask::IOM_VERTTEXCOORD) )
					{
						fwrite(&b6,sizeof(char),1,fpout);
						float t[6];
						for(int k=0;k<3;++k)
						{
							t[k*2+0] = fp->V(k)->T().u();
							t[k*2+1] = fp->V(k)->T().v();
						}
						fwrite(t,sizeof(float),6,fpout);
					}
          else if( HasPerWedgeTexCoord(m) && (pi.mask & Mask::IOM_WEDGTEXCOORD)  )
					{
						fwrite(&b6,sizeof(char),1,fpout);
						float t[6];
						for(int k=0;k<3;++k)
						{
							t[k*2+0] = fp->WT(k).u();
							t[k*2+1] = fp->WT(k).v();
						}
						fwrite(t,sizeof(float),6,fpout);
					}

					if(multit)
					{
						int t = fp->WT(0).n();
						fwrite(&t,sizeof(int),1,fpout);
					}
          
          if( HasPerFaceColor(m) && (pi.mask & Mask::IOM_FACECOLOR) )
					   fwrite(&( fp->C() ),sizeof(char),4,fpout);


          if( HasPerWedgeColor(m) && (pi.mask & Mask::IOM_WEDGCOLOR)  )
					{
						fwrite(&b9,sizeof(char),1,fpout);
						float t[3];
						for(int z=0;z<3;++z)
						{
							t[0] = float(fp->WC(z)[0])/255;
							t[1] = float(fp->WC(z)[1])/255;
							t[2] = float(fp->WC(z)[2])/255;
							fwrite( t,sizeof(float),3,fpout);
						}
					}

          if( HasPerFaceQuality(m) && (pi.mask & Mask::IOM_FACEQUALITY) )
						fwrite( &(fp->Q()),sizeof(float),1,fpout);


					for(i=0;i<pi.fdn;i++)
						{
						double td(0); float tf(0);int ti;short ts; char tc; unsigned char tuc;
						switch (pi.FaceData[i].stotype1){
								case ply::T_FLOAT	 :		PlyConv(pi.FaceData[i].memtype1,  ((char *)fp)+pi.FaceData[i].offset1, tf );	fwrite(&tf, sizeof(float),1,fpout); break;
								case ply::T_DOUBLE :		PlyConv(pi.FaceData[i].memtype1,  ((char *)fp)+pi.FaceData[i].offset1, td );	fwrite(&td, sizeof(double),1,fpout); break;
								case ply::T_INT		 :		PlyConv(pi.FaceData[i].memtype1,  ((char *)fp)+pi.FaceData[i].offset1, ti );	fwrite(&ti, sizeof(int),1,fpout); break;
								case ply::T_SHORT	 :		PlyConv(pi.FaceData[i].memtype1,  ((char *)fp)+pi.FaceData[i].offset1, ts );	fwrite(&ts, sizeof(short),1,fpout); break;
								case ply::T_CHAR	 :		PlyConv(pi.FaceData[i].memtype1,  ((char *)fp)+pi.FaceData[i].offset1, tc );	fwrite(&tc, sizeof(char),1,fpout); break;
								case ply::T_UCHAR	 :		PlyConv(pi.FaceData[i].memtype1,  ((char *)fp)+pi.FaceData[i].offset1, tuc);	fwrite(&tuc,sizeof(unsigned char),1,fpout); break;
								default : assert(0);
						}
					}
				}
				else	// ***** ASCII *****
				{
					fprintf(fpout,"3 %d %d %d ",
						indices[fp->cV(0)],	indices[fp->cV(1)], indices[fp->cV(2)] );

          if(HasPerFaceFlags(m)&&( pi.mask & Mask::IOM_FACEFLAGS ))
						fprintf(fpout,"%d ",fp->Flags());

          if( HasPerVertexTexCoord(m) && (pi.mask & Mask::IOM_WEDGTEXCOORD) ) // you can save VT as WT if you really want it...
					{
						fprintf(fpout,"6 ");
						for(int k=0;k<3;++k)
							fprintf(fpout,"%g %g "
								,fp->V(k)->T().u()
								,fp->V(k)->T().v()
							);
					}
          else if( HasPerWedgeTexCoord(m) && (pi.mask & Mask::IOM_WEDGTEXCOORD)  )
					{
						fprintf(fpout,"6 ");
						for(int k=0;k<3;++k)
							fprintf(fpout,"%g %g "
								,fp->WT(k).u()
								,fp->WT(k).v()
							);
					}

					if(multit)
					{
						fprintf(fpout,"%d ",fp->WT(0).n());
					}

          if( HasPerFaceColor(m) && (pi.mask & Mask::IOM_FACECOLOR)  )
					{
						float t[3];
						t[0] = float(fp->C()[0])/255;
						t[1] = float(fp->C()[1])/255;
						t[2] = float(fp->C()[2])/255;
						fprintf(fpout,"9 ");
						fprintf(fpout,"%g %g %g ",t[0],t[1],t[2]);
						fprintf(fpout,"%g %g %g ",t[0],t[1],t[2]);
						fprintf(fpout,"%g %g %g ",t[0],t[1],t[2]);
					}
          else if( HasPerWedgeColor(m) && (pi.mask & Mask::IOM_WEDGCOLOR)  )
					{
						fprintf(fpout,"9 ");
						for(int z=0;z<3;++z)
							fprintf(fpout,"%g %g %g "
								,double(fp->WC(z)[0])/255
								,double(fp->WC(z)[1])/255
								,double(fp->WC(z)[2])/255
							);
					}

          if( HasPerFaceQuality(m) && (pi.mask & Mask::IOM_FACEQUALITY) )
						fprintf(fpout,"%g ",fp->Q());

					for(i=0;i<pi.fdn;i++)
					{
						float tf(0); double td(0);
						int ti;
						switch (pi.FaceData[i].memtype1)
						{
						case  ply::T_FLOAT	:		tf=*( (float  *)        (((char *)fp)+pi.FaceData[i].offset1));	fprintf(fpout,"%g ",tf); break;
						case  ply::T_DOUBLE :		td=*( (double *)        (((char *)fp)+pi.FaceData[i].offset1));	fprintf(fpout,"%g ",tf); break;
						case  ply::T_INT		:		ti=*( (int    *)        (((char *)fp)+pi.FaceData[i].offset1));	fprintf(fpout,"%i ",ti); break;
						case  ply::T_SHORT	:		ti=*( (short  *)        (((char *)fp)+pi.FaceData[i].offset1));	fprintf(fpout,"%i ",ti); break;
						case  ply::T_CHAR		:		ti=*( (char   *)        (((char *)fp)+pi.FaceData[i].offset1));	fprintf(fpout,"%i ",ti); break;
						case  ply::T_UCHAR	:		ti=*( (unsigned char *) (((char *)fp)+pi.FaceData[i].offset1));	fprintf(fpout,"%i ",ti); break;
						default : assert(0);
						}
					}

					fprintf(fpout,"\n");
				}			  
			}
		}
	assert(fcnt==m.fn);
	int eauxvv[2];
	if( pi.mask & Mask::IOM_EDGEINDEX )
	{
	  int ecnt=0;
	  for(EdgeIterator ei=m.edge.begin();ei!=m.edge.end();++ei)
	  {
		if( ! ei->IsD() )
		{
		  ++ecnt;
		  if(binary)
		  {
			eauxvv[0]=indices[ei->cV(0)];
			eauxvv[1]=indices[ei->cV(1)];
			fwrite(vv,sizeof(int),2,fpout);
		  }
		  else // ***** ASCII *****
			fprintf(fpout,"%d %d \n", indices[ei->cV(0)],	indices[ei->cV(1)]);
		}
	  }
	  assert(ecnt==m.en);
	}
	fclose(fpout);
	return 0;
}

static const char *ErrorMsg(int error)
{
  static std::vector<std::string> ply_error_msg;
  if(ply_error_msg.empty())
  {
    ply_error_msg.resize(PlyInfo::E_MAXPLYINFOERRORS );
    ply_error_msg[ply::E_NOERROR				]="No errors";
	  ply_error_msg[ply::E_CANTOPEN				]="Can't open file";
    ply_error_msg[ply::E_NOTHEADER ]="Header not found";
	  ply_error_msg[ply::E_UNESPECTEDEOF	]="Eof in header";
	  ply_error_msg[ply::E_NOFORMAT				]="Format not found";
	  ply_error_msg[ply::E_SYNTAX				]="Syntax error on header";
	  ply_error_msg[ply::E_PROPOUTOFELEMENT]="Property without element";
	  ply_error_msg[ply::E_BADTYPENAME		]="Bad type name";
	  ply_error_msg[ply::E_ELEMNOTFOUND		]="Element not found";
	  ply_error_msg[ply::E_PROPNOTFOUND		]="Property not found";
	  ply_error_msg[ply::E_BADTYPE				]="Bad type on addtoread";
	  ply_error_msg[ply::E_INCOMPATIBLETYPE]="Incompatible type";
	  ply_error_msg[ply::E_BADCAST				]="Bad cast";

    ply_error_msg[PlyInfo::E_NO_VERTEX      ]="No vertex field found";
    ply_error_msg[PlyInfo::E_NO_FACE        ]="No face field found";
	  ply_error_msg[PlyInfo::E_SHORTFILE      ]="Unespected eof";
	  ply_error_msg[PlyInfo::E_NO_3VERTINFACE ]="Face with more than 3 vertices";
	  ply_error_msg[PlyInfo::E_BAD_VERT_INDEX ]="Bad vertex index in face";
	  ply_error_msg[PlyInfo::E_NO_6TCOORD     ]="Face with no 6 texture coordinates";
	  ply_error_msg[PlyInfo::E_DIFFER_COLORS  ]="Number of color differ from vertices";
  }

  if(error>PlyInfo::E_MAXPLYINFOERRORS || error<0) return "Unknown error";
  else return ply_error_msg[error].c_str();
};

  static int GetExportMaskCapability()
  {
	  int capability = 0;			
	  capability |= vcg::tri::io::Mask::IOM_VERTCOORD    ;
	  capability |= vcg::tri::io::Mask::IOM_VERTFLAGS    ;
	  capability |= vcg::tri::io::Mask::IOM_VERTCOLOR    ;
	  capability |= vcg::tri::io::Mask::IOM_VERTQUALITY  ;
	  capability |= vcg::tri::io::Mask::IOM_VERTNORMAL   ;
	  capability |= vcg::tri::io::Mask::IOM_VERTRADIUS   ;
	  capability |= vcg::tri::io::Mask::IOM_VERTTEXCOORD ;
	  capability |= vcg::tri::io::Mask::IOM_FACEINDEX    ;
	  capability |= vcg::tri::io::Mask::IOM_FACEFLAGS    ;
	  capability |= vcg::tri::io::Mask::IOM_FACECOLOR    ;
	  capability |= vcg::tri::io::Mask::IOM_FACEQUALITY  ;
	  // capability |= vcg::tri::io::Mask::IOM_FACENORMAL   ;
	  capability |= vcg::tri::io::Mask::IOM_WEDGCOLOR    ;
	  capability |= vcg::tri::io::Mask::IOM_WEDGTEXCOORD ;
	  capability |= vcg::tri::io::Mask::IOM_WEDGTEXMULTI ;
    capability |= vcg::tri::io::Mask::IOM_WEDGNORMAL   ;
		capability |= vcg::tri::io::Mask::IOM_CAMERA   ;
		capability |= vcg::tri::io::Mask::IOM_BITPOLYGONAL;
	  return capability;
  }


}; // end class



} // end namespace tri
} // end namespace io
} // end namespace vcg
//@}
#endif
