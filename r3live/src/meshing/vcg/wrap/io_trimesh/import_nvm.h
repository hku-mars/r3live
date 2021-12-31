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
#ifndef __VCGLIB_IMPORTERNVM
#define __VCGLIB_IMPORTERNVM

#include <stddef.h>
#include <stdio.h>
#include <vcg/complex/complex.h>
//#include <vcg/complex/allocate.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <wrap/callback.h>
#include <wrap/io_trimesh/io_mask.h>

#include <QString>
#include <QImageReader>

namespace vcg {
namespace tri {
namespace io {

  /*struct Correspondence{
                Correspondence(unsigned int id_img_,unsigned int key_,float x_,float y_):id_img(id_img_),key(key_),x(x_),y(y_){}
    unsigned int id_img,key;
    float x;
    float y;
  };*/

        typedef std::vector<Correspondence> CorrVec;

/**
This class encapsulate a filter for opening bundler file
*/
template <class OpenMeshType>
class ImporterNVM
{
public:

typedef typename OpenMeshType::VertexPointer VertexPointer;
typedef typename OpenMeshType::ScalarType ScalarType;
typedef typename OpenMeshType::VertexType VertexType;
typedef typename OpenMeshType::FaceType FaceType;
typedef typename OpenMeshType::VertexIterator VertexIterator;
typedef typename OpenMeshType::FaceIterator FaceIterator;
typedef typename OpenMeshType::EdgeIterator EdgeIterator;

static void readline(FILE *fp, char *line, int max=1000){
    int i=0;
    char c;
    fscanf(fp, "%c", &c);
    while( (c!=10) && (c!=13) && (i<max-1) ){
        line[i++] = c;
        fscanf(fp, "%c", &c);
    }
    line[i] = '\0'; //end of string
}

static bool ReadHeader(FILE *fp,unsigned int &num_cams){
    char line[1000];
    readline(fp, line); 
	if( (line[0]=='\0') || (0!=strcmp("NVM_V3 ", line)) ) return false;
	readline(fp, line);
    readline(fp, line); if(line[0]=='\0') return false;
    sscanf(line, "%d", &num_cams);
    return true;
}

static bool ReadHeader(const char * filename,unsigned int &/*num_cams*/, unsigned int &/*num_points*/){
	FILE *fp = fopen(filename, "r");
	if(!fp) return false;
	ReadHeader(fp);
	fclose(fp);
	return true;
}


static int Open( OpenMeshType &m, std::vector<Shot<ScalarType> >  & shots,
                 std::vector<std::string > & image_filenames,
                 const char * filename, CallBackPos *cb=0)
{
  unsigned int   num_cams,num_points;

  FILE *fp = fopen(filename,"r");
  if(!fp) return false;
  ReadHeader(fp, num_cams);
  char line[1000], name[1000];
 /* if(cb) cb(0,"Reading images");
  ReadImagesFilenames(filename_images, image_filenames);*/

  if(cb) cb(50,"Reading cameras");
  //image_filenames.resize(num_cams);
  shots.resize(num_cams);
  for(uint i = 0; i < num_cams;++i)
  {
    float f, k1;
    vcg::Point4f R;
    vcg::Point3f t;

	readline(fp, line); if(line[0]=='\0') return false; sscanf(line, "%s %f %f %f %f %f %f %f %f %f", name, &f, &(R[0]), &(R[1]), &(R[2]), &(R[3]), &(t[0]), &(t[1]), &(t[2]), &k1);

	std::string n(name);
	image_filenames.push_back(n);

	/*readline(fp, line); if(line[0]=='\0') return false; sscanf(line, "%f %f %f", &f, &k1, &k2);

    readline(fp, line); if(line[0]=='\0') return false; sscanf(line, "%f %f %f", &(R[0]), &(R[1]), &(R[2]));  R[3] = 0;
    readline(fp, line); if(line[0]=='\0') return false; sscanf(line, "%f %f %f", &(R[4]), &(R[5]), &(R[6]));  R[7] = 0;
    readline(fp, line); if(line[0]=='\0') return false; sscanf(line, "%f %f %f", &(R[8]), &(R[9]), &(R[10])); R[11] = 0;

    readline(fp, line); if(line[0]=='\0') return false; sscanf(line, "%f %f %f", &(t[0]), &(t[1]), &(t[2]));*/

    //vcg::Matrix44f mat = vcg::Matrix44<vcg::Shotf::ScalarType>::Construct<float>(R);

    vcg::Quaternion<float> qfrom; qfrom.Import(R);

	vcg::Matrix44f mat; qfrom.ToMatrix(mat);

    /*vcg::Matrix33f Rt = vcg::Matrix33f( vcg::Matrix44f(mat), 3);
    Rt.Transpose();

    vcg::Point3f pos = Rt * vcg::Point3f(t[0], t[1], t[2]);*/

	mat[1][0]=-mat[1][0];
	mat[1][1]=-mat[1][1];
	mat[1][2]=-mat[1][2];

	mat[2][0]=-mat[2][0];
	mat[2][1]=-mat[2][1];
	mat[2][2]=-mat[2][2];

    shots[i].Extrinsics.SetTra(vcg::Point3<vcg::Shotf::ScalarType>::Construct<float>(t[0],t[1],t[2]));
    shots[i].Extrinsics.SetRot(mat);

    shots[i].Intrinsics.FocalMm    = f/100.0f;
    shots[i].Intrinsics.k[0] = 0.0;//k1; To be uncommented when distortion is taken into account reliably
    shots[i].Intrinsics.k[1] = 0.0;//k2;
    shots[i].Intrinsics.PixelSizeMm = vcg::Point2f(0.01,0.01);
	QImageReader sizeImg(QString::fromStdString(image_filenames[i]));
	QSize size=sizeImg.size();
	shots[i].Intrinsics.ViewportPx = vcg::Point2i(size.width(),size.height());
	shots[i].Intrinsics.CenterPx[0] = (int)((double)shots[i].Intrinsics.ViewportPx[0]/2.0f);
	shots[i].Intrinsics.CenterPx[1] = (int)((double)shots[i].Intrinsics.ViewportPx[1]/2.0f);
  }
  readline(fp, line);
  readline(fp, line); if(line[0]=='\0') return false;
    sscanf(line, "%d", &num_points);

  // load all correspondences
  typename OpenMeshType::template PerVertexAttributeHandle<CorrVec> ch = vcg::tri::Allocator<OpenMeshType>::template GetPerVertexAttribute<CorrVec>(m,"correspondences");

  typename OpenMeshType::VertexIterator vi = vcg::tri::Allocator<OpenMeshType>::AddVertices(m,num_points);
  for(uint i = 0; i < num_points;++i,++vi){
    float x,y,z;
    unsigned int r,g,b,i_cam, key_sift,n_corr;
    fscanf(fp,"%f %f %f ",&x,&y,&z);
    (*vi).P() = vcg::Point3<typename OpenMeshType::ScalarType>(x,y,z);
    fscanf(fp,"%d %d %d ",&r,&g,&b);
    (*vi).C() = vcg::Color4b(r,g,b,255);

    fscanf(fp,"%d ",&n_corr);
    for(uint j = 0; j < n_corr; ++j){
      fscanf(fp,"%d %d %f %f ",&i_cam,&key_sift,&x,&y);
      Correspondence corr(i_cam,key_sift,x,y);
      ch[i].push_back(corr);
    }
  }
  vcg::tri::UpdateBounding<OpenMeshType>::Box(m);
  fclose(fp);

  return (shots.size() == 0);
}


static bool ReadImagesFilenames(const char *  filename,std::vector<std::string> &image_filenames)
{
	FILE * fp = fopen(filename,"r");
	if (!fp) return false;
	else
	{
		char line[1000], name[1000];
		while(!feof(fp)){
			readline(fp, line, 1000);
			if(line[0] == '\0') continue; //ignore empty lines (in theory, might happen only at end of file)
			sscanf(line, "%s", name);
			std::string n(name);
			image_filenames.push_back(n);
		}
	}
	fclose(fp);
		return true;
}

}; // end class



} // end namespace tri
} // end namespace io
} // end namespace vcg

#endif

