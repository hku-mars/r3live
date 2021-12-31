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

#ifndef __VCGLIB_EXPORTERIDTF
#define __VCGLIB_EXPORTERIDTF


#include <sstream>
#include <fstream>
#include <ostream>
#include <string>
#include <ios>
#include <vcg/space/color4.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <wrap/io_trimesh/io_mask.h>
#include <QString>
#include <QFile>



class TextUtility
{
public:
	template<typename NUMERICTYPE>
	static std::string nmbToStr(NUMERICTYPE n)
	{
		std::stringstream ss;
		ss.setf(std::ios::fixed);
		ss << n;
		ss.setf(std::ios::scientific);
		return ss.str();
	}
};

class Output_File
{
public:
	Output_File(const std::string& file)
		:_file()
	{
		_file.open(file.c_str(),std::ios::out);
	}

	void write(unsigned int tabl,const std::string& st)
	{
		std::string tmp;
		for(unsigned int ii = 0;ii < tabl;++ii)
			tmp += '\t';
		_file << tmp << st << std::endl;
	}

	~Output_File()
	{
		_file.close();
	}

private:
	std::ofstream _file;
	std::string _tab;
};

#include <QString>
#include <QtGlobal>
#include <fstream>
#include <QImage>


namespace vcg {
namespace tri {
namespace io {


namespace QtUtilityFunctions
{
	static void splitFilePath(const QString& filepath,QStringList& trim_path)
	{
		QString file_uniformed = filepath;
		file_uniformed.replace(QString("\\"),QString("/"));
		trim_path = file_uniformed.split("/");
	}

	static QString fileNameFromTrimmedPath(const QStringList& file_path)
	{

		if (file_path.size() > 0)
			return file_path.at(file_path.size() - 1);
		else 
			return QString();
	}

	static QString fileNameFromPath(const QString& filepath)
	{
		QStringList list;
		splitFilePath(filepath,list);
		return fileNameFromTrimmedPath(list);
	}

	static QString pathWithoutFileName(const QString& filepath)
	{
		QString tmp(filepath);
		tmp.remove(fileNameFromPath(filepath));
		return tmp;
	}

	static QString fileExtension(const QString& filepath)
	{
		QStringList trim_list;
		splitFilePath(filepath,trim_list);
		QString file = fileNameFromTrimmedPath(trim_list);
		trim_list = file.split(".");
		return trim_list.at(trim_list.size() - 1);
	}
}

class TGA_Exporter
{
public:

	struct TGAHeader
	{
		unsigned char  identsize;          
		unsigned char  colourmaptype;      
		unsigned char  imagetype;          

		unsigned char colormapspecs[5];

		short xstart;             
		short ystart;             
		short width;              
		short height;             
		unsigned char  bits;               
		unsigned char  descriptor;         
	};

	static void convert(const QString& outfile,const QImage& im)
	{
		TGAHeader tga;
		tga.identsize = 0;
		tga.colourmaptype = 0;
		tga.imagetype = 2;

		memset(tga.colormapspecs,0,5);
		tga.xstart = (short) im.offset().x();
		tga.ystart = (short) im.offset().y();
		tga.height = (short) im.height();
		tga.width =  (short) im.width();
		

		//QString moutfile = QString("C:/Users/Guido/AppData/Local/Temp/duckCM.tga");

		QFile file(qPrintable(outfile));
		file.setPermissions(QFile::WriteOther);
		file.open(QIODevice::WriteOnly);
		QString err = file.errorString();
		//bool val = file.failbit;
		unsigned char* tmpchan;
		int totbyte;
		if (im.hasAlphaChannel())
		{
			//is a 8-digits binary number code  
			// always 0 0  |  mirroring | bits 
			//(future uses)|  image     | for alpha-channel
			//--------------------------------------------			
			//     7 6     |      5 4   |      3 2 1 0
			//--------------------------------------------
			//     0 0     |      1 0   |      1 0 0 0
			tga.descriptor = (char) 40;
			tga.bits = (char) 32;
		}
		else
		{
			//is a 8-digits binary number code  
			// always 0 0  |  mirroring | bits 
			//(future uses)|  image     | for alpha-channel
			//--------------------------------------------			
			//     7 6     |      5 4   |      3 2 1 0
			//--------------------------------------------
			//     0 0     |      1 0   |      0 0 0 0
			tga.descriptor = (char) 32;
			tga.bits = (char) 24;
		}

		totbyte = tga.height * tga.width * (tga.bits / 8);

		if (im.hasAlphaChannel())
			tmpchan = const_cast<unsigned char*>(im.bits());
		else
		{
			tmpchan = new unsigned char[totbyte];

			int ii = 0;
			while(ii < totbyte)
			{
				tmpchan[ii] = const_cast<unsigned char*>(im.bits())[ii + (ii/3)];
				++ii;
			}
		}

		file.write((char *) &tga,qint64(sizeof(tga)));
		file.write(reinterpret_cast<const char*>(tmpchan),qint64(totbyte));
		file.close();
	}
	
	template<typename SaveMeshType>
	static void convertTexturesFiles(SaveMeshType& m,const QString& file_path,QStringList& conv_file)
	{
		for(unsigned int ii = 0; ii < m.textures.size(); ++ii)
		{
			QString qtmp(m.textures[ii].c_str());
			QString ext = QtUtilityFunctions::fileExtension(qtmp);
			QString filename = QtUtilityFunctions::fileNameFromPath(qtmp);
			if (ext.toLower() != "tga")
			{
				QImage img(qtmp);
				QString stmp;
				if ((file_path.at(file_path.length() - 1) != '/') || (file_path.at(file_path.length() - 1) != '\\'))
					stmp = file_path + QString("/");
				else
					stmp = file_path;
				filename = stmp + filename.remove(ext) + "tga";
				m.textures[ii] = filename.toStdString();
				TGA_Exporter::convert(filename,img);
				conv_file.push_back(filename);
			}
		}
	}

	static void removeConvertedTexturesFiles(const QStringList& conv_file)
	{
        for(QStringList::size_type ii = 0;ii < conv_file.size();++ii)
		{
			QDir dir(QtUtilityFunctions::pathWithoutFileName(conv_file[ii]));
			dir.remove(QtUtilityFunctions::fileNameFromPath(conv_file[ii]));
		}
	}
};



template<typename SaveMeshType>
class ExporterIDTF
{

public:
typedef typename SaveMeshType::VertexPointer VertexPointer;
typedef typename SaveMeshType::ScalarType ScalarType;
typedef typename SaveMeshType::VertexType VertexType;
typedef typename SaveMeshType::FaceType FaceType;
typedef typename SaveMeshType::ConstVertexIterator ConstVertexIterator;
typedef typename SaveMeshType::VertexIterator VertexIterator;
typedef typename SaveMeshType::FaceIterator FaceIterator;
typedef typename SaveMeshType::ConstFaceIterator ConstFaceIterator;
typedef typename SaveMeshType::CoordType CoordType;

	enum IDTFError 
	{
		E_NOERROR				// 0
	};

	static const char *ErrorMsg(int error)
	{
		static const char * dae_error_msg[] =
		{
			"No errors"
		};

		if(error>0 || error<0) return "Unknown error";
		else return dae_error_msg[error];
	};
	
	static QStringList convertInTGATextures(SaveMeshType& m,const QString& path,QStringList& textures_to_be_restored)
	{
		//if there are textures file that aren't in tga format I have to convert them
		//I maintain the converted file name (i.e. file_path + originalname without extension + tga) in mesh.textures but I have to revert to the original ones
		//before the function return. 
		for(unsigned int ii = 0; ii < m.textures.size();++ii)
			textures_to_be_restored.push_back(m.textures[ii].c_str());

		//tmp vector to save the tga created files that should be deleted.
		QStringList convfile;
		vcg::tri::io::TGA_Exporter::convertTexturesFiles(m,path,convfile);
		return convfile;
	}

	static void removeConvertedTGATextures(const QStringList& convfile)
	{
		//if some tga files have been created I have to delete them
		vcg::tri::io::TGA_Exporter::removeConvertedTexturesFiles(convfile);
	}

	static void restoreConvertedTextures(SaveMeshType& mesh_with_textures_to_be_restored,const QStringList& textures_to_be_restored)
	{
		mesh_with_textures_to_be_restored.textures.clear();
		for(QStringList::ConstIterator it = textures_to_be_restored.begin();it != textures_to_be_restored.end();++it)
			mesh_with_textures_to_be_restored.textures.push_back(it->toStdString());
	}

	static int Save(SaveMeshType& m,const char* file,const int mask)
	{
		
		Output_File idtf(file);

		idtf.write(0,"FILE_FORMAT \"IDTF\"");
		idtf.write(0,"FORMAT_VERSION 100\n");

		idtf.write(0,"NODE \"MODEL\" {");
		idtf.write(1,"NODE_NAME \"VcgMesh01\"");
		idtf.write(1,"PARENT_LIST {");
		idtf.write(2,"PARENT_COUNT 1");
		idtf.write(2,"PARENT 0 {");
		idtf.write(3,"PARENT_NAME \"<NULL>\"");
		idtf.write(3,"PARENT_TM {");
		idtf.write(4,"1.000000 0.000000 0.000000 0.000000");
		idtf.write(4,"0.000000 1.000000 0.000000 0.000000");
		idtf.write(4,"0.000000 0.000000 1.000000 0.000000");
		idtf.write(4,"0.000000 0.000000 0.000000 1.000000");
		idtf.write(3,"}");
		idtf.write(2,"}");
		idtf.write(1,"}");
		idtf.write(1,"RESOURCE_NAME \"MyVcgMesh01\"");
		idtf.write(0,"}");


		if ((mask & vcg::tri::io::Mask::IOM_WEDGTEXCOORD) | (mask & vcg::tri::io::Mask::IOM_VERTCOLOR) | (mask & vcg::tri::io::Mask::IOM_FACECOLOR))
		{
			idtf.write(0,"");
			idtf.write(0,"RESOURCE_LIST \"SHADER\" {");
			idtf.write(1,"RESOURCE_COUNT " + TextUtility::nmbToStr(m.textures.size()));
			for(unsigned int ii = 0; ii < m.textures.size(); ++ii)
			{
				idtf.write(1,"RESOURCE " + TextUtility::nmbToStr(ii) + " {");
				idtf.write(2,"RESOURCE_NAME \"ModelShader" + TextUtility::nmbToStr(ii) +"\"");
				std::string vertcol;
				if (mask & vcg::tri::io::Mask::IOM_VERTCOLOR)
					vertcol = "TRUE";
				else
					vertcol = "FALSE";

				idtf.write(2,"ATTRIBUTE_USE_VERTEX_COLOR \"" + vertcol + "\"");
				idtf.write(2,"SHADER_MATERIAL_NAME \"Mat01\"");


				int texcount = 0;
				if (mask & vcg::tri::io::Mask::IOM_WEDGTEXCOORD)
					texcount = m.textures.size();

				
				idtf.write(2,"SHADER_ACTIVE_TEXTURE_COUNT " + TextUtility::nmbToStr(texcount));
				if (mask & vcg::tri::io::Mask::IOM_WEDGTEXCOORD)
				{
					idtf.write(2,"SHADER_TEXTURE_LAYER_LIST {");
					idtf.write(3,"TEXTURE_LAYER 0 {");
					idtf.write(4,"TEXTURE_NAME \"Texture" + TextUtility::nmbToStr(ii) +"\"");
					idtf.write(3,"}");
					idtf.write(2,"}");
				}

                idtf.write(1,"}");
			}
			idtf.write(0,"}");
			idtf.write(0,"");
		}

		if ((mask & Mask::IOM_WEDGTEXCOORD) | (mask & vcg::tri::io::Mask::IOM_VERTCOLOR) | (mask & vcg::tri::io::Mask::IOM_FACECOLOR))
		{
			idtf.write(0,"RESOURCE_LIST \"MATERIAL\" {");
			idtf.write(1,"RESOURCE_COUNT 1");
			idtf.write(1,"RESOURCE 0 {");
			idtf.write(2,"RESOURCE_NAME \"Mat01\"");
			idtf.write(2,"MATERIAL_AMBIENT 0.2 0.2 0.2");
			idtf.write(2,"MATERIAL_DIFFUSE 0.8 0.8 0.8");
			idtf.write(2,"MATERIAL_SPECULAR 0.0 0.0 0.0");
			idtf.write(2,"MATERIAL_EMISSIVE 0.0 0.0 0.0");
			idtf.write(2,"MATERIAL_REFLECTIVITY 0.000000");
			idtf.write(2,"MATERIAL_OPACITY 1.000000");
			idtf.write(1,"}");
			idtf.write(0,"}");
			idtf.write(0,"");
			if ((mask & Mask::IOM_WEDGTEXCOORD))
			{
				idtf.write(0,"RESOURCE_LIST \"TEXTURE\" {");
				idtf.write(1,"RESOURCE_COUNT " + TextUtility::nmbToStr(m.textures.size()));
				for(unsigned int ii = 0; ii < m.textures.size();++ii)
				{
					idtf.write(1,"RESOURCE " + TextUtility::nmbToStr(ii) + " {");
					idtf.write(2,"RESOURCE_NAME \"Texture" + TextUtility::nmbToStr(ii) + "\"");
					idtf.write(2,"TEXTURE_PATH \"" + m.textures[ii] + "\"");
					idtf.write(1,"}");
				}
				idtf.write(0,"}");
			}

		}
		idtf.write(0,"");
		idtf.write(0,"RESOURCE_LIST \"MODEL\" {");
		idtf.write(1,"RESOURCE_COUNT 1");
		idtf.write(1,"RESOURCE 0 {");
		idtf.write(2,"RESOURCE_NAME \"MyVcgMesh01\"");
		idtf.write(2,"MODEL_TYPE \"MESH\"");
		idtf.write(2,"MESH {");
		idtf.write(3,"FACE_COUNT " + TextUtility::nmbToStr(m.face.size()));
		idtf.write(3,"MODEL_POSITION_COUNT " + TextUtility::nmbToStr(m.vert.size()));
		idtf.write(3,"MODEL_NORMAL_COUNT " + TextUtility::nmbToStr(m.face.size() * 3));
		if ((mask & vcg::tri::io::Mask::IOM_VERTCOLOR) | (mask & vcg::tri::io::Mask::IOM_FACECOLOR))
			idtf.write(3,"MODEL_DIFFUSE_COLOR_COUNT " + TextUtility::nmbToStr(m.face.size() * 3));
		else 
			idtf.write(3,"MODEL_DIFFUSE_COLOR_COUNT 0");
		idtf.write(3,"MODEL_SPECULAR_COLOR_COUNT 0");
		if (mask & vcg::tri::io::Mask::IOM_WEDGTEXCOORD) idtf.write(3,"MODEL_TEXTURE_COORD_COUNT " + TextUtility::nmbToStr(m.face.size() * 3));
		else idtf.write(3,"MODEL_TEXTURE_COORD_COUNT 0");
		idtf.write(3,"MODEL_BONE_COUNT 0");
		unsigned int mod_sha;
		if (m.textures.size() == 0)
			mod_sha = 1;
		else
			mod_sha = m.textures.size();
		idtf.write(3,"MODEL_SHADING_COUNT " + TextUtility::nmbToStr(mod_sha));
		idtf.write(3,"MODEL_SHADING_DESCRIPTION_LIST {");
		unsigned int hh = 0;
		do
		{
			idtf.write(4,"SHADING_DESCRIPTION " + TextUtility::nmbToStr(hh) + " {");
			if (mask & vcg::tri::io::Mask::IOM_WEDGTEXCOORD)
			{
				idtf.write(5,"TEXTURE_LAYER_COUNT 1");
				idtf.write(5,"TEXTURE_COORD_DIMENSION_LIST {");
				idtf.write(6,"TEXTURE_LAYER 0	DIMENSION: 2");
				idtf.write(5,"}");
				idtf.write(5,"SHADER_ID 0");
			}
			else 
			{
				idtf.write(5,"TEXTURE_LAYER_COUNT 0");
				idtf.write(5,"SHADER_ID 0");
			}
			idtf.write(4,"}");
			++hh;
		}
		while(hh < m.textures.size());
		idtf.write(3,"}");
		idtf.write(3,"MESH_FACE_POSITION_LIST {");
		for(ConstFaceIterator fit = m.face.begin();fit != m.face.end();++fit)  
		{
			idtf.write(4,TextUtility::nmbToStr(fit->cV(0) - &(*m.vert.begin())) + " " +
				TextUtility::nmbToStr(fit->cV(1) - &(*m.vert.begin())) + " " +
				TextUtility::nmbToStr(fit->cV(2) - &(*m.vert.begin())));
		}
		idtf.write(3,"}");

		idtf.write(3,"MESH_FACE_NORMAL_LIST {");
		unsigned int nn = 0;
		for(ConstFaceIterator fit = m.face.begin();fit != m.face.end();++fit)  
		{
			idtf.write(4,TextUtility::nmbToStr(nn) + " " +
				TextUtility::nmbToStr(nn + 1) + " " + 
				TextUtility::nmbToStr(nn + 2));
			nn += 3;
		}
		idtf.write(3,"}");

		idtf.write(3,"MESH_FACE_SHADING_LIST {");
		for(FaceIterator fit = m.face.begin();fit != m.face.end();++fit)  
		{
			unsigned int texind = 0;
			if (mask & vcg::tri::io::Mask::IOM_WEDGTEXCOORD) 
				texind = fit->WT(0).N();
			idtf.write(4,TextUtility::nmbToStr(texind));
		}
		idtf.write(3,"}");

		if (mask & vcg::tri::io::Mask::IOM_WEDGTEXCOORD) 
		{
			idtf.write(3,"MESH_FACE_TEXTURE_COORD_LIST {"); 
			for(unsigned int ii = 0; ii < m.face.size();++ii)
			{
				idtf.write(4,"FACE " + TextUtility::nmbToStr(ii) + " {");
				idtf.write(5,"TEXTURE_LAYER 0 TEX_COORD: " + TextUtility::nmbToStr(ii * 3) + " " + TextUtility::nmbToStr(ii * 3 + 1) + " " + TextUtility::nmbToStr(ii * 3 + 2));
				idtf.write(4,"}");
			}
			idtf.write(3,"}");
		}

		if ((mask & vcg::tri::io::Mask::IOM_VERTCOLOR) | (mask & vcg::tri::io::Mask::IOM_FACECOLOR))
		{
			idtf.write(3,"MESH_FACE_DIFFUSE_COLOR_LIST {");
			nn = 0;
			for(FaceIterator fit = m.face.begin();fit != m.face.end();++fit)  
			{
				idtf.write(4,TextUtility::nmbToStr(nn) + " " +
					TextUtility::nmbToStr(nn + 1) + " " + 
					TextUtility::nmbToStr(nn + 2));
				nn += 3;
			}
			idtf.write(3,"}");
		}

		idtf.write(3,"MODEL_POSITION_LIST {");
		//vcg::tri::UpdateBounding<SaveMeshType>::Box(m);
		//ScalarType diag = m.bbox.Diag();
		//CoordType center = m.bbox.Center();
		for(ConstVertexIterator vit = m.vert.begin();vit != m.vert.end();++vit)  
		{
			CoordType tmp = vit->P();// - center);// /diag;
			idtf.write(4,TextUtility::nmbToStr(-tmp.X()) + " " +
				TextUtility::nmbToStr(tmp.Z()) + " " + 
				TextUtility::nmbToStr(tmp.Y()));
		}
		idtf.write(3,"}");

		idtf.write(3,"MODEL_NORMAL_LIST {");
		for(FaceIterator fitn = m.face.begin();fitn != m.face.end();++fitn)  
		{
			for(unsigned int ii = 0;ii < 3;++ii)
			{
				fitn->N().Normalize();
				idtf.write(4,TextUtility::nmbToStr(-fitn->N().X()) + " " +
					TextUtility::nmbToStr(fitn->N().Z()) + " " + 
					TextUtility::nmbToStr(fitn->N().Y()));
			}
		}
		idtf.write(3,"}");

		if ((mask & vcg::tri::io::Mask::IOM_VERTCOLOR) | (mask & vcg::tri::io::Mask::IOM_FACECOLOR))
		{
			idtf.write(3,"MODEL_DIFFUSE_COLOR_LIST {");
			//ScalarType diag = m.bbox.Diag();
			//CoordType center = m.bbox.Center();
			for(FaceIterator vit = m.face.begin();vit != m.face.end();++vit)  
			{
				
				for (unsigned int ii =0; ii <3;++ii)
				{
					vcg::Color4b cc;
					if (mask & vcg::tri::io::Mask::IOM_VERTCOLOR)
						cc = vit->V(ii)->C();
					else
						cc = vit->C();
					idtf.write(4,TextUtility::nmbToStr(float(cc.X()) / 255.0f) + " " +
						TextUtility::nmbToStr(float(cc.Y()) / 255.0f) + " " + 
						TextUtility::nmbToStr(float(cc.Z()) / 255.0f) + " " + TextUtility::nmbToStr(float(cc.W()) / 255.0f));
				}
			}
			idtf.write(3,"}");
		}

		if (mask & vcg::tri::io::Mask::IOM_WEDGTEXCOORD)
		{
			idtf.write(3,"MODEL_TEXTURE_COORD_LIST {");
			for(FaceIterator fitn = m.face.begin();fitn != m.face.end();++fitn)  
			{
				for(unsigned int ii = 0;ii < 3;++ii)
				{
					idtf.write(4,TextUtility::nmbToStr(fitn->WT(ii).U()) + " " +
						TextUtility::nmbToStr(-fitn->WT(ii).V()) + " " + TextUtility::nmbToStr(0.0f) + " " + TextUtility::nmbToStr(0.0f));
				}
			}
			idtf.write(3,"}");
		}

		idtf.write(2,"}");
		idtf.write(1,"}");
		idtf.write(0,"}");

		//if (!(mask & vcg::tri::io::Mask::IOM_WEDGTEXCOORD) & (mask & vcg::tri::io::Mask::IOM_VERTCOLOR) | (mask & vcg::tri::io::Mask::IOM_FACECOLOR))
		//{
		//	idtf.write(0,"RESOURCE_LIST \"SHADER\" {");
		//	idtf.write(1,"RESOURCE_COUNT 1");
		//	idtf.write(1,"RESOURCE 0 {");
		//	idtf.write(2,"RESOURCE_NAME \"VcgMesh010\"");
		//	

		//	//WARNING! IF VERTEX COLOR AND FACE COLOR ARE BOTH CHECKED THE FILTER SAVE ONLY VERTEX COLOR! THIS IS A DESIGN CHOICE A NOT A BUG!

		//	std::string vertcol;
		//	if (mask & vcg::tri::io::Mask::IOM_VERTCOLOR)
		//		vertcol = "TRUE";
		//	else
		//		vertcol = "FALSE";
		//	
		//	idtf.write(2,"ATTRIBUTE_USE_VERTEX_COLOR \"" + vertcol + "\"");
		//	idtf.write(2,"SHADER_MATERIAL_NAME \"Mat01\"");
		//	idtf.write(2,"SHADER_ACTIVE_TEXTURE_COUNT 0");
		//	idtf.write(1,"}");
		//	idtf.write(0,"}");
		//}

		if ((mask & vcg::tri::io::Mask::IOM_WEDGTEXCOORD) | (mask & vcg::tri::io::Mask::IOM_VERTCOLOR) | (mask & vcg::tri::io::Mask::IOM_FACECOLOR))
		{
			idtf.write(0,"");
			idtf.write(0,"MODIFIER \"SHADING\" {");
			idtf.write(1,"MODIFIER_NAME \"VcgMesh01\"");
			idtf.write(1,"PARAMETERS {");

			idtf.write(2,"SHADER_LIST_COUNT " + TextUtility::nmbToStr(m.textures.size()));
			idtf.write(2,"SHADING_GROUP {");
			for(unsigned int ii = 0; ii < m.textures.size();++ii)
			{
				idtf.write(3,"SHADER_LIST " +  TextUtility::nmbToStr(ii) + "{");
				idtf.write(4,"SHADER_COUNT 1");
				idtf.write(4,"SHADER_NAME_LIST {");
				idtf.write(5,"SHADER 0 NAME: \"ModelShader" + TextUtility::nmbToStr(ii) + "\"");
				idtf.write(4,"}");
				idtf.write(3,"}");
			}
			idtf.write(2,"}");
			idtf.write(1,"}");
			idtf.write(0,"}");
		}
	
		return E_NOERROR;
	}

	static int GetExportMaskCapability()
	{
		int capability = 0;

		//vert
		capability |= vcg::tri::io::Mask::IOM_VERTNORMAL;
		capability |= vcg::tri::io::Mask::IOM_VERTCOLOR;

		//face
		capability |= vcg::tri::io::Mask::IOM_FACECOLOR;

		////wedg
		capability |= vcg::tri::io::Mask::IOM_WEDGTEXCOORD;
		capability |= vcg::tri::io::Mask::IOM_WEDGNORMAL;

		return capability;
	}
};
}
}
}

#endif
