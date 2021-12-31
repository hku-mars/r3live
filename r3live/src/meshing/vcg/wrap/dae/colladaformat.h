/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2004-2008                                           \/)\/    *
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
#ifndef _COLLADA_FORMAT_H
#define _COLLADA_FORMAT_H

#include <wrap/dae/xmldocumentmanaging.h>
#include <vcg/space/point4.h>
#include <vcg/space/color4.h>

#include <QtGui/QImage>
#include <QtCore/QVector>

#include <QDateTime>




template<typename POINTTYPE>
struct CoordNumber{public:	static unsigned int coord()	{		return 0; 	}};

template<> struct CoordNumber<vcg::Point2f> { public:	static unsigned int coord() { return 2;	} };
template<> struct CoordNumber<vcg::Point3f> { public:	static unsigned int coord() {	return 3;	} };
template<> struct CoordNumber<vcg::Point4f> { public: static unsigned int coord() {	return 4;	} };

template<> struct CoordNumber<vcg::Color4b> { public:	static unsigned int coord() { return 4;	} };


namespace Collada
{
namespace Tags
{
	static const QString testSharp(const QString& str)
	{
		QString sharp = "";
		if (str.at(0) != '#')
			sharp = '#';
		return (sharp + str);
	}

	class ColladaTag : public XMLTag
	{
	public:
		ColladaTag()
			:XMLTag("COLLADA")
		{
			_attributes.push_back(TagAttribute("xmlns","http://www.collada.org/2005/11/COLLADASchema"));
			_attributes.push_back(TagAttribute("version","1.4.1"));
		}
	};

	class AssetTag : public XMLTag
	{
	public:
		AssetTag()
			:XMLTag("asset")
		{
		}
	};

	class ContributorTag : public XMLTag
	{
	public:
		ContributorTag()
			:XMLTag("contributor")
		{
		}
	};

	class AuthorTag : public XMLLeafTag
	{
	public:
		AuthorTag()
			:XMLLeafTag("author")
		{
			_text.push_back("VCGLab");
		}
	};

	class AuthoringToolTag : public XMLLeafTag
	{
	public:
		AuthoringToolTag()
			:XMLLeafTag("authoring_tool")
		{
			_text.push_back("VCGLib | MeshLab");
		}
	};

	class UpAxisTag : public XMLLeafTag
	{
	public:
		UpAxisTag(const QString& up = "Y_UP")
			:XMLLeafTag("up_axis")
		{
			_text.push_back(up);
		}
	};

	class LibraryImagesTag : public XMLTag
	{
	public:
		LibraryImagesTag()
			:XMLTag("library_images")
		{
		}
	};

	class ImageTag : public XMLTag
	{
	public:
		ImageTag(const QString& id,const QString& name)
			:XMLTag("image")
		{
			_attributes.push_back(TagAttribute("id",id));
			_attributes.push_back(TagAttribute("name",name));
		}
	};

	class InitFromTag : public XMLLeafTag
	{
	public:
		InitFromTag(const QString& txtpathname)
			:XMLLeafTag("init_from")
		{
			_text.push_back(txtpathname);
		}
	};

	class LibraryMaterialsTag : public XMLTag
	{
	public:
		LibraryMaterialsTag()
			:XMLTag("library_materials")
		{
		}
	};
	
	class MaterialTag : public XMLTag
	{
	public:
		MaterialTag(const QString& id,const QString& name)
			:XMLTag("material")
		{
			_attributes.push_back(TagAttribute("id",id));
			_attributes.push_back(TagAttribute("name",name));
		}
	};

	class InstanceEffectTag : public XMLLeafTag
	{
	public:
		InstanceEffectTag(const QString& url)
			:XMLLeafTag("instance_effect")
		{
			_attributes.push_back(TagAttribute("url",testSharp(url)));
		}
	};

	class LibraryEffectsTag : public XMLTag
	{
	public:
		LibraryEffectsTag()
			:XMLTag("library_effects")
		{
		}
	};

	class EffectTag : public XMLTag
	{
	public:
		EffectTag(const QString& id)
			:XMLTag("effect")
		{
			_attributes.push_back(TagAttribute("id",id));
		}
	};
	
	class ProfileCommonTag : public XMLTag
	{
	public:
		ProfileCommonTag()
			:XMLTag("profile_COMMON")
		{
		}
	};

	class NewParamTag : public XMLTag
	{
	public:
		NewParamTag(const QString& sid)
			:XMLTag("newparam")
		{
			_attributes.push_back(TagAttribute("sid",sid));
		}
	};

	class SurfaceTag : public XMLTag
	{
	public:
		SurfaceTag(const QString& type = QString("2D"))
			:XMLTag("surface")
		{
			_attributes.push_back(TagAttribute("type",type));
		}
	};

	class FormatTag : public XMLLeafTag
	{
	public:
		FormatTag(const QString& format)
			:XMLLeafTag("format")
		{
			_text.push_back(format);
		}
	};
	
	class Sampler2DTag : public XMLTag
	{
	public:
		Sampler2DTag()
			:XMLTag("sampler2D")
		{
		}
	};

	class SourceTag : public XMLLeafTag
	{
	public:
		SourceTag(const QString& id,const QString& name)
			:XMLLeafTag("source")
		{
			_attributes.push_back(TagAttribute("id",id));
			_attributes.push_back(TagAttribute("name",name));
		}

		SourceTag(const QString& source)
			:XMLLeafTag("source")
		{
			_text.push_back(source);
		}
	};

	class MinFilterTag : public XMLLeafTag
	{
	public:
		MinFilterTag(const QString& filter)
			:XMLLeafTag("minfilter")
		{
			_text.push_back(filter);
		}
	};

	class MagFilterTag : public XMLLeafTag
	{
	public:
		MagFilterTag(const QString& filter)
			:XMLLeafTag("magfilter")
		{
			_text.push_back(filter);
		}
	};

	class TechniqueTag : public XMLTag
	{
	public:
		TechniqueTag(const QString& sid)
			:XMLTag("technique")
		{
			_attributes.push_back(TagAttribute("sid",sid));
		}
	};

	class TechniqueCommonTag : public XMLTag
	{
	public:
		TechniqueCommonTag()
			:XMLTag("technique_common")
		{
		}
	};

	class BlinnTag : public XMLTag
	{
	public:
		BlinnTag()
			:XMLTag("blinn")
		{
		}
	};

	class EmissionTag : public XMLTag
	{
	public:
		EmissionTag()
			:XMLTag("emission")
		{
		}
	};

	class ColorTag : public XMLLeafTag
	{
	public:
		ColorTag(const float r,const float g,const float b,const float a)
			:XMLLeafTag("color")
		{
			
			_text.push_back(QString::number(r));
			_text.push_back(QString::number(g));
			_text.push_back(QString::number(b));
			_text.push_back(QString::number(a));
		}
	};

	class AmbientTag : public XMLTag
	{
	public:
		AmbientTag()
			:XMLTag("ambient")
		{
		}
	};

	class DiffuseTag : public XMLTag
	{
	public:
		DiffuseTag()
			:XMLTag("diffuse")
		{
		}
	};

	class TextureTag : public XMLLeafTag
	{
	public:
		TextureTag(const QString& texture,const QString& texcoord)
			:XMLLeafTag("texture")
		{
			_attributes.push_back(TagAttribute("texture",texture));
			_attributes.push_back(TagAttribute("texcoord",texcoord));
		}
	};

	class SpecularTag : public XMLTag
	{
	public:
		SpecularTag()
			:XMLTag("specular")
		{
		}
	};

	class ShininessTag : public XMLTag
	{
	public:
		ShininessTag()
			:XMLTag("shininess")
		{
		}
	};

	class FloatTag : public XMLLeafTag
	{
	public:
		FloatTag(const float floatnum)
			:XMLLeafTag("float")
		{
			_text.push_back(QString::number(floatnum));
		}
	};

	class ReflectiveTag : public XMLTag
	{
	public:
		ReflectiveTag()
			:XMLTag("reflective")
		{
		}
	};

	class ReflectivityTag : public XMLTag
	{
	public:
		ReflectivityTag()
			:XMLTag("reflectivity")
		{
		}
	};

	class TransparentTag : public XMLTag
	{
	public:
		TransparentTag()
			:XMLTag("transparent")
		{
		}
	};

	class TransparencyTag : public XMLTag
	{
	public:
		TransparencyTag()
			:XMLTag("transparency")
		{
		}
	};

	class IndexOfRefractionTag : public XMLTag
	{
	public:
		IndexOfRefractionTag()
			:XMLTag("index_of_refraction")
		{
		}
	};

	class LibraryGeometriesTag : public XMLTag
	{
	public:
		LibraryGeometriesTag()
			:XMLTag("library_geometries")
		{
		}
	};
	
	class GeometryTag : public XMLTag
	{
	public:
		GeometryTag(const QString& id,const QString& name)
			:XMLTag("geometry")
		{
			_attributes.push_back(TagAttribute("id",id));
			_attributes.push_back(TagAttribute("name",name));
		}
	};

	class MeshTag : public XMLTag
	{
	public:
		MeshTag()
			:XMLTag("mesh")
		{
		}
	};

	class ArraySourceTag : public XMLTag
	{
	public:
		ArraySourceTag(const QString& id,const QString& name)
			:XMLTag("source")
		{
			_attributes.push_back(TagAttribute("id",id));
			_attributes.push_back(TagAttribute("name",name));
		}
	};

	class FloatArrayTag : public XMLLeafTag
	{
	public:
		enum ARRAYSEMANTIC {VERTPOSITION,VERTNORMAL,VERTCOLOR, FACENORMAL,WEDGETEXCOORD};

		template<typename MESHTYPE>
		FloatArrayTag(const QString& id,const int count,const MESHTYPE& m,ARRAYSEMANTIC sem,const unsigned int componenttype)
			:XMLLeafTag("float_array")
		{
			_attributes.push_back(TagAttribute("id",id));
			_attributes.push_back(TagAttribute("count",QString::number(count)));

			if ((sem == VERTPOSITION) || (sem == VERTNORMAL) || (sem == VERTCOLOR))
			{
				for(typename MESHTYPE::ConstVertexIterator vit = m.vert.begin();vit != m.vert.end();++vit)
				{
					for(unsigned int ii = 0; ii < componenttype;++ii)
					{
						if (sem == VERTPOSITION)
							_text.push_back(QString::number(vit->P()[ii]));
						else if (sem == VERTCOLOR)
							_text.push_back(QString::number((vit->C()[ii])/255.0));
						else
						{
							typename MESHTYPE::VertexType::NormalType r = vit->cN();
							r.Normalize();
							_text.push_back(QString::number(r[ii]));
					
						}
					}
				}
			}
			else
			{
				for(typename MESHTYPE::ConstFaceIterator fit = m.face.begin();fit != m.face.end();++fit)
				{
					if (sem == FACENORMAL)
					{
						for(unsigned int ii = 0; ii < componenttype;++ii)
						{
								typename MESHTYPE::FaceType::NormalType r = fit->cN();
								r.Normalize();
								_text.push_back(QString::number(r[ii]));
						}
					}
					else
					{
						for(unsigned int ii = 0; ii < 3;++ii)
						{				
							_text.push_back(QString::number(fit->cWT(ii).U()));
							_text.push_back(QString::number(fit->cWT(ii).V()));
						}
					}
				}
			}
		}
	};

	//class FloatWedgeArrayTag : public XMLLeafTag
	//{
	//public:
	//	template<typename MESHTYPE,typename SIMPLEXACCESSOR>
	//	FloatWedgeArrayTag(const QString& id,const int count,const MESHTYPE& m,const AccessorComponentNumberInfo<MESHTYPE,SIMPLEXACCESSOR>& accessor)
	//		:XMLLeafTag("float_array")
	//	{
	//		_attributes.push_back(TagAttribute("id",id));
	//		_attributes.push_back(TagAttribute("count",QString::number(count)));
	//		for(typename SIMPLEXACCESSOR::ConstIterator it= accessor._a.begin();it != accessor._a.end(); ++it)
	//		{
	//			for(unsigned int ii = 0; ii < 3;++ii)
	//			{				
	//				_text.push_back(QString::number(accessor._a(*it,ii).U()));
	//				_text.push_back(QString::number(accessor._a(*it,ii).V()));
	//			}
	//		}
	//	}
	//};

	class AccessorTag : public XMLTag
	{
	public:
		AccessorTag(const int count,const QString& source,const int stride)
			:XMLTag("accessor")
		{
			_attributes.push_back(TagAttribute("count",QString::number(count)));
			_attributes.push_back(TagAttribute("source",testSharp(source)));
			_attributes.push_back(TagAttribute("stride",QString::number(stride)));
		}
	};

	class ParamTag : public XMLTag
	{
	public:
		ParamTag(const QString& name,const QString& type)
			:XMLTag("param")
		{
			_attributes.push_back(TagAttribute("name",name));
			_attributes.push_back(TagAttribute("type",type));
		}
	};

	class VerticesTag : public XMLTag
	{
	public:
		VerticesTag(const QString& id)
			:XMLTag("vertices")
		{
			_attributes.push_back(TagAttribute("id",id));
		}
	};

	class InputTag : public XMLTag
	{
	public:

		InputTag(const QString& semantic,const QString& source)
			:XMLTag("input")
		{
			_attributes.push_back(TagAttribute("semantic",semantic));
			_attributes.push_back(TagAttribute("source",testSharp(source)));
		}

		InputTag(const int offset,const QString& semantic,const QString& source)
			:XMLTag("input")
		{
			_attributes.push_back(TagAttribute("offset",QString::number(offset)));
			_attributes.push_back(TagAttribute("semantic",semantic));
			_attributes.push_back(TagAttribute("source",testSharp(source)));
		}
	};

	class TrianglesTag : public XMLTag
	{
	public:
		TrianglesTag(const int count)
			:XMLTag("triangles")
		{
			_attributes.push_back(TagAttribute("count",QString::number(count)));
		}

		TrianglesTag(const int count,const QString& material)
			:XMLTag("triangles")
		{
			_attributes.push_back(TagAttribute("count",QString::number(count)));
			_attributes.push_back(TagAttribute("material",material));
		}
	};

	class PTag : public XMLLeafTag
	{
	public:
		template<typename MESHTYPE>
		PTag(const MESHTYPE& m,const unsigned int nedge,bool vcol=false, bool norm = false,bool texcoord = false)
			:XMLLeafTag("p")
		{
			int cont = 0;
			for(typename MESHTYPE::ConstFaceIterator it= m.face.begin();it != m.face.end(); ++it)
			{
				for(unsigned int ii = 0; ii < nedge; ++ii)
				{
					int dist  = vcg::tri::Index(m,it->cV(ii));
					_text.push_back(QString::number(dist));
					if (vcol)
						_text.push_back(QString::number(dist));
					if (norm)
						_text.push_back(QString::number(cont));
					if (texcoord)
						_text.push_back(QString::number(cont * nedge + ii));
				}
				++cont;
			}
		}

		template<typename MESHTYPE>
		PTag(const MESHTYPE& m,const unsigned int nedge,QVector<int>& patchfaces,bool vcol = false, bool norm = false,bool texcoord = false)
			:XMLLeafTag("p")
		{
			int cont = 0;
			for(QVector<int>::iterator it = patchfaces.begin();it != patchfaces	.end(); ++it)
			{
				for(unsigned int ii = 0; ii < nedge; ++ii)
				{
					const typename MESHTYPE::FaceType& f = m.face[*it];
					int dist  = f.cV(ii) - &(*m.vert.begin());
					_text.push_back(QString::number(dist));
					if (vcol)
						_text.push_back(QString::number(dist));
					if (norm)
						_text.push_back(QString::number(*it));
					if (texcoord)
						_text.push_back(QString::number(*it * nedge + ii));
				}
				++cont;
			}
		}
	};

	class LibraryVisualScenesTag : public XMLTag
	{
	public:
		LibraryVisualScenesTag()
			:XMLTag("library_visual_scenes")
		{
		}
	};

	class VisualSceneTag : public XMLTag
	{
	public:
		VisualSceneTag(const QString& id,const QString& name)
			:XMLTag("visual_scene")
		{
			_attributes.push_back(TagAttribute("id",id));
			_attributes.push_back(TagAttribute("name",name));
		}
	};
	
	class NodeTag : public XMLTag
	{
	public:
		NodeTag(const QString& id,const QString& name)
			:XMLTag("node")
		{
			_attributes.push_back(TagAttribute("id",id));
			_attributes.push_back(TagAttribute("name",name));
		}
	};

	class RotateTag : public XMLLeafTag
	{
	public:
		RotateTag(const QString& sid,const vcg::Point4f& p)
			:XMLLeafTag("rotate")
		{
			_attributes.push_back(TagAttribute("sid",sid));

			for(unsigned int ii =0;ii < 4; ++ii)
				_text.push_back(QString::number(p[ii]));
		}
	};

	class TranslateTag : public XMLLeafTag
	{
	public:
		TranslateTag(const QString& sid,const vcg::Point4f& p)
			:XMLLeafTag("translate")
		{
			_attributes.push_back(TagAttribute("sid",sid));

			for(unsigned int ii =0;ii < 4; ++ii)
				_text.push_back(QString::number(p[ii]));
		}
	}; 

	class InstanceGeometryTag : public XMLTag
	{
	public:
		InstanceGeometryTag(const QString& url)
			:XMLTag("instance_geometry")
		{
			_attributes.push_back(TagAttribute("url",testSharp(url)));
		}
	};

	class BindMaterialTag : public XMLTag
	{
	public:
		BindMaterialTag()
			:XMLTag("bind_material")
		{
		}
	};

	class InstanceMaterialTag : public XMLTag
	{
	public:
		InstanceMaterialTag(const QString& symbol,const QString& target)
			:XMLTag("instance_material")
		{
			_attributes.push_back(TagAttribute("symbol",symbol));
			_attributes.push_back(TagAttribute("target",testSharp(target)));
		}
	};

	class BindVertexInputTag : public XMLTag
	{
	public:
		BindVertexInputTag(const QString& semantic,const QString& input_semantic,const QString& input_set)
			:XMLTag("bind_vertex_input")
		{
			_attributes.push_back(TagAttribute("semantic",semantic));
			_attributes.push_back(TagAttribute("input_semantic",input_semantic));
		}
	};

	class SceneTag : public XMLTag
	{
	public:
		SceneTag()
			:XMLTag("scene")
		{
		}
	};

	class CreatedTag : public XMLLeafTag//added
	{
	public:
		CreatedTag()
			:XMLLeafTag("created")
		{
			QDateTime dateCreated = QDateTime::currentDateTime().toUTC();
			QString dateCreatedStr = dateCreated.toString();
			_text.push_back(dateCreatedStr);
		}
	};

	class ModifiedTag : public XMLLeafTag//added
	{
	public:
		ModifiedTag()
			:XMLLeafTag("modified")
		{
			QDateTime dateModified = QDateTime::currentDateTime().toUTC();
			QString dateModifiedStr = dateModified.toString();
			_text.push_back(dateModifiedStr);
		}
	};

	class InstanceVisualSceneTag : public XMLTag
	{
	public:
		InstanceVisualSceneTag(const QString& url)
			:XMLTag("instance_visual_scene")
		{
			_attributes.push_back(TagAttribute("url",testSharp(url)));
		}
	};
} //Tags

class DocumentManager
{
private:
	static void connectHierarchyNode(XMLInteriorNode* node0,XMLInteriorNode* node1,XMLLeafNode* leaf)
	{
		node1->_sons.push_back(leaf);
		node0->_sons.push_back(node1);
	}

	static void connectHierarchyNode(XMLInteriorNode* node0,XMLInteriorNode* node1,XMLInteriorNode* node2,XMLInteriorNode* node3,XMLNode* node4)
	{
		node3->_sons.push_back(node4);
		node2->_sons.push_back(node3);
		node1->_sons.push_back(node2);
		node0->_sons.push_back(node1);
	}

	static void connectHierarchyNode(XMLInteriorNode* node0,XMLInteriorNode* node1,XMLInteriorNode* node2,XMLInteriorNode* node3)
	{
		node2->_sons.push_back(node3);
		node1->_sons.push_back(node2);
		node0->_sons.push_back(node1);
	}

	static void connectHierarchyNode(XMLInteriorNode* node0,XMLInteriorNode* node1,XMLInteriorNode* node2)
	{
		node1->_sons.push_back(node2);
		node0->_sons.push_back(node1);
	}

	template<typename MESHMODELTYPE>
	static void splitMeshInTexturedPatches(const MESHMODELTYPE& m,QVector<QVector<int> >& patches)
	{
		patches.resize(m.textures.size());
		int cc = 0;
		for(typename MESHMODELTYPE::ConstFaceIterator itf = m.face.begin();itf != m.face.end();++itf)
		{
			int tmp = itf->cWT(0).N();
			assert(tmp>=0 && tmp<patches.size());	
			patches[tmp].push_back(cc);
			++cc;
		}
	}

public:
	template<typename MESHMODELTYPE>
	static XMLDocument* createColladaDocument(const MESHMODELTYPE& m,const int mask)
	{
		//for now we export only triangularface
		const unsigned int edgefacenum = 3;
		typedef XMLInteriorNode XNode;
		typedef XMLLeafNode XLeaf;

		XNode* root = new XNode(new Tags::ColladaTag());
		XNode* assetnode = new XNode(new Tags::AssetTag());
		XNode* contributornode = new XNode(new Tags::ContributorTag());
		contributornode->_sons.push_back(new XLeaf(new Tags::AuthorTag()));
		contributornode->_sons.push_back(new XLeaf(new Tags::AuthoringToolTag()));
		
		assetnode->_sons.push_back(contributornode);
		assetnode->_sons.push_back(new XLeaf(new Tags::CreatedTag()));//added
		assetnode->_sons.push_back(new XLeaf(new Tags::ModifiedTag()));
		assetnode->_sons.push_back(new XLeaf(new Tags::UpAxisTag()));
		root->_sons.push_back(assetnode);
		
		XNode* libimages = NULL;
		for(unsigned int ii = 0;ii < m.textures.size();++ii)
		{
			if ( ii == 0)
				libimages = new XNode(new Tags::LibraryImagesTag());
			QString subfix = QString::number(ii); 
			XNode* imagenode = new XNode(new Tags::ImageTag(QString("texture") + subfix,QString("texture") + subfix));
			XLeaf* initfromnode = new XLeaf(new Tags::InitFromTag(QString::fromStdString(m.textures[ii])));
			imagenode->_sons.push_back(initfromnode);
			libimages->_sons.push_back(imagenode);
			if (ii == 0)
				root->_sons.push_back(libimages);
		}

		XNode* libmaterials = NULL;
		for(unsigned int ii = 0;ii < m.textures.size();++ii)
		{
			if ( ii == 0)
				libmaterials = new XNode(new Tags::LibraryMaterialsTag());
			QString subfix = QString::number(ii); 
			QString mat = "material" + subfix;
			XNode* materialnode = new XNode(new Tags::MaterialTag(mat,mat));
			XLeaf* instanceeff = new XLeaf(new Tags::InstanceEffectTag(mat+"-fx"));
			materialnode->_sons.push_back(instanceeff);
			libmaterials->_sons.push_back(materialnode);
			if ( ii == 0)
				root->_sons.push_back(libmaterials);
		}

		XNode* libeffects = NULL;
		for(unsigned int ii = 0;ii < m.textures.size();++ii)
		{
			if ( ii == 0)
				libeffects = new XNode(new Tags::LibraryEffectsTag());
			QString subfix = QString::number(ii); 
			QString mat = "material" + subfix + "-fx";
			XNode* effectnode = new XNode(new Tags::EffectTag(mat));
			XNode* procommnode = new XNode(new Tags::ProfileCommonTag());
			QString tex = QString("texture")+subfix;
			XNode* newparamnode = new XNode(new Tags::NewParamTag(tex+"-surface"));
			XNode* surfacenode = new XNode(new Tags::SurfaceTag());
			XLeaf* initfromnode = new XLeaf(new Tags::InitFromTag(tex));
			QImage img(QString::fromStdString(m.textures[ii]));
			QImage::Format f = img.format();
			QString form = "R8G8B8";
			if (f==QImage::Format_ARGB32)
				form = "A8R8G8B8";
			XLeaf* formatnode = new XLeaf(new Tags::FormatTag(form));
			surfacenode->_sons.push_back(initfromnode);
			surfacenode->_sons.push_back(formatnode);
			newparamnode->_sons.push_back(surfacenode);
			procommnode->_sons.push_back(newparamnode);

			XNode* newparamnode2 = new XNode(new Tags::NewParamTag(tex+"-sampler"));
			XNode* samplernode = new XNode(new Tags::Sampler2DTag());
			XLeaf* sourcenode = new XLeaf(new Tags::SourceTag(tex+"-surface"));
			XLeaf* minfilt = new XLeaf(new Tags::MinFilterTag("LINEAR"));
			XLeaf* magfilt = new XLeaf(new Tags::MagFilterTag("LINEAR"));
			samplernode->_sons.push_back(sourcenode);
			samplernode->_sons.push_back(minfilt);
			samplernode->_sons.push_back(magfilt);
			newparamnode2->_sons.push_back(samplernode);
			procommnode->_sons.push_back(newparamnode2);

			XNode* technode = new XNode(new Tags::TechniqueTag("common"));
			XNode* blinnnode = new XNode(new Tags::BlinnTag());
			
			XNode* diffusenode = new XNode(new Tags::DiffuseTag());
			XLeaf* texturenode = new XLeaf(new Tags::TextureTag(tex+"-sampler","UVSET0"));

			connectHierarchyNode(blinnnode,diffusenode,texturenode);
			connectHierarchyNode(procommnode,technode,blinnnode);

			effectnode->_sons.push_back(procommnode);
			libeffects->_sons.push_back(effectnode);
			if ( ii == 0)
				root->_sons.push_back(libeffects);
		}

		XNode* libgeo = new XNode(new Tags::LibraryGeometriesTag());
		QString subfix = "0"; 
		QString shape = "shape" + subfix;
		XNode* geometrynode = new XNode(new Tags::GeometryTag(shape+"-lib",shape));
		XNode* meshnode = new XNode(new Tags::MeshTag());
		XNode* sourcepos = new XNode(new Tags::SourceTag(shape+"-lib-positions","position"));
		//AccessorComponentNumberInfo<MESHMODELTYPE,MeshAccessors::VertexPositionAccessor<const MESHMODELTYPE>> acc(m);
		unsigned int return_component_number = CoordNumber<typename MESHMODELTYPE::CoordType>::coord();
		XLeaf* floatarr = new XLeaf(new Tags::FloatArrayTag(shape+"-lib-positions-array",m.vert.size() * return_component_number,m,Tags::FloatArrayTag::VERTPOSITION,return_component_number));
		XNode* techcommnode = new XNode(new Tags::TechniqueCommonTag());
		XNode* accessornode = new XNode(new Tags::AccessorTag(m.vert.size(),shape+"-lib-positions-array",return_component_number));
		XNode* paramx = new XNode(new Tags::ParamTag("X","float"));
		XNode* paramy = new XNode(new Tags::ParamTag("Y","float"));
		XNode* paramz = new XNode(new Tags::ParamTag("Z","float"));
		
		sourcepos->_sons.push_back(floatarr);
		accessornode->_sons.push_back(paramx);
		accessornode->_sons.push_back(paramy);
		accessornode->_sons.push_back(paramz);

		techcommnode->_sons.push_back(accessornode);
		sourcepos->_sons.push_back(techcommnode);

		meshnode->_sons.push_back(sourcepos);
		

		//CHANGE THIS PIECE OF CODE!
		bool normalmask = bool((mask & vcg::tri::io::Mask::IOM_FACENORMAL) || (mask & vcg::tri::io::Mask::IOM_WEDGNORMAL) || (mask & vcg::tri::io::Mask::IOM_VERTNORMAL));
		if (normalmask)
		{
			XNode* sourcenormal = new XNode(new Tags::SourceTag(shape+"-lib-normals","normal"));

			//we export only triangular face
			XLeaf* floatnormarr = new XLeaf(new Tags::FloatArrayTag(shape+"-lib-normals-array",m.face.size() * return_component_number,m,Tags::FloatArrayTag::FACENORMAL,return_component_number));
			XNode* techcommnormnode = new XNode(new Tags::TechniqueCommonTag());
			XNode* accessornormnode = new XNode(new Tags::AccessorTag(m.face.size(),shape+"-lib-normals-array",return_component_number));
			
			//I have to make up the following piece of code
			XNode* paramnormx = new XNode(new Tags::ParamTag("X","float"));
			XNode* paramnormy = new XNode(new Tags::ParamTag("Y","float"));
			XNode* paramnormz = new XNode(new Tags::ParamTag("Z","float"));

			sourcenormal->_sons.push_back(floatnormarr);

			accessornormnode->_sons.push_back(paramnormx);
			accessornormnode->_sons.push_back(paramnormy);
			accessornormnode->_sons.push_back(paramnormz);

			techcommnormnode->_sons.push_back(accessornormnode);
			sourcenormal->_sons.push_back(techcommnormnode);

			meshnode->_sons.push_back(sourcenormal);
		}

		//CHANGE THIS PIECE OF CODE!
		bool vcolormask = bool((mask & vcg::tri::io::Mask::IOM_VERTCOLOR));
		if (vcolormask)
		{
			unsigned int color_component_number = 4;
			XNode* sourcevcolor = new XNode(new Tags::SourceTag(shape+"-lib-vcolor","vcolor"));

			//we export only triangular face
			XLeaf* floatvcolorarr = new XLeaf(new Tags::FloatArrayTag(shape+"-lib-vcolor-array",m.vert.size() * color_component_number,m,Tags::FloatArrayTag::VERTCOLOR,color_component_number));
			XNode* techcommvcolornode = new XNode(new Tags::TechniqueCommonTag());
			XNode* accessorvcolornode = new XNode(new Tags::AccessorTag(m.vert.size(),shape+"-lib-vcolor-array",color_component_number));
			
			//I have to make up the following piece of code
			XNode* paramvcolorx = new XNode(new Tags::ParamTag("R","float"));
			XNode* paramvcolory = new XNode(new Tags::ParamTag("G","float"));
			XNode* paramvcolorz = new XNode(new Tags::ParamTag("B","float"));
			XNode* paramvcolora = new XNode(new Tags::ParamTag("A","float"));

			sourcevcolor->_sons.push_back(floatvcolorarr);

			accessorvcolornode->_sons.push_back(paramvcolorx);
			accessorvcolornode->_sons.push_back(paramvcolory);
			accessorvcolornode->_sons.push_back(paramvcolorz);
			accessorvcolornode->_sons.push_back(paramvcolora);

			techcommvcolornode->_sons.push_back(accessorvcolornode);
			sourcevcolor->_sons.push_back(techcommvcolornode);

			meshnode->_sons.push_back(sourcevcolor);
		}
		
		bool texmask = bool(mask & vcg::tri::io::Mask::IOM_WEDGTEXCOORD);
		if (texmask)
		{
			XNode* sourcewedge = new XNode(new Tags::SourceTag(shape+"-lib-map","map"));
			return_component_number = CoordNumber<typename MESHMODELTYPE::FaceType::TexCoordType::PointType>::coord();
			//we export only triangular face
			XLeaf* floatwedgearr = new XLeaf(new Tags::FloatArrayTag(shape+"-lib-map-array",m.face.size() * return_component_number * edgefacenum,m,Tags::FloatArrayTag::WEDGETEXCOORD,return_component_number));
			XNode* techcommwedgenode = new XNode(new Tags::TechniqueCommonTag());
			XNode* accessorwedgenode = new XNode(new Tags::AccessorTag(m.face.size() * edgefacenum,shape+"-lib-map-array",return_component_number));

			//I have to make up the following piece of code
			XNode* paramwedgeu = new XNode(new Tags::ParamTag("U","float"));
			XNode* paramwedgev = new XNode(new Tags::ParamTag("V","float"));

			sourcewedge->_sons.push_back(floatwedgearr);

			accessorwedgenode->_sons.push_back(paramwedgeu);
			accessorwedgenode->_sons.push_back(paramwedgev);
		
			techcommwedgenode->_sons.push_back(accessorwedgenode);
			sourcewedge->_sons.push_back(techcommwedgenode);

			meshnode->_sons.push_back(sourcewedge);
		}

		XNode* vertnode = new XNode(new Tags::VerticesTag(shape+"-lib-vertices"));
		XNode* inputposnode = new XNode(new Tags::InputTag("POSITION",shape+"-lib-positions"));
		vertnode->_sons.push_back(inputposnode);
	
		//XNode* inputvcolnode = new XNode(new Tags::InputTag("COLOR",shape+"-lib-vcolor"));
		//vertnode->_sons.push_back(inputvcolnode);

		meshnode->_sons.push_back(vertnode);
		XNode* trianglesnode = NULL;
		//if ((m.textures.size() == 0) || (!texmask))
		//{	
		//	//there isn't any texture file
		//	trianglesnode = new XNode(new Tags::TrianglesTag(m.face.size()));
		//}
		//else
		//{
		//	std::vector<std::vector<int>> _mytripatches(m.textures.size());
		//	if ((texmask) && (m.textures.size() > 1))
		//	{
		//		//there are many textures files - I have to split the mesh in triangular patches that share the same texture's file.
		//		for(MESHMODELTYPE::ConstFaceIterator itf = m.face.begin();itf != m.face.end();++itf)
		//		{
		//			
		//		}
		//		trianglesnode = new XNode(new Tags::TrianglesTag(m.face.size(),"instancematerial"));
		//}
		
		QVector<QVector<int> > mytripatches;
		if ((texmask) && (m.textures.size() != 0))
			splitMeshInTexturedPatches(m,mytripatches);
		
		QVector<QVector<int> >::iterator itp = mytripatches.begin();
		int indmat = 0;
		do 
		{
			if ((m.textures.size() == 0) || (!texmask))
			{	
				//there isn't any texture file
				trianglesnode = new XNode(new Tags::TrianglesTag(m.face.size()));
			}
			else
				trianglesnode = new XNode(new Tags::TrianglesTag(mytripatches[indmat].size(),"material" + QString::number(indmat)));

			XNode* inputtrinode = new XNode(new Tags::InputTag(0,"VERTEX",shape+"-lib-vertices"));
			trianglesnode->_sons.push_back(inputtrinode);

			int offs=1;
			if (vcolormask)
			{
				XNode* inputvcolnode = new XNode(new Tags::InputTag(offs,"COLOR",shape+"-lib-vcolor"));
				trianglesnode->_sons.push_back(inputvcolnode);
				++offs;
			}

			if (normalmask)
			{
				XNode* inputnormnode = new XNode(new Tags::InputTag(offs,"NORMAL",shape+"-lib-normals"));
				trianglesnode->_sons.push_back(inputnormnode);
				++offs;
			}

			if (texmask)
			{
				XNode* inputwedgenode = new XNode(new Tags::InputTag(offs,"TEXCOORD",shape+"-lib-map"));
				trianglesnode->_sons.push_back(inputwedgenode);
				offs++;
			}

			XLeaf* polyleaf = NULL;
			if (itp == mytripatches.end())
				polyleaf = new XLeaf(new Tags::PTag(m,edgefacenum,vcolormask,normalmask,texmask));
			else
				polyleaf = new XLeaf(new Tags::PTag(m,edgefacenum,(*itp),vcolormask,normalmask,texmask));

			trianglesnode->_sons.push_back(polyleaf);
			meshnode->_sons.push_back(trianglesnode);
			
			++indmat;
			if (itp != mytripatches.end()) 
				++itp;
		}while(itp != mytripatches.end());
		
		connectHierarchyNode(libgeo,geometrynode,meshnode);
		root->_sons.push_back(libgeo);

		XNode* libvisualnode = new XNode(new Tags::LibraryVisualScenesTag());
		XNode* visualscenenode = new XNode(new Tags::VisualSceneTag("VisualSceneNode","VisualScene"));
		XNode* nodenode = new XNode(new Tags::NodeTag("node","node"));
		XNode* instgeonode = new XNode(new Tags::InstanceGeometryTag(shape+"-lib"));
		if (m.textures.size() > 0)
		{
			XNode* bindnode = new XNode(new Tags::BindMaterialTag());
			XNode* techcommmatnode = new XNode(new Tags::TechniqueCommonTag());
			for(unsigned int ii = 0; ii < m.textures.size(); ++ii)
			{
				XNode* instmatnode = new XNode(new Tags::InstanceMaterialTag("material"  + QString::number(ii),"material" + QString::number(ii)));
				XNode* bindvertnode = new XNode(new Tags::BindVertexInputTag("UVSET0","TEXCOORD","0"));
				connectHierarchyNode(techcommmatnode,instmatnode,bindvertnode);
			}
			connectHierarchyNode(visualscenenode,nodenode,instgeonode,bindnode,techcommmatnode);
		}
		else
			connectHierarchyNode(visualscenenode,nodenode,instgeonode);
		libvisualnode->_sons.push_back(visualscenenode);
		root->_sons.push_back(libvisualnode);

		XNode* scenenode = new XNode(new Tags::SceneTag());
		XNode* instvisualscenenode = new XNode(new Tags::InstanceVisualSceneTag("VisualSceneNode"));

		scenenode->_sons.push_back(instvisualscenenode);
		root->_sons.push_back(scenenode);

		return new XMLDocument(root);
	}

	static void destroyColladaDocument(XMLDocument* doc)
	{
		delete doc;
	}

	//template<typename MESHMODELTYPE>
	//static int importColladaDocument(const QDomDocument& doc,MESHMODELTYPE& m,const int mask)
	//{
	//	QDomElement root = doc.toElement();
	//	if (root.isNull())
	//		return UtilDAE::E_BAD_CONVERTION_FROM_NODE_TO_ELEMENT;
	//	QDomNodeList lst = root.elementsByTagName("COLLADA");
	//	int err = UtilDAE::checkOccurencies(lst,UtilDAE::ONE);
	//	if (
	//	return vcg::tri::io::UtilDAE::E_NOERROR; 
	//}
};
} //Collada
#endif
