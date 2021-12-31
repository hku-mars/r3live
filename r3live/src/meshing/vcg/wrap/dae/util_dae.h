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
#ifndef __VCGLIB_UTILDAE
#define __VCGLIB_UTILDAE


#include <vcg/complex/append.h>
#include <wrap/io_trimesh/additionalinfo.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/position.h>

#include <wrap/io_trimesh/io_mask.h>

#include <QDomDocument>
#include <QFile>
#include <QXmlStreamWriter>
#include <QStringList>
#include <QMap>

#include <vcg/space/point3.h>
#include <vcg/space/texcoord2.h>
#include <vcg/space/color4.h>
#include <vcg/space/texcoord2.h>
#include <wrap/callback.h>

#include <vector>

namespace vcg {
namespace tri {
namespace io {
	class InfoDAE  : public AdditionalInfo
	{
		public:
		
		InfoDAE() :AdditionalInfo(){
			doc = NULL;
			textureIdMap.clear();
		}

		~InfoDAE(){
			if(doc!=NULL) delete doc;
		}

		QDomDocument* doc;		
		QMap<QString,int> textureIdMap;
	};

	class UtilDAE
	{
	public:
		enum DAEError 
		{
			E_NOERROR,				// 0
			E_CANTOPEN,				// 1
			E_NOGEOMETRYLIBRARY,     // 2 
			E_NOMESH,      // 3
			E_NOVERTEXPOSITION,            // 4
			E_NO3DVERTEXPOSITION,			// 5
			E_NO3DSCENE, // 6
			E_INCOMPATIBLECOLLADA141FORMAT, //7
			E_UNREFERENCEBLEDCOLLADAATTRIBUTE, // 8
			E_NOPOLYGONALMESH, //9
			E_CANTSAVE, //10
			E_NOACCESSORELEMENT
		};
		

		static const char *ErrorMsg(int error)
		{
			static const char * dae_error_msg[] =
			{
				"No errors",
				"Can't open file",
				"File without a geometry library",
				"There isn't mesh in file",
				"The meshes in file haven't the vertex position attribute",
				"The importer assumes that the OpenMeshType uses a 3D point for the vertex position",
				"There isn't any scene in Collada file",
				"The input file is not compatible with COLLADA 1.41 standard format",
				"Collada file is trying to referece an attribute that is not in the file",
				"This version of Collada Importer support only triangular and polygonal mesh file"
			};

			if(error>9 || error<0) return "Unknown error";
			else return dae_error_msg[error];
		};
	protected:
		// This function take care of removing the standard '#' from the beginning of the url
		// For example 
	  // <instance_geometry url="#shape0-lib">
		// means that you have to search in the <library_geometries> for a node <geometry> with id = "shape0-lib"
		// you have to call this function to get ther right refernece name

		inline static void referenceToANodeAttribute(const QDomNode n,const QString& attr,QString& url_st)
		{
			url_st = n.toElement().attribute(attr);
			int sz = url_st.size() - 1;
			url_st = url_st.right(sz);
			assert(url_st.size() != 0);
		}

		inline static QDomNode findNodeBySpecificAttributeValue(const QDomNodeList& ndl,const QString& attrname,const QString& attrvalue)
		{
			int ndl_size = ndl.size();
			int ind = 0;
			while(ind < ndl_size)
			{
				QString st = ndl.at(ind).toElement().attribute(attrname);
				if (st == attrvalue)
					return ndl.at(ind);
				++ind;
			}
			return QDomNode();
		}

		inline static QDomNode findNodeBySpecificAttributeValue(const QDomNode n,const QString& tag,const QString& attrname,const QString& attrvalue)
		{
			return findNodeBySpecificAttributeValue(n.toElement().elementsByTagName(tag),attrname,attrvalue);
		}

		inline static QDomNode findNodeBySpecificAttributeValue(const QDomDocument n,const QString& tag,const QString& attrname,const QString& attrvalue)
		{
			return findNodeBySpecificAttributeValue(n.elementsByTagName(tag),attrname,attrvalue);
		}
		
		inline static bool isThereTag(const QDomNodeList& list)
		{
			return ((list.size() > 0) ? true : false);
		}

		inline static bool isThereTag(const QDomNode n,const QString& tagname)
		{
			return isThereTag(n.toElement().elementsByTagName(tagname));
		}

		inline static bool isThereTag(const QDomDocument n,const QString& tagname)
		{
			return isThereTag(n.elementsByTagName(tagname));
		}

    // Very important function that given a <vertices> element find one of the its attribute (like position, color etc)
		inline static QDomNode attributeSourcePerSimplex(const QDomNode n,const QDomDocument startpoint,const QString& sem)
		{
			QDomNodeList vertattr = n.toElement().elementsByTagName("input");
			for(int ind = 0;ind < vertattr.size();++ind)
			{
				if (vertattr.at(ind).toElement().attribute("semantic") == sem)
				{
					QString url; 
					referenceToANodeAttribute(vertattr.at(ind),"source",url);
					return findNodeBySpecificAttributeValue(startpoint,"source","id",url);
				}
			}
			return QDomNode();
		}

		// This function is used to build up a list of strings that are scalar values.
		inline static void valueStringList(QStringList& res,const QDomNode srcnode,const QString& tag) 
		{
			QDomNodeList list = srcnode.toElement().elementsByTagName(tag);
			//assert(list.size() == 1);
			QString nd = list.at(0).firstChild().nodeValue();
			res = nd.simplified().split(" ",QString::SkipEmptyParts);
            if(res.empty())
                {
                    qDebug("Warning valueStringList returned and emtpy list. nothing inside element with tag '%s'", qPrintable(tag));
                    return;
                }
			if (res.last() == "")
				res.removeLast();
		
//			int emptyCount = res.removeAll(QString(""));
//			if(emptyCount>0) qDebug("- - - - - - - - valueStringList: Removed %i null strings when parsing tag %s",emptyCount,qPrintable(tag));
//			for(int i =0;i<res.size();++i)
//				qDebug("- - - - - - - - - - - - %3i = '%s'",i,qPrintable(res.at(i)));
		
		}

		/*inline static bool removeChildNode(QDomNodeList*/
		
		inline static bool removeChildNodeList(QDomNodeList& nodelst,const QString& tag = "", const QString& attribname = "", const QString& attribvalue = "")
		{
			for(int jj = 0;jj < nodelst.size();++jj)
			{
				removeChildNode(nodelst.at(jj),tag,attribname,attribvalue); 
			}
			return true;
		}


		inline static bool removeChildNode(QDomNode node,const QString& tag = "", const QString& attribname = "", const QString& attribvalue = "")
		{
			QDomNodeList clst = node.childNodes();
			for(int ii = 0;ii < clst.size();++ii)
			{
				QDomNode oldchild = node.childNodes().at(ii); 
				if (tag != "")
				{
					if ((attribname != "") && (attribvalue != ""))
					{
						if (clst.at(ii).toElement().attribute(attribname) == attribvalue)
							node.removeChild(oldchild);
					}
					else 
					{	
						QString nm = clst.at(ii).nodeName();
						if (clst.at(ii).nodeName() == tag) 
						{
							node.removeChild(oldchild);
						}
					}
				}
				else node.removeChild(oldchild);
			}
			return true;
		}

		static void ParseRotationMatrix(vcg::Matrix44f& m,const std::vector<QDomNode>& t)
		{
			vcg::Matrix44f rotTmp;
			vcg::Matrix44f tmp;
			rotTmp.SetIdentity();
			tmp.SetIdentity();
			for(unsigned int ii = 0;ii < t.size();++ii)
			{
				QString rt = t[ii].firstChild().nodeValue();
				QStringList rtl = rt.split(" ");
				if (rtl.last() == "") rtl.removeLast();
				assert(rtl.size() == 4);
				tmp.SetRotateDeg(rtl.at(3).toFloat(),vcg::Point3f(rtl.at(0).toFloat(),rtl.at(1).toFloat(),rtl.at(2).toFloat()));
				rotTmp = rotTmp*tmp;	
			}
			m = m * rotTmp;
		}

		static void ParseTranslation(vcg::Matrix44f& m,const QDomNode t)
		{
			assert(t.toElement().tagName() == "translate");
			QDomNode tr = t.firstChild();
			QString coord = tr.nodeValue();
			QStringList coordlist = coord.split(" ");
			if (coordlist.last() == "") 
				coordlist.removeLast();
			assert(coordlist.size() == 3);
			m[0][0] = 1.0f;
			m[1][1] = 1.0f;
			m[2][2] = 1.0f;
			m[3][3] = 1.0f;
			m[0][3] = coordlist.at(0).toFloat();
			m[1][3] = coordlist.at(1).toFloat();
			m[2][3] = coordlist.at(2).toFloat();
		}
		
		static void ParseMatrixNode(vcg::Matrix44f& m,const QDomNode t)
		{
			assert(t.toElement().tagName() == "matrix");
			QDomNode tr = t.firstChild();
			QString coord = tr.nodeValue().simplified();
			qDebug("Parsing matrix node; text value is '%s'",qPrintable(coord));
			QStringList coordlist = coord.split(" ");
			if (coordlist.last() == "") 
				coordlist.removeLast();
			assert(coordlist.size() == 16);
			for(int i=0;i<4;++i)
			{
				m[i][0] = coordlist.at(i*4+0).toFloat();
				m[i][1] = coordlist.at(i*4+1).toFloat();
				m[i][2] = coordlist.at(i*4+2).toFloat();
				m[i][3] = coordlist.at(i*4+3).toFloat();
			}
		}

		static void TransfMatrix(const QDomNode parentnode,const QDomNode presentnode,vcg::Matrix44f& m)
		{
			if (presentnode == parentnode) return;
			else
			{
				QDomNode par = presentnode.parentNode();
				std::vector<QDomNode> rotlist;
				QDomNode trans;
				for(int ch = 0;ch < par.childNodes().size();++ch)
				{
					if (par.childNodes().at(ch).nodeName() == "rotate")
						rotlist.push_back(par.childNodes().at(ch));
					else if (par.childNodes().at(ch).nodeName() == "translate")
						 {
							trans = par.childNodes().at(ch);
					     }		
				}
				vcg::Matrix44f tmp;
				tmp.SetIdentity();
				if (!trans.isNull()) ParseTranslation(tmp,trans);
				ParseRotationMatrix(tmp,rotlist);
				m = m * tmp;
				TransfMatrix(parentnode,par,m);
			}
		}

		inline static int findOffSetForASingleSimplex(QDomNode node)
		{
			QDomNodeList wedatts = node.toElement().elementsByTagName("input");
			int max = 0;
			if (wedatts.size() == 0) return -1;
			else 
			{
				for(int ii = 0;ii < wedatts.size();++ii)
				{
					int tmp = wedatts.at(ii).toElement().attribute("offset").toInt();
					if (tmp > max) max = tmp;
				}
			}
			return max + 1;
		}

		inline static int findStringListAttribute(QStringList& list,const QDomNode node,const QDomNode poly,const QDomDocument startpoint,const char* token)
		{
			int offset = 0;
			if (!node.isNull())
			{
				offset = node.toElement().attribute("offset").toInt();
				QDomNode st = attributeSourcePerSimplex(poly,startpoint,token);
				valueStringList(list,st,"float_array");
			}
			return offset;
		}

		
		
		
		
		/* Very important procedure 
			it has the task to finde the name of the image node corresponding to a given material id, 
			it assuemes that the material name that is passed have already been bound with the current bindings  
		*/
		
		inline static QDomNode textureFinder(QString& boundMaterialName, QString &textureFileName, const QDomDocument doc)
		{
			boundMaterialName.remove('#');
			//library_material -> material -> instance_effect
			QDomNodeList lib_mat = doc.elementsByTagName("library_materials");
			if (lib_mat.size() != 1) 
				return QDomNode();
			QDomNode material = findNodeBySpecificAttributeValue(lib_mat.at(0),QString("material"),QString("id"),boundMaterialName);
			if (material.isNull()) 
				return QDomNode();
			QDomNodeList in_eff = material.toElement().elementsByTagName("instance_effect");
			if (in_eff.size() == 0) 
				return QDomNode();
			QString url = in_eff.at(0).toElement().attribute("url");
			if ((url.isNull()) || (url == ""))
				return QDomNode();
			url = url.remove('#');
      qDebug("====== searching among library_effects the effect with id '%s' ",qPrintable(url));
			//library_effects -> effect -> instance_effect
			QDomNodeList lib_eff = doc.elementsByTagName("library_effects");
			if (lib_eff.size() != 1) 
				return QDomNode();
			QDomNode effect = findNodeBySpecificAttributeValue(lib_eff.at(0),QString("effect"),QString("id"),url);
			if (effect.isNull()) 
				return QDomNode();
			QDomNodeList init_from = effect.toElement().elementsByTagName("init_from");
			if (init_from.size() == 0)
				return QDomNode();
			QString img_id = init_from.at(0).toElement().text();
			if ((img_id.isNull()) || (img_id == ""))
				return QDomNode();
			
			//library_images -> image
			QDomNodeList libraryImageNodeList = doc.elementsByTagName("library_images");
			qDebug("====== searching among library_images the effect with id '%s' ",qPrintable(img_id));
			if (libraryImageNodeList.size() != 1) 
				return QDomNode();
			QDomNode imageNode = findNodeBySpecificAttributeValue(libraryImageNodeList.at(0),QString("image"),QString("id"),img_id);
			QDomNodeList initfromNode = imageNode.toElement().elementsByTagName("init_from");
			textureFileName= initfromNode.at(0).firstChild().nodeValue();
			qDebug("====== the image '%s' has a %i init_from nodes text '%s'",qPrintable(img_id),initfromNode.size(),qPrintable(textureFileName));
			
			return imageNode;			
		}

		static int indexTextureByImgNode(const QDomDocument doc,const QDomNode node)
		{	
			QDomNodeList libim = doc.elementsByTagName(QString("library_images"));
			if (libim.size() != 1)
				return -1;
			QDomNodeList imgs = libim.at(0).toElement().elementsByTagName("image");
			
			int ii = 0;
			bool found = false;
			while((ii < imgs.size()) && (!found))
			{
				if (imgs.at(ii) == node) 
					found = true;
				else ++ii;
			}
			if (found) 
				return ii;
			else
				return -1;
		}

		struct WedgeAttribute
		{
			QDomNode wnsrc;
			QStringList wn;
			int offnm;

			QDomNode wtsrc;
			QStringList wt;
			int stridetx;
			int offtx;

			QDomNode wcsrc;
			QStringList wc;
			int stridecl;
			int offcl;
		};
	};
}
}
}

#endif
