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

#ifndef __VCGLIB_IMPORTERDAE
#define __VCGLIB_IMPORTERDAE

//importer for collada's files

#include <wrap/dae/util_dae.h>
#include <wrap/dae/poly_triangulator.h>

// uncomment one of the following line to enable the Verbose debugging for the parsing
//#define QDEBUG if(1) ; else {assert(0);} 
#define QDEBUG qDebug

namespace vcg {
namespace tri {
namespace io {
	template<typename OpenMeshType>
	class ImporterDAE : public UtilDAE
	{
  public:
    class ColladaFace;
    class ColladaVertex;

    class ColladaTypes: public vcg::UsedTypes < vcg::Use<ColladaVertex>::template AsVertexType,
																								vcg::Use<ColladaFace  >::template AsFaceType >{};

    class ColladaVertex  : public vcg::Vertex< ColladaTypes,
      vcg::vertex::Coord3f,           /* 12b */
      vcg::vertex::BitFlags,          /*  4b */
      vcg::vertex::Normal3f,          /* 12b */
      vcg::vertex::Color4b           /*  4b */
      > {};


    class ColladaFace    : public vcg::Face<  ColladaTypes,
          vcg::face::VertexRef,            /*12b */
          vcg::face::BitFlags,             /* 4b */
          vcg::face::Normal3f,             /*12b */
          vcg::face::Color4b,           /* 0b */
          vcg::face::WedgeTexCoord2f     /* 0b */
        > {};

    class ColladaMesh    : public vcg::tri::TriMesh< std::vector<ColladaVertex>, std::vector<ColladaFace> > {};



	private:



	
	
	
		static int WedgeNormalAttribute(ColladaMesh& m,const QStringList face,const QStringList wn,const QDomNode wnsrc,const int meshfaceind,const int faceind,const int component)
		{
			int indnm = -1;
			if (!wnsrc.isNull())
			{
				indnm = face.at(faceind).toInt();
				assert(indnm * 3 < wn.size());
				m.face[meshfaceind].WN(component) = vcg::Point3f(wn.at(indnm * 3).toFloat(),wn.at(indnm * 3 + 1).toFloat(),wn.at(indnm * 3 + 2).toFloat());
			}
			return indnm;
		}

		static int WedgeTextureAttribute(ColladaMesh& m,const QStringList face,int ind_txt,const QStringList wt,const QDomNode wtsrc,const int meshfaceind,const int faceind,const int component,const int stride = 2)
		{
			int indtx = -1;
			if (!wtsrc.isNull())
			{
				indtx = face.at(faceind).toInt();
				//int num = wt.size(); 
				assert(indtx * stride < wt.size());
				m.face[meshfaceind].WT(component) = vcg::TexCoord2<float>();
				m.face[meshfaceind].WT(component).U() = wt.at(indtx * stride).toFloat();
				m.face[meshfaceind].WT(component).V() = wt.at(indtx * stride + 1).toFloat();
				
				m.face[meshfaceind].WT(component).N() = ind_txt;
				
			}
			return indtx;
		}
		
		// this one is used for the polylist nodes
		static int WedgeTextureAttribute(typename ColladaMesh::FaceType::TexCoordType & WT, const QStringList faceIndexList, int ind_txt, const QStringList wt, const QDomNode wtsrc,const int faceind,const int stride = 2)
		{
			int indtx = -1;
			if (!wtsrc.isNull())
			{
				indtx = faceIndexList.at(faceind).toInt();
				//int num = wt.size(); 
				assert(indtx * stride < wt.size());
				WT = vcg::TexCoord2<float>();
				WT.U() = wt.at(indtx * stride).toFloat();
				WT.V() = wt.at(indtx * stride + 1).toFloat();				
				WT.N() = ind_txt;
			}
			return indtx;
		}

		static int VertexColorAttribute(ColladaMesh& m,const QStringList face,const QStringList wc,const QDomNode wcsrc,const int faceind, const int vertind,const int colorcomponent)
		{
			int indcl = -1;
			if (!wcsrc.isNull())
			{
				indcl = face.at(faceind).toInt();
				assert((colorcomponent == 4) || (colorcomponent == 3));
				assert(indcl * colorcomponent < wc.size());
				vcg::Color4b c;
				if (colorcomponent == 3)
					c[3] = 255;
				for(unsigned int ii = 0;ii < colorcomponent;++ii)
					c[ii] = (unsigned char)(wc.at(indcl * colorcomponent + ii).toFloat()*255.0);
				m.vert[vertind].C() = c;
			}
			return indcl;
		}


		static void FindStandardWedgeAttributes(WedgeAttribute& wed,const QDomNode nd,const QDomDocument doc)
		{
			wed.wnsrc = findNodeBySpecificAttributeValue(nd,"input","semantic","NORMAL");
			wed.offnm = findStringListAttribute(wed.wn,wed.wnsrc,nd,doc,"NORMAL");

			wed.wtsrc = findNodeBySpecificAttributeValue(nd,"input","semantic","TEXCOORD");
			if (!wed.wtsrc.isNull())
			{
				QDomNode src = attributeSourcePerSimplex(nd,doc,"TEXCOORD");
				if (isThereTag(src,"accessor"))
				{
					QDomNodeList wedatts = src.toElement().elementsByTagName("accessor");
					wed.stridetx = wedatts.at(0).toElement().attribute("stride").toInt();
				}
				else 
					wed.stridetx = 2;
			}
			//else
			//	wed.stridetx = 2;

			wed.offtx = findStringListAttribute(wed.wt,wed.wtsrc,nd,doc,"TEXCOORD"); 

			wed.wcsrc = findNodeBySpecificAttributeValue(nd,"input","semantic","COLOR");
			if (!wed.wcsrc.isNull())
			{
				QDomNode src = attributeSourcePerSimplex(nd,doc,"COLOR");
				if (isThereTag(src,"accessor"))
				{
					QDomNodeList wedatts = src.toElement().elementsByTagName("accessor");
					wed.stridecl = wedatts.at(0).toElement().attribute("stride").toInt();
				}
				else 
					wed.stridecl = 3;
			}
			/*else
				wed.stridecl = 3;*/
			wed.offcl = findStringListAttribute(wed.wc,wed.wcsrc,nd,doc,"COLOR"); 
		}
	
        static DAEError LoadPolygonalMesh(QDomNodeList& polypatch,ColladaMesh& m,const size_t offset,InfoDAE & info)
		{
			return E_NOERROR;
		}

        static DAEError	LoadPolygonalListMesh(QDomNodeList& polylist,ColladaMesh& m,const size_t offset,InfoDAE& info,QMap<QString,QString> &materialBinding)
		{
			if(polylist.isEmpty()) return E_NOERROR; 
			QDEBUG("****** LoadPolygonalListMesh (initial mesh size %i %i)",m.vert.size(),m.fn);
			for(int tript = 0; tript < polylist.size();++tript)
			{
				QString materialId =  polylist.at(tript).toElement().attribute(QString("material"));
				QDEBUG("******    material id '%s' -> '%s'",qPrintable(materialId),qPrintable(materialBinding[materialId]));
				
				QString textureFilename;
                QDomNode img_node = textureFinder(materialBinding[materialId],textureFilename,*(info.doc));
				if(img_node.isNull())
				{
					QDEBUG("******   but we were not able to find the corresponding image node");
				}
				
				int ind_txt = -1;
				if (!img_node.isNull())
				{
                    if(info.textureIdMap.contains(textureFilename))
                         ind_txt=info.textureIdMap[textureFilename];
					else
					{
						QDEBUG("Found use of Texture %s, adding it to texutres",qPrintable(textureFilename));
                        info.textureIdMap[textureFilename]=m.textures.size();
						m.textures.push_back(qPrintable(textureFilename));
                        ind_txt=info.textureIdMap[textureFilename];
					}
				}
				// number of the attributes associated to each vertex of a face (vert, normal, tex etc)
				int faceAttributeNum = polylist.at(tript).toElement().elementsByTagName("input").size();

				// the list of indexes composing the size of each polygon. 
				// The size of this list is the number of the polygons.
				QStringList faceSizeList; 				
				valueStringList(faceSizeList,polylist.at(tript),"vcount");

				// The long list of indexes composing the various polygons. 
				// for each polygon there are numvert*numattrib indexes.
				QStringList faceIndexList; 
				valueStringList(faceIndexList,polylist.at(tript),"p");

				//int offsetface = (int)m.face.size();
				if (faceIndexList.size() != 0 && faceSizeList.size() != 0 ) 
				{	
					WedgeAttribute wa;
                    FindStandardWedgeAttributes(wa,polylist.at(tript),*(info.doc));
					QDEBUG("*******                 Start Reading faces. Attributes Offsets: offtx %i - offnm %i - offcl %i",wa.offtx,wa.offnm,wa.offcl);
					
          int faceIndexCnt=0;
					int jj = 0;	
					for(int ff = 0; ff < (int) faceSizeList.size();++ff) // for each polygon
					{ 
						int curFaceVertNum = faceSizeList.at(ff).toInt();
						
						MyPolygon<typename ColladaMesh::VertexType>  polyTemp(curFaceVertNum);						
						for(int tt = 0;tt < curFaceVertNum ;++tt)  // for each vertex of the polygon
						{
							int indvt = faceIndexList.at(faceIndexCnt).toInt();
							if(faceSizeList.size()<100) QDEBUG("*******                 Reading face[%3i].V(%i) = %4i  (%i-th of the index list) (face has %i vertices)",ff,tt,indvt,faceIndexCnt,curFaceVertNum);
							assert(indvt + offset < m.vert.size());
							polyTemp._pv[tt] = &(m.vert[indvt + offset]);
							faceIndexCnt +=faceAttributeNum;
							
							WedgeTextureAttribute(polyTemp._txc[tt],faceIndexList,ind_txt, wa.wt ,wa.wtsrc, jj + wa.offtx,wa.stridetx);

							/****************
						
							if(tri::HasPerWedgeNormal(m)) WedgeNormalAttribute(m,face,wa.wn,wa.wnsrc,ff,jj + wa.offnm,tt);
							if(tri::HasPerWedgeColor(m)) 	WedgeColorAttribute(m,face,wa.wc,wa.wcsrc,ff,jj + wa.offcl,tt);

							if(tri::HasPerWedgeTexCoord(m) && ind_txt != -1)
							{
															WedgeTextureAttribute(m,face,ind_txt,wa.wt,wa.wtsrc,ff,jj + wa.offtx,tt,wa.stride);
							}
									****************/

							jj += faceAttributeNum;
						}
						
						AddPolygonToMesh(polyTemp,m);
					}
				} 
			
			}
			QDEBUG("****** LoadPolygonalListMesh (final  mesh size vn %i vertsize %i - fn %i facesize %i)",m.vn,m.vert.size(),m.fn,m.face.size());
			return E_NOERROR;
		}

    static DAEError AddPolygonToMesh(MyPolygon<typename ColladaMesh::VertexType>  &polyTemp, ColladaMesh& m)
		{
			int vertNum=polyTemp._pv.size();
			int triNum= vertNum -2;
			typename ColladaMesh::FaceIterator fp=vcg::tri::Allocator<ColladaMesh>::AddFaces(m,triNum);
			// Very simple fan triangulation of the polygon.
			for(int i=0;i<triNum;++i)
			{
				assert(fp!=m.face.end());
				(*fp).V(0)=polyTemp._pv[0];
				(*fp).WT(0)=polyTemp._txc[0];
								
				(*fp).V(1) =polyTemp._pv [i+1];
				(*fp).WT(1)=polyTemp._txc[i+1];
				
				(*fp).V(2) =polyTemp._pv[i+2];
				(*fp).WT(2)=polyTemp._txc[i+2];
				
				++fp;
			}
			assert(fp==m.face.end());
			return E_NOERROR;
		}
		
        static DAEError	OldLoadPolygonalListMesh(QDomNodeList& polylist,ColladaMesh& m,const size_t offset,InfoDAE& info)
		{
			typedef PolygonalMesh< MyPolygon<typename ColladaMesh::VertexType> > PolyMesh;
			PolyMesh pm;
			
			//copying vertices 
			for(typename ColladaMesh::VertexIterator itv = m.vert.begin();itv != m.vert.end();++itv)
			{	
				vcg::Point3f p(itv->P().X(),itv->P().Y(),itv->P().Z());
				typename PolyMesh::VertexType v;
				v.P() = p;
				pm.vert.push_back(v);
			}

			int polylist_size = polylist.size();
			for(int pl = 0; pl < polylist_size;++pl)
			{ 
				QString mat =  polylist.at(pl).toElement().attribute(QString("material"));
				QString textureFilename;
                QDomNode txt_node = textureFinder(mat,textureFilename,*(info.doc));
				int ind_txt = -1;
				if (!txt_node.isNull())
                    ind_txt = indexTextureByImgNode(*(info.doc),txt_node);

				//PolyMesh::PERWEDGEATTRIBUTETYPE att = PolyMesh::NONE;
				WedgeAttribute wa;
                FindStandardWedgeAttributes(wa,polylist.at(pl),*(info.doc));
				QStringList vertcount;
				valueStringList(vertcount,polylist.at(pl),"vcount");
				int indforpol = findOffSetForASingleSimplex(polylist.at(pl));
				int offpols = 0;
				int npolig = vertcount.size();
				QStringList polyind;
				valueStringList(polyind,polylist.at(pl),"p");
				for(int ii = 0;ii < npolig;++ii)
				{
					int nvert = vertcount.at(ii).toInt();
					typename PolyMesh::FaceType p(nvert);
				
					for(int iv = 0;iv < nvert;++iv)
					{
						int index = offset + polyind.at(offpols + iv * indforpol).toInt();
						p._pv[iv] = &(pm.vert[index]);
						int nmindex = -1;

						if (!wa.wnsrc.isNull())
							nmindex = offset + polyind.at(offpols + iv * indforpol + wa.offnm).toInt();

						int txindex = -1;
						if (!wa.wtsrc.isNull())
						{
							txindex = offset + polyind.at(offpols + iv * indforpol + wa.offtx).toInt();
							/*p._txc[iv].U() = wa.wt.at(txindex * 2).toFloat();
							p._txc[iv].V() = wa.wt.at(txindex * 2 + 1).toFloat();
							p._txc[iv].N() = ind_txt;*/
						}
					}
					pm._pols.push_back(p);
					offpols += nvert * indforpol;
				}
			}
			pm.triangulate(m);
			return E_NOERROR;
		}
		/*
		 Called to load into a given mesh 
		 */
        static DAEError LoadTriangularMesh(QDomNodeList& triNodeList, ColladaMesh& m, const size_t offset, InfoDAE& info,QMap<QString,QString> &materialBinding)
		{
			if(triNodeList.isEmpty()) return E_NOERROR; 
			QDEBUG("****** LoadTriangularMesh (initial mesh size %i %i)",m.vn,m.fn);
			for(int tript = 0; tript < triNodeList.size();++tript)
			{
				QString materialId =  triNodeList.at(tript).toElement().attribute(QString("material"));
				QDEBUG("******    material id '%s' -> '%s'",qPrintable(materialId),qPrintable(materialBinding[materialId]));
				
				QString textureFilename;
                QDomNode img_node = textureFinder(materialBinding[materialId],textureFilename,*(info.doc));
				if(img_node.isNull())
				{
					QDEBUG("******   but we were not able to find the corresponding image node");
				}
				
				int ind_txt = -1;
				if (!img_node.isNull())
				{
                    if(info.textureIdMap.contains(textureFilename))
                         ind_txt=info.textureIdMap[textureFilename];
					else
					{
						QDEBUG("Found use of Texture %s, adding it to texutres",qPrintable(textureFilename));
                        info.textureIdMap[textureFilename]=m.textures.size();
						m.textures.push_back(qPrintable(textureFilename));
                        ind_txt=info.textureIdMap[textureFilename];
					}
                //	ind_txt = indexTextureByImgNode(*(info.doc),txt_node);
				}
				int faceAttributeNum = triNodeList.at(tript).toElement().elementsByTagName("input").size();

				QStringList face;
				valueStringList(face,triNodeList.at(tript),"p");
				int offsetface = (int)m.face.size();
				if (face.size() != 0) 
				{	
					vcg::tri::Allocator<ColladaMesh>::AddFaces(m,face.size() / (faceAttributeNum * 3));
					WedgeAttribute wa;
                    FindStandardWedgeAttributes(wa,triNodeList.at(tript),*(info.doc));

					int jj = 0;	
					for(int ff = offsetface;ff < (int) m.face.size();++ff)
					{ 
						
						for(unsigned int tt = 0;tt < 3;++tt)
						{
							int indvt = face.at(jj).toInt();
							assert(indvt + offset < m.vert.size());
							m.face[ff].V(tt) = &(m.vert[indvt + offset]);

							if(tri::HasPerWedgeNormal(m)) 
								WedgeNormalAttribute(m,face,wa.wn,wa.wnsrc,ff,jj + wa.offnm,tt);
							if(tri::HasPerVertexColor(m)) 	
							{
								VertexColorAttribute(m,face,wa.wc,wa.wcsrc,jj + wa.offcl,indvt + offset,wa.stridecl);
							}

							if(tri::HasPerWedgeTexCoord(m) && ind_txt != -1)
							{
								WedgeTextureAttribute(m,face,ind_txt,wa.wt,wa.wtsrc,ff,jj + wa.offtx,tt,wa.stridetx);
							}

							jj += faceAttributeNum;
						}
						if( ! ( (m.face[ff].V(0) != m.face[ff].V(1)) &&  
										(m.face[ff].V(0) != m.face[ff].V(2)) &&  
										(m.face[ff].V(1) != m.face[ff].V(2)) )  )
										QDEBUG("********* WARNING face %i, (%i %i %i) is a DEGENERATE FACE!",ff, m.face[ff].V(0) - &m.vert.front(), m.face[ff].V(1) - &m.vert.front(), m.face[ff].V(2) - &m.vert.front());

					}
				}
			}
			QDEBUG("****** LoadTriangularMesh (final  mesh size %i %i - %i %i)",m.vn,m.vert.size(),m.fn,m.face.size());
			return E_NOERROR;
		}

        static int LoadControllerMesh(ColladaMesh& m, InfoDAE& info, const QDomElement& geo,QMap<QString, QString> materialBindingMap, CallBackPos *cb=0)
		{
			(void)cb;

			assert(geo.tagName() == "controller");
			QDomNodeList skinList = geo.toElement().elementsByTagName("skin");
			if(skinList.size()!=1) return E_CANTOPEN;
			QDomElement skinNode = skinList.at(0).toElement();
			
			QString geomNode_url;
			referenceToANodeAttribute(skinNode,"source",geomNode_url);
			QDEBUG("Found a controller referencing a skin with url '%s'", qPrintable(geomNode_url));
            QDomNode refNode = findNodeBySpecificAttributeValue(*(info.doc),"geometry","id",geomNode_url);
			
			QDomNodeList bindingNodes = skinNode.toElement().elementsByTagName("bind_material");
			if(	bindingNodes.size()>0) { 
				QDEBUG("**   skin node of a controller has a material binding");
				GenerateMaterialBinding(skinNode,materialBindingMap);
			}
			return LoadGeometry(m, info, refNode.toElement(),materialBindingMap);
		}
		
		/* before instancing a geometry you can make a binding that allow you to substitute next material names with other names. 
		this is very useful for instancing the same geometry with different materials. therefore when you encounter a material name in a mesh, this name can be a 'symbol' that you have to bind.
		*/
		static bool GenerateMaterialBinding(QDomNode instanceGeomNode, QMap<QString,QString> &binding)
		{
			QDomNodeList instanceMaterialList=instanceGeomNode.toElement().elementsByTagName("instance_material");
			QDEBUG("++++ Found %i instance_material binding",instanceMaterialList.size() );
			for(int i=0;i<instanceMaterialList.size();++i)
			{
				QString symbol = instanceMaterialList.at(i).toElement().attribute("symbol");
				QString target = instanceMaterialList.at(i).toElement().attribute("target");
				binding[symbol]=target;
				QDEBUG("++++++ %s -> %s",qPrintable(symbol),qPrintable(target));
			}
			return true;
		}
		
		
    /*
		 Basic function that get in input a node <geometry> with a map from material names to texture names.
		 this map is necessary because when using a geometry when it is instanced its material can be bind with different names.
		 if the map fails you should directly search in the material library.
		 
		 */
		
        static int LoadGeometry(ColladaMesh& m, InfoDAE& info, const QDomElement& geo, QMap<QString,QString> &materialBinding, CallBackPos *cb=0)
		{
			assert(geo.tagName() == "geometry");
			if (!isThereTag(geo,"mesh")) return E_NOMESH;
			
            if ((cb !=NULL) && (((info.numvert + info.numface)%100)==0) && !(*cb)((100*(info.numvert + info.numface))/(info.numvert + info.numface), "Vertex Loading"))
					return E_CANTOPEN;
			QDEBUG("**** Loading a Geometry Mesh **** (initial mesh size %i %i)",m.vn,m.fn);
			QDomNodeList vertices = geo.toElement().elementsByTagName("vertices");
			if (vertices.size() != 1) return E_INCOMPATIBLECOLLADA141FORMAT;
			QDomElement vertNode = vertices.at(0).toElement();

            QDomNode positionNode = attributeSourcePerSimplex(vertNode,*(info.doc),"POSITION");
			if (positionNode.isNull()) return E_NOVERTEXPOSITION;

			QStringList geosrcposarr;
			valueStringList(geosrcposarr, positionNode, "float_array");

			int geosrcposarr_size = geosrcposarr.size();
			if ((geosrcposarr_size % 3) != 0)
				return E_CANTOPEN;
			int nvert = geosrcposarr_size / 3;
			size_t offset = m.vert.size();
			if (geosrcposarr_size != 0)
			{
					vcg::tri::Allocator<ColladaMesh>::AddVertices(m,nvert);

                    QDomNode srcnodenorm = attributeSourcePerSimplex(vertices.at(0),*(info.doc),"NORMAL");
					QStringList geosrcvertnorm;
					if (!srcnodenorm.isNull())
						valueStringList(geosrcvertnorm,srcnodenorm,"float_array");

                    QDomNode srcnodetext = attributeSourcePerSimplex(vertices.at(0),*(info.doc),"TEXCOORD");
					QStringList geosrcverttext;
					if (!srcnodetext.isNull())
						valueStringList(geosrcverttext,srcnodetext,"float_array");

                    QDomNode srcnodecolor = attributeSourcePerSimplex(vertices.at(0),*(info.doc),"COLOR");
					QDomNodeList accesslist = srcnodecolor.toElement().elementsByTagName("accessor");

					QStringList geosrcvertcol;
					if (!srcnodecolor.isNull())
						valueStringList(geosrcvertcol,srcnodecolor,"float_array");

					int ii = 0;
					for(size_t vv = offset;vv < m.vert.size();++vv)
					{						
						Point3f positionCoord(geosrcposarr[ii * 3].toFloat(),geosrcposarr[ii * 3 + 1].toFloat(),geosrcposarr[ii * 3 + 2].toFloat());
						m.vert[vv].P() = positionCoord;

						if (!srcnodenorm.isNull())
						{
							Point3f normalCoord(geosrcvertnorm[ii * 3].toFloat(),
																	geosrcvertnorm[ii * 3 + 1].toFloat(),
																	geosrcvertnorm[ii * 3 + 2].toFloat());
							normalCoord.Normalize();
							m.vert[vv].N() = normalCoord;
						}

						if (!srcnodecolor.isNull())
						{
							if (accesslist.size() > 0)
							{
								//component per color...obviously we assume they are RGB or RGBA if ARGB you get fancy effects....
								if (accesslist.at(0).childNodes().size() == 4)
									m.vert[vv].C() = vcg::Color4b(geosrcvertcol[ii * 4].toFloat()*255.0,geosrcvertcol[ii * 4 + 1].toFloat()*255.0,geosrcvertcol[ii * 4 + 2].toFloat()*255.0,geosrcvertcol[ii * 4 + 3].toFloat()*255.0);
								else
									if (accesslist.at(0).childNodes().size() == 3)
										m.vert[vv].C() = vcg::Color4b(geosrcvertcol[ii * 3].toFloat()*255.0,geosrcvertcol[ii * 3 + 1].toFloat()*255.0,geosrcvertcol[ii * 3 + 2].toFloat()*255.0,255.0);
							}
						}

						if (!srcnodetext.isNull())
						{

							assert((ii * 2 < geosrcverttext.size()) && (ii * 2 + 1 < geosrcverttext.size()));
							m.vert[vv].T() = vcg::TexCoord2<float>();
							m.vert[vv].T().u() = geosrcverttext[ii * 2].toFloat();
							m.vert[vv].T().v() = geosrcverttext[ii * 2 + 1].toFloat();
						}
						++ii;
					}

					QDomNodeList tripatch = geo.toElement().elementsByTagName("triangles");
					QDomNodeList polypatch = geo.toElement().elementsByTagName("polygons");
					QDomNodeList polylist = geo.toElement().elementsByTagName("polylist");
					QStringList vertcount;
					valueStringList(vertcount,polylist.at(0),"vcount");
					int isTri=true;
					for (int i=0; i<vertcount.size(); i++)
					{
						if (vertcount[i]!="3")
						{
							isTri=false;
							break;
						}
						
					}
					if (isTri && tripatch.isEmpty())
						tripatch=polylist;
					if (tripatch.isEmpty()  && polypatch.isEmpty() && polylist.isEmpty())
						return E_NOPOLYGONALMESH;
					
					DAEError err = E_NOERROR;
					err = LoadTriangularMesh(tripatch,m,offset,info,materialBinding);
					//err = LoadPolygonalMesh(polypatch,m,offset,info);
			//					err = OldLoadPolygonalListMesh(polylist,m,offset,info);
                    err = LoadPolygonalListMesh(polylist,m,offset,info,materialBinding);
					if (err != E_NOERROR) 
						return err;
				}
			QDEBUG("**** Loading a Geometry Mesh **** (final   mesh size %i %i - %i %i)",m.vn,m.vert.size(),m.fn,m.face.size());
			return E_NOERROR;						 
		}

		static void GetTexCoord(const QDomDocument& doc, QStringList &texturefile)
		{
			QDomNodeList txlst = doc.elementsByTagName("library_images");
			for(int img = 0;img < txlst.at(0).childNodes().size();++img)
			{
				QDomNodeList nlst = txlst.at(0).childNodes().at(img).toElement().elementsByTagName("init_from");
				if (nlst.size() > 0)
				{
                                    texturefile.push_back( nlst.at(0).firstChild().nodeValue());
				}
			}
		}

	// This recursive function add to a mesh the subtree starting from the passed node. 
	// When you start from a visual_scene, you can find nodes. 
  // nodes can be directly instanced or referred from the node library.
		
		static void AddNodeToMesh(QDomElement node, 
															ColladaMesh& m, Matrix44f curTr,
                                                            InfoDAE& info)
		{
				QDEBUG("Starting processing <node> with id %s",qPrintable(node.attribute("id")));
 
				curTr = curTr * getTransfMatrixFromNode(node);
				
				QDomNodeList geomNodeList = node.elementsByTagName("instance_geometry");
				for(int ch = 0;ch < geomNodeList.size();++ch) 
				{
					QDomElement instGeomNode= geomNodeList.at(ch).toElement();
					if(instGeomNode.parentNode()==node) // process only direct child
					{
						QDEBUG("** instance_geometry with url %s (intial mesh size %i %i T = %i)",qPrintable(instGeomNode.attribute("url")),m.vn,m.fn,m.textures.size());
						//assert(m.textures.size()>0 == HasPerWedgeTexCoord(m));
						QString geomNode_url;
						referenceToANodeAttribute(instGeomNode,"url",geomNode_url);
                        QDomNode refNode = findNodeBySpecificAttributeValue(*(info.doc),"geometry","id",geomNode_url);
						QDomNodeList bindingNodes = instGeomNode.toElement().elementsByTagName("bind_material");
						QMap<QString,QString> materialBindingMap;
						if(	bindingNodes.size()>0) { 
							QDEBUG("**    instance_geometry has a material binding");
							GenerateMaterialBinding(instGeomNode,materialBindingMap);
						}
						
						ColladaMesh newMesh;
//						newMesh.face.EnableWedgeTex();
						LoadGeometry(newMesh, info, refNode.toElement(),materialBindingMap);
						tri::UpdatePosition<ColladaMesh>::Matrix(newMesh,curTr);
						tri::Append<ColladaMesh,ColladaMesh>::Mesh(m,newMesh);
						QDEBUG("** instance_geometry with url %s (final mesh size %i %i - %i %i)",qPrintable(instGeomNode.attribute("url")),m.vn,m.vert.size(),m.fn,m.face.size());						
					}
				}
				
				QDomNodeList controllerNodeList = node.elementsByTagName("instance_controller");
				for(int ch = 0;ch < controllerNodeList.size();++ch) 
				{
					QDomElement instContrNode= controllerNodeList.at(ch).toElement();
					if(instContrNode.parentNode()==node) // process only direct child
					{
						QDEBUG("Found a instance_controller with url %s",qPrintable(instContrNode.attribute("url")));
					
						QString controllerNode_url;
						referenceToANodeAttribute(instContrNode,"url",controllerNode_url);
						QDEBUG("Found a instance_controller with url '%s'", qPrintable(controllerNode_url));
                        QDomNode refNode = findNodeBySpecificAttributeValue(*(info.doc),"controller","id",controllerNode_url);
						
						QDomNodeList bindingNodes = instContrNode.toElement().elementsByTagName("bind_material");
						QMap<QString, QString> materialBindingMap;
							if(	bindingNodes.size()>0) { 
							QDEBUG("**   instance_controller node of has a material binding");
							GenerateMaterialBinding(instContrNode,materialBindingMap);
						}
						
						ColladaMesh newMesh;
						LoadControllerMesh(newMesh, info, refNode.toElement(),materialBindingMap);
						tri::UpdatePosition<ColladaMesh>::Matrix(newMesh,curTr);
						tri::Append<ColladaMesh,ColladaMesh>::Mesh(m,newMesh);
					}
				}
								
				QDomNodeList nodeNodeList = node.elementsByTagName("node");
				for(int ch = 0;ch < nodeNodeList.size();++ch)
				{
					if(nodeNodeList.at(ch).parentNode()==node) // process only direct child
							AddNodeToMesh(nodeNodeList.at(ch).toElement(), m,curTr, info);
				}
				
				QDomNodeList instanceNodeList = node.elementsByTagName("instance_node");
				for(int ch = 0;ch < instanceNodeList.size();++ch)
				{
					if(instanceNodeList.at(ch).parentNode()==node) // process only direct child
					{
						QDomElement instanceNode =  instanceNodeList.at(ch).toElement();
						QString node_url;
						referenceToANodeAttribute(instanceNode,"url",node_url);
						QDEBUG("Found a instance_node with url '%s'", qPrintable(node_url));
                        QDomNode refNode = findNodeBySpecificAttributeValue(*(info.doc),"node","id",node_url);
						if(refNode.isNull()) 
							QDEBUG("findNodeBySpecificAttributeValue returned a null node for %s",qPrintable(node_url));
										 
						AddNodeToMesh(refNode.toElement(), m,curTr, info);
					}
				}
		}
		

// Retrieve the transformation matrix that is defined in the childs of a node.
// used during the recursive descent.
static Matrix44f getTransfMatrixFromNode(const QDomElement parentNode)
{
	QDEBUG("getTrans form node with tag %s",qPrintable(parentNode.tagName()));
	assert(parentNode.tagName() == "node");
	
	std::vector<QDomNode> rotationList;
	QDomNode matrixNode;
	QDomNode translationNode;
	for(int ch = 0;ch < parentNode.childNodes().size();++ch)
		{
			if (parentNode.childNodes().at(ch).nodeName() == "rotate")    
				rotationList.push_back(parentNode.childNodes().at(ch));
			if (parentNode.childNodes().at(ch).nodeName() == "translate")	
				translationNode = parentNode.childNodes().at(ch);							
			if (parentNode.childNodes().at(ch).nodeName() == "matrix")	  
				matrixNode = parentNode.childNodes().at(ch);							
		}

		Matrix44f rotM;		   rotM.SetIdentity();
		Matrix44f transM; transM.SetIdentity();

		if (!translationNode.isNull()) ParseTranslation(transM,translationNode);
		if (!rotationList.empty()) ParseRotationMatrix(rotM,rotationList);
		if (!matrixNode.isNull()) 
		{
			ParseMatrixNode(transM,matrixNode);
		  return transM;
		}
	  return transM*rotM;
}

	public:

		//merge all meshes in the collada's file in the templeted mesh m
		//I assume the mesh 
		
        static int Open(OpenMeshType& m,const char* filename, InfoDAE& info, CallBackPos *cb=0)
		{
			(void)cb;

			QDEBUG("----- Starting the processing of %s ------",filename);
            //AdditionalInfoDAE& inf = new AdditionalInfoDAE();
            //info = new InfoDAE();
			
			QDomDocument* doc = new QDomDocument(filename);
            info.doc = doc;
			QFile file(filename);
			if (!file.open(QIODevice::ReadOnly))
				return E_CANTOPEN;
			if (!doc->setContent(&file)) 
			{
				file.close();
				return E_CANTOPEN;
			}
			file.close();
			
            //GetTexture(*(info.doc),inf);
			
//			GenerateMaterialToTextureMap(info);
			//scene->instance_visual_scene
            QDomNodeList scenes = info.doc->elementsByTagName("scene");
			int scn_size = scenes.size();
			if (scn_size == 0) 
				return E_NO3DSCENE;
			QDEBUG("File Contains %i Scenes",scenes.size());
			int problem = E_NOERROR;
			//bool found_a_mesh = false;
			//Is there geometry in the file? 
			//bool geoinst_found = false;
			
			// The main loading loop
			// for each scene in COLLADA FILE
			/*
			Some notes on collada structure.
			top level nodes are :
			<asset>
			<library_images>
			<library_materials>
			<library_effects>
			<library_geometries>
			<library_visual_scene>
			<scene>
			
			The REAL top root is the <scene> that can contains one of more (instance of) <visual_scene>.
			<visual_scene> can be directly written there (check!) or instanced from their definition in the <library_visual_scene>
			each <visual_scene> contains a hierarchy of <node>	
		  each <node> contains
				transformation
				other nodes (to build up a hierarchy)
				instance of geometry 
				instance of controller
			instance can be direct or refers name of stuff described in a library. 
			An instance of geometry node should contain the <mesh> node and as a son of the <instance geometry> the material node (again referenced from a library)
			-- structure of the geometry node -- 
			*/
			for(int scn = 0;scn < scn_size;++scn)
			{
				QDomNodeList instscenes = scenes.at(scn).toElement().elementsByTagName("instance_visual_scene");
				int instscn_size = instscenes.size();
				QDEBUG("Scene %i contains %i instance_visual_scene ",scn,instscn_size);
				if (instscn_size == 0)  return E_INCOMPATIBLECOLLADA141FORMAT;

				//for each scene instance in a COLLADA scene
				for(int instscn = 0;instscn < instscn_size; ++instscn)
				{
					QString libscn_url;
					referenceToANodeAttribute(instscenes.at(instscn),"url",libscn_url);	
					QDEBUG("instance_visual_scene %i refers %s ",instscn,qPrintable(libscn_url));
					
//					QDomNode nd = QDomNode(*(inf->doc));
                    QDomNode visscn = findNodeBySpecificAttributeValue(*(info.doc),"visual_scene","id",libscn_url);
					if(visscn.isNull()) return E_UNREFERENCEBLEDCOLLADAATTRIBUTE;
					
					//assert (visscn.toElement().Attribute("id") == libscn_url);
					//for each node in the libscn_url visual scene  
					QDomNodeList visscn_child = visscn.childNodes();
					QDEBUG("instance_visual_scene %s has %i children",qPrintable(libscn_url),visscn_child.size());
					
					// for each direct child of a visual scene process it
					for(int chdind = 0; chdind < visscn_child.size();++chdind)
					{
						QDomElement node=visscn_child.at(chdind).toElement();
						if(node.isNull()) continue;
						QDEBUG("Processing Visual Scene child %i - of type '%s'",chdind,qPrintable(node.tagName()));
						Matrix44f baseTr; baseTr.SetIdentity();
						
						if(node.toElement().tagName()=="node")
						{
							ColladaMesh newMesh;
							AddNodeToMesh(node.toElement(), newMesh, baseTr,info);
							tri::Append<OpenMeshType,ColladaMesh>::Mesh(m,newMesh);
						}
					}	// end for each node of a given scene				
				} // end for each visual scene instance
			} // end for each scene instance 
			return problem;
		}

        static bool LoadMask(const char * filename, InfoDAE& info)
		{
			bool bHasPerWedgeTexCoord = false;
			bool bHasPerWedgeNormal		= false;
			//bool bHasPerWedgeColor		= false;
			bool bHasPerVertexColor		= false;
			bool bHasPerFaceColor			= false;
			bool bHasPerVertexNormal = false;
			bool bHasPerVertexText = false;
			
			QDomDocument* doc = new QDomDocument(filename);
			QFile file(filename);
			if (!file.open(QIODevice::ReadOnly))
				return false;
			if (!doc->setContent(&file)) 
			{
				file.close();
				return false;
			}
			file.close();
			
			QStringList textureFileList;
            info.doc = doc;
            GetTexCoord(*(info.doc),textureFileList);
            QDomNodeList scenes = info.doc->elementsByTagName("scene");
			int scn_size = scenes.size();
			

			//Is there geometry in the file? 
			bool geoinst_found = false;
			//for each scene in COLLADA FILE
			for(int scn = 0;scn < scn_size;++scn)
			{
				QDomNodeList instscenes = scenes.at(scn).toElement().elementsByTagName("instance_visual_scene");
				int instscn_size = instscenes.size();
				if (instscn_size == 0)  return false;

				//for each scene instance in a COLLADA scene
				for(int instscn = 0;instscn < instscn_size; ++instscn)
				{
					QString libscn_url;
					referenceToANodeAttribute(instscenes.at(instscn),"url",libscn_url);	
                    QDomNode nd = QDomNode(*(info.doc));
                    QDomNode visscn = findNodeBySpecificAttributeValue(*(info.doc),"visual_scene","id",libscn_url);
					if(visscn.isNull()) 	return false;
					
					//for each node in the libscn_url visual scene  
					//QDomNodeList& visscn_child = visscn.childNodes();
					QDomNodeList visscn_child = visscn.childNodes();
					
					//for each direct child of a libscn_url visual scene find if there is some geometry instance
					for(int chdind = 0; chdind < visscn_child.size();++chdind)
					{
						//QDomNodeList& geoinst = visscn_child.at(chdind).toElement().elementsByTagName("instance_geometry");
						QDomNodeList geoinst = visscn_child.at(chdind).toElement().elementsByTagName("instance_geometry");
						int geoinst_size = geoinst.size();
						if (geoinst_size != 0)
						{
							
							geoinst_found |= true;
                            QDomNodeList geolib = info.doc->elementsByTagName("library_geometries");
							//assert(geolib.size() == 1);
							if (geolib.size() != 1)
								return false;
							//!!!!!!!!!!!!!!!!!here will be the code for geometry transformations!!!!!!!!!!!!!!!!!!!!!!
                            info.numvert = 0;
                            info.numface = 0;
							for(int geoinst_ind = 0;geoinst_ind < geoinst_size;++geoinst_ind)
							{
								QString geo_url;
								referenceToANodeAttribute(geoinst.at(geoinst_ind),"url",geo_url);
								
								QDomNode geo = findNodeBySpecificAttributeValue(geolib.at(0),"geometry","id",geo_url);
								if (geo.isNull())
									return false;
							
								QDomNodeList vertlist = geo.toElement().elementsByTagName("vertices");

								for(int vert = 0;vert < vertlist.size();++vert)
								{
									QDomNode no;
									no = findNodeBySpecificAttributeValue(vertlist.at(vert),"input","semantic","POSITION");
									QString srcurl;
									referenceToANodeAttribute(no,"source",srcurl);
									no = findNodeBySpecificAttributeValue(geo,"source","id",srcurl);
									QDomNodeList fa = no.toElement().elementsByTagName("float_array");
									assert(fa.size() == 1);
                                    info.numvert += (fa.at(0).toElement().attribute("count").toInt() / 3);
									no = findNodeBySpecificAttributeValue(vertlist.at(vert),"input","semantic","COLOR");									
									if (!no.isNull()) 
										bHasPerVertexColor = true;
									no = findNodeBySpecificAttributeValue(vertlist.at(vert),"input","semantic","NORMAL");									
									if (!no.isNull()) 
										bHasPerVertexNormal = true;
									no = findNodeBySpecificAttributeValue(vertlist.at(vert),"input","semantic","TEXCOORD");									
									if (!no.isNull()) 
										bHasPerVertexText = true;
								}

								const char* arr[] = {"triangles","polylist","polygons"};

								for(unsigned int tt= 0;tt < 3;++tt)
								{
									QDomNodeList facelist = geo.toElement().elementsByTagName(arr[tt]);
									for(int face = 0;face < facelist.size();++face)
									{
                                        info.numface += facelist.at(face).toElement().attribute("count").toInt() ;
										QDomNode no;
										no = findNodeBySpecificAttributeValue(facelist.at(face),"input","semantic","NORMAL");
										if (!no.isNull()) 
											bHasPerWedgeNormal = true;
										no = findNodeBySpecificAttributeValue(facelist.at(face),"input","semantic","COLOR");
										if (!no.isNull()) 
											bHasPerVertexColor = true;
										no = findNodeBySpecificAttributeValue(facelist.at(face),"input","semantic","TEXCOORD");
										if (!no.isNull()) 
											bHasPerWedgeTexCoord = true;
									}
								}
							}
						}
					}
				}
			}
			
			if (!geoinst_found)
			{
                QDomNodeList geolib = info.doc->elementsByTagName("library_geometries");
				//assert(geolib.size() == 1);
				if (geolib.size() != 1)
					return false;
				QDomNodeList geochild = geolib.at(0).toElement().elementsByTagName("geometry");
				//!!!!!!!!!!!!!!!!!here will be the code for geometry transformations!!!!!!!!!!!!!!!!!!!!!!
                info.numvert = 0;
                info.numface = 0;
				for(int geoinst_ind = 0;geoinst_ind < geochild.size();++geoinst_ind)
				{
					QDomNodeList vertlist = geochild.at(geoinst_ind).toElement().elementsByTagName("vertices");

					for(int vert = 0;vert < vertlist.size();++vert)
					{
						QDomNode no;
						no = findNodeBySpecificAttributeValue(vertlist.at(vert),"input","semantic","POSITION");
						QString srcurl;
						referenceToANodeAttribute(no,"source",srcurl);
						no = findNodeBySpecificAttributeValue(geochild.at(geoinst_ind),"source","id",srcurl);
						QDomNodeList fa = no.toElement().elementsByTagName("float_array");
						assert(fa.size() == 1);
                        info.numvert += (fa.at(0).toElement().attribute("count").toInt() / 3);
						no = findNodeBySpecificAttributeValue(vertlist.at(vert),"input","semantic","COLOR");									
						if (!no.isNull()) 
							bHasPerVertexColor = true;
						no = findNodeBySpecificAttributeValue(vertlist.at(vert),"input","semantic","NORMAL");									
						if (!no.isNull()) 
							bHasPerVertexNormal = true;
						no = findNodeBySpecificAttributeValue(vertlist.at(vert),"input","semantic","TEXCOORD");									
						if (!no.isNull()) 
							bHasPerVertexText = true;
					}

					QDomNodeList facelist = geochild.at(geoinst_ind).toElement().elementsByTagName("triangles");
					for(int face = 0;face < facelist.size();++face)
					{
                        info.numface += facelist.at(face).toElement().attribute("count").toInt() ;
						QDomNode no;
						no = findNodeBySpecificAttributeValue(facelist.at(face),"input","semantic","NORMAL");
						if (!no.isNull()) 
							bHasPerWedgeNormal = true;
						no = findNodeBySpecificAttributeValue(facelist.at(face),"input","semantic","TEXCOORD");
						if (!no.isNull()) 
							bHasPerWedgeTexCoord = true;
					}
				}
			}

            info.mask = 0;
		
			if (bHasPerWedgeTexCoord) 
                info.mask |= vcg::tri::io::Mask::IOM_WEDGTEXCOORD;
			if (bHasPerWedgeNormal) 
                info.mask |= vcg::tri::io::Mask::IOM_WEDGNORMAL;
			if (bHasPerVertexColor)	
                info.mask |= vcg::tri::io::Mask::IOM_VERTCOLOR;
			if (bHasPerFaceColor) 
                info.mask |= vcg::tri::io::Mask::IOM_FACECOLOR;
			if (bHasPerVertexNormal) 
                info.mask |= vcg::tri::io::Mask::IOM_VERTNORMAL;
			if (bHasPerVertexText) 
                info.mask |= vcg::tri::io::Mask::IOM_VERTTEXCOORD;
			
			

            delete (info.doc);
            info.doc = NULL;
            //addinfo = info;
			return true;
		}
	};
}
}
}

#endif
