#ifndef EXPORT_FBX
#define EXPORT_FBX

#include <fbxsdk.h>

/******************************
ExporterFBX is the class devoted to export the info contained in a VCG mesh to an FBX file.
In order to compile the following class you need to:
	- download the VCGlib from our SVN server and put the code in ../../vcglib
	- download from Autodesk website the FBXSDK-2012.1 and put the lib and include folders in ../fbx-2012.1
*/

template <class SaveMeshType>
class ExporterFBX
{
public:
	typedef typename SaveMeshType::VertexPointer VertexPointer;
	typedef typename SaveMeshType::ScalarType ScalarType;
	typedef typename SaveMeshType::VertexType VertexType;
	typedef typename SaveMeshType::FaceType FaceType;
	typedef typename SaveMeshType::FacePointer FacePointer;
	typedef typename SaveMeshType::VertexIterator VertexIterator;
	typedef typename SaveMeshType::FaceIterator FaceIterator;
	typedef typename SaveMeshType::CoordType CoordType;

	/****
		The Save function save the info contained in a mesh into a FBX file.
		Parameters:
			- m is the mesh containing geometry, textures and materials info
			- filename is the path of the file in which you want to save the data.
			- binary says to the function if you want to save the file in binary or ascii format.
			- embed Do you want textures directly embedded inside the file? 
			Please note that accordingly to FBX SDK semantics is meaningless to ask for a ascii file with embedded textures. 
			At the opposite it's perfectly legal to have a binary file without embedded textures.
			- mask is used by the programmer to specify which attributes the function should save on the file. For example if I want to save vertex normal and wedge texture
			I should write something like mask = tri::io::Mask::IOM_WEDGTEXCOORD | tri::io::Mask::IOM_VERTCOLOR | tri::io::Mask::IOM_VERTNORMAL;
	*/

	static int Save(SaveMeshType &m, const char * filename,const bool binary,const bool embed,const int mask)
	{
		KFbxSdkManager* sdkman = KFbxSdkManager::Create();

		KFbxIOSettings * ios = KFbxIOSettings::Create(sdkman, IOSROOT );
		ios->SetBoolProp(EXP_FBX_MATERIAL,true);
		ios->SetBoolProp(EXP_FBX_TEXTURE,true);
		ios->SetBoolProp(EXP_FBX_EMBEDDED,embed);
		ios->SetBoolProp(EXP_FBX_SHAPE,true);
		ios->SetBoolProp(EXP_FBX_ANIMATION,false);
		ios->SetBoolProp(EXP_FBX_GLOBAL_SETTINGS, true);
		sdkman->SetIOSettings(ios);

		if(sdkman == NULL) 
			return -1;

		KFbxExporter* lExporter = KFbxExporter::Create(sdkman,"VCGFbxFileExporter");
		int asciiexportind = -1;

		if ((!binary) && (!embed))
		{
			int formatnum = sdkman->GetIOPluginRegistry()->GetWriterFormatCount();
			bool asciifound = false;
			int ii = 0;
			while ((!asciifound) && (ii<formatnum))
			{
				if (sdkman->GetIOPluginRegistry()->WriterIsFBX(ii))
				{
					KString plugdesc =sdkman->GetIOPluginRegistry()->GetWriterFormatDescription(ii);
					if (plugdesc.Find("ascii")>=0)
					{
						asciiexportind = ii;
						asciifound = true;
					}
				}
				++ii;
			}
		}


		if(!lExporter->Initialize(filename, asciiexportind,ios))
			return false;

		KFbxScene* scene = KFbxScene::Create(sdkman,"VCGScene");
		bool result = fillScene(m,*scene,mask);
		if (!result)
			return -1;
		result = lExporter->Export(scene);
		lExporter->Destroy();
		if (!result)
			return -1;
		return 0;
	}

	/********
		GetExportCapability returns a mask with extra attributes that ExporterFBX class is able to export.  
		It doesn't mean that in every file you export you will find all these attributes.
		The attributes set you want to actually export are defined by the mask parameter in the Save function.
	*/

	static int GetExportMaskCapability()
	{
		int capability = 0;			
		capability |= vcg::tri::io::Mask::IOM_VERTCOORD;
		capability |= vcg::tri::io::Mask::IOM_VERTCOLOR;
		capability |= vcg::tri::io::Mask::IOM_VERTNORMAL;
		capability |= vcg::tri::io::Mask::IOM_WEDGTEXCOORD;
		return capability;
	}

private:

	static KFbxFileTexture* newTexture(KFbxScene& scene,const char* name,const KFbxVector2& trans = KFbxVector2(0.0,0.0),const KFbxVector2& rot = KFbxVector2(0.0,0.0),const KFbxVector2& scal = KFbxVector2(1.0,1.0))
	{
		static int conttext = 0;
		std::string label = createName("texture_",conttext);
		KFbxFileTexture* text = KFbxFileTexture::Create(&scene,label.c_str());
		text->SetFileName(name); 
		text->SetTextureUse(KFbxTexture::eSTANDARD);
		text->SetMappingType(KFbxTexture::eUV);
		text->SetMaterialUse(KFbxFileTexture::eMODEL_MATERIAL);
		text->SetSwapUV(false);
		text->SetTranslation(trans[0],trans[1]);
		text->SetScale(scal[0],scal[1]);
		text->SetRotation(rot[0],rot[1]);
		++conttext;
		return text;
	}

	static KFbxSurfacePhong* newPhongMaterial(KFbxScene& scene,const char* name,KFbxFileTexture* text = NULL,const fbxDouble3& dif = fbxDouble3(1.0,1.0,1.0)/*,const fbxDouble3& amb = fbxDouble3(1.0,1.0,1.0),const fbxDouble3& em = fbxDouble3(1.0,1.0,1.0),const double& transp = 0.0,const double& shin = 0.0*/)
	{
		KFbxSurfacePhong* mat = KFbxSurfacePhong::Create(&scene, name);
		mat->Diffuse.Set(dif);
		//mat->Ambient.Set(amb);
		//mat->Emissive.Set(em);
		//mat->TransparencyFactor.Set(transp);
		//mat->ShadingModel.Set("Phong");
		//mat->Shininess.Set(shin);
		if (text)
			mat->Diffuse.ConnectSrcObject(text);
		return mat;
	}

	static void insertMaterialElementLayer(KFbxMesh* vcgmesh)
	{
		KFbxGeometryElementMaterial* lMaterialElement = vcgmesh->CreateElementMaterial();
		lMaterialElement->SetMappingMode(KFbxGeometryElement::eBY_POLYGON);
		lMaterialElement->SetReferenceMode(KFbxGeometryElement::eINDEX_TO_DIRECT);
	}

	static std::string	createName(const char* base,const int ind)
	{
		char buffer[33];
		sprintf(buffer,"%d",ind);
		std::string res = std::string(base) + std::string(buffer);
		return res;
	}

	static bool fillScene(SaveMeshType& m,KFbxScene& scene,const int mask)
	{
		bool wedgetexcoord = (vcg::tri::HasPerWedgeTexCoord(m) && (mask | vcg::tri::io::Mask::IOM_WEDGTEXCOORD));
		bool vertnorm = (vcg::tri::HasPerVertexNormal(m) && (mask | vcg::tri::io::Mask::IOM_VERTNORMAL));
		bool facecolasmaterial = false;
		int fn = int(m.face.size());
		std::set<vcg::Color4b> matset;	
		
		

		if (wedgetexcoord)
		{
			for(int ii = 0;ii < fn;++ii)
			{
				if (!m.face[ii].IsD())
				{
					int ind = m.face[ii].WT(0).N();
					if ((ind == -1) && vcg::tri::HasPerVertexColor(m))
					{
						for(int hh = 0;hh < 3;++hh)
							matset.insert(m.face[ii].V(hh)->C());
						facecolasmaterial = true;
					}
				}
			}
		}

		bool vertcol = (vcg::tri::HasPerVertexColor(m) && (mask | vcg::tri::io::Mask::IOM_VERTCOLOR)) && !facecolasmaterial;
		/*KFbxNode* cam  = insertCamera(&scene, "VCGCamera");*/

		KFbxMesh* vcgmesh = KFbxMesh::Create(&scene,"VCGMeshAttribute");
		KFbxNode* meshnode = KFbxNode::Create(&scene,"VCGMeshNode");
		meshnode->SetNodeAttribute(vcgmesh);
		if (m.textures.size() > 0)
			meshnode->SetShadingMode(KFbxNode::eTEXTURE_SHADING);
		else
			meshnode->SetShadingMode(KFbxNode::eHARD_SHADING);

		KFbxGeometryElementVertexColor* vcolorlay = NULL;
		if (vertcol)
		{
			vcolorlay = vcgmesh->CreateElementVertexColor();
			vcolorlay->SetMappingMode(KFbxGeometryElement::eBY_CONTROL_POINT);
			vcolorlay->SetReferenceMode(KFbxGeometryElement::eDIRECT);
		}

		KFbxGeometryElementNormal* vnormlay = NULL;
		if (vertnorm)
		{
			vnormlay = vcgmesh->CreateElementNormal();
			vnormlay->SetMappingMode(KFbxGeometryElement::eBY_CONTROL_POINT);
			vnormlay->SetReferenceMode(KFbxGeometryElement::eDIRECT);
		}
		int zz;
		for(zz = 0;zz < m.textures.size();++zz)
		{
			KFbxFileTexture* tex = newTexture(scene,m.textures[zz].c_str());
			KFbxSurfacePhong* phong = newPhongMaterial(scene,createName("mat_",zz).c_str(),tex);
			KFbxNode* meshnode = vcgmesh->GetNode();
			if(meshnode == NULL) 
				return false;
			meshnode->AddMaterial(phong);
		}
		
		for(std::set<vcg::Color4b>::iterator it = matset.begin();it != matset.end();++it)
		{
			vcg::Color4b vcgcol = *it;
			fbxDouble3 fbxcol(vcgcol[0] / 255.0f,vcgcol[1]/ 255.0f,vcgcol[2]/ 255.0f);
			KFbxSurfacePhong* phong = newPhongMaterial(scene,createName("mat_",zz).c_str(),NULL,fbxcol);
			KFbxNode* meshnode = vcgmesh->GetNode();
			if(meshnode == NULL) 
				return false;
			meshnode->AddMaterial(phong);
			++zz;
		}
		if (m.textures.size() > 0)
			insertMaterialElementLayer(vcgmesh);

		int vn = int(m.vert.size());
		int notdeletedvert = m.vn;
		int deletedvert = int(vn - notdeletedvert);
		vcgmesh->InitControlPoints(notdeletedvert);
		std::vector<int> deletedBeforeValid(vn);
		KFbxVector4* controlp = vcgmesh->GetControlPoints();
		int validvert = 0;
		int invalidvert = 0;
		for(int ii = 0;ii < vn;++ii)
		{
			if (!m.vert[ii].IsD())
			{
				deletedBeforeValid[ii] = invalidvert;
				vcg::Point3<ScalarType> p = m.vert[ii].P();
				
				controlp[validvert] = KFbxVector4(p.X(),p.Y(),p.Z());
				if (vertcol)
				{
					//Paolo imposed the non templetization of color scalar type. Always byte!
					vcg::Color4b vcgcol = m.vert[ii].C();
					KFbxColor fbxcol(vcgcol[0] / 255.0f,vcgcol[1] / 255.0f,vcgcol[2] / 255.0f,vcgcol[3] / 255.0f);
					if (vcolorlay)
					{
						KFbxLayerElementArrayTemplate<KFbxColor>& carr = vcolorlay->GetDirectArray();	
						carr.Add(fbxcol);
					}
				}

				if(vertnorm)
				{
					CoordType vcgnorm = m.vert[ii].N();
					KFbxVector4 fbxnorm(vcgnorm[0],vcgnorm[1],vcgnorm[2]);
					if (vnormlay)
					{
						KFbxLayerElementArrayTemplate<KFbxVector4>& narr = vnormlay->GetDirectArray();	
						narr.Add(fbxnorm);
					}
				}
				++validvert;
			}
			else
				++invalidvert;
		}

		KFbxGeometryElementUV* uvel = NULL; 
		

		if (wedgetexcoord)
		{
			uvel = vcgmesh->CreateElementUV("UV");
			uvel->SetMappingMode(KFbxGeometryElement::eBY_POLYGON_VERTEX);
			uvel->SetReferenceMode(KFbxGeometryElement::eINDEX_TO_DIRECT);
		}

		int validface = 0;
		for(int ii = 0;ii < fn;++ii)
		{
			if (!m.face[ii].IsD())
			{
				if (wedgetexcoord)
				{
					vcg::Color4b facecol;
					int textind = m.face[ii].WT(0).N();
					int matind = textind;
					if ((textind == -1) && vcg::tri::HasPerVertexColor(m))
					{	
						std::set<vcg::Color4b>::iterator it = matset.find(m.face[ii].V(0)->C());
						matind = int(std::distance(matset.begin(),it) + m.textures.size());
					}
					vcgmesh->BeginPolygon(matind,textind);
				}
				else
					vcgmesh->BeginPolygon();
				int validvertex = 0;
				for (int jj = 0;jj < 3;++jj)
				{
					
					if (!m.face[ii].V(jj)->IsD())
					{
						int vi_with_invalid = int(m.face[ii].V(jj) - &(*m.vert.begin()));
						int vi = vi_with_invalid - deletedBeforeValid[vi_with_invalid];
						vcgmesh->AddPolygon(vi);
						if (wedgetexcoord)
						{
							ScalarType u = m.face[ii].WT(jj).U();
							ScalarType v = m.face[ii].WT(jj).V();
							
							uvel->GetDirectArray().Add(KFbxVector2(u,v));
							uvel->GetIndexArray().Add(validface * 3 + validvertex);
							++validvertex;
						}
					}
				}
				vcgmesh->EndPolygon();
				++validface;
			}
		}
		/*for(std::set<KFbxVector2>::iterator it = indexset.begin();it != indexset.end();++it)
			uvel->GetDirectArray().Add(*it);*/
		
		KFbxNode* root = scene.GetRootNode();
		root->AddChild(meshnode);

		return true;
	}
};
#endif