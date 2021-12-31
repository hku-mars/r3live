#ifndef	IMPORT_FBX
#define IMPORT_FBX

#include<vcg/complex/allocate.h>
#include <wrap/callback.h>
#include <wrap/io_trimesh/io_mask.h>
#include <wrap/io_trimesh/io_material.h>
#include <vcg/space/color4.h>
#include <fbxsdk.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <map>


/******************************
ImporterFBX is the class devoted to import the info contained in a FBX file inside a mesh defined following the vcg standards.
In order to compile the following class you need to:
	- download the VCGlib from our SVN server and put the code in ../../vcglib
	- download from Autodesk website the FBXSDK-2012.1 and put the lib and include folders in ../fbx-2012.1
*/

template <class OpenMeshType>
class ImporterFBX
{
public:

	class Info
	{
	public:

		Info()
		{
			mask	= 0;
			cb		= 0;
			numVertices = 0;
			numFaces = 0;
			numMeshPatches = 0;
		}

		/// It returns a bit mask describing the field preesnt in the ply file
		int mask;  

		/// a Simple callback that can be used for long obj parsing. 
		// it returns the current position, and formats a string with a description of what th efunction is doing (loading vertexes, faces...)
		vcg::CallBackPos *cb;

		/// number of vertices
		int numVertices;
		/// number of faces (the number of triangles could be 
		/// larger in presence of polygonal faces
		int numFaces;
		/// number meshes inside the fbx file
		int numMeshPatches;

	}; // end class
	
	typedef typename OpenMeshType::VertexPointer VertexPointer;
	typedef typename OpenMeshType::ScalarType ScalarType;
	typedef typename OpenMeshType::VertexType VertexType;
	typedef typename OpenMeshType::FaceType FaceType;
	typedef typename OpenMeshType::VertexIterator VertexIterator;
	typedef typename OpenMeshType::FaceIterator FaceIterator;
	typedef typename OpenMeshType::CoordType CoordType;
	typedef typename OpenMeshType::VertexType::ColorType ColorType;
	typedef typename OpenMeshType::VertexType::ColorType::ScalarType CSType;
private:
	class VCGMaterialBridge
	{
	private:
		//colorAttributeName is something like: diffuse color,ambient color etc.
		void insertMaterialInfoAndTextureName(KFbxSurfaceMaterial& mat,const char* colorAttributeName,OpenMeshType& m)
		{
			MaterialInfo matinfo;	
			matinfo.setSurfaceMaterial(&mat);
			//bool hastextcoords = (vcg::tri::HasPerWedgeTexCoord(m) || vcg::tri::HasPerVertexTexCoord(m));
			//bool hascolors =  (vcg::tri::HasPerWedgeColor(m) || vcg::tri::HasPerVertexColor(m) || vcg::tri::HasPerFaceColor(m));
			KFbxFileTexture* tex = matinfo.getTextureFileObject(colorAttributeName); 
			if(tex)
			{
				const char* texfile = tex->GetFileName();
				std::vector<std::string>::iterator it = std::find(m.textures.begin(),m.textures.end(),texfile);
				if (it == m.textures.end())
				{
					m.textures.push_back(texfile);
					matinfo.setTextureIndex(m.textures.size() - 1);
				}
				else
				{
					int dist = int(std::distance(m.textures.begin(),it));
					matinfo.setTextureIndex(dist);
				}
			}	
			else
			{
				if (mat.GetClassId().Is(KFbxSurfaceLambert::ClassId))
				{
					KFbxPropertyDouble3 diffProp = ((KFbxSurfaceLambert* ) (&mat))->Diffuse;
					fbxDouble3 diffCol	= diffProp.Get();
					vcg::Color4b tmpc = vcg::Color4b(diffCol[0] * 255,diffCol[1] * 255,diffCol[2] * 255,255);
					matinfo.setColor(tmpc);
				}
			}
			material[colorAttributeName] = matinfo;
		}
	public:
		//given a texture or color attribute return

		VCGMaterialBridge(OpenMeshType& m,KFbxSurfaceMaterial& mat)
			:material()
		{
			insertMaterialInfoAndTextureName(mat,KFbxSurfaceMaterial::sDiffuse,m);
		}

		class MaterialInfo
		{
		private:
			int textindex;
			ColorType color;
			bool hascolor;
			KFbxSurfaceMaterial* surfMat;
		public:
			MaterialInfo():hascolor(false),textindex(-1),surfMat(NULL){}
			void setColor(const ColorType& col) {color = col;hascolor= true;}
			void setTextureIndex(const int i) {textindex = i;}
			bool hasColor() const {return hascolor;}
			bool hasTexture() const {return (textindex != -1);}
			int getTextureIndex() const {return textindex;}
			KFbxSurfaceMaterial* getSurfaceMaterial() const {return surfMat;}
			void setSurfaceMaterial(KFbxSurfaceMaterial* mat) {surfMat = mat;}
			KFbxFileTexture* getTextureFileObject (const char* attributeName) 
			{
				if (surfMat)
				{	
					KFbxProperty prop = surfMat->FindProperty(attributeName);
					if(prop.IsValid())	
					{
						int texnum = prop.GetSrcObjectCount(KFbxTexture::ClassId);
						//the texnum value should be or 0 or 1 (i.e. for a diffuse texture i should have at most one texture file) but in order to avoid extremely strange file... 
						if (texnum > 0)
							return KFbxCast <KFbxFileTexture> (prop.GetSrcObject(KFbxTexture::ClassId,0));
						else
							return NULL;
					}
				}
				return NULL;
			}

			ColorType getColor() const {return color;}
		};

		MaterialInfo* getMaterial(const char* attr)
		{
                        typename std::map<const char*,MaterialInfo>::iterator it = material.find(attr);
			if (it != material.end())
				return &(it->second);
			return NULL;
		}

	private:
		std::map<const char*,MaterialInfo> material;
	};
	
	static int ImportNode(OpenMeshType& m,KFbxNode* node,KFbxPose* pose,KFbxXMatrix* globPosMat,Info &oi)
	{
		int result = E_NOERROR;
		if (node)
		{
			KFbxXMatrix mat = getGlobalMatrix(node, KTime(), pose, globPosMat);


			KFbxNodeAttribute* attr = node->GetNodeAttribute();
			if (attr != NULL)
			{
				KFbxNodeAttribute::EAttributeType att = attr->GetAttributeType();
				if (attr->GetAttributeType() == KFbxNodeAttribute::eMESH)
				{
					KFbxXMatrix geoOff = GetGeometry(node);
					KFbxXMatrix final = mat * geoOff;

					result = ImportMesh(m,node,final,oi);
				}

			}
			for(int ii = 0;ii < node->GetChildCount();++ii)
				ImportNode(m,node->GetChild(ii),pose,&mat,oi);					
		}
		return result;
	}

	static int ImportScene(OpenMeshType& m,KFbxScene* scene,Info &oi)
	{
		scene->ConnectTextures();
		KFbxPose* pose = NULL;
		int ii = scene->GetPoseCount(); 
		if (ii > 0)
			pose = scene->GetPose(ii-1);
		//KFbxAnimEvaluator* eval = scene->GetEvaluator();
		KFbxNode* node = scene->GetRootNode();
		int result = E_NOERROR;
		KFbxXMatrix globPosMat;
		for (int ii = 0; ii < scene->GetRootNode()->GetChildCount(); ++ii)
			result = ImportNode(m,scene->GetRootNode()->GetChild(ii),pose,&globPosMat,oi);
		return result;
	}

	static int ImportMesh(OpenMeshType& m,KFbxNode* node,const KFbxXMatrix& mat,Info& oi)
	{
		KFbxMesh* newMesh = node->GetMesh();
		if (!newMesh->IsTriangleMesh())
		{
			KFbxGeometryConverter* conv = new KFbxGeometryConverter(newMesh->GetFbxSdkManager());
			newMesh = conv->TriangulateMesh(newMesh);
		}
		//KFbxMatrix mat2(mat);
		vcg::Matrix44<ScalarType> mat2;
		for(int ll = 0;ll < 4;++ll)
			for(int vv = 0;vv < 4;++vv)
				mat2[ll][vv] = mat[vv][ll];

		
		vcg::Matrix33<ScalarType> matnorm;
		for(int ll = 0;ll < 3;++ll)
			for(int vv = 0;vv < 3;++vv)
				matnorm[ll][vv] = mat[vv][ll];

		int matnum = node->GetSrcObjectCount(KFbxSurfaceMaterial::ClassId);
		KFbxLayerElementArrayTemplate< int >* matarr = NULL;
		bool found = newMesh->GetMaterialIndices(&matarr);
		//displacement of material inside m.textures and number of textures inside the material
		std::vector<VCGMaterialBridge> matAtlas; 
		for (int matind = 0; matind < matnum; ++matind)
		{
			KFbxSurfaceMaterial *mater =  KFbxCast <KFbxSurfaceMaterial>(node->GetSrcObject(KFbxSurfaceMaterial::ClassId, matind));
			matAtlas.push_back(VCGMaterialBridge(m,*mater));
		}
		size_t vertDispl = m.vert.size();
		int currmeshvert = newMesh->GetControlPointsCount();
		vcg::tri::Allocator<OpenMeshType>::AddVertices(m,currmeshvert);
		int currmeshface = newMesh->GetPolygonCount();
		size_t faceDispl = m.face.size();
		vcg::tri::Allocator<OpenMeshType>::AddFaces(m,currmeshface);
		KFbxVector4* posVert = newMesh->GetControlPoints();
		KFbxLayerElementArrayTemplate<KFbxVector2>* UVArr = NULL;   
		int uvparcount = newMesh->GetElementUVCount();

		KFbxLayer* lnorm = newMesh->GetLayer(0,KFbxLayerElement::eNORMAL);
		KFbxLayerElementNormal* normalLayer = NULL;
		if (lnorm)
			normalLayer =  lnorm->GetNormals();
		KFbxLayerElement::EMappingMode normalMapMode = KFbxLayerElement::eNONE;
		if (normalLayer)
			normalMapMode = normalLayer->GetMappingMode();
		KFbxLayer* lcol = newMesh->GetLayer(0,KFbxLayerElement::eVERTEX_COLOR);
		KFbxLayerElementVertexColor* colorLayer = NULL;
		if (lcol)
			colorLayer =  lcol->GetVertexColors();
		KFbxLayerElement::EMappingMode colorMapMode = KFbxLayerElement::eNONE;
		if (colorLayer)
			colorMapMode = colorLayer->GetMappingMode();

		newMesh->GetTextureUV(&UVArr, KFbxLayerElement::eDIFFUSE_TEXTURES); 

		int numFacPlusVert = oi.numFaces + oi.numVertices;
		float det = mat2.Determinant();
		bool invert = (det< 0.0);
		for(int ii = 0;ii < currmeshvert;++ii)
		{

			ScalarType a = ScalarType(posVert[ii][0]);
			ScalarType b = ScalarType(posVert[ii][1]);
			ScalarType c = ScalarType(posVert[ii][2]);
			vcg::Point4<ScalarType> fbxpos(a,b,c,ScalarType(1.0));
			fbxpos = mat2 * fbxpos;
			CoordType t(fbxpos[0],fbxpos[1],fbxpos[2]); 
			m.vert[ii + vertDispl].P() = t;
			/*if (vcg::tri::HasPerVertexColor(m) && )*/
			if (vcg::tri::HasPerVertexNormal(m) && (normalLayer) && (normalMapMode ==  KFbxLayerElement::eBY_CONTROL_POINT))
			{
				KFbxVector4 n;
				int normInd = -1;
				switch(normalLayer->GetReferenceMode())
				{
				case (KFbxLayerElement::eDIRECT):
					{
						normInd = ii;
						break;
					}
				case (KFbxLayerElement::eINDEX_TO_DIRECT):
					{
						normInd = normalLayer->GetIndexArray()[ii];
						break;
					}
				}
				n = normalLayer->GetDirectArray()[normInd]; 
				CoordType nn(n[0],n[1],n[2]);
				nn = (matnorm * nn);

				nn.Normalize();
				//if (invert)
				//	nn = -nn;
				m.vert[ii + vertDispl].N() = nn;
			}
			if (vcg::tri::HasPerVertexColor(m) && (colorLayer) && (colorMapMode ==  KFbxLayerElement::eBY_CONTROL_POINT))
			{
				KFbxColor c;
				int colorInd = -1;
				switch(colorLayer->GetReferenceMode())
				{
				case (KFbxLayerElement::eDIRECT):
					{
						colorInd = ii;
						break;
					}
				case (KFbxLayerElement::eINDEX_TO_DIRECT):
					{
						colorInd = colorLayer->GetIndexArray()[ii];
						break;
					}
				}
				c = colorLayer->GetDirectArray()[colorInd];
				vcg::Color4b vcgcol(c[0] * 255,c[1] * 255,c[2] * 255,c[3] * 255);
				m.vert[ii + vertDispl].C() = vcgcol;
			}
		}

		int* index = newMesh->GetPolygonVertices();
		bool allsame = false;
		int matarrsize = -1;
		if (matarr)
			matarrsize = matarr->GetCount();
		if (matarrsize == 1)
			allsame = true;

		for(int ii = 0;ii < currmeshface;++ii)
		{
			int diffusematind = -1;
			int matptr = -1;
                        typename VCGMaterialBridge::MaterialInfo* mymat = NULL;
			if (matAtlas.size() > 0)
			{
				if ((!allsame) && (matarrsize == currmeshface))
					matptr = (*matarr)[ii];
				else
				{
					if (allsame) 
					{
						matptr = (*matarr)[0]; 	
					}
				}
				if ((matptr >= 0) && (matptr < matAtlas.size()))
				{
					mymat = matAtlas[matptr].getMaterial(KFbxSurfaceMaterial::sDiffuse);
					if (mymat && mymat->hasTexture())
						diffusematind = mymat->getTextureIndex();
				}
				else
					if (matptr == node->GetMaterialCount())
					{
						mymat = matAtlas[matptr-1].getMaterial(KFbxSurfaceMaterial::sDiffuse);
						if (mymat && mymat->hasTexture())
							diffusematind = mymat->getTextureIndex();
					}
			}
			for (int jj = 0;jj < 3;++jj)
			{
				VertexIterator mov = m.vert.begin();
				std::advance(mov,index[ii * 3 + jj] + vertDispl);
				int invind = (3 - jj) % 3;
				if (!invert)
					m.face[ii + faceDispl].V(jj) = &(*mov);
				else
					m.face[ii + faceDispl].V(invind) = &(*mov);
				if (vcg::tri::HasPerWedgeTexCoord(m) && (uvparcount > 0) && mymat && mymat->hasTexture())
				{
					int texind = -1;
					KFbxLayerElement::EReferenceMode mapmode = newMesh->GetElementUV(0)->GetReferenceMode();
                                        if (mapmode == KFbxLayerElement::eINDEX_TO_DIRECT)
						texind  = newMesh->GetTextureUVIndex(ii,jj);
					else 
                                                if (mapmode == KFbxLayerElement::eDIRECT)
							texind = ii * 3 + jj;
					if (UVArr)
					{
						
						KFbxVector2 texcoord =  UVArr->GetAt(texind); 
						if (!invert)
						{
							m.face[ii + faceDispl].WT(jj).U() = ScalarType(texcoord[0]);
							m.face[ii + faceDispl].WT(jj).V() = ScalarType(texcoord[1]);
							m.face[ii + faceDispl].WT(jj).N() = diffusematind;
						}
						else
						{
							m.face[ii + faceDispl].WT(invind).U() = ScalarType(texcoord[0]);
							m.face[ii + faceDispl].WT(invind).V() = ScalarType(texcoord[1]);
							m.face[ii + faceDispl].WT(invind).N() = diffusematind;
						}
					}
				}

				else 
					if (vcg::tri::HasPerVertexColor(m) && mymat && mymat->hasColor())
					{
						if (!invert)
							m.face[ii + faceDispl].V(jj)->C() = mymat->getColor();
						else
							m.face[ii + faceDispl].V(invind)->C() = mymat->getColor();
					}
			}
			if (oi.cb)
			{
				char numstr[33];
				sprintf(numstr,"%d",oi.numMeshPatches);
				std::string st("Loading Vertices and Faces from ");
				st = st + numstr + " submeshes";
				oi.cb( (100*(m.vert.size() +m.face.size()))/ numFacPlusVert, st.c_str());
			}
		}
		
		//std::string tx = m.textures[firstTexInd];
		return E_NOERROR;
	}	

	static void LoadMaskNode(KFbxNode* node, Info &oi)
	{
		if (node)
		{
			KFbxNodeAttribute* attr = node->GetNodeAttribute();
			if (attr != NULL)
			{
				KFbxNodeAttribute::EAttributeType att = attr->GetAttributeType();
				if (attr->GetAttributeType() == KFbxNodeAttribute::eMESH)
				{
					int matnum = node->GetSrcObjectCount(KFbxSurfaceMaterial::ClassId);
					KFbxMesh* newMesh = node->GetMesh();
					int uvparcount = newMesh->GetElementUVCount();
					if (uvparcount > 0)
						oi.mask |= vcg::tri::io::Mask::IOM_WEDGTEXCOORD;

					KFbxLayer* lcolo = newMesh->GetLayer(0,KFbxLayerElement::eVERTEX_COLOR);
					if (matnum || lcolo)
						oi.mask |= vcg::tri::io::Mask::IOM_VERTCOLOR;
					KFbxLayer* lnorm = newMesh->GetLayer(0,KFbxLayerElement::eNORMAL);
					KFbxLayerElementNormal* nnom = lnorm->GetNormals();
					if (lnorm)	
						oi.mask |= vcg::tri::io::Mask::IOM_VERTNORMAL;
					oi.numVertices = oi.numVertices + newMesh->GetControlPointsCount();
					if (!newMesh->IsTriangleMesh())
					{
						KFbxGeometryConverter* conv = new KFbxGeometryConverter(newMesh->GetFbxSdkManager());
						newMesh = conv->TriangulateMesh(newMesh);
					}
					oi.numFaces = oi.numFaces + newMesh->GetPolygonCount();
					oi.numMeshPatches += 1;
				}
			}
			for(int ii = 0;ii < node->GetChildCount();++ii)
				LoadMaskNode(node->GetChild(ii),oi);					
		}
	}

	static KFbxXMatrix getGlobalMatrix(KFbxNode* pNode, const KTime& pTime, KFbxPose* pPose, KFbxXMatrix* pParentGlobalPosition = NULL)
	{
		KFbxXMatrix lGlobalPosition;
		bool        lPositionFound = false;

		if (pPose)
		{
			int lNodeIndex = pPose->Find(pNode);

			if (lNodeIndex > -1)
			{
				if (pPose->IsBindPose() || !pPose->IsLocalMatrix(lNodeIndex))
				{
					lGlobalPosition = GetPoseMatrix(pPose, lNodeIndex);
				}
				else
				{
					KFbxXMatrix lParentGlobalPosition;

					if (pParentGlobalPosition)
					{
						lParentGlobalPosition = *pParentGlobalPosition;
					}
					else
					{
						if (pNode->GetParent())
						{
							lParentGlobalPosition = getGlobalMatrix(pNode->GetParent(), pTime, pPose);
						}
					}

					KFbxXMatrix lLocalPosition = GetPoseMatrix(pPose, lNodeIndex);
					lGlobalPosition = lParentGlobalPosition * lLocalPosition;
				}

				lPositionFound = true;
			}
		}

		if (!lPositionFound)
		{
			lGlobalPosition = pNode->EvaluateGlobalTransform(pTime);
		}

		return lGlobalPosition;
	}

	static KFbxXMatrix GetPoseMatrix(KFbxPose* pPose, int pNodeIndex)
	{
		KFbxXMatrix lPoseMatrix;
		KFbxMatrix lMatrix = pPose->GetMatrix(pNodeIndex);

		memcpy((double*)lPoseMatrix, (double*)lMatrix, sizeof(lMatrix.mData));

		return lPoseMatrix;
	}

	static KFbxXMatrix GetGeometry(KFbxNode* pNode)
	{
		const KFbxVector4 lT = pNode->GetGeometricTranslation(KFbxNode::eSOURCE_SET);
		const KFbxVector4 lR = pNode->GetGeometricRotation(KFbxNode::eSOURCE_SET);
		const KFbxVector4 lS = pNode->GetGeometricScaling(KFbxNode::eSOURCE_SET);

		return KFbxXMatrix(lT, lR, lS);
	}

public:
	/*****
		Enum containing possible error codes you can get opening an FBX file.
	*/
	enum FBXError 
	{
		// Successfull opening
		E_NOERROR = 0,													

		// Critical Opening Errors (only even numbers)
		E_CANTCREATEFBXMANAGER	= 1,	
		E_CANTCREATEFBXSCENE	= 2,
		E_CANTCREATEFBXIMPORTER	= 3,		
		E_CANTFINDFILE			= 4,	
		E_CANTFINDFBXFILE		= 5,
		E_CANTIMPORTFBXFILE		= 6	
	};

	/****
		Function devoted to check if a given error is critical or the elaboration can go on.
	*/
	static bool ErrorCritical(int err)
	{ 
		if(err == E_NOERROR) 
			return false;
		return true;
	}

	/****
		The function return the sting info associated with an error code.
	*/
	static const char* ErrorMsg(int error)
	{
		static const char* fbx_error_msg[] =
		{
			"No errors",

			"Failed to create a KFbxSdkManager",
			"Failed to create a KFbxScene",
			"Failed to create a KFbxImporter",
			"Failed to locate requested file",
			"Failed to import requested file",
		};

		if(error>6 || error<0) 
			return "Unknown error";
		else 
			return fbx_error_msg[error];
	};

	/****
		The open function import info contained in a FBX file into a mesh defined using the VCG standards.
		Parameters:
			- m is the mesh in which you want to load the file info
			- filename is the filename path of a valid FBX file. It could be a relative path or an absolute one.
			- oi is a small structure devoted to contain some feedbacks about the loaded file (like vertex number or face number). Typically you can ignore it. 

		Semantics:
			If the file has been correctly loaded (i.e. Open function returned a E_NOERROR code, you will find in the mesh m at least:
				- a vertex position for each vertex
				- the triangles componing the mesh surface

			Depending on which type mesh you passed to the function you could also have:
				- a vertex normal for each vertex
				- a vertex color (ONLY IF THE FBX FILE HAS ALL THE MATERIALS WITHOUT TEXTURES AT ALL)
				- a single material for each triangle. We are using the one containing a diffuse texture and/or a diffuse color. 
				- a UV parameterization for each vertex componing a triangle

			Please note that if the file is composed by a set of meshes we compact all these meshes in a single one.
	*/

	static int Open( OpenMeshType &m, const char * filename, Info &oi)
	{
		int result = E_NOERROR;
		m.Clear();
		vcg::CallBackPos *cb = oi.cb;

		KFbxSdkManager* FBXmanager = KFbxSdkManager::Create();
		if (FBXmanager==NULL)
			return E_CANTCREATEFBXMANAGER;
		KFbxScene* FBXscene = KFbxScene::Create(FBXmanager,"");
		if (FBXscene==NULL)
			return E_CANTCREATEFBXSCENE;
		KFbxImporter* importer = KFbxImporter::Create(FBXmanager,"");
		if (importer==NULL)
			return E_CANTCREATEFBXIMPORTER;

		bool initfile = importer->Initialize(filename);
		if (!initfile)
		{
			printf("Fbx error: %s",importer->GetLastErrorString());		
			return E_CANTFINDFBXFILE;
		}
		bool importfile = importer->Import(FBXscene);
		if (!importfile)
		{
			printf("Fbx error: %s",importer->GetLastErrorString());		
			return E_CANTIMPORTFBXFILE;
		}
		result = ImportScene(m,FBXscene,oi);

		// if LoadMask has not been called yet, we call it here
		
		if (FBXmanager)
			FBXmanager->Destroy();
	  return result;
	} 

	/****
	The LoadMask function gives you a preview of which attributes you will find inside the fbx file. 
	It's useful only if you are planning to use a mesh with dynamic attributes.

	Parameters:
		- filename is the filename path of a valid FBX file. It could be a relative path or an absolute one.
		- oi is a small structure devoted to contain some feedbacks about the loaded file (like vertex number or face number). 
		In oi.mask you will find the mask containing the attributes contained in the file. You can query the presence of a specific attribute using the operators on bits.
		( for example if I want to check if the file contains the vertex normal layer I have to call the LoadMask function and after check if (oi.mask & vcg::tri::io::Mask::IOM_VERTNORMAL) is true).
	*/

	static int LoadMask(const char * filename, Info &oi)
	{
		KFbxSdkManager* FBXmanager = KFbxSdkManager::Create();
		if (FBXmanager==NULL)
			return E_CANTCREATEFBXMANAGER;
		KFbxScene* FBXscene = KFbxScene::Create(FBXmanager,"");
		if (FBXscene==NULL)
			return E_CANTCREATEFBXSCENE;
		KFbxImporter* importer = KFbxImporter::Create(FBXmanager,"");
		if (importer==NULL)
			return E_CANTCREATEFBXIMPORTER;

		bool initfile = importer->Initialize(filename);
		if (!initfile)
		{
			printf("Fbx error: %s",importer->GetLastErrorString());		
			return E_CANTFINDFBXFILE;
		}
		bool importfile = importer->Import(FBXscene);
		if (!importfile)
		{
			printf("Fbx error: %s",importer->GetLastErrorString());		
			return E_CANTIMPORTFBXFILE;
		}
		LoadMaskNode(FBXscene->GetRootNode(),oi);

		// if LoadMask has not been called yet, we call it here

		if (FBXmanager)
			FBXmanager->Destroy();
		return E_NOERROR;
	}
	
}; // end class


#endif 
