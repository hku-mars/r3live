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

#ifndef VCG_UV_UTILS
#define VCG_UV_UTILS


namespace vcg {
	namespace tri{
		template <class MeshType>
		class UV_Utils
		{
		  typedef typename MeshType::CoordType CoordType;
			typedef typename MeshType::ScalarType ScalarType;
			typedef typename MeshType::FaceType FaceType;
			typedef typename MeshType::VertexType VertexType;
			typedef typename MeshType::VertexIterator VertexIterator;
			typedef typename MeshType::FaceIterator FaceIterator;
			typedef typename vcg::Point2<ScalarType> UVCoordType;

		public:
			///calculate the BBox in UV space
			static vcg::Box2<ScalarType> PerWedgeUVBox(MeshType &m)
			{
				vcg::Box2<ScalarType> UVBox;
				FaceIterator fi;
				for (fi=m.face.begin();fi!=m.face.end();fi++)
				{
					if ((*fi).IsD()) continue;
					for (int i=0;i<3;i++)
						UVBox.Add((*fi).WT(i).P());
				}
				return UVBox;
			}
			
			///calculate the BBox in UV space
			static vcg::Box2<ScalarType> PerVertUVBox(MeshType &m)
			{
				vcg::Box2<ScalarType> UVBox;
				VertexIterator vi;
				for (vi=m.vert.begin();vi!=m.vert.end();vi++)
				{
					if ((*vi).IsD()) continue;
					UVBox.Add((*vi).T().P());
				}
				return UVBox;
			}

		void PerWedgeMakeUnitaryUV(MeshType &m)
		{
		  vcg::Box2<typename MeshType::ScalarType> UVBox = PerWedgeUVBox(m);

			typename MeshType::FaceIterator fi;
			Point2f boxSize(UVBox.max-UVBox.min);
			for (fi=m.face.begin();fi!=m.face.end();fi++)
			{
				if ((*fi).IsD()) continue;
				for (int i=0;i<3;i++)
				{
				  (*fi).WT(i).U() = ((*fi).WT(i).U()-UVBox.min[0])/boxSize[0] ;
				  (*fi).WT(i).V() = ((*fi).WT(i).V()-UVBox.min[1])/boxSize[1] ;
				}
			}
		}
			///transform curvature to UV space
			static UVCoordType Coord3DtoUV(FaceType &f,const CoordType &dir)
			{
				///then transform to UV
				CoordType bary3d=(f.P(0)+f.P(1)+f.P(2))/3.0;
				UVCoordType baryUV=(f.WT(0).P()+f.WT(1).P()+f.WT(2).P())/3.0;
				CoordType dir3d=bary3d+dir;
				CoordType baryCoordsUV;
				vcg::InterpolationParameters<FaceType,ScalarType>(f,dir3d,baryCoordsUV);
				UVCoordType dirUV=baryCoordsUV.X()*f.WT(0).P()+
									baryCoordsUV.Y()*f.WT(1).P()+
									baryCoordsUV.Z()*f.WT(2).P()-baryUV;
				dirUV.Normalize();
				return dirUV;
			}

			static void GloballyMirrorX(MeshType &m)
			{
				vcg::Box2<ScalarType> BBuv=PerVertUVBox(m);
				ScalarType Xmin=BBuv.min.X();
				ScalarType Xmax=BBuv.max.X();
				ScalarType XAv=(Xmax+Xmin)/2;
				VertexIterator vi;
				for (vi=m.vert.begin();vi!=m.vert.end();vi++)
				{
					ScalarType distAV=(*vi).T().P().X()-XAv;
					(*vi).T().P().X()=XAv-distAV;
				}
			}
			
			static void LaplacianUVVert(MeshType &m,bool fix_borders=false,int steps=3)
			{
				FaceIterator fi;
				for (int s=0;s<steps;s++)
				{
					std::vector<int> num(m.vert.size(),0);
					std::vector<UVCoordType> UVpos(m.vert.size(),UVCoordType(0,0));
					for (fi=m.face.begin();fi!=m.face.end();fi++)
					{
						for (int j=0;j<3;j++)
						{
							VertexType *v0=(*fi).V(0);
							VertexType *v1=(*fi).V1(0);
							VertexType *v2=(*fi).V2(0);
                            assert(v0!=v1);
                            assert(v1!=v2);
                            assert(v0!=v2);
							UVCoordType uv1=v1->T().P();
							UVCoordType uv2=v2->T().P();
							int index=v0-&(m.vert[0]);
							num[index]+=2;
							UVpos[index]+=uv1;
							UVpos[index]+=uv2;
						}
					}
					VertexIterator vi;
					for (int i=0;i<m.vert.size();i++)
					{
						if ((fix_borders)&&(m.vert[i].IsB()))continue;
                        if (num[i]==0)continue;
						m.vert[i].T().P()=UVpos[i]/(ScalarType)num[i];
					}
				}
			}
		};
	} //End Namespace Tri
} // End Namespace vcg
#endif
