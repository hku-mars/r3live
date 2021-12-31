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

#ifndef VCG_PARAM_DISTORTION
#define VCG_PARAM_DISTORTION
#include <vcg/complex/algorithms/parametrization/uv_utils.h>

namespace vcg {
	namespace tri{
		template <class MeshType, bool PerWedgeFlag>
		class Distortion
		{
		public:
			typedef typename MeshType::FaceType FaceType;
			typedef typename MeshType::VertexType VertexType;
			typedef typename MeshType::CoordType CoordType;
			typedef typename MeshType::ScalarType ScalarType;

			static ScalarType Area3D(const FaceType *f)
			{
				return DoubleArea(*f)*(0.5);
			}

			static ScalarType AreaUV(const FaceType *f)
			{
				Point2<ScalarType> uv0,uv1,uv2;
				if(PerWedgeFlag) {
				  uv0=f->cWT(0).P();
				  uv1=f->cWT(1).P();
				  uv2=f->cWT(2).P();
				} else {
				  uv0=f->cV(0)->T().P();
				  uv1=f->cV(1)->T().P();
				  uv2=f->cV(2)->T().P();
				}
				ScalarType AreaUV=((uv1-uv0)^(uv2-uv0))/2.0;
				return AreaUV;
			}

            static ScalarType EdgeLenght3D(const FaceType *f,int e)
			{
				assert((e>=0)&&(e<3));
				ScalarType length=(f->cP0(e)-f->cP1(e)).Norm();
				return (length);
			}

            static ScalarType EdgeLenghtUV(const FaceType *f,int e)
			{
				assert((e>=0)&&(e<3));
				Point2<ScalarType> uv0,uv1;
				if(PerWedgeFlag) {
                  uv0=f->cWT(e+0).P();
                  uv1=f->cWT((e+1)%3).P();
				} else {
                  uv0=f->cV0(e)->T().P();
                  uv1=f->cV1(e)->T().P();
				}
				ScalarType UVlength=Distance(uv0,uv1);
				return UVlength;
			}

			static ScalarType AngleCos3D(const FaceType *f,int e)
			{
				assert((e>=0)&&(e<3));
				CoordType p0=f->P((e+2)%3);
				CoordType p1=f->P(e);
				CoordType p2=f->P((e+1)%3);
				typedef typename CoordType::ScalarType ScalarType;
				CoordType dir0=p2-p1;
				CoordType dir1=p0-p1;
				dir0.Normalize();
				dir1.Normalize();
				ScalarType angle=dir0*dir1;
				return angle;
			}

			static ScalarType AngleCosUV(const FaceType *f,int e)
			{
			  Point2<ScalarType> uv0,uv1,uv2;
			  if(PerWedgeFlag) {
				uv0=f->cWT((e+2)%3).P();
				uv1=f->cWT((e+0)%3).P();
				uv2=f->cWT((e+1)%3).P();
			  } else {
				uv0=f->V2(e)->T().P();
				uv1=f->V0(e)->T().P();
				uv2=f->V1(e)->T().P();
			  }
				vcg::Point2<ScalarType> dir0=uv2-uv1;
				vcg::Point2<ScalarType> dir1=uv0-uv1;
				dir0.Normalize();
				dir1.Normalize();
				ScalarType angle=dir0*dir1;
				return angle;
			}

			static ScalarType AngleRad3D(const FaceType *f,int e)
			{
				assert((e>=0)&&(e<3));
				CoordType p0=f->cP((e+2)%3);
				CoordType p1=f->cP(e);
				CoordType p2=f->cP((e+1)%3);
				typedef typename CoordType::ScalarType ScalarType;
				CoordType dir0=p2-p1;
				CoordType dir1=p0-p1;
				return Angle(dir0,dir1);
			}

			static ScalarType AngleRadUV(const FaceType *f,int e)
			{
			  Point2<ScalarType> uv0,uv1,uv2;
			  if(PerWedgeFlag) {
				uv0=f->cWT((e+2)%3).P();
				uv1=f->cWT((e+0)%3).P();
				uv2=f->cWT((e+1)%3).P();
			  } else {
				uv0=f->cV2(e)->T().P();
				uv1=f->cV0(e)->T().P();
				uv2=f->cV1(e)->T().P();
			  }
				vcg::Point2<ScalarType> dir0=uv2-uv1;
				vcg::Point2<ScalarType> dir1=uv0-uv1;
				dir0.Normalize();
				dir1.Normalize();
				ScalarType t=dir0*dir1;
				if(t>1) t = 1;
				else if(t<-1) t = -1;
				return acos(t);
			}


		public:
			enum DistType{AreaDist,EdgeDist,AngleDist};

			///return the absolute difference between angle in 3D space and texture space
			///Actually the difference in cos space
			static ScalarType AngleCosDistortion(const FaceType *f,int e)
			{
				ScalarType Angle_3D=AngleCos3D(f,e);
				ScalarType Angle_UV=AngleCosUV(f,e);
				ScalarType diff=fabs(Angle_3D-Angle_UV);///Angle_3D;
				return diff;
			}
			///return the absolute difference between angle in 3D space and texture space
			///Actually the difference in cos space
			static ScalarType AngleRadDistortion(const FaceType *f,int e)
			{
				ScalarType Angle_3D=AngleRad3D(f,e);
				ScalarType Angle_UV=AngleRadUV(f,e);
				ScalarType diff=fabs(Angle_3D-Angle_UV);///Angle_3D;
				return diff;
			}

			///return the variance of angle, normalized
			///in absolute value
			static ScalarType AngleDistortion(const FaceType *f)
			{
				return  AngleRadDistortion(f,0) +
						AngleRadDistortion(f,1) +
						AngleRadDistortion(f,2);
			}

			///return the global scaling factors  from 3D to UV
            static void MeshScalingFactor(const MeshType &m,
                                          ScalarType &AreaScale,
                                          ScalarType &EdgeScale)
			{
				ScalarType SumArea3D=0;
				ScalarType SumArea2D=0;
				ScalarType SumEdge3D=0;
				ScalarType SumEdge2D=0;
				for (int i=0;i<m.face.size();i++)
				{
					SumArea3D+=Area3D(&m.face[i]);
					SumArea2D+=AreaUV(&m.face[i]);
					for (int j=0;j<3;j++)
					{
						SumEdge3D+=EdgeLenght3D(&m.face[i],j);
						SumEdge2D+=EdgeLenghtUV(&m.face[i],j);
					}
				}
				AreaScale=SumArea3D/SumArea2D;
				EdgeScale=SumEdge3D/SumEdge2D;
			}

			///return the variance of edge length, normalized in absolute value,
			// the needed scaling factor EdgeScaleVal may be calculated
			///by using the ScalingFactor function
            static ScalarType EdgeDistortion(const FaceType *f,int e,
				ScalarType EdgeScaleVal)
			{
				ScalarType edgeUV=EdgeLenghtUV(f,e)*EdgeScaleVal;
				ScalarType edge3D=EdgeLenght3D(f,e);
				assert(edge3D > 0);
				ScalarType diff=fabs(edge3D-edgeUV)/edge3D;
				assert(!math::IsNAN(diff));
				return diff;
			}

			///return the variance of area, normalized
			///in absolute value, the scalar AreaScaleVal may be calculated
			///by using the ScalingFactor function
            static ScalarType AreaDistortion(const FaceType *f,
											 ScalarType AreaScaleVal)
			{
			  ScalarType areaUV=AreaUV(f)*AreaScaleVal;
			  ScalarType area3D=Area3D(f);
			  assert(area3D > 0);
			  ScalarType diff=fabs(areaUV-area3D)/area3D;
			  assert(!math::IsNAN(diff));
			  return diff;
			}

			///return the number of folded faces
			static bool Folded(const FaceType *f)
			{
				ScalarType areaUV=AreaUV(f);
				/*if (areaUV<0)
					printf("area %5.5f \n",areaUV);*/
				return (areaUV<0);
			}

			static int Folded(const MeshType &m)
			{
				int folded=0;
				for (size_t i=0;i<m.face.size();i++)
				{
					if (m.face[i].IsD())continue;
					if(Folded(&m.face[i]))folded++;
				}
				return folded;
			}

			static bool GloballyUnFolded(const MeshType &m)
			{
				int num=Folded(m);
				return (num>(m.fn)/2);
			}

			static ScalarType MeshAngleDistortion(const MeshType &m)
			{
				ScalarType UDdist=0;
				for (int i=0;i<m.face.size();i++)
				{
					if (m.face[i].IsD())continue;
					const FaceType *f=&(m.face[i]);
					UDdist+=AngleDistortion(f)*Area3D(f);
				}
				return UDdist;
			}

            static void SetQasDistorsion(MeshType &m,
                                         DistType DType=AreaDist)
            {
                ScalarType edge_scale,area_scale;
                MeshScalingFactor(m,area_scale,edge_scale);
                for (int i=0;i<m.face.size();i++)
                {
                    if (m.face[i].IsD())continue;
                    if (DType==AreaDist)
                        m.face[i].Q()=1-AreaDistortion(&m.face[i],area_scale);
                    else
                    if (DType==AngleDist)
                        m.face[i].Q()=1-AngleDistortion(&m.face[i]);
                    else
                        m.face[i].Q()=3-EdgeDistortion(&m.face[i],0,edge_scale)-
                                        EdgeDistortion(&m.face[i],1,edge_scale)-
                                        EdgeDistortion(&m.face[i],2,edge_scale);
                }
            }
		};
	}
}
#endif
