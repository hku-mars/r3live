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


#ifndef __VCG_TRIMESH_CLOSEST
#define __VCG_TRIMESH_CLOSEST
#include <math.h>

#include <vcg/space/point3.h>
#include <vcg/space/box3.h>
#include <vcg/space/point4.h>
#include <vcg/math/base.h>
#include <vcg/simplex/face/distance.h>
#include <vcg/simplex/vertex/distance.h>
#include <vcg/space/intersection3.h>
#include <vcg/space/index/space_iterators.h>
#include <vcg/complex/complex.h>

namespace vcg {
	namespace tri {

		//**MARKER CLASSES**//
		template <class MESH_TYPE,class OBJ_TYPE>
		class Tmark
		{
			MESH_TYPE *m;
		public:
			Tmark(){}
			Tmark(	MESH_TYPE *m) {SetMesh(m);}
			void UnMarkAll(){ vcg::tri::UnMarkAll(*m);}
			bool IsMarked(OBJ_TYPE* obj){return (vcg::tri::IsMarked(*m,obj));}
			void Mark(OBJ_TYPE* obj){ vcg::tri::Mark(*m,obj);}
			void SetMesh(MESH_TYPE *_m)
			{m=_m;}
		};

		template <class MESH_TYPE>
		class FaceTmark:public Tmark<MESH_TYPE,typename MESH_TYPE::FaceType>
		{
		public:
			FaceTmark(){}
			FaceTmark(MESH_TYPE *m) {this->SetMesh(m);}
		};

		template <class MESH_TYPE>
		class VertTmark
		{
		public:
		typedef typename  MESH_TYPE::VertexType VertexType;
			inline VertTmark(){}
			inline VertTmark(MESH_TYPE *){}
			inline void UnMarkAll() const {}
			inline bool IsMarked(VertexType*) const { return false; }
			inline void Mark(VertexType*) const {}
			inline void SetMesh(void * /*m=0*/) const {}
		};

		//**CLOSEST FUNCTION DEFINITION**//

		/*

		aka MetroCore
		data una mesh m e una ug sulle sue facce trova il punto di m piu' vicino ad
		un punto dato.
		*/

		// input: mesh, punto, griglia (gr), distanza limite (mdist)
		// output: normale (interpolata) alla faccia e punto piu' vicino su di essa, e coord baricentriche del punto trovato

		// Nota che il parametro template GRID non ci dovrebbe essere, visto che deve essere 
		// UGrid<MESH::FaceContainer >, ma non sono riuscito a definirlo implicitamente 

		template <class MESH, class GRID>
			typename MESH::FaceType * GetClosestFaceEP( MESH & mesh, GRID & gr, const typename GRID::CoordType & _p,
														const typename GRID::ScalarType & _maxDist, typename GRID::ScalarType & _minDist,
														typename GRID::CoordType & _closestPt, typename GRID::CoordType & _normf,
														typename GRID::CoordType & _ip)
		{
			typedef typename GRID::ScalarType ScalarType;
			typedef Point3<ScalarType> Point3x;

			typedef FaceTmark<MESH> MarkerFace;
			MarkerFace mf(&mesh);
			vcg::face::PointDistanceEPFunctor<ScalarType> FDistFunct;
			_minDist=_maxDist;
			typename MESH::FaceType* bestf= gr.GetClosest(FDistFunct, mf, _p, _maxDist, _minDist, _closestPt);

			if(_maxDist> ScalarType(fabs(_minDist)))
			{
				// f=bestf;
				//calcolo normale con interpolazione trilineare
			  InterpolationParameters<typename MESH::FaceType,typename MESH::ScalarType>(*bestf,bestf->N(),_closestPt, _ip);
			  _normf =  (bestf->V(0)->cN())*_ip[0]+
				  (bestf->V(1)->cN())*_ip[1]+
				  (bestf->V(2)->cN())*_ip[2] ;

			  _minDist = fabs(_minDist);
				return(bestf);
			}
			return (0);
		}

		template <class MESH, class GRID>
			typename MESH::FaceType * GetClosestFaceBase( MESH & mesh,GRID & gr,const typename GRID::CoordType & _p,
														  const typename GRID::ScalarType _maxDist,typename GRID::ScalarType & _minDist,
														  typename GRID::CoordType &_closestPt)
		{
			typedef typename GRID::ScalarType ScalarType;
			typedef Point3<ScalarType> Point3x;
			typedef FaceTmark<MESH> MarkerFace;
			MarkerFace mf;
			mf.SetMesh(&mesh);
			vcg::face::PointDistanceBaseFunctor<ScalarType> PDistFunct;
			_minDist=_maxDist;
			return (gr.GetClosest(PDistFunct,mf,_p,_maxDist,_minDist,_closestPt));
		}

		template <class MESH, class GRID>
			typename MESH::FaceType * GetClosestFaceEP( MESH & mesh,GRID & gr,const typename GRID::CoordType & _p,
			const typename GRID::ScalarType _maxDist, typename GRID::ScalarType & _minDist,
			typename GRID::CoordType &_closestPt)
		{
			typedef typename GRID::ScalarType ScalarType;
			typedef Point3<ScalarType> Point3x;
			typedef FaceTmark<MESH> MarkerFace;
			MarkerFace mf;
			mf.SetMesh(&mesh);
			vcg::face::PointDistanceEPFunctor<ScalarType> PDistFunct;
			_minDist=_maxDist;
			return (gr.GetClosest(PDistFunct,mf,_p,_maxDist,_minDist,_closestPt));
		}

		template <class MESH, class GRID>
		typename MESH::FaceType * GetClosestFaceNormal(MESH & mesh,GRID & gr,const typename MESH::VertexType & _p, 
			const typename GRID::ScalarType & _maxDist,typename GRID::ScalarType & _minDist,
			typename GRID::CoordType &_closestPt)
		{
			typedef typename GRID::ScalarType ScalarType;
			typedef FaceTmark<MESH> MarkerFace;
			MarkerFace mf;
			mf.SetMesh(&mesh);
			typedef vcg::face::PointNormalDistanceFunctor<typename MESH::VertexType> PDistFunct;
			PDistFunct fn;
			_minDist=_maxDist;
			//return (gr.GetClosest(PDistFunct,mf,_p,_maxDist,_minDist,_closestPt.P()));
			return (gr.template GetClosest <PDistFunct,MarkerFace>(fn,mf,_p,_maxDist,_minDist,_closestPt));
		}

		template <class MESH, class GRID>
			typename MESH::VertexType * GetClosestVertex( MESH & mesh,GRID & gr,const typename GRID::CoordType & _p, 
			const typename GRID::ScalarType & _maxDist,typename GRID::ScalarType & _minDist )
		{
			typedef typename GRID::ScalarType ScalarType;
			typedef Point3<ScalarType> Point3x;
			typedef VertTmark<MESH> MarkerVert;
			MarkerVert mv;
			mv.SetMesh(&mesh);
			typedef vcg::vertex::PointDistanceFunctor<typename MESH::ScalarType> VDistFunct;
			VDistFunct fn;
			_minDist=_maxDist;
			Point3x _closestPt;
			return (gr.template GetClosest<VDistFunct,MarkerVert>(fn,mv,_p,_maxDist,_minDist,_closestPt));
		}

		template <class MESH, class GRID>
			typename MESH::VertexType * GetClosestVertexScale( MESH & mesh,GRID & gr,const typename GRID::CoordType & _p, 
			const typename MESH::CoordType & center, const typename GRID::ScalarType & _maxDist,typename GRID::ScalarType & _minDist )
		{
			typedef typename GRID::ScalarType ScalarType;
			typedef Point3<ScalarType> Point3x;
			typedef VertTmark<MESH> MarkerVert;
			MarkerVert mv;
			mv.SetMesh(&mesh);
			typedef vcg::vertex::PointScaledDistanceFunctor<typename MESH::VertexType> VDistFunct;
			VDistFunct fn;
			fn.Cen() = center;
			_minDist=_maxDist;
			Point3x _closestPt;
			return (gr.template GetClosest<VDistFunct,MarkerVert>(fn,mv,_p,_maxDist,_minDist,_closestPt));
		}

		template <class MESH, class GRID>
			typename MESH::VertexType * GetClosestVertexNormal( MESH & mesh,GRID & gr,const typename MESH::VertexType & _p, 
			const typename GRID::ScalarType & _maxDist,typename GRID::ScalarType & _minDist )
		{
			typedef typename GRID::ScalarType ScalarType;
			typedef Point3<ScalarType> Point3x;
			typedef VertTmark<MESH> MarkerVert;
			MarkerVert mv;
			mv.SetMesh(&mesh);
			typedef vcg::vertex::PointNormalDistanceFunctor<typename MESH::VertexType> VDistFunct;
			VDistFunct fn;
			_minDist=_maxDist;
			Point3x _closestPt;
			return (gr.template GetClosest <VDistFunct,MarkerVert>(fn,mv,_p,_maxDist,_minDist,_closestPt));
		}

    template <class MESH, class GRID, class OBJPTRCONTAINER,class DISTCONTAINER, class POINTCONTAINER>
      unsigned int GetKClosestFaceEP(MESH & mesh,GRID & gr, const unsigned int _k,
      const typename GRID::CoordType & _p, const typename GRID::ScalarType & _maxDist,
      OBJPTRCONTAINER & _objectPtrs,DISTCONTAINER & _distances, POINTCONTAINER & _points)
    {
      typedef FaceTmark<MESH> MarkerFace;
      MarkerFace mf;
      mf.SetMesh(&mesh);
      vcg::face::PointDistanceEPFunctor<typename MESH::ScalarType> FDistFunct;
      return (gr.GetKClosest /*<vcg::face::PointDistanceFunctor, MarkerFace,OBJPTRCONTAINER,DISTCONTAINER,POINTCONTAINER>*/
        (FDistFunct,mf,_k,_p,_maxDist,_objectPtrs,_distances,_points));
    }

    // This version does not require that the face type has the
    // EdgePlane component and use a less optimized (but more memory efficient) point-triangle distance
    template <class MESH, class GRID, class OBJPTRCONTAINER,class DISTCONTAINER, class POINTCONTAINER>
      unsigned int GetKClosestFaceBase(MESH & mesh,GRID & gr, const unsigned int _k,
      const typename GRID::CoordType & _p, const typename GRID::ScalarType & _maxDist,
      OBJPTRCONTAINER & _objectPtrs,DISTCONTAINER & _distances, POINTCONTAINER & _points)
    {
      typedef FaceTmark<MESH> MarkerFace;
      MarkerFace mf;
      mf.SetMesh(&mesh);
      vcg::face::PointDistanceBaseFunctor<typename MESH::ScalarType> FDistFunct;
      return (gr.GetKClosest /*<vcg::face::PointDistanceFunctor, MarkerFace,OBJPTRCONTAINER,DISTCONTAINER,POINTCONTAINER>*/
        (FDistFunct,mf,_k,_p,_maxDist,_objectPtrs,_distances,_points));
    }

		template <class MESH, class GRID, class OBJPTRCONTAINER,class DISTCONTAINER, class POINTCONTAINER>
			unsigned int GetKClosestVertex(MESH & mesh,GRID & gr, const unsigned int _k, 
			const typename GRID::CoordType & _p, const typename GRID::ScalarType & _maxDist,
			OBJPTRCONTAINER & _objectPtrs,DISTCONTAINER & _distances, POINTCONTAINER & _points)
		{
			typedef VertTmark<MESH> MarkerVert;
			MarkerVert mv;
			mv.SetMesh(&mesh);			
			typedef vcg::vertex::PointDistanceFunctor<typename MESH::ScalarType> VDistFunct;
			VDistFunct distFunct;
			return (gr.GetKClosest/* <VDistFunct,MarkerVert,OBJPTRCONTAINER,DISTCONTAINER,POINTCONTAINER>*/
				(distFunct,mv,_k,_p,_maxDist,_objectPtrs,_distances,_points));
		}

		template <class MESH, class GRID, class OBJPTRCONTAINER, class DISTCONTAINER, class POINTCONTAINER>
			unsigned int GetInSphereFaceBase(MESH & mesh,
			GRID & gr,
			const typename GRID::CoordType & _p,
			const typename GRID::ScalarType & _r,
			OBJPTRCONTAINER & _objectPtrs,
			DISTCONTAINER & _distances, 
			POINTCONTAINER & _points)
		{
			typedef FaceTmark<MESH> MarkerFace;
			MarkerFace mf;
			mf.SetMesh(&mesh);
			typedef vcg::face::PointDistanceBaseFunctor<typename MESH::ScalarType> FDistFunct;
			return (gr.GetInSphere/*<FDistFunct,MarkerFace,OBJPTRCONTAINER,DISTCONTAINER,POINTCONTAINER>*/
				(FDistFunct(),mf,_p,_r,_objectPtrs,_distances,_points));
		}

		template <class MESH, class GRID, class OBJPTRCONTAINER, class DISTCONTAINER, class POINTCONTAINER>
			unsigned int GetInSphereVertex(MESH & mesh,
			GRID & gr,
			const typename GRID::CoordType & _p,
			const typename GRID::ScalarType & _r,
			OBJPTRCONTAINER & _objectPtrs,
			DISTCONTAINER & _distances, 
			POINTCONTAINER & _points)
		{
			typedef VertTmark<MESH> MarkerVert;
			MarkerVert mv;
			mv.SetMesh(&mesh);
			typedef vcg::vertex::PointDistanceFunctor<typename MESH::ScalarType> VDistFunct;
			VDistFunct fn;
			return (gr.GetInSphere/*<VDistFunct,MarkerVert,OBJPTRCONTAINER,DISTCONTAINER,POINTCONTAINER>*/
				(fn, mv,_p,_r,_objectPtrs,_distances,_points));
		}

		template <class MESH, class GRID, class OBJPTRCONTAINER>
			unsigned int GetInBoxFace(MESH & mesh,
			GRID & gr,
			const vcg::Box3<typename GRID::ScalarType> _bbox,
			OBJPTRCONTAINER & _objectPtrs) 
		{
			typedef FaceTmark<MESH> MarkerFace;
			MarkerFace mf(&mesh);
			return(gr.GetInBox/*<MarkerFace,OBJPTRCONTAINER>*/(mf,_bbox,_objectPtrs));
		}

		template <class MESH, class GRID, class OBJPTRCONTAINER>
			unsigned int GetInBoxVertex(MESH & mesh,
			GRID & gr,
			const vcg::Box3<typename GRID::ScalarType> _bbox,
			OBJPTRCONTAINER & _objectPtrs) 
		{
			typedef VertTmark<MESH> MarkerVert;
			MarkerVert mv;
			mv.SetMesh(&mesh);
			return(gr.GetInBox/*<MarkerVert,OBJPTRCONTAINER>*/(mv,_bbox,_objectPtrs));
		}

		template <class MESH, class GRID>
			typename GRID::ObjPtr DoRay(MESH & mesh,GRID & gr, const Ray3<typename GRID::ScalarType> & _ray,
			const typename GRID::ScalarType & _maxDist, typename GRID::ScalarType & _t) 
		{
			typedef typename MESH::FaceType FaceType;
			typedef typename MESH::ScalarType ScalarType;
			typedef FaceTmark<MESH> MarkerFace;
			MarkerFace mf;
			mf.SetMesh(&mesh);
			Ray3<typename GRID::ScalarType> _ray1=_ray;
			_ray1.Normalize();
			typedef vcg::RayTriangleIntersectionFunctor<true> FintFunct;
      FintFunct ff;
      return(gr.DoRay(ff,mf,_ray1,_maxDist,_t));
		}
		
		template <class MESH, class GRID>
			typename GRID::ObjPtr DoRay(MESH & mesh,GRID & gr, const Ray3<typename GRID::ScalarType> & _ray,
										const typename GRID::ScalarType & _maxDist, 
										typename GRID::ScalarType & _t, 
										typename GRID::CoordType & _normf) 
		{
			typedef typename MESH::FaceType FaceType;
			typedef typename MESH::ScalarType ScalarType;
			typedef FaceTmark<MESH> MarkerFace;
			MarkerFace mf;
			mf.SetMesh(&mesh);
			typedef vcg::RayTriangleIntersectionFunctor<true> FintFunct;
            FintFunct fintfunct;
			Ray3<typename GRID::ScalarType> _ray1=_ray;
			_ray1.Normalize();
            FaceType *f=gr.DoRay(fintfunct,mf,_ray1,_maxDist,_t);
			typename GRID::CoordType dir=_ray.Direction();
			dir.Normalize();
			typename GRID::CoordType int_point=_ray.Origin()+_ray1.Direction()*_t;
			if (f!=NULL)
			{
                Point3<typename GRID::ScalarType> bary;
                InterpolationParameters<FaceType,ScalarType>(*f,f->N(),int_point, bary);

                _normf =  (f->V(0)->cN())*bary.X()+
                                    (f->V(1)->cN())*bary.Y()+
                                    (f->V(2)->cN())*bary.Z() ;
			}
			return f;
		}

		///Iteratively Do Ray sampling on spherical coordinates 
		///sampling along the two angles
		template <class MESH, class GRID, class OBJPTRCONTAINER, class COORDCONTAINER>
			void RaySpherical(MESH & mesh,GRID & gr, const Ray3<typename GRID::ScalarType> & _ray,
											   const typename GRID::ScalarType & _maxDist,
										   	   const typename GRID::ScalarType & _theta_interval,
											   const typename GRID::ScalarType & _phi_interval,
										 	   const int &n_samples,
											   OBJPTRCONTAINER & _objectPtrs,
											   COORDCONTAINER & _pos,
												 COORDCONTAINER & _norm) 
		{
			typedef typename MESH::FaceType FaceType;
			typedef typename MESH::ScalarType ScalarType;
			ScalarType delta_theta=_theta_interval/(ScalarType)(n_samples*2);
      ScalarType delta_phi  =_phi_interval/(ScalarType)(n_samples*2);
      ScalarType theta_init,phi_init,ro;
      typename GRID::CoordType dir0=_ray.Direction();
      dir0.ToPolarRad(ro,theta_init,phi_init);
			for (int x=-n_samples;x<=n_samples;x++)
				for (int y=-n_samples;y<=n_samples;y++)
				{
					ScalarType theta=theta_init+x*delta_theta;
          if (theta<0) theta=2.0*M_PI-theta;
          ScalarType phi=phi_init+y*delta_phi;

          typename GRID::CoordType dir;
          dir.FromxPolar(ro,theta,phi);
					dir.Normalize();
					Ray3<typename GRID::ScalarType> curr_ray(_ray.Origin(),dir);
          typename GRID::ScalarType _t;
          typename GRID::ObjPtr f=NULL;
					f=DoRay(mesh,gr,curr_ray,_maxDist,_t);
					if (f!=NULL)
					{
            typename GRID::CoordType pos=curr_ray.Origin()+curr_ray.Direction()*_t;
						_objectPtrs.push_back(f);
						_pos.push_back(pos);

						///find the normal
						typename GRID::ScalarType alfa,beta,gamma;
						InterpolationParameters<FaceType,ScalarType>(*f,*f.N(),pos, alfa, beta, gamma);
						typename GRID::CoordType norm =  (f->V(0)->cN())*alfa+
																						 (f->V(1)->cN())*beta+
																						 (f->V(2)->cN())*gamma ;
						_norm.push_back(norm);
					}
				}
			}

		//**ITERATORS DEFINITION**//

		template <class GRID,class MESH>
		class ClosestFaceEPIterator:public vcg::ClosestIterator<GRID,
			vcg::face::PointDistanceEPFunctor<typename MESH::ScalarType>, FaceTmark<MESH> >
		{
		public:
			typedef GRID GridType;
			typedef MESH MeshType;
			typedef FaceTmark<MESH> MarkerFace;
			typedef vcg::face::PointDistanceEPFunctor<typename MESH::ScalarType> PDistFunct;
			typedef vcg::ClosestIterator<GRID,PDistFunct, FaceTmark<MESH> > ClosestBaseType;
			typedef typename MESH::FaceType FaceType;
			typedef typename MESH::ScalarType ScalarType;

			//ClosestFaceIterator(GridType &_Si):ClosestBaseType(_Si,PDistFunct<FaceType,ScalarType>()){}
			ClosestFaceEPIterator(GridType &_Si):ClosestBaseType(_Si,PDistFunct()){}

//    Commented out: it seems unuseful and make gcc complain. p.
      void SetMesh(MeshType *m)
			{this->tm.SetMesh(m);}
		};

		template <class GRID,class MESH>
		class ClosestVertexIterator:public vcg::ClosestIterator<GRID, vcg::vertex::PointDistanceFunctor<typename MESH::ScalarType>, VertTmark<MESH> >
		{
		public:
			typedef GRID GridType;
			typedef MESH MeshType;
			typedef VertTmark<MESH> MarkerVert;
			typedef vcg::vertex::PointDistanceFunctor<typename MESH::ScalarType> VDistFunct;
			typedef vcg::ClosestIterator<GRID, VDistFunct, VertTmark<MESH> > ClosestBaseType;
            VDistFunct fn;
			ClosestVertexIterator(GridType &_Si):ClosestBaseType(_Si,fn){}

//    Commented out: it seems unuseful and make gcc complain. p.
			void SetMesh(MeshType *m)
			{this->tm.SetMesh(m);}
		};

		template <class GRID,class MESH>
		class TriRayIterator:public vcg::RayIterator<GRID,vcg::RayTriangleIntersectionFunctor<true>,FaceTmark<MESH> >
		{
		public:
			typedef GRID GridType;
			typedef MESH MeshType;
			typedef FaceTmark<MESH> MarkerFace;
			typedef vcg::RayTriangleIntersectionFunctor<true> FintFunct;
			typedef vcg::RayIterator<GRID,FintFunct, FaceTmark<MESH> > RayBaseType;
			typedef typename MESH::FaceType FaceType;
			typedef typename MESH::ScalarType ScalarType;

			TriRayIterator(GridType &_Si,const ScalarType &max_d):RayBaseType(_Si,FintFunct(),max_d){}

//    Commented out: it  seems unuseful and make gcc complain. p.
			void SetMesh(MeshType *m)
			{this->tm.SetMesh(m);}

		};

	}	 // end namespace tri
}	 // end namespace vcg

#endif
