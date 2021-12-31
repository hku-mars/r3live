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
/*! \file refine_loop.h
 * 
 * \brief This file contain code for Loop's subdivision scheme for triangular
 * mesh and some similar method.
 * 
 */

#ifndef __VCGLIB_REFINE_LOOP
#define __VCGLIB_REFINE_LOOP

#include <cmath>
#include <vcg/space/point3.h>
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/refine.h>
#include <vcg/space/color4.h>
#include <vcg/container/simple_temporary_data.h>
#include <vcg/complex/algorithms/update/flag.h>
#include <vcg/complex/algorithms/update/color.h>


namespace vcg{
namespace tri{

/*
Metodo di Loop dalla documentazione "Siggraph 2000 course on subdivision"

		d4------d3							d4------d3
	 /	\		 / 	\						 /	\		 / 	\							u
	/		 \  /  	 \					/		e4--e3 	 \					 / \
 /	   	\/	 		\				 /	 / 	\/	\		\					/		\
d5------d1------d2	->	d5--e5--d1--e2--d2			 l--M--r
 \	   	/\	 		/				 \	 \ 	/\	/		/				  \	  /
	\		 /  \ 	 /					\		e6--e7	 /					 \ /
	 \	/		 \ 	/						 \	/		 \ 	/							d
		d6------d7							d6------d7

*******************************************************

*/

/*!
 * \brief Weight class for classical Loop's scheme.
 * 
 * See Zorin, D. & Schröeder, P.: Subdivision for modeling and animation. Proc. ACM SIGGRAPH [Courses], 2000
 */
template<class SCALAR_TYPE>
struct LoopWeight {
	inline SCALAR_TYPE beta(int k) {
		return (k>3)?(5.0/8.0 - std::pow((3.0/8.0 + std::cos(2.0*M_PI/SCALAR_TYPE(k))/4.0),2))/SCALAR_TYPE(k):3.0/16.0;
	}
	
	inline SCALAR_TYPE incidentRegular(int) {
		return 3.0/8.0;
	}
	inline SCALAR_TYPE incidentIrregular(int) {
		return 3.0/8.0;
	}
	inline SCALAR_TYPE opposite(int) {
		return 1.0/8.0;
	}
};

/*!
 * \brief Modified Loop's weight to optimise continuity.
 * 
 * See Barthe, L. & Kobbelt, L.: Subdivision scheme tuning around extraordinary vertices. Computer Aided Geometric Design, 2004, 21, 561-583
 */
template<class SCALAR_TYPE>
struct RegularLoopWeight {
	inline SCALAR_TYPE beta(int k) {
		static SCALAR_TYPE bkPolar[] = {
				.32517,
				.49954,
				.59549,
				.625,
				.63873,
				.64643,
				.65127,
				.67358,
				.68678,
				.69908
		};

		return (k<=12)?(1.0-bkPolar[k-3])/k:LoopWeight<SCALAR_TYPE>().beta(k);
	}
	
	inline SCALAR_TYPE incidentRegular(int k) {
		return 1.0 - incidentIrregular(k) - opposite(k)*2;
	}
	inline SCALAR_TYPE incidentIrregular(int k) {
		static SCALAR_TYPE bkPolar[] = {
				.15658,
				.25029,
				.34547,
				.375,
				.38877,
				.39644,
				.40132,
				.42198,
				.43423,
				.44579
		};

		return (k<=12)?bkPolar[k-3]:LoopWeight<SCALAR_TYPE>().incidentIrregular(k);
	}
	inline SCALAR_TYPE opposite(int k) {
		static SCALAR_TYPE bkPolar[] = {
				.14427,
				.12524,
				.11182,
				.125,
				.14771,
				.1768,
				.21092,
				.20354,
				.20505,
				.19828
		};

		return (k<=12)?bkPolar[k-3]:LoopWeight<SCALAR_TYPE>().opposite(k);
	}
};

template<class SCALAR_TYPE>
struct ContinuityLoopWeight {
	inline SCALAR_TYPE beta(int k) {
		static SCALAR_TYPE bkPolar[] = {
				.32517,
				.50033,
				.59464,
				.625,
				.63903,
				.67821,
				.6866,
				.69248,
				.69678,
				.70014
		};

		return (k<=12)?(1.0-bkPolar[k-3])/k:LoopWeight<SCALAR_TYPE>().beta(k);
	}
	
	inline SCALAR_TYPE incidentRegular(int k) {
		return 1.0 - incidentIrregular(k) - opposite(k)*2;
	}
	inline SCALAR_TYPE incidentIrregular(int k) {
		static SCALAR_TYPE bkPolar[] = {
				.15658,
				.26721,
				.33539,
				.375,
				.36909,
				.25579,
				.2521,
				.24926,
				.24706,
				.2452
		};

		return (k<=12)?bkPolar[k-3]:LoopWeight<SCALAR_TYPE>().incidentIrregular(k);
	}
	inline SCALAR_TYPE opposite(int k) {
		static SCALAR_TYPE bkPolar[] = {
				.14427,
				.12495,
				.11252,
				.125,
				.14673,
				.16074,
				.18939,
				.2222,
				.25894,
				.29934
		};

		return (k<=12)?bkPolar[k-3]:LoopWeight<SCALAR_TYPE>().opposite(k);
	}
};

// Centroid and LS3Projection classes may be pettre placed in an other file. (which one ?)

/*!
 * \brief Allow to compute classical Loop subdivision surface with the same code than LS3.
 */
template<class MESH_TYPE, class LSCALAR_TYPE = typename MESH_TYPE::ScalarType>
struct Centroid {
	typedef typename MESH_TYPE::ScalarType Scalar;
	typedef typename MESH_TYPE::CoordType Coord;
	typedef LSCALAR_TYPE LScalar;
	typedef vcg::Point3<LScalar> LVector;

	LVector sumP;
	LScalar sumW;

	Centroid() { reset(); }
	inline void reset() {
		sumP.SetZero();
		sumW = 0.;
	}
	inline void addVertex(const typename MESH_TYPE::VertexType &v, LScalar w) {
		LVector p(v.cP().X(), v.cP().Y(), v.cP().Z());
		sumP += p * w;
		sumW += w;
	}
	inline void project(std::pair<Point3f,Point3f> &nv) const {
		LVector position = sumP / sumW;
		nv.first = Coord(position.X(), position.Y(), position.Z());
	}
};

/*! Implementation of the APSS projection for the LS3 scheme.
 * 
 * See Gael Guennebaud and Marcel Germann and Markus Gross
 * 		Dynamic sampling and rendering of algebraic point set surfaces.
 * 		Computer Graphics Forum (Proceedings of Eurographics 2008), 2008, 27, 653-662
 * and Simon Boye and Gael Guennebaud and Christophe Schlick
 * 		Least squares subdivision surfaces
 * 		Computer Graphics Forum, 2010
 */
template<class MESH_TYPE, class LSCALAR_TYPE = typename MESH_TYPE::ScalarType>
struct LS3Projection {
	typedef typename MESH_TYPE::ScalarType Scalar;
	typedef typename MESH_TYPE::CoordType Coord;
	typedef LSCALAR_TYPE LScalar;
	typedef vcg::Point3<LScalar> LVector;

	Scalar beta;

	LVector sumP;
	LVector sumN;
	LScalar sumDotPN;
	LScalar sumDotPP;
	LScalar sumW;
	
	inline LS3Projection(Scalar beta = 1.0) : beta(beta) { reset(); }
	inline void reset() {
		sumP.SetZero();
		sumN.SetZero();
		sumDotPN = 0.;
		sumDotPP = 0.;
		sumW = 0.;
	}
	inline void addVertex(const typename MESH_TYPE::VertexType &v, LScalar w) {
		LVector p(v.cP().X(), v.cP().Y(), v.cP().Z());
		LVector n(v.cN().X(), v.cN().Y(), v.cN().Z());

		sumP += p * w;
		sumN += n * w;
		sumDotPN += w * n.dot(p);
		sumDotPP += w * vcg::SquaredNorm(p);
		sumW += w;
	}
	void project(std::pair<Point3f,Point3f>  &nv) const {
		LScalar invSumW = Scalar(1)/sumW;
		LScalar aux4 = beta * LScalar(0.5) *
									(sumDotPN - invSumW*sumP.dot(sumN))
									/(sumDotPP - invSumW*vcg::SquaredNorm(sumP));
		LVector uLinear = (sumN-sumP*(Scalar(2)*aux4))*invSumW;
		LScalar uConstant = -invSumW*(uLinear.dot(sumP) + sumDotPP*aux4);
		LScalar uQuad = aux4;
		LVector orig = sumP*invSumW;

		// finalize
		LVector position;
		LVector normal;
		if (fabs(uQuad)>1e-7)
		{
			LScalar b = 1./uQuad;
			LVector center = uLinear*(-0.5*b);
			LScalar radius = sqrt( vcg::SquaredNorm(center) - b*uConstant );
			
			normal = orig - center;
			normal.Normalize();
			position = center + normal * radius;

			normal = uLinear + position * (LScalar(2) * uQuad);
			normal.Normalize();
		}
		else if (uQuad==0.)
		{
			LScalar s = LScalar(1)/vcg::Norm(uLinear);
			assert(!vcg::math::IsNAN(s) && "normal should not have zero len!");
			uLinear *= s;
			uConstant *= s;
			
			normal = uLinear;
			position = orig - uLinear * (orig.dot(uLinear) + uConstant);
		}
		else
		{
			// normalize the gradient
			LScalar f = 1./sqrt(vcg::SquaredNorm(uLinear) - Scalar(4)*uConstant*uQuad);
			uConstant *= f;
			uLinear *= f;
			uQuad *= f;
			
			// Newton iterations
			LVector grad;
			LVector dir = uLinear+orig*(2.*uQuad);
			LScalar ilg = 1./vcg::Norm(dir);
			dir *= ilg;
			LScalar ad = uConstant + uLinear.dot(orig) + uQuad * vcg::SquaredNorm(orig);
			LScalar delta = -ad*std::min<Scalar>(ilg,1.);
			LVector p = orig + dir*delta;
			for (int i=0 ; i<2 ; ++i)
			{
				grad = uLinear+p*(2.*uQuad);
				ilg = 1./vcg::Norm(grad);
				delta = -(uConstant + uLinear.dot(p) + uQuad * vcg::SquaredNorm(p))*std::min<Scalar>(ilg,1.);
				p += dir*delta;
			}
			position = p;

			normal = uLinear + position * (Scalar(2) * uQuad);
			normal.Normalize();
		}

		nv.first = Coord(position.X(), position.Y(), position.Z());
		nv.second = Coord(normal.X(), normal.Y(), normal.Z());
	}
};

template<class MESH_TYPE, class METHOD_TYPE=Centroid<MESH_TYPE>, class WEIGHT_TYPE=LoopWeight<typename MESH_TYPE::ScalarType> >
struct OddPointLoopGeneric : public std::unary_function<face::Pos<typename MESH_TYPE::FaceType> , typename MESH_TYPE::VertexType>
{
  typedef METHOD_TYPE Projection;
	typedef WEIGHT_TYPE Weight;
	typedef typename MESH_TYPE::template PerVertexAttributeHandle<int> ValenceAttr;
	
	MESH_TYPE &m;
	Projection proj;
	Weight weight;
	ValenceAttr *valence;
	
	inline OddPointLoopGeneric(MESH_TYPE &_m, Projection proj = Projection(), Weight weight = Weight()) :
		m(_m), proj(proj), weight(weight), valence(0) {}
	
	void operator()(typename MESH_TYPE::VertexType &nv, face::Pos<typename MESH_TYPE::FaceType>  ep)	{
		proj.reset();
		
		face::Pos<typename MESH_TYPE::FaceType> he(ep.f,ep.z,ep.f->V(ep.z));
		typename MESH_TYPE::VertexType *l,*r,*u,*d;
		l = he.v;
		he.FlipV();
		r = he.v;

		if( tri::HasPerVertexColor(m))
			nv.C().lerp(ep.f->V(ep.z)->C(),ep.f->V1(ep.z)->C(),.5f);

		if (he.IsBorder()) {
			proj.addVertex(*l, 0.5);
			proj.addVertex(*r, 0.5);
			std::pair<Point3f,Point3f>pp;
			proj.project(pp);
			nv.P()=pp.first;
			nv.N()=pp.second;
		}
		else {
			he.FlipE();	he.FlipV();
			u = he.v;
			he.FlipV();	he.FlipE();
			assert(he.v == r); // back to r
			he.FlipF();	he.FlipE();	he.FlipV();
			d = he.v;

			if(valence && ((*valence)[l]==6 || (*valence)[r]==6)) {
				int extra = ((*valence)[l]==6)?(*valence)[r]:(*valence)[l];
				proj.addVertex(*l, ((*valence)[l]==6)?weight.incidentRegular(extra):weight.incidentIrregular(extra));
				proj.addVertex(*r, ((*valence)[r]==6)?weight.incidentRegular(extra):weight.incidentIrregular(extra));
				proj.addVertex(*u, weight.opposite(extra));
				proj.addVertex(*d, weight.opposite(extra));
			}
			// unhandled case that append only at first subdivision step: use Loop's weights
			else {
				proj.addVertex(*l, 3.0/8.0);
				proj.addVertex(*r, 3.0/8.0);
				proj.addVertex(*u, 1.0/8.0);
				proj.addVertex(*d, 1.0/8.0);
			}
			std::pair<Point3f,Point3f>pp;
			proj.project(pp);
			nv.P()=pp.first;
			nv.N()=pp.second;
//			proj.project(nv);
		}

	}

	Color4<typename MESH_TYPE::ScalarType> WedgeInterp(Color4<typename MESH_TYPE::ScalarType> &c0, Color4<typename MESH_TYPE::ScalarType> &c1)
	{
		Color4<typename MESH_TYPE::ScalarType> cc;
		return cc.lerp(c0,c1,0.5f);
	}

	template<class FL_TYPE>
	TexCoord2<FL_TYPE,1> WedgeInterp(TexCoord2<FL_TYPE,1> &t0, TexCoord2<FL_TYPE,1> &t1)
	{
		TexCoord2<FL_TYPE,1> tmp;
		tmp.n()=t0.n();
		tmp.t()=(t0.t()+t1.t())/2.0;
		return tmp;
	}

	inline void setValenceAttr(ValenceAttr *valence) {
		this->valence = valence;
	}
};

template<class MESH_TYPE, class METHOD_TYPE=Centroid<MESH_TYPE>, class WEIGHT_TYPE=LoopWeight<typename MESH_TYPE::ScalarType> >
struct EvenPointLoopGeneric : public std::unary_function<face::Pos<typename MESH_TYPE::FaceType> , typename MESH_TYPE::VertexType>
{
	typedef METHOD_TYPE Projection;
	typedef WEIGHT_TYPE Weight;
	typedef typename MESH_TYPE::template PerVertexAttributeHandle<int> ValenceAttr;

	Projection proj;
	Weight weight;
	ValenceAttr *valence;
	
	inline EvenPointLoopGeneric(Projection proj = Projection(), Weight weight = Weight()) :
		proj(proj), weight(weight), valence(0) {}
	
	void operator()(std::pair<Point3f,Point3f> &nv, face::Pos<typename MESH_TYPE::FaceType>  ep)	{
		proj.reset();
		
		face::Pos<typename MESH_TYPE::FaceType> he(ep.f,ep.z,ep.f->V(ep.z));
		typename MESH_TYPE::VertexType *r, *l,  *curr;
		curr = he.v;
		face::Pos<typename MESH_TYPE::FaceType> heStart = he;

		// compute valence of this vertex or find a border
		int k = 0;
		do {
			he.NextE();
			k++;
		}	while(!he.IsBorder() && he != heStart);

		if (he.IsBorder()) {	//	Border rule
			// consider valence of borders as if they are half+1 of an inner vertex. not perfect, but better than nothing.
			if(valence) {
				k = 0;
				do {
					he.NextE();
					k++;
				}	while(!he.IsBorder());
				(*valence)[he.V()] = std::max(2*(k-1), 3);
//				(*valence)[he.V()] = 6;
			}
			
			he.FlipV();
			r = he.v;
			he.FlipV();
			he.NextB();
			l = he.v;
			
			proj.addVertex(*curr, 3.0/4.0);
			proj.addVertex(*l, 1.0/8.0);
			proj.addVertex(*r, 1.0/8.0);
//			std::pair<Point3f,Point3f>pp;
			proj.project(nv);
//			nv.P()=pp.first;
//			nv.N()=pp.second;
//			proj.project(nv);
		}
		else {	//	Inner rule
//			assert(!he.v->IsB()); border flag no longer updated (useless)
			if(valence)
				(*valence)[he.V()] = k;
			
			typename MESH_TYPE::ScalarType beta = weight.beta(k);

			proj.addVertex(*curr, 1.0 - (typename MESH_TYPE::ScalarType)(k) * beta);
			do {
				proj.addVertex(*he.VFlip(), beta);
				he.NextE();
			}	while(he != heStart);
			
			proj.project(nv);
		}
	} // end of operator()

	Color4<typename MESH_TYPE::ScalarType> WedgeInterp(Color4<typename MESH_TYPE::ScalarType> &c0, Color4<typename MESH_TYPE::ScalarType> &c1)
	{
		Color4<typename MESH_TYPE::ScalarType> cc;
		return cc.lerp(c0,c1,0.5f);
	}
	Color4b WedgeInterp(Color4b &c0, Color4b &c1)
	{
		Color4b cc;
		cc.lerp(c0,c1,0.5f);
		return cc;
	}

	template<class FL_TYPE>
	TexCoord2<FL_TYPE,1> WedgeInterp(TexCoord2<FL_TYPE,1> &t0, TexCoord2<FL_TYPE,1> &t1)
	{
		TexCoord2<FL_TYPE,1> tmp;
		// assert(t0.n()== t1.n());
		tmp.n()=t0.n();
		tmp.t()=(t0.t()+t1.t())/2.0;
		return tmp;
	}

	inline void setValenceAttr(ValenceAttr *valence) {
		this->valence = valence;
	}
};

template<class MESH_TYPE>
struct OddPointLoop : OddPointLoopGeneric<MESH_TYPE, Centroid<MESH_TYPE> >
{
  OddPointLoop(MESH_TYPE &_m):OddPointLoopGeneric<MESH_TYPE, Centroid<MESH_TYPE> >(_m){}
};

template<class MESH_TYPE>
struct EvenPointLoop : EvenPointLoopGeneric<MESH_TYPE, Centroid<MESH_TYPE> >
{
};

template<class MESH_TYPE,class ODD_VERT, class EVEN_VERT>
bool RefineOddEven(MESH_TYPE &m, ODD_VERT odd, EVEN_VERT even,float length,
                    bool RefineSelected=false, CallBackPos *cbOdd = 0, CallBackPos *cbEven = 0)
{
  EdgeLen <MESH_TYPE, typename MESH_TYPE::ScalarType> ep(length);
  return RefineOddEvenE(m, odd, even, ep, RefineSelected, cbOdd, cbEven);
}

/*!
 * \brief Perform diadic subdivision using given rules for odd and even vertices.
 */
template<class MESH_TYPE, class ODD_VERT, class EVEN_VERT, class PREDICATE>
bool RefineOddEvenE(MESH_TYPE &m, ODD_VERT odd, EVEN_VERT even, PREDICATE edgePred,
										bool RefineSelected=false, CallBackPos *cbOdd = 0, CallBackPos *cbEven = 0)
{
	typedef typename MESH_TYPE::template PerVertexAttributeHandle<int> ValenceAttr;

	// momentaneamente le callback sono identiche, almeno cbOdd deve essere passata
	cbEven = cbOdd;

	// to mark visited vertices
	int evenFlag = MESH_TYPE::VertexType::NewBitFlag();
	for (int i = 0; i < m.vn ; i++ ) {
		m.vert[i].ClearUserBit(evenFlag);
	}

	int j = 0;
    // di texture per wedge (uno per ogni edge)

  ValenceAttr valence = vcg::tri::Allocator<MESH_TYPE>:: template AddPerVertexAttribute<int>(m);
	odd.setValenceAttr(&valence);
	even.setValenceAttr(&valence);
	
	// store updated vertices
	std::vector<bool> updatedList(m.vn, false);
	//std::vector<typename MESH_TYPE::VertexType> newEven(m.vn);
	std::vector<std::pair<Point3f,Point3f> > newEven(m.vn);

	typename MESH_TYPE::VertexIterator vi;
	typename MESH_TYPE::FaceIterator fi;
	for (fi = m.face.begin(); fi != m.face.end(); fi++) if(!(*fi).IsD() && (!RefineSelected || (*fi).IsS())){ //itero facce
		for (int i = 0; i < 3; i++) { //itero vert
			if ( !(*fi).V(i)->IsUserBit(evenFlag) && ! (*fi).V(i)->IsD() ) {
				(*fi).V(i)->SetUserBit(evenFlag);
				//	use face selection, not vertex selection, to be coherent with RefineE
				//if (RefineSelected && !(*fi).V(i)->IsS() )
				//	break;
				face::Pos<typename MESH_TYPE::FaceType>aux (&(*fi),i);
				if( tri::HasPerVertexColor(m) ) {
					(*fi).V(i)->C().lerp((*fi).V0(i)->C() , (*fi).V1(i)->C(),0.5f);
				}

				if (cbEven) {
					(*cbEven)(int(100.0f * (float)j / (float)m.fn),"Refining");
					j++;
				}
				int index = tri::Index(m, (*fi).V(i));
				updatedList[index] = true;
				even(newEven[index], aux);
			}
		}
	}
	
	MESH_TYPE::VertexType::DeleteBitFlag(evenFlag);

	// Now apply the stored normal and position to the initial vertex set (note that newEven is << m.vert)
	RefineE< MESH_TYPE, ODD_VERT > (m, odd, edgePred, RefineSelected, cbOdd);
	for(size_t i=0;i<newEven.size();++i) {
			if(updatedList[i]) {
			  m.vert[i].P()=newEven[i].first;
			  m.vert[i].N()=newEven[i].second;
		}
	}

	odd.setValenceAttr(0);
	even.setValenceAttr(0);

  vcg::tri::Allocator<MESH_TYPE>::DeletePerVertexAttribute(m, valence);
	
	return true;
}

} // namespace tri
} // namespace vcg




#endif



