/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2006                                                \/)\/    *
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
/****************************************************************************
History

$Log: not supported by cvs2svn $
Revision 1.25  2007/03/08 17:05:50  pietroni
line 375,  corrected 1 error concerning intersection with bounding of the grid

Revision 1.24  2007/02/20 16:22:50  ganovelli
modif in ClosestIterator to include  the last shell Si.siz [X|Y|X]. Tested with minialign and point based animation

Revision 1.23  2006/12/06 12:53:14  pietroni
changed 1 wrong comment RayIterator---- Refresh .. was the opposite

Revision 1.22  2006/10/26 08:28:50  pietroni
corrected 1 bug in operator ++ of closest iterator

Revision 1.21  2006/10/25 15:59:29  pietroni
corrected bug in closest iterator.. if doesn't find any alement at first cells examinated continue until find some element

Revision 1.20  2006/10/25 09:47:53  pietroni
added max dist control and constructor

Revision 1.19  2006/10/02 07:47:57  cignoni
Reverted to version 1.14 to nullify dangerous marfr960's changes

Revision 1.14  2006/06/01 20:53:56  cignoni
added missing header

****************************************************************************/
#ifndef __VCGLIB_SPATIAL_ITERATORS
#define __VCGLIB_SPATIAL_ITERATORS

#include <vector>
#include <vcg/space/intersection3.h>
#include <vcg/space/point3.h>
#include <vcg/space/box3.h>
#include <vcg/space/ray3.h>
#include <vcg/math/base.h>
#include <algorithm>
#include <float.h>
#include <limits>

namespace vcg{
	template <class Spatial_Idexing,class INTFUNCTOR,class TMARKER>
	class RayIterator
	{
	public:
		typedef typename Spatial_Idexing::ScalarType ScalarType;
		typedef typename vcg::Ray3<ScalarType> RayType;
		typedef typename Spatial_Idexing::Box3x IndexingBoxType;
	protected:
		typedef typename Spatial_Idexing::ObjType ObjType;
		typedef typename vcg::Point3<ScalarType>  CoordType;
		typedef typename Spatial_Idexing::CellIterator CellIterator;
		ScalarType max_dist;

		///control right bonding current cell index (only on initialization)
		void _ControlLimits()
		{
			for (int i=0;i<3;i++)
			{
				vcg::Point3i dim=Si.siz;
				if (CurrentCell.V(i)<0)
					CurrentCell.V(i) = 0;
				else
					if (CurrentCell.V(i)>=dim.V(i))
						CurrentCell.V(i)=dim.V(i)-1;
			}
		}

		///find initial line parameters
		void _FindLinePar()
		{
			/* Punti goal */

			///da verificare se vanno oltre ai limiti
			vcg::Point3i ip;
			Si.PToIP(start,ip);
			Si.IPiToPf(ip,goal);
			for (int i=0;i<3;i++)
				if(r.Direction().V(i)>0.0)
					goal.V(i)+=Si.voxel.V(i);

			ScalarType gx=goal.X();
			ScalarType gy=goal.Y();
			ScalarType gz=goal.Z();

			dist=(r.Origin()-goal).Norm();

      const float LocalMaxScalar = (std::numeric_limits<float>::max)();
                        const float	EPS = std::numeric_limits<float>::min();

			/* Parametri della linea */
			ScalarType tx,ty,tz;

                        if(	fabs(r.Direction().X())>EPS	)
				tx = (gx-r.Origin().X())/r.Direction().X();
			else
				tx	=LocalMaxScalar;

                        if(	fabs(r.Direction().Y())>EPS)
				ty = (gy-r.Origin().Y())/r.Direction().Y();
			else
				ty	=LocalMaxScalar;

                        if(	fabs(r.Direction().Z())>EPS	)
				tz = (gz-r.Origin().Z())/r.Direction().Z();
			else
				tz	=LocalMaxScalar;

			t=CoordType(tx,ty,tz);
		}

		bool _controlEnd()
		{
			return  (((CurrentCell.X()<0)||(CurrentCell.Y()<0)||(CurrentCell.Z()<0))||
				((CurrentCell.X()>=Si.siz.X())||(CurrentCell.Y()>=Si.siz.Y())||(CurrentCell.Z()>=Si.siz.Z())));
		}

		void _NextCell()
		{
			assert(!end);
			vcg::Box3<ScalarType> bb_current;

			Si.IPiToPf(CurrentCell,bb_current.min);
			Si.IPiToPf(CurrentCell+vcg::Point3i(1,1,1),bb_current.max);

			CoordType inters;
			IntersectionRayBox(bb_current,r,inters);
			ScalarType testmax_dist=(inters-r.Origin()).Norm();

			if (testmax_dist>max_dist)
				end=true;
			else
			{
			if( t.X()<t.Y() && t.X()<t.Z() )
			{
				if(r.Direction().X()<0.0)
				{goal.X() -= Si.voxel.X(); --CurrentCell.X();}
				else
				{goal.X() += Si.voxel.X(); ++CurrentCell.X();}
				t.X() = (goal.X()-r.Origin().X())/r.Direction().X();
			}
			else if( t.Y()<t.Z() ){
				if(r.Direction().Y()<0.0)
				{goal.Y() -= Si.voxel.Y(); --CurrentCell.Y();}
				else
				{goal.Y() += Si.voxel.Y(); ++CurrentCell.Y();}
				t.Y() = (goal.Y()-r.Origin().Y())/r.Direction().Y();
			} else {
				if(r.Direction().Z()<0.0)
				{ goal.Z() -= Si.voxel.Z(); --CurrentCell.Z();}
				else
				{ goal.Z() += Si.voxel.Z(); ++CurrentCell.Z();}
				t.Z() = (goal.Z()-r.Origin().Z())/r.Direction().Z();
			}

			dist=(r.Origin()-goal).Norm();
			end=_controlEnd();
		}
		}

	public:


		///contructor
		RayIterator(Spatial_Idexing &_Si,
					INTFUNCTOR _int_funct
					,const ScalarType &_max_dist)
					:Si(_Si),int_funct(_int_funct)
		{
			max_dist=_max_dist;
		};

		void SetMarker(TMARKER  _tm)
		{
			tm=_tm;
		}

		void Init(const RayType _r)
		{
			r=_r;
			end=false;
			tm.UnMarkAll();
			Elems.clear();
			//CoordType ip;
			//control if intersect the bounding box of the mesh
			if (Si.bbox.IsIn(r.Origin()))
				start=r.Origin();
			else
      if (!(vcg::IntersectionRayBox<ScalarType>(Si.bbox,r,start))){
				end=true;
				return;
			}
				Si.PToIP(start,CurrentCell);
				_ControlLimits();
				_FindLinePar();
				//go to first intersection
				while ((!End())&& Refresh())
					_NextCell();

		}

		bool End()
		{return end;}


		///refresh current cell intersection , return false if there are
		///at lest 1 intersection
		bool Refresh()
		{
			//Elems.clear();

      typename Spatial_Idexing::CellIterator first,last,l;

			///take first, last iterators to elements in the cell
			Si.Grid(CurrentCell.X(),CurrentCell.Y(),CurrentCell.Z(),first,last);
			for(l=first;l!=last;++l)
			{
				ObjType* elem=&(*(*l));
				ScalarType t;
				CoordType Int;
				if((!elem->IsD())&&(!tm.IsMarked(elem))&&(int_funct((**l),r,t))&&(t<=max_dist))
				{
					Int=r.Origin()+r.Direction()*t;
					Elems.push_back(Entry_Type(elem,t,Int));
					tm.Mark(elem);
				}
			}
			////then control if there are more than 1 element
			std::sort(Elems.begin(),Elems.end());
			CurrentElem=Elems.rbegin();

			return((Elems.size()==0)||(Dist()>dist));
		}

		void operator ++()
		{
			if (!Elems.empty()) Elems.pop_back();

			CurrentElem = Elems.rbegin();

			if (Dist()>dist)
			{
				if (!End())
				{
					_NextCell();
					while ((!End())&&Refresh())
						_NextCell();
				}
			}
		}

		ObjType &operator *(){return *((*CurrentElem).elem);}

		CoordType IntPoint()
		{return ((*CurrentElem).intersection);}

		ScalarType Dist()
		{
			if (Elems.size()>0)
				return ((*CurrentElem).dist);
			else
				return ((ScalarType)FLT_MAX);
		}

		///set the current spatial indexing structure used
		void SetIndexStructure(Spatial_Idexing &_Si)
		{Si=_Si;}



	protected:

		///structure that mantain for the current cell pre-calculated data
		struct Entry_Type
		{
		public:

			Entry_Type(ObjType* _elem,ScalarType _dist,CoordType _intersection)
			{
				elem=_elem;
				dist=_dist;
				intersection=_intersection;
			}
			inline bool operator <  ( const Entry_Type & l ) const{return (dist > l.dist); }
			ObjType* elem;
			ScalarType dist;
			CoordType intersection;
		};

		RayType r;							//ray to find intersections
		Spatial_Idexing &Si;	  //reference to spatial index algorithm
		bool end;								//true if the scan is terminated
		INTFUNCTOR &int_funct;
		TMARKER tm;

		std::vector<Entry_Type> Elems;					//element loaded from curren cell
		typedef typename std::vector<Entry_Type>::reverse_iterator ElemIterator;
		ElemIterator CurrentElem;	//iterator to current element

		vcg::Point3i CurrentCell;						//current cell

		//used for raterization
		CoordType start;
		CoordType goal;
		ScalarType dist;
		CoordType t;

	};


	template <class Spatial_Idexing,class DISTFUNCTOR,class TMARKER>
	class ClosestIterator
	{
		typedef typename Spatial_Idexing::ObjType ObjType;
		typedef typename Spatial_Idexing::ScalarType ScalarType;
		typedef typename vcg::Point3<ScalarType>  CoordType;
		typedef typename Spatial_Idexing::CellIterator CellIterator;



		///control the end of scanning
		bool  _EndGrid()
		{
			if ((explored.min==vcg::Point3i(0,0,0))&&(explored.max==Si.siz))
				end =true;
			return end;
		}

		void _UpdateRadius()
		{
			if (radius>=max_dist)
				end=true;

			radius+=step_size;
			//control bounds
			if (radius>max_dist)
				radius=max_dist;
		}

		///add cell to the curren set of explored cells
		bool _NextShell()
		{

			//then expand the box
			explored=to_explore;
			_UpdateRadius();
			Box3<ScalarType> b3d(p,radius);
			Si.BoxToIBox(b3d,to_explore);
			Box3i ibox(Point3i(0,0,0),Si.siz-Point3i(1,1,1));
			to_explore.Intersect(ibox);
			if (!to_explore.IsNull())
			{
				assert(!( to_explore.min.X()<0 || to_explore.max.X()>=Si.siz[0] ||
					to_explore.min.Y()<0 || to_explore.max.Y()>=Si.siz[1] ||  to_explore.min.Z()<0
					|| to_explore.max.Z()>=Si.siz[2] ));
				return true;
			}
			return false;
		}



	public:

		///contructor
		ClosestIterator(Spatial_Idexing &_Si,DISTFUNCTOR _dist_funct):Si(_Si),dist_funct(_dist_funct){}

		///set the current spatial indexing structure used
		void SetIndexStructure(Spatial_Idexing &_Si)
		{Si=_Si;}

		void SetMarker(TMARKER _tm)
		{
			tm=_tm;
		}

		///initialize the Iterator
		void Init(CoordType _p,const ScalarType &_max_dist)
		{
			explored.SetNull();
			to_explore.SetNull();
			p=_p;
			max_dist=_max_dist;
			Elems.clear();
			end=false;
			tm.UnMarkAll();
			//step_size=Si.voxel.X();
			step_size=Si.voxel.Norm();
			radius=0;

			///inflate the bbox until find a valid bbox
			while ((!_NextShell())&&(!End())) {}

			while ((!End())&& Refresh()&&(!_EndGrid()))
					_NextShell();
		}

		//return true if the scan is complete
		bool End()
		{return end;}

		///refresh Object found	also considering current shere radius,
		//and object comes from	previos	that are already in	the	stack
		//return false if no elements find
		bool Refresh()
		{
			int	ix,iy,iz;
			for( iz = to_explore.min.Z();iz <=	to_explore.max.Z(); ++iz)
				for(iy =to_explore.min.Y(); iy	<=to_explore.max.Y(); ++iy)
					for(ix =to_explore.min.X(); ix	<= to_explore.max.X();++ix)
					{
						// this test is to avoid to re-process already analyzed cells.
						if((explored.IsNull())||
							(ix<explored.min[0] || ix>explored.max[0] ||
							iy<explored.min[1] || iy>explored.max[1] ||
							iz<explored.min[2] || iz>explored.max[2] ))
						{
							typename Spatial_Idexing::CellIterator first,last,l;

							Si.Grid(ix,iy,iz,first,last);
							for(l=first;l!=last;++l)
							{
								ObjType	*elem=&(**l);
								if (!tm.IsMarked(elem))
								{

									CoordType nearest;
									ScalarType dist=max_dist;
									if (dist_funct((**l),p,dist,nearest))
										Elems.push_back(Entry_Type(elem,fabs(dist),nearest));
									tm.Mark(elem);
								}
							}
						}

					}

				////sort the elements in Elems and take a iterator to the last one
				std::sort(Elems.begin(),Elems.end());
				CurrentElem=Elems.rbegin();

			return((Elems.size()==0)||(Dist()>radius));
		}

		bool ToUpdate()
		{return ((Elems.size()==0)||(Dist()>radius));}

		void operator ++()
		{
			if (!Elems.empty()) Elems.pop_back();

			CurrentElem = Elems.rbegin();

			if ((!End())&& ToUpdate())
				do{_NextShell();}
					while (Refresh()&&(!_EndGrid()));
		}

		ObjType &operator *(){return *((*CurrentElem).elem);}

		//return distance of the element form the point if no element
		//are in the vector then return max dinstance
		ScalarType Dist()
		{
			if (Elems.size()>0)
				return ((*CurrentElem).dist);
			else
				return ((ScalarType)FLT_MAX);
		}

		CoordType NearestPoint()
		{return ((*CurrentElem).intersection);}

	protected:

		///structure that mantain for the current cell pre-calculated data
		struct Entry_Type
		{
		public:

			Entry_Type(ObjType* _elem,ScalarType _dist,CoordType _intersection)
			{
				elem=_elem;
				dist=_dist;
				intersection=_intersection;
			}

			inline bool operator <  ( const Entry_Type & l ) const{return (dist > l.dist); }

			inline bool operator ==  ( const Entry_Type & l ) const{return (elem == l.elem); }

			ObjType* elem;
			ScalarType dist;
			CoordType intersection;
		};

		CoordType p;							//initial point
		Spatial_Idexing &Si;		  //reference to spatial index algorithm
		bool end;									//true if the scan is terminated
		ScalarType max_dist;		  //max distance when the scan terminate
		vcg::Box3i explored;		  //current bounding box explored
		vcg::Box3i to_explore;		//current bounding box explored
		ScalarType radius;			  //curret radius for sphere expansion
		ScalarType step_size;		  //radius step
		std::vector<Entry_Type> Elems; //element loaded from the current sphere

		DISTFUNCTOR dist_funct;
		TMARKER tm;

		typedef typename std::vector<Entry_Type>::reverse_iterator ElemIterator;
		ElemIterator CurrentElem;	//iterator to current element

};
}

#endif
