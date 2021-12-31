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
/****************************************************************************
  History

$Log: not supported by cvs2svn $
Revision 1.4  2006/09/27 08:49:32  spinelli
bug fix, add return type to Init

Revision 1.3  2006/08/23 15:20:14  marfr960
corrected minor bugs

Revision 1.2  2005/12/02 00:29:00  cignoni
updated the templates of BasicGrid

Revision 1.1  2005/07/28 08:41:00  cignoni
First working version

Revision 1.13  2005/04/14 17:23:08  ponchio
*** empty log message ***

****************************************************************************/

#ifndef __VCGLIB_UGRID_OBJ
#define __VCGLIB_UGRID_OBJ

#include <vector>
#include <algorithm>
#include <stdio.h>

#include <vcg/space/box3.h>
#include <vcg/space/line3.h>
#include <vcg/space/index/grid_util.h>
namespace vcg {
/** Static Uniform Grid
A simple Spatial grid of object. 
Kept in the most trivial way. Every cell is allocated 
and contains one istance of the template class.
*/

template < class ObjType, class FLT=float  >
class GridStaticObj : public BasicGrid<FLT>
{
 public:

	 /// La matriciona della griglia
	 ObjType *grid;

	 int size() const { return this->siz[0]*this->siz[1]*this->siz[2];}

	 inline  GridStaticObj() { grid = 0; }
	 inline ~GridStaticObj() { if(grid) delete[] grid; }
	 inline void Init(const ObjType &val)
	 {
		 fill(grid,grid+size(),val);
	 }


	 /// Date le coordinate ritorna la cella
	 inline ObjType & Grid( const int x, const int y, const int z ) {return grid[GridIndI(Point3i(x,y,z))]; }

	 // Dato un punto ritorna la cella  
	 inline ObjType & Grid( const Point3<FLT> & p )                 {				return grid[GridIndF(p)];		}

	 inline int GridIndI( const Point3i & pi ) const
	 {
#ifndef NDEBUG
		 if ( pi[0]<0 || pi[0]>=this->siz[0] || pi[1]<0 || pi[1]>=this->siz[1] || pi[2]<0 || pi[2]>=this->siz[2] )
		 {	assert(0);
		 return 0;
		 } 
#endif
		 return pi[0]+this->siz[0]*(pi[1]+this->siz[1]*pi[2]);
	 }

	 // Dato un punto ritorna l'indice della cella
   inline int GridIndF( const Point3<FLT> & p ) const { return GridIndI(this->GridP(p)); 	}
  
	void Create( const Point3i &_siz, const ObjType & init )
	{
		this->siz=_siz;
	 	this->voxel[0] = this->dim[0]/this->siz[0];
		this->voxel[1] = this->dim[1]/this->siz[1];
		this->voxel[2] = this->dim[2]/this->siz[2];

		if(grid) delete[] grid;
		int n = this->siz[0]*this->siz[1]*this->siz[2];
		grid = new ObjType[n];
    std::fill(grid,grid+n,init);
	}

	/// Crea una griglia di un dato bbox e con un certo numero di elem.
	/// il bbox viene gonfiato appositamente.

	template<class FLT2>
	void Create(const Box3<FLT2> & b, int ncell, const ObjType & init, bool Inflate = true )
	{
		this->bbox.Import(b);
		if(Inflate) this->bbox.Offset(0.01*this->bbox.Diag());
		this->dim  = this->bbox.max - this->bbox.min;

		// Calcola la dimensione della griglia
		Point3i _siz;
		BestDim( ncell, this->dim, _siz );
		Create(_siz, init );
	}
};
//end class SGrid


}
#endif
