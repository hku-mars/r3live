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

#ifndef __VCGLIB_OBOX3
#define __VCGLIB_OBOX3

#include <vcg/space/box3.h>

namespace vcg {

	template<class T>
        class Obox3:public Box3<T>{

	public:
		
		Matrix44<T> m;			// a matrix to go in OB space
		Matrix44<T> mi;		  // inverse of m: from OB space to World

		/// The bounding box constructor
		inline  Obox3():Box3<T>(){}
		/// Copy constructor
                inline  Obox3(const Obox3 &b):Box3<T>(b.min,b.max),m(b.m),mi(b.mi){}
		/// Min Max Frame constructor
		inline Obox3(const Point3<T> &min, const Point3<T> &max, const Point3<T> *frame):Box3<T>(min,max){
			T v[16];
			//BaseX:					 BaseY:		 					 BaseZ:								O:
			v[0]=frame[0].X(); v[1] =frame[1].X(); v[2] =frame[2].X();  v[3] =frame[3].X();			
			v[4]=frame[0].Y(); v[5] =frame[1].Y(); v[6] =frame[2].Y();  v[7] =frame[3].Y();	
			v[8]=frame[0].Z(); v[9] =frame[1].Z(); v[10]=frame[2].Z();  v[11]=frame[3].Z();
			v[12]=0.0;				 v[13]=0.0;					 v[14]=0.0;						v[15]=1;			

			mi=Matrix44f(v);
			m =Inverse(mi);
		}
		//Verifica se un punto appartiene ad un oriented bounding box.
		bool IsIn( Point3<T> const &p) const{
			vcg::Point3<T> mod= m*p;
                        return Box3<T>::IsIn(mod);
		}
		/// The bounding box distructor
		inline ~Obox3(){}


	}; // end class definition
	typedef Obox3<short>  Obox3s;
	typedef Obox3<int>	  Obox3i;
	typedef Obox3<float>  Obox3f;
	typedef Obox3<double> Obox3d;

} // end namespace


#endif
