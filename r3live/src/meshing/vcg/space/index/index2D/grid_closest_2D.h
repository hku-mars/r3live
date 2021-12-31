/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2005                                                \/)\/    *
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

#ifndef __VCGLIB_GRID_CLOSEST
#define __VCGLIB_GRID_CLOSEST

#include <vcg/space/index/space_iterators.h>

namespace vcg{


		template <class SPATIALINDEXING,class OBJMARKER, class OBJPTRCONTAINER>
			unsigned int GridGetInBox2D(SPATIALINDEXING &_Si,
			OBJMARKER & _marker, 
			const vcg::Box2<typename SPATIALINDEXING::ScalarType> &_bbox,
            OBJPTRCONTAINER & _objectPtrs,
            bool update_global_mark=true)
		{
			typename SPATIALINDEXING::CellIterator first,last,l;
            //_objectPtrs.clear();
			vcg::Box2i ibbox;
			Box2i Si_ibox(Point2i(0,0),_Si.siz-Point2i(1,1));
			_Si.BoxToIBox(_bbox, ibbox);
			ibbox.Intersect(Si_ibox);

            if (update_global_mark)
                _marker.UnMarkAll();

			if (ibbox.IsNull())
				return 0;
			else
			{
                int ix,iy;
				for (ix=ibbox.min[0]; ix<=ibbox.max[0]; ix++) 
					for (iy=ibbox.min[1]; iy<=ibbox.max[1]; iy++) 
						{
                            _Si.Grid( ix, iy, first, last );
							for(l=first;l!=last;++l) 
								if (!(**l).IsD())
								{
									typename SPATIALINDEXING::ObjPtr elem=&(**l);
									vcg::Box2<typename SPATIALINDEXING::ScalarType> box_elem;
									elem->GetBBox(box_elem);
									if(( ! _marker.IsMarked(elem))&&(box_elem.Collide(_bbox))){
										_objectPtrs.push_back(elem);
										_marker.Mark(elem);
									}
								}
						}
						return (static_cast<unsigned int>(_objectPtrs.size()));
			}
		}

        /*template <class SPATIALINDEXING,class OBJMARKER, class OBJPTRCONTAINER>
            unsigned int GridGetInBoxes2D(SPATIALINDEXING &_Si,
            OBJMARKER & _marker,
            const std::vector<vcg::Box2<typename SPATIALINDEXING::ScalarType> > &_bbox,
            OBJPTRCONTAINER & _objectPtrs)
        {
            typename SPATIALINDEXING::CellIterator first,last,l;
            _objectPtrs.clear();
            _marker.UnMarkAll();
            for (int i=0;i<_bbox.size();i++)
                GridGetInBox2D(_Si,_marker,_bbox[i],_objectPtrs,false);
            return (static_cast<unsigned int>(_objectPtrs.size()));
        }*/

        template <class SPATIALINDEXING,class OBJMARKER, class OBJPTRCONTAINER>
            unsigned int GridGetInBoxes2D(SPATIALINDEXING &_Si,
            OBJMARKER & _marker,
            const std::vector<vcg::Box2<typename SPATIALINDEXING::ScalarType> > &_bbox,
            OBJPTRCONTAINER & _objectPtrs)
        {
            typename SPATIALINDEXING::CellIterator first,last,l;
            _objectPtrs.clear();
            _marker.UnMarkAll();
            std::vector<vcg::Point2i> cells;

            for (int i=0;i<_bbox.size();i++)
            {
                vcg::Box2i ibbox;
                Box2i Si_ibox(Point2i(0,0),_Si.siz-Point2i(1,1));
                _Si.BoxToIBox(_bbox[i], ibbox);
                ibbox.Intersect(Si_ibox);

                if (ibbox.IsNull())continue;
                int ix,iy;

                 for (ix=ibbox.min[0]; ix<=ibbox.max[0]; ix++)
                     for (iy=ibbox.min[1]; iy<=ibbox.max[1]; iy++)
                         cells.push_back(vcg::Point2i(ix,iy));
            }
            //printf("%d \n",cells.size());
            std::sort(cells.begin(),cells.end());
            std::vector<vcg::Point2i>::iterator it=std::unique(cells.begin(),cells.end());
            cells.resize( it - cells.begin() );

            for (int i=0;i<cells.size();i++)
            {
                _Si.Grid( cells[i].X(), cells[i].Y(), first, last );
                for(l=first;l!=last;++l)
                if (!(**l).IsD())
                {
                    typename SPATIALINDEXING::ObjPtr elem=&(**l);
                    if ( ! _marker.IsMarked(elem))
                    {
                        _objectPtrs.push_back(elem);
                        _marker.Mark(elem);
                    }
                }
            }
           return (static_cast<unsigned int>(_objectPtrs.size()));
        }

	}//end namespace vcg
#endif

