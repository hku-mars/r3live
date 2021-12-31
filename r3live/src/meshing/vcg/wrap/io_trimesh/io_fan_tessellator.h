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
#ifndef IO_FAN_TESSELLATOR_H
#define IO_FAN_TESSELLATOR_H

namespace vcg {
namespace tri {
namespace io {

/*
*  A face polygon composed of more than three vertices is triangulated
*  according to the following schema:
*                      v5
*                     /  \
*                    /    \
*                   /      \
*                  v1------v4
*                  |\      /
*                  | \    /
*                  |  \  /
*                 v2---v3
*
*  As shown above, the 5 vertices polygon (v1,v2,v3,v4,v5)
*  has been split into the triangles (v1,v2,v3), (v1,v3,v4) e (v1,v4,v5).
*  This way vertex v1 becomes the common vertex of all newly generated
*  triangles, and this may lead to the creation of very thin triangles.
*
*  This function is intended as a trivial fallback when glutessellator is not available.
*  it assumes just ONE outline
*/
template < class PointType>
void FanTessellator(const std::vector< std::vector<PointType> > & outlines, std::vector<int> & indices)
{
    indices.clear();
    if(outlines.empty()) return;
    const std::vector<PointType> &points=outlines[0];

    for(size_t i=0;i<points.size()-2;++i)
    {
        indices.push_back(0);
        indices.push_back(i+1);
        indices.push_back(i+2);
    }
}

} // end Namespace tri
} // end Namespace io
} // end Namespace vcg


#endif // IO_FAN_TESSELLATOR_H
