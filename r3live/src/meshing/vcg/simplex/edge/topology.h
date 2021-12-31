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

#ifndef _VCG_EDGE_TOPOLOGY
#define _VCG_EDGE_TOPOLOGY

#include <vector>
#include <algorithm>
#include <vcg/simplex/edge/pos.h>

namespace vcg {
namespace edge {
/** \addtogroup edge */
/*@{*/template <class EdgeType>
inline bool IsEdgeManifold( EdgeType const & e, const int j )
{
  assert(e.cFFp(j) != 0); // never try to use this on uncomputed topology

  if(EdgeType::HasFFAdjacency())
    return ( e.cFFp(j) == &e || &e == e.cFFp(j)->cFFp(e.cFFi(j)) );
  else 
    return true;
}

/** Return a boolean that indicate if the j-th edge of the face is a border.
	@param j Index of the edge
	@return true if j is an edge of border, false otherwise
*/
template <class EdgeType>
inline bool IsEdgeBorder(EdgeType const & e,  const int j )
{
  if(EdgeType::HasEEAdjacency())
    return e.cEEp(j)==&e;

  assert(0);
  return true;
}

template <class EdgeType>
void VVStarVE(typename EdgeType::VertexType* vp, std::vector<typename EdgeType::VertexType *> &starVec)
{
  typedef typename EdgeType::VertexType* VertexPointer;
  starVec.clear();
  edge::VEIterator<EdgeType> vei(vp);
  while(!vei.End())
      {
        starVec.push_back(vei.V1());
        ++vei;
      }
}

template <class EdgeType>
void VEStarVE(typename EdgeType::VertexType* vp, std::vector<EdgeType *> &starVec)
{
  starVec.clear();
  edge::VEIterator<EdgeType> vei(vp);
  while(!vei.End())
      {
        starVec.push_back(vei.E());
        ++vei;
      }
}

} // end namespace edge
} // end namespace vcg

#endif
