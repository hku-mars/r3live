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
#ifndef __VCG_TRI_UPDATE_EDGES
#define __VCG_TRI_UPDATE_EDGES

#include <vcg/space/plane3.h>

namespace vcg {
namespace tri {

/// \ingroup trimesh 

	/// \brief This class is used to compute or update the precomputed data used to efficiently compute point-face distances.
	template <class ComputeMeshType>
	class UpdateComponentEP
	{

	public:
		typedef ComputeMeshType MeshType; 
		typedef typename MeshType::VertexType     VertexType;
		typedef typename MeshType::VertexPointer  VertexPointer;
		typedef typename MeshType::VertexIterator VertexIterator;
		typedef typename MeshType::FaceType       FaceType;
		typedef typename MeshType::FacePointer    FacePointer;
		typedef typename MeshType::FaceIterator   FaceIterator;
	  typedef typename MeshType::FaceType::CoordType::ScalarType     ScalarType;

		static void ComputeEdgePlane(FaceType &f)
		{
			f.Flags() = f.Flags() & (~(FaceType::NORMX|FaceType::NORMY|FaceType::NORMZ));
		
			// Primo calcolo degli edges
			f.Edge(0) = f.V(1)->P(); f.Edge(0) -= f.V(0)->P();
			f.Edge(1) = f.V(2)->P(); f.Edge(1) -= f.V(1)->P();
			f.Edge(2) = f.V(0)->P(); f.Edge(2) -= f.V(2)->P();
			// Calcolo di plane
			f.Plane().SetDirection(f.Edge(0)^f.Edge(1));
			f.Plane().SetOffset(f.Plane().Direction().dot(f.V(0)->P()));
			f.Plane().Normalize();
			// Calcolo migliore proiezione
			ScalarType nx = math::Abs(f.Plane().Direction()[0]);
			ScalarType ny = math::Abs(f.Plane().Direction()[1]);
			ScalarType nz = math::Abs(f.Plane().Direction()[2]);
			ScalarType d;
			if(nx>ny && nx>nz) { f.Flags() |= FaceType::NORMX; d = 1/f.Plane().Direction()[0]; }
			else if(ny>nz)     { f.Flags() |= FaceType::NORMY; d = 1/f.Plane().Direction()[1]; }
			else               { f.Flags() |= FaceType::NORMZ; d = 1/f.Plane().Direction()[2]; }

			// Scalatura spigoli
			f.Edge(0)*=d;
			f.Edge(1)*=d;
			f.Edge(2)*=d;
		}

		static void Set(ComputeMeshType &m)
		{
		  if(!FaceType::HasEdgePlane()) throw vcg::MissingComponentException("PerFaceEdgePlane");
			for(FaceIterator f = m.face.begin(); f!=m.face.end(); ++f)
				if(!(*f).IsD())
					ComputeEdgePlane(*f);
		}

	}; // end class

}	// End namespace
}	// End namespace


#endif
