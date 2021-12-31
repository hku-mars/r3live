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
****************************************************************************/


#ifndef __VCG_JUMPING_FACE_POS
#define __VCG_JUMPING_FACE_POS

#include <assert.h>
#include <vcg/simplex/face/pos.h>

namespace vcg 
{
	namespace face 
	{
		/** \addtogroup face */
		/*@{*/

		template < class FACE_TYPE >
		class JumpingPos : public Pos< FACE_TYPE >
		{
		public: // Typedefs
			typedef						FACE_TYPE								FaceType;
			typedef						Pos<FaceType>						PosType;
			typedef						JumpingPos<FaceType>		JumpingPosType;
			typedef typename	FaceType::VertexType		VertexType;
			typedef typename	VertexType::CoordType		CoordType;
			typedef typename	VertexType::ScalarType	ScalarType;
			using Pos<FACE_TYPE>::f;
			using Pos<FACE_TYPE>::z;
			using Pos<FACE_TYPE>::FFlip;
		public:
			// Constructors
			JumpingPos()																																: Pos<FACE_TYPE>()   								{ }
			JumpingPos(FaceType * const pFace, int const z, VertexType * const pVertex) : Pos<FACE_TYPE>(pFace, z, pVertex)	{   }
			JumpingPos(FaceType * const pFace, int const z)															: Pos<FACE_TYPE>(pFace, z)						{  }
			JumpingPos(FaceType * const pFace, VertexType * const pVertex)							: Pos<FACE_TYPE>(pFace, pVertex)			{  }

      /// Move the pos to a border (if exists). Return true if actually succeed to find a border
      bool FindBorder()
			{
				PosType startPos=*this;
				do
				{
					if(f==FFlip() ) {
					PosType::FlipE();
					return true; // we are on a border
					}
					PosType::FlipF();
					PosType::FlipE();
				} while(*this != startPos);
				
				return false;
			}
			
			/*!
			* Returns the next edge skipping the border
			*      _________
			*     /\ c | b /\
			*    /  \  |  /  \
			*   / d  \ | / a  \
			*  /______\|/______\
			*          v
			* In this example, if a and d are of-border and the pos is iterating counterclockwise, this method iterate through the faces incident on vertex v,
			* producing the sequence a, b, c, d, a, b, c, ... 
			*/
			bool NextFE()
			{
				if ( f==FFlip() ) // we are on a border
				{
					do {
					PosType::FlipF();
					PosType::FlipE();
					} while (f!=FFlip());
					PosType::FlipE();
					return false;
				}
				else
				{
					PosType::FlipF();
					PosType::FlipE();
					return true;
				}
			}
		};

		/*@}*/
	} // end of namespace face
} // end of namespace vcg

#endif // __VCG_JUMPING_FACE_POS
