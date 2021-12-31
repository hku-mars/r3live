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

#ifndef __VCG_TRI_UPDATE_NORMALS
#define __VCG_TRI_UPDATE_NORMALS

#include <vcg/complex/algorithms/update/flag.h>
#include <vcg/math/matrix44.h>
#include <vcg/complex/exception.h>

namespace vcg {
namespace tri {

/// \ingroup trimesh

/// \headerfile normal.h vcg/complex/algorithms/update/normal.h

/// \brief Management, updating and computation of per-vertex, per-face, and per-wedge normals.
/**
This class is used to compute or to update the normals that can be stored in the various component of a mesh.
A number of different algorithms for computing per vertex normals are present.

It must be included \b after complex.h
*/

template <class ComputeMeshType>
class   UpdateNormal
{
public:
typedef ComputeMeshType MeshType; 	
typedef typename MeshType::VertexType     VertexType;
typedef typename MeshType::CoordType     CoordType;
typedef typename VertexType::NormalType     NormalType;
typedef typename VertexType::ScalarType ScalarType;
typedef typename MeshType::VertexPointer  VertexPointer;
typedef typename MeshType::VertexIterator VertexIterator;
typedef typename MeshType::FaceType       FaceType;
typedef typename MeshType::FacePointer    FacePointer;
typedef typename MeshType::FaceIterator   FaceIterator;

/// \brief Set to zero all the PerVertex normals
/**
 Set to zero all the PerVertex normals. Used by all the face averaging algorithms.
 by default it does not clear the normals of unreferenced vertices because they could be still useful
 */
static void PerVertexClear(ComputeMeshType &m, bool ClearAllVertNormal=false)
{
  if(!HasPerVertexNormal(m)) throw vcg::MissingComponentException("PerVertexNormal");
  if(ClearAllVertNormal)
    UpdateFlags<ComputeMeshType>::VertexClearV(m);
  else
  {
    UpdateFlags<ComputeMeshType>::VertexSetV(m);
    for(FaceIterator f=m.face.begin();f!=m.face.end();++f)
     if( !(*f).IsD() )
       for(int i=0;i<3;++i) (*f).V(i)->ClearV();
   }
  VertexIterator vi;
  for(vi=m.vert.begin();vi!=m.vert.end();++vi)
     if( !(*vi).IsD() && (*vi).IsRW() && (!(*vi).IsV()) )
         (*vi).N() = NormalType((ScalarType)0,(ScalarType)0,(ScalarType)0);
}

///  \brief Calculates the vertex normal as the classic area weighted average. It does not need or exploit current face normals.
/**
 The normal of a vertex v is the classical area-weigthed average of the normals of the faces incident on v.
 */
 static void PerVertex(ComputeMeshType &m)
{
 PerVertexClear(m);
 FaceIterator f;
 for(f=m.face.begin();f!=m.face.end();++f)
   if( !(*f).IsD() && (*f).IsR() )
   {
    //typename FaceType::NormalType t = (*f).Normal();
    typename FaceType::NormalType t = vcg::Normal(*f);

    for(int j=0; j<3; ++j)
     if( !(*f).V(j)->IsD() && (*f).V(j)->IsRW() )
      (*f).V(j)->N() += t;
   }
}

///  \brief Calculates the vertex normal as an angle weighted average. It does not need or exploit current face normals.
/**
 The normal of a vertex v computed as a weighted sum f the incident face normals. 
 The weight is simlply the angle of the involved wedge.  Described in:
 
G. Thurmer, C. A. Wuthrich 
"Computing vertex normals from polygonal facets"
Journal of Graphics Tools, 1998
 */
 static void PerVertexAngleWeighted(ComputeMeshType &m)
{
  PerVertexClear(m);
  FaceIterator f;
  for(f=m.face.begin();f!=m.face.end();++f)
   if( !(*f).IsD() && (*f).IsR() )
   {
    typename FaceType::NormalType t = vcg::NormalizedNormal(*f);
		NormalType e0 = ((*f).V1(0)->cP()-(*f).V0(0)->cP()).Normalize();
		NormalType e1 = ((*f).V1(1)->cP()-(*f).V0(1)->cP()).Normalize();
		NormalType e2 = ((*f).V1(2)->cP()-(*f).V0(2)->cP()).Normalize();
		
		(*f).V(0)->N() += t*AngleN(e0,-e2);
		(*f).V(1)->N() += t*AngleN(-e0,e1);
		(*f).V(2)->N() += t*AngleN(-e1,e2);
   }
}

///  \brief Calculates the vertex normal using the Max et al. weighting scheme. It does not need or exploit current face normals.
/**
 The normal of a vertex v is computed according to the formula described by Nelson Max in 
 Max, N., "Weights for Computing Vertex Normals from Facet Normals", Journal of Graphics Tools, 4(2) (1999)
 
 The weight for each wedge is the cross product of the two edge over the product of the square of the two edge lengths. 
 According to the original paper it is perfect only for spherical surface, but it should perform well...
 */
static void PerVertexNelsonMaxWeighted(ComputeMeshType &m)
{
 PerVertexClear(m);
 FaceIterator f;
 for(f=m.face.begin();f!=m.face.end();++f)
   if( !(*f).IsD() && (*f).IsR() )
   {
    typename FaceType::NormalType t = vcg::Normal(*f);
		ScalarType e0 = SquaredDistance((*f).V0(0)->cP(),(*f).V1(0)->cP());
		ScalarType e1 = SquaredDistance((*f).V0(1)->cP(),(*f).V1(1)->cP());
		ScalarType e2 = SquaredDistance((*f).V0(2)->cP(),(*f).V1(2)->cP());
		
		(*f).V(0)->N() += t/(e0*e2);
		(*f).V(1)->N() += t/(e0*e1);
		(*f).V(2)->N() += t/(e1*e2);
   }
}

/// \brief Calculates the face normal
///
/// Not normalized. Use PerFaceNormalized() or call NormalizePerVertex() if you need unit length per face normals.
static void PerFace(ComputeMeshType &m)
{
  if(!HasPerFaceNormal(m)) throw vcg::MissingComponentException("PerFaceNormal");
  for(FaceIterator f=m.face.begin();f!=m.face.end();++f)
            if( !(*f).IsD() )
                face::ComputeNormal(*f);
}

/// \brief Calculates the vertex normal by averaging the current per-face normals.
/**
    The normal of a vertex v is the average of the un-normalized normals of the faces incident on v.
*/
static void PerVertexFromCurrentFaceNormal(ComputeMeshType &m)
{
  tri::RequirePerVertexNormal(m);

 VertexIterator vi;
 for(vi=m.vert.begin();vi!=m.vert.end();++vi)
   if( !(*vi).IsD() && (*vi).IsRW() )
     (*vi).N()=CoordType(0,0,0);

 FaceIterator fi;
 for(fi=m.face.begin();fi!=m.face.end();++fi)
   if( !(*fi).IsD())
   {
    for(int j=0; j<3; ++j)
            if( !(*fi).V(j)->IsD())
                    (*fi).V(j)->N() += (*fi).cN();
   }
}

/// \brief Calculates the face normal by averaging the current per-vertex normals.
/**
    The normal of a face f is the average of the normals of the vertices of f.
*/
static void PerFaceFromCurrentVertexNormal(ComputeMeshType &m)
{
  tri::RequirePerVertexNormal(m);
  tri::RequirePerFaceNormal(m);
  for (FaceIterator fi=m.face.begin(); fi!=m.face.end(); ++fi)
   if( !(*fi).IsD())
        {
        NormalType n;
        n.SetZero();
        for(int j=0; j<3; ++j)
            n += fi->V(j)->cN();
        n.Normalize();
        fi->N() = n;
    }
}

/// \brief Normalize the length of the vertex normals.
static void NormalizePerVertex(ComputeMeshType &m)
{
  tri::RequirePerVertexNormal(m);
  for(VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi)
		if( !(*vi).IsD() && (*vi).IsRW() ) 
			(*vi).N().Normalize();
}

/// \brief Normalize the length of the face normals.
static void NormalizePerFace(ComputeMeshType &m)
{
  tri::RequirePerFaceNormal(m);
  for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi)
      if( !(*fi).IsD() )	(*fi).N().Normalize();
}

/// \brief Set the length of the face normals to their area (without recomputing their directions).
static void NormalizePerFaceByArea(ComputeMeshType &m)
{
  tri::RequirePerFaceNormal(m);
  FaceIterator fi;
  for(fi=m.face.begin();fi!=m.face.end();++fi)
    if( !(*fi).IsD() )
			{
				(*fi).N().Normalize();
				(*fi).N() = (*fi).N() * DoubleArea(*fi);
			}
}

/// \brief Equivalent to PerVertex() and NormalizePerVertex()
static void PerVertexNormalized(ComputeMeshType &m)
{
  PerVertex(m);
  NormalizePerVertex(m);
}

/// \brief Equivalent to PerFace() and NormalizePerVertex()
static void PerFaceNormalized(ComputeMeshType &m)
{
  PerFace(m);
  NormalizePerFace(m);
}

/// \brief Equivalent to PerVertex() and PerFace().
static void PerVertexPerFace(ComputeMeshType &m)
{
 PerFace(m);
 PerVertex(m);
}

/// \brief Equivalent to PerVertexNormalized() and PerFace().
static void PerVertexNormalizedPerFace(ComputeMeshType &m)
{
	PerVertexPerFace(m);
	NormalizePerVertex(m);
}

/// \brief Equivalent to PerVertexNormalizedPerFace() and NormalizePerFace().
static void PerVertexNormalizedPerFaceNormalized(ComputeMeshType &m)
{
	PerVertexNormalizedPerFace(m);
	NormalizePerFace(m);
}

/// \brief Exploit bitquads to compute a per-polygon face normal
static void PerBitQuadFaceNormalized(ComputeMeshType &m)
{
	PerFace(m);
	FaceIterator f;
	for(f=m.face.begin();f!=m.face.end();++f) {
      if( !(*f).IsD() )	{
        for (int k=0; k<3; k++) if (f->IsF(k)) 
        if (&*f < f->FFp(k)) {
          f->N() = f->FFp(k)->N() = (f->FFp(k)->N() + f->N()).Normalize();
        }
      }
  }
}

/// \brief Multiply the vertex normals by the matrix passed. By default, the scale component is removed.
static void PerVertexMatrix(ComputeMeshType &m, const Matrix44<ScalarType> &mat, bool remove_scaling= true)
{
  tri::RequirePerVertexNormal(m);
    float scale;

	Matrix33<ScalarType> mat33(mat,3);
	

	if(remove_scaling){
		scale = pow(mat33.Determinant(),(ScalarType)(1.0/3.0));
		mat33[0][0]/=scale;
		mat33[1][1]/=scale;
		mat33[2][2]/=scale;
	}
	
  for(VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi)
   if( !(*vi).IsD() && (*vi).IsRW() )
     (*vi).N()  = mat33*(*vi).N();
}

/// \brief Multiply the face normals by the matrix passed. By default, the scale component is removed.
static void PerFaceMatrix(ComputeMeshType &m, const Matrix44<ScalarType> &mat, bool remove_scaling= true)
{
  tri::RequirePerFaceNormal(m);
  float scale;

	Matrix33<ScalarType> mat33(mat,3);

	if( !HasPerFaceNormal(m)) return;

	if(remove_scaling){
		scale = pow(mat33.Determinant(),ScalarType(1.0/3.0));
		mat33[0][0]/=scale;
		mat33[1][1]/=scale;
		mat33[2][2]/=scale;
	}
	
  for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi)
   if( !(*fi).IsD() && (*fi).IsRW() )
     (*fi).N() = mat33* (*fi).N();
}

/// \brief Compute per wedge normals taking into account the angle between adjacent faces.
///
/// The PerWedge normals are averaged on common vertexes only if the angle between two faces is \b larger than \p angleRad.
/// It requires FFAdjacency.
static void PerWedgeCrease(ComputeMeshType &m, ScalarType angleRad)
{
  tri::RequirePerFaceWedgeNormal(m);
  tri::RequireFFAdjacency(m);

  ScalarType cosangle=math::Cos(angleRad);

  // Clear the per wedge normals
  for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi) if(!(*fi).IsD())
  {
    (*fi).WN(0)=NormalType(0,0,0);
    (*fi).WN(1)=NormalType(0,0,0);
    (*fi).WN(2)=NormalType(0,0,0);
  }

  for(FaceIterator fi=m.face.begin();fi!=m.face.end();++fi)		 if(!(*fi).IsD())
  {
    NormalType nn= vcg::Normal(*fi);
    for(int i=0;i<3;++i)
    {
      const NormalType &na=vcg::Normal(*(*fi).FFp(i));
      if(nn*na > cosangle )
      {
        fi->WN((i+0)%3) +=na;
        fi->WN((i+1)%3) +=na;
      }
    }
  }
}


static void PerFaceRW(ComputeMeshType &m, bool normalize=false)
{
  tri::RequirePerFaceNormal(m);
    FaceIterator f;
	bool cn = true;

	if(normalize)
	{
		for(f=m.m.face.begin();f!=m.m.face.end();++f)
		if( !(*f).IsD() && (*f).IsRW() )
		{
			for(int j=0; j<3; ++j)
				if( !(*f).V(j)->IsR()) 	cn = false;
	  if( cn ) face::ComputeNormalizedNormal(*f);
			cn = true;
		}
	}
	else
	{
		for(f=m.m.face.begin();f!=m.m.face.end();++f)
			if( !(*f).IsD() && (*f).IsRW() )
			{
				for(int j=0; j<3; ++j)
					if( !(*f).V(j)->IsR()) 	cn = false;

				if( cn )
					(*f).ComputeNormal();
				cn = true;
			}
	}
}


}; // end class

}	// End namespace
}	// End namespace

#endif
