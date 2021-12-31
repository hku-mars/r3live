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

****************************************************************************/

#ifndef __VCGLIB_TRI_CLIP
#define __VCGLIB_TRI_CLIP

#include <vector>

#ifdef _WIN32
 #ifndef __MINGW32__
  #include <hash_map>
  #define STDEXT stdext
 #else
  #include <ext/hash_map>
  #define STDEXT __gnu_cxx
 #endif
#else
 #include <ext/hash_map>
 #define STDEXT __gnu_cxx
#endif
namespace vcg
{
namespace tri
{

  template <typename MESH_TYPE>
  class GenericVertexInterpolator
  {
  public:
    typedef typename MESH_TYPE::VertexType VertexType;
    typedef GenericVertexInterpolator<MESH_TYPE> ClassType;
    typedef typename VertexType::CoordType CoordType;
    typedef typename CoordType::ScalarType ScalarType;
    GenericVertexInterpolator(MESH_TYPE &_m) : m(_m) {}
  private:
    MESH_TYPE &m;
  public:
    inline void operator () (const VertexType & v0, const VertexType & v1, const VertexType & v2, const ScalarType & a, const ScalarType & b, VertexType & r) const
    {
      // position
      r.P() = v0.cP() + (v1.cP() - v0.cP()) * a + (v2.cP() - v0.cP()) * b;

      // normal
      if (tri::HasPerVertexNormal(m))
      {
        r.N() = v0.cN() + (v1.cN() - v0.cN()) * a + (v2.cN() - v0.cN()) * b;
      }

      // color
      if (tri::HasPerVertexColor(m))
      {
        vcg::Point4<ScalarType> vc[3];
        vc[0].Import(v0.cC());
        vc[1].Import(v1.cC());
        vc[2].Import(v2.cC());
        const vcg::Point4<ScalarType> rc = (vc[0] + (vc[1] - vc[0]) * a + (vc[2] - vc[0]) * b);
        r.C()[0] = (typename vcg::Color4b::ScalarType)(rc[0]);
        r.C()[1] = (typename vcg::Color4b::ScalarType)(rc[1]);
        r.C()[2] = (typename vcg::Color4b::ScalarType)(rc[2]);
        r.C()[3] = (typename vcg::Color4b::ScalarType)(rc[3]);
      }

      // texcoord
      if (tri::HasPerVertexTexCoord(m))
      {
        const short nt = 1; //typename VertexType::TextureType::N();
        for (short i=0; i<nt; ++i)
        {
          r.T().t(i) = v0.cT().t(i) + (v1.cT().t(i) - v0.cT().t(i)) * a + (v2.cT().t(i) - v0.cT().t(i)) * b;
        }
      }
    }
  };
template <typename TRIMESHTYPE>
class TriMeshClipper
{
public:

	typedef TriMeshClipper<TRIMESHTYPE> ClassType;
	typedef TRIMESHTYPE TriMeshType;
	typedef typename TriMeshType::FaceType FaceType;
	typedef typename FaceType::VertexType VertexType;
	typedef typename VertexType::CoordType CoordType;
	typedef typename CoordType::ScalarType ScalarType;

	/*
		static inline void Box(const Box3<ScalarType> & b, VERTEXINTEPOLATOR & vInterp, TriMeshType & m);

		Clip mesh "m" against an axis aligned box (in place version);

		Notes:
			1) faces marked as deleted are skipped;
			2) faces completely outside box are marked as deleted;
			3) faces completely inside box are left unchanged;
			4) faces intersecting box's sides are marked as deleted:
			   they are replaced with proper tesselation; new vertices and faces
			   are created, so reallocation could occour; previously saved pointers
			   could not to be valid anymore, thus they should be updated;
			5) vInterp functor must implement a n operator with signature
			       void operator () (const VERTEX & v0, const VERTEX & v1, const VERTEX & v2, const Scalar & a, const Scalar & b, VERTEX & r);
			   its semantic is to intepolate vertex attribute across triangle; a typical implementation is;
				   r.P() = v0.P() + a * (v1.P() - v0.P()) + b * (v2.P() - v0.P());  // interpolate position
				   r.N() = v0.N() + a * (v1.N() - v0.N()) + b * (v2.N() - v0.N());  // interpolate normal
				   ...    // interpolate other vertex attributes
	*/
  template <class ScalarType>
      class VertexClipInfo
  {
  public:
//			typedef VertexClipInfo ClassType;

    ScalarType fU;
    ScalarType fV;
    unsigned int idx;
    unsigned int tref;
  };
  typedef typename std::vector< VertexClipInfo<ScalarType> > VertexClipInfoVec;
  class TriangleInfo
  {
  public:
    typedef TriangleInfo ClassType;

    unsigned int v[3];
    unsigned int idx;
  };

  typedef std::vector<TriangleInfo> TriangleInfoVec;

  class EdgeIsect
  {
  public:
    CoordType p;
    unsigned int idx;
  };

	template <typename VERTEXINTEPOLATOR>
	static inline void Box(const Box3<ScalarType> & b, VERTEXINTEPOLATOR & vInterp, TriMeshType & m)
	{
		std::vector<unsigned int> facesToDelete;
		ClassType::Box(b, vInterp, m, facesToDelete);
		for (size_t i=0; i<facesToDelete.size(); ++i)
		{
			m.face[facesToDelete[i]].SetD();
		}
	}

  class EdgeIntersections
  {
  public:
    unsigned int n;
    EdgeIsect isects[6];

    EdgeIntersections(void)
    {
      this->n = 0;
    }
  };

  typedef STDEXT::hash_map<unsigned int, EdgeIntersections> UIntHMap;
  typedef typename UIntHMap::iterator UIntHMap_i;
  typedef typename UIntHMap::value_type UIntHMap_v;

  typedef STDEXT::hash_map<unsigned int, UIntHMap> EdgeMap;
  typedef typename EdgeMap::iterator EdgeMap_i;
  typedef typename EdgeMap::value_type EdgeMap_v;

  typedef typename TriMeshType::FaceIterator FaceIterator;

	template <typename VERTEXINTEPOLATOR, typename FACEINDEXCONTAINER>
	static inline void Box(const Box3<ScalarType> & b, VERTEXINTEPOLATOR & vInterp, TriMeshType & m, FACEINDEXCONTAINER & facesToDelete)
	{
		if (m.fn <= 0)
		{
			return;
		}

		EdgeMap edges;
    VertexClipInfoVec vInfos;
		TriangleInfoVec tInfos;

		CoordType vTriangle[4];
		CoordType vClipped[64];

		CoordType pvP0[64];
		CoordType pvP1[64];

		unsigned int numDeletedTris = 0;
		unsigned int numTriangles = 0;
		unsigned int numVertices = 0;

		unsigned int vIdx = (unsigned int)(m.vn);
		unsigned int tIdx = (unsigned int)(m.fn);

		ScalarType boxOffsets[6];

		boxOffsets[0] =  b.min[0];
		boxOffsets[1] = -b.max[0];
		boxOffsets[2] =  b.min[1];
		boxOffsets[3] = -b.max[1];
		boxOffsets[4] =  b.min[2];
		boxOffsets[5] = -b.max[2];

		UIntHMap emptyMap;
		EdgeIntersections emptyIsects;

		const ScalarType eps = (ScalarType)(1e-6);

		for (FaceIterator it=m.face.begin(); it!=m.face.end(); ++it)
		{
			if ((*it).IsD())
			{
				continue;
			}

			unsigned int cc[3];

			cc[0] = ClassType::BoxClipCode(boxOffsets, (*it).V(0)->P());
			cc[1] = ClassType::BoxClipCode(boxOffsets, (*it).V(1)->P());
			cc[2] = ClassType::BoxClipCode(boxOffsets, (*it).V(2)->P());

			if ((cc[0] | cc[1] | cc[2]) == 0)
			{
				continue;
			}

			const unsigned int refT = (unsigned int)(std::distance(m.face.begin(), it));

			if ((cc[0] & cc[1] & cc[2]) != 0)
			{
				facesToDelete.push_back(refT);
				(*it).SetD();
				numDeletedTris++;
				continue;
			}

			facesToDelete.push_back(refT);

			vTriangle[0] = (*it).V(0)->P();
			vTriangle[1] = (*it).V(1)->P();
			vTriangle[2] = (*it).V(2)->P();
			vTriangle[3] = (*it).V(0)->P();

			unsigned int n, n0, n1;

			ClipPolygonLine(0, b.min[0], vTriangle,  4, pvP1,     n1);
			ClipPolygonLine(1, b.max[0], pvP1,      n1, pvP0,     n0);

			ClipPolygonLine(2, b.min[1], pvP0,      n0, pvP1,     n1);
			ClipPolygonLine(3, b.max[1], pvP1,      n1, pvP0,     n0);

			ClipPolygonLine(4, b.min[2], pvP0,      n0, pvP1,     n1);
			ClipPolygonLine(5, b.max[2], pvP1,      n1, vClipped,  n);

			assert(n < 64);

			unsigned int firstV, lastV;

			if (n > 2)
			{
				if (vClipped[0] == vClipped[n - 1])
				{
					n--;
				}

				const CoordType vU = vTriangle[1] - vTriangle[0];
				const CoordType vV = vTriangle[2] - vTriangle[0];

				const ScalarType tArea = (vU ^ vV).SquaredNorm();
				if (tArea < eps)
				{
					continue;
				}

				unsigned int tvidx[3];
				tvidx[0] = (*it).V(0) - &(*(m.vert.begin()));
				tvidx[1] = (*it).V(1) - &(*(m.vert.begin()));
				tvidx[2] = (*it).V(2) - &(*(m.vert.begin()));

				numTriangles += n - 2;

//				size_t vBegin = vInfos.size();

        VertexClipInfo<ScalarType> vnfo;
				TriangleInfo tnfo;

				unsigned int vmin[3];
				unsigned int vmax[3];

				if (tvidx[0] < tvidx[1])
				{
					vmin[0] = tvidx[0];
					vmax[0] = tvidx[1];
				}
				else
				{
					vmin[0] = tvidx[1];
					vmax[0] = tvidx[0];
				}

				if (tvidx[0] < tvidx[2])
				{
					vmin[1] = tvidx[0];
					vmax[1] = tvidx[2];
				}
				else
				{
					vmin[1] = tvidx[2];
					vmax[1] = tvidx[0];
				}

				if (tvidx[1] < tvidx[2])
				{
					vmin[2] = tvidx[1];
					vmax[2] = tvidx[2];
				}
				else
				{
					vmin[2] = tvidx[2];
					vmax[2] = tvidx[1];
				}

				for (unsigned int i=0; i<n; ++i)
				{
					vnfo.tref = refT;

					const CoordType vP = vClipped[i] - vTriangle[0];

					ScalarType tAreaU = (vU ^ vP).SquaredNorm();
					ScalarType tAreaV = (vP ^ vV).SquaredNorm();

					vnfo.fU = (ScalarType)(sqrt(tAreaU / tArea));
					vnfo.fV = (ScalarType)(sqrt(tAreaV / tArea));

					if (vClipped[i] == vTriangle[0])
					{
						vnfo.idx = tvidx[0];
					}
					else if (vClipped[i] == vTriangle[1])
					{
						vnfo.idx = tvidx[1];
					}
					else if (vClipped[i] == vTriangle[2])
					{
						vnfo.idx = tvidx[2];
					}
					else if (vnfo.fV < eps)
					{
						std::pair<EdgeMap_i, bool> mi = edges.insert(std::make_pair(vmin[1], emptyMap));
						std::pair<UIntHMap_i, bool> hi = (*(mi.first)).second.insert(std::make_pair(vmax[1], emptyIsects));
						bool found = false;
						for (unsigned int s=0; s<(*(hi.first)).second.n; ++s)
						{
							if (vClipped[i] == (*(hi.first)).second.isects[s].p)
							{
								found = true;
								vnfo.idx = (*(hi.first)).second.isects[s].idx;
								break;
							}
						}
						if (!found)
						{
							vnfo.idx = vIdx++;
							numVertices++;
							vInfos.push_back(vnfo);

							(*(hi.first)).second.isects[(*(hi.first)).second.n].p = vClipped[i];
							(*(hi.first)).second.isects[(*(hi.first)).second.n].idx = vnfo.idx;
							(*(hi.first)).second.n++;
						}
					}
					else if (vnfo.fU < eps)
					{
						std::pair<EdgeMap_i, bool> mi = edges.insert(std::make_pair(vmin[0], emptyMap));
						std::pair<UIntHMap_i, bool> hi = (*(mi.first)).second.insert(std::make_pair(vmax[0], emptyIsects));
						bool found = false;
						for (unsigned int s=0; s<(*(hi.first)).second.n; ++s)
						{
							if (vClipped[i] == (*(hi.first)).second.isects[s].p)
							{
								found = true;
								vnfo.idx = (*(hi.first)).second.isects[s].idx;
								break;
							}
						}
						if (!found)
						{
							vnfo.idx = vIdx++;
							numVertices++;
							vInfos.push_back(vnfo);

							(*(hi.first)).second.isects[(*(hi.first)).second.n].p = vClipped[i];
							(*(hi.first)).second.isects[(*(hi.first)).second.n].idx = vnfo.idx;
							(*(hi.first)).second.n++;
						}
					}
					else if ((vnfo.fU + vnfo.fV) >= ((ScalarType)(1.0 - 1e-5)))
					{
						std::pair<EdgeMap_i, bool> mi = edges.insert(std::make_pair(vmin[2], emptyMap));
						std::pair<UIntHMap_i, bool> hi = (*(mi.first)).second.insert(std::make_pair(vmax[2], emptyIsects));
						bool found = false;
						for (unsigned int s=0; s<(*(hi.first)).second.n; ++s)
						{
							if (vClipped[i] == (*(hi.first)).second.isects[s].p)
							{
								found = true;
								vnfo.idx = (*(hi.first)).second.isects[s].idx;
								break;
							}
						}
						if (!found)
						{
							vnfo.idx = vIdx++;
							numVertices++;
							vInfos.push_back(vnfo);

							(*(hi.first)).second.isects[(*(hi.first)).second.n].p = vClipped[i];
							(*(hi.first)).second.isects[(*(hi.first)).second.n].idx = vnfo.idx;
							(*(hi.first)).second.n++;
						}
					}
					else
					{
						vnfo.idx = vIdx++;
						numVertices++;
						vInfos.push_back(vnfo);
					}

					if (i == 0)
					{
						firstV = vnfo.idx;
					}

					if (i > 1)
					{
						tnfo.idx = tIdx++;
						tnfo.v[0] = firstV;
						tnfo.v[1] = lastV;
						tnfo.v[2] = vnfo.idx;

						tInfos.push_back(tnfo);
					}

					lastV = vnfo.idx;
				}
			}
		}

		if (numTriangles == 0)
		{
			return;
		}

		const unsigned int vSize = (unsigned int)(m.vn);
		const unsigned int tSize = (unsigned int)(m.fn);

		typedef Allocator<TriMeshType> TriMeshAllocatorType;

		TriMeshAllocatorType::AddVertices(m, numVertices);
		TriMeshAllocatorType::AddFaces(m, numTriangles);

		unsigned int j = vSize;
		for (size_t i=0; i<vInfos.size(); ++i)
		{
			if (vInfos[i].idx >= vSize)
			{
				const unsigned int tref = vInfos[i].tref;
				vInterp(*(m.face[tref].V(0)), *(m.face[tref].V(1)), *(m.face[tref].V(2)), vInfos[i].fV, vInfos[i].fU, m.vert[j]);
				j++;
			}
		}

		j = tSize;
		for (size_t i=0; i<tInfos.size(); ++i)
		{
			m.face[j].V(0) = &(m.vert[tInfos[i].v[0]]);
			m.face[j].V(1) = &(m.vert[tInfos[i].v[1]]);
			m.face[j].V(2) = &(m.vert[tInfos[i].v[2]]);
			j++;
		}
	}


	/*
		static inline void Box(const Box3<ScalarType> & b, VERTEXINTEPOLATOR & vInterp, const TriMeshType & m, TriMeshType & r);

		Clip mesh "m" against an axis aligned box and put resulting data in mesh "r" (out of place version);

		Notes:
			1) input mesh is not modified;
			2) faces marked as deleted are skipped;
			3) vInterp functor must implement a n operator with signature
			       void operator () (const VERTEX & v0, const VERTEX & v1, const VERTEX & v2, const Scalar & a, const Scalar & b, VERTEX & r);
			   its semantic is to intepolate vertex attribute across triangle; a typical implementation is;
				   r.P() = v0.P() + a * (v1.P() - v0.P()) + b * (v2.P() - v0.P());  // interpolate position
				   r.N() = v0.N() + a * (v1.N() - v0.N()) + b * (v2.N() - v0.N());  // interpolate normal
				   ...    // interpolate other vertex attributes
	*/

	template <typename VERTEXINTEPOLATOR>
	static inline void Box(const Box3<ScalarType> & b, VERTEXINTEPOLATOR & vInterp, const TriMeshType & m, TriMeshType & r)
	{
		r.Clear();

		if (m.fn <= 0)
		{
			return;
		}

    class VertexClipInfo
		{
		public:
      typedef VertexClipInfo ClassType;

			ScalarType fU;
			ScalarType fV;
			unsigned int idx;
			unsigned int tref;
		};

    typedef std::vector<VertexClipInfo> VertexClipInfoVec;

		class TriangleInfo
		{
		public:
			typedef TriangleInfo ClassType;

			unsigned int v[3];
			unsigned int idx;
		};

		typedef std::vector<TriangleInfo> TriangleInfoVec;

		class EdgeIsect
		{
		public:
			CoordType p;
			unsigned int idx;
		};

		class EdgeIntersections
		{
		public:
			unsigned int n;
			EdgeIsect isects[6];

			EdgeIntersections(void)
			{
				this->n = 0;
			}
		};

    typedef STDEXT::hash_map<unsigned int, EdgeIntersections> UIntHMap;
		typedef typename UIntHMap::iterator UIntHMap_i;
		typedef typename UIntHMap::value_type UIntHMap_v;

    typedef STDEXT::hash_map<unsigned int, UIntHMap> EdgeMap;
		typedef typename EdgeMap::iterator EdgeMap_i;
		typedef typename EdgeMap::value_type EdgeMap_v;

    typedef STDEXT::hash_map<unsigned int, unsigned int> UIHMap;
		typedef typename UIHMap::iterator UIHMap_i;

		typedef typename TriMeshType::ConstFaceIterator ConstFaceIterator;

		UIHMap origVertsMap;
		EdgeMap edges;
    VertexClipInfoVec vInfos;
		TriangleInfoVec tInfos;

		CoordType vTriangle[4];
		CoordType vClipped[64];

		CoordType pvP0[64];
		CoordType pvP1[64];

		unsigned int numDeletedTris = 0;
		unsigned int numTriangles = 0;
		unsigned int numVertices = 0;

		unsigned int vIdx = 0;
		unsigned int tIdx = 0;

		ScalarType boxOffsets[6];

		boxOffsets[0] =  b.min[0];
		boxOffsets[1] = -b.max[0];
		boxOffsets[2] =  b.min[1];
		boxOffsets[3] = -b.max[1];
		boxOffsets[4] =  b.min[2];
		boxOffsets[5] = -b.max[2];

		UIntHMap emptyMap;
		EdgeIntersections emptyIsects;

		const ScalarType eps = (ScalarType)(1e-6);

		for (ConstFaceIterator it=m.face.begin(); it!=m.face.end(); ++it)
		{
			if ((*it).IsD())
			{
				continue;
			}

			unsigned int cc[3];

			cc[0] = ClassType::BoxClipCode(boxOffsets, (*it).V(0)->P());
			cc[1] = ClassType::BoxClipCode(boxOffsets, (*it).V(1)->P());
			cc[2] = ClassType::BoxClipCode(boxOffsets, (*it).V(2)->P());

			if ((cc[0] | cc[1] | cc[2]) == 0)
			{
				TriangleInfo tnfo;
        VertexClipInfo vnfo;

				tnfo.idx = tIdx++;

				for (int i=0; i<3; ++i)
				{
					const unsigned int v = (*it).V(i) - &(*(m.vert.begin()));
					std::pair<UIHMap_i, bool> hi = origVertsMap.insert(std::make_pair(v, vIdx));

					if (hi.second)
					{
						vnfo.idx = v;
						vInfos.push_back(vnfo);
						tnfo.v[i] = vIdx++;
					}
					else
					{
						tnfo.v[i] = (*(hi.first)).second;
					}
				}

				tInfos.push_back(tnfo);

				continue;
			}

			if ((cc[0] & cc[1] & cc[2]) != 0)
			{
				numDeletedTris++;
				continue;
			}

			vTriangle[0] = (*it).V(0)->P();
			vTriangle[1] = (*it).V(1)->P();
			vTriangle[2] = (*it).V(2)->P();
			vTriangle[3] = (*it).V(0)->P();

			unsigned int n, n0, n1;

			ClipPolygonLine(0, b.min[0], vTriangle,  4, pvP1,     n1);
			ClipPolygonLine(1, b.max[0], pvP1,      n1, pvP0,     n0);

			ClipPolygonLine(2, b.min[1], pvP0,      n0, pvP1,     n1);
			ClipPolygonLine(3, b.max[1], pvP1,      n1, pvP0,     n0);

			ClipPolygonLine(4, b.min[2], pvP0,      n0, pvP1,     n1);
			ClipPolygonLine(5, b.max[2], pvP1,      n1, vClipped,  n);

			assert(n < 64);

			unsigned int firstV, lastV;


			if (n > 2)
			{
				if (vClipped[0] == vClipped[n - 1])
				{
					n--;
				}

				const CoordType vU = vTriangle[1] - vTriangle[0];
				const CoordType vV = vTriangle[2] - vTriangle[0];

				const ScalarType tArea = (vU ^ vV).SquaredNorm();
				if (tArea < eps)
				{
					continue;
				}

				unsigned int tvidx[3];
				tvidx[0] = (*it).V(0) - &(*(m.vert.begin()));
				tvidx[1] = (*it).V(1) - &(*(m.vert.begin()));
				tvidx[2] = (*it).V(2) - &(*(m.vert.begin()));

				unsigned int refT = (unsigned int)(std::distance(m.face.begin(), it));

				numTriangles += n - 2;

				size_t vBegin = vInfos.size();

        VertexClipInfo vnfo;
				TriangleInfo tnfo;

				unsigned int vmin[3];
				unsigned int vmax[3];

				if (tvidx[0] < tvidx[1])
				{
					vmin[0] = tvidx[0];
					vmax[0] = tvidx[1];
				}
				else
				{
					vmin[0] = tvidx[1];
					vmax[0] = tvidx[0];
				}

				if (tvidx[0] < tvidx[2])
				{
					vmin[1] = tvidx[0];
					vmax[1] = tvidx[2];
				}
				else
				{
					vmin[1] = tvidx[2];
					vmax[1] = tvidx[0];
				}

				if (tvidx[1] < tvidx[2])
				{
					vmin[2] = tvidx[1];
					vmax[2] = tvidx[2];
				}
				else
				{
					vmin[2] = tvidx[2];
					vmax[2] = tvidx[1];
				}

				for (unsigned int i=0; i<n; ++i)
				{
					vnfo.tref = refT;

					const CoordType vP = vClipped[i] - vTriangle[0];

					ScalarType tAreaU = (vU ^ vP).SquaredNorm();
					ScalarType tAreaV = (vP ^ vV).SquaredNorm();

					vnfo.fU = (ScalarType)(sqrt(tAreaU / tArea));
					vnfo.fV = (ScalarType)(sqrt(tAreaV / tArea));

					unsigned int currVIdx;

					if (vClipped[i] == vTriangle[0])
					{
						std::pair<UIHMap_i, bool> hi = origVertsMap.insert(std::make_pair(tvidx[0], vIdx));
						if (hi.second)
						{
							vnfo.idx = tvidx[0];
							vInfos.push_back(vnfo);
							currVIdx = vIdx++;
						}
						else
						{
							currVIdx = (*(hi.first)).second;
						}
					}
					else if (vClipped[i] == vTriangle[1])
					{
						std::pair<UIHMap_i, bool> hi = origVertsMap.insert(std::make_pair(tvidx[1], vIdx));
						if (hi.second)
						{
							vnfo.idx = tvidx[1];
							vInfos.push_back(vnfo);
							currVIdx = vIdx++;
						}
						else
						{
							currVIdx = (*(hi.first)).second;
						}
					}
					else if (vClipped[i] == vTriangle[2])
					{
						std::pair<UIHMap_i, bool> hi = origVertsMap.insert(std::make_pair(tvidx[2], vIdx));
						if (hi.second)
						{
							vnfo.idx = tvidx[2];
							vInfos.push_back(vnfo);
							currVIdx = vIdx++;
						}
						else
						{
							currVIdx = (*(hi.first)).second;
						}
					}
					else if (vnfo.fV < eps)
					{
						std::pair<EdgeMap_i, bool> mi = edges.insert(std::make_pair(vmin[1], emptyMap));
						std::pair<UIntHMap_i, bool> hi = (*(mi.first)).second.insert(std::make_pair(vmax[1], emptyIsects));
						bool found = false;
						for (unsigned int s=0; s<(*(hi.first)).second.n; ++s)
						{
							if (vClipped[i] == (*(hi.first)).second.isects[s].p)
							{
								found = true;
								vnfo.idx = (unsigned int)(-1);
								currVIdx = (*(hi.first)).second.isects[s].idx;
								break;
							}
						}
						if (!found)
						{
							(*(hi.first)).second.isects[(*(hi.first)).second.n].p = vClipped[i];
							(*(hi.first)).second.isects[(*(hi.first)).second.n].idx = vIdx;
							(*(hi.first)).second.n++;

							vnfo.idx = (unsigned int)(-1);
							numVertices++;
							vInfos.push_back(vnfo);
							currVIdx = vIdx++;
						}
					}
					else if (vnfo.fU < eps)
					{
						std::pair<EdgeMap_i, bool> mi = edges.insert(std::make_pair(vmin[0], emptyMap));
						std::pair<UIntHMap_i, bool> hi = (*(mi.first)).second.insert(std::make_pair(vmax[0], emptyIsects));
						bool found = false;
						for (unsigned int s=0; s<(*(hi.first)).second.n; ++s)
						{
							if (vClipped[i] == (*(hi.first)).second.isects[s].p)
							{
								found = true;
								vnfo.idx = (unsigned int)(-1);
								currVIdx = (*(hi.first)).second.isects[s].idx;
								break;
							}
						}
						if (!found)
						{
							(*(hi.first)).second.isects[(*(hi.first)).second.n].p = vClipped[i];
							(*(hi.first)).second.isects[(*(hi.first)).second.n].idx = vIdx;
							(*(hi.first)).second.n++;

							vnfo.idx = (unsigned int)(-1);
							numVertices++;
							vInfos.push_back(vnfo);
							currVIdx = vIdx++;
						}
					}
					else if ((vnfo.fU + vnfo.fV) >= ((ScalarType)(1.0 - 1e-5)))
					{
						std::pair<EdgeMap_i, bool> mi = edges.insert(std::make_pair(vmin[2], emptyMap));
						std::pair<UIntHMap_i, bool> hi = (*(mi.first)).second.insert(std::make_pair(vmax[2], emptyIsects));
						bool found = false;
						for (unsigned int s=0; s<(*(hi.first)).second.n; ++s)
						{
							if (vClipped[i] == (*(hi.first)).second.isects[s].p)
							{
								found = true;
								vnfo.idx = (unsigned int)(-1);
								currVIdx = (*(hi.first)).second.isects[s].idx;
								break;
							}
						}
						if (!found)
						{
							(*(hi.first)).second.isects[(*(hi.first)).second.n].p = vClipped[i];
							(*(hi.first)).second.isects[(*(hi.first)).second.n].idx = vIdx;
							(*(hi.first)).second.n++;

							vnfo.idx = (unsigned int)(-1);
							numVertices++;
							vInfos.push_back(vnfo);
							currVIdx = vIdx++;
						}
					}
					else
					{
						vnfo.idx = (unsigned int)(-1);
						numVertices++;
						vInfos.push_back(vnfo);
						currVIdx = vIdx++;
					}

					if (i == 0)
					{
						firstV = currVIdx;
					}

					if (i > 1)
					{
						tnfo.idx = tIdx++;
						tnfo.v[0] = firstV;
						tnfo.v[1] = lastV;
						tnfo.v[2] = currVIdx;

						tInfos.push_back(tnfo);
					}

					lastV = currVIdx;
				}
			}
		}

		if (tInfos.empty())
		{
			return;
		}

		const unsigned int vSize = (unsigned int)(m.vn);
		const unsigned int tSize = (unsigned int)(m.fn);

		typedef Allocator<TriMeshType> TriMeshAllocatorType;

		TriMeshAllocatorType::AddVertices(r, (int)(vInfos.size()));
		TriMeshAllocatorType::AddFaces(r, (int)(tInfos.size()));

		for (size_t i=0; i<vInfos.size(); ++i)
		{
			if (vInfos[i].idx != ((unsigned int)(-1)))
			{
				r.vert[i] = m.vert[vInfos[i].idx];
			}
			else
			{
				const unsigned int tref = vInfos[i].tref;
				vInterp(*(m.face[tref].V(0)), *(m.face[tref].V(1)), *(m.face[tref].V(2)), vInfos[i].fV, vInfos[i].fU, r.vert[i]);
			}
		}

		for (size_t i=0; i<tInfos.size(); ++i)
		{
			r.face[i].V(0) = &(r.vert[tInfos[i].v[0]]);
			r.face[i].V(1) = &(r.vert[tInfos[i].v[1]]);
			r.face[i].V(2) = &(r.vert[tInfos[i].v[2]]);
		}
	}

protected:

	static inline unsigned int BoxClipCode(const ScalarType * offsets, const CoordType & p)
	{
		//const ScalarType eps = (ScalarType)(-1e-5);
		const ScalarType eps = (ScalarType)(0);
		unsigned int code = 0;

		code |= ((( p[0] - offsets[0]) < eps) ? (1 << 0) : (0));
		code |= (((-p[0] - offsets[1]) < eps) ? (1 << 1) : (0));
		code |= ((( p[1] - offsets[2]) < eps) ? (1 << 2) : (0));
		code |= (((-p[1] - offsets[3]) < eps) ? (1 << 3) : (0));
		code |= ((( p[2] - offsets[4]) < eps) ? (1 << 4) : (0));
		code |= (((-p[2] - offsets[5]) < eps) ? (1 << 5) : (0));

		return (code);
	}

	static inline unsigned int InRegion(int mode, const ScalarType & value, const CoordType & p_in)
	{
		//const ScalarType eps = (ScalarType)(-1e-5);
		const ScalarType eps = (ScalarType)(0);
		unsigned int flag = 0;

		switch(mode)
		{
			case 0:
				flag = p_in[0] + eps < value;
				break;
			case 1:
				flag = p_in[0] > value + eps;
				break;
			case 2:
				flag = p_in[1] + eps < value;
				break;
			case 3:
				flag = p_in[1] > value + eps;
				break;
			case 4:
				flag = p_in[2] + eps < value;
				break;
			case 5:
				flag = p_in[2] > value + eps;
				break;
			default:
				break;
		}

		return (flag);
	}

	static inline void CrossPoint(int mode, const ScalarType & value, const CoordType & SP, const CoordType & PP, CoordType & p_out)
	{
		switch(mode)
		{
			case 0:
			case 1:
				p_out[0] = value;
				if ((PP[0] - SP[0]) == ((ScalarType)(0)))
				{
					p_out[1] = PP[1];
					p_out[2] = PP[2];
				}
				else
				{
					p_out[1] = SP[1] + (value - SP[0]) * (PP[1] - SP[1]) / (PP[0] - SP[0]);
					p_out[2] = SP[2] + (value - SP[0]) * (PP[2] - SP[2]) / (PP[0] - SP[0]);
				}
				break;
			case 2:
			case 3:
				p_out[1] = value;
				if ((PP[1] - SP[1]) == ((ScalarType)(0)))
				{
					p_out[0] = PP[0];
					p_out[2] = PP[2];
				}
				else
				{
					p_out[0] = SP[0] + (value - SP[1]) * (PP[0] - SP[0]) / (PP[1] - SP[1]);
					p_out[2] = SP[2] + (value - SP[1]) * (PP[2] - SP[2]) / (PP[1] - SP[1]);
				}
				break;
			case 4:
			case 5:
				p_out[2] = value;
				if ((PP[2] - SP[2]) == ((ScalarType)(0)))
				{
					p_out[0] = PP[0];
					p_out[1] = PP[1];
				}
				else
				{
					p_out[0] = SP[0] + (value - SP[2]) * (PP[0] - SP[0]) / (PP[2] - SP[2]);
					p_out[1] = SP[1] + (value - SP[2]) * (PP[1] - SP[1]) / (PP[2] - SP[2]);
				}
				break;
			default:
				break;
		}
	}

	static inline void ClipPolygonLine(int mode, const ScalarType & value, CoordType * P_in, unsigned int n_in, CoordType * P_out, unsigned int & n_out)
	{
		unsigned int ps;
		CoordType * SP;
		CoordType * PP;

		n_out = 0;
		SP = &P_in[n_in-1];

		if (ClassType::InRegion(mode, value, *SP))
		{
			ps = 0;
		}
		else
		{
			ps = 2;
		}

		for(unsigned int i=0; i<n_in; ++i)
		{
			PP = &(P_in[i]);
			ps = (ps >> 1) | ((ClassType::InRegion(mode, value, *PP)) ? (0) : (2));

			switch(ps)
			{
				case 0:
					break;
				case 1:
					ClassType::CrossPoint(mode, value, *SP, *PP, P_out[n_out]);
					n_out++;
					break;
				case 2:
					ClassType::CrossPoint(mode, value, *SP, *PP, P_out[n_out]);
					n_out++;
					P_out[n_out] = *PP;
					n_out++;
					break;
				case 3:
					P_out[n_out] = *PP;
					n_out++;
					break;
				default:
					break;
			}

			SP = PP;
		}
	}

};

} // end namespace tri
} // end namespace vcg

#endif // __VCGLIB_TRI_CLIP
