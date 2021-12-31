#ifndef POLY_TRIANGULATOR_H
#define POLY_TRIANGULATOR_H

#include "util_dae.h"
#include <wrap/gl/glu_tesselator.h>

namespace vcg {
namespace tri {
namespace io {	
		// These two classes is used for temporary storing of the
		// collected data of the polgons during the reading of files. 
		
	template<typename VERTEX_TYPE>
	class MyPolygon 
	{
	public:
		typedef VERTEX_TYPE BaseVertexType;

		int _nvert;
		std::vector<VERTEX_TYPE*> _pv;
		std::vector< vcg::TexCoord2<float> > _txc;


		MyPolygon(int n)
		:_nvert(n),_pv(_nvert),_txc(_nvert)
		{
		}
	};

	template<typename POLYGONAL_TYPE>
	class PolygonalMesh
	{
	public:
		typedef POLYGONAL_TYPE FaceType;

		enum PERWEDGEATTRIBUTETYPE {NONE = 0,NORMAL = 1,MULTITEXTURECOORD = 2,MULTICOLOR = 4};

		typedef typename FaceType::BaseVertexType VertexType;
		typedef VertexType* VertexPointer;
		typedef typename std::vector<VertexType>::iterator VertexIterator; 
		typedef typename std::vector<FaceType>::iterator PolygonIterator; 

		vcg::Box3<float> bbox;

		std::vector<VertexType> vert;
		std::vector<FaceType> _pols;

		void generatePointsVector(std::vector<std::vector<vcg::Point3f> >& v)
		{
			for(typename PolygonalMesh::PolygonIterator itp = _pols.begin();itp != _pols.end();++itp)
			{
				v.push_back(std::vector<vcg::Point3f>());
				for(typename std::vector<VertexPointer>::iterator itv = itp->_pv.begin();itv != itp->_pv.end();++itv)
				{
					v[v.size() - 1].push_back((*itv)->P());
				}	
			}
		}

		void usePerWedgeAttributes(PERWEDGEATTRIBUTETYPE att,const unsigned int multitexture = 1,const unsigned int multicolor = 1)
		{
			if (att != NONE)
			{
				for(PolygonIterator itp = _pols.begin();itp != _pols.end();++itp)
				{
					if (att & MULTICOLOR) itp->usePerWedgeColor(multicolor);
					if (att & MULTITEXTURECOORD) itp->usePerWedgeMultiTexture(multitexture);
					if (att & NORMAL) itp->usePerWedgeNormal();
				}
			}
		}

		template<class TRIMESH>
		void triangulate(TRIMESH& mesh)
		{
			std::vector<std::vector<vcg::Point3f> > pl;
			mesh.vert.resize(vert.size());
			int multicoor = 0;
			//PolygonalMesh's points has been copied in TriangularMesh
			for(size_t jj = 0;jj < mesh.vert.size();++jj)
				mesh.vert[jj].P() = vert[jj].P();

			bool texen = mesh.face.IsWedgeTexEnabled();
			unsigned int totaltri = 0;
			for(size_t ii = 0;ii < _pols.size();++ii)
					totaltri += _pols[ii]._nvert - 2;
				
			mesh.face.resize(totaltri);

			//transform the polygonal mesh in a vector<vector<Point>>
			generatePointsVector(pl);


			int trioff = 0;
			//foreach Polygon
			for(size_t ii = 0;ii < pl.size();++ii)
			{
				std::vector<int> tx;
				std::vector<std::vector<vcg::Point3f> > pl2(1);
				pl2[0] = pl[ii];

				vcg::glu_tesselator::tesselate(pl2,tx);
				size_t ntri = tx.size() / 3;
				assert(tx.size() % 3 == 0);
					

				int polvert = 0;
				//foreach triangle
				for(size_t tr = 0;tr < ntri;++tr)
				{
						
					//typename TRIMESH::FaceType& f = mesh.face[tr];

					//typename TRIMESH::FaceType& f = mesh.face[tr];
					for(unsigned int tt = 0;tt < 3; ++tt)
					{
						mesh.face[trioff + tr].V(tt) = &(mesh.vert[_pols[ii]._pv[tx[3 * tr + tt]] - &(vert[0])]);
						//vcg::Point3f ppp = mesh.face[tr].V(tt)->P();
						if (texen)
						{
						/*	f.WT(multicoor).U() = _pols[ii]._txc[polvert].U();
							f.WT(multicoor).V() = _pols[ii]._txc[polvert].V();
							f.WT(multicoor).N() = _pols[ii]._txc[polvert].N();*/
								
						}
						polvert = (polvert + 1) % _pols[ii]._nvert;
					}
					//mesh.face.push_back(f);
				}
				trioff += ntri;
			}
			assert(trioff == totaltri);
		}
	};
}
}
}

#endif
