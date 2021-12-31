#ifndef GLU_TESSELLATOR_CAP_H
#define GLU_TESSELLATOR_CAP_H
#include "glu_tesselator.h"
#include <vcg/simplex/edge/pos.h>
#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/update/bounding.h>

namespace vcg {
namespace tri {


// This function take a mesh with one or more boundary stored as edges, and fill another mesh with a triangulation of that boundaries.
// it assumes that boundary are planar and exploits glutessellator for the triangulaiton
template <class MeshType>
void CapEdgeMesh(MeshType &em, MeshType &cm, bool revertFlag=false)
{
  typedef typename MeshType::EdgeType EdgeType;
  std::vector< std::vector<Point3f> > outlines;
  std::vector<Point3f> outline;
  UpdateFlags<MeshType>::EdgeClearV(em);
  UpdateTopology<MeshType>::EdgeEdge(em);
  int nv=0;
  for(size_t i=0;i<em.edge.size();i++) if(!em.edge[i].IsD())
  {
    if (!em.edge[i].IsV())
    {
      edge::Pos<EdgeType> startE(&em.edge[i],0);
       edge::Pos<EdgeType> curE=startE;
      do
      {
        curE.E()->SetV();
        outline.push_back(curE.V()->P());
        curE.NextE();
        nv++;
      }
      while(curE != startE);
      if(revertFlag) std::reverse(outline.begin(),outline.end());
      outlines.push_back(outline);
      outline.clear();
    }
  }
  if (nv<2) return;
//  printf("Found %i outlines for a total of %i vertices",outlines.size(),nv);

  typename MeshType::VertexIterator vi=vcg::tri::Allocator<MeshType>::AddVertices(cm,nv);
  for (size_t i=0;i<outlines.size();i++)
  {
    for(size_t j=0;j<outlines[i].size();++j,++vi)
      (&*vi)->P()=outlines[i][j];
  }

  std::vector<int> indices;
  glu_tesselator::tesselate(outlines, indices);
  std::vector<Point3f> points;
  glu_tesselator::unroll(outlines, points);
  //typename MeshType::FaceIterator fi=tri::Allocator<MeshType>::AddFaces(cm,nv-2);
  typename MeshType::FaceIterator fi=tri::Allocator<MeshType>::AddFaces(cm,indices.size()/3);
  for (size_t i=0; i<indices.size(); i+=3,++fi)
  {
    (*&fi)->V(0)=&cm.vert[ indices[i+0] ];
    (*&fi)->V(1)=&cm.vert[ indices[i+1] ];
    (*&fi)->V(2)=&cm.vert[ indices[i+2] ];
  }
  Clean<MeshType>::RemoveDuplicateVertex(cm);
  UpdateBounding<MeshType>::Box(cm);
}

} // end namespace tri
} // end namespace vcg

#endif // GLU_TESSELLATOR_CAP_H
