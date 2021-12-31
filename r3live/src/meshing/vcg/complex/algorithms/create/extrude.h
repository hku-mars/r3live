#ifndef EXTRUDE_H
#define EXTRUDE_H

namespace vcg {
namespace tri {


template <class MeshType> class Extrude
{
  public:
  typedef typename MeshType::FacePointer FacePointer;
  typedef typename MeshType::EdgePointer EdgePointer;
  typedef typename MeshType::VertexPointer VertexPointer;
  typedef typename MeshType::FaceType FaceType;
  typedef typename MeshType::EdgeType EdgeType;
  typedef typename MeshType::VertexIterator VertexIterator;
  typedef typename MeshType::EdgeIterator EdgeIterator;
  typedef typename MeshType::FaceIterator FaceIterator;

static void ProfileWithCap(MeshType &profile, MeshType &fullSurface, const vcg::Similarityf &sim )
{
  fullSurface.Clear();
  MeshType lowCapSurf,topCapSurf;
  ProfileWithCap(profile,fullSurface,lowCapSurf,topCapSurf,sim);
  tri::Append<MeshType,MeshType>::Mesh(fullSurface,lowCapSurf);
  tri::Append<MeshType,MeshType>::Mesh(fullSurface,topCapSurf);
  tri::Clean<MeshType>::RemoveDuplicateVertex(fullSurface);
  bool oriented,orientable;
  tri::UpdateTopology<MeshType>::FaceFace(fullSurface);
  tri::Clean<MeshType>::OrientCoherentlyMesh(fullSurface,oriented,orientable);
}

static void ProfileWithCap(MeshType &profile, MeshType &sideSurf, MeshType &lowCapSurf, MeshType &topCapSurf, const vcg::Similarityf &sim )
{
  sideSurf.Clear();
  lowCapSurf.Clear();
  topCapSurf.Clear();

  for(VertexIterator vi=profile.vert.begin();vi!=profile.vert.end();++vi)
  {
    VertexIterator vp=tri::Allocator<MeshType>::AddVertices(sideSurf,2);
    vp->P()=vi->P();
    ++vp;
    vp->P()= sim*vi->P() ;
  }

  for(EdgeIterator ei=profile.edge.begin();ei!=profile.edge.end();++ei)
  {
    int i0=tri::Index(profile,ei->V(0));
    int i1=tri::Index(profile,ei->V(1));

    FaceIterator fp= tri::Allocator<MeshType>::AddFaces(sideSurf,2);
    fp->V(0) = &sideSurf.vert[i0*2];
    fp->V(1) = &sideSurf.vert[i1*2];
    fp->V(2) = &sideSurf.vert[i0*2+1];
    ++fp;
    fp->V(0) = &sideSurf.vert[i1*2+1];
    fp->V(1) = &sideSurf.vert[i0*2+1];
    fp->V(2) = &sideSurf.vert[i1*2];
  }

  tri::CapEdgeMesh(profile,lowCapSurf);
  if(lowCapSurf.fn==0) CapEdgeMesh(profile,lowCapSurf,true);

  tri::Append<MeshType,MeshType>::Mesh(topCapSurf,lowCapSurf);
  for(VertexIterator vi=topCapSurf.vert.begin();vi!=topCapSurf.vert.end();++vi)
    vi->P() = sim*vi->P();
//  tri::Append<MeshType,MeshType>::Mesh(sideSurf,lowCapSurf);
}

static void ProfileWithCap(MeshType &profile, MeshType &surface, const Point3f offset)
{
  Similarityf tra;
  tra.SetTranslate(offset);
  ProfileWithCap(profile,surface,tra);
}

}; // end class

} // end namespace tri
} // end namespace vcg

#endif // EXTRUDE_H
