#ifndef NORMAL_EXTRAPOLATION_H
#define NORMAL_EXTRAPOLATION_H

#include <vcg/space/index/kdtree/kdtree.h>
#include <vcg/space/fitting3.h>
#include <vcg/complex/algorithms/smooth.h>

namespace vcg {
namespace tri {
///
/** \addtogroup trimesh */
/*@{*/
/// Class of static functions to smooth and fair meshes and their attributes.



template <typename MeshType>
class PointCloudNormal {
public:

  typedef typename MeshType::VertexType     VertexType;
  typedef typename MeshType::VertexType::CoordType     CoordType;
  typedef typename MeshType::VertexPointer  VertexPointer;
  typedef typename MeshType::VertexIterator VertexIterator;
  typedef typename MeshType::ScalarType			ScalarType;

  class WArc
  {
  public:
    WArc(VertexPointer _s,VertexPointer _t):src(_s),trg(_t),w(fabs(_s->cN()*_t->cN())){}

    VertexPointer src;
    VertexPointer trg;
    float w;
    bool operator< (const WArc &a) const {return w<a.w;}
  };

  static void ComputeUndirectedNormal(MeshType &m, int nn, float maxDist, KdTree<float> &tree,vcg::CallBackPos * cb=0)
  {
    tree.setMaxNofNeighbors(nn);
    int cnt=0;
    int step=m.vn/100;
    for (VertexIterator vi=m.vert.begin();vi!=m.vert.end();++vi)
    {
        tree.doQueryK(vi->cP());
        if(cb && (++cnt%step)==0) cb(cnt/step,"Fitting planes");

        int neighbours = tree.getNofFoundNeighbors();
        std::vector<CoordType> ptVec;

        for (int i = 0; i < neighbours; i++)
        {
            int neightId = tree.getNeighborId(i);
            if(Distance(vi->cP(),m.vert[neightId].cP())<maxDist)
            ptVec.push_back(m.vert[neightId].cP());
        }
        Plane3f plane;
        FitPlaneToPointSet(ptVec,plane);
        vi->N()=plane.Direction();
    }
  }

  static void AddNeighboursToHeap( MeshType &m, VertexPointer vp, KdTree<float> &tree, std::vector<WArc> &heap)
  {
    tree.doQueryK(vp->cP());

    int neighbours = tree.getNofFoundNeighbors();
    for (int i = 0; i < neighbours; i++)
    {
        int neightId = tree.getNeighborId(i);
        if (neightId < m.vn && (&m.vert[neightId] != vp))
        {
          if(!m.vert[neightId].IsV())
          {
            heap.push_back(WArc(vp,&(m.vert[neightId])));
            //std::push_heap(heap.begin(),heap.end());
            if(heap.back().w < 0.3f) 
				heap.pop_back();
			else
				std::push_heap(heap.begin(),heap.end());
          }
        }
    }
    //std::push_heap(heap.begin(),heap.end());
  }
  /*! \brief parameters for the normal generation
   */
  struct Param
  {
    Param():
      fittingAdjNum(10),
      smoothingIterNum(0),
      coherentAdjNum(8),
      viewPoint(0,0,0),
      useViewPoint(false)
    {}

    int fittingAdjNum; /// number of adjacent nodes used for computing the fitting plane
    int smoothingIterNum; /// number of itaration of a simple normal smoothing (use the same number of ajdacent of fittingAdjNjm)
    int coherentAdjNum; /// number of nodes used in the coherency pass
    Point3f viewPoint;  /// position of a viewpoint used to disambiguate direction
    bool useViewPoint;  /// if the position of the viewpoint has to be used.
  };

  static void Compute(MeshType &m, Param p, vcg::CallBackPos * cb=0)
  {
    tri::Allocator<MeshType>::CompactVertexVector(m);
    if(cb) cb(1,"Building KdTree...");
    VertexConstDataWrapper<MeshType> DW(m);
    KdTree<float> tree(DW);

    ComputeUndirectedNormal(m, p.fittingAdjNum, std::numeric_limits<ScalarType>::max(), tree,cb);

    tri::Smooth<MeshType>::VertexNormalPointCloud(m,p.fittingAdjNum,p.smoothingIterNum,&tree);

    if(p.coherentAdjNum==0) return;
    tree.setMaxNofNeighbors(p.coherentAdjNum+1);

    tri::UpdateFlags<MeshType>::VertexClearV(m);
    std::vector<WArc> heap;
    VertexIterator vi=m.vert.begin();

    while(true)
    {
      // search an unvisited vertex
      while(vi!=m.vert.end() && vi->IsV())
        ++vi;

      if(vi==m.vert.end()) return;

      if ( p.useViewPoint &&
          ( vi->N().dot(p.viewPoint- vi->P())<0.0f) )
          vi->N()=-(*vi).N();

      vi->SetV();
      AddNeighboursToHeap(m,&*vi,tree,heap);

      while(!heap.empty())
      {
        std::pop_heap(heap.begin(),heap.end());
        WArc a = heap.back();
        heap.pop_back();
        if(!a.trg->IsV())
        {
          a.trg->SetV();
          if(a.src->cN()*a.trg->cN()<0.0f)
            if(!p.useViewPoint || ( a.trg->N().dot(p.viewPoint- a.trg->P())<0.0f)) // test to prevent flipping according to viewpos
              a.trg->N()=-a.trg->N();
          AddNeighboursToHeap(m,a.trg,tree,heap);
        }
      }
    }

    return;
  }

};
}//end namespace vcg
}//end namespace vcg
#endif // NORMAL_EXTRAPOLATION_H
