#ifndef OUTLINE_SUPPORT_H
#define OUTLINE_SUPPORT_H

#include <vcg/math/random_generator.h>
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/bounding.h>

namespace vcg {
namespace tri {

// Naming
//  Outline2 just a vector of point2
//  Outline3 just a vector of point2
//  Outline3Vec a vector of Outline2
//  Outline2Vec a vector of Outline2
//  Mesh a common mesh of triangles
//  EdgeMesh a polyline mesh.

template < class ScalarType>
class OutlineUtil
{
public:


  /**
   * Returns the area of the polygon defined by the parameter points
   * @param a vector of points
   * @return the area of the polygon
   */
  static ScalarType Outline2Area(const std::vector< Point2<ScalarType> > &outline2)
  {
      float   area=0;
      for (size_t i=0,j=outline2.size()-1; i<outline2.size(); i++) {
          area+=(outline2[j][0]+outline2[i][0])*(outline2[j][1]-outline2[i][1]);
          j=i;
      }
      return -area*.5;
  }

  /**
   * Returns the length of the perimeter of the polygon defined by the parameter points
   * @param a vector of points
   * @return the length of the perimeter
   */
  static ScalarType Outline2Perimeter(const std::vector< Point2<ScalarType>  > &outline2)
  {
      float dd=0; int sz = outline2.size();

      //sums all the distances between point i and point i+1 (modulus sz)
      for(int j=0; j<sz; ++j)
      {
          dd += Distance (Point2f(outline2[j][0],outline2[j][1]),Point2f(outline2[(j+1)%sz][0],outline2[(j+1)%sz][1]));
      }
      return dd;
  }

  /**
   * Returns the bounding box of the polygon defined by the parameter points
   * @param a vector of points
   * @return the bounding box of the polygon
   * @see Box2
   */
  static Box2<ScalarType> Outline2BBox(const std::vector<Point2<ScalarType> > &outline2)
  {
    Box2<ScalarType> bb;
    for(size_t i=0;i<outline2.size();++i)
      bb.Add(outline2[i]);

    return bb;
  }

  /**
   * Returns the bounding box of the polygon defined by the parameter points
   * @param a vector of points
   * @return the bounding box of the polygon
   * @see Box2
   */
  static Box2<ScalarType> Outline2VecBBox(const std::vector<std::vector<Point2<ScalarType> > > &outline2Vec)
  {
    Box2<ScalarType> bb;
    for(size_t j=0;j<outline2Vec.size();++j)
      for(size_t i=0;i<outline2Vec[j].size();++i)
        bb.Add(outline2Vec[j][i]);

    return bb;
  }

  static void ReverseOutline2(std::vector< Point2<ScalarType> > &outline2)
  {
    std::reverse(outline2.begin(),outline2.end());
  }


  static void BuildRandomOutlineVec(int outlineNum, std::vector< std::vector< Point2f > > &outline2Vec, int seed=0)
  {
    vcg::math::MarsenneTwisterRNG rnd;
    if(seed==0) seed=time(0);
    rnd.initialize(seed);
    for(int i=0;i<outlineNum;++i)
    {
      std::vector<Point2f> poly;
      for(int j=0;j<10;j++)
        poly.push_back(Point2f(0.5+0.5*rnd.generate01(),2.0f*M_PI*rnd.generate01()));

      std::sort(poly.begin(),poly.end());

      float ratio = rnd.generateRange(0.2,0.9);
      float rot = rnd.generateRange(-M_PI,M_PI);
      float scale = pow(rnd.generateRange(0.3,0.9),1);

      for(size_t j=0;j<poly.size();j++)
      {
        poly[j].Polar2Cartesian();
        poly[j][1]*=ratio;
        poly[j] *= scale;
        poly[j].Cartesian2Polar();
        poly[j][1]+=rot;
        poly[j].Polar2Cartesian();
      }

      Point2f randTras(rnd.generateRange(-5,5),rnd.generateRange(-5,5));
      for(size_t j=0;j<poly.size();j++)
        poly[j]+=randTras;

      outline2Vec.push_back(poly);
    }
  }

  static int LargestOutline2(const std::vector< std::vector< Point2f > > &outline2Vec)
  {
    float maxArea =0;
    int maxInd=-1;
    for(size_t i=0;i<outline2Vec.size();++i)
    {
      float curArea = fabs(Outline2Area(outline2Vec[i]));
      if(curArea > maxArea)
      {
        maxArea=curArea;
        maxInd=i;
      }
    }
    assert(maxInd>=0);
    return maxInd;
  }

  template<class PointType>
  static bool ConvertOutline3VecToOutline2Vec(std::vector< std::vector< PointType> > &outline3Vec, std::vector< std::vector< Point2f> > &outline2Vec )
  {
    outline2Vec.resize(outline3Vec.size());
    for(size_t i=0;i<outline3Vec.size();++i)
    {
      //    printf("ConvertToPoint2fOutlines: Outline %4lu (%2lu) :  ",i,outlineVec[i].size());
      outline2Vec[i].resize(outline3Vec[i].size());
      for(size_t j=0;j<outline3Vec[i].size();++j)
      {
        outline2Vec[i][j][0]=outline3Vec[i][j][0];
        outline2Vec[i][j][1]=outline3Vec[i][j][1];
        //      printf("(%5.2f %5.2f)",outlineVec2f[i][j][0],outlineVec2f[i][j][1]);
      }
      //    printf("\n");
    }
    return true;
  }

  template<class MeshType>
  static int ConvertMeshBoundaryToOutline3Vec(MeshType &m, std::vector< std::vector<Point3f> > &outline3Vec)
  {
    typedef typename MeshType::FaceType FaceType;
    std::vector<Point3f> outline;

    tri::Allocator<MeshType>::CompactVertexVector(m);
    tri::Allocator<MeshType>::CompactFaceVector(m);
    tri::UpdateFlags<MeshType>::FaceClearV(m);
    tri::UpdateFlags<MeshType>::VertexClearV(m);
    tri::UpdateTopology<MeshType>::FaceFace(m);
    int totalVn=0;
    for(size_t i=0;i<m.face.size();i++)
    {
      for (int j=0;j<3;j++)
        if (!m.face[i].IsV() && face::IsBorder(m.face[i],j))
        {
          FaceType* startB=&(m.face[i]);
          face::Pos<FaceType> p(startB,j);
          face::Pos<FaceType> startPos = p;
          assert(p.IsBorder());
          do
          {
            assert(p.IsManifold());
            p.F()->SetV();
            outline.push_back(p.V()->P());
            p.NextB();
            totalVn++;
          }
          while(p != startPos);
          outline3Vec.push_back(outline);
          outline.clear();
        }
    }
    return totalVn;
  }

  template<class MeshType>
  static void ConvertMeshBoundaryToEdgeMesh(MeshType &m, MeshType &em)
  {
    typedef typename MeshType::VertexIterator VertexIterator;
    typedef typename MeshType::EdgeIterator EdgeIterator;
    typedef typename MeshType::VertexPointer VertexPointer;
    em.Clear();
    std::vector< std::vector<Point3f> > outlines;
    int nv = ConvertMeshBoundaryToOutlines(m,outlines);
    if (nv<2) return;
    VertexIterator vi=vcg::tri::Allocator<MeshType>::AddVertices(em,nv);
    EdgeIterator ei=vcg::tri::Allocator<MeshType>::AddEdges(em,nv);

    //  printf("Building an edge mesh of %i v and %i e and %lu outlines\n",em.vn,em.en,outlines.size());

    for (size_t i=0;i<outlines.size();i++)
    {
      VertexPointer firstVp = &*vi;
      for(size_t j=0;j<outlines[i].size();++j,++vi,++ei)
      {
        (&*vi)->P()=outlines[i][j];
        //      printf("(%5.2f %5.2f %5.2f)",vi->cP()[0],vi->cP()[1],vi->cP()[2]);
        ei->V(0)=&*vi;
        if((j+1)<outlines[i].size()) ei->V(1)=&*(vi+1);
        else ei->V(1)=firstVp;
      }
      //    printf("\n");
    }
  }

  template<class MeshType>
  static bool ConvertOutline3VecToEdgeMesh(std::vector< std::vector< Point3f> > &outlineVec, MeshType &m)
  {
    typedef typename MeshType::VertexPointer VertexPointer;
    typedef typename MeshType::EdgePointer EdgePointer;

    m.Clear();
    std::vector< std::vector<int> > Indexes(outlineVec.size());
    for(size_t i=0;i<outlineVec.size();++i)
    {
      for(size_t j=0;j<outlineVec[i].size();++j)
      {
        Indexes[i].push_back(m.vert.size());
        VertexPointer vp=&*tri::Allocator<MeshType>::AddVertices(m,1);
        Point3f pp=Point3f(outlineVec[i][j][0],outlineVec[i][j][1],outlineVec[i][j][2]);
        vp->P()= pp;
      }
    }

    for(size_t i=0;i<outlineVec.size();++i)
      for(size_t j=0;j<outlineVec[i].size();++j)
      { int polyLen =outlineVec[i].size();
        EdgePointer ep=&*tri::Allocator<MeshType>::AddEdges(m,1);
        ep->V(0)=&m.vert[Indexes[i][j]];
        ep->V(1)=&m.vert[Indexes[i][(j+1)%polyLen]];
      }
    tri::UpdateBounding<MeshType>::Box(m);
    return true;
  }

  template<class MeshType>
  static bool ConvertOutline3VecToEdgeMesh(std::vector< Point3f> &outline, MeshType &m)
  {
    std::vector< std::vector< Point3f> > outlineVec;
    outlineVec.push_back(outline);
    return Convert3DOutlinesToEdgeMesh(outlineVec,m);
  }

};

} // end namespace tri
} // end namespace vcg

#endif // OUTLINE_SUPPORT_H
