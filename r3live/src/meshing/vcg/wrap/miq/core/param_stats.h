#ifndef PARAM_STATS_H
#define PARAM_STATS_H
#ifdef MIQ_USE_ROBUST
#include <predicates.h>
#endif
#include <vcg/space/triangle2.h>

template<typename CoordType2D>
inline bool IsFlipped(const CoordType2D &uv0,
                      const CoordType2D &uv1,
                      const CoordType2D &uv2)
{
    #ifdef MIQ_USE_ROBUST
    double pa[2] = {uv0.X(), uv0.Y()};
    double pb[2] = {uv1.X(), uv1.Y()};
    double pc[2] = {uv2.X(), uv2.Y()};
    return (orient2d(pa, pb, pc) < 0);
    #else
    vcg::Triangle2<typename CoordType2D::ScalarType> t2(uv0,uv1,uv2);
    return (!t2.IsCCW());
    #endif
}

template<typename FaceType>
inline bool IsFlipped( FaceType &f)
{

    typedef typename FaceType::ScalarType ScalarType;
    typedef typename vcg::Point2<ScalarType> CoordType2D;
    CoordType2D uv0=f.WT(0).P();
    CoordType2D uv1=f.WT(1).P();
    CoordType2D uv2=f.WT(2).P();
    return (IsFlipped(uv0,uv1,uv2));

}

template< class MeshType>
int NumFlips(MeshType &Tmesh)
{
    int numFl=0;
    for (unsigned int i=0;i<Tmesh.face.size();i++)
    {
        if (Tmesh.face[i].IsD())continue;
        if (IsFlipped(Tmesh.face[i]))numFl++;
    }
    return numFl;
}

template< class MeshType>
void SelectFlippedFaces(MeshType &Tmesh)
{
    typedef typename MeshType::ScalarType ScalarType;
    typedef typename vcg::Point2<ScalarType> CoordType2D;
    vcg::tri::UpdateFlags<MeshType>::FaceClearS(Tmesh);
    for (unsigned int i=0;i<Tmesh.face.size();i++)
    {
        CoordType2D uv0=Tmesh.face[i].WT(0).P();
        CoordType2D uv1=Tmesh.face[i].WT(1).P();
        CoordType2D uv2=Tmesh.face[i].WT(2).P();
        if (IsFlipped(uv0,uv1,uv2) ) Tmesh.face[i].SetS();
    }
}

// Compute the lambda (distortion) of a triangle in the current
//  parameerization from the Mixed Integer paper.
//  f   facet on which to compute distortion
//  h   scaling factor applied to cross field
//  return the  triangle's distortion
template< class FaceType>
typename FaceType::ScalarType Distortion(FaceType &f,typename FaceType::ScalarType h)
{
    typedef typename FaceType::CoordType CoordType3D;
    typedef typename FaceType::ScalarType ScalarType;
    typedef typename vcg::Point2<ScalarType> CoordType2D;
    typedef typename FaceType::ScalarType ScalarType;

    //Facet_const_handle self = f;
    assert(h > 0);

    //TriangleIndex ftri = m_flattenedTriangles[f.index()];
    //TriangleIndex  tri = m_triangles[f.index()];
    CoordType2D uv0 = f.WT(0).P();
    CoordType2D uv1 = f.WT(1).P();
    CoordType2D uv2 = f.WT(2).P();

    CoordType3D p0 = f.P(0);
    CoordType3D p1 = f.P(1);
    CoordType3D p2 = f.P(2);

    CoordType3D norm = (p1 - p0) ^ (p2 - p0);
    ScalarType area2 = (norm).Norm();
    ScalarType area2_inv  = 1.0 / area2;
    norm *= area2_inv;

    if (area2 > 0) {
        // Singular values of the Jacobian
        CoordType3D neg_t0 = norm ^ (p2 - p1);
        CoordType3D neg_t1 = norm ^ ( p0 - p2);
        CoordType3D neg_t2 = norm ^ ( p1 - p0);

        /*CoordType3D diffu = area2_inv * (uv0.X() * neg_t0 +
                              uv1.X() * neg_t1 + uv2.X() * neg_t2);
        CoordType3D diffv = area2_inv * (uv0.Y() * neg_t0 +
                              uv1.Y() * neg_t1 + uv2.Y() * neg_t2);*/

        CoordType3D diffu =  (neg_t0 * uv0.X() +neg_t1 *uv1.X() +  neg_t2 * uv2.X() )*area2_inv;
        CoordType3D diffv = (neg_t0 * uv0.Y() + neg_t1*uv1.Y() +  neg_t2*uv2.Y() )*area2_inv;
        // first fundamental form
        ScalarType I00 = diffu*diffu;  // guaranteed non-neg
        ScalarType I01 = diffu*diffv;  // I01 = I10
        ScalarType I11 = diffv*diffv;  // guaranteed non-neg
        // eigenvalues of a 2x2 matrix
        // [a00 a01]
        // [a10 a11]
        // 1/2 * [ (a00 + a11) +/- sqrt((a00 - a11)^2 + 4 a01 a10) ]
        ScalarType trI = I00 + I11;                     // guaranteed non-neg
        ScalarType diffDiag = I00 - I11;                // guaranteed non-neg
        ScalarType sqrtDet = sqrt(std::max(0.0, diffDiag*diffDiag +
                                 4 * I01 * I01)); // guaranteed non-neg
        ScalarType sig1 = 0.5 * (trI + sqrtDet); // higher singular value
        ScalarType sig2 = 0.5 * (trI - sqrtDet); // lower singular value

        // Avoid sig2 < 0 due to numerical error
        if (fabs(sig2) < 1.0e-8) sig2 = 0;
            assert(sig1 >= 0);
        assert(sig2 >= 0);

        if (sig2 < 0) {
            printf("Distortion will be NaN! sig1^2 is negative (%lg)\n",
                    sig2);
        }

        // The singular values of the Jacobian are the sqrts of the
        // eigenvalues of the first fundamental form.
        sig1 = sqrt(sig1);
        sig2 = sqrt(sig2);

        // distortion
        ScalarType tao = IsFlipped(f) ? -1 : 1;
        ScalarType factor = tao / h;
        ScalarType lam = fabs(factor * sig1 - 1) + fabs(factor * sig2 - 1);
        return lam;
    }
    else {
        return 10; // something "large"
    }
}

////////////////////////////////////////////////////////////////////////////
// Approximate the distortion laplacian using a uniform laplacian on
//  the dual mesh:
//      ___________
//      \-1 / \-1 /
//       \ / 3 \ /
//        \-----/
//         \-1 /
//          \ /
//
//  @param[in]  f   facet on which to compute distortion laplacian
//  @param[in]  h   scaling factor applied to cross field
//  @return     distortion laplacian for f
///////////////////////////////////////////////////////////////////////////
template< class FaceType>
typename FaceType::ScalarType LaplaceDistortion(FaceType &f ,typename FaceType::ScalarType h)
{
    typedef typename FaceType::ScalarType ScalarType;
    ScalarType mydist = Distortion(f, h);
    ScalarType lapl=0;
    for (int i=0;i<3;i++)
        lapl += (mydist- Distortion(*f.FFp(i), h));
    return lapl;
}

template< class MeshType>
void UpdateUVBox(MeshType &mesh)
{
  typedef typename MeshType::ScalarType ScalarType;
  typename  MeshType::template PerMeshAttributeHandle<vcg::Box2<ScalarType> > Handle_UVBox;

    Handle_UVBox=vcg::tri::Allocator<MeshType>::template GetPerMeshAttribute<vcg::Box2<ScalarType> >(mesh,std::string("UVBox"));

  Handle_UVBox().SetNull();
  for (unsigned int i=0;i<mesh.face.size();i++)
  {
      if (mesh.face[i].IsD())continue;
      Handle_UVBox().Add(mesh.face[i].WT(0).P());
      Handle_UVBox().Add(mesh.face[i].WT(1).P());
      Handle_UVBox().Add(mesh.face[i].WT(2).P());
  }
}

template< class MeshType>
void SetFaceQualityByDistortion(MeshType &Tmesh,
                                typename MeshType::ScalarType h)
{
    typedef typename MeshType::FaceType FaceType;
    typedef typename MeshType::ScalarType ScalarType;
    ScalarType minD=100;
    ScalarType maxD=0;

    ///evaluate min and max
    for (unsigned int i=0;i<Tmesh.face.size();i++)
    {
        ScalarType dist=Distortion<FaceType>(Tmesh.face[i],h);
        if (dist>maxD)maxD=dist;
        if (dist<minD)minD=dist;
        Tmesh.face[i].Q()= dist;
    }
    for (unsigned int i=0;i<Tmesh.face.size();i++)
    {
        Tmesh.face[i].Q()= maxD-Tmesh.face[i].Q();
    }
    printf("Distortion MIN %4.4f \n",(float)minD);
    printf("Distortion MAX %4.4f \n",(float)maxD);
}

#endif // PARAM_STATS_H
