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

#ifndef VCGLIB_UPDATE_CURVATURE_FITTING
#define VCGLIB_UPDATE_CURVATURE_FITTING

#include <vcg/space/index/grid_static_ptr.h>
#include <vcg/simplex/face/topology.h>
#include <vcg/simplex/face/pos.h>
#include <vcg/simplex/face/jumping_pos.h>
#include <vcg/container/simple_temporary_data.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/point_sampling.h>
#include <vcg/complex/algorithms/intersection.h>
#include <vcg/complex/algorithms/inertia.h>
#include <vcg/complex/algorithms/nring.h>

#include <eigenlib/Eigen/Core>
#include <eigenlib/Eigen/QR>
#include <eigenlib/Eigen/LU>
#include <eigenlib/Eigen/SVD>
#include <eigenlib/Eigen/Eigenvalues>


namespace vcg {
namespace tri {

/// \ingroup trimesh

/// \headerfile curvature_fitting.h vcg/complex/algorithms/update/curvature_fitting.h

/// \brief Computation of per-vertex directions and values of curvature.
/**
This class is used to compute the per-vertex directions and values of curvature using a quadric fitting method.
*/

template <class MeshType>
class UpdateCurvatureFitting
{

public:
	typedef typename MeshType::FaceType FaceType;
	typedef typename MeshType::FacePointer FacePointer;
	typedef typename MeshType::FaceIterator FaceIterator;
	typedef typename MeshType::VertexIterator VertexIterator;
	typedef typename MeshType::VertContainer VertContainer;
	typedef typename MeshType::VertexType VertexType;
	typedef typename MeshType::VertexPointer VertexPointer;
        typedef typename MeshType::VertexPointer VertexTypeP;
	typedef vcg::face::VFIterator<FaceType> VFIteratorType;
	typedef typename MeshType::CoordType CoordType;
	typedef typename CoordType::ScalarType ScalarType;

class Quadric
{
    public:

    Quadric(double av, double bv, double cv, double dv, double ev)
    {
        a() = av;
        b() = bv;
        c() = cv;
        d() = dv;
        e() = ev;
    }

    double& a() { return data[0];}
    double& b() { return data[1];}
    double& c() { return data[2];}
    double& d() { return data[3];}
    double& e() { return data[4];}

    double data[5];

    double evaluate(double u, double v)
    {
        return a()*u*u + b()*u*v + c()*v*v + d()*u + e()*v;
    }

    double du(double u, double v)
    {
        return 2.0*a()*u + b()*v + d();
    }

    double dv(double u, double v)
    {
        return 2.0*c()*v + b()*u + e();
    }

    double duv(double /*u*/, double /*v*/)
    {
        return b();
    }

    double duu(double /*u*/, double /*v*/)
    {
        return 2.0*a();
    }

    double dvv(double /*u*/, double /*v*/)
    {
        return 2.0*c();
    }

    static Quadric fit(std::vector<CoordType> VV)
    {
        assert(VV.size() >= 5);
        Eigen::MatrixXf A(VV.size(),5);
        Eigen::MatrixXf b(VV.size(),1);
        Eigen::MatrixXf sol(VV.size(),1);

        for(unsigned int c=0; c < VV.size(); ++c)
        {
            double u = VV[c].X();
            double v = VV[c].Y();
            double n = VV[c].Z();

            A(c,0) = u*u;
            A(c,1) = u*v;
            A(c,2) = v*v;
            A(c,3) = u;
            A(c,4) = v;

            b(c,0) = n;
        }

        sol = ((A.transpose()*A).inverse()*A.transpose())*b;
        return Quadric(sol(0,0),sol(1,0),sol(2,0),sol(3,0),sol(4,0));
    }
};

    static CoordType project(VertexType* v, VertexType* vp)
    {
      return vp->P() - (v->N() * ((vp->P() - v->P()) * v->N()));
    }


    static std::vector<CoordType> computeReferenceFrames(VertexTypeP vi)
    {
      vcg::face::VFIterator<FaceType> vfi(vi);

      int i = (vfi.I()+1)%3;
      VertexTypeP vp = vfi.F()->V(i);

      CoordType x = (project(&*vi,vp) - vi->P()).Normalize();
      //assert(fabs(x * vi->N()) < 0.1);

      std::vector<CoordType> res(3);

      res[0] = x;
      res[1] = (vi->N() ^ res[0]).Normalize();
      res[2] = (vi->N())/(vi->N()).Norm();

      return res;
    }

    static std::set<CoordType> getSecondRing(VertexTypeP v)
    {
        std::set<VertexTypeP> ris;
        std::set<CoordType> coords;

        vcg::face::VFIterator<FaceType> vvi(v);
        for(;!vvi.End();++vvi)
        {
            vcg::face::VFIterator<FaceType> vvi2(vvi.F()->V((vvi.I()+1)%3));
            for(;!vvi2.End();++vvi2)
            {
                ris.insert(vvi2.F()->V((vvi2.I()+1)%3));
            }
        }
				typename std::set<VertexTypeP>::iterator it;
        for(it = ris.begin(); it != ris.end(); ++it)
            coords.insert((*it)->P());

        return coords;
    }

    static Quadric fitQuadric(VertexTypeP v, std::vector<CoordType>& ref)
    {
        std::set<CoordType> ring = getSecondRing(v);
        if (ring.size() < 5)
            return Quadric(1,1,1,1,1);
        std::vector<CoordType> points;

        typename std::set<CoordType>::iterator b = ring.begin();
        typename std::set<CoordType>::iterator e = ring.end();

        while(b != e)
        {
            // vtang non e` il v tangente!!!
            CoordType vTang = *b - v->P();

            double x = vTang * ref[0];
            double y = vTang * ref[1];
            double z = vTang * ref[2];
            points.push_back(CoordType(x,y,z));
            ++b;
        }
        return Quadric::fit(points);
    }


    static void computeCurvature(MeshType & m)
    {
      Allocator<MeshType>::CompactVertexVector(m);

      if(!HasFVAdjacency(m)) throw vcg::MissingComponentException("FVAdjacency");

        vcg::tri::UpdateTopology<MeshType>::VertexFace(m);

        vcg::tri::UpdateNormal<MeshType>::PerVertexAngleWeighted(m);
        vcg::tri::UpdateNormal<MeshType>::NormalizePerVertex(m);


        VertexIterator vi;
        for(vi = m.vert.begin(); vi!=m.vert.end(); ++vi )
        {
            std::vector<CoordType> ref = computeReferenceFrames(&*vi);

            Quadric q = fitQuadric(&*vi,ref);
            double a = q.a();
            double b = q.b();
            double c = q.c();
            double d = q.d();
            double e = q.e();

            double E = 1.0 + d*d;
            double F = d*e;
            double G = 1.0 + e*e;

            CoordType n = CoordType(-d,-e,1.0).Normalize();

            vi->N() = ref[0] * n[0] + ref[1] * n[1] + ref[2] * n[2];

            double L = 2.0 * a * n.Z();
            double M = b * n.Z();
            double N = 2 * c * n.Z();

            // ----------------- Eigen stuff
            Eigen::Matrix2d m;
            m << L*G - M*F, M*E-L*F, M*E-L*F, N*E-M*F;
            m = m / (E*G-F*F);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(m);

            Eigen::Vector2d c_val = eig.eigenvalues();
            Eigen::Matrix2d c_vec = eig.eigenvectors();

            c_val = -c_val;

            CoordType v1, v2;
            v1[0] = c_vec(0,0);
            v1[1] = c_vec(0,1);
            v1[2] = 0;

            v2[0] = c_vec(1,0);
            v2[1] = c_vec(1,1);
            v2[2] = 0;

            v1 = v1.Normalize();
            v2 = v2.Normalize();

            v1 = v1 * c_val[0];
            v2 = v2 * c_val[1];

            CoordType v1global = ref[0] * v1[0] + ref[1] * v1[1] + ref[2] * v1[2];
            CoordType v2global = ref[0] * v2[0] + ref[1] * v2[1] + ref[2] * v2[2];

            v1global.Normalize();
            v2global.Normalize();

            if (c_val[0] > c_val[1])
            {
                (*vi).PD1() = v1global;
                (*vi).PD2() = v2global;
                (*vi).K1()  = c_val[0];
                (*vi).K2()  = c_val[1];
            }
            else
            {
                (*vi).PD1() = v2global;
                (*vi).PD2() = v1global;
                (*vi).K1()  = c_val[1];
                (*vi).K2()  = c_val[0];
            }
            // ---- end Eigen stuff
        }
    }

    // GG LOCAL CURVATURE

    class QuadricLocal
    {
    public:

        QuadricLocal ()
        {
            a() = b() = c() = d() = e() = 1.0;
        }

        QuadricLocal (double av, double bv, double cv, double dv, double ev)
        {
            a() = av;
            b() = bv;
            c() = cv;
            d() = dv;
            e() = ev;
        }

        double& a() { return data[0];}
        double& b() { return data[1];}
        double& c() { return data[2];}
        double& d() { return data[3];}
        double& e() { return data[4];}

        double data[5];

        double evaluate(double u, double v)
        {
            return a()*u*u + b()*u*v + c()*v*v + d()*u + e()*v;
        }

        double du(double u, double v)
        {
            return 2.0*a()*u + b()*v + d();
        }

        double dv(double u, double v)
        {
            return 2.0*c()*v + b()*u + e();
        }

        double duv(double /*u*/, double /*v*/)
        {
            return b();
        }

        double duu(double /*u*/, double /*v*/)
        {
            return 2.0*a();
        }

        double dvv(double /*u*/, double /*v*/)
        {
            return 2.0*c();
        }


        static QuadricLocal fit(std::vector<CoordType> &VV, bool svdRes, bool detCheck)
        {
            assert(VV.size() >= 5);
            Eigen::MatrixXd A(VV.size(),5);
            Eigen::MatrixXd b(VV.size(),1);
            Eigen::MatrixXd sol(5,1);

            for(unsigned int c=0; c < VV.size(); ++c)
            {
                double u = VV[c].X();
                double v = VV[c].Y();
                double n = VV[c].Z();

                A(c,0) = u*u;
                A(c,1) = u*v;
                A(c,2) = v*v;
                A(c,3) = u;
                A(c,4) = v;

                b[c] = n;
            }


            static int count = 0, index = 0;
            double min = 0.000000000001; //1.0e-12
            /*
            if (!count)
                printf("GNE %e\n", min);
             */

            if (detCheck && ((A.transpose()*A).determinant() < min && (A.transpose()*A).determinant() > -min))
            {
                //A.svd().solve(b, &sol); A.svd().solve(b, &sol);
                //cout << sol << endl;
                printf("Quadric: unsolvable vertex %d %d\n", count, ++index);
                //return Quadric (1, 1, 1, 1, 1);
//                A.svd().solve(b, &sol);
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(A);
                sol=svd.solve(b);
                return QuadricLocal(sol[0],sol[1],sol[2],sol[3],sol[4]);
            }
            count++;

            //for (int i = 0; i < 100; i++)
            {
                if (svdRes)
                {
                  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A);
                  sol=svd.solve(b);
                  //A.svd().solve(b, &sol);
                }
                else
                    sol = ((A.transpose()*A).inverse()*A.transpose())*b;

            }

            return QuadricLocal(sol[0],sol[1],sol[2],sol[3],sol[4]);
        }
    };

    static void expandMaxLocal (MeshType & mesh, VertexType *v, int max, std::vector<VertexType*> *vv)
    {
	Nring<MeshType> rw = Nring<MeshType> (v, &mesh);
	do rw.expand (); while (rw.allV.size() < max+1);
	if (rw.allV[0] != v)
	    printf ("rw.allV[0] != *v\n");
	vv->reserve ((size_t)max);
	for (int i = 1; i < max+1; i++)
	    vv->push_back(rw.allV[i]);

	rw.clear();
    }


    static void expandSphereLocal (MeshType & mesh, VertexType *v, float r, int min, std::vector<VertexType*> *vv)
    {
	Nring<MeshType> rw = Nring<MeshType> (v, &mesh);

	bool isInside = true;
	while (isInside)
	{
	    rw.expand();
	    vv->reserve(rw.allV.size());

	    typename std::vector<VertexType*>::iterator b = rw.lastV.begin();
	    typename std::vector<VertexType*>::iterator e = rw.lastV.end();
	    isInside = false;
	    while(b != e)
	    {
		if (((*b)->P() - v->P()).Norm() < r)
		{
		    vv->push_back(*b);;
		    isInside = true;
		}
		++b;
	    }
	}
	//printf ("%d\n", vv->size());
	rw.clear();

	if (vv->size() < min)
	{
	    vv->clear();
	    expandMaxLocal (mesh, v, min, vv);
	}
    }


    static void getAverageNormal (VertexType *vp, std::vector<VertexType*> &vv, CoordType *ppn)
    {
	*ppn = CoordType (0,0,0);
	for (typename std::vector<VertexType*>::iterator vpi = vv.begin(); vpi != vv.end(); ++vpi)
	    *ppn += (*vpi)->N();
	*ppn += (*vp).N();
	*ppn /= vv.size() + 1;
	ppn->Normalize();
    }


    static void applyProjOnPlane (CoordType ppn, std::vector<VertexType*> &vin, std::vector<VertexType*> *vout)
    {
	for (typename std::vector<VertexType*>::iterator vpi = vin.begin(); vpi != vin.end(); ++vpi)
	    if ((*vpi)->N() * ppn > 0.0f)
		vout->push_back (*vpi);
    }

    static CoordType projectLocal(VertexType* v, VertexType* vp, CoordType ppn)
    {
        return vp->P() - (ppn * ((vp->P() - v->P()) * ppn));
    }


    static void computeReferenceFramesLocal (VertexType *v, CoordType ppn, std::vector<CoordType> *ref)
    {
	vcg::face::VFIterator<FaceType> vfi (v);

	int i = (vfi.I() + 1) % 3;
	VertexTypeP vp = vfi.F()->V(i);

	CoordType x = (projectLocal (v, vp, ppn) - v->P()).Normalize();

	assert(fabs(x * ppn) < 0.1);

	*ref = std::vector<CoordType>(3);

	(*ref)[0] = x;
	(*ref)[1] = (ppn ^ (*ref)[0]).Normalize();
	(*ref)[2] = ppn.Normalize(); //ppn / ppn.Norm();
    }


    static void fitQuadricLocal (VertexType *v, std::vector<CoordType> ref, std::vector<VertexType*> &vv, QuadricLocal *q)
    {
	bool svdResolution = false;
	bool zeroDeterminantCheck = false;
	
	std::vector<CoordType> points;
	points.reserve (vv.size());

	typename std::vector<VertexType*>::iterator b = vv.begin();
	typename std::vector<VertexType*>::iterator e = vv.end();

	while(b != e)
	{
	    CoordType cp = (*b)->P();

	    // vtang non e` il v tangente!!!
	    CoordType vTang = cp - v->P();

	    double x = vTang * ref[0];
	    double y = vTang * ref[1];
	    double z = vTang * ref[2];
	    points.push_back(CoordType(x,y,z));
	    ++b;
	}
	
	*q = QuadricLocal::fit (points, svdResolution, zeroDeterminantCheck);
    }


    static void finalEigenStuff (VertexType *v, std::vector<CoordType> ref, QuadricLocal q)
    {
	double a = q.a();
	double b = q.b();
	double c = q.c();
	double d = q.d();
	double e = q.e();

	double E = 1.0 + d*d;
	double F = d*e;
	double G = 1.0 + e*e;

	CoordType n = CoordType(-d,-e,1.0).Normalize();

	v->N() = ref[0] * n[0] + ref[1] * n[1] + ref[2] * n[2];

	double L = 2.0 * a * n.Z();
	double M = b * n.Z();
	double N = 2 * c * n.Z();

	// ----------------- Eigen stuff
	Eigen::Matrix2d m;
	m << L*G - M*F, M*E-L*F, M*E-L*F, N*E-M*F;
	m = m / (E*G-F*F);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(m);

	Eigen::Vector2d c_val = eig.eigenvalues();
	Eigen::Matrix2d c_vec = eig.eigenvectors();

	c_val = -c_val;

	CoordType v1, v2;
	v1[0] = c_vec[0];
	v1[1] = c_vec[1];
	v1[2] = d * v1[0] + e * v1[1];

	v2[0] = c_vec[2];
	v2[1] = c_vec[3];
	v2[2] = d * v2[0] + e * v2[1];

	v1 = v1.Normalize();
	v2 = v2.Normalize();

	CoordType v1global = ref[0] * v1[0] + ref[1] * v1[1] + ref[2] * v1[2];
	CoordType v2global = ref[0] * v2[0] + ref[1] * v2[1] + ref[2] * v2[2];

	v1global.Normalize();
	v2global.Normalize();

	v1global *= c_val[0];
	v2global *= c_val[1];

	if (c_val[0] > c_val[1])
	{
	    (*v).PD1() = v1global;
	    (*v).PD2() = v2global;
	    (*v).K1()  = c_val[0];
	    (*v).K2()  = c_val[1];
	}
	else
	{
	    (*v).PD1() = v2global;
	    (*v).PD2() = v1global;
	    (*v).K1()  = c_val[1];
	    (*v).K2()  = c_val[0];
	}
	// ---- end Eigen stuff
    }



    static void updateCurvatureLocal (MeshType & mesh, float radiusSphere)
    {
	bool verbose = false;
	bool projectionPlaneCheck = true;
	int vertexesPerFit = 0;

	int i = 0;
	VertexIterator vi;
	for(vi = mesh.vert.begin(); vi != mesh.vert.end(); ++vi, i++)
	{
	    std::vector<VertexType*> vv;
	    std::vector<VertexType*> vvtmp;

	    int count;
	    if (verbose && !((count = (vi - mesh.vert.begin())) % 1000))
		printf ("vertex %d of %d\n",count,mesh.vert.size());

	    // if (kRing != 0)
	    // 	expandRing (&*vi, kRing, 5, &vv);
	    // else
	    expandSphereLocal (mesh, &*vi, radiusSphere, 5, &vv);

	    assert (vv.size() >= 5);

	    CoordType ppn;
	    // if (averageNormalMode)
	    // 	//ppn = (*vi).N();
	    getAverageNormal (&*vi, vv, &ppn);
	    // else
	    // 	getProjPlaneNormal (&*vi, vv, &ppn);

	    if (projectionPlaneCheck)
	    {
		vvtmp.reserve (vv.size ());
		applyProjOnPlane (ppn, vv, &vvtmp);
		if (vvtmp.size() >= 5)
		    vv = vvtmp;
	    }

	    vvtmp.clear();

	    // if (montecarloMaxVertexNum)
	    // {
	    // 	//printf ("P: %d\n", vv.size());
	    // 	vvtmp.reserve (vv.size ());
	    // 	//printf ("TP: %d\n", vvtmp.size());
	    // 	applyMontecarlo (montecarloMaxVertexNum, vv, &vvtmp);
	    // 	//printf ("TD: %d\n", vvtmp.size());
	    // 	vv = vvtmp;
	    // 	//printf ("D: %d\n", vv.size());
	    // 	//printf ("\n");
	    // }

	    assert (vv.size() >= 5);

	    std::vector<CoordType> ref;
	    computeReferenceFramesLocal (&*vi, ppn, &ref);

	    /*
	      printf ("%lf %lf %lf - %lf %lf %lf - %lf %lf %lf\n",
	      ref[0][0], ref[0][1], ref[0][2],
	      ref[1][0], ref[1][1], ref[1][2],
	      ref[2][0], ref[2][1], ref[2][2]);
	    */

	    vertexesPerFit += vv.size();
	    //printf ("size: %d\n", vv.size());

	    QuadricLocal q;
	    fitQuadricLocal (&*vi, ref, vv, &q);

	    finalEigenStuff (&*vi, ref, q);

	}

	//if (verbose)
        //printf ("average vertex num in each fit: %f, total %d, vn %d\n", ((float) vertexesPerFit) / mesh.vn, vertexesPerFit, mesh.vn);
	if (verbose)
	    printf ("average vertex num in each fit: %f\n", ((float) vertexesPerFit) / mesh.vn);
    }

};

}
}
#endif
