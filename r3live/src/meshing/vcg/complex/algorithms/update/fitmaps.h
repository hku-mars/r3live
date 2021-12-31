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

#ifndef FITMAPS_H
#define FITMAPS_H

#include <vcg/math/histogram.h>

#include <vcg/simplex/face/jumping_pos.h>
#include <vcg/complex/algorithms/update/flag.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/curvature.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include "vcg/complex/algorithms/update/curvature_fitting.h"

#include <eigenlib/Eigen/Core>
#include <eigenlib/Eigen/QR>
#include <eigenlib/Eigen/LU>
#include <eigenlib/Eigen/SVD>

#include <vcg/complex/algorithms/nring.h>

#include <vcg/complex/algorithms/smooth.h>


using namespace Eigen;

namespace vcg { namespace tri {

template<class MeshType>
class Fitmaps
{
public:

    typedef typename MeshType::FaceType   FaceType;
    typedef typename MeshType::VertexType VertexType;
    typedef typename MeshType::ScalarType ScalarType;
    typedef typename MeshType::FaceIterator FaceIterator;
    typedef typename MeshType::VertexIterator VertexIterator;
    typedef typename MeshType::CoordType CoordType;

    typedef vcg::tri::Nring<MeshType> RingWalker;


    class Bicubic
    {
        public:

        Bicubic() {};

        Bicubic(vector<double>& input)
        {
            data = input;
            if (input.size() != 16)
            {
                assert(0);
            }
        }

//        (u3 u2 u 1)  (v3 v2 v 1)
//
//        a u3 v3
//        b u3 v2
//        c u3 v1
//        d u3 1
//        e u2 v3
//        f u2 v2
//        g u2 v1
//        h u2 1
//        i u1 v3
//        l u1 v2
//        m u1 v1
//        n u1 1
//        o 1 v3
//        p 1 v2
//        q 1 v1
//        r 1 1

        double& a() { return data[0];}
        double& b() { return data[1];}
        double& c() { return data[2];}
        double& d() { return data[3];}
        double& e() { return data[4];}
        double& f() { return data[5];}
        double& g() { return data[6];}
        double& h() { return data[7];}
        double& i() { return data[8];}
        double& l() { return data[9];}
        double& m() { return data[10];}
        double& n() { return data[11];}
        double& o() { return data[12];}
        double& p() { return data[13];}
        double& q() { return data[14];}
        double& r() { return data[15];}

        vector<double> data;

        double evaluate(double u, double v)
        {

            return
                    a() * u*u*u * v*v*v    +
                    b() * u*u*u * v*v      +
                    c() * u*u*u * v        +
                    d() * u*u*u            +
                    e() * u*u   * v*v*v    +
                    f() * u*u   * v*v      +
                    g() * u*u   * v        +
                    h() * u*u              +
                    i() * u     * v*v*v    +
                    l() * u     * v*v      +
                    m() * u     * v        +
                    n() * u                +
                    o() *         v*v*v    +
                    p() *         v*v      +
                    q() *         v        +
                    r();
        }


        double distanceRMS(std::vector<CoordType>& VV)
        {
            double error = 0;
            for(typename std::vector<CoordType>::iterator it = VV.begin(); it != VV.end(); ++it)
            {
                double u = it->X();
                double v = it->Y();
                double n = it->Z();

                double temp = evaluate(u,v);
                error += (n - temp)*(n - temp);
            }
            error /= (double) VV.size();
            return sqrt(error);
        }

        static Bicubic fit(std::vector<CoordType> VV)
        {
            assert(VV.size() >= 16);
            Eigen::MatrixXf A(VV.size(),16);
            Eigen::MatrixXf b(VV.size(),1);
            Eigen::MatrixXf sol(16,1);

            for(unsigned int c=0; c < VV.size(); ++c)
            {
                double u = VV[c].X();
                double v = VV[c].Y();
                double n = VV[c].Z();

                A(c,0)  = u*u*u * v*v*v;
                A(c,1)  = u*u*u * v*v;
                A(c,2)  = u*u*u * v;
                A(c,3)  = u*u*u;
                A(c,4)  = u*u   * v*v*v;
                A(c,5)  = u*u   * v*v;
                A(c,6)  = u*u   * v;
                A(c,7)  = u*u;
                A(c,8)  = u     * v*v*v;
                A(c,9)  = u     * v*v;
                A(c,10) = u     * v;
                A(c,11) = u;
                A(c,12) =         v*v*v;
                A(c,13) =         v*v;
                A(c,14) =         v;
                A(c,15) = 1;


                b[c] = n;
            }

            A.svd().solve(b, &sol);

            vector<double> r(16);

            for (int i=0; i < 16; ++i)
                r.at(i) = sol[i];

            return Bicubic(r);
        }
    };

    Fitmaps()
    {}

    class radSorter
    {
    public:

        radSorter(VertexType* v)
        {
            origin = v;
        }

        VertexType* origin;

        bool operator() (VertexType* v1, VertexType* v2)
        {
            return (v1->P() - origin->P()).SquaredNorm() < (v2->P() - origin->P()).SquaredNorm();
        }
    };

    float getMeanCurvature(VertexType* vp)
    {
        return (vp->K1() + vp->K2())/2.0;
    }

    static bool fitBicubicPoints(VertexType* v, std::vector<CoordType>& ref, Bicubic& ret, std::vector<CoordType>& points, std::vector<VertexType*>& ring)
    {
        points.clear();

        if (ring.size() < 16)
        {
            return false;
        }

        typename std::vector<VertexType*>::iterator b = ring.begin();
        typename std::vector<VertexType*>::iterator e = ring.end();

        while(b != e)
        {
            CoordType vT = (*b)->P() - v->P();

            double x = vT * ref[0];
            double y = vT * ref[1];
            double z = vT * ref[2];

            points.push_back(CoordType(x,y,z));
            ++b;
        }
        ret = Bicubic::fit(points);
        return true;
    }

    static double AverageEdgeLenght(MeshType& m)
    {
      double doubleA = 0;
      for (FaceIterator fi = m.face.begin();  fi!=m.face.end(); fi++) if (!fi->IsD()) {
        doubleA+=vcg::DoubleArea(*fi);
      }
      int nquads = m.fn / 2;
      return sqrt( doubleA/(2*nquads) );
    }

    static void computeMFitmap(MeshType& m, float perc, int ringMax = 50)
    {
        vcg::tri::UpdateCurvatureFitting<MeshType>::computeCurvature(m);
        vcg::tri::UpdateNormal<MeshType>::PerVertexAngleWeighted(m);

        vcg::tri::UpdateTopology<MeshType>::FaceFace(m);
        vcg::tri::UpdateTopology<MeshType>::VertexFace(m);

        vcg::tri::UpdateBounding<MeshType>::Box(m);

        int countTemp = 0;

        RingWalker::clearFlags(&m);

        for(VertexIterator it=m.vert.begin(); it!=m.vert.end();++it)
        {
            if ((countTemp++ % 100) == 0)
                cerr << countTemp << "/" << m.vert.size() << endl;

            RingWalker rw(&*it,&m);

            CoordType nor = it->N();

            float okfaces = 0;
            float flipfaces = 0;

            int count = 0;
            do
            {
                count++;
                rw.expand();
                for(unsigned i=0; i<rw.lastF.size();++i)
                {
                    CoordType vet1 = nor;
                    CoordType vet2 = rw.lastF[i]->N();

                    vet1.Normalize();
                    vet2.Normalize();


                    double scal = vet1 * vet2;
                    if ((scal) > 0)
                        okfaces +=   (vcg::DoubleArea(*rw.lastF[i]));
                    else
                        flipfaces += (vcg::DoubleArea(*rw.lastF[i]));
                }
            } while ((((flipfaces)/(okfaces + flipfaces))*100.0 < perc) && (count < ringMax));

            std::sort(rw.lastV.begin(),rw.lastV.end(),radSorter(&*it));

            it->Q() = ((*rw.lastV.begin())->P() - it->P()).Norm();
            rw.clear();

        }

        vcg::tri::Smooth<MeshType>::VertexQualityLaplacian(m,2);
    }

    static vector<VertexType*> gatherNeighsSurface(VertexType* vt, float sigma, MeshType& m)
    {
        vector<VertexType*> current;

        RingWalker rw(vt,&m);

        bool exit = false;

        do
        {
            rw.expand();

            exit = true;

            for(typename vector<VertexType*>::iterator it = rw.lastV.begin(); it != rw.lastV.end(); ++it)
            {
                if (((*it)->P() - vt->P()).Norm() < sigma)
                {
                    current.push_back(*it);
                    exit = false;
                }
            }

        } while (!exit);

        rw.clear();
        return current;
    }

    static void computeSFitmap(MeshType& m)//, float target = 1000)
    {

            vcg::tri::UpdateCurvatureFitting<MeshType>::computeCurvature(m);
            vcg::tri::UpdateNormal<MeshType>::PerVertexAngleWeighted(m);

            vcg::tri::UpdateTopology<MeshType>::FaceFace(m);
            vcg::tri::UpdateTopology<MeshType>::VertexFace(m);

            // update bounding box
            vcg::tri::UpdateBounding<MeshType>::Box(m);

            int countTemp = 0;

            double e = AverageEdgeLenght(m);

            int iteraz = 5; //2.0 * sqrt(m.vert.size()/target);

            for(VertexIterator it=m.vert.begin(); it!=m.vert.end();++it)
            {
                if ((countTemp++ % 100) == 0)
                    cerr << countTemp << "/" << m.vert.size() << endl;

                vector<float> oneX;


                for (int iteration = 0; iteration<iteraz; ++iteration)
                {
                    oneX.push_back((iteration+1)*(e));
                }

                std::vector<CoordType> ref(3);
                ref[0] = it->PD1();
                ref[1] = it->PD2();
                ref[2] = it->PD1() ^ it->PD2();

                ref[0].Normalize();
                ref[1].Normalize();
                ref[2].Normalize();

                Bicubic b;

                RingWalker::clearFlags(&m);

                std::vector<VertexType*> pointsGlobal = gatherNeighsSurface(&*it,oneX.at(iteraz-1),m);

                vector<float> onedimensional;

                for (int iteration = 0; iteration<iteraz; ++iteration)
                {
                    std::vector<VertexType*> points; // solo quelli nel raggio

                    std::vector<CoordType> projected; // come sopra ma in coord locali
                    for (typename std::vector<VertexType*>::iterator it2 = pointsGlobal.begin(); it2 != pointsGlobal.end(); ++it2)
                    {
                        if (((*it).P() - (*it2)->P()).Norm() < oneX.at(iteration))
                            points.push_back(*it2);
                    }

                    std::vector<VertexType*>& pointsFitting = points;


                    if (!fitBicubicPoints(&*it, ref, b, projected,pointsFitting))
                    {
                        onedimensional.push_back(0);
                    }
                    else
                    {
                        onedimensional.push_back(b.distanceRMS(projected));
                    }

                }


    //            // vecchio fit ax^4
                Eigen::MatrixXf Am(onedimensional.size(),1);
                Eigen::MatrixXf bm(onedimensional.size(),1);
                Eigen::MatrixXf sol(1,1);

                for(unsigned int c=0; c < onedimensional.size(); ++c)
                {
                    double x = oneX.at(c);

                    Am(c,0) = pow(x,4);
                    bm[c] = onedimensional[c];
                }

                Am.svd().solve(bm, &sol);

                it->Q() = pow((double)sol[0],0.25);

    //            // nuovo fit ax^4 + b
    //            Eigen::MatrixXf Am(onedimensional.size()+1,2);
    //            Eigen::MatrixXf bm(onedimensional.size()+1,1);
    //            Eigen::MatrixXf sol(2,1);
    //
    //            Am(0,0) = 0;
    //            Am(0,1) = 0;
    //            bm[0] = 0;
    //
    //            for(unsigned int c=0; c < onedimensional.size(); ++c)
    //            {
    //                double x = oneX.at(c);
    //
    //                Am(c,0) = pow(x,4);
    //                Am(c,1) = 1;
    //                bm[c] = onedimensional[c];
    //            }
    //
    //            //sol = ((Am.transpose()*Am).inverse()*Am.transpose())*bm;
    //            Am.svd().solve(bm, &sol);
    //
    //            cerr << "------" << sol[0] << " " << sol[1] << endl;
    //            if (sol[0] > 0)
    //                saliency[it] = pow((double)sol[0],0.25);
    //            else
    //                saliency[it] = 0;


            }

            vcg::tri::Smooth<MeshType>::VertexQualityLaplacian(m,1);
    }


    ~Fitmaps(){};

};

}} // END NAMESPACES

#endif // FITMAPS_H
