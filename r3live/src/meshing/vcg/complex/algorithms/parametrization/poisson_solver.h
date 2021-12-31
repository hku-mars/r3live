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

#ifndef VCG_POISSON_SOLVER
#define VCG_POISSON_SOLVER

#include <eigenlib/Eigen/Sparse>

#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/parametrization/distortion.h>
#include <vcg/complex/algorithms/parametrization/uv_utils.h>

namespace vcg {
namespace tri{
template <class MeshType>
class PoissonSolver
{

    typedef typename MeshType::ScalarType ScalarType;
    typedef typename MeshType::FaceType FaceType;
    typedef typename MeshType::VertexType VertexType;
    typedef typename MeshType::CoordType CoordType;
    typedef typename MeshType:: template PerFaceAttributeHandle<CoordType> PerFaceCoordHandle;

    ///the mesh itself
    MeshType &mesh;

    ///solver data

    std::map<VertexType*,int> VertexToInd;
    std::map<int, VertexType*> IndToVertex;

    ///vertices to fix
    std::vector<VertexType *> to_fix;

    ///unknown vector

    Eigen::SparseMatrix<double> A; // A
    Eigen::VectorXd b,x;// x and b

    //number of variables
    unsigned int n_vert_vars;
    ///total system size
    unsigned int total_size;
    ///number of fixed variables
    unsigned int n_fixed_vars;

    ///if you intend to follow the cross field
    bool use_direction_field,fix_selected,correct_fixed;
    ///size of the scalar field
    ScalarType fieldScale;
    ///handle per direction field
    PerFaceCoordHandle Fh0,Fh1;

    int VertexIndex(VertexType* v)
    {
        typename std::map<VertexType*,int>::iterator iteMap=VertexToInd.find(v);
        assert(iteMap!=VertexToInd.end());
        return ((*iteMap).second);
    }

    VertexType* IndexVertex(int index)
    {
        typename std::map<int,VertexType*>::iterator iteMap=IndToVertex.find(index);
        assert(iteMap!=IndToVertex.end());
        return ((*iteMap).second);
    }

    void AddVertexIndex(VertexType* v,int index)
    {
        VertexToInd.insert(std::pair<VertexType*,int>(v,index));
        IndToVertex.insert(std::pair<int,VertexType*>(index,v));
    }
    ///set the value of A of the system Ax=b
    void SetValA(int Xindex,int Yindex,ScalarType val)
    {
        //int size=(int)S.nrows();
        assert(0 <= Xindex && Xindex < int(total_size));
        assert(0 <= Yindex && Yindex < int(total_size));
        //S.A().addEntryReal(Xindex,Yindex,val);
        //if (Xindex>=Yindex)
        A.coeffRef(Xindex,Yindex) +=val;

    }


    void FindFarthestVert(VertexType* &v0,VertexType* &v1)
    {
        UpdateBounding<MeshType>::Box(mesh);

        tri::UpdateTopology<MeshType>::FaceFace(mesh);
        tri::UpdateFlags<MeshType>::FaceBorderFromFF(mesh);
        tri::UpdateFlags<MeshType>::VertexBorderFromFace(mesh);

        ScalarType dmax=0;
        v0=NULL;
        v1=NULL;
        for (unsigned int i=0;i<mesh.vert.size();i++)
            for (unsigned int j=(i+1);j<mesh.vert.size();j++)
            {
                VertexType *vt0=&mesh.vert[i];
                VertexType *vt1=&mesh.vert[j];
                if (vt0->IsD())continue;
                if (vt1->IsD())continue;
                if (!vt0->IsB())continue;
                if (!vt1->IsB())continue;
                ScalarType d_test=(vt0->P()-vt1->P()).Norm();
//                ScalarType Dx=fabs(vt0->P().X()-vt1->P().X());
//                ScalarType Dy=fabs(vt0->P().Y()-vt1->P().Y());
//                ScalarType Dz=fabs(vt0->P().Z()-vt1->P().Z());

                //ScalarType d_test=std::max(Dx,std::max(Dy,Dz));
                //ScalarType d_test=std::max(fabs(Dx-Dy),std::max(fabs(Dx-Dz),fabs(Dy-Dz)));
                if (d_test>dmax)
                {
                    dmax=d_test;
                    v0=vt0;
                    v1=vt1;
                }
            }
        assert(v0!=NULL);
        assert(v1!=NULL);
    }

    ///set the value of b of the system Ax=b
    void SetValB(int Xindex,
                 ScalarType val)
    {
        /*S.b()[Xindex] += val;*/
        b[Xindex] += val;
    }

    ///add the area term, scalefactor is used to sum up
    ///and normalize on the overlap zones
    void AddAreaTerm(int index[3][3][2],ScalarType ScaleFactor)
    {
        const ScalarType entry=0.5*ScaleFactor;
        ScalarType val[3][3]= { {0,  entry, -entry},
                                {-entry,  0,  entry},
                                {entry, -entry,  0} };

        for (int i=0;i<3;i++)
            for (int j=0;j<3;j++)
            {
                ///add for both u and v
                int Xindex=index[i][j][0]*2;
                int Yindex=index[i][j][1]*2;

                SetValA(Xindex+1,Yindex,-val[i][j]);
                SetValA(Xindex,Yindex+1,val[i][j]);

            }
    }

    ///set the diagonal of the matrix (which is zero at the beginning)
    ///as the sum of the other element inverted by sign
    void SetDiagonal(ScalarType val[3][3])
    {
        for (int i=0;i<3;i++)
        {
            ScalarType sum=0;
            for (int j=0;j<3;j++)
                sum+=val[i][j];
            val[i][i]=-sum;
        }
    }

    ///add this values to the right hand side
    void AddRHS(ScalarType b[6],
                int index[3])
    {
        for (int i=0;i<3;i++)
        {
            ScalarType valU=b[i*2];
            ScalarType valV=b[(i*2)+1];
            SetValB((index[i]*2),valU);
            SetValB((index[i]*2)+1,valV);
        }
    }

    ///add a 3x3 block matrix to the system matrix...
    ///indexes are specified in the 3x3 matrix of x,y pairs
    ///indexes must be multiplied by 2 cause u and v
    void Add33Block(ScalarType val[3][3],int index[3][3][2])
    {
        for (int i=0;i<3;i++)
            for (int j=0;j<3;j++)
            {
                ///add for both u and v
                int Xindex=index[i][j][0]*2;
                int Yindex=index[i][j][1]*2;
                assert(Xindex<int(n_vert_vars*2));
                assert(Yindex<int(n_vert_vars*2));
                SetValA(Xindex,Yindex,val[i][j]);
                SetValA(Xindex+1,Yindex+1,val[i][j]);
            }

    }

    ///add a 3x3 block matrix to the system matrix...
    ///indexes are specified in the 3x3 matrix of x,y pairs
    ///indexes must be multiplied by 2 cause u and v
    void Add44Block(ScalarType val[4][4],int index[4][4][2])
    {
        for (int i=0;i<4;i++)
            for (int j=0;j<4;j++)
            {
                ///add for both u and v
                int Xindex=index[i][j][0]*2;
                int Yindex=index[i][j][1]*2;
                assert(Xindex<(n_vert_vars*2));
                assert(Yindex<(n_vert_vars*2));
                SetValA(Xindex,Yindex,val[i][j]);
                SetValA(Xindex+1,Yindex+1,val[i][j]);
            }

    }

    ///return the LHS for a given face
    void perElementLHS(FaceType *f,
                       ScalarType val[3][3],
                       int index[3][3][2])
    {
        ///initialize to zero
        for (int x=0;x<3;x++)
            for (int y=0;y<3;y++)
                val[x][y]=0;

        ///get the vertices
        VertexType *v[3];
        v[0]=f->V(0);
        v[1]=f->V(1);
        v[2]=f->V(2);

        ///get the indexes of vertex instance (to consider cuts)
        ///for the current face
        int Vindexes[3];
        Vindexes[0]=VertexIndex(f->V(0));
        Vindexes[1]=VertexIndex(f->V(1));
        Vindexes[2]=VertexIndex(f->V(2));

        ///initialize the indexes for the block
        for (int x=0;x<3;x++)
            for (int y=0;y<3;y++)
            {
                index[x][y][0]=Vindexes[x];
                index[x][y][1]=Vindexes[y];
            }

        ///initialize edges
        CoordType e[3];
        for (int k=0;k<3;k++)
            e[k]=v[(k+2)%3]->P()-v[(k+1)%3]->P();

        ///then consider area but also considering scale factor dur to overlaps
        ScalarType areaT=((f->P(1)-f->P(0))^(f->P(2)-f->P(0))).Norm()/2.0;
        for (int x=0;x<3;x++)
            for (int y=0;y<3;y++)
                if (x!=y)
                {
                    ScalarType num=(e[x]*e[y]);
                    val[x][y] =num/(4.0*areaT);
                }

        ///set the matrix as diagonal
        SetDiagonal(val);
    }

    ///return the RHS for a given face
    void perElementRHS(FaceType *f,
                       ScalarType b[6],
                       ScalarType vector_field_scale=1)
    {

        /// then set the rhs
        CoordType scaled_Kreal;
        CoordType scaled_Kimag;
        CoordType fNorm=f->N();
        fNorm.Normalize();
        CoordType p[3];
        p[0]=f->P0(0);
        p[1]=f->P0(1);
        p[2]=f->P0(2);

        CoordType neg_t[3];
        neg_t[0] = fNorm ^ (p[2] - p[1]);
        neg_t[1] = fNorm ^ (p[0] - p[2]);
        neg_t[2] = fNorm ^ (p[1] - p[0]);

        CoordType K1,K2;
        /*MyMesh::PerFaceCoordHandle<ScalarType> Fh = tri::Allocator<MyMesh>::AddPerVertexAttribute<float>  (m,std::string("Irradiance"));
        bool CrossDir0 = tri::HasPerVertexAttribute(mesh,"CrossDir0");
                bool CrossDir1 = tri::HasPerVertexAttribute(mesh,"CrossDir1");
                assert(CrossDir0);
                assert(CrossDir1);*/

        //K1=f->Q3();
        K1=Fh0[f];
        K1.Normalize();
        //K2=fNorm^K1;
        K2=Fh1[f];
        K2.Normalize();

        scaled_Kreal = K1*(vector_field_scale);///2);
        scaled_Kimag = K2*(vector_field_scale);///2);

        b[0] = scaled_Kreal * neg_t[0];
        b[1] = scaled_Kimag * neg_t[0];
        b[2] = scaled_Kreal * neg_t[1];
        b[3] = scaled_Kimag * neg_t[1];
        b[4] = scaled_Kreal * neg_t[2];
        b[5] = scaled_Kimag * neg_t[2];
        ////fine codice mio
    }

    ///return the LHS and RHS for a given face
    void PerElementSystemReal(FaceType *f,
                              ScalarType val[3][3],
                              int index[3][3][2],
                              ScalarType b[6],
                              ScalarType vector_field_scale=1.0)
    {
        perElementLHS(f,val,index);

        if (use_direction_field)
            perElementRHS(f,b,vector_field_scale);
    }

    void FixPointLSquares()
    {
        ScalarType penalization=1000;
        int offset_row=n_vert_vars;
        assert(to_fix.size()>0);
        for (size_t i=0;i<to_fix.size();i++)
        {
            ///take a vertex
            VertexType *v=to_fix[i];
            assert(!v->IsD());
            int index=VertexIndex(v);
            //v->vertex_index[0];
            int indexvert=index*2;
            int indexRow=(offset_row+i)*2;

            SetValA(indexRow,indexRow,penalization);
            SetValA(indexRow+1,indexRow+1,penalization);

            ///add values to the B vector
            ScalarType U=v->T().U()*penalization;
            ScalarType V=v->T().V()*penalization;
            SetValB(indexRow,U);
            SetValB(indexRow+1,V);

            /*///set upper right part
            SetValA(indexvert,indexCol,penalization);
            SetValA(indexvert+1,indexCol+1,penalization);*/

            SetValA(indexvert,indexvert,penalization);
            SetValA(indexvert+1,indexvert+1,penalization);
            SetValA(indexRow,indexRow,penalization);
            SetValA(indexRow+1,indexRow+1,penalization);
            SetValA(indexvert,indexRow,-penalization);
            SetValA(indexvert+1,indexRow+1,-penalization);
            SetValA(indexRow,indexvert,-penalization);
            SetValA(indexRow+1,indexvert+1,-penalization);
            //SetValA(indexCol+1,indexCol+1,-1);
        }
    }

    //build the laplacian matrix cyclyng over all rangemaps
    //and over all faces
    void BuildLaplacianMatrix(double vfscale=1)
    {

        ///then for each face
        for (unsigned int j=0;j<mesh.face.size();j++)
        {

            FaceType *f=&mesh.face[j];
            if (f->IsD())
                continue;

            int var_idx[3];//vertex variable indices
            for(int k = 0; k < 3; ++k)
            {
                VertexType *v=f->V(k);
                var_idx[k] = VertexIndex(v);
            }
            ScalarType val[3][3];
            int index[3][3][2];
            ScalarType b[6];
            PerElementSystemReal(f, val,index, b, vfscale);

            //Add the element to the matrix
            Add33Block(val,index);

            /////add area term.. to test if needed
            /*if (!use_direction_field)
                AddAreaTerm(index,1.0);//f->area);*/
            /*ScalarType area=((f->P(1)-f->P(0))^(f->P(2)-f->P(0))).Norm();
            if (!use_direction_field)
                AddAreaTerm(index,area);*/

            //ScalarType area=((f->P(1)-f->P(0))^(f->P(2)-f->P(0))).Norm();
            if (!use_direction_field)
                AddAreaTerm(index,1);

            ///add right hand side
            if (use_direction_field)
                AddRHS(b,var_idx);
        }
    }


    void FindSizes()
    {
        // tag vertices and compute numbers of equations to determine the number of rows in the matrix
        //TagVertices_Constrained(n_vert_vars, n_transition_eqs, n_align_sharp_eqs);
        n_vert_vars=mesh.vn;

        ///initialize matrix size
        total_size = (n_fixed_vars + n_vert_vars)*2;///must be multiplied by 2 becasue of u and v

    }

    void AllocateSystem()
    {
        //--- Allocates the data for Ax=b
        A=Eigen::SparseMatrix<double>(total_size, total_size); // A
        b = Eigen::VectorXd::Zero(total_size);  // x and b
    }



    ///intitialize the whole matrix
    void InitMatrix()
    {
        FindSizes();
        AllocateSystem();
    }

    bool Solve()
    {
        //return true;
        A.finalize();
        Eigen::SparseMatrix<double> As=Eigen::SparseMatrix<double>(A);
        As.finalize();

        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double> > solver(As);
        x = solver.solve(b);
        return (solver.info()==Eigen::Success);
    }


    void InitIndex()
    {
        for (size_t i=0;i<mesh.vert.size();i++)
            if (!mesh.vert[i].IsD())
                AddVertexIndex(&mesh.vert[i],i);
    }

    ///map back values to vertex
    ///if normalize==true then set the
    ///coordinates between 0 and 1
    void MapCoords(bool normalize=false,
                   ScalarType /*fieldScale*/=1.0)
    {
        ///clear Visited Flag
        if (correct_fixed)
            tri::UpdateFlags<MeshType>::VertexClearV(mesh);
        //set fixed to V
        for (size_t i=0;i<to_fix.size();i++)
            to_fix[i]->SetV();

        Box2<ScalarType> bbox;
        if (normalize)
        {
            for (size_t i=0;i<n_vert_vars;i++)
            {
                ScalarType U=x[i*2];
                ScalarType V=x[(i*2)+1];
                bbox.Add(Point2<ScalarType>(U,V));
            }
        }

        //for each vertex
        for (size_t i=0;i<n_vert_vars;i++)
        {
            VertexType* v=IndexVertex(i);
            //take U and V
            ScalarType U=x[i*2];
            ScalarType V=x[(i*2)+1];
            Point2<ScalarType> p;
            if (!v->IsV())
                p=Point2<ScalarType>(U,V);
            else
                p=v->T().P();
            //p/=fieldScale;
            if  (normalize)
            {
                p-=bbox.min;
                p*=1/bbox.Diag();
            }

            v->T().P()=p;
        }

        ///then copy to faces
        for (size_t i=0;i<mesh.face.size();i++)
        {
            FaceType *f=&mesh.face[i];
            for (int j=0;j<3;j++)
            {
                VertexType* v=f->V(j);
                Point2<ScalarType> p=v->T().P();
                f->WT(j).P()=p;
            }
        }
    }

public:

    ///return true if is possible to
    bool IsFeaseable()
    {
        tri::UpdateTopology<MeshType>::FaceFace(mesh);
        int NNmanifoldE=tri::Clean<MeshType>::CountNonManifoldEdgeFF(mesh);
        if (NNmanifoldE!=0)
        {
            printf("Non Manifold");
            return false;
        }
        /*int NNmanifoldV=tri::Clean<MeshType>::CountNonManifoldVertexFF(mesh);
        if (NNmanifoldV!=0)return false;*/
        int G=tri::Clean<MeshType>::MeshGenus(mesh);
        int numholes=tri::Clean<MeshType>::CountHoles(mesh);
        if (numholes==0)
        {
            printf("Non omeomorph to a disc");
            return false;
        }
        return (G==0);
    }

    ///set the border as fixed
    void SetBorderAsFixed()
    {
        for (size_t i=0;i<mesh.vert.size();i++)
        {
            VertexType* v=&mesh.vert[i];
            if (v->IsD())continue;
            if(v->IsB())to_fix.push_back(v);
        }
        std::sort(to_fix.begin(),to_fix.end());
        typename std::vector<VertexType*>::iterator new_end=std::unique(to_fix.begin(),to_fix.end());
        int dist=distance(to_fix.begin(),new_end);
        to_fix.resize(dist);
    }

    ///set selected vertices as fixed
    void SetSelectedAsFixed()
    {
        for (int i=0;i<mesh.vert.size();i++)
        {
            VertexType* v=&mesh.vert[i];
            if (v->IsD())continue;
            if(v->IsS())to_fix.push_back(v);
        }
        std::sort(to_fix.begin(),to_fix.end());
        typename std::vector<VertexType*>::iterator new_end=std::unique(to_fix.begin(),to_fix.end());
        int dist=distance(to_fix.begin(),new_end);
        to_fix.resize(dist);
    }


    ///fix default vertices no need if already border on other vertices are fixed
    ///you need at least 2 fixed for solving without field ,
    ///while only 1 if you conforms to a given cross field
    void FixDefaultVertices()
    {
        ///in this case there are already vertices fixed, so no need to fix by default
        assert(to_fix.size()==0);
        ///then fix only one vertex
        if (use_direction_field)
        {
            for (size_t i=0;i<mesh.vert.size();i++)
                if (!mesh.vert[i].IsD())
                {
                    mesh.vert[i].T().P()=Point2<ScalarType>(0,0);
                    to_fix.push_back(&mesh.vert[i]);
                    return;
                }
        }
        ///then fix 2 vertices
        else
        {
            VertexType *v0;
            VertexType *v1;
            FindFarthestVert(v0,v1);
            if (v0==v1)
            {
                //				tri::io::ExporterPLY<MeshType>::Save(mesh,"./parametrized.ply");
                assert(0);
            }
            v0->T().P()=Point2<ScalarType>(0,0);
            v1->T().P()=Point2<ScalarType>(1,0);
            to_fix.push_back(v0);
            to_fix.push_back(v1);
            return;
        }
    }
    ///intialize parameters and setup fixed vertices vector
    void Init(bool _use_direction_field=false,
              bool _correct_fixed=true,
              ScalarType _fieldScale=1.0)
    {
        use_direction_field=_use_direction_field;
        //query if an attribute is present or not
        if (use_direction_field)
        {
            bool CrossDir0 = tri::HasPerFaceAttribute(mesh,"CrossDir0");
            bool CrossDir1 = tri::HasPerFaceAttribute(mesh,"CrossDir1");
            assert(CrossDir0);
            assert(CrossDir1);
            Fh0= tri::Allocator<MeshType> :: template GetPerFaceAttribute<CoordType>(mesh,std::string("CrossDir0"));
            Fh1= tri::Allocator<MeshType> :: template GetPerFaceAttribute<CoordType>(mesh,std::string("CrossDir1"));
        }
        correct_fixed=_correct_fixed;
        fieldScale=_fieldScale;
        to_fix.clear();
    }

    ///solve the system, it return false if the matrix is singular
    bool SolvePoisson(bool _write_messages=false,
                      ScalarType fieldScale=1.0,
                      bool solve_global_fold=true)
    {
        int t0,t1,t2,t3;

        ///Initializing Matrix
        if (_write_messages)
        {
            printf("\n INITIALIZING THE MATRIX \n");
            t0=clock();
        }

        ///set vertex indexes
        InitIndex();

        /*///find vertex to fix
        std::vector<VertexType *> to_fix;
        FindFixedVertices(to_fix);
        n_fixed_vars=to_fix.size();*/
        if (use_direction_field)
        {
            assert(to_fix.size()>0);
        }
        else
        {
            assert(to_fix.size()>1);
        }

        n_fixed_vars=to_fix.size();
        ///initialize the matrix ALLOCATING SPACE
        InitMatrix();

        if (use_direction_field)
        {
            bool CrossDir0 = tri::HasPerFaceAttribute(mesh,"CrossDir0");
            bool CrossDir1 = tri::HasPerFaceAttribute(mesh,"CrossDir1");
            assert(CrossDir0);
            assert(CrossDir1);
        }

        ///build the laplacian system
        BuildLaplacianMatrix(fieldScale);

        ////add the lagrange multiplier
        FixPointLSquares();

        if (_write_messages)
        {
            t1=clock();
            printf("\n time:%d \n",t1-t0);
            printf("\n SOLVING \n");
        }

        //int n_vars=(n_vert_vars)*2;
        //int integer_constr_size=(n_transition_vars+n_fixed_vars+n_bary_transition_vars)*2;
        //X=std::vector< double >(n_vars+n_fixed_vars*2);
        bool done=Solve();
        if (!done)
            return false;
        if (_write_messages)
        {
            t2=clock();
            printf("\n time:%d \n",t2-t1);
            printf("\n ASSIGNING COORDS \n");
        }

        MapCoords(false,fieldScale);
        if (_write_messages)
        {
            t3=clock();
            printf("\n time:%d \n",t3-t2);
        }

        ///then check if majority of faces are folded
        if (!solve_global_fold) return true;
        if (tri::Distortion<MeshType,false>::GloballyUnFolded(mesh))
        {
            tri::UV_Utils<MeshType>::GloballyMirrorX(mesh);
            bool isUnfolded = tri::Distortion<MeshType,false>::GloballyUnFolded(mesh);
            assert( ! isUnfolded);
        }
        return true;
    }

    PoissonSolver(MeshType &_mesh):mesh(_mesh)
    {
        assert(mesh.vert.size()>3);
        assert(mesh.face.size()>1);
    }


}; // end class
} //End Namespace Tri
} // End Namespace vcg
#endif
