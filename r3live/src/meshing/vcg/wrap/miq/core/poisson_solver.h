#ifndef MIQ_POISSON_SOLVER
#define MIQ_POISSON_SOLVER

#include <gmm/gmm.h>
#include <ConstrainedSolver.hh>
#include <MISolver.hh>
#include <GMM_Tools.hh>

#include "auxmath.h"
#include "sparsesystemdata.h"
#include "vertex_indexing.h"

template <class MeshType>
class PoissonSolver
{
    typedef typename MeshType::ScalarType ScalarType;
    typedef typename MeshType::FaceType FaceType;
    typedef typename MeshType::VertexType VertexType;
    typedef typename MeshType::CoordType CoordType;
    //typedef VertexIndexing<MeshType> VertexIndexingType;
    //typedef typename VertexIndexingType::SeamInfo SeamInfo;

    typename MeshType::template PerFaceAttributeHandle<vcg::Point3i> HandleS_Index;
    typename MeshType::template PerVertexAttributeHandle<bool> Handle_Singular;
    typename MeshType::template PerFaceAttributeHandle<float> Handle_Stiffness;
    ///this handle for mesh
    typename  MeshType::template PerMeshAttributeHandle<MeshSystemInfo> Handle_SystemInfo;

	///range map data
    MeshType &mesh;

	///vertex indexing structure
    //VertexIndexingType &VIndex;

    ///solver data
	SparseSystemData S;

	///vector of unknowns
    std::vector< double > X;

	////REAL PART
	///number of fixed vertex
	unsigned int n_fixed_vars;
	///the number of REAL variables for vertices
	unsigned int n_vert_vars;
    ///total number of variables of the system,
    ///do not consider constraints, but consider integer vars
    unsigned int num_total_vars;

    ///the number of scalar variables
    //unsigned int n_scalar_vars;

	//////INTEGER PART
	///the total number of integer variables
	unsigned int n_integer_vars; 

	///CONSTRAINT PART
	///number of cuts constraints
    unsigned int num_cut_constraint;

    ///total number of constraints equations
    unsigned int num_constraint_equations;

	///total size of the system including constraints
	unsigned int system_size;

	///if you intend to make integer rotation 
	///and translations
	bool integer_jumps_bary;

	///vector of blocked vertices
	std::vector<VertexType*> Hard_constraints;

	///vector of indexes to round
	std::vector<int> ids_to_round;

    ///boolean that is true if rounding to integer is needed
    bool integer_rounding;

    ///START SYSTEM ACCESS METHODS
	///add an entry to the LHS
	void AddValA(int Xindex,
		int Yindex,
		ScalarType val)
	{
		int size=(int)S.nrows();
		assert(0 <= Xindex && Xindex < size);
		assert(0 <= Yindex && Yindex < size);
		S.A().addEntryReal(Xindex,Yindex,val);
	}

	///add a complex entry to the LHS
	void AddComplexA(int VarXindex,
		int VarYindex,
		Cmplx val)
	{
		int size=(int)S.nrows()/2;
		assert(0 <= VarXindex && VarXindex < size);
		assert(0 <= VarYindex && VarYindex < size);
		S.A().addEntryCmplx(VarXindex,VarYindex,val);
	}

	///add a velue to the RHS
	void AddValB(int Xindex,
		ScalarType val)
	{
		int size=(int)S.nrows();
		assert(0 <= Xindex && Xindex < size);
		S.b()[Xindex] += val;
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

				AddValA(Xindex+1,Yindex,-val[i][j]);
				AddValA(Xindex,Yindex+1,val[i][j]);
			}
	}

	///set the diagonal of the matrix (which is zero at the beginning)
	///such that the sum of a row or a colums is zero
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

	///given a vector of scalar values and
	///a vector of indexes add such values 
	///as specified by the indexes
	void AddRHS(ScalarType b[6],
		int index[3])
	{
		for (int i=0;i<3;i++)
		{
			ScalarType valU=b[i*2];
			ScalarType valV=b[(i*2)+1];
			AddValB((index[i]*2),valU);
			AddValB((index[i]*2)+1,valV);
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
                assert((unsigned)Xindex<(n_vert_vars*2));
                assert((unsigned)Yindex<(n_vert_vars*2));
				AddValA(Xindex,Yindex,val[i][j]);
				AddValA(Xindex+1,Yindex+1,val[i][j]);
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
                assert((unsigned)Xindex<(n_vert_vars*2));
                assert((unsigned)Yindex<(n_vert_vars*2));
				AddValA(Xindex,Yindex,val[i][j]);
				AddValA(Xindex+1,Yindex+1,val[i][j]);
			}
	}
    ///END SYSTEM ACCESS METHODS

    ///START COMMON MATH FUNCTIONS
    ///return the complex encoding the rotation
    ///for a given missmatch interval
    Cmplx GetRotationComplex(int interval)
    {
        assert((interval>=0)&&(interval<4));

        switch(interval)
        {
        case 0:return Cmplx(1,0);
        case 1:return Cmplx(0,1);
        case 2:return Cmplx(-1,0);
        default:return Cmplx(0,-1);
        }
    }


    vcg::Point2i Rotate(vcg::Point2i p,int interval)
    {
        assert(interval>=0);
        assert(interval<4);
        Cmplx rot=GetRotationComplex(interval);
        /*
        | real  -imag| |p.x|
        |			 |*|   |
        | imag  real | |p.y|
        */
        vcg::Point2i ret;
        ret.X()=rot.real()*p.X()-rot.imag()*p.Y();
        ret.Y()=rot.imag()*p.X()+rot.real()*p.Y();
        return (ret);
    }
    ///END COMMON MATH FUNCTIONS


    ///START ENERGY MINIMIZATION PART
	///initialize the LHS for a given face
	///for minimization of Dirichlet's energy
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
        //Vindexes[0]=f->syst_index[0];
        //Vindexes[1]=f->syst_index[1];
        //Vindexes[2]=f->syst_index[2];
        Vindexes[0]=HandleS_Index[f][0];
        Vindexes[1]=HandleS_Index[f][1];
        Vindexes[2]=HandleS_Index[f][2];
        //VIndex.getIndexInfo(f,Vindexes);

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
            //ScalarType ScaleFactor=f->area;
			for (int x=0;x<3;x++)
				for (int y=0;y<3;y++)
					if (x!=y)
					{
						ScalarType num=(e[x]*e[y]);
						val[x][y] =num/(4.0*areaT);//*ScaleFactor);//*(ScalarType)convex_size);
                        val[x][y]*=Handle_Stiffness[f];//f->stiffening;
						//val[x][y]*=ScaleFactor;
					}

					///set the matrix as diagonal
					SetDiagonal(val);
	}

	///initialize the RHS for a given face
	///for minimization of Dirichlet's energy
	void perElementRHS(FaceType *f,
		ScalarType b[6],
		ScalarType vector_field_scale=1)
	{

		/// then set the rhs
        CoordType scaled_Kreal;
		CoordType scaled_Kimag;
		CoordType fNorm=f->N();
		CoordType p[3];
		p[0]=f->P0(0);
		p[1]=f->P0(1);
		p[2]=f->P0(2);

		CoordType neg_t[3];
        neg_t[0] = fNorm ^ (p[2] - p[1]);
		neg_t[1] = fNorm ^ (p[0] - p[2]);
        neg_t[2] = fNorm ^ (p[1] - p[0]);

		CoordType K1,K2;
        //K1=CrossVector<FaceType>(*f,0);
        //K2=CrossVector<FaceType>(*f,1);
        K1=f->PD1();
        K2=f->PD2();

        scaled_Kreal = K1*(vector_field_scale)/2;
        scaled_Kimag = K2*(vector_field_scale)/2;

        ScalarType stiff_val=(ScalarType)Handle_Stiffness[f];
        b[0] = scaled_Kreal * neg_t[0]*stiff_val;
        b[1] = scaled_Kimag * neg_t[0]*stiff_val;
        b[2] = scaled_Kreal * neg_t[1]*stiff_val;
        b[3] = scaled_Kimag * neg_t[1]*stiff_val;
        b[4] = scaled_Kreal * neg_t[2]*stiff_val;
        b[5] = scaled_Kimag * neg_t[2]*stiff_val;
	}

	///evaluate the LHS and RHS for a single face
	///for minimization of Dirichlet's energy
	void PerElementSystemReal(FaceType *f,
		ScalarType val[3][3],
		int index[3][3][2],
		ScalarType b[6],
		ScalarType vector_field_scale=1.0) 
	{
		perElementLHS(f,val,index);
		perElementRHS(f,b,vector_field_scale);
	}
    ///END ENERGY MINIMIZATION PART

    ///START FIXING VERTICES
	///set a given vertex as fixed 
    void AddFixedVertex(VertexType *v)
	{
		n_fixed_vars++;
		Hard_constraints.push_back(v);
        //v->blocked=true;
	}

    ///find vertex to fix in case we're using
    ///a vector field NB: multiple components not handled
    void FindFixedVertField()
    {
       Hard_constraints.clear();

       n_fixed_vars=0;
       ///fix the first singularity
       for (unsigned int j=0;j<mesh.vert.size();j++)
         {
             VertexType *v=&mesh.vert[j];
             if (v->IsD())continue;
             //if (v->IsSingular())
             if (Handle_Singular[v])
             {
                 AddFixedVertex(v);
                 v->T().P()=vcg::Point2<ScalarType>(0,0);
                 return;
             }
         }
       ///if anything fixed fix the first
       AddFixedVertex(&mesh.vert[0]);
       mesh.vert[0].T().P()=vcg::Point2<ScalarType>(0,0);
    }

	///find hard constraint depending if using or not 
    ///a vector field
	void FindFixedVert()
	{
		Hard_constraints.clear();
        FindFixedVertField();
    }

    int GetFirstVertexIndex(VertexType *v)
    {
        FaceType *f=v->VFp();
        int index=v->VFi();
        //return (f->syst_index[index]);
        return (HandleS_Index[f][index]);
        //int indexV[3];
        //VIndex.getIndexInfo(f,indexV);
        //return indexV[index];
    }

	///fix the vertices which are flagged as fixed
	void FixBlockedVertex()
	{
        int offset_row=n_vert_vars+num_cut_constraint;

		unsigned int constr_num = 0; 
        for (unsigned int i=0;i<Hard_constraints.size();i++)
		{
			VertexType *v=Hard_constraints[i];
			assert(!v->IsD());
			///get first index of the vertex that must blocked
            //int index=v->vertex_index[0];
            int index=GetFirstVertexIndex(v);
			///multiply times 2 because of uv
			int indexvert=index*2;
			///find the first free row to add the constraint
			int indexRow=(offset_row+constr_num)*2;
			int indexCol=indexRow;

			///add fixing constraint LHS
			AddValA(indexRow,indexvert,1);
			AddValA(indexRow+1,indexvert+1,1);

			///add fixing constraint RHS
            //AddValB(indexCol,(ScalarType)v->range_index.X());
            //AddValB(indexCol+1,(ScalarType)v->range_index.Y());
            AddValB(indexCol,(ScalarType)v->T().P().X());
            AddValB(indexCol+1,(ScalarType)v->T().P().Y());
			//AddValB(indexCol,0);
			//AddValB(indexCol+1,0);
			constr_num++;
		}
		assert(constr_num==n_fixed_vars);
	}
    ///END FIXING VERTICES

    ///HANDLING SINGULARITY
    //set the singularity round to integer location
    void AddSingularityRound()
    {
       for (unsigned int j=0;j<mesh.vert.size();j++)
          {
              VertexType *v=&mesh.vert[j];
              if (v->IsD())continue;
              //if (v->IsSingular())
              if (Handle_Singular[v])
              {
                  //assert(v->vertex_index.size()==1);
                  //ids_to_round.push_back(v->vertex_index[0]*2);
                  //ids_to_round.push_back((v->vertex_index[0]*2)+1);
                  int index0=GetFirstVertexIndex(v);
                  ids_to_round.push_back(index0*2);
                  ids_to_round.push_back((index0*2)+1);
              }
          }
    }

    ///START GENERIC SYSTEM FUNCTIONS
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
            //for(int k = 0; k < 3; ++k)
            //    var_idx[k] = f->syst_index[k];
            for(int k = 0; k < 3; ++k)
                  var_idx[k] = HandleS_Index[f][k];

            //VIndex.getIndexInfo(f,var_idx);

            ///block of variables
            ScalarType val[3][3];
            ///block of vertex indexes
            int index[3][3][2];
            ///righe hand side
            ScalarType b[6];
            ///compute the system for the given face
            PerElementSystemReal(f, val,index, b, vfscale);

            //Add the element to the matrix
            Add33Block(val,index);

            ///add area term.. to test if needed
            //if (!use_direction_field)
            //    AddAreaTerm(index,1.0);//f->area);

            ///add right hand side
            //if (use_direction_field)
                AddRHS(b,var_idx);
        }

	}

	
	///find different sized of the system
	void FindSizes()
	{
		///find the vertex that need to be fixed
		FindFixedVert();

        ///REAL PART
        n_vert_vars=Handle_SystemInfo().num_vert_variables;//VIndex.NumVertexVariables();

        ///INTEGER PART
        ///the total number of integer variables
        n_integer_vars=Handle_SystemInfo().num_integer_cuts;//VIndex.NumInteger();

        ///CONSTRAINT PART
        num_cut_constraint=Handle_SystemInfo().EdgeSeamInfo.size()*2;//VIndex.NumCutsConstraints();


        num_constraint_equations=num_cut_constraint+n_fixed_vars;

		///total variable of the system
        num_total_vars=n_vert_vars+n_integer_vars;

		///initialize matrix size
		//get max constraint-integer size
        int MaxAddSize=std::max(n_integer_vars,num_constraint_equations);
		
        system_size = (n_vert_vars+MaxAddSize)*2;

		printf("\n*** SYSTEM VARIABLES *** \n");
		printf("* NUM REAL VERTEX VARIABLES %d \n",n_vert_vars);

        printf("\n*** SINGULARITY *** \n ");
        printf("* NUM SINGULARITY %d\n",(int)ids_to_round.size()/2);

        printf("\n*** INTEGER VARIABLES *** \n");
        printf("* NUM INTEGER VARIABLES %d \n",(int)n_integer_vars);

		printf("\n*** CONSTRAINTS *** \n ");
		printf("* NUM FIXED CONSTRAINTS %d\n",n_fixed_vars);
		printf("* NUM CUTS CONSTRAINTS %d\n",num_cut_constraint);

		printf("\n*** TOTAL SIZE *** \n");
		printf("* TOTAL VARIABLE SIZE (WITH INTEGER TRASL) %d \n",num_total_vars);
		printf("* TOTAL CONSTRAINTS %d \n",num_constraint_equations);
        printf("* MATRIX SIZE  %d \n",system_size);
	}

	void AllocateSystem()
	{

		S.initialize(system_size, system_size);
        printf("\n INITIALIZED SPARSE MATRIX OF %d x %d \n",system_size, system_size);
        // number of nonzero entries in the matrix
        int num_facet=mesh.fn;
		//unsigned int nentries_vert  = 4*6*6*2*n_scalar_vars; // 6x6 for each facet
        unsigned int nentries_vert  = 6*6*num_facet*2;
		unsigned int nentries_fixed = 2*n_fixed_vars;      // 1 complex variable fixed in each constraint, i.e. 2 regular
		unsigned int nentries_transition  = 2*6*(n_integer_vars); // 3 complex variables involved in each constraint = 8 regular
		unsigned int nentries_constraints  = 4*4*6*(num_constraint_equations); //


		int total_reserve=nentries_vert
						+ nentries_fixed
						+ nentries_transition
						+nentries_constraints;

		printf("\n*** SPACE ALLOCATION *** \n");
		printf("* number of reserved vertices variables %d \n",nentries_vert);
		printf("* number of reserved entries fixed %d \n",nentries_fixed);
		printf("* number of reserved entries integer %d \n",nentries_transition);
		printf("* number of reserved entries constraints %d \n",nentries_constraints);
		printf("* total number of reserved entries %d \n",total_reserve);
		S.A().reserve(2*total_reserve);
		
		printf("\n*** ALLOCATED *** \n");

	}

	///intitialize the whole matrix
	void InitMatrix()
	{
        ///find singularities that must be rounded
        //AddSingularityRound();
		FindSizes();
		AllocateSystem();
	}

	///map back coordinates after that 
	///the system has been solved
	void MapCoords()
	{
		///map coords to faces
        for (unsigned int j=0;j<mesh.face.size();j++)
        {
            FaceType *f=&mesh.face[j];
            if (f->IsD())continue;
            //int indexV[3];
            //VIndex.getIndexInfo(f,indexV);
             for (int k=0;k<3;k++)
             {
                 //get the index of the variable in the system
                 //int indexUV=indexV[k];//f->syst_index[k];
                 int indexUV=HandleS_Index[f][k];
                 ///then get U and V coords
                 double U=X[indexUV*2];
                 double V=X[indexUV*2+1];
                 vcg::Point2<ScalarType> uv=vcg::Point2<ScalarType>(U,V);
                 ///assing
                 //f->realUV[k]=uv;
//                 ScalarType factor=(ScalarType)SIZEQUADS/(ScalarType)SIZEPARA;
//                 uv*=factor;
                 ///assing
                 f->WT(k).P()=uv;
             }
        }
        ///initialize the vector of integer variables to return their values
        Handle_SystemInfo().IntegerValues.resize(n_integer_vars*2);
        int baseIndex=(n_vert_vars)*2;
        int endIndex=baseIndex+n_integer_vars*2;
        int index=0;
        for (int i=baseIndex;i<endIndex;i++)
        {
            ///assert that the value is an integer value
            ScalarType value=X[i];
            ScalarType diff=value-(int)floor(value+0.5);
            assert(diff<0.00000001);
            Handle_SystemInfo().IntegerValues[index]=value;
            index++;
        }
     }

    ///END GENERIC SYSTEM FUNCTIONS

	///set the constraints for the inter-range cuts
	void BuildSeamConstraintsExplicitTranslation() 
	{
        ///add constraint(s) for every seam edge (not halfedge)
        int offset_row=n_vert_vars;
        ///current constraint row
		int constr_row=offset_row;
        ///current constraint
		unsigned int constr_num = 0;
		///get the info for seams
        //std::vector<SeamInfo> EdgeSeamInfo;
        //VIndex.GetSeamInfo(EdgeSeamInfo);
        for (unsigned int i=0;i<Handle_SystemInfo().EdgeSeamInfo.size();i++)
		{
            //if (i<25)continue;
            unsigned char interval=Handle_SystemInfo().EdgeSeamInfo[i].MMatch;
            if (interval==1)
                interval=3;
            else
             if(interval==3)
                    interval=1;
            //printf("%d\n",interval);
            int p0 = Handle_SystemInfo().EdgeSeamInfo[i].v0;
            int p1 = Handle_SystemInfo().EdgeSeamInfo[i].v1;
            int p0p = Handle_SystemInfo().EdgeSeamInfo[i].v0p;
            int p1p = Handle_SystemInfo().EdgeSeamInfo[i].v1p;
            /*assert(p1!=p1p);
            assert(p0!=p0p);*/
			Cmplx rot=GetRotationComplex(interval);

			///get the integer variable
            int integerVar=offset_row+Handle_SystemInfo().EdgeSeamInfo[i].integerVar;

            //Cmplx rotInt=GetRotationComplex(interval);
            if (integer_rounding)
            {
                ids_to_round.push_back(integerVar*2);
                ids_to_round.push_back(integerVar*2+1);
            }

            AddComplexA(constr_row, p0 ,  rot);
            AddComplexA(constr_row, p0p, -1);
			///then translation...considering the rotation 
			///due to substitution
            AddComplexA(constr_row, integerVar, 1);

			AddValB(2*constr_row,0);
			AddValB(2*constr_row+1,0);
			constr_row +=1;
            constr_num++;

            AddComplexA(constr_row, p1,  rot);
            AddComplexA(constr_row, p1p, -1);
            ///other translation
            AddComplexA(constr_row, integerVar  , 1);

			AddValB(2*constr_row,0);
			AddValB(2*constr_row+1,0);

			constr_row +=1;
			constr_num++;

			//// r p1  - p1' + t = 0  
			}
        //assert(constr_num==num_cut_constraint);
	}


	
	///call of the mixed integer solver
	void MixedIntegerSolve(double cone_grid_res=1, 
							bool direct_round=true,
                            int localIter=0)
    {
        X=std::vector< double >((n_vert_vars+n_integer_vars)*2);

		///variables part
        int ScalarSize=(n_vert_vars)*2;
        int SizeMatrix=(n_vert_vars+n_integer_vars)*2;
		printf("\n ALLOCATED X \n");

		///matrix A
        gmm::col_matrix< gmm::wsvector< double > > A(SizeMatrix,SizeMatrix); // lhs matrix variables +
		///constraints part
		int CsizeX=num_constraint_equations*2;
		int CsizeY=SizeMatrix+1;
        gmm::row_matrix< gmm::wsvector< double > > C(CsizeX,CsizeY); // constraints
		printf("\n ALLOCATED QMM STRUCTURES \n");

        std::vector< double > rhs(SizeMatrix,0);  // rhs
		printf("\n ALLOCATED RHS STRUCTURES \n");
        //// copy LHS
		for(int i = 0; i < (int)S.A().nentries(); ++i) 
		{
			int row = S.A().rowind()[i];
			int col = S.A().colind()[i];
			int size=(int)S.nrows();
			assert(0 <= row && row < size);
			assert(0 <= col && col < size);

			// it's either part of the matrix
			if (row < ScalarSize) {
				A(row, col) += S.A().vals()[i];
			}
			// or it's a part of the constraint
			else 
			{
                //if (row<(n_vert_vars+num_constraint_equations)*2)
                assert ((unsigned int)row<(n_vert_vars+num_constraint_equations)*2);
                //{
					int r = row - ScalarSize;
					assert(r<CsizeX);
					assert(col<CsizeY);
					C(r  , col  ) +=  S.A().vals()[i];
                //}
			}
		}
		printf("\n SET %d INTEGER VALUES \n",n_integer_vars);
        ///add penalization term for integer variables
        double penalization=0.000001;
        int offline_index=ScalarSize;
        for(unsigned int i = 0; i < (n_integer_vars)*2; ++i)
		{
			int index=offline_index+i;
			A(index,index)=penalization;
		}
		printf("\n SET RHS \n");
        // copy RHS
		for(int i = 0; i < (int)ScalarSize; ++i) 
		{
			rhs[i] = S.getRHSReal(i) * cone_grid_res;
		}
		// copy constraint RHS
		printf("\n SET %d CONSTRAINTS \n",num_constraint_equations);
        for(unsigned int i = 0; i < num_constraint_equations; ++i)
		{
            C(i, SizeMatrix) = -S.getRHSReal(ScalarSize + i) * cone_grid_res;
		}
        ///copy values back into S
		COMISO::ConstrainedSolver solver;

        solver.misolver().set_local_iters(localIter);
		solver.misolver().set_direct_rounding(direct_round);
		
		std::sort(ids_to_round.begin(),ids_to_round.end());
		std::vector<int>::iterator new_end=std::unique(ids_to_round.begin(),ids_to_round.end());
		int dist=distance(ids_to_round.begin(),new_end);
		ids_to_round.resize(dist);
        solver.solve( C, A, X, rhs, ids_to_round, 0.0, true, true);
    }

    void GetAttributes()
    {
       // you can query if an attribute is present or not
       bool hasSystIndex = vcg::tri::HasPerFaceAttribute(mesh,std::string("SystemIndex"));
       bool hasSingular = vcg::tri::HasPerVertexAttribute(mesh,std::string("Singular"));
       bool hasStiffness = vcg::tri::HasPerFaceAttribute(mesh,std::string("Stiffness"));
       bool HasSystemInfo=vcg::tri::HasPerMeshAttribute(mesh,std::string("SystemInfo"));

       assert(hasSystIndex);
       assert(hasSingular);
       assert(hasStiffness);
       assert(HasSystemInfo);

       HandleS_Index = vcg::tri::Allocator<MeshType>::template GetPerFaceAttribute<vcg::Point3i>(mesh,"SystemIndex");
       Handle_Singular=vcg::tri::Allocator<MeshType>::template GetPerVertexAttribute<bool>(mesh,std::string("Singular"));
       Handle_Stiffness=vcg::tri::Allocator<MeshType>::template GetPerFaceAttribute<float>(mesh,std::string("Stiffness"));
       Handle_SystemInfo=vcg::tri::Allocator<MeshType>::template GetPerMeshAttribute<MeshSystemInfo>(mesh,"SystemInfo");
     }

public:

    void SolvePoisson(ScalarType vector_field_scale=0.1f,
                      ScalarType grid_res=1.f,
					  bool direct_round=true,
                      int localIter=0,
                      bool _integer_rounding=true)
	{
        GetAttributes();
		//initialization of flags and data structures
        //ClearFlags();
        integer_rounding=_integer_rounding;

		ids_to_round.clear();

		///Initializing Matrix
		
		int t0=clock();

		///initialize the matrix ALLOCATING SPACE
		InitMatrix();
		printf("\n ALLOCATED THE MATRIX \n");

		///build the laplacian system
		BuildLaplacianMatrix(vector_field_scale);

        BuildSeamConstraintsExplicitTranslation();
		
		////add the lagrange multiplier
        FixBlockedVertex();

		printf("\n BUILT THE MATRIX \n");

        if (integer_rounding)
            AddSingularityRound();

        int t1=clock();
		printf("\n time:%d \n",t1-t0);
		printf("\n SOLVING \n");

		MixedIntegerSolve(grid_res,direct_round,localIter);

		int t2=clock();
		printf("\n time:%d \n",t2-t1);
		printf("\n ASSIGNING COORDS \n");
		MapCoords();
		int t3=clock();
		printf("\n time:%d \n",t3-t2);
        printf("\n FINISHED \n");
        //TagFoldedFaces();
	}
	

    PoissonSolver(MeshType &_mesh):mesh(_mesh)
    {}

};
#endif
