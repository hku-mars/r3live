#ifndef MIQ_VERTEX_INDEXING
#define MIQ_VERTEX_INDEXING
#include <vcg/complex/complex.h>
#include <vcg/simplex/face/pos.h>
#include <vcg/simplex/face/jumping_pos.h>
#include <vcg/simplex/face/topology.h>



struct SeamInfo
{
    int v0,v0p,v1,v1p;
    int integerVar;
    //unsigned char RotInt;
    unsigned char MMatch;

    SeamInfo(int _v0,
             int _v1,
             int _v0p,
             int _v1p,
             int _MMatch,
             int _integerVar)
    {
        v0=_v0;
        v1=_v1;
        v0p=_v0p;
        v1p=_v1p;
        integerVar=_integerVar;
        MMatch=_MMatch;
    }

    SeamInfo(const SeamInfo &S1)
    {
        v0=S1.v0;
        v1=S1.v1;
        v0p=S1.v0p;
        v1p=S1.v1p;
        integerVar=S1.integerVar;
        MMatch=S1.MMatch;
    }
};

struct MeshSystemInfo
{
    ///total number of scalar variables
    int num_scalar_variables;
    ////number of vertices variables
    int num_vert_variables;
    ///num of integer for cuts
    int num_integer_cuts;
    ///this are used for drawing purposes
    std::vector<SeamInfo> EdgeSeamInfo;
    ///this are values of integer variables after optimization
    std::vector<int> IntegerValues;
};

template <class MeshType>
class VertexIndexing
{

private:
    typedef typename MeshType::ScalarType ScalarType;
    typedef typename MeshType::FaceType FaceType;
    typedef typename MeshType::VertexType VertexType;
    typedef typename MeshType::CoordType CoordType;
    typedef typename MeshType::FaceIterator FaceIterator;

    MeshType *mesh;

    ///this maps back index to vertices
    std::vector<VertexType*> IndexToVert;
    ///this maps the integer for edge
    typename MeshType::template PerFaceAttributeHandle<vcg::Point3i> Handle_Integer;
    ///per face indexes of vertex in the solver
    typename  MeshType::template PerFaceAttributeHandle<vcg::Point3i> HandleS_Index;
    ///per face per edge of mmatch in the solver
    typename  MeshType::template PerFaceAttributeHandle<vcg::Point3i> Handle_MMatch;
    ///per vertex variable indexes
    typename  MeshType::template PerVertexAttributeHandle<std::vector<int> > HandleV_Integer;
    ///per vertex singular or not
    typename  MeshType::template PerVertexAttributeHandle<bool> Handle_Singular;
    ///per vertex degree of a singularity
    typename  MeshType::template PerVertexAttributeHandle<int> Handle_SingularDegree;
    ///seam per face
    typename  MeshType::template PerFaceAttributeHandle<vcg::Point3<bool> > Handle_Seams;
    ///this handle for mesh
    typename  MeshType::template PerMeshAttributeHandle<MeshSystemInfo> Handle_SystemInfo;

    ///this are used for drawing purposes
    std::vector<VertexType*> duplicated;

    void FirstPos(const VertexType* v,FaceType *&f,int &edge)
    {
        f=v->cVFp();
        edge=v->cVFi();
    }


    int AddNewIndex(VertexType* v0)
    {
        Handle_SystemInfo().num_scalar_variables++;
        HandleV_Integer[v0].push_back(Handle_SystemInfo().num_scalar_variables);
        IndexToVert.push_back(v0);
        return Handle_SystemInfo().num_scalar_variables;
    }

    bool HasIndex(int indexVert,int indexVar)
    {
        for (unsigned int i=0;i<HandleV_Integer[indexVert].size();i++)
            if (HandleV_Integer[indexVert][i]==indexVar)return true;
        return false;
    }

    bool HasIndex(VertexType* v0,int indexVar)
    {
        for (unsigned int i=0;i<HandleV_Integer[v0].size();i++)
            if (HandleV_Integer[v0][i]==indexVar)return true;
        return false;
    }

    void GetSeamInfo(const FaceType *f0,
                     const FaceType *f1,
                     const int indexE,
                     int &v0,int &v1,
                     int &v0p,int &v1p,
                     unsigned char &_MMatch,
                     int &integerVar)
    {
        int edgef0=indexE;
        v0=HandleS_Index[f0][edgef0];
        v1=HandleS_Index[f0][(edgef0+1)%3];
        ////get the index on opposite side
        assert(f0->cFFp(edgef0)==f1);
        int edgef1=f0->cFFi(edgef0);
        v1p=HandleS_Index[f1][edgef1];
        v0p=HandleS_Index[f1][(edgef1+1)%3];

        integerVar=Handle_Integer[f0][edgef0];
        _MMatch=Handle_MMatch[f0][edgef0];
        assert(f0->cV0(edgef0)==f1->cV1(edgef1));
        assert(f0->cV1(edgef0)==f1->cV0(edgef1));
    }

    bool IsSeam(FaceType *f0,FaceType *f1)
    {
        for (int i=0;i<3;i++)
        {
            FaceType *f_clos=f0->FFp(i);
            assert(!f_clos->IsD());   ///check not deleted
            if (f_clos==f0)continue;///border
            if(f_clos==f1)
                return(Handle_Seams[f0][i]);
        }
        assert(0);
        return false;
    }

    ///find initial position of the pos to
    // assing face to vert inxex correctly
    void FindInitialPos(const VertexType * vert,
                        int &edge,
                        FaceType *&face)
    {
        FaceType *f_init;
        int edge_init;
        FirstPos(vert,f_init,edge_init);
        vcg::face::JumpingPos<FaceType> VFI(f_init,edge_init);
        bool vertexB=vert->IsB();
        bool possible_split=false;
        bool complete_turn=false;
        do
        {
            FaceType *curr_f=VFI.F();
            int curr_edge=VFI.E();
            VFI.NextFE();
            FaceType *next_f=VFI.F();
            ///test if I've just crossed a border
            bool on_border=(curr_f->FFp(curr_edge)==curr_f);
            //bool mismatch=false;
            bool seam=false;

            ///or if I've just crossed a seam
            ///if I'm on a border I MUST start from the one next t othe border
            if (!vertexB)
                //seam=curr_f->IsSeam(next_f);
                seam=IsSeam(curr_f,next_f);
            if (vertexB)
                assert(!Handle_Singular[vert]);
            ;
                //assert(!vert->IsSingular());
            possible_split=((on_border)||(seam));
            complete_turn=(next_f==f_init);
        }while ((!possible_split)&&(!complete_turn));
        face=VFI.F();
        edge=VFI.E();
        ///test that is not on a border
        //assert(face->FFp(edge)!=face);
    }

    ///intialize the mapping given an initial pos
    ///whih must be initialized with FindInitialPos
    void MapIndexes(VertexType * vert,
                    int &edge_init,
                    FaceType *&f_init)
    {
        ///check that is not on border..
        ///in such case maybe it's non manyfold
        ///insert an initial index
        int curr_index=AddNewIndex(vert);
        ///and initialize the jumping pos
        vcg::face::JumpingPos<FaceType> VFI(f_init,edge_init);
        bool complete_turn=false;
        do
        {
            FaceType *curr_f=VFI.F();
            int curr_edge=VFI.E();
            ///assing the current index
            HandleS_Index[curr_f][curr_edge]=curr_index;
            VFI.NextFE();
            FaceType *next_f=VFI.F();
            ///test if I've finiseh with the face exploration
            complete_turn=(next_f==f_init);
            ///or if I've just crossed a mismatch
            if (!complete_turn)
            {
                bool seam=false;
                //seam=curr_f->IsSeam(next_f);
                seam=IsSeam(curr_f,next_f);
                if (seam)
                {
                    ///then add a new index
                    curr_index=AddNewIndex(vert);
                }
            }
        }while (!complete_turn);
    }

    ///intialize the mapping for a given vertex
    void InitMappingSeam(VertexType *vert)
    {
        ///first rotate until find the first pos after a mismatch
        ///or a border or return to the first position...
        FaceType *f_init=vert->VFp();
        int indexE=vert->VFi();
        vcg::face::JumpingPos<FaceType> VFI(f_init,indexE);

        int edge_init;
        FaceType *face_init;
        FindInitialPos(vert,edge_init,face_init);
        MapIndexes(vert,edge_init,face_init);
    }

    ///intialize the mapping for a given sampled mesh
    void InitMappingSeam()
    {
        //num_scalar_variables=-1;
        Handle_SystemInfo().num_scalar_variables=-1;
        for (unsigned int i=0;i<mesh->vert.size();i++)
            if (!mesh->vert[i].IsD())
                InitMappingSeam(&mesh->vert[i]);

        for (unsigned int j=0;j<mesh->vert.size();j++)
        {
            VertexType *v= &(mesh->vert[j]);
            if (!v->IsD()){
                assert(HandleV_Integer[j].size()>0);
                if (HandleV_Integer[j].size()>1)
                    duplicated.push_back(v);
            }
        }
    }

    ///initialized mapping structures if are not already initialized
    void AddAttributesIfNeeded()
    {
        IndexToVert.clear();

        ///other per mesh attributes
        ///see if already exists otherwise it creates it
        Handle_SystemInfo=vcg::tri::Allocator<MeshType>::template GetPerMeshAttribute<MeshSystemInfo>(*mesh,std::string("SystemInfo"));

        Handle_SystemInfo().num_scalar_variables=0;
        Handle_SystemInfo().num_vert_variables=0;
        Handle_SystemInfo().num_integer_cuts=0;

        duplicated.clear();

        HandleS_Index = vcg::tri::Allocator<MeshType>::template GetPerFaceAttribute<vcg::Point3i>(*mesh,std::string("SystemIndex"));

        Handle_Integer= vcg::tri::Allocator<MeshType>::template GetPerFaceAttribute<vcg::Point3i>(*mesh,std::string("Integer"));
        HandleV_Integer=vcg::tri::Allocator<MeshType>::template GetPerVertexAttribute<std::vector<int> >(*mesh,std::string("VertInteger"));


        bool HasHandleMMatch=vcg::tri::HasPerFaceAttribute(*mesh,std::string("MissMatch"));
        assert(HasHandleMMatch);
        Handle_MMatch = vcg::tri::Allocator<MeshType>::template FindPerFaceAttribute<vcg::Point3i>(*mesh,std::string("MissMatch"));

        bool HasHandleSingularity=vcg::tri::HasPerVertexAttribute(*mesh,std::string("Singular"));
        assert(HasHandleSingularity);
        Handle_Singular=vcg::tri::Allocator<MeshType>::template FindPerVertexAttribute<bool>(*mesh,std::string("Singular"));

        bool HasHandleSingularityDegree=vcg::tri::HasPerVertexAttribute(*mesh,std::string("SingularityDegree"));
        assert(HasHandleSingularityDegree);
        Handle_SingularDegree=vcg::tri::Allocator<MeshType>::template FindPerVertexAttribute<int>(*mesh,std::string("SingularityDegree"));

        bool HasHandleSeams=vcg::tri::HasPerFaceAttribute(*mesh,std::string("Seams"));
        assert(HasHandleSeams);
        Handle_Seams=vcg::tri::Allocator<MeshType>::template FindPerFaceAttribute<vcg::Point3<bool> >(*mesh,std::string("Seams"));

    }

    ///test consistency of face variables per vert mapping
    void TestSeamMapping(FaceType *f)
    {
        for (int k=0;k<3;k++)
        {
            int indexV=HandleS_Index[f][k];
            VertexType *v=f->V(k);
            bool has_index=HasIndex(v,indexV);
            assert(has_index);
        }
    }

    ///test consistency of face variables per vert mapping
    void TestSeamMapping(int indexVert)
    {
        VertexType *v=&mesh->vert[indexVert];
        for (unsigned int k=0;k<HandleV_Integer[indexVert].size();k++)
        {
            int indexV=HandleV_Integer[indexVert][k];

            ///get faces sharing vertex
            std::vector<FaceType*> faces;
            std::vector<int> indexes;

            vcg::face::VFStarVF(v,faces,indexes);
            for (unsigned int j=0;j<faces.size();j++)
            {
                FaceType *f=faces[j];
                int index=indexes[j];
                assert(!f->IsD());
                assert(f->V(index)==v);
                assert((index>=0)&&(index<3));

                if (HandleS_Index[f][index]==indexV)
                    return;
            }
        }
        assert(0);
    }


    ///check consistency of variable mapping across seams
    void TestSeamMapping()
    {
        printf("\n TESTING SEAM INDEXES \n");
        ///test F-V mapping
        for (unsigned int j=0;j<mesh->face.size();j++)
        {
            FaceType *f=&mesh->face[j];
            if (f->IsD()) continue;
            TestSeamMapping(f);
        }

        ///TEST  V-F MAPPING
        for (unsigned int j=0;j<mesh->vert.size();j++)
        {
            VertexType *v=&mesh->vert[j];
            if (v->IsD()) continue;
            TestSeamMapping(j);
        }

    }


public:

    ///vertex to variable mapping
    void InitMapping()
    {
        //use_direction_field=_use_direction_field;

        IndexToVert.clear();
        duplicated.clear();

        InitMappingSeam();

        Handle_SystemInfo().num_vert_variables=Handle_SystemInfo().num_scalar_variables+1;

        ///end testing...
        TestSeamMapping();
    }

    void InitFaceIntegerVal()
    {
        Handle_SystemInfo().num_integer_cuts=0;
        for (unsigned int j=0;j<mesh->face.size();j++)
        {
            FaceType *f=&mesh->face[j];
            if (f->IsD())continue;
            for (int k=0;k<3;k++)
            {
                 if (Handle_Seams[f][k])
                {
                    Handle_Integer[j][k]=Handle_SystemInfo().num_integer_cuts;
                    Handle_SystemInfo().num_integer_cuts++;
                }
                else
                    Handle_Integer[j][k]=-1;
            }
        }
    }

    void InitSeamInfo()
    {
        Handle_SystemInfo().EdgeSeamInfo.clear();
        for (unsigned int j=0;j<mesh->face.size();j++)
        {
            FaceType *f0=&mesh->face[j];
            if (f0->IsD())continue;
            for (int k=0;k<3;k++)
            {
                FaceType *f1=f0->FFp(k);
                if (f1==f0)continue;
                bool seam=Handle_Seams[f0][k];//f0->IsSeam(k);
                if ((seam) )//&&(f0<f1))
                {
                    int v0,v0p,v1,v1p;
                    unsigned char MM;
                    int integerVar;
                    GetSeamInfo(f0,f1,k,v0,v1,v0p,v1p,MM,integerVar);
                    Handle_SystemInfo().EdgeSeamInfo.push_back(SeamInfo(v0,v1,v0p,v1p,MM,integerVar));
                }
            }
        }
    }

    void Init(MeshType *_mesh){mesh=_mesh;AddAttributesIfNeeded();}

    VertexIndexing(){mesh=NULL;}
};

#endif
