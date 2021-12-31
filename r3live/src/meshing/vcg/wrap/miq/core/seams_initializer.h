#ifndef MIQ_SEAMS_INTIALIZER
#define MIQ_SEAMS_INTIALIZER
#include <vcg/complex/complex.h>
#include <vcg/simplex/face/pos.h>
#include <vcg/simplex/face/jumping_pos.h>
#include <vcg/simplex/face/topology.h>
#include <wrap/io_trimesh/import_field.h>

template <class MeshType>
class SeamsInitializer
{

private:
    typedef typename MeshType::ScalarType ScalarType;
    typedef typename MeshType::FaceType FaceType;
    typedef typename MeshType::VertexType VertexType;
    typedef typename MeshType::CoordType CoordType;
    typedef typename MeshType::FaceIterator FaceIterator;

    MeshType *mesh;

    ///per face per edge of mmatch in the solver
    typename  MeshType::template PerFaceAttributeHandle<vcg::Point3i> Handle_MMatch;
    ///per vertex singular or not
    typename  MeshType::template PerVertexAttributeHandle<bool> Handle_Singular;
    ///per vertex degree of a singularity
    typename  MeshType::template PerVertexAttributeHandle<int> Handle_SingularDegree;
    ///seam per face
    typename  MeshType::template PerFaceAttributeHandle<vcg::Point3<bool> > Handle_Seams;
    ///seam index per face
    typename  MeshType::template PerFaceAttributeHandle<vcg::Point3i > Handle_SeamsIndex;

    bool IsRotSeam(const FaceType *f0,const int edge)
    {
        unsigned char MM=Handle_MMatch[f0][edge];//MissMatch(f0,edge);
        return (MM!=0);
    }

    ///return true if a vertex is singluar by looking at initialized missmatches
    bool IsSingularByMMatch(const VertexType &v,int &missmatch)
    {
        ///check that is on border..
        if (v.IsB())return false;

        std::vector<FaceType*> faces;
        std::vector<int> edges;

        vcg::face::Pos<FaceType> pos(v.cVFp(), v.cVFi());
        vcg::face::VFOrderedStarFF(pos,faces,edges);

        missmatch=0;
        for (unsigned int i=0;i<faces.size();i++)
        {
            FaceType *curr_f=faces[i];
            int currMM=Handle_MMatch[curr_f][edges[i]];
            missmatch+=currMM;
        }
        missmatch=missmatch%4;
        return(missmatch!=0);
    }

    ///initialized mapping structures if are not already initialized
    void AddAttributesIfNeeded()
    {
        Handle_MMatch = vcg::tri::Allocator<MeshType>::template GetPerFaceAttribute<vcg::Point3i>(*mesh,std::string("MissMatch"));
        Handle_Singular=vcg::tri::Allocator<MeshType>::template GetPerVertexAttribute<bool>(*mesh,std::string("Singular"));
        Handle_SingularDegree=vcg::tri::Allocator<MeshType>::template GetPerVertexAttribute<int>(*mesh,std::string("SingularityDegree"));
        Handle_Seams=vcg::tri::Allocator<MeshType>::template GetPerFaceAttribute<vcg::Point3<bool> >(*mesh,std::string("Seams"));
        Handle_SeamsIndex=vcg::tri::Allocator<MeshType>::template GetPerFaceAttribute<vcg::Point3i >(*mesh,std::string("SeamsIndex"));
    }


    void FloodFill(FaceType* start)
    {
        std::deque<FaceType*> d;
        ///clean the visited flag
        start->SetV();
        d.push_back(start);

        while (!d.empty()){
            FaceType *f = d.at(0); d.pop_front();
            for (int s = 0; s<3; s++)
            {
                FaceType *g = f->FFp(s);
                int j = f->FFi(s);
                if ((!(IsRotSeam(f,s))) && (!(IsRotSeam(g,j)))  && (!g->IsV()) )
                {
                    Handle_Seams[f][s]=false;
                    Handle_Seams[g][j]=false;
                    g->SetV();
                    d.push_back(g);
                }
            }
        }
    }

    void Retract(){
        std::vector<int> e(mesh->vert.size(),0); // number of edges per vert
        VertexType *vb = &(mesh->vert[0]);
        for (FaceIterator f = mesh->face.begin(); f!=mesh->face.end(); f++) if (!f->IsD()){
            for (int s = 0; s<3; s++){
                //if (f->seam[s])
                if (Handle_Seams[f][s])
                    if (f->FFp(s)<=&*f)  {
                        e[ f->V(s) - vb ] ++;
                        e[ f->V1(s) - vb ] ++;
                    }
            }
        }
        bool over=true;
        int guard = 0;
        do {
            over = true;
            for (FaceIterator f = mesh->face.begin(); f!=mesh->face.end(); f++) if (!f->IsD()){
                for (int s = 0; s<3; s++){
                    //if (f->seam[s])
                    if (Handle_Seams[f][s])
                        if (!(IsRotSeam(&(*f),s))) // never retract rot seams
                            //if (f->FFp(s)<=&*f)
                        {
                            if (e[ f->V(s) - vb ] == 1) {
                                // dissolve seam
                                //f->seam[s] = false;
                                Handle_Seams[f][s]=false;
                                //f->FFp(s)->seam[(int)f->FFi(s)] = false;
                                Handle_Seams[f->FFp(s)][(int)f->FFi(s)]=false;
                                e[ f->V(s) - vb ] --;
                                e[ f->V1(s) - vb ] --;
                                over = false;
                            }
                        }
                }
            }

            if (guard++>10000) over = true;

        } while (!over);
    }

    void AddSeamsByMM()
    {
        for (unsigned int i=0;i<mesh->face.size();i++)
        {
            FaceType *f=&mesh->face[i];
            if (f->IsD())continue;
            for (int j=0;j<3;j++)
            {
                if (IsRotSeam(f,j))
                    Handle_Seams[f][j]=true;
                    //f->SetSeam(j);
            }
        }
    }

    void SelectSingularityByMM()
    {

        for (unsigned int i=0;i<mesh->vert.size();i++)
        {
            if (mesh->vert[i].IsD())continue;
            int missmatch;
            bool isSing=IsSingularByMMatch(mesh->vert[i],missmatch);
            if (isSing)
            {
                mesh->vert[i].SetS();
                Handle_Singular[i]=true;
                if (missmatch==3)missmatch=1;
                else
                if (missmatch==1)missmatch=3;
                Handle_SingularDegree[i]=missmatch;
            }
            else
            {
                mesh->vert[i].ClearS();
                Handle_Singular[i]=false;
                Handle_SingularDegree[i]=0;
            }
        }
    }


    int InitTopologycalCuts(){
        vcg::tri::UpdateFlags<MeshType>::FaceClearV(*mesh);

        for (FaceIterator f = mesh->face.begin(); f!=mesh->face.end(); f++)
            if (!f->IsD())
            {
                Handle_Seams[f][0]=true;
                Handle_Seams[f][1]=true;
                Handle_Seams[f][2]=true;
            }

        int index=0;
        for (FaceIterator f = mesh->face.begin(); f!=mesh->face.end(); f++)
            if (!f->IsD())
            {
                if (!f->IsV())
                {
                    index++;
                    FloodFill(&*f);
                }
            }

        Retract();
        return index;
    }

    void InitMMatch()
    {
        for (unsigned int i=0;i<mesh->face.size();i++)
        {
            FaceType *curr_f=&mesh->face[i];
            for (int j=0;j<3;j++)
            {
                FaceType *opp_f=curr_f->FFp(j);
                if (curr_f==opp_f)
                    Handle_MMatch[curr_f][j]=0;
                else
                    Handle_MMatch[curr_f][j]=vcg::tri::CrossField<MeshType>::MissMatchByCross(*curr_f,*opp_f);
            }
        }
    }

    void InitSeamIndexes()
    {
        ///initialize seams indexes
        for (unsigned int i=0;i<mesh->face.size();i++)
        {
            FaceType *f=&mesh->face[i];
            for (int j=0;j<3;j++)
               Handle_SeamsIndex[f][j]=-1;
        }

        std::vector<std::vector<std::pair<FaceType*,int> > > seamsVert;
        seamsVert.resize(mesh->vert.size());
        for (unsigned int i=0;i<mesh->face.size();i++)
        {
            FaceType *f=&mesh->face[i];
             for (int j=0;j<3;j++)
             {
                 if (f->IsB(j))continue;
                 if (Handle_Seams[f][j])
                 {
                     VertexType *v0=f->V0(j);
                     VertexType *v1=f->V1(j);
                     if (v0>v1)continue;
                     int index0=v0-&mesh->vert[0];
                     int index1=v1-&mesh->vert[0];
                     std::pair<FaceType*,int> entry=std::pair<FaceType*,int>(f,j);
                     seamsVert[index0].push_back(entry);
                     seamsVert[index1].push_back(entry);
                 }
             }
        }

        int curr_index=0;
        for (unsigned int i=0;i<mesh->vert.size();i++)
        {
            VertexType *v_seam=&mesh->vert[i];
            bool IsVertex=(Handle_Singular[i])||(seamsVert[i].size()>2)||
                          (v_seam->IsB());

            if(!IsVertex)continue;

            ///then follows the seam
            for (unsigned int j=0;j<seamsVert[i].size();j++)
            {
                FaceType *f=seamsVert[i][j].first;
                int edge=seamsVert[i][j].second;

                VertexType *v0=v_seam;
                VertexType *v1=NULL;

                ///the index is already initialized
                if (Handle_SeamsIndex[f][edge]!=-1)continue;
                bool finished=false;
                do
                {
                    ///otherwise follow the index
                    Handle_SeamsIndex[f][edge]=curr_index;///set current value
                    FaceType *f1=f->FFp(edge);
                    int edge1=f->FFi(edge);
                    Handle_SeamsIndex[f1][edge1]=curr_index;///set current value

                    assert((v0==f->V0(edge))||
                           (v0==f->V1(edge)));

                    if (f->V0(edge)==v0)
                        v1=f->V1(edge);
                    else
                        v1=f->V0(edge);

                   assert(v0!=v1);
                   //int index0=v0-&mesh->vert[0];
                   int index1=v1-&mesh->vert[0];

                   bool IsVertex=(Handle_Singular[v1])||
                                 (seamsVert[index1].size()>2)||
                                 (v1->IsB());
                   if(IsVertex)
                       finished=true;
                   else
                   {
                       assert(seamsVert[index1].size()==2);
                       FaceType *f_new[2];
                       int edge_new[2];

                       f_new[0]=seamsVert[index1][0].first;
                       edge_new[0]=seamsVert[index1][0].second;
                       f_new[1]=seamsVert[index1][1].first;
                       edge_new[1]=seamsVert[index1][1].second;

                       assert((f_new[0]==f)||(f_new[1]==f));
                       assert((edge_new[0]==edge)||(edge_new[1]==edge));

                       if ((f_new[0]==f)&&(edge_new[0]==edge))
                       {
                           f=f_new[1];
                           edge=edge_new[1];
                       }
                       else
                       {
                           f=f_new[0];
                           edge=edge_new[0];
                       }
                       v0=v1;
                   }
                }
                while (!finished);
                //return;
                curr_index++;
            }
        }

    }

public:


    void Init(MeshType *_mesh,
              bool orient_globally,
              bool initMM,
              bool initCuts)
    {

        mesh=_mesh;

        vcg::tri::UpdateTopology<MeshType>::FaceFace(*_mesh);
        vcg::tri::UpdateFlags<MeshType>::FaceBorderFromFF(*_mesh);
        vcg::tri::UpdateFlags<MeshType>::VertexBorderFromFace(*_mesh);


        AddAttributesIfNeeded();
        if (orient_globally)
            vcg::tri::CrossField<MeshType>::MakeDirectionFaceCoherent(*mesh);
        if (initMM)
            InitMMatch();
        SelectSingularityByMM();
        if (initCuts)
        {
            InitTopologycalCuts();
            AddSeamsByMM();
        }
        //InitSeamIndexes();
    }

    SeamsInitializer(){mesh=NULL;}
};

#endif
