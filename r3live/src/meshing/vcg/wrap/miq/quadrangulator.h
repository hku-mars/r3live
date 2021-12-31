#ifndef MIQ_QUADRANGULATOR_H
#define MIQ_QUADRANGULATOR_H

#include <vcg/complex/complex.h>
#include <vcg/simplex/face/pos.h>
#include <vcg/simplex/face/jumping_pos.h>
#include <vcg/complex/algorithms/attribute_seam.h>
#include <vcg/complex/algorithms/refine.h>
#include <vcg/complex/algorithms/smooth.h>
#include <vcg/complex/algorithms/clean.h>


template <class MeshType>
inline void ExtractVertex(const MeshType & srcMesh,
                          const typename MeshType::FaceType & f,
                          int whichWedge,
                          const MeshType &dstMesh,
                          typename MeshType::VertexType & v)
{
    (void)srcMesh;
    (void)dstMesh;

    //v.P() = f.cP(whichWedge);
    v.ImportData(*f.cV(whichWedge));
    v.T() = f.cWT(whichWedge);
}

template <class MeshType>
inline bool CompareVertex(const MeshType & m,
                          const typename MeshType::VertexType & vA,
                          const typename MeshType::VertexType & vB)
{
    (void)m;
    return ((vA.cT() == vB.cT())&&(vA.cP()==vB.cP()));
}


template <class TriMesh,class PolyMesh>
class Quadrangulator
{

public:
    typedef typename TriMesh::FaceType TriFaceType;
    typedef typename TriMesh::VertexType TriVertexType;
    typedef typename TriMesh::CoordType CoordType;
    typedef typename TriMesh::ScalarType ScalarType;

    typedef typename PolyMesh::FaceType PolyFaceType;
    typedef typename PolyMesh::VertexType PolyVertexType;
    typedef typename PolyMesh::CoordType PolyCoordType;
    typedef typename PolyMesh::ScalarType PolyScalarType;


    ///the set of all edges that belongs to integer lines
    std::set<std::pair<TriFaceType*,int> > IntegerEdges;

    ///the set of all integer vertices and the other vertices on integer lines which is connectes to
    std::map<TriVertexType*,std::vector<TriVertexType*> > IntegerLineAdj;
    ///the set of integer vertices
    std::set<TriVertexType*> IntegerVertices;
    ///temporary polygons
    std::vector<std::vector<TriVertexType *> > polygons;

    ///drawing debug structures
    std::vector<std::pair<TriFaceType*,int> > IntegerLines;
    std::vector<TriVertexType*> IntegerVertex;

private:
    static bool ToSplit(const vcg::Point2<ScalarType> &uv0,
                        const vcg::Point2<ScalarType> &uv1,
                        int Dir,
                        int IntegerLine,
                        ScalarType &alpha,
                        ScalarType tolerance=0.0001)
    {
        ScalarType lineF=(ScalarType)IntegerLine;
        ScalarType val0=std::min(uv0.V(Dir),uv1.V(Dir));
        ScalarType val1=std::max(uv0.V(Dir),uv1.V(Dir));
        if (lineF<(val0+tolerance))return false;
        if (lineF>(val1-tolerance))return false;
        ScalarType dist=fabs(uv0.V(Dir)-uv1.V(Dir));
        if (dist<tolerance) return false;
        alpha=(1.0-fabs(uv0.V(Dir)-lineF)/dist);
        return true;
    }

    ///return true if the edge has to be splitted,
    ///by considering the tolerance to the closest integer
    static bool ToSplit(const vcg::face::Pos<TriFaceType> &ep,
                        ScalarType &alpha,
                        ScalarType factor=1.0,
                        ScalarType tolerance=0.0001)
    {
        //TriFaceType *f=ep.f;
        //int z=ep.z;
        TriVertexType* v0=ep.f->V(ep.z);
        TriVertexType* v1=ep.f->V1(ep.z);
        vcg::Point2<ScalarType> uv0=v0->T().P()*factor;
        vcg::Point2<ScalarType> uv1=v1->T().P()*factor;

        ///then test integer for each direction
        for (int dir=0;dir<2;dir++)
        {
            int Int0=std::min((int)uv0.V(dir),(int)uv1.V(dir));
            int Int1=std::max((int)uv0.V(dir),(int)uv1.V(dir));

            for (int i=Int0;i<=Int1;i++)
            {
                bool to_split=ToSplit(uv0,uv1,dir,i,alpha,tolerance);
                if (to_split)return true;
            }
        }
        return false;
    }


    // Basic subdivision class
    // This class must provide methods for finding the position of the newly created vertices
    // In this implemenation we simply put the new vertex in the MidPoint position.
    // Color and TexCoords are interpolated accordingly.
    template<class MESH_TYPE>
    struct SplitMidPoint : public   std::unary_function<vcg::face::Pos<typename MESH_TYPE::FaceType> ,  typename MESH_TYPE::CoordType >
    {
        typedef typename MESH_TYPE::VertexType VertexType;
        typedef typename MESH_TYPE::FaceType FaceType;
        typedef typename MESH_TYPE::CoordType CoordType;

        ScalarType factor;
        ScalarType tolerance;
        ScalarType alpha;

        void operator()(typename MESH_TYPE::VertexType &nv,
                        vcg::face::Pos<typename MESH_TYPE::FaceType>  ep)
        {
            bool to_split=ToSplit(ep,alpha,factor,tolerance);
            assert(to_split);

            ///get the value on which the edge must be splitted
            VertexType* v0=ep.f->V(ep.z);
            VertexType* v1=ep.f->V1(ep.z);

            nv.P()= v0->P()*alpha+v1->P()*(1.0-alpha);
            //nv.N()= v0->N()*alpha+v1->N()*(1.0-alpha);
            nv.T().P()=v0->T().P()*alpha+v1->T().P()*(1.0-alpha);
        }

        vcg::TexCoord2<ScalarType> WedgeInterp(vcg::TexCoord2<ScalarType> &t0,
                                               vcg::TexCoord2<ScalarType> &t1)
        {
            vcg::TexCoord2<ScalarType> tmp;
//            if (t0.n() != t1.n())
//                cerr << "Failed assertion: Quadrangulator::WedgeInterp1" << endl;
//            // assert(t0.n()== t1.n()); TODO put back
            tmp.n()=t0.n();
            // assert(alpha>=0); TODO put back
            if (alpha<0)
                cerr << "Failed assertion: Quadrangulator::WedgeInterp2" << endl;
            tmp.t()=(alpha*t0.t()+(1.0-alpha)*t1.t());
            return tmp;
        }

        SplitMidPoint(){alpha=-1;}
    };

    template <class MESH_TYPE>
    class EdgePredicate
    {
        typedef typename MESH_TYPE::VertexType VertexType;
        typedef typename MESH_TYPE::FaceType FaceType;
        typedef typename MESH_TYPE::ScalarType ScalarType;

    public:
        ScalarType factor;
        ScalarType tolerance;

        bool operator()(vcg::face::Pos<typename MESH_TYPE::FaceType> ep) const
        {
            ScalarType alpha;
            return(ToSplit(ep,alpha,factor,tolerance));
        }
    };

    void SplitTris(TriMesh &to_split,
                   ScalarType factor=1.0,
                   ScalarType tolerance=0.0001)
    {
        bool done=true;
        SplitMidPoint<TriMesh> splMd;
        EdgePredicate<TriMesh> eP;

        splMd.tolerance=tolerance;
        splMd.factor=factor;
        eP.tolerance=tolerance;
        eP.factor=factor;

        while (done)
            done=vcg::tri::RefineE<TriMesh,SplitMidPoint<TriMesh>,EdgePredicate<TriMesh> >(to_split,splMd,eP);

        for (unsigned int i=0;i<to_split.face.size();i++)
            for (int j=0;j<3;j++) to_split.face[i].WT(j).P()=to_split.face[i].V(j)->T().P();
    }


    bool IsOnIntegerLine(vcg::Point2<ScalarType> uv0,
                         vcg::Point2<ScalarType> uv1,
                         ScalarType tolerance=0.0001)
    {
        for (int dir=0;dir<2;dir++)
        {
            ScalarType val0=uv0.V(dir);
            ScalarType val1=uv1.V(dir);
            int integer0=floor(uv0.V(dir)+0.5);
            int integer1=floor(uv1.V(dir)+0.5);
            if (integer0!=integer1)continue;
            if ((fabs(val0-(ScalarType)integer0))>tolerance)continue;
            if ((fabs(val1-(ScalarType)integer1))>tolerance)continue;
            return true;
        }
        return false;
    }

    bool IsOnIntegerVertex(vcg::Point2<ScalarType> uv,
                           ScalarType tolerance=0.0001)
    {
        for (int dir=0;dir<2;dir++)
        {
            ScalarType val0=uv.V(dir);
            int integer0=floor(val0+0.5);
            if ((fabs(val0-(ScalarType)integer0))>tolerance)return false;
        }
        return true;
    }

    void InitIntegerVectors()
    {
        IntegerLines=std::vector<std::pair<TriFaceType*,int> >(IntegerEdges.begin(),IntegerEdges.end());
        IntegerVertex=std::vector<TriVertexType* > (IntegerVertices.begin(),IntegerVertices.end());
    }

    void EraseIntegerEdge(const vcg::face::Pos<TriFaceType> &ep)
    {
        std::pair<TriFaceType*,int> edge(ep.F(),ep.E());
        assert(IntegerEdges.count(edge)!=0);
        IntegerEdges.erase(edge);
    }

    void EraseIntegerEdge(const std::vector<std::pair<TriFaceType*,int> > &to_erase)
    {
        for (unsigned int i=0;i<to_erase.size();i++)
            IntegerEdges.erase(to_erase[i]);
    }

    void TestIntegerEdges()
    {
        typedef typename std::pair<TriFaceType*,int> pair_type;
        typedef typename std::vector< pair_type > vect_type;
        typename vect_type::iterator IteIntl;
        for (IteIntl=IntegerLines.begin();
             IteIntl!=IntegerLines.end();
             IteIntl++)
        {
            int E=(*IteIntl).second;
            TriFaceType *F=(*IteIntl).first;
            TriFaceType *F1=F->FFp(E);
            if (F==F1) continue;
            int E1=F->FFi(E);
            std::pair<TriFaceType*,int> curr_edge(F1,E1);
            assert(IntegerEdges.count(curr_edge)!=0);
        }
    }

    void InitIntegerEdgesVert(TriMesh &Tmesh,
                              ScalarType factor=1.0,
                              ScalarType tolerance=0.0001)
    {
        IntegerEdges.clear();
        for (unsigned int i=0;i<Tmesh.face.size();i++)
        {
            TriFaceType *f=&Tmesh.face[i];
            if (f->IsD())continue;
            for (int j=0;j<3;j++)
            {
                TriFaceType *f1=f->FFp(j);
                int e1=f->FFi(j);
                bool IsBorder=f->IsB(j);
                TriVertexType *v0=f->V0(j);
                TriVertexType *v1=f->V1(j);
                vcg::Point2<ScalarType> uv0=f->WT(j).P()*factor;
                vcg::Point2<ScalarType> uv1=f->WT((j+1)%3).P()*factor;
                if (IsOnIntegerLine(uv0,uv1,tolerance)||IsBorder)
                {
                    //IntegerEdges.insert(std::pair<TriVertexType*,TriVertexType*>(v0,v1));
                    IntegerEdges.insert(std::pair<TriFaceType*,int>(f,j));
                    if (!IsBorder)
                        IntegerEdges.insert(std::pair<TriFaceType*,int>(f1,e1));
                    else
                    {
                        IntegerVertices.insert(v0);
                        IntegerVertices.insert(v1);
                    }
                }
                if (IsOnIntegerVertex(uv0))
                    IntegerVertices.insert(v0);

                if (IsOnIntegerVertex(uv1))
                    IntegerVertices.insert(v1);

            }
        }
        //InitIntegerNeigh(Tmesh);
        InitIntegerVectors();
        TestIntegerEdges();
    }


    ///return the first and the last edge
    ///following an integer line
    ///until if reach anothe integer edge
    bool OneIntegerStep(vcg::face::Pos<TriFaceType> &ep)
    {
        TriFaceType *f_init=ep.f;
        TriFaceType *currF=f_init;
        //int edge_init=ep.z;
        //ep.V()=f_init->V(edge_init);
        TriVertexType* v_init=ep.V();
        bool complete_turn=false;
        do
        {
            ep.FlipE();
            ///see if found an integer vert
            currF=ep.F();
            int currE=ep.E();
            assert((currE>=0)&&(currE<=4));

            std::pair<TriFaceType*,int> curr_edge(currF,currE);

            if (IntegerEdges.count(curr_edge)!=0)
            {
                ///go to the other side
                ep.FlipV();
                assert(ep.V()!=v_init);
                return true;
            }
            ep.FlipF();
            ///see if there's a border
            bool jumped=(currF==ep.F());
            if (jumped)
                return false;
            ///test the complete turn
            complete_turn=(ep.F()==f_init);
        }while (!complete_turn);
        return false;
    }

    ///find a polygon starting from half edge ep, return true if found
    bool FindPolygon(vcg::face::Pos<TriFaceType> &ep,
                     std::vector<TriVertexType*> &poly)
    {

        poly.clear();
        TriVertexType* v_init=ep.V();
        ///it must start from an integer vert
        assert(IntegerVertices.count(v_init)!=0);
        poly.push_back(v_init);
        std::vector<std::pair<TriFaceType*,int> > to_erase;
        to_erase.push_back(std::pair<TriFaceType*,int>(ep.F(),ep.E()));
        do
        {
            bool done=OneIntegerStep(ep);
            if (!done)
            {
                EraseIntegerEdge(to_erase);
                return false;
            }
            to_erase.push_back(std::pair<TriFaceType*,int>(ep.F(),ep.E()));
            TriVertexType* v_curr=ep.V();
            if ((IntegerVertices.count(v_curr)!=0)&&
                    (v_curr!=v_init))
                poly.push_back(v_curr);
        }while(ep.V()!=v_init);
        EraseIntegerEdge(to_erase);
        return true;
    }

    void FindPolygons(TriMesh &Tmesh,
                      std::vector<std::vector<TriVertexType *> > &polygons)
    {
        //int limit=2;
        for (unsigned int i=0;i<Tmesh.face.size();i++)
        {
            TriFaceType * f=&Tmesh.face[i];
            for (int j=0;j<3;j++)
            {
                TriVertexType* v0=f->V0(j);
                //TriVertexType* v1=f->V1(j);

                //std::pair<TriVertexType*,TriVertexType*> edge(v0,v1);*/
                std::pair<TriFaceType*,int> edge(f,j);
                if (IntegerEdges.count(edge)==0)continue;///edge already used or not integer
                if (IntegerVertices.count(v0)==0)continue;	///must start from integer vert
                ///create the pos
                vcg::face::Pos<TriFaceType> ep(f,j);
                std::vector<TriVertexType *> poly;

                bool found=FindPolygon(ep,poly);
                if (found)
                {
                    std::reverse(poly.begin(),poly.end());///REVERSE ORDER
                    polygons.push_back(poly);
                }
            }
        }

    }

    void InitVertexQuadMesh(TriMesh &Tmesh)
    {
        FindPolygons(Tmesh,polygons);
    }

public:


    void TestIsProper(TriMesh &Tmesh)
    {
        ///test manifoldness
        int test=vcg::tri::Clean<TriMesh>::CountNonManifoldVertexFF(Tmesh);
        //assert(test==0);
        if (test != 0)
            cerr << "Assertion failed: TestIsProper NonManifoldVertices!" << endl;

        test=vcg::tri::Clean<TriMesh>::CountNonManifoldEdgeFF(Tmesh);
        //assert(test==0);
        if (test != 0)
            cerr << "Assertion failed: TestIsProper NonManifoldEdges" << endl;

        for (unsigned int i=0;i<Tmesh.face.size();i++)
        {
            TriFaceType *f=&Tmesh.face[i];
            assert (!f->IsD());
            for (int z=0;z<3;z++)
            {
                //int indexOpp=f->FFi(z);
                TriFaceType *Fopp=f->FFp(z);
                if (Fopp==f) continue;
                //assert( f->FFp(z)->FFp(f->FFi(z))==f );
                if (f->FFp(z)->FFp(f->FFi(z))!=f)
                    cerr << "Assertion failed: TestIsProper f->FFp(z)->FFp(f->FFi(z))!=f " << endl;
            }
        }
    }

    void Quadrangulate(TriMesh &Tmesh,
                       PolyMesh &Pmesh,
                       ScalarType factor=1.0,
                       ScalarType tolerance=0.000001)
    {
        TestIsProper(Tmesh);
        vcg::tri::AttributeSeam::SplitVertex(Tmesh, ExtractVertex<TriMesh>, CompareVertex<TriMesh>);
        vcg::tri::Allocator<TriMesh>::CompactVertexVector(Tmesh);
        vcg::tri::Allocator<TriMesh>::CompactFaceVector(Tmesh);
        vcg::tri::UpdateTopology<TriMesh>::FaceFace(Tmesh);
        (void)Pmesh;
        TestIsProper(Tmesh);

        ///then split the tris
        SplitTris(Tmesh,factor,tolerance);
        ///join the vertices back!
        //ScalarType EPS=(ScalarType)0.00000001;
        ScalarType EPS=(ScalarType)0.000001;
        vcg::tri::Clean<TriMesh>::MergeCloseVertex(Tmesh,EPS);

        vcg::tri::UpdateNormal<TriMesh>::PerFaceNormalized(Tmesh);	 // update Normals
        vcg::tri::UpdateNormal<TriMesh>::PerVertexNormalized(Tmesh);// update Normals
        ///compact the mesh
        vcg::tri::Allocator<TriMesh>::CompactVertexVector(Tmesh);
        vcg::tri::Allocator<TriMesh>::CompactFaceVector(Tmesh);
        vcg::tri::UpdateTopology<TriMesh>::FaceFace(Tmesh);			 // update Topology
        vcg::tri::UpdateTopology<TriMesh>::TestFaceFace(Tmesh);		 //and test it
        ///set flags
        vcg::tri::UpdateFlags<TriMesh>::VertexClearV(Tmesh);
        vcg::tri::UpdateFlags<TriMesh>::FaceBorderFromFF(Tmesh);
        vcg::tri::UpdateFlags<TriMesh>::VertexBorderFromFace(Tmesh);
        ///test manifoldness
        TestIsProper(Tmesh);

        vcg::tri::UpdateFlags<TriMesh>::VertexClearV(Tmesh);

        InitIntegerEdgesVert(Tmesh,factor,tolerance);
        InitVertexQuadMesh(Tmesh);

        ///then add to the polygonal mesh
        Pmesh.Clear();
        ///first create vertices
        vcg::tri::Allocator<PolyMesh>::AddVertices(Pmesh,IntegerVertex.size());
        std::map<TriVertexType*,int> VertMap;
        for(unsigned int i=0;i<IntegerVertex.size();i++)
        {
            CoordType pos=IntegerVertex[i]->P();
            CoordType norm=IntegerVertex[i]->N();
            Pmesh.vert[i].P()=typename PolyMesh::CoordType(pos.X(),pos.Y(),pos.Z());
            Pmesh.vert[i].N()=typename PolyMesh::CoordType(norm.X(),norm.Y(),norm.Z());
            VertMap[IntegerVertex[i]]=i;
        }
        ///then add polygonal mesh
        vcg::tri::Allocator<PolyMesh>::AddFaces(Pmesh,polygons.size());
        for (unsigned int i=0;i<polygons.size();i++)
        {
            int size=polygons[i].size();
            Pmesh.face[i].Alloc(size);
            for (int j=0;j<size;j++)
            {
                TriVertexType* v=polygons[i][j];
                int index=VertMap[v];
                Pmesh.face[i].V(j)=&(Pmesh.vert[index]);
            }
        }
    }
};

#endif
