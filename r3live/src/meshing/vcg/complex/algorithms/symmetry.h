#ifndef VCG_SYMMETRY_H
#define VCG_SYMMETRY_H

#include <vcg/space/plane3.h>
#include <vcg/space/index/grid_static_ptr.h>
#include <vcg/complex/algorithms/closest.h>
#include <vcg/complex/algorithms/create/platonic.h>
#include <vcg/complex/algorithms/point_sampling.h>

namespace vcg {
namespace tri {

template <class TriMeshType>
class ExtrinsicPlaneSymmetry
{
    typedef typename TriMeshType::VertexType VertexType;
    typedef typename TriMeshType::FaceType FaceType;
    typedef typename TriMeshType::CoordType CoordType;
    typedef typename TriMeshType::ScalarType ScalarType;

    TriMeshType &tri_mesh;

    CoordType AlignZeroTr;

    std::vector<std::vector< ScalarType > > Weight;
    //std::vector<std::vector<std::pair<VertexType*,VertexType*> > > VotingVertx;
    std::vector<std::vector<std::pair<CoordType,CoordType>  > > VotingPos;

    std::vector<ScalarType> Votes;

    TriMeshType *sphere;
    typename vcg::GridStaticPtr<FaceType> GridSph;

    ScalarType RadiusInterval;
    ScalarType MaxRadius;
    int radiusSph;

    std::vector<std::pair<ScalarType,int> > SortedPlanes;
    std::vector<vcg::Plane3<ScalarType> > SymmetricPlanes;

    int Bucket(const vcg::Plane3<ScalarType> &Pl)
    {
        ScalarType Offset=Pl.Offset();
        CoordType Direction=Pl.Direction();
        Direction.Normalize();
        ///get the offset interval
        int OffsetI=floor((Offset/RadiusInterval)+0.5);
        assert(OffsetI<radiusSph);
        ///then get the closest face
        ScalarType MaxD=sphere->bbox.Diag();
        ScalarType MinD;
        CoordType ClosePt;
        FaceType *choosen=NULL;
        choosen=vcg::tri::GetClosestFaceBase(*sphere,GridSph,Direction,MaxD,MinD,ClosePt);
        assert(choosen!=NULL);
        int IndexF=choosen-&(sphere->face[0]);
        ///compose the final index
        int OffsetRadius=OffsetI * (sphere->face.size());
        return(OffsetRadius+IndexF);
    }

    void Elect(CoordType p0,CoordType p1)
    {
        CoordType AvP=(p0+p1)/2.0;
        AvP-=AlignZeroTr;
        CoordType Direction=p0-p1;
        Direction.Normalize();
        vcg::Plane3<ScalarType> Pl;
        Pl.Init(AvP,Direction);
        //invert if needed
        if (Pl.Offset()<0)
        {
            ScalarType Off=Pl.Offset();
            CoordType Dir=Pl.Direction();
            Pl.Set(-Dir,-Off);
        }
        int index=Bucket(Pl);
        assert(index>=0);
        assert(index<(int)Votes.size());
        ScalarType VoteValue=1;
        Votes[index]+=VoteValue;
        Weight[index].push_back(VoteValue);
        VotingPos[index].push_back(std::pair<CoordType,CoordType> (p0,p1));
    }

    vcg::Plane3<ScalarType> GetInterpolatedPlane(int &Index)
    {
        ///then fit the plane
        CoordType AvDir=CoordType(0,0,0);
        ScalarType WSum=0;
        ScalarType AvOffset=0;
        for (size_t i=0;i<VotingPos[Index].size();i++)
        {
            CoordType p0=VotingPos[Index][i].first;
            CoordType p1=VotingPos[Index][i].second;
            ScalarType W=Weight[Index][i];
            CoordType Dir=(p0-p1);
            Dir.Normalize();

            CoordType AvP=(p0+p1)/2.0;
            ScalarType Offset=AvP*Dir;

            //invert if needed
            if (Offset<0)
            {
                Offset=-Offset;
                Dir=-Dir;
            }

            AvDir+=(Dir*W);
            AvOffset+=(Offset*W);
            WSum+=W;
        }
        AvDir.Normalize();
        AvOffset/=WSum;
        vcg::Plane3<ScalarType> Pl;
        Pl.Set(AvDir,AvOffset);
        return Pl;
    }

    vcg::Plane3<ScalarType> GetBasePlane(int &Index)
    {
        ///get offset value
        int OffsetIndex=Index/sphere->face.size();
        ScalarType OffsetVal=OffsetIndex*RadiusInterval;
        int DirectionIndex=Index % sphere->face.size();
        CoordType PlaneDirection=(sphere->face[DirectionIndex].P(0)+
                                  sphere->face[DirectionIndex].P(1)+
                                  sphere->face[DirectionIndex].P(2))/3.0;
        PlaneDirection.Normalize();
        CoordType CenterPl=PlaneDirection*OffsetVal;
        CenterPl+=AlignZeroTr;
        vcg::Plane3<ScalarType> RetPl;
        RetPl.Init(CenterPl,PlaneDirection);
        return RetPl;
    }

    void InitSymmetricPlanes(const int SubN=4)
    {
        assert(SortedPlanes.size()>0);
        SymmetricPlanes.clear();
        int BestN=pow((ScalarType)2,(ScalarType)SubN);
        if (BestN>=(int)SortedPlanes.size())BestN=SortedPlanes.size()-1;
        for (size_t j=SortedPlanes.size()-1;j>SortedPlanes.size()-1-SubN;j--)
        {
            int Index=SortedPlanes[j].second;
            SymmetricPlanes.push_back(GetInterpolatedPlane(Index));
        }
    }


public:

    void GetPlanes(std::vector<vcg::Plane3<ScalarType> > &Planes,int Num)
    {
        if (SymmetricPlanes.size()==0)return;

        for (int i=0;i<Num;i++)
            Planes.push_back(SymmetricPlanes[i]);
    }

    void Init(bool OnlyBorder=false,
              int SubDirections=4,
              int NumberBestPlanes=16)
    {
        if (OnlyBorder)
        {
            vcg::tri::UpdateTopology<TriMeshType>::FaceFace(tri_mesh);
            vcg::tri::UpdateFlags<TriMeshType>::FaceBorderFromFF(tri_mesh);
            vcg::tri::UpdateFlags<TriMeshType>::VertexBorderFromFace(tri_mesh);
        }
        AlignZeroTr=tri_mesh.bbox.Center();

        ///initialize the mesh
        if (sphere!=NULL)
            sphere->Clear();
        else
            sphere=new TriMeshType();

        //create the sphere
        vcg::tri::Sphere<TriMeshType>(*sphere,SubDirections);
        sphere->UpdateAttributes();

        ///initialize grid
        GridSph.Set(sphere->face.begin(),sphere->face.end());

        ///then get radius division steps
        ScalarType MaxRadius=tri_mesh.bbox.Diag()/2;
        int AreaSph=sphere->fn;
        radiusSph=ceil(sqrt((ScalarType)AreaSph/4.0*M_PI));
        RadiusInterval=MaxRadius/(ScalarType)radiusSph;

        ///and finally allocate space for votes
        Votes.resize(radiusSph*sphere->fn,0);

        Weight.resize(radiusSph*sphere->fn);
        VotingPos.resize(radiusSph*sphere->fn);

        ///then count votes
        for (size_t i=0;i<tri_mesh.vert.size();i++)
            for (size_t j=i+1;j<tri_mesh.vert.size();j++)
            {
                VertexType *v0=&tri_mesh.vert[i];
                VertexType *v1=&tri_mesh.vert[j];
                if ((OnlyBorder)&&(!((v0->IsB())&&(v1->IsB()))))continue;
                Elect(v0->P(),v1->P());
            }

        SortedPlanes.resize(Votes.size());
        for (size_t i=0;i<Votes.size();i++)
            SortedPlanes[i]=std::pair<ScalarType,int>(Votes[i],i);

        std::sort(SortedPlanes.begin(),SortedPlanes.end());

        InitSymmetricPlanes(NumberBestPlanes);

    }

    ExtrinsicPlaneSymmetry(TriMeshType &_tri_mesh):tri_mesh(_tri_mesh)
    {
        sphere=NULL;
        RadiusInterval=-1;
        radiusSph=-1;
    }

};

}///end namespace vcg
}///end namespace tri
#endif
