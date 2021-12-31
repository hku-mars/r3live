#ifndef MIQ_STIFFENING
#define MIQ_STIFFENING


template <class ScalarType>
ScalarType Gauss(ScalarType &value)
{
    const ScalarType E_NEPER=2.71828;
    const ScalarType den=sqrt(2.0*M_PI);
    ScalarType exponent=-0.5*pow(value,2);
    ScalarType res=((1.0/den)*pow(E_NEPER,exponent));
    return res;
}

template <class MeshType>
class StiffeningInitializer
{
    typedef typename MeshType::ScalarType ScalarType;
    typedef typename MeshType::FaceType FaceType;
    typedef typename MeshType::VertexType VertexType;
    typedef typename MeshType::CoordType CoordType;

public:

    static void colorByStiffening(MeshType & mesh,
                                  typename MeshType::ScalarType MaxVal=16)
    {
        bool hasStiffness = vcg::tri::HasPerFaceAttribute(mesh,std::string("Stiffness"));
        assert(hasStiffness);
        typename MeshType::template PerFaceAttributeHandle<float> Handle_Stiffness=vcg::tri::Allocator<MeshType>::template GetPerFaceAttribute<float>(mesh,std::string("Stiffness"));

        for (unsigned int i=0;i<mesh.face.size();i++)
        {
            //MeshType::ScalarType val=MaxVal-mesh.face[i].stiffening+1;
            ScalarType val=MaxVal-Handle_Stiffness[i]+1;
            if (val<1)val=1;
            mesh.face[i].C()=vcg::Color4b::ColorRamp(1.0,MaxVal,val);
        }
    }

    static void AddGaussStiffening(MeshType & mesh,ScalarType C)
    {
        int radius=floor(C);
        if (C<4)radius=4;
        typename MeshType::template PerFaceAttributeHandle<float> Handle_Stiffness;

        bool hasStiffness = vcg::tri::HasPerFaceAttribute(mesh,std::string("Stiffness"));
        if(!hasStiffness)
            Handle_Stiffness=vcg::tri::Allocator<MeshType>::template AddPerFaceAttribute<float>(mesh,std::string("Stiffness"));
        else
            Handle_Stiffness=vcg::tri::Allocator<MeshType>::template FindPerFaceAttribute<float>(mesh,std::string("Stiffness"));

        bool hasSingular = vcg::tri::HasPerVertexAttribute(mesh,std::string("Singular"));
        assert(hasSingular);

        typename MeshType::template PerVertexAttributeHandle<bool> Handle_Singular;
        Handle_Singular=vcg::tri::Allocator<MeshType>:: template GetPerVertexAttribute<bool>(mesh,std::string("Singular"));

        std::vector<VertexType*> to_stiff;
        for(unsigned int i=0;i<mesh.vert.size();i++)
        {
            VertexType *v=&mesh.vert[i];
            if (v->IsD())continue;
            //if (!v->IsSingular())continue;
            if (!Handle_Singular[v])continue;
            to_stiff.push_back(v);
        }
        for(unsigned int i=0;i<mesh.face.size();i++)
        {

            FaceType *f=&(mesh.face[i]);
            if (f->IsD())continue;
            if (!f->IsV())continue;
            to_stiff.push_back(f->V(0));
            to_stiff.push_back(f->V(1));
            to_stiff.push_back(f->V(2));
        }
        std::sort(to_stiff.begin(),to_stiff.end());
        typename std::vector<VertexType*>::iterator new_end;
        new_end=std::unique(to_stiff.begin(),to_stiff.end());
        int dist=distance(to_stiff.begin(),new_end);
        to_stiff.resize(dist);
        for (unsigned int i=0;i<to_stiff.size();i++)
        {
            VertexType *v=to_stiff[i];
            for (int r=0;r<radius;r++)
            {
                ScalarType stiffVal=((ScalarType)r)/(ScalarType)radius;//((ScalarType)(radius-r))/(ScalarType)radius;
                stiffVal*=3.0;
                stiffVal=Gauss(stiffVal)/0.4;
                stiffVal=1+(stiffVal*C);
                std::vector<FaceType*> ring;
                //mesh.GetConnectedFaces(v,r,ring);
                VFExtendedStarVF(v,r,ring);
                ///then set stiffening
                for (unsigned int k=0;k<ring.size();k++)
                {
                    FaceType* f=ring[k];
                    //if (f->stiffening<stiffVal)
                    //    f->stiffening=stiffVal;
                    if (Handle_Stiffness[f]<stiffVal)
                        Handle_Stiffness[f]=stiffVal;
                }
            }
        }
    }

    static bool updateStiffeningJacobianDistorsion(MeshType & mesh,ScalarType grad_size)
    {

        typename MeshType::template PerFaceAttributeHandle<float> Handle_Stiffness;

        bool hasStiffness = vcg::tri::HasPerFaceAttribute(mesh,std::string("Stiffness"));
        if(!hasStiffness)
            Handle_Stiffness=vcg::tri::Allocator<MeshType>::template AddPerFaceAttribute<float>(mesh,std::string("Stiffness"));
        else
            Handle_Stiffness=vcg::tri::Allocator<MeshType>::template FindPerFaceAttribute<float>(mesh,std::string("Stiffness"));

        bool flipped = NumFlips(mesh)>0;
        //if (h == 0.0)
        //    return flipped;
        //
        //assert(h != 0.0);

        if (!flipped)
            return false;
       ScalarType maxL=0;
       ScalarType maxD=0;
        if (flipped)
        {
            const double c = 1.0;
            const double d = 5.0;

            for (unsigned int i = 0; i < mesh.face.size(); ++i)
            {
                ScalarType dist=Distortion(mesh.face[i],grad_size);
                if (dist>maxD)maxD=dist;
                ScalarType absLap=fabs(LaplaceDistortion(mesh.face[i], grad_size));
                if (absLap>maxL)maxL=absLap;
                ScalarType stiffDelta = std::min(c * absLap, d);
                //mesh.face[i].stiffening+=stiffDelta;
                Handle_Stiffness[i]+=stiffDelta;
            }
        }
        printf("Maximum Distorsion %4.4f \n",maxD);
        printf("Maximum Laplacian %4.4f \n",maxL);
        return flipped;
    }

    static void InitDefaultStiffening(MeshType & mesh)
    {
        typename MeshType::template PerFaceAttributeHandle<float> Handle_Stiffness;

        bool hasStiffness = vcg::tri::HasPerFaceAttribute(mesh,std::string("Stiffness"));
        if(!hasStiffness)
            Handle_Stiffness=vcg::tri::Allocator<MeshType>::template AddPerFaceAttribute<float>(mesh,std::string("Stiffness"));
        else
            Handle_Stiffness=vcg::tri::Allocator<MeshType>::template FindPerFaceAttribute<float>(mesh,std::string("Stiffness"));

        for(unsigned int i=0;i<mesh.face.size();i++)
        {
            FaceType *f=&(mesh.face[i]);
            //f->stiffening=1;
            Handle_Stiffness[f]=1;
        }
    }

};

#endif
