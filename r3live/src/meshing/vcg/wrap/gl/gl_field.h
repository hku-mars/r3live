#ifndef GL_FIELD
#define GL_FIELD

#include <vcg/complex/algorithms/parametrization/tangent_field_operators.h>

namespace vcg{
template <class MeshType>
class GLField
{
	typedef typename MeshType::FaceType FaceType;
	typedef typename MeshType::VertexType VertexType;
    typedef typename MeshType::CoordType CoordType;
	typedef typename MeshType::ScalarType ScalarType;
	
	static void GLDrawField(CoordType dir[4],
							CoordType center,
							ScalarType &size)
	{

        glLineWidth(2);
        vcg::glColor(vcg::Color4b(0,0,255,255));
        glBegin(GL_LINES);
            glVertex(center-dir[0]*size);
            glVertex(center+dir[0]*size);
        glEnd();

        glLineWidth(2);
        vcg::glColor(vcg::Color4b(0,255,0,255));
        glBegin(GL_LINES);
            glVertex(center-dir[1]*size);
            glVertex(center+dir[1]*size);
        glEnd();
        /*glLineWidth(1);
        vcg::glColor(vcg::Color4b(0,0,0,255));

		glBegin(GL_LINES);
        for (int i=1;i<4;i++)
		{            
            glVertex(center);
            glVertex(center+dir[i]*size);
		}
        glEnd();*/
	}


	///draw the cross field of a given face
    static void GLDrawFaceField(const FaceType &f,
							ScalarType &size)
	{
        CoordType center=(f.cP0(0)+f.cP0(1)+f.cP0(2))/3;
		CoordType normal=f.cN();
		CoordType dir[4];
		vcg::tri::CrossField<MeshType>::CrossVector(f,dir);
		GLDrawField(dir,center,size);
	}
	
//    static void GLDrawFaceSeams(const FaceType &f,
//                                vcg::Point3<bool> seams,
//                                vcg::Color4b seamCol[3])
//    {
//        glLineWidth(2);

//        glBegin(GL_LINES);
//        for (int i=0;i<3;i++)
//        {
//            if (!seams[i])continue;
//            vcg::glColor(seamCol[i]);
//            glVertex(f.V0(i)->P());
//            glVertex(f.V1(i)->P());
//        }
//        glEnd();
//    }

	static void GLDrawVertField(const MeshType &mesh,
								const VertexType &v,
								ScalarType &size)
	{
		CoordType center=v.cP();
		CoordType normal=v.cN();
		CoordType dir[4];
		vcg::tri::CrossField<MeshType>::CrossVector(v,dir);
		GLDrawField(dir,center,size);
	}

public:


	static void GLDrawFaceField(const MeshType &mesh)
	{

		glPushAttrib(GL_ALL_ATTRIB_BITS);
        glDepthRange(0.0,0.9999);
		glEnable(GL_COLOR_MATERIAL);
        glDisable(GL_LIGHTING);
        glDisable(GL_BLEND);
        ScalarType size=mesh.bbox.Diag()/400.0;
        for (unsigned int i=0;i<mesh.face.size();i++)
		{
			if (mesh.face[i].IsD())continue;
			//if (!mesh.face[i].leading)continue;
            GLDrawFaceField(mesh.face[i],size);
		}
		glPopAttrib();
	}

	static void GLDrawVertField(const MeshType &mesh)
	{
		glPushAttrib(GL_ALL_ATTRIB_BITS);
        glDepthRange(0.0,0.9999);
		glEnable(GL_COLOR_MATERIAL);
		glDisable(GL_LIGHTING);
        glDisable(GL_BLEND);
        ScalarType size=mesh.bbox.Diag()/400.0;
        for (int i=0;i<mesh.vert.size();i++)
		{
			if (mesh.vert[i].IsD())continue;
			//if (!mesh.face[i].leading)continue;
			GLDrawVertField(mesh,mesh.vert[i],size);
		}
		glPopAttrib();
	}
};

}

#endif
