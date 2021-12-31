/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2004-2012                                           \/)\/    *
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
#ifndef VCG_TRI_ATTRIBUTE_SEAM_H
#define VCG_TRI_ATTRIBUTE_SEAM_H

/*

// sample extract functor
void v_extract(const src_mesh_t & wm, const src_face_t & f, int k, const dst_mesh_t & vm, dst_vertex_t & v)
{
    (void)wm;
    (void)vm;

    v.P() = f.cP (k);
    v.N() = f.cWN(k);
    v.C() = f.cWC(k);
    v.T() = f.cWT(k);
}

// sample compare functor
bool v_compare(const dst_mesh_t & vm, const dst_vertex_t & u, const dst_vertex_t & v)
{
    (void)vm;

    return
    (
           (u.cN() == v.cN())
        && (u.cC() == v.cC())
        && (u.cT() == v.cT())
    );
}

// sample copy functor
void v_copy(const dst_mesh_t & vm, const dst_vertex_t & u, dst_vertex_t & v)
{
    (void)vm;

    v.P() = u.cP();
    v.N() = u.cN();
    v.C() = u.cC();
    v.T() = u.cT();
}

// create seams
AttributeSeam::SplitVertex(src, dst, v_extract, v_compare, v_copy, 1.10f);

*/

namespace vcg
{

namespace tri
{

class AttributeSeam
{
    public:

        typedef AttributeSeam ThisType;

        enum ASMask
        {
            POSITION_PER_VERTEX = (1 << 0),

            NORMAL_PER_VERTEX   = (1 << 1),
            NORMAL_PER_WEDGE    = (1 << 2),
            NORMAL_PER_FACE     = (1 << 3),

            COLOR_PER_VERTEX    = (1 << 4),
            COLOR_PER_WEDGE     = (1 << 5),
            COLOR_PER_FACE      = (1 << 6),

            TEXCOORD_PER_VERTEX = (1 << 7),
            TEXCOORD_PER_WEDGE  = (1 << 8)
        };

        template <typename src_trimesh_t, typename dst_trimesh_t>
        struct ASExtract
        {
            const unsigned int mask;

            ASExtract(unsigned int vmask = 0) : mask(vmask)
            {
                ;
            }

            void operator () (const src_trimesh_t & sm, const typename src_trimesh_t::FaceType & f, int k, const dst_trimesh_t & dm, typename dst_trimesh_t::VertexType & v) const
            {
                (void)sm;
                (void)dm;

                const unsigned int m = this->mask;
                const typename src_trimesh_t::VertexType & u = *(f.cV(k));

                if ((m & AttributeSeam::POSITION_PER_VERTEX) != 0) v.P() = f.cP (k);

                if ((m & AttributeSeam::NORMAL_PER_VERTEX)   != 0) v.N() = u.cN ( );
                if ((m & AttributeSeam::NORMAL_PER_WEDGE)    != 0) v.N() = f.cWN(k);
                if ((m & AttributeSeam::NORMAL_PER_FACE)     != 0) v.N() = f.cN ( );

                if ((m & AttributeSeam::COLOR_PER_VERTEX)    != 0) v.C() = u.cC ( );
                if ((m & AttributeSeam::COLOR_PER_WEDGE)     != 0) v.C() = f.cWC(k);
                if ((m & AttributeSeam::COLOR_PER_FACE)      != 0) v.C() = f.cC ( );

                if ((m & AttributeSeam::TEXCOORD_PER_VERTEX) != 0) v.T() = u.cT ( );
                if ((m & AttributeSeam::TEXCOORD_PER_WEDGE)  != 0) v.T() = f.cWT(k);
            }
        };

        template <typename dst_trimesh_t>
        struct ASCompare
        {
            const unsigned int mask;

            ASCompare(unsigned int vmask = 0) : mask(vmask)
            {
                ;
            }

            bool operator () (const dst_trimesh_t & sm, const typename dst_trimesh_t::VertexType & u, const typename dst_trimesh_t::VertexType & v) const
            {
                (void)sm;

                const unsigned int m = this->mask;

                /*
                if ((m & (AttributeSeam::POSITION_PER_VERTEX)) != 0)
                {
                    if (u.cP() != v.cP()) return false;
                }
                */

                if ((m & (AttributeSeam::NORMAL_PER_VERTEX | AttributeSeam::NORMAL_PER_WEDGE | AttributeSeam::NORMAL_PER_FACE)) != 0)
                {
                    if (u.cN() != v.cN()) return false;
                }

                if ((m & (AttributeSeam::COLOR_PER_VERTEX | AttributeSeam::COLOR_PER_WEDGE | AttributeSeam::COLOR_PER_FACE)) != 0)
                {
                    if (u.cC() != v.cC()) return false;
                }

                if ((m & (AttributeSeam::TEXCOORD_PER_VERTEX | AttributeSeam::TEXCOORD_PER_WEDGE)) != 0)
                {
                    if (u.cT() != v.cT()) return false;
                }

                return true;
            }
        };

        // in-place version
        template <typename src_trimesh_t, typename extract_wedge_attribs_t, typename compare_vertex_attribs_t>
        static inline bool SplitVertex(src_trimesh_t & src, extract_wedge_attribs_t v_extract, compare_vertex_attribs_t & v_compare)
        {
            typedef typename src_trimesh_t::VertexType      src_vertex_t;
            typedef typename src_trimesh_t::VertexIterator  src_vertex_i;
            typedef typename src_trimesh_t::FaceType        src_face_t;
            typedef typename src_trimesh_t::FaceIterator    src_face_i;
            typedef typename src_trimesh_t::VertContainer   src_vertex_container_t;

            typedef vcg::tri::Allocator<src_trimesh_t>      src_mesh_allocator_t;
            typedef typename src_mesh_allocator_t :: template PointerUpdater<typename src_trimesh_t::VertexPointer> src_pointer_updater_t;

            if ((src.vn <= 0) || (src.fn <= 0))
            {
                return true;
            }

            src_pointer_updater_t pt_upd;
            src_vertex_i   vi  = src_mesh_allocator_t::AddVertices(src, 1, pt_upd);
            src_vertex_t * vtx = &(*vi);
            src_vertex_t * vtxbase = &(src.vert[0]);

            const size_t vertex_count     = src.vert.size();
            const size_t vertex_pool_size = vertex_count;

            std::vector<int> vloc;
            vloc.reserve(vertex_pool_size);
            vloc.resize(vertex_count, -2);

            int vcount = int(src.vert.size());
            int idx    = 0;

            for (src_face_i it=src.face.begin(); it!=src.face.end(); ++it)
            {
                src_face_t & f = (*it);
                if (f.IsD()) continue;

                for (int k=0; k<3; ++k)
                {
                    idx = (f.cV(k) - vtxbase);
                    v_extract(src, f, k, src, *vtx);

                    if (vloc[idx] == -2)
                    {
                        vloc[idx] = -1;
                        src.vert[idx].ImportData(*vtx);
                    }
                    else
                    {
                        int vidx = idx;
                        do
                        {
                            if (v_compare(src, src.vert[vidx], *vtx)) break;
                            vidx = vloc[vidx];
                        } while (vidx >= 0);

                        if (vidx < 0)
                        {
                            vloc.push_back(vloc[idx]);
                            vloc[idx] = vcount;

                            vi = src_mesh_allocator_t::AddVertices(src, 1, pt_upd);
                            pt_upd.Update(vtx);
                            pt_upd.Update(vtxbase);

                            (*vi).ImportData(*vtx);

                            idx = vcount;
                            vcount++;
                        }
                        else
                        {
                            idx = vidx;
                        }
                    }

                    f.V(k) = &(src.vert[idx]);
                }
            }

            src_mesh_allocator_t::DeleteVertex(src, *vtx);

            return true;
        }

        // out-of-place version
        template <typename src_trimesh_t, typename dst_trimesh_t, typename extract_wedge_attribs_t, typename compare_vertex_attribs_t, typename copy_vertex_t>
        static inline bool SplitVertex(const src_trimesh_t & src, dst_trimesh_t & dst, extract_wedge_attribs_t & v_extract, compare_vertex_attribs_t & v_compare, copy_vertex_t & v_copy)
        {
            typedef typename src_trimesh_t::VertexType           src_vertex_t;
            typedef typename src_trimesh_t::FaceType             src_face_t;
            typedef typename src_trimesh_t::ConstFaceIterator    src_face_ci;

            typedef typename dst_trimesh_t::VertContainer        dst_vertex_container_t;
            typedef typename dst_trimesh_t::VertexType           dst_vertex_t;
            typedef typename dst_trimesh_t::VertexIterator       dst_vertex_i;
            typedef typename dst_trimesh_t::FaceType             dst_face_t;
            typedef typename dst_trimesh_t::FaceIterator         dst_face_i;

            typedef vcg::tri::Allocator<dst_trimesh_t>           dst_mesh_allocator_t;

            /* GCC gets in troubles and need some hints ("template") to parse the following line */
            typedef typename dst_mesh_allocator_t :: template PointerUpdater<typename dst_trimesh_t::VertexPointer> dst_pointer_updater_t;

            if (reinterpret_cast<const void *>(&src) == reinterpret_cast<const void *>(&dst))
            {
                return false;
            }

            dst.Clear();

            if ((src.vn <= 0) || (src.fn <= 0))
            {
                return true;
            }

            const size_t vertex_count     = src.vert.size();
            const size_t vertex_pool_size = vertex_count;

            const src_vertex_t * vtxbase = &(src.vert[0]);

            std::vector<int> vloc;
            vloc.reserve(vertex_pool_size);
            vloc.resize(vertex_count, -2);

            dst_vertex_i vv;
            dst_pointer_updater_t pt_upd;
            pt_upd.preventUpdateFlag = true;
            dst_mesh_allocator_t::AddVertices(dst, 1 + int(vertex_count), pt_upd);
            dst_vertex_t * vtx = &(dst.vert[0]);

            dst_face_i fbase = dst_mesh_allocator_t::AddFaces(dst, src.fn);
            dst_face_i fi    = fbase;

            int vcount = int(dst.vert.size());
            int idx    = 0;

            for (src_face_ci it=src.face.begin(); it!=src.face.end(); ++it)
            {
                const src_face_t & wf = (*it);
                if (wf.IsD()) continue;

                dst_face_t & vf = (*fi);

                for (int k=0; k<3; ++k)
                {
                    idx = (wf.cV(k) - vtxbase);

                    v_extract(src, wf, k, dst, *vtx);

                    if (vloc[idx] == -2)
                    {
                        vloc[idx] = -1;
                        v_copy(dst, *vtx, dst.vert[idx]);
                    }
                    else
                    {
                        int vidx = idx;
                        do
                        {
                            if (v_compare(dst, dst.vert[vidx], *vtx)) break;
                            vidx = vloc[vidx];
                        } while (vidx >= 0);

                        if (vidx < 0)
                        {
                            vloc.push_back(vloc[idx]);
                            vloc[idx] = vcount;

                            vv = dst_mesh_allocator_t::AddVertices(dst, 1, pt_upd);
                            pt_upd.Update(vtx);
                            v_copy(dst, *vtx, *vv);

                            idx = vcount;
                            vcount++;
                        }
                        else
                        {
                            idx = vidx;
                        }
                    }

                    vf.V(k) = reinterpret_cast<dst_vertex_t *>(idx);
                }

                fi++;
            }

            {
                std::vector<int> tmp;
                vloc.swap(tmp);
            }

            dst_vertex_t * vstart = &(dst.vert[0]);

            for (dst_face_i it=fbase; it!=dst.face.end(); ++it)
            {
                dst_face_t & vf = (*it);

                vf.V(0) = vstart + reinterpret_cast<const int>(vf.V(0));
                vf.V(1) = vstart + reinterpret_cast<const int>(vf.V(1));
                vf.V(2) = vstart + reinterpret_cast<const int>(vf.V(2));
            }

            dst_mesh_allocator_t::DeleteVertex(dst, *vtx);

            return true;
        }
};

} // end namespace tri

} // end namespace vcg

#endif // VCG_TRI_ATTRIBUTE_SEAM_H
