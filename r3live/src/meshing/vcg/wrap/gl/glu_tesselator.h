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
#ifndef __VCGLIB_GLU_TESSELATOR_H
#define __VCGLIB_GLU_TESSELATOR_H
#include <vcg/space/point2.h>
#include <vector>

#ifndef GL_VERSION_1_1
#error "Please include OpenGL before including this file"
#endif


// The inclusion of glu should be always safe (if someone has already included gl stuff).
#ifndef GLU_VERSIONS
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#ifdef _WIN32
  #include <windows.h>
#endif
#include <GL/glu.h>
#endif
#endif

#ifndef CALLBACK
#ifdef _WIN32
#define CALLBACK __stdcall
#else
#define CALLBACK
#endif
#endif

namespace vcg
{

class glu_tesselator
{
    public:

        typedef glu_tesselator this_type;

        /*
            Works with Point2 and Point3;

            sample usage:

            // tesselation input: each outline represents a polygon contour
            std::vector< std::vector<point_type> > outlines = ...;

            // tesselation output (triangles indices)
            std::vector<int> indices;

            // compute triangles indices
            glu_tesselator::tesselate(outlines, indices);

            // unroll input contours points
            std::vector<point_type> points;

            for (size_t i=0; i<outlines.size(); ++i)
            {
                for (size_t j=0; j<outlines[i].size(); ++j)
                {
                    points.push_back(outlines[i][j]);
                }
            }
            // or simply call glu_tesselator::unroll(outlines, points);

            // create triangles
            for (size_t i=0; i<indices.size(); i+=3)
            {
                create_triangle(
                    points[ indices[i+0] ],
                    points[ indices[i+1] ],
                    points[ indices[i+2] ]);
            }
        */

        template <class point_type>
        static inline void unroll(const std::vector< std::vector<point_type> > & outlines, std::vector<point_type> & points)
        {
            for (size_t i=0; i<outlines.size(); ++i)
            {
                for (size_t j=0; j<outlines[i].size(); ++j)
                {
                    points.push_back(outlines[i][j]);
                }
            }
        }

        template <class point_type>
        static inline void tesselate(const std::vector< std::vector<point_type> > & outlines, std::vector<int> & indices)
        {
            tess_prim_data_vec t_data;

            this_type::do_tesselation(outlines, t_data);

            //int k = 0;
            for (size_t i=0; i<t_data.size(); ++i)
            {
                const size_t st = t_data[i].indices.size();
                if (st < 3) continue;

                switch (t_data[i].type)
                {
                    case GL_TRIANGLES:
                        for (size_t j=0; j<st; ++j)
                        {
                            indices.push_back(t_data[i].indices[j]);
                        }
                        break;

                    case GL_TRIANGLE_STRIP:
                        {
                        int i0 = t_data[i].indices[0];
                        int i1 = t_data[i].indices[1];

                        bool ccw = true;

                        for (size_t j=2; j<st; ++j)
                        {
                            const int i2 = t_data[i].indices[j];

                            indices.push_back(i0);
                            indices.push_back(i1);
                            indices.push_back(i2);

                            if (ccw) i0 = i2;
                            else     i1 = i2;

                            ccw = !ccw;
                        }																	}
                        break;

                    case GL_TRIANGLE_FAN:
                        {
                        const int first = t_data[i].indices[0];
                        int prev = t_data[i].indices[1];

                        for (size_t j=2; j<st; ++j)
                        {
                            const int curr = t_data[i].indices[j];

                            indices.push_back(first);
                            indices.push_back(prev);
                            indices.push_back(curr);

                            prev = curr;
                        }
                        }
                        break;

                    default:
                        break;
                }
            }
        }

    protected:

        class tess_prim_data
        {
            public:

                typedef tess_prim_data this_type;

                GLenum type;
                std::vector<int> indices;

                tess_prim_data(void) { }
                tess_prim_data(GLenum t) : type(t) { }
        };

        typedef std::vector<tess_prim_data> tess_prim_data_vec;

        static void CALLBACK begin_cb(GLenum type, void * polygon_data)
        {
            tess_prim_data_vec * t_data = (tess_prim_data_vec *)polygon_data;
            t_data->push_back(tess_prim_data(type));
        }

        static void CALLBACK end_cb(void * polygon_data)
        {
            (void)polygon_data;
        }

        static void CALLBACK vertex_cb(void * vertex_data, void * polygon_data)
        {
            tess_prim_data_vec * t_data = (tess_prim_data_vec *)polygon_data;
            t_data->back().indices.push_back((int)((size_t)vertex_data));
        }

        template <class point_type>
        static void do_tesselation(const std::vector< std::vector<point_type> > & outlines, tess_prim_data_vec & t_data)
        {
            GLUtesselator * tess = gluNewTess();
//#ifdef __APPLE__
//			gluTessCallback(tess, GLU_TESS_BEGIN_DATA,  (GLvoid (CALLBACK *)(...))(this_type::begin_cb));
//			gluTessCallback(tess, GLU_TESS_END_DATA,    (GLvoid (CALLBACK *)(...))(this_type::end_cb));
//			gluTessCallback(tess, GLU_TESS_VERTEX_DATA, (GLvoid (CALLBACK *)(...))(this_type::vertex_cb));
//#else
            gluTessCallback(tess, GLU_TESS_BEGIN_DATA,  (GLvoid (CALLBACK *)())(this_type::begin_cb));
            gluTessCallback(tess, GLU_TESS_END_DATA,    (GLvoid (CALLBACK *)())(this_type::end_cb));
            gluTessCallback(tess, GLU_TESS_VERTEX_DATA, (GLvoid (CALLBACK *)())(this_type::vertex_cb));
//#endif
            void * polygon_data = (void *)(&t_data);

            GLdouble vertex[3];

            size_t k = 0;
            gluTessBeginPolygon(tess, polygon_data);
            for (size_t i=0; i<outlines.size(); ++i)
            {
                gluTessBeginContour(tess);
                for (size_t j=0; j<outlines[i].size(); ++j)
                {
                    this_type::get_position(outlines[i][j], vertex);
                    gluTessVertex(tess, vertex, (void *)k);
                    ++k;
                }
                gluTessEndContour(tess);
            }
            gluTessEndPolygon(tess);

            gluDeleteTess(tess);
        }

        template <class scalar_type>
        static inline void get_position(const vcg::Point2<scalar_type> & p, GLdouble * d)
        {
            d[0] = (GLdouble)(p[0]);
            d[1] = (GLdouble)(p[1]);
            d[2] = (GLdouble)(0);
        }

        template <class scalar_type>
        static inline void get_position(const vcg::Point3<scalar_type> & p, GLdouble * d)
        {
            d[0] = (GLdouble)(p[0]);
            d[1] = (GLdouble)(p[1]);
            d[2] = (GLdouble)(p[2]);
        }
};

} // end namespace vcg

#endif // __VCGLIB_GLU_TESSELATOR_H
