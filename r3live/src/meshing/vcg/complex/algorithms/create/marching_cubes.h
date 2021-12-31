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
/***************************************************************************/

#ifndef __VCG_MARCHING_CUBES
#define __VCG_MARCHING_CUBES

#include "mc_lookup_table.h"

namespace vcg
{
    namespace tri
    {
        // Doxygen documentation
        /** \addtogroup trimesh */
        /*@{*/

        /*
        * Cube description:
        *         3 ________ 2           _____2__
        *         /|       /|         / |       /|
        *       /  |     /  |      11/  3   10/  |
        *   7 /_______ /    |      /__6_|__ /    |1
        *    |     |  |6    |     |     |  |     |
        *    |    0|__|_____|1    |     |__|__0__|
        *    |    /   |    /      7   8/   5    /
        *    |  /     |  /        |  /     |  /9
        *    |/_______|/          |/___4___|/
        *   4          5
        */

        //! This class implements the Marching Cubes algorithm.
        /*!
        *	The implementation is enough generic: this class works only on one volume cell for each
        *	call to <CODE>ProcessCell</CODE>. Using the field value at the cell corners, it adds to the
        *	mesh the triangles set approximating the surface that cross that cell. The ambiguities
        *	are resolved using an enhanced topologically controlled lookup table.
        *	@param TRIMESH_TYPE (Template parameter) the mesh type that will be constructed
        *	@param WALKER_TYPE	(Template parameter) the class that implement the traversal ordering of the volume
        **/
        template<class TRIMESH_TYPE, class WALKER_TYPE>
        class MarchingCubes
        {
        public:
            enum Dimension		 {X, Y, Z};

#if defined(__GNUC__)
            typedef unsigned int				size_t;
#else
#ifdef			_WIN64
            typedef unsigned __int64    size_t;
#else
            typedef _W64 unsigned int   size_t;
#endif
#endif
            typedef typename vcg::tri::Allocator< TRIMESH_TYPE > AllocatorType;
            typedef typename TRIMESH_TYPE::ScalarType			ScalarType;
            typedef typename TRIMESH_TYPE::VertexType			VertexType;
            typedef typename TRIMESH_TYPE::VertexPointer	VertexPointer;
            typedef typename TRIMESH_TYPE::VertexIterator	VertexIterator;
            typedef typename TRIMESH_TYPE::FaceType				FaceType;
            typedef typename TRIMESH_TYPE::FacePointer		FacePointer;
            typedef typename TRIMESH_TYPE::FaceIterator		FaceIterator;
            typedef typename TRIMESH_TYPE::CoordType			CoordType;
            typedef typename TRIMESH_TYPE::CoordType*			CoordPointer;

            /*!
            *	Constructor
            *	\param mesh		the mesh that will be constructed
            *	\param walker	the class implementing the traversal policy
            */
            MarchingCubes(TRIMESH_TYPE &mesh, WALKER_TYPE &walker)
            {
                _mesh		= &mesh;
                _walker	= &walker;
            };

            /*!
            *	Execute the initialiazation.
            *	This method must be executed before the first call to <CODE>ApplyMC</CODE>
            */
            void Initialize()
            {
                _mesh->Clear();
            }; // end of Initialize()

            /*!
            *
            *	This method must be executed after the last call to <CODE>ApplyMC</CODE>
            */
            void Finalize()
            {
                _mesh		= NULL;
                _walker = NULL;
            }; // end of Finalize()

            /*!
            * Apply the <I>marching cubes</I> algorithm to the volume cell identified by the two points <CODE>min</CODE> and <CODE>max</CODE>.
            *	All the three coordinates of the first point must be smaller than the respectives three coordinatas of the second point.
            *	\param min	the first point
            *	\param max	the second point
            */
            void ProcessCell(const vcg::Point3i &min, const vcg::Point3i &max)
            {
                _case = _subconfig = _config = -1;
                assert(min[0]<max[0] && min[1]<max[1] && min[2]<max[2]);
                _corners[0].X()=min.X();		_corners[0].Y()=min.Y();		_corners[0].Z()=min.Z();
                _corners[1].X()=max.X();		_corners[1].Y()=min.Y();		_corners[1].Z()=min.Z();
                _corners[2].X()=max.X();		_corners[2].Y()=max.Y();		_corners[2].Z()=min.Z();
                _corners[3].X()=min.X();		_corners[3].Y()=max.Y();		_corners[3].Z()=min.Z();
                _corners[4].X()=min.X();		_corners[4].Y()=min.Y();		_corners[4].Z()=max.Z();
                _corners[5].X()=max.X();		_corners[5].Y()=min.Y();		_corners[5].Z()=max.Z();
                _corners[6].X()=max.X();		_corners[6].Y()=max.Y();		_corners[6].Z()=max.Z();
                _corners[7].X()=min.X();		_corners[7].Y()=max.Y();		_corners[7].Z()=max.Z();

                for (int i=0; i<8; i++)
                    _field[i] = _walker->V( _corners[i].X(), _corners[i].Y(), _corners[i].Z() );

                unsigned char cubetype = 0;
                for (int i=0; i<8; i++)
                    if (_field[i]>0) cubetype += 1<<i;

                _case				= MCLookUpTable::Cases(cubetype, 0); //_case				= cases[cubetype][0];
                _config			= MCLookUpTable::Cases(cubetype, 1); //_config			= cases[cubetype][1];
                _subconfig	= 0;

                VertexPointer v12 = NULL;

                switch( _case )
                {
                case  0 : { break ; }
                case  1 : { AddTriangles( MCLookUpTable::Tiling1(_config), 1 ); break; } //case  1 : { AddTriangles( tiling1[_config], 1 ); break; }
                case  2 : { AddTriangles( MCLookUpTable::Tiling2(_config), 2 ); break; } //case  2 : { AddTriangles( tiling2[_config], 2 ); break; }
                case  3 :
                    {
                        //if( TestFace( test3[_config]) ) AddTriangles( tiling3_2[_config], 4 ) ; // 3.2
                        if( TestFace( MCLookUpTable::Test3(_config)) )
                            AddTriangles( MCLookUpTable::Tiling3_2(_config), 4 ) ; // 3.2
                        else
                            AddTriangles( MCLookUpTable::Tiling3_1(_config), 2 ) ; // 3.1
                        break ;
                    }
                case  4 :
                    {
                        //if( TestInterior( test4[_config]) ) AddTriangles( tiling4_1[_config], 2 ) ; // 4.1.1
                        if( TestInterior( MCLookUpTable::Test4(_config) ) )
                            AddTriangles( MCLookUpTable::Tiling4_1(_config), 2 ) ; // 4.1.1
                        else
                            AddTriangles( MCLookUpTable::Tiling4_2(_config), 6 ) ; // 4.1.2
                        break ;
                    }
                case  5 : { AddTriangles( MCLookUpTable::Tiling5(_config), 3 ); break; }
                case  6 :
                    {
                        //if( TestFace( test6[_config][0]) )
                        if( TestFace( MCLookUpTable::Test6(_config, 0)) )
                            AddTriangles( MCLookUpTable::Tiling6_2(_config), 5 ) ; // 6.2
                        else
                        {
                            if( TestInterior( MCLookUpTable::Test6(_config, 1)) )
                                AddTriangles( MCLookUpTable::Tiling6_1_1(_config), 3 ) ; // 6.1.1
                            else
                                AddTriangles( MCLookUpTable::Tiling6_1_2(_config), 7 ) ; // 6.1.2
                        }
                        break ;
                    }
                case  7 :
                    {
                        //if( TestFace( test7[_config][0] ) ) _subconfig +=  1 ;
                        //if( TestFace( test7[_config][1] ) ) _subconfig +=  2 ;
                        //if( TestFace( test7[_config][2] ) ) _subconfig +=  4 ;
                        if( TestFace( MCLookUpTable::Test7(_config, 0) ) ) _subconfig +=  1 ;
                        if( TestFace( MCLookUpTable::Test7(_config, 1) ) ) _subconfig +=  2 ;
                        if( TestFace( MCLookUpTable::Test7(_config, 2) ) ) _subconfig +=  4 ;
                        switch( _subconfig )
                        {
                        case 0 : { AddTriangles( MCLookUpTable::Tiling7_1(_config),		3 ) ; break; }
                        case 1 : { AddTriangles( MCLookUpTable::Tiling7_2(_config,0), 5 ) ; break; }
                        case 2 : { AddTriangles( MCLookUpTable::Tiling7_2(_config,1), 5 ) ; break; }
                        case 3 : { ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling7_3(_config,0), 9, v12 ) ; break ; }
                        case 4 : { AddTriangles( MCLookUpTable::Tiling7_2(_config, 2), 5 ) ; break ;}
                        case 5 : { ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling7_3(_config,1), 9, v12 ) ; break ; }
                        case 6 : { ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling7_3(_config,2), 9, v12 ) ; break ; }
                        case 7 :
                            {
                                if( TestInterior( MCLookUpTable::Test7(_config, 3) ) )
                                    AddTriangles( MCLookUpTable::Tiling7_4_2(_config), 9 ) ;
                                else
                                    AddTriangles( MCLookUpTable::Tiling7_4_1(_config), 5 ) ;
                                break ;
                            }
                        };
                        break ;
                    } // end of case 7
                case  8 : { AddTriangles( MCLookUpTable::Tiling8(_config), 2 ) ; break ;}
                case  9 : { AddTriangles( MCLookUpTable::Tiling9(_config), 4 ) ; break ;}
                case 10 :
                    {
                        if( TestFace( MCLookUpTable::Test10(_config, 0)) ) //if( TestFace( test10[_config][0]) )
                        {
                            if( TestFace( MCLookUpTable::Test10(_config,1) ) )
                                AddTriangles( MCLookUpTable::Tiling10_1_1_(_config), 4 ) ; // 10.1.1
                            else
                            {
                                ComputeCVertex(v12);
                                AddTriangles( MCLookUpTable::Tiling10_2(_config), 8, v12 ) ; // 10.2
                            }
                        }
                        else
                        {
                            if( TestFace( MCLookUpTable::Test10(_config, 1) ) )
                            {
                                ComputeCVertex(v12) ;
                                AddTriangles( MCLookUpTable::Tiling10_2_(_config), 8, v12 ) ; // 10.2
                            }
                            else
                            {
                                if( TestInterior( MCLookUpTable::Test10(_config, 2) ) )
                                    AddTriangles( MCLookUpTable::Tiling10_1_1(_config), 4 ) ; // 10.1.1
                                else
                                    AddTriangles( MCLookUpTable::Tiling10_1_2(_config), 8 ) ; // 10.1.2
                            }
                        }
                        break ;
                    } // end of case 10
                case 11 : { AddTriangles( MCLookUpTable::Tiling11(_config), 4 ) ; break ; }
                case 12 :
                    {
                        if( TestFace( MCLookUpTable::Test12(_config, 0) ) ) //if( TestFace( test12[_config][0]) )
                        {
                            if( TestFace( MCLookUpTable::Test12(_config, 1) ) )
                                AddTriangles( MCLookUpTable::Tiling12_1_1_(_config), 4 ) ; // 12.1.1
                            else
                            {
                                ComputeCVertex(v12) ;
                                AddTriangles( MCLookUpTable::Tiling12_2(_config), 8, v12 ) ; // 12.2
                            }
                        }
                        else
                        {
                            if( TestFace( MCLookUpTable::Test12(_config, 1) ) )
                            {
                                ComputeCVertex(v12) ;
                                AddTriangles( MCLookUpTable::Tiling12_2_(_config), 8, v12 ) ; // 12.2
                            }
                            else
                            {
                                if( TestInterior( MCLookUpTable::Test12(_config, 2) ) )
                                    AddTriangles( MCLookUpTable::Tiling12_1_1(_config), 4 ) ; // 12.1.1
                                else
                                    AddTriangles( MCLookUpTable::Tiling12_1_2(_config), 8 ) ; // 12.1.2
                            }
                        }
                        break ;
                    } // end of case 12
                case 13 :
                    {
                        //if( TestFace( test13[_config][0] ) ) _subconfig +=  1 ;
                        //if( TestFace( test13[_config][1] ) ) _subconfig +=  2 ;
                        //if( TestFace( test13[_config][2] ) ) _subconfig +=  4 ;
                        //if( TestFace( test13[_config][3] ) ) _subconfig +=  8 ;
                        //if( TestFace( test13[_config][4] ) ) _subconfig += 16 ;
                        //if( TestFace( test13[_config][5] ) ) _subconfig += 32 ;
                        if( TestFace( MCLookUpTable::Test13(_config, 0) ) ) _subconfig +=  1 ;
                        if( TestFace( MCLookUpTable::Test13(_config, 1) ) ) _subconfig +=  2 ;
                        if( TestFace( MCLookUpTable::Test13(_config, 2) ) ) _subconfig +=  4 ;
                        if( TestFace( MCLookUpTable::Test13(_config, 3) ) ) _subconfig +=  8 ;
                        if( TestFace( MCLookUpTable::Test13(_config, 4) ) ) _subconfig += 16 ;
                        if( TestFace( MCLookUpTable::Test13(_config, 5) ) ) _subconfig += 32 ;
                        switch( MCLookUpTable::Subconfig13(_subconfig) ) //switch( subconfig13[_subconfig] )
                        {
                        case 0  : { /* 13.1 */ AddTriangles( MCLookUpTable::Tiling13_1(_config)	 , 4 ) ; break ; }
                        case 1  : { /* 13.2 */ AddTriangles( MCLookUpTable::Tiling13_2(_config, 0), 6 ) ; break ; }
                        case 2  : { /* 13.2 */ AddTriangles( MCLookUpTable::Tiling13_2(_config, 1), 6 ) ; break ; }
                        case 3  : { /* 13.2 */ AddTriangles( MCLookUpTable::Tiling13_2(_config, 2), 6 ) ; break ; }
                        case 4  : { /* 13.2 */ AddTriangles( MCLookUpTable::Tiling13_2(_config, 3), 6 ) ; break ; }
                        case 5  : { /* 13.2 */ AddTriangles( MCLookUpTable::Tiling13_2(_config, 4), 6 ) ; break ; }
                        case 6  : { /* 13.2 */ AddTriangles( MCLookUpTable::Tiling13_2(_config, 5), 6 ) ; break ; }
                        case 7  : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3(_config, 0), 10, v12 ) ; break ; }
                        case 8  : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3(_config, 1), 10, v12 ) ; break ; }
                        case 9  : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3(_config, 2), 10, v12 ) ; break ; }
                        case 10 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3(_config, 3), 10, v12 ) ; break ; }
                        case 11 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3(_config, 4), 10, v12 ) ; break ; }
                        case 12 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3(_config, 5), 10, v12 ) ; break ; }
                        case 13 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3(_config, 6), 10, v12 ) ; break ; }
                        case 14 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3(_config, 7), 10, v12 ) ; break ; }
                        case 15 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3(_config, 8), 10, v12 ) ; break ; }
                        case 16 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3(_config, 9), 10, v12 ) ; break ; }
                        case 17 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3(_config,10), 10, v12 ) ; break ; }
                        case 18 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3(_config,11), 10, v12 ) ; break ; }
                        case 19 : { /* 13.4 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_4(_config, 0), 12, v12 ) ; break ; }
                        case 20 : { /* 13.4 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_4(_config, 1), 12, v12 ) ; break ; }
                        case 21 : { /* 13.4 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_4(_config, 2), 12, v12 ) ; break ; }
                        case 22 : { /* 13.4 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_4(_config, 3), 12, v12 ) ; break ; }
                        case 23 :
                            { /* 13.5 */
                                _subconfig = 0 ;
                                if( TestInterior( MCLookUpTable::Test13(_config, 6) ) )
                                    AddTriangles( MCLookUpTable::Tiling13_5_1(_config, 0), 6 ) ;
                                else
                                    AddTriangles( MCLookUpTable::Tiling13_5_2(_config, 0), 10 ) ;
                                break ;
                            }
                        case 24 :
                            { /* 13.5 */
                                _subconfig = 1 ;
                                if( TestInterior( MCLookUpTable::Test13(_config, 6) ) )
                                    AddTriangles( MCLookUpTable::Tiling13_5_1(_config, 1), 6 ) ;
                                else
                                    AddTriangles( MCLookUpTable::Tiling13_5_2(_config, 1), 10 ) ;
                                break ;
                            }
                        case 25 :
                            {/* 13.5 */
                                _subconfig = 2 ;
                                if( TestInterior( MCLookUpTable::Test13(_config, 6) ) )
                                    AddTriangles( MCLookUpTable::Tiling13_5_1(_config, 2), 6 ) ;
                                else
                                    AddTriangles( MCLookUpTable::Tiling13_5_2(_config, 2), 10 ) ;
                                break ;
                            }
                        case 26 :
                            {/* 13.5 */
                                _subconfig = 3 ;
                                if( TestInterior( MCLookUpTable::Test13(_config, 6) ) )
                                    AddTriangles( MCLookUpTable::Tiling13_5_1(_config, 3), 6 ) ;
                                else
                                    AddTriangles( MCLookUpTable::Tiling13_5_2(_config, 3), 10 ) ;
                                break ;
                            }
                        case 27 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3_(_config, 0), 10, v12 ) ; break ; }
                        case 28 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3_(_config, 1), 10, v12 ) ; break ; }
                        case 29 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3_(_config, 2), 10, v12 ) ; break ; }
                        case 30 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3_(_config, 3), 10, v12 ) ; break ; }
                        case 31 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3_(_config, 4), 10, v12 ) ; break ; }
                        case 32 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3_(_config, 5), 10, v12 ) ; break ; }
                        case 33 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3_(_config, 6), 10, v12 ) ; break ; }
                        case 34 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3_(_config, 7), 10, v12 ) ; break ; }
                        case 35 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3_(_config, 8), 10, v12 ) ; break ; }
                        case 36 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3_(_config, 9), 10, v12 ) ; break ; }
                        case 37 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3_(_config,10), 10, v12 ) ; break ; }
                        case 38 : { /* 13.3 */ ComputeCVertex(v12); AddTriangles( MCLookUpTable::Tiling13_3_(_config,11), 10, v12 ) ; break ; }
                        case 39 : { /* 13.2 */ AddTriangles( MCLookUpTable::Tiling13_2_(_config,0), 6 ) ; break ; }
                        case 40 : { /* 13.2 */ AddTriangles( MCLookUpTable::Tiling13_2_(_config,1), 6 ) ; break ; }
                        case 41 : { /* 13.2 */ AddTriangles( MCLookUpTable::Tiling13_2_(_config,2), 6 ) ; break ; }
                        case 42 : { /* 13.2 */ AddTriangles( MCLookUpTable::Tiling13_2_(_config,3), 6 ) ; break ; }
                        case 43 : { /* 13.2 */ AddTriangles( MCLookUpTable::Tiling13_2_(_config,4), 6 ) ; break ; }
                        case 44 : { /* 13.2 */ AddTriangles( MCLookUpTable::Tiling13_2_(_config,5), 6 ) ; break ; }
                        case 45 : { /* 13.1 */ AddTriangles( MCLookUpTable::Tiling13_1_(_config)	, 4 ) ; break ; }
                        default : { /*Impossible case 13*/  assert(false); }
                        }
                        break ;
                    } // end of case 13

                case 14 : { AddTriangles( MCLookUpTable::Tiling14(_config), 4 ) ; }
                                    break ;
                } //end of switch (_case)

            }; // end of ApplyMC

        private:
            /*!
            */
            WALKER_TYPE		*_walker;
            /*!
            */
            TRIMESH_TYPE	*_mesh;

            /*!
            *	The field value at the cell corners
            */
            ScalarType _field[8];

            /*!
            *	Array of the 8 corners of the volume cell being processed
            */
            vcg::Point3i _corners[8];

            /*!
            *	Case of the volume cell being processed
            */
            unsigned char _case;

            /*!
            *	 Configuration of the volume cell being processed
            */
            unsigned char _config;

            /*!
            *	Subconfiguration of the volume cell being processed
            */
            unsigned char _subconfig;

            /*!
            *	Tests if the components of the tesselation of the cube should be connected
            *	by the interior of an ambiguous face
            */
            inline bool TestFace(signed char face)
            {
                ScalarType A,B,C,D ;

                switch( face )
                {
                case -1 : case 1 :  A = _field[0] ;  B = _field[4] ;  C = _field[5] ;  D = _field[1] ;  break ;
                case -2 : case 2 :  A = _field[1] ;  B = _field[5] ;  C = _field[6] ;  D = _field[2] ;  break ;
                case -3 : case 3 :  A = _field[2] ;  B = _field[6] ;  C = _field[7] ;  D = _field[3] ;  break ;
                case -4 : case 4 :  A = _field[3] ;  B = _field[7] ;  C = _field[4] ;  D = _field[0] ;  break ;
                case -5 : case 5 :  A = _field[0] ;  B = _field[3] ;  C = _field[2] ;  D = _field[1] ;  break ;
                case -6 : case 6 :  A = _field[4] ;  B = _field[7] ;  C = _field[6] ;  D = _field[5] ;  break ;
                default : assert(false); // Invalid face code
                };

                return face * A * ( A*C - B*D ) >= 0  ;  // face and A invert signs
            } // end of TestFace


            /*!
            *	Tests if the components of the tesselation of the cube should be connected
            *	through the interior of the cube
            */
            inline bool TestInterior(signed char s)
            {
                ScalarType t, At=0, Bt=0, Ct=0, Dt=0, a, b ;
                char  test =  0 ;
                char  edge = -1 ; // reference edge of the triangulation

                switch( _case )
                {
                case  4 :
                case 10 :
                    {
                        a = (_field[4]-_field[0])*(_field[6]-_field[2]) - (_field[7]-_field[3])*(_field[5]-_field[1]);
                        b =  _field[2]*(_field[4]-_field[0])+_field[0]*(_field[6]-_field[2])-_field[1]*(_field[7]-_field[3])-_field[3]*(_field[5]-_field[1]);
                        t = - b / (2*a) ;
                        if( t<0 || t>1 )
                            return s>0 ;

                        At = _field[0] + ( _field[4] - _field[0] ) * t ;
                        Bt = _field[3] + ( _field[7] - _field[3] ) * t ;
                        Ct = _field[2] + ( _field[6] - _field[2] ) * t ;
                        Dt = _field[1] + ( _field[5] - _field[1] ) * t ;
                        break ;
                    }
                case  6 :
                case  7 :
                case 12 :
                case 13 :
                    switch( _case )
                    {
                    case  6 : edge = MCLookUpTable::Test6 (_config, 2) ; break ;
                    case  7 : edge = MCLookUpTable::Test7 (_config, 4) ; break ;
                    case 12 : edge = MCLookUpTable::Test12(_config, 3) ; break ;
                    case 13 : edge = MCLookUpTable::Tiling13_5_1(_config, _subconfig)[0] ; break ;
                    }
                    switch( edge )
                    {
                    case  0 :
                        t  = _field[0] / ( _field[0] - _field[1] ) ;
                        At = 0 ;
                        Bt = _field[3] + ( _field[2] - _field[3] ) * t ;
                        Ct = _field[7] + ( _field[6] - _field[7] ) * t ;
                        Dt = _field[4] + ( _field[5] - _field[4] ) * t ;
                        break ;
                    case  1 :
                        t  = _field[1] / ( _field[1] - _field[2] ) ;
                        At = 0 ;
                        Bt = _field[0] + ( _field[3] - _field[0] ) * t ;
                        Ct = _field[4] + ( _field[7] - _field[4] ) * t ;
                        Dt = _field[5] + ( _field[6] - _field[5] ) * t ;
                        break ;
                    case  2 :
                        t  = _field[2] / ( _field[2] - _field[3] ) ;
                        At = 0 ;
                        Bt = _field[1] + ( _field[0] - _field[1] ) * t ;
                        Ct = _field[5] + ( _field[4] - _field[5] ) * t ;
                        Dt = _field[6] + ( _field[7] - _field[6] ) * t ;
                        break ;
                    case  3 :
                        t  = _field[3] / ( _field[3] - _field[0] ) ;
                        At = 0 ;
                        Bt = _field[2] + ( _field[1] - _field[2] ) * t ;
                        Ct = _field[6] + ( _field[5] - _field[6] ) * t ;
                        Dt = _field[7] + ( _field[4] - _field[7] ) * t ;
                        break ;
                    case  4 :
                        t  = _field[4] / ( _field[4] - _field[5] ) ;
                        At = 0 ;
                        Bt = _field[7] + ( _field[6] - _field[7] ) * t ;
                        Ct = _field[3] + ( _field[2] - _field[3] ) * t ;
                        Dt = _field[0] + ( _field[1] - _field[0] ) * t ;
                        break ;
                    case  5 :
                        t  = _field[5] / ( _field[5] - _field[6] ) ;
                        At = 0 ;
                        Bt = _field[4] + ( _field[7] - _field[4] ) * t ;
                        Ct = _field[0] + ( _field[3] - _field[0] ) * t ;
                        Dt = _field[1] + ( _field[2] - _field[1] ) * t ;
                        break ;
                    case  6 :
                        t  = _field[6] / ( _field[6] - _field[7] ) ;
                        At = 0 ;
                        Bt = _field[5] + ( _field[4] - _field[5] ) * t ;
                        Ct = _field[1] + ( _field[0] - _field[1] ) * t ;
                        Dt = _field[2] + ( _field[3] - _field[2] ) * t ;
                        break ;
                    case  7 :
                        t  = _field[7] / ( _field[7] - _field[4] ) ;
                        At = 0 ;
                        Bt = _field[6] + ( _field[5] - _field[6] ) * t ;
                        Ct = _field[2] + ( _field[1] - _field[2] ) * t ;
                        Dt = _field[3] + ( _field[0] - _field[3] ) * t ;
                        break ;
                    case  8 :
                        t  = _field[0] / ( _field[0] - _field[4] ) ;
                        At = 0 ;
                        Bt = _field[3] + ( _field[7] - _field[3] ) * t ;
                        Ct = _field[2] + ( _field[6] - _field[2] ) * t ;
                        Dt = _field[1] + ( _field[5] - _field[1] ) * t ;
                        break ;
                    case  9 :
                        t  = _field[1] / ( _field[1] - _field[5] ) ;
                        At = 0 ;
                        Bt = _field[0] + ( _field[4] - _field[0] ) * t ;
                        Ct = _field[3] + ( _field[7] - _field[3] ) * t ;
                        Dt = _field[2] + ( _field[6] - _field[2] ) * t ;
                        break ;
                    case 10 :
                        t  = _field[2] / ( _field[2] - _field[6] ) ;
                        At = 0 ;
                        Bt = _field[1] + ( _field[5] - _field[1] ) * t ;
                        Ct = _field[0] + ( _field[4] - _field[0] ) * t ;
                        Dt = _field[3] + ( _field[7] - _field[3] ) * t ;
                        break ;
                    case 11 :
                        t  = _field[3] / ( _field[3] - _field[7] ) ;
                        At = 0 ;
                        Bt = _field[2] + ( _field[6] - _field[2] ) * t ;
                        Ct = _field[1] + ( _field[5] - _field[1] ) * t ;
                        Dt = _field[0] + ( _field[4] - _field[0] ) * t ;
                        break ;
                    default: { assert(false); /* Invalid edge */ break ; }
                    }
                    break ;

                default : assert(false); /* Invalid ambiguous case */  break;
                }

                if( At >= 0 ) test ++ ;
                if( Bt >= 0 ) test += 2 ;
                if( Ct >= 0 ) test += 4 ;
                if( Dt >= 0 ) test += 8 ;
                switch( test )
                {
                case  0 : return s>0 ;
                case  1 : return s>0 ;
                case  2 : return s>0 ;
                case  3 : return s>0 ;
                case  4 : return s>0 ;
                case  5 : if( At * Ct <  Bt * Dt ) return s>0 ; break ;
                case  6 : return s>0 ;
                case  7 : return s<0 ;
                case  8 : return s>0 ;
                case  9 : return s>0 ;
                case 10 : if( At * Ct >= Bt * Dt ) return s>0 ; break ;
                case 11 : return s<0 ;
                case 12 : return s>0 ;
                case 13 : return s<0 ;
                case 14 : return s<0 ;
                case 15 : return s<0 ;
                }
                return s<0 ;
            } //end of TestInterior

            /*!
            *	Adds a vertex inside the current cube
            *	\param v	The pointer to the new vertex along the edge
            */
            inline void ComputeCVertex(VertexPointer &v12)
            {
                v12 = &*AllocatorType::AddVertices(*_mesh, 1);
                v12->P() = CoordType(0.0, 0.0, 0.0);

                unsigned int count = 0;
                VertexPointer v = NULL;
                if (_walker->Exist(_corners[0], _corners[1], v) )
                {
                    count++;
                    v12->P() += v->P();
                }
                if (_walker->Exist(_corners[1], _corners[2], v) )
                {
                    count++;
                    v12->P() += v->P();
                }
                if (_walker->Exist(_corners[3], _corners[2], v) )
                {
                    count++;
                    v12->P() += v->P();
                }
                if (_walker->Exist(_corners[0], _corners[3], v) )
                {
                    count++;
                    v12->P() += v->P();
                }
                if (_walker->Exist(_corners[4], _corners[5], v) )
                {
                    count++;
                    v12->P() += v->P();
                }
                if (_walker->Exist(_corners[5], _corners[6], v) )
                {
                    count++;
                    v12->P() += v->P();
                }
                if (_walker->Exist(_corners[7], _corners[6], v) )
                {
                    count++;
                    v12->P() += v->P();
                }
                if (_walker->Exist(_corners[4], _corners[7], v) )
                {
                    count++;
                    v12->P() += v->P();
                }
                if (_walker->Exist(_corners[0], _corners[4], v) )
                {
                    count++;
                    v12->P() += v->P();
                }
                if (_walker->Exist(_corners[1], _corners[5], v) )
                {
                    count++;
                    v12->P() += v->P();
                }
                if (_walker->Exist(_corners[2], _corners[6], v) )
                {
                    count++;
                    v12->P() += v->P();
                }
                if (_walker->Exist(_corners[3], _corners[7], v) )
                {
                    count++;
                    v12->P() += v->P();
                }
                v12->P() /= (float) count;
            } // end of AddCVertex
            /*!
            *	Adds new triangles to the mesh
            *	\param vertices_list	The list of vertex indices
            *	\param n							The number of triangles that will be added to the mesh
            *	\param v12						The pointer to the vertex inside the current cell
            */
            inline void AddTriangles(const char *vertices_list, char n, VertexPointer v12=NULL)
            {
                VertexPointer vp		= NULL;
                size_t face_idx			= _mesh->face.size();
                size_t v12_idx			= -1;
                size_t vertices_idx[3];
                if (v12 != NULL) v12_idx = v12 - &_mesh->vert[0];
                AllocatorType::AddFaces(*_mesh, (int) n);

                for (int trig=0; trig<3*n; face_idx++ )
                {
                    vp = NULL;
                    memset(vertices_idx, -1, 3*sizeof(size_t));
                    for (int vert=0; vert<3; vert++, trig++) //ok
                    {

                        switch ( vertices_list[trig] )
                        {
                        case  0: { _walker->GetXIntercept(_corners[0], _corners[1], vp); vertices_idx[vert] = tri::Index(*_mesh,vp); break; }
                        case  1: { _walker->GetYIntercept(_corners[1], _corners[2], vp); vertices_idx[vert] = tri::Index(*_mesh,vp); break; }
                        case  2: { _walker->GetXIntercept(_corners[3], _corners[2], vp); vertices_idx[vert] = tri::Index(*_mesh,vp); break; }
                        case  3: { _walker->GetYIntercept(_corners[0], _corners[3], vp); vertices_idx[vert] = tri::Index(*_mesh,vp); break; }
                        case  4: { _walker->GetXIntercept(_corners[4], _corners[5], vp); vertices_idx[vert] = tri::Index(*_mesh,vp); break; }
                        case  5: { _walker->GetYIntercept(_corners[5], _corners[6], vp); vertices_idx[vert] = tri::Index(*_mesh,vp); break; }
                        case  6: { _walker->GetXIntercept(_corners[7], _corners[6], vp); vertices_idx[vert] = tri::Index(*_mesh,vp); break; }
                        case  7: { _walker->GetYIntercept(_corners[4], _corners[7], vp); vertices_idx[vert] = tri::Index(*_mesh,vp); break; }
                        case  8: { _walker->GetZIntercept(_corners[0], _corners[4], vp); vertices_idx[vert] = tri::Index(*_mesh,vp); break; }
                        case  9: { _walker->GetZIntercept(_corners[1], _corners[5], vp); vertices_idx[vert] = tri::Index(*_mesh,vp); break; }
                        case 10: { _walker->GetZIntercept(_corners[2], _corners[6], vp); vertices_idx[vert] = tri::Index(*_mesh,vp); break; }
                        case 11: { _walker->GetZIntercept(_corners[3], _corners[7], vp); vertices_idx[vert] = tri::Index(*_mesh,vp); break; }
                        case 12: { assert(v12 != NULL); vertices_idx[vert] = v12_idx; break; }
                        default: { assert(false); /* Invalid edge identifier */ }
                        } // end of switch

                        // Note that vp can be zero if we are in case 12 and that vertices_idx is surely >0 so the following assert has to be corrected as below.
                        // assert((vp - &_mesh->vert[0])>=0 && vertices_idx[vert]<_mesh->vert.size());
                        assert(vertices_idx[vert]<_mesh->vert.size());
                    } // end for (int vert=0 ...)

                    _mesh->face[face_idx].V(0) = &_mesh->vert[vertices_idx[0]];
                    _mesh->face[face_idx].V(1) = &_mesh->vert[vertices_idx[1]];
                    _mesh->face[face_idx].V(2) = &_mesh->vert[vertices_idx[2]];
                } // end for (int trig=0...)
            } // end of AddTriangles


        }; // end of class MarchingCubes

        /*! @} */
        //end of Doxygen documentation

    } // end of namespace tri
} // end of namespace vcg

#endif //__VCG_MARCHING_CUBES
