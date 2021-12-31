/* 
This code is the implementation of our paper "R3LIVE: A Robust, Real-time, RGB-colored, 
LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package".

Author: Jiarong Lin   < ziv.lin.ljr@gmail.com >

If you use any code of this repo in your academic research, please cite at least
one of our papers:
[1] Lin, Jiarong, and Fu Zhang. "R3LIVE: A Robust, Real-time, RGB-colored, 
    LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package." 
[2] Xu, Wei, et al. "Fast-lio2: Fast direct lidar-inertial odometry."
[3] Lin, Jiarong, et al. "R2LIVE: A Robust, Real-time, LiDAR-Inertial-Visual
     tightly-coupled state Estimator and mapping." 
[4] Xu, Wei, and Fu Zhang. "Fast-lio: A fast, robust lidar-inertial odometry 
    package by tightly-coupled iterated kalman filter."
[5] Cai, Yixi, Wei Xu, and Fu Zhang. "ikd-Tree: An Incremental KD Tree for 
    Robotic Applications."
[6] Lin, Jiarong, and Fu Zhang. "Loam-livox: A fast, robust, high-precision 
    LiDAR odometry and mapping package for LiDARs of small FoV."

For commercial use, please contact me < ziv.lin.ljr@gmail.com > and
Dr. Fu Zhang < fuzhang@hku.hk >.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from this
    software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once
#include <unordered_map>
#include "pointcloud_rgbd.hpp"
#include "image_frame.hpp"

class Offline_map_recorder
{
  public:
    std::string m_working_dir;

    std::unordered_map< std::shared_ptr< RGB_pts >, std::vector< int > > m_pts_with_view;
    std::vector< std::shared_ptr< Image_frame > >                    m_image_pose_vec;
    std::vector< std::vector< std::shared_ptr< RGB_pts > > >             m_pts_in_views_vec;
    std::deque< std::vector< RGB_pt_ptr > >                              m_visited_pts_buffer;
    std::vector< std::unordered_set< RGB_voxel_ptr > >                   m_visited_voxel_vec;
    Global_map *                                                         m_global_map;
    int                                                                  m_image_id = 0;

    Offline_map_recorder() = default;
    ~Offline_map_recorder() = default;

    void init( Eigen::Matrix3d &camera_intrinsic, double image_width, Global_map *global_map_ptr );
    void set_working_dir( std::string working_dir );
    void insert_image_and_pts( std::shared_ptr< Image_frame > &img_ptr, std::unordered_set< RGB_voxel_ptr > & visited_voxel );
    void insert_image_and_pts( std::shared_ptr< Image_frame > &img_ptr, std::vector< RGB_pt_ptr > & visited_points );
    void export_to_mvs( Global_map &rgb_map );
    
  private:
    friend class boost::serialization::access;
    template < typename Archive >
    void serialize( Archive &ar, const unsigned int version )
    {
        boost::serialization::split_free( ar, *this, version );
    }
};

template < typename Archive >
inline void load( Archive &ar, Offline_map_recorder &map_recorder, const unsigned int /*version*/ )
{
    Common_tools::Timer tim;
    int                 frame_size;
    tim.tic();
    scope_color( ANSI_COLOR_BLUE_BOLD );
    cout << "Loading R3LIVE offline maps, please wait with patience~~~" << endl;
    ar >> *map_recorder.m_global_map;
    ar >> frame_size;
    cout << ANSI_COLOR_BLUE_BOLD;
    for ( int i = 0; i < frame_size; i++ )
    {
        std::vector< int >                        pts_index_in_view;
        std::vector< std::shared_ptr< RGB_pts > > pts_ptr_in_view;
        std::shared_ptr< Image_frame >        img_ptr = std::make_shared< Image_frame >();
        ar >> pts_index_in_view;
        ar >> *img_ptr;
        pts_ptr_in_view.resize( pts_index_in_view.size() );
        
        for ( int j = 0; j < pts_index_in_view.size(); j++ )
        {
            // pts_ptr_in_view[ j ] = map_recorder.m_global_map->m_rgb_pts_vec[ pts_index_in_view[ j ] ];
            pts_ptr_in_view[ j ] = map_recorder.m_global_map->m_rgb_pts_vec[ pts_index_in_view[ j ] ];
            CV_Assert( pts_ptr_in_view[ j ]->m_pt_index == pts_index_in_view[ j ] );
        }
        map_recorder.m_pts_in_views_vec.push_back( pts_ptr_in_view );
        map_recorder.m_image_pose_vec.push_back( img_ptr );
        cout << ANSI_DELETE_CURRENT_LINE << "Loading views and camera poses " << i*100.0 /  (frame_size-1) << " % ...";
        ANSI_SCREEN_FLUSH;
    }
    cout << endl;
    cout << "Load offine R3LIVE offline maps cost: " << tim.toc() << " ms" << ANSI_COLOR_RESET << endl;
}


inline int get_all_pts_in_boxes( std::unordered_set< std::shared_ptr< RGB_Voxel > >  box_hitted, std::vector< std::shared_ptr< RGB_pts > > &res_pts_vec )
{
    res_pts_vec.clear();
    for ( std::unordered_set< std::shared_ptr< RGB_Voxel > >::iterator it_box = box_hitted.begin(); it_box != box_hitted.end(); it_box++ )
    {
        auto it_end = ( *it_box )->m_pts_in_grid.end();
        res_pts_vec.insert( res_pts_vec.end(), ( *it_box )->m_pts_in_grid.begin(), it_end );
    }
    return res_pts_vec.size();
}

template < typename Archive >
inline void save( Archive &ar, const Offline_map_recorder &map_recorder, const unsigned int /*version*/ )
{
    int frame_size = map_recorder.m_pts_in_views_vec.size();
    CV_Assert( map_recorder.m_image_pose_vec.size() == map_recorder.m_pts_in_views_vec.size() );
    std::vector< std::vector< int > > pts_idx_in_view_vec_vec;
    Common_tools::Timer               tim;
    tim.tic();
    scope_color( ANSI_COLOR_MAGENTA_BOLD );
    cout << "Saving globale map..." << endl;
    ar << *map_recorder.m_global_map;
    cout << "Save globale map cost " << tim.toc( " ", 1 ) << " ms" << endl;
    ar << frame_size;
    for ( int i = 0; i < frame_size; i++ )
    {
        std::vector< int > pts_index_in_view;
        for ( int j = 0; j < map_recorder.m_pts_in_views_vec[ i ].size(); j++ )
        {
            pts_index_in_view.push_back( map_recorder.m_pts_in_views_vec[ i ][ j ]->m_pt_index );
        }
        ar << pts_index_in_view;
        ar << *map_recorder.m_image_pose_vec[ i ];
        cout << ANSI_DELETE_CURRENT_LINE << "Saving views and camera poses " << i*100.0 /  (frame_size-1) << " % ...";
        ANSI_SCREEN_FLUSH;
    }
    cout << endl;
    cout << "Save views and camera poses cost " << tim.toc( " ", 1 ) << " ms" << endl;
}