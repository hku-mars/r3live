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
#include <atomic>
#include <unordered_set>
#include "tools_logger.hpp"
#include "opencv2/opencv.hpp"
#include "image_frame.hpp"
#include "tools_kd_hash.hpp"
#include "tools_thread_pool.hpp"
#include "tools_serialization.hpp"
// #include "assert.h"
#define R3LIVE_MAP_MAJOR_VERSION 1
#define R3LIVE_MAP_MINOR_VERSION 0
extern cv::RNG g_rng;

// extern std::atomic< long > g_pts_index;
class RGB_pts
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#if 0
    std::atomic<double> m_pos[3];
    std::atomic<double> m_rgb[3];
    std::atomic<double> m_cov_rgb[3];
    std::atomic<double> m_gray;
    std::atomic<double> m_cov_gray;
    std::atomic<int> m_N_gray;
    std::atomic<int> m_N_rgb;
#else
    double m_pos[ 3 ] = { 0 };
    double m_rgb[ 3 ] = { 0 };
    double m_cov_rgb[ 3 ] = { 0 };
    double m_gray = 0;
    double m_cov_gray = 0;
    int    m_N_gray = 0;
    int    m_N_rgb = 0;
    int    m_pt_index = 0;
#endif
    vec_2      m_img_vel;
    vec_2      m_img_pt_in_last_frame;
    vec_2      m_img_pt_in_current_frame;
    int        m_is_out_lier_count = 0;
    cv::Scalar m_dbg_color;
    double     m_obs_dis = 0;
    double     m_last_obs_time = 0;
    void       clear()
    {
        m_rgb[ 0 ] = 0;
        m_rgb[ 1 ] = 0;
        m_rgb[ 2 ] = 0;
        m_gray = 0;
        m_cov_gray = 0;
        m_N_gray = 0;
        m_N_rgb = 0;
        m_obs_dis = 0;
        m_last_obs_time = 0;
        int r = g_rng.uniform( 0, 256 );
        int g = g_rng.uniform( 0, 256 );
        int b = g_rng.uniform( 0, 256 );
        m_dbg_color = cv::Scalar( r, g, b );
        // m_rgb = vec_3(255, 255, 255);
    };

    RGB_pts()
    {
        // m_pt_index = g_pts_index++;
        clear();
    };
    ~RGB_pts(){};

    void set_pos( const vec_3 &pos );
    vec_3          get_pos();
    vec_3          get_rgb();
    mat_3_3        get_rgb_cov();
    pcl::PointXYZI get_pt();
    void update_gray( const double gray, double obs_dis = 1.0 );
    int update_rgb( const vec_3 &rgb, const double obs_dis, const vec_3 obs_sigma, const double obs_time );

  private:
    friend class boost::serialization::access;
    template < typename Archive >
    void serialize( Archive &ar, const unsigned int version )
    {
        ar &m_pos;
        ar &m_rgb;
        ar &m_pt_index;
        ar &m_cov_rgb;
        ar &m_gray;
        ar &m_N_rgb;
        ar &m_N_gray;
    }
};
using RGB_pt_ptr = std::shared_ptr< RGB_pts >;

class RGB_Voxel
{
  public:
    std::vector< RGB_pt_ptr > m_pts_in_grid;
    double                    m_last_visited_time = 0;
    RGB_Voxel() = default;
    ~RGB_Voxel() = default;
    void add_pt( RGB_pt_ptr &rgb_pts ) { m_pts_in_grid.push_back( rgb_pts ); }
};

using RGB_voxel_ptr = std::shared_ptr< RGB_Voxel >;
using Voxel_set_iterator = std::unordered_set< std::shared_ptr< RGB_Voxel > >::iterator;

struct Global_map
{
    int                                                          m_map_major_version = R3LIVE_MAP_MAJOR_VERSION;
    int                                                          m_map_minor_version = R3LIVE_MAP_MINOR_VERSION;
    int                                                          m_if_get_all_pts_in_boxes_using_mp = 1;
    std::vector< RGB_pt_ptr >                    m_rgb_pts_vec;
    // std::vector< RGB_pt_ptr >                    m_rgb_pts_in_recent_visited_voxels;
    std::shared_ptr< std::vector< RGB_pt_ptr> >                  m_pts_rgb_vec_for_projection = nullptr;
    std::shared_ptr< std::mutex >                                m_mutex_pts_vec;
    std::shared_ptr< std::mutex >                                m_mutex_recent_added_list;
    std::shared_ptr< std::mutex >                                m_mutex_img_pose_for_projection;
    std::shared_ptr< std::mutex >                                m_mutex_rgb_pts_in_recent_hitted_boxes;
    std::shared_ptr< std::mutex >                                m_mutex_m_box_recent_hitted;
    std::shared_ptr< std::mutex >                                m_mutex_pts_last_visited;
    Image_frame                                              m_img_for_projection;
    double                                                       m_recent_visited_voxel_activated_time = 0.0;
    bool                                                         m_in_appending_pts = 0;
    int                                                          m_updated_frame_index = 0;
    std::shared_ptr< std::thread >                               m_thread_service;
    int                                                          m_if_reload_init_voxel_and_hashed_pts = true;

    Hash_map_3d< long, RGB_pt_ptr >   m_hashmap_3d_pts;
    Hash_map_3d< long, std::shared_ptr< RGB_Voxel > > m_hashmap_voxels;
    std::unordered_set< std::shared_ptr< RGB_Voxel > > m_voxels_recent_visited;
    std::vector< std::shared_ptr< RGB_pts > >          m_pts_last_hitted;
    double                                   m_minimum_pts_size = 0.05; // 5cm minimum distance.
    double                                   m_voxel_resolution = 0.1;
    double                                   m_maximum_depth_for_projection = 200;
    double                                   m_minimum_depth_for_projection = 3;
    int                                      m_last_updated_frame_idx = -1;
    void                                     clear();
    void set_minmum_dis( double minimum_dis );

    Global_map( int if_start_service = 1 );
    ~Global_map();

    void service_refresh_pts_for_projection();
    void render_points_for_projection( std::shared_ptr< Image_frame > &img_ptr );
    void update_pose_for_projection( std::shared_ptr< Image_frame > &img, double fov_margin = 0.0001 );
    bool is_busy();
    template < typename T >
    int append_points_to_global_map( pcl::PointCloud< T > &pc_in, double  added_time,  std::vector< RGB_pt_ptr > *pts_added_vec = nullptr, int step = 1 );
    void render_with_a_image( std::shared_ptr< Image_frame > &img_ptr, int if_select = 1 );
    void selection_points_for_projection( std::shared_ptr< Image_frame > &image_pose, std::vector< std::shared_ptr< RGB_pts > > *pc_out_vec = nullptr,
                                          std::vector< cv::Point2f > *pc_2d_out_vec = nullptr, double minimum_dis = 5, int skip_step = 1,int use_all_pts = 0 );
    void save_to_pcd( std::string dir_name, std::string file_name = std::string( "/rgb_pt" ) , int save_pts_with_views = 3);
    void save_and_display_pointcloud( std::string dir_name = std::string( "/home/ziv/temp/" ), std::string file_name = std::string( "/rgb_pt" ) ,  int save_pts_with_views = 3);
    void render_pts_in_voxels( std::shared_ptr< Image_frame > &img_ptr, std::vector< std::shared_ptr< RGB_pts > > &voxels_for_render, double obs_time = 0 );

  private:
    friend class boost::serialization::access;
    template < typename Archive >
    void serialize( Archive &ar, const unsigned int version )
    {
        boost::serialization::split_free( ar, *this, version );
    }
};




void render_pts_in_voxels_mp( std::shared_ptr< Image_frame > &img_ptr, std::unordered_set< RGB_voxel_ptr > *voxels_for_render, const double &obs_time = 0 );

template < typename Archive >
inline void save( Archive &ar, const Global_map &global_map, const unsigned int /*version*/ )
{
    int vector_size;
    vector_size = global_map.m_rgb_pts_vec.size();
    ar << global_map.m_map_major_version;
    ar << global_map.m_map_minor_version;
    ar << global_map.m_minimum_pts_size;
    ar << global_map.m_voxel_resolution;
    ar << vector_size;
    cout << ANSI_COLOR_YELLOW_BOLD;
    for ( int i = 0; i < vector_size; i++ )
    {
        ar << ( *global_map.m_rgb_pts_vec[ i ] );
        CV_Assert( global_map.m_rgb_pts_vec[ i ]->m_pt_index == i );
        if ( ( i % 10000 == 0 ) || ( i == vector_size - 1 ) )
        {
            cout << ANSI_DELETE_CURRENT_LINE << "Saving global map: " << ( i * 100.0 / (vector_size-1) ) << " %";
            ANSI_SCREEN_FLUSH;
        }
    }
    cout << endl;
}

template < typename Archive >
inline void load( Archive &ar, Global_map &global_map, const unsigned int /*version*/ )
{
    Common_tools::Timer tim;
    tim.tic();
    int vector_size;
    vector_size = global_map.m_rgb_pts_vec.size();
    ar >> global_map.m_map_major_version;
    ar >> global_map.m_map_minor_version;
    ar >> global_map.m_minimum_pts_size;
    ar >> global_map.m_voxel_resolution;
    ar >> vector_size;
    int grid_x, grid_y, grid_z, box_x, box_y, box_z;
    scope_color( ANSI_COLOR_YELLOW_BOLD );
    for ( int i = 0; i < vector_size; i++ )
    {
        // printf_line;
        std::shared_ptr< RGB_pts > rgb_pt = std::make_shared< RGB_pts >();
        ar >> *rgb_pt;
        CV_Assert( rgb_pt->m_pt_index == global_map.m_rgb_pts_vec.size() );
        global_map.m_rgb_pts_vec.push_back( rgb_pt );
        grid_x = std::round( rgb_pt->m_pos[ 0 ] / global_map.m_minimum_pts_size );
        grid_y = std::round( rgb_pt->m_pos[ 1 ] / global_map.m_minimum_pts_size );
        grid_z = std::round( rgb_pt->m_pos[ 2 ] / global_map.m_minimum_pts_size );
        box_x = std::round( rgb_pt->m_pos[ 0 ] / global_map.m_voxel_resolution );
        box_y = std::round( rgb_pt->m_pos[ 1 ] / global_map.m_voxel_resolution );
        box_z = std::round( rgb_pt->m_pos[ 2 ] / global_map.m_voxel_resolution );

        if ( global_map.m_if_reload_init_voxel_and_hashed_pts )
        {
            // if init voxel and hashmap_3d_pts, comment this to save loading time if necessary.
            global_map.m_hashmap_3d_pts.insert( grid_x, grid_y, grid_z, rgb_pt );
            if ( !global_map.m_hashmap_voxels.if_exist( box_x, box_y, box_z ) )
            {
                std::shared_ptr< RGB_Voxel > box_rgb = std::make_shared< RGB_Voxel >();
                global_map.m_hashmap_voxels.insert( box_x, box_y, box_z, box_rgb );
            }
            global_map.m_hashmap_voxels.m_map_3d_hash_map[ box_x ][ box_y ][ box_z ]->add_pt( rgb_pt );
        }
        if ( ( i % 10000 == 0 ) || ( i == vector_size - 1 ) )
        {
            cout << ANSI_DELETE_CURRENT_LINE << "Loading global map: " << ( i * 100.0 / (vector_size-1) ) << " %";
            ANSI_SCREEN_FLUSH;
        }
    }
    cout << endl;
    cout << "Load offine global map cost: " << tim.toc() << " ms" << ANSI_COLOR_RESET << endl;
}
