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
#include "offline_map_recorder.hpp"

void Offline_map_recorder::init( Eigen::Matrix3d &camera_intrinsic, double image_width, Global_map *global_map_ptr )
{
    m_global_map = global_map_ptr;
}

void Offline_map_recorder::set_working_dir( std::string working_dir )
{
    m_working_dir = working_dir;
}

vec_3                                             last_pose_t = vec_3( -100, 0, 0 );
eigen_q                                           last_pose_q = eigen_q( 1, 0, 0, 1 );
std::vector< std::shared_ptr< Image_frame > > img_ptr_vec;
#define BUILD_GLOBAL_MVS 0
#define IF_SAVE_IMAGE 0

void Offline_map_recorder::insert_image_and_pts( std::shared_ptr< Image_frame > &img_ptr,  std::vector< RGB_pt_ptr > & _visited_points )
{
    std::vector< RGB_pt_ptr > visited_points , points_viewed_in_this_frame;
    m_image_pose_vec.push_back( soft_copy_image_frame( img_ptr ) );
    std::vector< std::shared_ptr< RGB_pts > > rgb_pts_vec;
    m_global_map->m_mutex_pts_last_visited->lock();
    visited_points = _visited_points;
    m_global_map->m_mutex_pts_last_visited->unlock();
   
    if ( 0 )
    {
        m_visited_pts_buffer.push_back(visited_points);
        if ( m_visited_pts_buffer.size() > 10 )
        {
            m_visited_pts_buffer.pop_front();
        }
        for ( auto it : m_visited_pts_buffer )
        {
            points_viewed_in_this_frame.insert( points_viewed_in_this_frame.end(), it.begin(), it.end() );
        }
        m_pts_in_views_vec.push_back( points_viewed_in_this_frame );
        // cout << "Input size = " << visited_points.size() << ", finial size = " << points_viewed_in_this_frame.size() << endl;
    }
    else
    {
        m_pts_in_views_vec.push_back( visited_points );
    }
    
}

void Offline_map_recorder::insert_image_and_pts( std::shared_ptr< Image_frame > &img_ptr,  std::unordered_set< RGB_voxel_ptr > & _visited_voxel )
{
    m_image_pose_vec.push_back( soft_copy_image_frame( img_ptr ) );

    m_global_map->m_mutex_m_box_recent_hitted->lock();
    std::unordered_set< RGB_voxel_ptr > visited_voxel = _visited_voxel;
    m_global_map->m_mutex_m_box_recent_hitted->unlock();

    m_visited_voxel_vec.push_back( visited_voxel );
    std::vector< std::shared_ptr< RGB_pts > > rgb_pts_vec;
    get_all_pts_in_boxes( visited_voxel, rgb_pts_vec );
    m_pts_in_views_vec.push_back( rgb_pts_vec );
}

void Offline_map_recorder::export_to_mvs( Global_map &rgb_map )
{
    scope_color( ANSI_COLOR_WHITE_BOLD );
    char file_name[ 1024 ];
    sprintf( file_name, "%s/test.r3live", m_working_dir.c_str() );
    cout << "Export the whole offline map to file: " << file_name << endl;
    Common_tools::dump_obj_to_file( this,file_name );
}
