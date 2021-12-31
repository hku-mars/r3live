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

#include "rgb_map/image_frame.hpp"
#include "rgb_map/pointcloud_rgbd.hpp"
#include "rgb_map/offline_map_recorder.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <boost/program_options.hpp>
#include "meshing/MVS/Common.h"
#include "meshing/MVS/Image.h"
#include "meshing/MVS/PointCloud.h"
#include "meshing/MVS/Mesh.h"
#include "meshing/build_mesh.hpp"

#include "tools_mem_used.h"
#include "tools_logger.hpp"
#include "tools_ros.hpp"
// #include ""
// cv::RNG g_rng = cv::RNG(0);
Common_tools::Cost_time_logger              g_cost_time_logger;
std::shared_ptr< Common_tools::ThreadPool > m_thread_pool_ptr;
std::string                                 data_path = std::string( "/home/ziv/color_temp_r3live/" );

std::string g_working_dir;
std::string g_offline_map_name;
double      g_insert_pt_dis;
bool        bUseConstantWeight;
bool        g_if_use_free_space_support;
double      g_thickness_factor;
double      g_quality_factor;
double      g_decimate_mesh;
double      g_if_remove_spurious;
bool        g_if_remove_spikes;
int         g_close_holes_dist;
int         g_smooth_mesh_factor;
double      g_add_keyframe_t = 0.15;
double      g_add_keyframe_R = 10;
int         g_texturing_smooth_factor = 3;
std::string g_str_export_type;
std::string strConfigFileName;

using PointType = pcl::PointXYZRGBA;
pcl::PointCloud<PointType>::Ptr pcl_pc_rgb = nullptr;
pcl::KdTreeFLANN<PointType> kdtree;


// TODO
void r3live_map_to_mvs_scene( Offline_map_recorder &r3live_map_recorder, MVS::ImageArr &m_images, MVS::PointCloud &m_pointcloud )
{
    vec_3   last_pose_t = vec_3( -100, 0, 0 );
    eigen_q last_pose_q = eigen_q( 1, 0, 0, 1 );
    int     m_image_id = 0;
    int     number_of_image_frame = r3live_map_recorder.m_pts_in_views_vec.size();
    cout << "Number of image frames: " << number_of_image_frame << endl;

    Eigen::Matrix3d camera_intrinsic;
    int             image_width = r3live_map_recorder.m_image_pose_vec[ 0 ]->m_img_cols;
    int             image_heigh = r3live_map_recorder.m_image_pose_vec[ 0 ]->m_img_rows;
    double          fx = r3live_map_recorder.m_image_pose_vec[ 0 ]->fx;
    double          fy = r3live_map_recorder.m_image_pose_vec[ 0 ]->fy;
    double          cx = r3live_map_recorder.m_image_pose_vec[ 0 ]->cx;
    double          cy = r3live_map_recorder.m_image_pose_vec[ 0 ]->cy;
    camera_intrinsic << fx / image_width, 0, cx / image_width, 0, fy / image_width, cy / image_width, 0, 0, 1;
    // camera_intrinsic << fx , 0, cx , 0, fy , cy , 0, 0, 1;
    cout << "Iamge resolution  = " << r3live_map_recorder.m_image_pose_vec[ 0 ]->m_img_cols << " X " << r3live_map_recorder.m_image_pose_vec[ 0 ]->m_img_rows << endl;
    // cout << "Camera intrinsic: \r\n" << camera_intrinsic << endl;

    MVS::Platform m_platforms = MVS::Platform();
    m_platforms.name = std::string( "platfrom" );
    m_platforms.cameras.push_back( MVS::Platform::Camera() );
    m_platforms.cameras[ 0 ].K = camera_intrinsic;
    m_platforms.cameras[ 0 ].R = Eigen::Matrix3d::Identity();
    m_platforms.cameras[ 0 ].C = Eigen::Vector3d::Zero();

    std::unordered_map< std::shared_ptr< RGB_pts >, std::vector< int > > m_pts_with_view;
    for ( int frame_idx = 0; frame_idx < number_of_image_frame; frame_idx++ )
    {
        std::shared_ptr< Image_frame > img_ptr = r3live_map_recorder.m_image_pose_vec[ frame_idx ];
        vec_3                              pose_t = -img_ptr->m_pose_c2w_q.toRotationMatrix().transpose() * img_ptr->m_pose_c2w_t;
        if ( ( pose_t - last_pose_t ).norm() < g_add_keyframe_t && ( img_ptr->m_pose_c2w_q.angularDistance( last_pose_q ) * 57.3 < g_add_keyframe_R ) )
        {
            continue;
        }
        MVS::Platform::Pose pose;
        MVS::Image          image;

        pose.R = img_ptr->m_pose_c2w_q.toRotationMatrix();
        pose.C = pose_t;
        last_pose_t = pose_t;
        last_pose_q = img_ptr->m_pose_c2w_q;
        m_platforms.poses.push_back( pose );
        // cout << "[ " << frame_idx << " ]: q = " << img_ptr->m_pose_c2w_q.coeffs().transpose() << " | "<< pose_t.transpose() << endl;

        image.ID = m_image_id;
        m_image_id++;
        image.poseID = image.ID;
        image.platformID = 0;
        image.cameraID = 0;
        image.width = img_ptr->m_img_cols;
        image.height = img_ptr->m_img_rows;
        image.camera = Camera( m_platforms.GetCamera( image.cameraID, image.poseID ) );

        // compute the unnormalized camera
        image.camera.K = image.camera.GetK< REAL >( image_width, image_heigh );
        image.camera.ComposeP();
        m_images.push_back( image );

        for ( int pt_idx = 0; pt_idx < r3live_map_recorder.m_pts_in_views_vec[ frame_idx ].size(); pt_idx++ )
        {
            m_pts_with_view[ r3live_map_recorder.m_pts_in_views_vec[ frame_idx ][ pt_idx ] ].push_back( image.ID  );
        }
        cout << ANSI_DELETE_CURRENT_LINE;
        printf( "\33[2K\rAdd frames: %u%%, total_points = %u ...", frame_idx * 100 / ( number_of_image_frame - 1 ), m_pts_with_view.size() );
        ANSI_SCREEN_FLUSH;
    }
    cout << endl;
    cout << "Number of image frames: " << m_image_id << endl;
    cout << "Number of points " << m_pts_with_view.size() << endl;

    int acc_count = 0;
    m_pointcloud.points.resize( m_pts_with_view.size() );
    m_pointcloud.pointViews.resize( m_pts_with_view.size() );
    m_pointcloud.colors.resize( m_pts_with_view.size() );
    long point_index = 0;
    long temp_int = 0;
    if(pcl_pc_rgb == nullptr)
    {
        pcl_pc_rgb = boost::make_shared<pcl::PointCloud<PointType>>();
    }
    pcl_pc_rgb->clear();
    pcl_pc_rgb->reserve(1e8);
    for ( std::unordered_map< std::shared_ptr< RGB_pts >, std::vector< int > >::iterator it = m_pts_with_view.begin(); it != m_pts_with_view.end(); it++ )
    {
        if ( ( it->second.size() >= 0 ) && ( ( it->first )->m_N_rgb > 5 ) )
        {
            acc_count++;
            MVS::PointCloud::Point    pt3d;
            MVS::PointCloud::ViewArr  pt_view_arr;
            MVS::PointCloud::ColorArr color_arr;
            MVS::PointCloud::Color    color;
            vec_3                     pt_pos = ( ( it->first ) )->get_pos();
            pt3d.x = pt_pos( 0 );
            pt3d.y = pt_pos( 1 );
            pt3d.z = pt_pos( 2 );
            color.r = ( ( it->first ) )->m_rgb[ 2 ];
            color.g = ( ( it->first ) )->m_rgb[ 1 ];
            color.b = ( ( it->first ) )->m_rgb[ 0 ];
            PointType pcl_pt ;
            pcl_pt.x = pt3d.x;
            pcl_pt.y = pt3d.y;
            pcl_pt.z = pt3d.z;
            pcl_pt.r = color.r;
            pcl_pt.g = color.g;
            pcl_pt.b = color.b;
            pcl_pc_rgb->points.push_back(pcl_pt);
            for ( auto _idx : it->second )
            {
                pt_view_arr.push_back( _idx );
            }
            m_pointcloud.points[ point_index ] = pt3d;
            m_pointcloud.pointViews[ point_index ] = pt_view_arr;
            m_pointcloud.colors[ point_index ] = color;
            point_index++;
            if ( ( temp_int + 1 ) % ( m_pts_with_view.size() / 10 ) == 0 )
            {
                printf( "\33[2K\rRetring points: %u%% ...", temp_int * 10 / ( m_pts_with_view.size() / 10 ) );
                ANSI_SCREEN_FLUSH;
            }
        }
        temp_int++;
    }
    printf( "\33[2K\rRetriving points: %u%% ...", 100 );
    m_pointcloud.points.resize( point_index );
    m_pointcloud.pointViews.resize( point_index );
    m_pointcloud.colors.resize( point_index );
    cout << endl;
    cout << "Total available points number " << point_index << endl;
}


void build_pcl_kdtree( Offline_map_recorder &r3live_map_recorder )
{
    if ( pcl_pc_rgb == nullptr )
    {
        pcl_pc_rgb = boost::make_shared< pcl::PointCloud< PointType > >();
    }
    if ( 1 ) // if reload all pts.
    {
        pcl_pc_rgb->clear();
        pcl_pc_rgb->points.resize( r3live_map_recorder.m_global_map->m_rgb_pts_vec.size() );
        for ( int i = 0; i < r3live_map_recorder.m_global_map->m_rgb_pts_vec.size(); i++ )
        {
            PointType  pcl_pt;
            
            RGB_pt_ptr rgb_pt = r3live_map_recorder.m_global_map->m_rgb_pts_vec[ i ];
            if(rgb_pt->m_N_rgb < 5) 
            {
                continue;
            }
            pcl_pt.x = rgb_pt->m_pos[ 0 ];
            pcl_pt.y = rgb_pt->m_pos[ 1 ];
            pcl_pt.z = rgb_pt->m_pos[ 2 ];
            pcl_pt.r = rgb_pt->m_rgb[ 2 ];
            pcl_pt.g = rgb_pt->m_rgb[ 1 ];
            pcl_pt.b = rgb_pt->m_rgb[ 0 ];
            pcl_pc_rgb->points[ i ]= pcl_pt ;
        }
    }
    kdtree.setInputCloud( pcl_pc_rgb );
}

void reconstruct_mesh( Offline_map_recorder &r3live_map_recorder, std::string output_dir )
{
    cout << "==== Work directory: " << output_dir << endl;

    MVS::ImageArr   m_images;
    MVS::PointCloud m_pointcloud;
    MVS::Mesh       reconstructed_mesh;

    r3live_map_to_mvs_scene( r3live_map_recorder, m_images, m_pointcloud );
    // return;
    
    ReconstructMesh( g_insert_pt_dis, g_if_use_free_space_support, 4, g_thickness_factor, g_quality_factor, reconstructed_mesh, m_images, m_pointcloud );
    printf( "Mesh reconstruction completed: %u vertices, %u faces\n", reconstructed_mesh.vertices.GetSize(), reconstructed_mesh.faces.GetSize() );
    
    cout << "Clean mesh [1/3]: ";
    reconstructed_mesh.Clean( g_decimate_mesh, g_if_remove_spurious, g_if_remove_spikes, g_close_holes_dist, g_smooth_mesh_factor, false );
    
    cout << "Clean mesh [2/3]: ";
    reconstructed_mesh.Clean( 1.f, 0.f, g_if_remove_spikes, g_close_holes_dist, 0, false ); // extra cleaning trying to close more holes
    
    cout << "Clean mesh [3/3]: ";
    reconstructed_mesh.Clean( 1.f, 0.f, false, 0, 0, true ); // extra cleaning to
                                                             // remove non-manifold
                                                             // problems created by
                                                             // closing holes
    reconstructed_mesh.Save( MAKE_PATH_SAFE( Util::getFileFullName( output_dir ) ) + _T("/reconstructed_mesh") + ".ply" );
    reconstructed_mesh.Save( MAKE_PATH_SAFE( Util::getFileFullName( output_dir ) ) + _T("/reconstructed_mesh") + ".obj" );
}

void texture_mesh(  Offline_map_recorder &r3live_map_recorder, std::string input_mesh_name, std::string output_mesh_name, int smooth_factor  )
{
    cout << "Performaning the mesh texturing..." << endl;
    cout << "Build Kd tree, please wait..." << endl;
    build_pcl_kdtree(r3live_map_recorder);
    cout << "Build Kd tree finish !" << endl;
    pcl::PolygonMesh        mesh_obj, mesh_textured;
    std::vector< int >      pointIdxNKNSearch( smooth_factor );
    std::vector< float >    pointNKNSquaredDistance( smooth_factor );
    pcl::PointCloud< PointType > rgb_pointcloud;

    cout << "Loading mesh to PCL polygon, please wait...." << endl;
    cout << "Load mesh from file: " << input_mesh_name << endl;
    pcl::io::loadOBJFile( input_mesh_name, mesh_obj );
    cout << "Loading mesh finish" << endl;
    pcl::fromPCLPointCloud2( mesh_obj.cloud, rgb_pointcloud );
    for ( int i = 0; i < rgb_pointcloud.points.size(); ++i )
    {
        uint8_t  r = 0;
        uint8_t  g = 0;
        uint8_t  b = 0;
        float    dist = 0.0;
        int      red = 0;
        int      green = 0;
        int      blue = 0;
        uint32_t rgb;
        if ( kdtree.nearestKSearch( rgb_pointcloud.points[ i ], smooth_factor, pointIdxNKNSearch, pointNKNSquaredDistance ) > 0 )
        {
            for ( int j = 0; j < pointIdxNKNSearch.size(); ++j )
            {
                r = pcl_pc_rgb->points[ pointIdxNKNSearch[ j ] ].r;
                g = pcl_pc_rgb->points[ pointIdxNKNSearch[ j ] ].g;
                b = pcl_pc_rgb->points[ pointIdxNKNSearch[ j ] ].b;
                red += int( r );
                green += int( g );
                blue += int( b );
                dist += 1.0 / pointNKNSquaredDistance[ j ];
            }
        }
        if(  pointNKNSquaredDistance.size() < smooth_factor )
        {
            cout << "\r\n" << "Knn search fail!" << endl << endl;
        }
        rgb_pointcloud.points[ i ].r = int( red / pointIdxNKNSearch.size() );
        rgb_pointcloud.points[ i ].g = int( green / pointIdxNKNSearch.size() );
        rgb_pointcloud.points[ i ].b = int( blue / pointIdxNKNSearch.size() );
        rgb_pointcloud.points[ i ].a = 255;
        if ( i % 10000 == 0 )
        {
            printf( "\33[2K\rTexturing mesh [%u%%] ...", i * 100 / ( rgb_pointcloud.points.size()-1 ) );
            ANSI_SCREEN_FLUSH;
        }
    }
    printf( "\33[2K\rTexturing mesh [100%%] \r\n" );
    pcl::toPCLPointCloud2( rgb_pointcloud, mesh_obj.cloud );
    cout << "Saved textured mesh to: " << output_mesh_name << endl;
    pcl::io::savePLYFileBinary( output_mesh_name, mesh_obj );
    cout << "Finish!!!" << endl;
    cout << "=== Mesh texturing finish ! ===" << endl;
}

void load_parameter(ros::NodeHandle & m_ros_node_handle )
{
    std::string _offline_map_name;
    Common_tools::get_ros_parameter( m_ros_node_handle, "add_keyframe_R", g_add_keyframe_R, 10.0 );
    Common_tools::get_ros_parameter( m_ros_node_handle, "add_keyframe_t", g_add_keyframe_t, 0.15 );
   
    Common_tools::get_ros_parameter( m_ros_node_handle, "insert_pt_dis", g_insert_pt_dis, 1.0 );
    Common_tools::get_ros_parameter( m_ros_node_handle, "if_use_free_space_support", g_if_use_free_space_support, false );
    Common_tools::get_ros_parameter( m_ros_node_handle, "thickness_factor", g_thickness_factor, 1.0 );
    Common_tools::get_ros_parameter( m_ros_node_handle, "quality_factor", g_quality_factor, 0.0 );
    Common_tools::get_ros_parameter( m_ros_node_handle, "decimate_mesh", g_decimate_mesh, 1.0 );
    Common_tools::get_ros_parameter( m_ros_node_handle, "if_remove_spurious", g_if_remove_spurious, 40.0 );
    Common_tools::get_ros_parameter( m_ros_node_handle, "if_remove_spikes", g_if_remove_spikes, true );
    Common_tools::get_ros_parameter( m_ros_node_handle, "close_holes_dist", g_close_holes_dist, 20 );
    Common_tools::get_ros_parameter( m_ros_node_handle, "smooth_mesh_factor", g_smooth_mesh_factor, 5 );
    Common_tools::get_ros_parameter( m_ros_node_handle, "working_dir", g_working_dir, std::string(Common_tools::get_home_folder()).append("/r3live_output") );
    Common_tools::get_ros_parameter( m_ros_node_handle, "offline_map_name", _offline_map_name, std::string("test_mid.r3live") );
    Common_tools::get_ros_parameter( m_ros_node_handle, "texturing_smooth_factor", g_texturing_smooth_factor, 10 );
    g_offline_map_name = std::string(g_working_dir).append("/").append(_offline_map_name);
}

int main( int argc, char **argv )
{

    // system( "clear" );
    printf_program( "R3LIVE_meshing" );
    Common_tools::printf_software_version();
    ros::init( argc, argv, "R3LIVE_meshing" );
    ros::NodeHandle m_ros_node_handle;
    load_parameter( m_ros_node_handle );

    Global_map       global_map( 0 );
    Offline_map_recorder r3live_map_recorder;
    cout << "Open file from: " << g_offline_map_name << endl;
    global_map.m_if_reload_init_voxel_and_hashed_pts = 0;
    r3live_map_recorder.m_global_map = &global_map;
    
    Common_tools::load_obj_from_file( &r3live_map_recorder, g_offline_map_name );
    
    cout << "Number of rgb points: " << global_map.m_rgb_pts_vec.size() << endl;
    cout << "Size of frames: " << r3live_map_recorder.m_image_pose_vec.size() << endl;
    reconstruct_mesh( r3live_map_recorder, g_working_dir );
    cout << "=== Reconstruct mesh finish ! ===" << endl;

    std::string input_mesh_name = std::string( g_working_dir ).append( "/reconstructed_mesh.obj" );
    std::string output_mesh_name = std::string( g_working_dir ).append( "/textured_mesh.ply" );
    texture_mesh(r3live_map_recorder, input_mesh_name, output_mesh_name, g_texturing_smooth_factor );

    exit(0);
    return 0;
}