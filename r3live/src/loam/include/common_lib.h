#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <so3_math.h>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>

#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include "tools_color_printf.hpp"
#include "tools_eigen.hpp"
#include "tools_ros.hpp"
#include <queue>
#include <deque>
#include "lib_sophus/se3.hpp"
#include "lib_sophus/so3.hpp"
// #define DEBUG_PRINT
#define USE_ikdtree
#define ESTIMATE_GRAVITY  1
#define ENABLE_CAMERA_OBS 1
// #define USE_FOV_Checker

#define printf_line std::cout << __FILE__ << " " << __LINE__ << std::endl;

#define PI_M (3.14159265358)
#define G_m_s2 (9.81)     // Gravity const in Hong Kong SAR, China
#if ENABLE_CAMERA_OBS
#define DIM_OF_STATES (29) // with vio obs
#else
#define DIM_OF_STATES (18) // For faster speed.
#endif
#define DIM_OF_PROC_N (12) // Dimension of process noise (Let Dim(SO(3)) = 3)
#define CUBE_LEN (6.0)
#define LIDAR_SP_LEN (2)
#define INIT_COV (0.0001)

#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define CONSTRAIN(v, min, max) ((v > min) ? ((v < max) ? v : max) : min)
#define ARRAY_FROM_EIGEN(mat) mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat) std::vector<decltype(mat)::Scalar>(mat.data(), mat.data() + mat.rows() * mat.cols())

#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "Log/" + name))
// using vins_estimator = fast_lio;

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZINormal;

static const Eigen::Matrix3d Eye3d(Eigen::Matrix3d::Identity());
static const Eigen::Matrix3f Eye3f(Eigen::Matrix3f::Identity());
static const Eigen::Vector3d Zero3d(0, 0, 0);
static const Eigen::Vector3f Zero3f(0, 0, 0);
// Eigen::Vector3d Lidar_offset_to_IMU(0.05512, 0.02226, 0.0297); // Horizon
static const Eigen::Vector3d Lidar_offset_to_IMU(0.04165, 0.02326, -0.0284); // Avia

struct Pose6D
{
    typedef double data_type;
    data_type offset_time;
    data_type rot[9];
    data_type acc[3];
    data_type vel[3];
    data_type pos[3];
    data_type gyr[3];
};

template <typename T = double>
inline Eigen::Matrix<T, 3, 3> vec_to_hat(Eigen::Matrix<T, 3, 1> &omega)
{
    Eigen::Matrix<T, 3, 3> res_mat_33;
    res_mat_33.setZero();
    res_mat_33(0, 1) = -omega(2);
    res_mat_33(1, 0) = omega(2);
    res_mat_33(0, 2) = omega(1);
    res_mat_33(2, 0) = -omega(1);
    res_mat_33(1, 2) = -omega(0);
    res_mat_33(2, 1) = omega(0);
    return res_mat_33;
}

template < typename T = double > 
T cot(const T theta)
{
    return 1.0 / std::tan(theta);
}

template < typename T = double >
inline Eigen::Matrix< T, 3, 3 > right_jacobian_of_rotion_matrix(const Eigen::Matrix< T, 3, 1 > & omega)
{
    //Barfoot, Timothy D, State estimation for robotics. Page 232-237
    Eigen::Matrix< T, 3, 3>   res_mat_33;

    T theta = omega.norm();
    if(std::isnan(theta) || theta == 0)
        return Eigen::Matrix< T, 3, 3>::Identity();
    Eigen::Matrix< T, 3, 1 > a = omega/ theta;
    Eigen::Matrix< T, 3, 3 > hat_a = vec_to_hat(a);
    res_mat_33 = sin(theta)/theta * Eigen::Matrix< T, 3, 3 >::Identity()
                    + (1 - (sin(theta)/theta))*a*a.transpose() 
                    + ((1 - cos(theta))/theta)*hat_a;
    // cout << "Omega: " << omega.transpose() << endl;
    // cout << "Res_mat_33:\r\n"  <<res_mat_33 << endl;
    return res_mat_33;
}

template < typename T = double >
Eigen::Matrix< T, 3, 3 > inverse_right_jacobian_of_rotion_matrix(const Eigen::Matrix< T, 3, 1> & omega)
{
    //Barfoot, Timothy D, State estimation for robotics. Page 232-237
    Eigen::Matrix< T, 3, 3>   res_mat_33;

    T theta = omega.norm();
    if(std::isnan(theta) || theta == 0)
        return Eigen::Matrix< T, 3, 3>::Identity();
    Eigen::Matrix< T, 3, 1 > a = omega/ theta;
    Eigen::Matrix< T, 3, 3 > hat_a = vec_to_hat(a);
    res_mat_33 = (theta / 2) * (cot(theta / 2)) * Eigen::Matrix<T, 3, 3>::Identity() 
                + (1 - (theta / 2) * (cot(theta / 2))) * a * a.transpose() 
                + (theta / 2) * hat_a;
    // cout << "Omega: " << omega.transpose() << endl;
    // cout << "Res_mat_33:\r\n"  <<res_mat_33 << endl;
    return res_mat_33;
}

struct Camera_Lidar_queue
{
    double m_first_imu_time = -3e88;
    double m_sliding_window_tim = 10000;
    double m_last_imu_time = -3e88;
    double m_last_visual_time = -3e88;
    double m_last_lidar_time = -3e88;
    double m_visual_init_time = 3e88;
    double m_lidar_drag_cam_tim = 5.0;
    double m_if_lidar_start_first = 1;
    double m_camera_imu_td = 0;

    int m_if_acc_mul_G = 0;

    int m_if_have_lidar_data = 0;
    int m_if_have_camera_data = 0;
    int m_if_lidar_can_start = 1;
    Eigen::Vector3d g_noise_cov_acc;
    Eigen::Vector3d g_noise_cov_gyro;

    std::string m_bag_file_name;
    int m_if_dump_log = 1;

    // std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> *m_camera_frame_buf = nullptr;
    std::deque<sensor_msgs::PointCloud2::ConstPtr> *m_liar_frame_buf = nullptr;

    double time_wrt_first_imu_time(double & time)
    {
        return time - m_first_imu_time;
    }
    
    Camera_Lidar_queue()
    {
        m_if_have_lidar_data = 0;
        m_if_have_camera_data = 0;
    };
    ~Camera_Lidar_queue(){};

    double imu_in(const double in_time)
    {
        if (m_first_imu_time < 0)
        {
            m_first_imu_time = in_time;
        }
        m_last_imu_time = std::max(in_time, m_last_imu_time);
        //m_last_imu_time = in_time;
        return m_last_imu_time;
    }

    int lidar_in(const double &in_time)
    {
        // cout << "LIDAR in " << endl;
        if (m_if_have_lidar_data == 0)
        {
            m_if_have_lidar_data = 1;
            // cout << ANSI_COLOR_BLUE_BOLD << "Have LiDAR data" << endl;
        }
        if (in_time < m_last_imu_time - m_sliding_window_tim)
        {
            std::cout << ANSI_COLOR_RED_BOLD << "LiDAR incoming frame too old, need to be drop!!!" << ANSI_COLOR_RESET << std::endl;
            // TODO: Drop LiDAR frame
        }
        // m_last_lidar_time = in_time;
        return 1;
    }

    int camera_in(const double &in_time)
    {
        if (in_time < m_last_imu_time - m_sliding_window_tim)
        {
            std::cout << ANSI_COLOR_RED_BOLD << "Camera incoming frame too old, need to be drop!!!" << ANSI_COLOR_RESET << std::endl;
            // TODO: Drop camera frame
        }
        return 1;
    }

    double get_lidar_front_time()
    {
        if (m_liar_frame_buf != nullptr && m_liar_frame_buf->size())
        {
            m_last_lidar_time = m_liar_frame_buf->front()->header.stamp.toSec() + 0.1;
            return m_last_lidar_time;
        }
        else
        {
            return -3e88;
        }
    }

    double get_camera_front_time()
    {
        return m_last_visual_time + m_camera_imu_td;
    }

    bool if_camera_can_process()
    {
        m_if_have_camera_data = 1;
        double cam_last_time = get_camera_front_time();
        double lidar_last_time = get_lidar_front_time();

        if (m_if_have_lidar_data != 1)
        {
            return true;
        }

        if (cam_last_time < 0 || lidar_last_time < 0)
        {
            return false;
        }

        if (lidar_last_time <= cam_last_time)
        {
            // LiDAR data need process first.
            // return true;
            return false;
        }
        else
        {
            // scope_color(ANSI_COLOR_YELLOW_BOLD);
            // cout << "Camera can update, " << get_lidar_front_time() - m_first_imu_time << " | " << get_camera_front_time() - m_first_imu_time << endl;
            return true;
        }
        return false;
    }

    void display_last_cam_LiDAR_time()
    {
        double cam_last_time = get_camera_front_time();
        double lidar_last_time = get_lidar_front_time();
        scope_color(ANSI_COLOR_GREEN_BOLD);
        cout<< std::setprecision(15) <<  "Camera time = " << cam_last_time << ", LiDAR last time =  "<< lidar_last_time << endl;        
    }

    bool if_lidar_can_process()
    {
        // m_if_have_lidar_data = 1;
        double cam_last_time = get_camera_front_time();
        double lidar_last_time = get_lidar_front_time();
        if (m_if_have_camera_data == 0)
        {
            return true;
        }

        if (cam_last_time < 0 || lidar_last_time < 0)
        {
            // cout << "Cam_tim = " << cam_last_time << ", lidar_last_time = " << lidar_last_time << endl; 
            return false;
        }


        if (lidar_last_time > cam_last_time)
        {
            // Camera data need process first.
            return false;
        }
        else
        {
            // scope_color(ANSI_COLOR_BLUE_BOLD);
            // cout << "LiDAR can update, " << get_lidar_front_time() - m_first_imu_time << " | " << get_camera_front_time() - m_first_imu_time << endl;
            // printf_line;
            return true;
        }
        return false;
    }
};

struct MeasureGroup // Lidar data and imu dates for the curent process
{
    MeasureGroup()
    {
        this->lidar.reset(new PointCloudXYZINormal());
    };
    double lidar_beg_time;
    double lidar_end_time;
    PointCloudXYZINormal::Ptr lidar;
    std::deque<sensor_msgs::Imu::ConstPtr> imu;
};

struct StatesGroup
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix3d rot_end;                                 // [0-2] the estimated attitude (rotation matrix) at the end lidar point
    Eigen::Vector3d pos_end;                                 // [3-5] the estimated position at the end lidar point (world frame)
    Eigen::Vector3d vel_end;                                 // [6-8] the estimated velocity at the end lidar point (world frame)
    Eigen::Vector3d bias_g;                                  // [9-11] gyroscope bias
    Eigen::Vector3d bias_a;                                  // [12-14] accelerator bias
    Eigen::Vector3d gravity;                                 // [15-17] the estimated gravity acceleration

    Eigen::Matrix3d rot_ext_i2c;                             // [18-20] Extrinsic between IMU frame to Camera frame on rotation.
    Eigen::Vector3d pos_ext_i2c;                             // [21-23] Extrinsic between IMU frame to Camera frame on position.
    double          td_ext_i2c_delta;                        // [24]    Extrinsic between IMU frame to Camera frame on position.
    vec_4           cam_intrinsic;                           // [25-28] Intrinsice of camera [fx, fy, cx, cy]
    Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> cov; // states covariance
    double last_update_time = 0;
    double          td_ext_i2c;
    StatesGroup()
    {
        rot_end = Eigen::Matrix3d::Identity();
        pos_end = vec_3::Zero();
        vel_end = vec_3::Zero();
        bias_g = vec_3::Zero();
        bias_a = vec_3::Zero();
        gravity = Eigen::Vector3d(0.0, 0.0, 9.805);
        // gravity = Eigen::Vector3d(0.0, 9.805, 0.0);

        //Ext camera w.r.t. IMU
        rot_ext_i2c = Eigen::Matrix3d::Identity();
        pos_ext_i2c = vec_3::Zero();

        cov = Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Identity() * INIT_COV;
        // cov.block(18, 18, 6,6) *= 0.1;
        last_update_time = 0;
        td_ext_i2c_delta = 0;
        td_ext_i2c = 0;
    }

    ~StatesGroup(){}

    StatesGroup operator+(const Eigen::Matrix<double, DIM_OF_STATES, 1> &state_add)
    {
        StatesGroup a = *this;
        // a.rot_end = this->rot_end * Sophus::SO3d::exp(vec_3(state_add(0, 0), state_add(1, 0), state_add(2, 0) ) );
        a.rot_end = this->rot_end * Exp(state_add(0), state_add(1), state_add(2));
        a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
        a.vel_end = this->vel_end + state_add.block<3, 1>(6, 0);
        a.bias_g = this->bias_g + state_add.block<3, 1>(9, 0);
        a.bias_a = this->bias_a + state_add.block<3, 1>(12, 0);
#if ESTIMATE_GRAVITY
        a.gravity = this->gravity + state_add.block<3, 1>(15, 0);
#endif

        a.cov = this->cov;
        a.last_update_time = this->last_update_time;
#if ENABLE_CAMERA_OBS                
        //Ext camera w.r.t. IMU
        a.rot_ext_i2c = this->rot_ext_i2c * Exp(  state_add(18), state_add(19), state_add(20) );
        a.pos_ext_i2c = this->pos_ext_i2c + state_add.block<3,1>( 21, 0 );
        a.td_ext_i2c_delta = this->td_ext_i2c_delta + state_add(24);
        a.cam_intrinsic = this->cam_intrinsic + state_add.block(25, 0, 4, 1);
#endif
        return a;
    }

    StatesGroup &operator+=(const Eigen::Matrix<double, DIM_OF_STATES, 1> &state_add)
    {
        this->rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
        this->pos_end += state_add.block<3, 1>(3, 0);
        this->vel_end += state_add.block<3, 1>(6, 0);
        this->bias_g += state_add.block<3, 1>(9, 0);
        this->bias_a += state_add.block<3, 1>(12, 0);
#if ESTIMATE_GRAVITY
        this->gravity += state_add.block<3, 1>(15, 0);
#endif
#if ENABLE_CAMERA_OBS        
        //Ext camera w.r.t. IMU
        this->rot_ext_i2c = this->rot_ext_i2c * Exp(  state_add(18), state_add(19), state_add(20));
        this->pos_ext_i2c = this->pos_ext_i2c + state_add.block<3,1>( 21, 0 );
        this->td_ext_i2c_delta = this->td_ext_i2c_delta + state_add(24);
        this->cam_intrinsic = this->cam_intrinsic + state_add.block(25, 0, 4, 1);   
#endif
        return *this;
    }

    Eigen::Matrix<double, DIM_OF_STATES, 1> operator-(const StatesGroup &b)
    {
        Eigen::Matrix<double, DIM_OF_STATES, 1> a;
        Eigen::Matrix3d rotd(b.rot_end.transpose() * this->rot_end);
        a.block<3, 1>(0, 0) = SO3_LOG(rotd);
        a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
        a.block<3, 1>(6, 0) = this->vel_end - b.vel_end;
        a.block<3, 1>(9, 0) = this->bias_g - b.bias_g;
        a.block<3, 1>(12, 0) = this->bias_a - b.bias_a;
        a.block<3, 1>(15, 0) = this->gravity - b.gravity;

#if ENABLE_CAMERA_OBS    
        //Ext camera w.r.t. IMU
        Eigen::Matrix3d rotd_ext_i2c(b.rot_ext_i2c.transpose() * this->rot_ext_i2c);
        a.block<3, 1>(18, 0) = SO3_LOG(rotd_ext_i2c);
        a.block<3, 1>(21, 0) = this->pos_ext_i2c - b.pos_ext_i2c;
        a(24) = this->td_ext_i2c_delta - b.td_ext_i2c_delta;
        a.block<4, 1>(25, 0) = this->cam_intrinsic - b.cam_intrinsic;
#endif
        return a;
    }

    static void display(const StatesGroup &state, std::string str = std::string("State: "))
    {
        vec_3 angle_axis = SO3_LOG(state.rot_end) * 57.3;
        printf("%s |", str.c_str());
        printf("[%.5f] | ", state.last_update_time);
        printf("(%.3f, %.3f, %.3f) | ", angle_axis(0), angle_axis(1), angle_axis(2));
        printf("(%.3f, %.3f, %.3f) | ", state.pos_end(0), state.pos_end(1), state.pos_end(2));
        printf("(%.3f, %.3f, %.3f) | ", state.vel_end(0), state.vel_end(1), state.vel_end(2));
        printf("(%.3f, %.3f, %.3f) | ", state.bias_g(0), state.bias_g(1), state.bias_g(2));
        printf("(%.3f, %.3f, %.3f) \r\n", state.bias_a(0), state.bias_a(1), state.bias_a(2));
    }
};

template <typename T>
T rad2deg(T radians)
{
    return radians * 180.0 / PI_M;
}

template <typename T>
T deg2rad(T degrees)
{
    return degrees * PI_M / 180.0;
}

template <typename T>
auto set_pose6d(const double t, const Eigen::Matrix<T, 3, 1> &a, const Eigen::Matrix<T, 3, 1> &g,
                const Eigen::Matrix<T, 3, 1> &v, const Eigen::Matrix<T, 3, 1> &p, const Eigen::Matrix<T, 3, 3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++)
            rot_kp.rot[i * 3 + j] = R(i, j);
    }
    // Eigen::Map<Eigen::Matrix3d>(rot_kp.rot, 3,3) = R;
    return std::move(rot_kp);
}

#endif
