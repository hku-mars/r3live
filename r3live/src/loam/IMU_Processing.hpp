#pragma once
#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
// #include <fast_lio/States.h>
#include <geometry_msgs/Vector3.h>

/// *************Preconfiguration
#define MAX_INI_COUNT (20)
const inline bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};
bool check_state(StatesGroup &state_inout);
void check_in_out_state(const StatesGroup &state_in, StatesGroup &state_inout);

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Process(const MeasureGroup &meas, StatesGroup &state, PointCloudXYZINormal::Ptr pcl_un_);
  void Reset();
  void IMU_Initial(const MeasureGroup &meas, StatesGroup &state, int &N);

  // Eigen::Matrix3d Exp(const Eigen::Vector3d &ang_vel, const double &dt);

  void IntegrateGyr(const std::vector<sensor_msgs::Imu::ConstPtr> &v_imu);

  void UndistortPcl(const MeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZINormal &pcl_in_out);
  void lic_state_propagate(const MeasureGroup &meas, StatesGroup &state_inout);
  void lic_point_cloud_undistort(const MeasureGroup &meas,  const StatesGroup &state_inout, PointCloudXYZINormal &pcl_out);
  StatesGroup imu_preintegration(const StatesGroup & state_inout, std::deque<sensor_msgs::Imu::ConstPtr> & v_imu,  double end_pose_dt = 0);
  ros::NodeHandle nh;

  void Integrate(const sensor_msgs::ImuConstPtr &imu);
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);

  Eigen::Vector3d angvel_last;
  Eigen::Vector3d acc_s_last;

  Eigen::Matrix<double,DIM_OF_PROC_N,1> cov_proc_noise;

  Eigen::Vector3d cov_acc;
  Eigen::Vector3d cov_gyr;

  // std::ofstream fout;

 public:
  /*** Whether is the first frame, init for first frame ***/
  bool b_first_frame_ = true;
  bool imu_need_init_ = true;

  int init_iter_num = 1;
  Eigen::Vector3d mean_acc;
  Eigen::Vector3d mean_gyr;

  /*** Undistorted pointcloud ***/
  PointCloudXYZINormal::Ptr cur_pcl_un_;

  //// For timestamp usage
  sensor_msgs::ImuConstPtr last_imu_;

  /*** For gyroscope integration ***/
  double start_timestamp_;
  /// Making sure the equal size: v_imu_ and v_rot_
  std::deque<sensor_msgs::ImuConstPtr> v_imu_;
  std::vector<Eigen::Matrix3d> v_rot_pcl_;
  std::vector<Pose6D> IMU_pose;
};
