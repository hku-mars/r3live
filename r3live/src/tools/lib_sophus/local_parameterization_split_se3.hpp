#ifndef SOPHUS_TEST_LOCAL_PARAMETERIZATION_SPLIT_SE3_HPP
#define SOPHUS_TEST_LOCAL_PARAMETERIZATION_SPLIT_SE3_HPP

#include "se3.hpp"
#include <ceres/local_parameterization.h>
#include <iostream>
using std::cout;
using std::endl;

inline Eigen::Quaterniond delta_2_quaternion( const double *delta )
{
    const double norm_delta =
        sqrt( delta[ 0 ] * delta[ 0 ] + delta[ 1 ] * delta[ 1 ] + delta[ 2 ] * delta[ 2 ] );
    if ( norm_delta > 0.0 )
    {
        const double sin_delta_by_delta = sin( norm_delta ) / norm_delta;

        // Note, in the constructor w is first.
        return Eigen::Quaterniond( cos( norm_delta ),
                                   sin_delta_by_delta * delta[ 0 ],
                                   sin_delta_by_delta * delta[ 1 ],
                                   sin_delta_by_delta * delta[ 2 ] );
    }
    else
    {
        return Eigen::Quaterniond( 1, 0, 0, 0 );
    }
}

namespace Sophus
{

class LocalParameterization_split_SE3_direct : public ceres::LocalParameterization
{
  public:
    virtual ~LocalParameterization_split_SE3_direct() {}

    // SE3 plus operation for Ceres
    //
    //  T * exp(x)
    //
    virtual bool Plus( double const *T_raw, double const *delta_raw,
                       double *T_plus_delta_raw ) const
    {
        Eigen::Map<SE3d const> const     T( T_raw );
        Eigen::Map<Vector6d const> const delta( delta_raw );
        Eigen::Map<SE3d>                 T_plus_delta( T_plus_delta_raw );
        // Eigen::Map<Vector7d const> const vis( T_plus_delta_raw );
        // cout << "==================" << endl;
        // cout << delta.transpose() << endl;
        // cout << vis.transpose() << " | " ;
        T_plus_delta.translation() = T.translation() + delta.tail<3>();
        T_plus_delta.so3() = T.so3() * ( SO3d::exp( delta.head<3>() ));
        // cout <<  vis.transpose() << endl;
        return true;
    }

    // Jacobian of SE3 plus operation for Ceres
    // Dx T * exp(x)  with  x=0
    virtual bool ComputeJacobian( double const *T_raw,
                                  double *      jacobian_raw ) const
    {
        // printf("Call ComputeJacobian\r\n");
        Eigen::Map<SE3d const>                                   T( T_raw );
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jacobian( jacobian_raw );
        jacobian.setZero();
        jacobian.block<3, 3>( 0, 0 ) = Eigen::Matrix3d::Identity();
        jacobian.block<3, 3>( 4, 3 ) = Eigen::Matrix3d::Identity();

        // jacobian.block<3, 3>( 0, 0 ) = Eigen::Matrix3d::Identity();
        // jacobian.block<3, 3>( 4, 3 ) = Eigen::Matrix3d::Identity();
        return true;
    }

    virtual int GlobalSize() const { return SE3d::num_parameters; }

    virtual int LocalSize() const { return SE3d::DoF; }
};

class LocalParameterization_split_SE3 : public ceres::LocalParameterization
{
  public:
    virtual ~LocalParameterization_split_SE3() {}

    // SE3 plus operation for Ceres
    //
    //  T * exp(x)
    //
    virtual bool Plus( double const *T_raw, double const *delta_raw,
                       double *T_plus_delta_raw ) const
    {
        Eigen::Map<SE3d const> const     T( T_raw );
        Eigen::Map<Vector6d const> const delta( delta_raw );
        Eigen::Map<SE3d>                 T_plus_delta( T_plus_delta_raw );
        // Eigen::Map<Vector7d const> const vis( T_plus_delta_raw );
        // cout << "==================" << endl;
        // cout << delta.transpose() << endl;
        // cout << vis.transpose() << " | " ;
        T_plus_delta.translation() = T.translation() + delta.tail<3>();
        T_plus_delta.so3() = T.so3() * ( SO3d::exp( delta.head<3>() ));
        // cout <<  vis.transpose() << endl;
        return true;
    }

    // Jacobian of SE3 plus operation for Ceres
    // Dx T * exp(x)  with  x=0
    virtual bool ComputeJacobian( double const *T_raw,
                                  double *      jacobian_raw ) const
    {
        // printf("Call ComputeJacobian\r\n");
        Eigen::Map<SE3d const>                                   T( T_raw );
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jacobian( jacobian_raw );
        jacobian.setZero();
        jacobian.block<4, 3>( 0, 0 ) = T.so3().Dx_this_mul_exp_x_at_0();
        jacobian.block<3, 3>( 4, 3 ) = Eigen::Matrix3d::Identity();

        // jacobian.block<3, 3>( 0, 0 ) = Eigen::Matrix3d::Identity();
        // jacobian.block<3, 3>( 4, 3 ) = Eigen::Matrix3d::Identity();
        return true;
    }

    static bool compute_local_jacobian( double const *T_raw,
                                  double *      jacobian_raw )
    {
        // printf("Call ComputeJacobian\r\n");
        Eigen::Map<SE3d const>                                   T( T_raw );
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jacobian( jacobian_raw );
        jacobian.setZero();
        jacobian.block<4, 3>( 0, 0 ) = T.so3().Dx_this_mul_exp_x_at_0();
        jacobian.block<3, 3>( 4, 3 ) = Eigen::Matrix3d::Identity();

        // jacobian.block<3, 3>( 0, 0 ) = Eigen::Matrix3d::Identity();
        // jacobian.block<3, 3>( 4, 3 ) = Eigen::Matrix3d::Identity();
        return true;
    }

    virtual int GlobalSize() const { return SE3d::num_parameters; }

    virtual int LocalSize() const { return SE3d::DoF; }
};

class LocalParameterization_split_SE3_left_product : public ceres::LocalParameterization
{
  public:
    virtual ~LocalParameterization_split_SE3_left_product() {}

    // SE3 plus operation for Ceres
    //
    //  T * exp(x)
    //
    virtual bool Plus( double const *T_raw, double const *delta_raw,
                       double *T_plus_delta_raw ) const
    {
        Eigen::Map<SE3d const> const     T( T_raw );
        Eigen::Map<Vector6d const> const delta( delta_raw );
        Eigen::Map<SE3d>                 T_plus_delta( T_plus_delta_raw );
        // Eigen::Map<Vector7d const> const vis( T_plus_delta_raw );
        // cout << "==================" << endl;
        // cout << delta.transpose() << endl;
        // cout << vis.transpose() << " | " ;
        T_plus_delta.translation() = T.translation() + delta.tail<3>();
        T_plus_delta.so3() = SO3d( delta_2_quaternion( delta_raw ) ) * T.so3();
        // cout <<  vis.transpose() << endl;
        return true;
    }

    // Jacobian of SE3 plus operation for Ceres
    // Dx T * exp(x)  with  x=0
    virtual bool ComputeJacobian( double const *T_raw,
                                  double *      jacobian_raw ) const
    {
        // printf("Call ComputeJacobian\r\n");
        Eigen::Map<SE3d const>                                   T( T_raw );
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jacobian( jacobian_raw );
        jacobian.setZero();
        // From ceres solver quaternion parameterlization
        //   jacobian[0] =  x[3]; jacobian[1]  =  x[2]; jacobian[2]  = -x[1];  // NOLINT
        //   jacobian[3] = -x[2]; jacobian[4]  =  x[3]; jacobian[5]  =  x[0];  // NOLINT
        //   jacobian[6] =  x[1]; jacobian[7]  = -x[0]; jacobian[8]  =  x[3];  // NOLINT
        //   jacobian[9] = -x[0]; jacobian[10] = -x[1]; jacobian[11] = -x[2];  // NOLINT
        jacobian.block<4, 3>( 0, 0 ) << T_raw[ 3 ], T_raw[ 2 ], -T_raw[ 1 ],
            -T_raw[ 2 ], T_raw[ 3 ], T_raw[ 0 ],
            T_raw[ 1 ], -T_raw[ 0 ], T_raw[ 3 ],
            -T_raw[ 0 ], -T_raw[ 1 ], -T_raw[ 2 ];
        jacobian.block<3, 3>( 4, 3 ) = Eigen::Matrix3d::Identity();

        // jacobian.block<3, 3>( 0, 0 ) = Eigen::Matrix3d::Identity();
        // jacobian.block<3, 3>( 4, 3 ) = Eigen::Matrix3d::Identity();
        return true;
    }

    virtual int GlobalSize() const { return SE3d::num_parameters; }

    virtual int LocalSize() const { return SE3d::DoF; }
};
} // namespace Sophus

#endif
