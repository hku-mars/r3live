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

// This tools aim at provideing a serious of convinient tools which enable user can code with Eigen library.
// The features of this tools mainly can be summarized as follow:
//      1. The alias (typedef, using) of the Eigen' data structs (or classes).
//      2. Operator-based tools for Eigen' data structs, include "A<<B", "A>>B", "A*B", "A*=B", and etc.
// Anthor: Jiarong Lin
// E-Mail: ziv.lin.ljr@gmail.com

#ifndef __TOOLS_EIGEN_HPP__
#define __TOOLS_EIGEN_HPP__
#include <Eigen/Eigen>
#include "tools_color_printf.hpp"

// For the consideration of avoiding alignment error while running, we prefer using datatype of Eigen with no alignment.
// In addition, for higher realtime performance (don't expect too much), you can modify the option with alignment, but you might carefully be aware of the crash of the program.
#define EIGEN_DATA_TYPE_DEFAULT_OPTION Eigen::DontAlign
// #define EIGEN_DATA_TYPE_DEFAULT_OPTION Eigen::AutoAlign

template < int M, int N, int option = (EIGEN_DATA_TYPE_DEFAULT_OPTION|Eigen::RowMajor) >
using eigen_mat_d = Eigen::Matrix< double, M, N, option >;

template < int M, int N, int option = (EIGEN_DATA_TYPE_DEFAULT_OPTION|Eigen::RowMajor) >
using eigen_mat_d = Eigen::Matrix< double, M, N, option >;

template < int M, int N, int option = (EIGEN_DATA_TYPE_DEFAULT_OPTION|Eigen::RowMajor) >
using eigen_mat_f = Eigen::Matrix< float, M, N, option >;

template < typename T, int M, int N, int option = (EIGEN_DATA_TYPE_DEFAULT_OPTION|Eigen::RowMajor) >
using eigen_mat_t = Eigen::Matrix< T, M, N, option >;

template < int M, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
using eigen_vec_d = Eigen::Matrix< double, M, 1, option >;

template < int M, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
using eigen_vec_f = Eigen::Matrix< float, M, 1, option >;

template < typename T, int M, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
using eigen_vec_t = Eigen::Matrix< T, M, 1, option >;

template < typename T, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
using eigen_q_t = Eigen::Quaternion< T, option >;

template < typename T >
using eigen_angleaxis_t = Eigen::AngleAxis< T >;

template < typename T, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
using eigen_pose_t = Eigen::Transform< T, 3, Eigen::Isometry, option >;

template < int M, int N >
using eigen_mat = eigen_mat_d< M, N >;

template < int M >
using eigen_vec = eigen_vec_d< M >;

typedef eigen_vec< 2 >                                                                 vec_2;
typedef eigen_vec< 3 >                                                                 vec_3;
typedef eigen_vec< 4 >                                                                 vec_4;
typedef eigen_vec< 6 >                                                                 vec_6;
typedef eigen_vec< 7 >                                                                 vec_7;
typedef eigen_vec< 12 >                                                                vec_12;
typedef eigen_mat< 3, 3 >                                                              mat_3_3;
typedef eigen_mat< 4, 4 >                                                              mat_4_4;
typedef eigen_mat< 6, 6 >                                                              mat_6_6;
typedef eigen_mat< 12, 12 >                                                            mat_12;
typedef eigen_mat< 6, 12 >                                                             mat_6_12;
typedef eigen_mat< 12, 6 >                                                             mat_12_6;
typedef eigen_angleaxis_t< double >                                                    eigen_angleaxis;
typedef Eigen::Quaternion< double, EIGEN_DATA_TYPE_DEFAULT_OPTION >                    eigen_q;
typedef Eigen::Transform< double, 3, Eigen::Isometry, EIGEN_DATA_TYPE_DEFAULT_OPTION > eigen_pose;
typedef std::vector< eigen_q >                                                         eigen_q_vec;

// namespace Common_tools
// {

template < typename T >
inline T angle_refine( const T &rad )
{
    // Refine angle to [-pi, pi]
    T rad_afr_refined = ( rad - ( floor( rad / T( 2 * M_PI ) ) * T( 2 * M_PI ) ) );
    if ( rad_afr_refined > T( M_PI ) )
    {
        rad_afr_refined -= T( 2 * M_PI );
    }
    return rad_afr_refined;
}

/*****
    Some operator based tools for Eigen::Vector<T>
    Example:
        a. Eigen::Vector<T> from array:                Eigen::Vector<T> << data_rhs
        b. Eigen::Vector<T> to array:                  Eigen::Vector<T> >> data_rhs
*****/
template < typename T, int M, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
inline const void operator<<( eigen_vec_t< T, M, option > &eigen_vec_lhs, const T *data_rhs )
{
    for ( size_t i = 0; i < M; i++ )
    {
        eigen_vec_lhs( i ) = data_rhs[ i ];
    }
}

template < typename T, int M, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
inline const void operator>>( const eigen_vec_t< T, M, option > &eigen_vec_lhs, T *data_rhs )
{
    for ( size_t i = 0; i < M; i++ )
    {
        data_rhs[ i ] = eigen_vec_lhs( i );
    }
}

template < typename T, int M, typename TT = T, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
inline const void operator<<( eigen_vec_t< T, M, option > &eigen_vec_lhs, const std::pair< std::vector< TT > *, int > &std_vector_start )
{
    // Loading data from a std::vector, from the starting point
    // Example: eigen_vec_lhs << std::make_pair(&std::vector, starting_point)
    for ( size_t i = 0; i < M; i++ )
    {
        eigen_vec_lhs( i ) = T( ( *std_vector_start.first )[ std_vector_start.second + i ] );
    }
}

template < typename T, int M, typename TT = T, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
inline const void operator<<( const eigen_vec_t< T, M, option > &eigen_vec_lhs, std::pair< std::vector< TT > *, int > &std_vector_start )
{
    for ( size_t i = 0; i < M; i++ )
    {
        ( *std_vector_start.first )[ std_vector_start.second + i ] = TT( eigen_vec_lhs( i ) );
    }
}

/*****
    Some operator based tools for Eigen::Quaternion, before using these tools, make sure you are using the uniform quaternion,
    otherwise some of the unwanted results will be happend.
    Example:
        a. Quaternion from array:                   Eigen::Quaternion << data_rhs
        b. Quaternion to array:                     Eigen::Quaternion >> data_rhs
        c. Rotation angle multiply(*=) a scalar:    Eigen::Quaternion *= scalar
        d. Rotation angle multiply(*=) a scalar:    Eigen::Quaternion * scalar
*****/
template < typename T, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
inline const void operator<<( eigen_q_t< T, option > &eigen_q_lhs, const T *data_rhs )
{
    eigen_q_lhs.w() = data_rhs[ 0 ];
    eigen_q_lhs.x() = data_rhs[ 1 ];
    eigen_q_lhs.y() = data_rhs[ 2 ];
    eigen_q_lhs.z() = data_rhs[ 3 ];
}

template < typename T, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
inline const void operator>>( const eigen_q_t< T, option > &eigen_q_lhs, T *data_rhs )
{
    data_rhs[ 0 ] = eigen_q_lhs.w();
    data_rhs[ 1 ] = eigen_q_lhs.x();
    data_rhs[ 2 ] = eigen_q_lhs.y();
    data_rhs[ 3 ] = eigen_q_lhs.z();
}

template < typename T, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
inline const eigen_q_t< T, option > &operator*=( eigen_q_t< T, option > &eigen_q_lhs, const T &s )
{
    Eigen::AngleAxis< T > angle_axis( eigen_q_lhs );
    angle_axis *= s;
    eigen_q_lhs = eigen_q_t< T, option >( angle_axis );
    return eigen_q_lhs;
}

template < typename T, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
inline const eigen_q_t< T, option > &operator*( const eigen_q_t< T, option > &eigen_q_lhs, const T &s )
{
    Eigen::AngleAxis< T > angle_axis( eigen_q_lhs );
    angle_axis *= s;
    return eigen_q_t< T, option >( angle_axis );
}

/*****
    Conversion between eigen angle_axis and data array
    Example:  
        a. AngleAxis from array: Eigen::AngleAxis << data_rhs
        b. AngleAxis to array:   Eigen::AngleAxis >> data_rhs
        c. Rotation angle multiply(*=) a scalar:    Eigen::AngleAxis *= scalar
        d. Rotation angle multiply(*=) a scalar:    Eigen::AngleAxis * scalar
*****/
template < typename T >
inline const void operator<<( Eigen::AngleAxis< T > &eigen_axisangle_lhs, const T *data_rhs )
{
    T vec_norm = sqrt( data_rhs[ 0 ] * data_rhs[ 0 ] + data_rhs[ 1 ] * data_rhs[ 1 ] + data_rhs[ 2 ] * data_rhs[ 2 ] );
    if ( vec_norm != T( 0.0 ) )
    {
        eigen_axisangle_lhs.angle() = vec_norm;
        eigen_axisangle_lhs.axis() << data_rhs[ 0 ] / vec_norm, data_rhs[ 1 ] / vec_norm, data_rhs[ 2 ] / vec_norm;
    }
    else
    {
        eigen_axisangle_lhs.angle() = vec_norm;
        eigen_axisangle_lhs.axis() << vec_norm * data_rhs[ 0 ], vec_norm * data_rhs[ 1 ], vec_norm * data_rhs[ 2 ]; // For the consideration of derivation
    }
}

template < typename T >
inline const void operator>>( const Eigen::AngleAxis< T > &eigen_axisangle_lhs, T *data_rhs )
{
    T vec_norm = eigen_axisangle_lhs.angle();
    data_rhs[ 0 ] = eigen_axisangle_lhs.axis()( 0 ) * vec_norm;
    data_rhs[ 1 ] = eigen_axisangle_lhs.axis()( 1 ) * vec_norm;
    data_rhs[ 2 ] = eigen_axisangle_lhs.axis()( 2 ) * vec_norm;
}

template < typename T >
inline const Eigen::AngleAxis< T > operator*=( Eigen::AngleAxis< T > &eigen_axisangle_lhs, const T &s )
{
    eigen_axisangle_lhs.angle() *= s;
    return eigen_axisangle_lhs;
}

template < typename T >
inline const Eigen::AngleAxis< T > operator*( const Eigen::AngleAxis< T > &eigen_axisangle_lhs, const T &s )
{
    Eigen::AngleAxis< T > angle_axis( eigen_axisangle_lhs );
    angle_axis.angle() *= s;
    return angle_axis;
}

// } // namespace Common_tools

#endif