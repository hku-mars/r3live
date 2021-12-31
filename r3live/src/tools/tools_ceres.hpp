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
#include <Eigen/Eigen>
#include <ceres/ceres.h>
#include <iostream>
#include <stdio.h>
#include "tools_eigen.hpp"
#include "tools_color_printf.hpp"
#include "tools_timer.hpp"
template <int Dim>
using jet_d = ceres::Jet<double, Dim>;
template <int Dim>
using jet_f = ceres::Jet<float, Dim>;


template < typename T, int M, int N >
inline eigen_mat_t<T, N, M > get_jacobian( const eigen_vec_t< ceres::Jet< T, M >, N > &eigen_vec_jet )
{
    eigen_mat_t<T, N, M> jacobian_mat;
    for(size_t i =0 ; i < N; i++)
    {
        for(size_t j =0; j< M; j++)
        {
            jacobian_mat(i, j) = eigen_vec_jet(i).v(j);
        }
    }
    return jacobian_mat;
}

template < typename T, int M, int N >
inline eigen_vec_t< T, N > get_val( const eigen_vec_t< ceres::Jet< T, M >, N > &eigen_vec_jet )
{
    eigen_vec_t< T, N > val_vec;
    for ( size_t i = 0; i < N; i++ )
    {
        val_vec( i ) = eigen_vec_jet( i ).a;
    }
    return val_vec;
}

namespace Common_tools
{
const double MAXIMUM_TOLERANCE  = 1e-5;
const int eval_abs = 1;
/****** Usage *****/
// ceres::Problem::EvaluateOptions eval_options;
// std::vector<double> gradients;
// ceres::CRSMatrix jacobian_matrix;
// problem.Evaluate( eval_options, &total_cost, &residuals, &gradients, &jacobian_matrix );
// save_ceres_crs_matrix_to_txt("/home/ziv/jacobian.txt", jacobian_matrix);
// Refer to crs_matrix.h @  https://github.com/ceres-solver/ceres-solver/blob/master/include/ceres/crs_matrix.h

inline Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >  crs_to_eigen_matrix( const ceres::CRSMatrix &crs_matrix )
{
    // Common_tools::Timer tim;
    // Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > dy_mat(crs_matrix.num_rows, crs_matrix.num_cols );
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > dy_mat =  Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic >::Zero(crs_matrix.num_rows, crs_matrix.num_cols);
    int val_idx = 0;
    for ( size_t i = 0; i < crs_matrix.rows.size() - 1; i++ )
    {
        int curr_row = i;
        for ( int ii = crs_matrix.rows[ i ]; ii < crs_matrix.rows[ i + 1 ]; ii++ )
        {
            int curr_col = crs_matrix.cols[ ii ];
            //printf( "[%d. %d][%d. %d]--%lf\r\n ", crs_matrix.num_rows, crs_matrix.num_cols, curr_row, curr_col, crs_matrix.values[ val_idx ] );
            dy_mat( curr_row, curr_col ) = crs_matrix.values[ val_idx ];
            val_idx++;
        }
    }
    //printf( "Matrix detisity = %.2f \r\n", ( float ) val_idx / ( float ) ( crs_matrix.num_rows * crs_matrix.num_cols ) );
    return dy_mat;
}

template < typename T >
Eigen::Matrix< T, 3, 1 > ceres_quadternion_parameterization( const Eigen::Quaternion< T > &q_in )
{
    Eigen::Matrix< T, 3, 1 > res;
    T                        mod_delta = acos( q_in.w() );
    if ( abs( q_in.w() - 1 ) <= 1e-10 )
    {
        res( 0 ) = 0;
        res( 1 ) = 0;
        res( 2 ) = 0;
    }
    else
    {
        T sin_mod_delta = std::sin( mod_delta ) / mod_delta;
        res( 0 ) = q_in.x() / sin_mod_delta;
        res( 1 ) = q_in.y() / sin_mod_delta;
        res( 2 ) = q_in.z() / sin_mod_delta;
    }
    return res;
}

template < typename T, typename TT >
void ceres_quadternion_delta( T *t_curr, TT &t_delta )
{
    // Refer from: http://ceres-solver.org/nnls_modeling.html#_CPPv2N5ceres26QuaternionParameterizationE
    // cout << "Input delta:" << t_delta.transpose() << endl;
    T mod_delta = t_delta.norm();
    if ( mod_delta == 0 ) //equal to identity.
    {
        return;
    }
    T sin_by_mod_delta = std::sin( mod_delta ) / mod_delta;

    Eigen::Quaternion< T >               q_delta;
    Eigen::Map< Eigen::Quaternion< T > > q_w_curr( t_curr );

    q_delta.w() = std::cos( mod_delta );
    q_delta.x() = sin_by_mod_delta * t_delta( 0 );
    q_delta.y() = sin_by_mod_delta * t_delta( 1 );
    q_delta.z() = sin_by_mod_delta * t_delta( 2 );
    // cout << q_delta.coeffs().transpose() << endl;
    q_w_curr = q_delta * ( q_w_curr );
}

template < typename T >
Eigen::Matrix< T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor > std_vector_to_eigen_matrix( std::vector< T > &in_vector, int number_or_cols = 0 )
{
    Eigen::Matrix< T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor > res_mat;
    size_t                                             total_size = in_vector.size();
    size_t                                             num_cols = 1;
    if ( number_or_cols )
    {
        num_cols = number_or_cols;
        assert( ( total_size % num_cols ) == 0 );
    }
    res_mat.resize( total_size / num_cols, num_cols );
    for ( size_t i = 0; i < total_size; i++ )
    {
        res_mat( i ) = in_vector[ i ];
    }
    //cout << "Size of res_mat: " << res_mat.rows() << " X " << res_mat.cols() << endl;
    return res_mat;
}

inline void save_ceres_crs_matrix_to_txt( std::string file_name, ceres::CRSMatrix &crs_matrix, int do_print = 0 )
{
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic , Eigen::RowMajor> dy_mat = crs_to_eigen_matrix( crs_matrix );
    FILE *                                                  fp;
    fp = fopen( file_name.c_str(), "w+" );
    if ( do_print )
    {
        for ( int j = 0; j < dy_mat.rows(); j++ )
        {
            for ( int i = 0; i < dy_mat.cols(); i++ )
            {
                fprintf( fp, "%lf ", dy_mat( j, i ) );
            }
        }
    }
    else
    {
        for ( int i = 0; i < dy_mat.cols(); i++ )
        {
            for ( int j = 0; j < dy_mat.rows(); j++ )
            {

                fprintf( fp, "%lf ", dy_mat( j, i ) );
            }
        }
    }

    fclose( fp );
};

/*****
    Evaluate is two ceres::Jet is the same, is_ceres_same_absolute() evaluate the absolute error while the is_ceres_same_relative evaluate the relative error.
*****/
template < typename T, int N >
inline const bool is_ceres_same_absolute( const ceres::Jet< T, N > &val_1, const ceres::Jet< T, N > &val_2, double threshold_tor = MAXIMUM_TOLERANCE )
{
    // Aim at compare the absolute error
    bool is_same = true;
    if ( std::fabs( val_1.a - val_2.a ) > threshold_tor )
    {
        is_same = false;
    }

    for ( size_t i = 0; i < N; i++ )
    {
        if ( std::fabs( val_1.v[ i ] - val_2.v[ i ] ) > threshold_tor )
        {
            is_same = false;
        }
    }
    return is_same;
}

template < typename T, int N >
inline const bool is_ceres_same_relative( const ceres::Jet< T, N > &val_1, const ceres::Jet< T, N > &val_2, double threshold_tor = MAXIMUM_TOLERANCE )
{
    // Aim at compare the relative error
    bool is_same = true;
    if ( std::fabs( val_1.a - val_2.a ) > threshold_tor * std::fabs( val_1.a + val_2.a ) )
    {
        is_same = false;
    }

    for ( size_t i = 0; i < N; i++ )
    {
        if ( std::fabs( val_1.v[ i ] - val_2.v[ i ] ) > threshold_tor * std::fabs( val_1.v[ i ] + val_2.v[ i ] ) )
        {
            is_same = false;
        }
    }
    return is_same;
}

template < typename T, int N >
inline const bool is_ceres_same( const ceres::Jet< T, N > &val_1, const ceres::Jet< T, N > &val_2, double threshold_tor = MAXIMUM_TOLERANCE )
{
    if ( eval_abs )
    {
        return is_ceres_same_absolute( val_1, val_2, threshold_tor );
    }
    else
    {
        return is_ceres_same_relative( val_1, val_2, threshold_tor );
    }
}

template < typename T, int N >
inline const bool is_ceres_array_same( const ceres::Jet< T, N > *ceres_array_1, const ceres::Jet< T, N > *ceres_array_2, int size_of_array = 1 )
{
    bool is_same = true;
    for ( size_t i = 0; i < size_of_array; i++ )
    {
        if ( !is_ceres_same( ceres_array_1[ i ], ceres_array_2[ i ] ) )
        {
            is_same = false;
            cout << "Idx [" << i << "] not the same " << endl;
        }
    }
    return is_same;
}

template < typename T, int N >
void printf_single_ceres_data( const ceres::Jet< T, N > *data_1, int size_of_array, short mode_0 = 0b11 )
{
    if ( mode_0 & 0b01 != 0 )
    {
        for ( size_t i = 0; i < size_of_array; i++ )
        {
            cout << data_1[ i ].a << ", ";
        }
        // cout <<"\b\b"; // delete the last two char
        cout << endl;
    }
    if ( mode_0 & 0b10 != 0 )
    {
        for ( size_t i = 0; i < size_of_array; i++ )
        {
            cout << data_1[ i ].v.transpose() << "\r\n";
        }
    }
}

template < typename T, int N >
void printf_paired_ceres_data( const ceres::Jet< T, N > *data_1, const ceres::Jet< T, N > *data_2, int size_of_array = 1 )
{
    cout << ANSI_COLOR_CYAN_BOLD << std::setprecision( 10 );
    printf_single_ceres_data( data_1, size_of_array );
    cout << ANSI_COLOR_YELLOW_BOLD;
    printf_single_ceres_data( data_2, size_of_array );
    cout << ANSI_COLOR_RESET;
}

// ANCHOR - huber_loss
inline void apply_ceres_loss_fun( ceres::LossFunction * loss_function,  Eigen::VectorXd &residuals, std::vector< Eigen::Matrix< double, -1, -1, Eigen::RowMajor > > & jacobians )
{
    // http://ceres-solver.org/nnls_modeling.html#lossfunction
    // How ceres use loss function:   ceres_1.14.x/internal/ceres/residual_block.cc
    // How jacobian matrix corrected: ceres_1.14.x/internal/ceres/corrector.cc
    if ( loss_function == nullptr )
    {
        return;
    }
    double   sq_norm = residuals.squaredNorm();
    double    rho[ 3 ];
    loss_function->Evaluate( sq_norm, rho );
    double residual_scaling_, alpha_sq_norm_;

    // printf("sq_norm: %f, rho[0]: %f, rho[1]: %f, rho[2]: %f\n", sq_norm, rho[0], rho[1], rho[2]);

    double sqrt_rho1_ = sqrt( rho[ 1 ] );

    sqrt_rho1_ = sqrt( rho[ 1 ] );
    if ( ( sq_norm == 0.0 ) || ( rho[ 2 ] <= 0.0 ) )
    {
        residual_scaling_ = sqrt_rho1_;
        alpha_sq_norm_ = 0.0;
    }
    else
    {
        const double D = 1.0 + 2.0 * sq_norm * rho[ 2 ] / rho[ 1 ];
        const double alpha = 1.0 - sqrt( D );
        // Calculate the constants needed by the correction routines.
        residual_scaling_ = sqrt_rho1_ / ( 1 - alpha );
        alpha_sq_norm_ = alpha / sq_norm;
    }
    for(unsigned int i =0 ; i< jacobians.size(); i++)
    {
        jacobians[i] = sqrt_rho1_ * (jacobians[i] - alpha_sq_norm_ * residuals * (residuals.transpose() * jacobians[i]));
    }
    residuals *= residual_scaling_;
}

inline void apply_ceres_loss_fun( ceres::LossFunction * loss_function,  Eigen::VectorXd &residuals,  Eigen::Matrix< double, -1, -1, Eigen::RowMajor >  & jacobian )
{
    std::vector< Eigen::Matrix< double, -1, -1, Eigen::RowMajor > >  jacobians {jacobian};
    apply_ceres_loss_fun(loss_function, residuals, jacobians);
    jacobian = jacobians[0];
}


}; // namespace Common_tools