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
#ifndef __TOOLS_RANDOM_HPP__
#define __TOOLS_RANDOM_HPP__
// A tools that can generate ture random variable, with modern c++ approach
// Author: Jiarong Lin
// In the develop of this tools, I refer to :
// [1]. https://www.fluentcpp.com/2019/05/24/how-to-fill-a-cpp-collection-with-random-values/
// [2]. https://www.zhihu.com/question/3999110/answer/84257121

#include <algorithm>
#include <iostream>
#include <random>
#include <stdio.h>
#include <stdlib.h>

namespace Common_tools
{
template < typename T >
struct Random_generator_float
{
    std::random_device                  random_device;
    std::mt19937                        m_random_engine;
    std::uniform_real_distribution< T > m_dist;
    std::normal_distribution< T >       m_dist_normal;
    Random_generator_float( bool if_true_random = true, int seed = 0 )
    {
        if ( if_true_random )
        {
            m_random_engine = std::mt19937( std::random_device{}() );
        }
        else
        {
            m_random_engine = std::mt19937( seed );
        }
    }
    ~Random_generator_float(){};

    T rand_uniform( T low = 0.0, T hight = 1.0 )
    {
        m_dist = std::uniform_real_distribution< T >( low, hight );
        return m_dist( m_random_engine );
    }

    T rand_normal( T mean = 0.0, T std = 100.0 )
    {
        m_dist_normal = std::normal_distribution< T >( mean, std );
        return m_dist_normal( m_random_engine );
    }

    T *rand_array_uniform( T low = 0.0, T hight = 1.0, size_t numbers = 100, T *res = nullptr )
    {
        if ( res == nullptr )
        {
            res = new T[ numbers ];
        }
        m_dist = std::uniform_real_distribution< T >( low, hight );
        for ( size_t i = 0; i < numbers; i++ )
        {
            res[ i ] = m_dist( m_random_engine );
        }
        return res;
    }

    T *rand_array_normal( T mean = 0.0, T std = 1.0, size_t numbers = 100, T *res = nullptr )
    {
        if ( res == nullptr )
        {
            res = new T[ numbers ];
        }
        m_dist_normal = std::normal_distribution< T >( mean, std );
        for ( size_t i = 0; i < numbers; i++ )
        {
            res[ i ] = m_dist_normal( m_random_engine );
        }
        return res;
    }
};

template < typename T >
struct Random_generator_int
{
    std::random_device                 random_device;
    std::mt19937                       m_random_engine;
    std::uniform_int_distribution< T > m_dist;
    Random_generator_int( bool if_true_random = true, int seed = 0 )
    {
        if ( if_true_random )
        {
            m_random_engine = std::mt19937( std::random_device{}() );
        }
        else
        {
            m_random_engine = std::mt19937( seed );
        }
    }

    ~Random_generator_int(){};

    T rand_uniform( T low = 0, T hight = 100 )
    {
        m_dist = std::uniform_int_distribution< T >( low, hight );
        return m_dist( m_random_engine );
    }

    T *rand_array_uniform( T low = 0.0, T hight = 1.0, size_t numbers = 100, T *res = nullptr )
    {
        if ( res == nullptr )
        {
            res = new T[ numbers ];
        }
        m_dist = std::uniform_int_distribution< T >( low, hight );
        for ( size_t i = 0; i < numbers; i++ )
        {
            res[ i ] = m_dist( m_random_engine );
        }
        return res;
    }

    T *rand_array_norepeat( T low, T high, T k )
    {
        T                  n = high - low;
        T *                res_array = new T[ k ];
        std::vector< int > foo;
        foo.resize( n );
        for ( T i = 1; i <= n; ++i )
            foo[ i ] = i + low;
        std::shuffle( foo.begin(), foo.end(), m_random_engine );
        for ( T i = 0; i < k; ++i )
        {
            res_array[ i ] = foo[ i ];
            // std::cout << foo[ i ] << " ";
        }
        return res_array;
    }
};

} // namespace Common_tools

#endif