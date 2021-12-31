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
#include "unordered_map"
#include "iostream"

template <typename data_type = float, typename T = void *>
struct Hash_map_3d
{
    using hash_3d_T = std::unordered_map<data_type, std::unordered_map<data_type, std::unordered_map<data_type, T>>>;
    hash_3d_T m_map_3d_hash_map;
    void insert(const data_type &x, const data_type &y, const data_type &z, const T &target)
    {
        m_map_3d_hash_map[x][y][z] = target;
    }
    
    int if_exist(const data_type &x, const data_type &y, const data_type &z)
    {
        if(m_map_3d_hash_map.find(x) == m_map_3d_hash_map.end()  )
        {
            return 0;
        }
        else if(m_map_3d_hash_map[x].find(y) ==  m_map_3d_hash_map[x].end() )
        {
            return 0;
        }
        else if( m_map_3d_hash_map[x][y].find(z) == m_map_3d_hash_map[x][y].end() )
        {
            return 0;
        }
        return 1;
    }

    void clear()
    {
        m_map_3d_hash_map.clear();
    }

    int total_size()
    {
        int count =0 ;
        for(auto it : m_map_3d_hash_map)
        {
            for(auto it_it: it.second)
            {
                for( auto it_it_it: it_it.second )
                {
                    count++;
                }
            }
        }
        return count;
    }
};


template <typename data_type = float, typename T = void *>
struct Hash_map_2d
{
    using hash_2d_T = std::unordered_map<data_type, std::unordered_map<data_type, T> >;
    // using hash_2d_it = typename std::unordered_map<data_type, std::unordered_map<data_type, T> >::iterator ;
    // using hash_2d_it_it = typename std::unordered_map<data_type, T>::iterator ;

    hash_2d_T m_map_2d_hash_map;
    void insert(const data_type &x, const data_type &y,  const T &target)
    {
        m_map_2d_hash_map[x][y] = target;
    }
    
    int if_exist(const data_type &x, const data_type &y )
    {
        if(m_map_2d_hash_map.find(x) == m_map_2d_hash_map.end()  )
        {
            return 0;
        }
        else if(m_map_2d_hash_map[x].find(y) ==  m_map_2d_hash_map[x].end() )
        {
            return 0;
        }
       
        return 1;
    }

    void clear()
    {
        m_map_2d_hash_map.clear();
    }

    int total_size()
    {
        int count =0 ;
        //for(hash_2d_it it =  m_map_2d_hash_map.begin(); it != m_map_2d_hash_map.end(); it++)
        for(auto it : m_map_2d_hash_map)
        {
            for(auto it_it: it.second)
            {
                count++;
            }
        }
        return count;
    }
};
