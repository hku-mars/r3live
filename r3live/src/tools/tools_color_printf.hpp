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
#ifndef __TOOLS_COLOR_PRINT_HPP__
#define __TOOLS_COLOR_PRINT_HPP__
#include <stdio.h>
#include <string.h>
/*
 * Make screen print colorful :)
 * Author: Jiarong Lin
 * Related link: 
 * [1] https://en.wikipedia.org/wiki/ANSI_escape_code 
 * [2] https://github.com/caffedrine/CPP_Utils/blob/597fe5200be87fa1db2d2aa5d8a07c3bc32a66cd/Utils/include/AnsiColors.h
 * 
*/

// const std::string _tools_color_printf_version = "V1.0";
// const std::string _tools_color_printf_info = "[Init]: Add macros, add scope_color()";
const std::string _tools_color_printf_version = "V1.2";
const std::string _tools_color_printf_info = "[Enh]: Add delete lines, ANSI_SCREEN_FLUSH";
using std::cout;
using std::endl;
// clang-format off
#ifdef EMPTY_ANSI_COLORS
    #define ANSI_COLOR_RED ""
    #define ANSI_COLOR_RED_BOLD ""
    #define ANSI_COLOR_GREEN ""
    #define ANSI_COLOR_GREEN_BOLD ""
    #define ANSI_COLOR_YELLOW ""
    #define ANSI_COLOR_YELLOW_BOLD ""
    #define ANSI_COLOR_BLUE ""
    #define ANSI_COLOR_BLUE_BOLD ""
    #define ANSI_COLOR_MAGENTA ""
    #define ANSI_COLOR_MAGENTA_BOLD ""
#else
    #define ANSI_COLOR_RED "\x1b[0;31m"
    #define ANSI_COLOR_RED_BOLD "\x1b[1;31m"
    #define ANSI_COLOR_RED_BG "\x1b[0;41m"

    #define ANSI_COLOR_GREEN "\x1b[0;32m"
    #define ANSI_COLOR_GREEN_BOLD "\x1b[1;32m"
    #define ANSI_COLOR_GREEN_BG "\x1b[0;42m"

    #define ANSI_COLOR_YELLOW "\x1b[0;33m"
    #define ANSI_COLOR_YELLOW_BOLD "\x1b[1;33m"
    #define ANSI_COLOR_YELLOW_BG "\x1b[0;43m"

    #define ANSI_COLOR_BLUE "\x1b[0;34m"
    #define ANSI_COLOR_BLUE_BOLD "\x1b[1;34m"
    #define ANSI_COLOR_BLUE_BG "\x1b[0;44m"

    #define ANSI_COLOR_MAGENTA "\x1b[0;35m"
    #define ANSI_COLOR_MAGENTA_BOLD "\x1b[1;35m"
    #define ANSI_COLOR_MAGENTA_BG "\x1b[0;45m"

    #define ANSI_COLOR_CYAN "\x1b[0;36m"
    #define ANSI_COLOR_CYAN_BOLD "\x1b[1;36m"
    #define ANSI_COLOR_CYAN_BG "\x1b[0;46m"

    #define ANSI_COLOR_WHITE "\x1b[0;37m"
    #define ANSI_COLOR_WHITE_BOLD "\x1b[1;37m"
    #define ANSI_COLOR_WHITE_BG "\x1b[0;47m"

    #define ANSI_COLOR_BLACK "\x1b[0;30m"
    #define ANSI_COLOR_BLACK_BOLD "\x1b[1;30m"
    #define ANSI_COLOR_BLACK_BG "\x1b[0;40m"

    #define ANSI_COLOR_RESET "\x1b[0m"

    #define ANSI_DELETE_LAST_LINE "\033[A\33[2K\r"
    #define ANSI_DELETE_CURRENT_LINE "\33[2K\r"
    #define ANSI_SCREEN_FLUSH std::fflush(stdout);

    #define SET_PRINT_COLOR( a ) cout << a ;

#endif
// clang-format on

struct _Scope_color
{
    _Scope_color( const char * color )
    {
        cout << color;
    }

    ~_Scope_color()
    {
        cout << ANSI_COLOR_RESET;
    }
};

#define scope_color(a) _Scope_color _scope(a);

// inline int demo_test_color_printf()
// {
//     int i, j, n;

//     for ( i = 0; i < 11; i++ )
//     {
//         for ( j = 0; j < 10; j++ )
//         {
//             n = 10 * i + j;
//             if ( n > 108 )
//                 break;
//             printf( "\033[%dm %3d\033[m", n, n );
//         }
//         printf( "\n" );
//     }
//     return ( 0 );
// };
#endif