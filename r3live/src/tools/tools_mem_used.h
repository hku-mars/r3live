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

/*
 * Related url: https://stackoverflow.com/questions/669438/how-to-get-memory-usage-at-runtime-using-c
 * Developer: Jiarong Lin
 * Site:    http://NadeauSoftware.com/
 * License: Creative Commons Attribution 3.0 Unported License
 *          http://creativecommons.org/licenses/by/3.0/deed.en_US
 */
#ifndef __TOOLS_MEM_USED_H__
#define __TOOLS_MEM_USED_H__
// const std::string _tools_mem_used_version = "V1.0";
// const std::string _tools_mem_used_version_infod = "[Init]: forker raw version from
// https://stackoverflow.com/questions/669438/how-to-get-memory-usage-at-runtime-using-c";

// const std::string _tools_mem_used_version = "V1.1";
// const std::string _tools_mem_used_version_infod = "[Enh]: Make functions inline";

const std::string _tools_mem_used_version = "V1.2";
const std::string _tools_mem_used_version_infod = "[Enh]: Add more interface.";

#if defined( _WIN32 )
#include <windows.h>
#include <psapi.h>

#elif defined( __unix__ ) || defined( __unix ) || defined( unix ) || ( defined( __APPLE__ ) && defined( __MACH__ ) )
#include <unistd.h>
#include <sys/resource.h>

#if defined( __APPLE__ ) && defined( __MACH__ )
#include <mach/mach.h>

#elif ( defined( _AIX ) || defined( __TOS__AIX__ ) ) ||                                                                                              \
    ( defined( __sun__ ) || defined( __sun ) || defined( sun ) && ( defined( __SVR4 ) || defined( __svr4__ ) ) )
#include <fcntl.h>
#include <procfs.h>

#elif defined( __linux__ ) || defined( __linux ) || defined( linux ) || defined( __gnu_linux__ )
#include <stdio.h>

#endif

#else
#error "Cannot define getPeakRSS( ) or getCurrentRSS( ) for an unknown OS."
#endif

/**
 * Returns the peak (maximum so far) resident set size (physical
 * memory use) measured in bytes, or zero if the value cannot be
 * determined on this OS.
 */
namespace Common_tools
{
inline size_t getPeakRSS()
{
#if defined( _WIN32 )
    /* Windows -------------------------------------------------- */
    PROCESS_MEMORY_COUNTERS info;
    GetProcessMemoryInfo( GetCurrentProcess(), &info, sizeof( info ) );
    return ( size_t ) info.PeakWorkingSetSize;

#elif ( defined( _AIX ) || defined( __TOS__AIX__ ) ) ||                                                                                              \
    ( defined( __sun__ ) || defined( __sun ) || defined( sun ) && ( defined( __SVR4 ) || defined( __svr4__ ) ) )
    /* AIX and Solaris ------------------------------------------ */
    struct psinfo psinfo;
    int           fd = -1;
    if ( ( fd = open( "/proc/self/psinfo", O_RDONLY ) ) == -1 )
        return ( size_t ) 0L; /* Can't open? */
    if ( read( fd, &psinfo, sizeof( psinfo ) ) != sizeof( psinfo ) )
    {
        close( fd );
        return ( size_t ) 0L; /* Can't read? */
    }
    close( fd );
    return ( size_t )( psinfo.pr_rssize * 1024L );

#elif defined( __unix__ ) || defined( __unix ) || defined( unix ) || ( defined( __APPLE__ ) && defined( __MACH__ ) )
    /* BSD, Linux, and OSX -------------------------------------- */
    struct rusage rusage;
    getrusage( RUSAGE_SELF, &rusage );
#if defined( __APPLE__ ) && defined( __MACH__ )
    return ( size_t ) rusage.ru_maxrss;
#else
    return ( size_t )( rusage.ru_maxrss * 1024L );
#endif

#else
    /* Unknown OS ----------------------------------------------- */
    return ( size_t ) 0L; /* Unsupported. */
#endif
}

/**
 * Returns the current resident set size (physical memory use) measured
 * in bytes, or zero if the value cannot be determined on this OS.
 */
inline size_t getCurrentRSS()
{
#if defined( _WIN32 )
    /* Windows -------------------------------------------------- */
    PROCESS_MEMORY_COUNTERS info;
    GetProcessMemoryInfo( GetCurrentProcess(), &info, sizeof( info ) );
    return ( size_t ) info.WorkingSetSize;

#elif defined( __APPLE__ ) && defined( __MACH__ )
    /* OSX ------------------------------------------------------ */
    struct mach_task_basic_info info;
    mach_msg_type_number_t      infoCount = MACH_TASK_BASIC_INFO_COUNT;
    if ( task_info( mach_task_self(), MACH_TASK_BASIC_INFO, ( task_info_t ) &info, &infoCount ) != KERN_SUCCESS )
        return ( size_t ) 0L; /* Can't access? */
    return ( size_t ) info.resident_size;

#elif defined( __linux__ ) || defined( __linux ) || defined( linux ) || defined( __gnu_linux__ )
    /* Linux ---------------------------------------------------- */
    long  rss = 0L;
    FILE *fp = NULL;
    if ( ( fp = fopen( "/proc/self/statm", "r" ) ) == NULL )
        return ( size_t ) 0L; /* Can't open? */
    if ( fscanf( fp, "%*s%ld", &rss ) != 1 )
    {
        fclose( fp );
        return ( size_t ) 0L; /* Can't read? */
    }
    fclose( fp );
    return ( size_t ) rss * ( size_t ) sysconf( _SC_PAGESIZE );

#else
    /* AIX, BSD, Solaris, and Unknown OS ------------------------ */
    return ( size_t ) 0L; /* Unsupported. */
#endif
}

inline double get_RSS_Gb() { return ( getCurrentRSS() / 1024.0 / 1024.0 / 1024.0 ); }

inline double get_RSS_Mb() { return ( getCurrentRSS() / 1024.0 / 1024.0 ); }

inline double get_RSS_Kb() { return ( getCurrentRSS() / 1024.0 ); }
}; // namespace Common_tools

#ifndef printf_line_mem_Gb
#define printf_line_mem_Gb                                                                                                                           \
    std::cout << __FILE__ << " " << __LINE__ << " (" << std::setprecision( 6 ) << Common_tools::get_RSS_Gb() << " Gb)" << std::endl;
#endif

#ifndef printf_line_mem_MB
#define printf_line_mem_MB                                                                                                                           \
    std::cout << __FILE__ << " " << __LINE__ << " (" << std::setprecision( 6 ) << Common_tools::get_RSS_Mb() << " Mb)" << std::endl;
#endif

#ifndef printf_line_mem_KB
#define printf_line_mem_KB                                                                                                                           \
    std::cout << __FILE__ << " " << __LINE__ << " (" << std::setprecision( 6 ) << Common_tools::get_RSS_Kb() << " Kb)" << std::endl;
#endif

#ifndef printf_line_mem
#define printf_line_mem printf_line_mem_MB
#endif
#endif