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
#ifndef __LOGGER_HPP__
#define __LOGGER_HPP__
#include "os_compatible.hpp"
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <ostream>
#include <sstream>
#include <stdarg.h> //need for such like printf(...)
#include <stdio.h>
#include <string>
#include <thread>
#include <iomanip>
#include "tools_color_printf.hpp"
#include "tools_timer.hpp"
// #define FILE_LOGGER_VERSION      "V1.0"
// #define FILE_LOGGER_VERSION_INFO "First version"

//#define FILE_LOGGER_VERSION "V1.1"
//#define FILE_LOGGER_VERSION_INFO "Add macro, make logger more easy to call"

//#define FILE_LOGGER_VERSION "V1.2"
//#define FILE_LOGGER_VERSION_INFO "Compatible with windows."

//#define FILE_LOGGER_VERSION "V1.3"
//#define FILE_LOGGER_VERSION_INFO "Support verbose"

// #define FILE_LOGGER_VERSION "V1.4"
// #define FILE_LOGGER_VERSION_INFO "Add macro code block, enable turn off the printf quickly"

// #define FILE_LOGGER_VERSION "V1.5"
// #define FILE_LOGGER_VERSION_INFO "Add variable restore block, enable turn off/on screen prinf in local block scope."

// #define FILE_LOGGER_VERSION "V1.6"
// #define FILE_LOGGER_VERSION_INFO "Fix discontruction bugs."

#define FILE_LOGGER_VERSION "V1.7"
#define FILE_LOGGER_VERSION_INFO "Add more tools, print out the hardware information in printf_program."

#ifndef printf_line
#define printf_line std::cout << __FILE__ << " " << __LINE__ << std::endl;
#endif
#define printf_program( a )                                                                                                                                                                            \
    std::cout << ANSI_COLOR_YELLOW_BOLD;                                                                                                                                                               \
    std::cout << "=============================================================" << std::endl;                                                                                                         \
    std::cout << "App name   : " << ANSI_COLOR_WHITE_BOLD << a << ANSI_COLOR_YELLOW_BOLD << std::endl;                                                                                                 \
    std::cout << "Build date : " << __DATE__ << "  " << __TIME__ << std::endl;                                                                                                                         \
    std::cout << "CPU infos  : " << Common_tools::get_cpu_info() << std::endl;                                                                                                                         \
    std::cout << "RAM infos  : " << Common_tools::get_RAM_info() << std::endl;                                                                                                                         \
    std::cout << "OS  infos  : " << Common_tools::get_OS_info() << std::endl;                                                                                                                          \
    std::cout << "Home dir   : " << Common_tools::get_home_folder() << std::endl;                                                                                                                      \
    std::cout << "Current dir: " << Common_tools::get_current_folder() << std::endl;                                                                                                                   \
    std::cout << "Date mow   : " << Common_tools::get_current_date_time_str() << std::endl;                                                                                                            \
    std::cout << "=============================================================" << ANSI_COLOR_RESET << std::endl;

#define LOG_FILE_LINE( x )                                                                                                                                                                             \
    *( ( x ).get_ostream() ) << __FILE__ << "   " << __LINE__ << endl;                                                                                                                                 \
    ( x ).get_ostream()->flush();

#define LOG_FILE_LINE_AB( a, b )                                                                                                                                                                       \
    *( ( a ).get_ostream( b ) ) << __FILE__ << "   " << __LINE__ << endl;                                                                                                                              \
    ( a ).get_ostream()->flush();

#define LOG_FUNCTION_LINE( x )                                                                                                                                                                         \
    *( ( x ).get_ostream() ) << __FUNCTION__ << "   " << __LINE__ << endl;                                                                                                                             \
    ( x ).get_ostream()->flush();

#define LOG_FUNCTION_LINE_AB( a, b )                                                                                                                                                                   \
    *( ( a ).get_ostream( b ) ) << __FUNCTION__ << "   " << __LINE__ << endl;                                                                                                                          \
    ( x ).get_ostream()->flush();

#define ADD_SCREEN_PRINTF_OUT_METHOD                                                                                                                                                                   \
    int                                  m_if_verbose_screen_printf = 1;                                                                                                                               \
    char *                               m_sceen_output_cstr = new char[ 10000 ]();                                                                                                                    \
    std::string                          m_sceen_output_string = std::string( m_sceen_output_cstr );                                                                                                   \
    std::shared_ptr< std::stringstream > m_sceen_output_stringstream = std::make_shared< std::stringstream >( m_sceen_output_string );                                                                 \
    inline void screen_printf( const char *fmt, ... )                                                                                                                                                  \
    {                                                                                                                                                                                                  \
        if ( m_if_verbose_screen_printf )                                                                                                                                                              \
            return;                                                                                                                                                                                    \
        va_list ap;                                                                                                                                                                                    \
        va_start( ap, fmt );                                                                                                                                                                           \
        if ( Common_tools::vasprintf( &m_sceen_output_cstr, fmt, ap ) == 0 )                                                                                                                           \
        {                                                                                                                                                                                              \
            return;                                                                                                                                                                                    \
        }                                                                                                                                                                                              \
        std::printf( "%s", m_sceen_output_cstr );                                                                                                                                                      \
    }                                                                                                                                                                                                  \
    std::ostream *sreen_outstream()                                                                                                                                                                    \
    {                                                                                                                                                                                                  \
        if ( m_if_verbose_screen_printf )                                                                                                                                                              \
        {                                                                                                                                                                                              \
            return m_sceen_output_stringstream.get();                                                                                                                                                  \
        }                                                                                                                                                                                              \
        else                                                                                                                                                                                           \
        {                                                                                                                                                                                              \
            return &std::cout;                                                                                                                                                                         \
        }                                                                                                                                                                                              \
    }

#define screen_out ( *sreen_outstream() )
#define ENABLE_SCREEN_PRINTF Common_tools::Variable_restore_point< int >  val_restore( &m_if_verbose_screen_printf, 0 ); // Enable printf in this scope.
#define DISABLE_SCREEN_PRINTF Common_tools::Variable_restore_point< int > val_restore( &m_if_verbose_screen_printf, 1 ); // Disable printf in this scope.

namespace Common_tools // Commond tools
{
using namespace std;

inline void printf_software_version()
{

    scope_color( ANSI_COLOR_GREEN_BOLD );
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++" << endl;
    std::cout << "Here is the your software environments: " << std::endl;
// #ifdef __GNUC__ &&__GNUC_MINOR__ &&__GNUC_PATCHLEVEL__
#ifdef __GNUC_PATCHLEVEL__
    std::cout << "GCC version          : " << __GNUC__ << "." << __GNUC_MINOR__ << "." << __GNUC_PATCHLEVEL__ << endl;
#endif
#ifdef BOOST_VERSION // Has Boost
    std::cout << "Boost version        : " << BOOST_VERSION / 100000 << "." << BOOST_VERSION / 100 % 1000 << "." << BOOST_VERSION % 100 << std::endl;
#endif

// #ifdef EIGEN_WORLD_VERSION &&EIGEN_MAJOR_VERSION &&EIGEN_MINOR_VERSION // Has Eigen
#ifdef EIGEN_MINOR_VERSION // Has Eigen
    std::cout << "Eigen version        : " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;
#endif

// #ifdef CV_MAJOR_VERSION &&CV_MINOR_VERSION &&CV_VERSION_REVISION // Has openCV
#ifdef CV_VERSION_REVISION // Has openCV
    std::cout << "OpenCV version       : " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << "." << CV_VERSION_REVISION << std::endl;
#endif

// #ifdef CERES_VERSION_MAJOR &&CERES_VERSION_MINOR &&CERES_VERSION_PATCH // Has Ceres-solver
#ifdef CERES_VERSION_PATCH // Has Ceres-solver
    std::cout << "Ceres-solver version : " << CERES_VERSION_MAJOR << "." << CERES_VERSION_MINOR << "." << CERES_VERSION_PATCH << std::endl;
#endif

// #ifdef CGAL_MAJOR_VERSION &&CGAL_MINOR_VERSION &&CGAL_BUGFIX_VERSION // didn't take effect...., any bugs here?
#ifdef CGAL_BUGFIX_VERSION // didn't take effect...., any bugs here?
    std::cout << "CGAL version         : " << CGAL_MAJOR_VERSION << "." << CGAL_MINOR_VERSION << "." << CGAL_BUGFIX_VERSION << std::endl;
#endif
#ifdef CGAL_VERSION // didn't take effect...., any bugs here?
    std::cout << "CGAL version         : " << CGAL_VERSION << std::endl;
#endif
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++" << endl;
}

template < typename T >
struct Variable_restore_point
{
    T *m_targer_variable;
    T  m_initial_value;

    Variable_restore_point( T *variable_ptr )
    {
        m_targer_variable = variable_ptr;
        m_initial_value = *m_targer_variable;
    }

    Variable_restore_point( T *variable_ptr, const T temp_val )
    {
        m_targer_variable = variable_ptr;
        m_initial_value = *m_targer_variable;
        *m_targer_variable = temp_val;
    }

    ~Variable_restore_point() { *m_targer_variable = m_initial_value; }
};

inline bool if_file_exist( const std::string &name )
{
    // Copy from: https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exist-using-standard-c-c11-c
    struct stat buffer;
    return ( stat( name.c_str(), &buffer ) == 0 );
};

class File_logger
{
  public:
    std::map< string, std::ostream * > m_map_file_os;
    char                                 m_temp_char[ 10000 ];
    string                               m_temp_string;
    std::shared_ptr< std::stringstream > m_temp_stringstream;
    string                               m_save_dir_name = string( "./" );
    std::shared_ptr< std::mutex >        m_mutex_log;
    int                                  m_if_verbose = 0;
    void                                 release()
    {
        for ( auto it = m_map_file_os.begin(); it != m_map_file_os.end(); it++ )
        {
            it->second->flush();
            ( it->second ) = NULL;
            delete it->second;
        }
        m_map_file_os.clear();
    };

    ~File_logger() { release(); };

    void set_log_dir( string _dir_name )
    {
        release();
        m_save_dir_name = _dir_name;
        create_dir( m_save_dir_name );
        // mkdir(m_save_dir_name.c_str(), 0775);
    }

    File_logger( string _dir_name = string( "/home/ziv/data/" ) )
    {
        m_mutex_log = std::make_shared< std::mutex >();
        set_log_dir( _dir_name );
        m_map_file_os.insert( std::pair< string, std::ostream * >( "screen", &std::cout ) );
        m_temp_string.reserve( 1e4 );
        m_temp_stringstream = std::make_shared< std::stringstream >( m_temp_string );
    }

    string version()
    {
        std::stringstream ss;
        ss << "===== This is version of File_logger =====" << endl;
        ss << "Version      : " << FILE_LOGGER_VERSION << endl;
        ss << "Version info : " << FILE_LOGGER_VERSION_INFO << endl;
        ss << "Complie date : " << __DATE__ << "  " << __TIME__ << endl;
        ss << "=====           End                  =====" << endl;
        return string( ss.str() );
    }

    void init( std::string _file_name, std::string prefix_name = string( "log" ), int mode = std::ios::out )
    {
        std::ofstream *ofs = new std::ofstream();
        sprintf( m_temp_char, "%s/%s_%s", m_save_dir_name.c_str(), prefix_name.c_str(), _file_name.c_str() );
        ofs->open( m_temp_char, ios::out );

        if ( ofs->is_open() )
        {
            // cout << "Open " << _file_name << " successful." << endl;
            m_map_file_os.insert( std::pair< string, std::ostream * >( prefix_name, ofs ) );
        }
        else
        {
            cout << "Fail to open " << _file_name << endl;
            m_map_file_os.insert( std::pair< string, std::ostream * >( prefix_name, &std::cout ) );
        }
    };

    std::ostream *get_ostream( std::string prefix_name = string( "log" ) )
    {
        if ( m_if_verbose )
            return m_temp_stringstream.get();
        auto it = m_map_file_os.find( prefix_name );

        if ( it != m_map_file_os.end() )
        {
            return ( it->second );
        }
        else // if no exit, create a new one.
        {
            init( "tempadd.txt", prefix_name );
            return get_ostream( prefix_name );
        }
    }

    int printf( const char *fmt, ... )
    {
        if ( m_if_verbose )
            return 0;
        std::unique_lock< std::mutex > lock( *m_mutex_log );
#ifdef _WIN32
        va_list ap;
        char *  result = 0;
        va_start( ap, fmt );
        if ( vasprintf( &result, fmt, ap ) == 0 )
        {
            return 0;
        }
        // cout << "Fmt = " << fmt <<endl;
        // cout << "Args" = << m_args <<endl;m_temp_char
        m_temp_string = string( result );
        // cout << m_temp_string;
        *( get_ostream() ) << m_temp_string;
        ( get_ostream( "log" ) )->flush();
        return m_temp_string.length();
// return 0;
#else
        va_list ap;
        char *  result = 0;
        va_start( ap, fmt );
        if ( vasprintf( &result, fmt, ap ) == 0 )
        {
            return 0;
        }
        // cout << "Fmt = " << fmt <<endl;
        // cout << "Args" = << m_args <<endl;m_temp_char
        m_temp_string = string( result );
        // cout << m_temp_string;
        *( get_ostream() ) << m_temp_string;
        ( get_ostream( "log" ) )->flush();
        return m_temp_string.length();
#endif
    }
};
}; // namespace Common_tools

#ifdef _MSC_VER
#include <Shlobj.h>
#ifndef _USE_WINSDKOS
#define _USE_WINSDKOS
#include <VersionHelpers.h>
#endif
#else
#include <sys/utsname.h>
#ifdef __APPLE__
#include <sys/sysctl.h>
#else
#include <sys/sysinfo.h>
#endif
#include <pwd.h>
#include <unistd.h>
#endif

#ifndef MAX_PATH
#define MAX_PATH 260
#endif

namespace Common_tools
{
using namespace std;

// Manage setting/removing bit flags
template < typename TYPE >
class TFlags
{
  public:
    typedef TYPE Type;

  public:
    inline TFlags() : flags( 0 ) {}
    inline TFlags( const TFlags &rhs ) : flags( rhs.flags ) {}
    inline TFlags( Type f ) : flags( f ) {}
    inline bool isSet( Type aFlag ) const { return ( flags & aFlag ) == aFlag; }
    inline bool isSet( Type aFlag, Type nF ) const { return ( flags & ( aFlag | nF ) ) == aFlag; }
    inline bool isSetExclusive( Type aFlag ) const { return flags == aFlag; }
    inline bool isAnySet( Type aFlag ) const { return ( flags & aFlag ) != 0; }
    inline bool isAnySet( Type aFlag, Type nF ) const
    {
        const Type m( flags & ( aFlag | nF ) );
        return m != 0 && ( m & nF ) == 0;
    }
    inline bool isAnySetExclusive( Type aFlag ) const { return ( flags & aFlag ) != 0 && ( flags & ~aFlag ) == 0; }
    inline void set( Type aFlag, bool bSet )
    {
        if ( bSet )
            set( aFlag );
        else
            unset( aFlag );
    }
    inline void set( Type aFlag ) { flags |= aFlag; }
    inline void unset( Type aFlag ) { flags &= ~aFlag; }
    inline void flip( Type aFlag ) { flags ^= aFlag; }
    inline void operator=( TFlags rhs ) { flags = rhs.flags; }
    inline operator Type() const { return flags; }
    inline operator Type &() { return flags; }
  protected:
    Type flags;
};
typedef class TFlags< uint32_t > Flags;

typedef struct CPUINFO_TYP
{
    bool bSSE;         // Streaming SIMD Extensions
    bool bSSE2;        // Streaming SIMD Extensions 2
    bool bSSE3;        // Streaming SIMD Extensions 3
    bool bSSE41;       // Streaming SIMD Extensions 4.1
    bool bSSE42;       // Streaming SIMD Extensions 4.2
    bool bAVX;         // Advanced Vector Extensions
    bool bFMA;         // Fused Multiply�Add
    bool b3DNOW;       // 3DNow! (vendor independent)
    bool b3DNOWEX;     // 3DNow! (AMD specific extensions)
    bool bMMX;         // MMX support
    bool bMMXEX;       // MMX (AMD specific extensions)
    bool bEXT;         // extended features available
    char vendor[ 13 ]; // vendor name
    char name[ 49 ];   // CPU name
} CPUINFO;
using String = std::string;
// F U N C T I O N S ///////////////////////////////////////////////

// Flags   InitCPU();
// CPUINFO GetCPUInfo();
// bool    OSSupportsSSE();
// bool    OSSupportsAVX();

#define PATH_SEPARATOR '/'
#define PATH_SEPARATOR_STR "/"
#define REVERSE_PATH_SEPARATOR '\\'

static String &trimUnifySlash( String &path )
{
    String::size_type start = 1;
    while ( ( start = path.find( PATH_SEPARATOR, start ) ) != String::npos )
        if ( path[ start - 1 ] == PATH_SEPARATOR )
            path.erase( start, 1 );
        else
            ++start;
    return path;
}
static String &ensureUnifySlash( String &path )
{
    String::size_type start = 0;
    while ( ( start = path.find( REVERSE_PATH_SEPARATOR, start ) ) != String::npos )
        path[ start ] = PATH_SEPARATOR;
    return trimUnifySlash( path );
}

static String formatBytes( int64_t aBytes )
{
    char buff[ 100 ];
    if ( aBytes < ( int64_t ) 1024 )
    {
        // return String::FormatString( "%dB", ( uint32_t ) aBytes & 0xffffffff );
        sprintf( buff, "%dB", ( uint32_t ) aBytes & 0xffffffff );
    }
    else if ( aBytes < ( int64_t ) 1024 * 1024 )
    {
        // return String::FormatString( "%.02fKB", ( double ) aBytes / ( 1024.0 ) );
        sprintf( buff, "%.02fKB", ( double ) aBytes / ( 1024.0 ) );
    }
    else if ( aBytes < ( int64_t ) 1024 * 1024 * 1024 )
    {
        // return String::FormatString( "%.02fMB", ( double ) aBytes / ( 1024.0 * 1024.0 ) );
        sprintf( buff, "%.02fMB", ( double ) aBytes / ( 1024.0 * 1024.0 ) );
    }
    else if ( aBytes < ( int64_t ) 1024 * 1024 * 1024 * 1024 )
    {
        // return String::FormatString( "%.02fGB", ( double ) aBytes / ( 1024.0 * 1024.0 * 1024.0 ) );
        sprintf( buff, "%.02fGB", ( double ) aBytes / ( 1024.0 * 1024.0 * 1024.0 ) );
    }
    else
    {
        // return String::FormatString( "%.02fTB", ( double ) aBytes / ( 1024.0 * 1024.0 * 1024.0 * 1024.0 ) );
        sprintf( buff, "%.02fTB", ( double ) aBytes / ( 1024.0 * 1024.0 * 1024.0 * 1024.0 ) );
    }
    std::string res_string = buff;
    return res_string;
}

inline String get_home_folder()
{
#ifdef _MSC_VER
    TCHAR homedir[ MAX_PATH ];
    if ( SHGetSpecialFolderPath( 0, homedir, CSIDL_PROFILE, TRUE ) != TRUE )
        return String();
#else
    const char *homedir;
    if ( ( homedir = getenv( "HOME" ) ) == NULL )
        homedir = getpwuid( getuid() )->pw_dir;
#endif // _MSC_VER
    String dir( String( homedir ) + PATH_SEPARATOR );
    return ensureUnifySlash( dir );
}

inline String get_application_folder()
{
#ifdef _MSC_VER
    TCHAR appdir[ MAX_PATH ];
    if ( SHGetSpecialFolderPath( 0, appdir, CSIDL_APPDATA, TRUE ) != TRUE )
        return String();
    String dir( String( appdir ) + PATH_SEPARATOR );
#else
    const char *homedir;
    if ( ( homedir = getenv( "HOME" ) ) == NULL )
        homedir = getpwuid( getuid() )->pw_dir;
    String dir( String( homedir ) + PATH_SEPARATOR + String( ".config" ) + PATH_SEPARATOR );
#endif // _MSC_VER
    return ensureUnifySlash( dir );
}
#ifndef __LOGGER_HPP__
#define __LOGGER_HPP__
#include "os_compatible.hpp"
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <ostream>
#include <sstream>
#include <stdarg.h> //need for such like printf(...)
#include <stdio.h>
#include <string>
#include <thread>
#include <iomanip>
#include "tools_color_printf.hpp"
#include "tools_timer.hpp"
// #define FILE_LOGGER_VERSION      "V1.0"
// #define FILE_LOGGER_VERSION_INFO "First version"

//#define FILE_LOGGER_VERSION "V1.1"
//#define FILE_LOGGER_VERSION_INFO "Add macro, make logger more easy to call"

//#define FILE_LOGGER_VERSION "V1.2"
//#define FILE_LOGGER_VERSION_INFO "Compatible with windows."

//#define FILE_LOGGER_VERSION "V1.3"
//#define FILE_LOGGER_VERSION_INFO "Support verbose"

// #define FILE_LOGGER_VERSION "V1.4"
// #define FILE_LOGGER_VERSION_INFO "Add macro code block, enable turn off the printf quickly"

// #define FILE_LOGGER_VERSION "V1.5"
// #define FILE_LOGGER_VERSION_INFO "Add variable restore block, enable turn off/on screen prinf in local block scope."

// #define FILE_LOGGER_VERSION "V1.6"
// #define FILE_LOGGER_VERSION_INFO "Fix discontruction bugs."

#define FILE_LOGGER_VERSION "V1.7"
#define FILE_LOGGER_VERSION_INFO "Add more tools, print out the hardware information in printf_program."

#ifndef printf_line
#define printf_line std::cout << __FILE__ << " " << __LINE__ << std::endl;
#endif
#define printf_program( a )                                                                                                                                                                            \
    std::cout << ANSI_COLOR_YELLOW_BOLD;                                                                                                                                                               \
    std::cout << "=============================================================" << std::endl;                                                                                                         \
    std::cout << "App name   : " << ANSI_COLOR_WHITE_BOLD << a << ANSI_COLOR_YELLOW_BOLD << std::endl;                                                                                                 \
    std::cout << "Build date : " << __DATE__ << "  " << __TIME__ << std::endl;                                                                                                                         \
    std::cout << "CPU infos  : " << Common_tools::get_cpu_info() << std::endl;                                                                                                                         \
    std::cout << "RAM infos  : " << Common_tools::get_RAM_info() << std::endl;                                                                                                                         \
    std::cout << "OS  infos  : " << Common_tools::get_OS_info() << std::endl;                                                                                                                          \
    std::cout << "Home dir   : " << Common_tools::get_home_folder() << std::endl;                                                                                                                      \
    std::cout << "Current dir: " << Common_tools::get_current_folder() << std::endl;                                                                                                                   \
    std::cout << "Date mow   : " << Common_tools::get_current_date_time_str() << std::endl;                                                                                                            \
    std::cout << "=============================================================" << ANSI_COLOR_RESET << std::endl;

#define LOG_FILE_LINE( x )                                                                                                                                                                             \
    *( ( x ).get_ostream() ) << __FILE__ << "   " << __LINE__ << endl;                                                                                                                                 \
    ( x ).get_ostream()->flush();

#define LOG_FILE_LINE_AB( a, b )                                                                                                                                                                       \
    *( ( a ).get_ostream( b ) ) << __FILE__ << "   " << __LINE__ << endl;                                                                                                                              \
    ( a ).get_ostream()->flush();

#define LOG_FUNCTION_LINE( x )                                                                                                                                                                         \
    *( ( x ).get_ostream() ) << __FUNCTION__ << "   " << __LINE__ << endl;                                                                                                                             \
    ( x ).get_ostream()->flush();

#define LOG_FUNCTION_LINE_AB( a, b )                                                                                                                                                                   \
    *( ( a ).get_ostream( b ) ) << __FUNCTION__ << "   " << __LINE__ << endl;                                                                                                                          \
    ( x ).get_ostream()->flush();

#define ADD_SCREEN_PRINTF_OUT_METHOD                                                                                                                                                                   \
    int                                  m_if_verbose_screen_printf = 1;                                                                                                                               \
    char *                               m_sceen_output_cstr = new char[ 10000 ]();                                                                                                                    \
    std::string                          m_sceen_output_string = std::string( m_sceen_output_cstr );                                                                                                   \
    std::shared_ptr< std::stringstream > m_sceen_output_stringstream = std::make_shared< std::stringstream >( m_sceen_output_string );                                                                 \
    inline void screen_printf( const char *fmt, ... )                                                                                                                                                  \
    {                                                                                                                                                                                                  \
        if ( m_if_verbose_screen_printf )                                                                                                                                                              \
            return;                                                                                                                                                                                    \
        va_list ap;                                                                                                                                                                                    \
        va_start( ap, fmt );                                                                                                                                                                           \
        if ( Common_tools::vasprintf( &m_sceen_output_cstr, fmt, ap ) == 0 )                                                                                                                           \
        {                                                                                                                                                                                              \
            return;                                                                                                                                                                                    \
        }                                                                                                                                                                                              \
        std::printf( "%s", m_sceen_output_cstr );                                                                                                                                                      \
    }                                                                                                                                                                                                  \
    std::ostream *sreen_outstream()                                                                                                                                                                    \
    {                                                                                                                                                                                                  \
        if ( m_if_verbose_screen_printf )                                                                                                                                                              \
        {                                                                                                                                                                                              \
            return m_sceen_output_stringstream.get();                                                                                                                                                  \
        }                                                                                                                                                                                              \
        else                                                                                                                                                                                           \
        {                                                                                                                                                                                              \
            return &std::cout;                                                                                                                                                                         \
        }                                                                                                                                                                                              \
    }

#define screen_out ( *sreen_outstream() )
#define ENABLE_SCREEN_PRINTF Common_tools::Variable_restore_point< int >  val_restore( &m_if_verbose_screen_printf, 0 ); // Enable printf in this scope.
#define DISABLE_SCREEN_PRINTF Common_tools::Variable_restore_point< int > val_restore( &m_if_verbose_screen_printf, 1 ); // Disable printf in this scope.

namespace Common_tools // Commond tools
{
using namespace std;

inline void printf_software_version()
{

    scope_color( ANSI_COLOR_GREEN_BOLD );
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++" << endl;
    std::cout << "Here is the your software environments: " << std::endl;
#ifdef __GNUC__ &&__GNUC_MINOR__ &&__GNUC_PATCHLEVEL__
    std::cout << "GCC version          : " << __GNUC__ << "." << __GNUC_MINOR__ << "." << __GNUC_PATCHLEVEL__ << endl;
#endif
#ifdef BOOST_VERSION // Has Boost
    std::cout << "Boost version        : " << BOOST_VERSION / 100000 << "." << BOOST_VERSION / 100 % 1000 << "." << BOOST_VERSION % 100 << std::endl;
#endif
#ifdef EIGEN_WORLD_VERSION &&EIGEN_MAJOR_VERSION &&EIGEN_MINOR_VERSION // Has Eigen
    std::cout << "Eigen version        : " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;
#endif
#ifdef CV_MAJOR_VERSION &&CV_MINOR_VERSION &&CV_VERSION_REVISION // Has openCV
    std::cout << "OpenCV version       : " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << "." << CV_VERSION_REVISION << std::endl;
#endif
#ifdef CERES_VERSION_MAJOR &&CERES_VERSION_MINOR &&CERES_VERSION_PATCH // Has Ceres-solver
    std::cout << "Ceres-solver version : " << CERES_VERSION_MAJOR << "." << CERES_VERSION_MINOR << "." << CERES_VERSION_PATCH << std::endl;
#endif
#ifdef CGAL_MAJOR_VERSION &&CGAL_MINOR_VERSION &&CGAL_BUGFIX_VERSION // didn't take effect...., any bugs here?
    std::cout << "CGAL version         : " << CGAL_MAJOR_VERSION << "." << CGAL_MINOR_VERSION << "." << CGAL_BUGFIX_VERSION << std::endl;
#endif
#ifdef CGAL_VERSION // didn't take effect...., any bugs here?
    std::cout << "CGAL version         : " << CGAL_VERSION << std::endl;
#endif
    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++" << endl;
}

template < typename T >
struct Variable_restore_point
{
    T *m_targer_variable;
    T  m_initial_value;

    Variable_restore_point( T *variable_ptr )
    {
        m_targer_variable = variable_ptr;
        m_initial_value = *m_targer_variable;
    }

    Variable_restore_point( T *variable_ptr, const T temp_val )
    {
        m_targer_variable = variable_ptr;
        m_initial_value = *m_targer_variable;
        *m_targer_variable = temp_val;
    }

    ~Variable_restore_point() { *m_targer_variable = m_initial_value; }
};

inline bool if_file_exist( const std::string &name )
{
    // Copy from: https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exist-using-standard-c-c11-c
    struct stat buffer;
    return ( stat( name.c_str(), &buffer ) == 0 );
};

class File_logger
{
  public:
    std::map< string, std::ostream * > m_map_file_os;
    char                                 m_temp_char[ 10000 ];
    string                               m_temp_string;
    std::shared_ptr< std::stringstream > m_temp_stringstream;
    string                               m_save_dir_name = string( "./" );
    std::shared_ptr< std::mutex >        m_mutex_log;
    int                                  m_if_verbose = 0;
    void                                 release()
    {
        for ( auto it = m_map_file_os.begin(); it != m_map_file_os.end(); it++ )
        {
            it->second->flush();
            ( it->second ) = NULL;
            delete it->second;
        }
        m_map_file_os.clear();
    };

    ~File_logger() { release(); };

    void set_log_dir( string _dir_name )
    {
        release();
        m_save_dir_name = _dir_name;
        create_dir( m_save_dir_name );
        // mkdir(m_save_dir_name.c_str(), 0775);
    }

    File_logger( string _dir_name = string( "/home/ziv/data/" ) )
    {
        m_mutex_log = std::make_shared< std::mutex >();
        set_log_dir( _dir_name );
        m_map_file_os.insert( std::pair< string, std::ostream * >( "screen", &std::cout ) );
        m_temp_string.reserve( 1e4 );
        m_temp_stringstream = std::make_shared< std::stringstream >( m_temp_string );
    }

    string version()
    {
        std::stringstream ss;
        ss << "===== This is version of File_logger =====" << endl;
        ss << "Version      : " << FILE_LOGGER_VERSION << endl;
        ss << "Version info : " << FILE_LOGGER_VERSION_INFO << endl;
        ss << "Complie date : " << __DATE__ << "  " << __TIME__ << endl;
        ss << "=====           End                  =====" << endl;
        return string( ss.str() );
    }

    void init( std::string _file_name, std::string prefix_name = string( "log" ), int mode = std::ios::out )
    {
        std::ofstream *ofs = new std::ofstream();
        sprintf( m_temp_char, "%s/%s_%s", m_save_dir_name.c_str(), prefix_name.c_str(), _file_name.c_str() );
        ofs->open( m_temp_char, ios::out );

        if ( ofs->is_open() )
        {
            // cout << "Open " << _file_name << " successful." << endl;
            m_map_file_os.insert( std::pair< string, std::ostream * >( prefix_name, ofs ) );
        }
        else
        {
            cout << "Fail to open " << _file_name << endl;
            m_map_file_os.insert( std::pair< string, std::ostream * >( prefix_name, &std::cout ) );
        }
    };

    std::ostream *get_ostream( std::string prefix_name = string( "log" ) )
    {
        if ( m_if_verbose )
            return m_temp_stringstream.get();
        auto it = m_map_file_os.find( prefix_name );

        if ( it != m_map_file_os.end() )
        {
            return ( it->second );
        }
        else // if no exit, create a new one.
        {
            init( "tempadd.txt", prefix_name );
            return get_ostream( prefix_name );
        }
    }

    int printf( const char *fmt, ... )
    {
        if ( m_if_verbose )
            return 0;
        std::unique_lock< std::mutex > lock( *m_mutex_log );
#ifdef _WIN32
        va_list ap;
        char *  result = 0;
        va_start( ap, fmt );
        if ( vasprintf( &result, fmt, ap ) == 0 )
        {
            return 0;
        }
        // cout << "Fmt = " << fmt <<endl;
        // cout << "Args" = << m_args <<endl;m_temp_char
        m_temp_string = string( result );
        // cout << m_temp_string;
        *( get_ostream() ) << m_temp_string;
        ( get_ostream( "log" ) )->flush();
        return m_temp_string.length();
// return 0;
#else
        va_list ap;
        char *  result = 0;
        va_start( ap, fmt );
        if ( vasprintf( &result, fmt, ap ) == 0 )
        {
            return 0;
        }
        // cout << "Fmt = " << fmt <<endl;
        // cout << "Args" = << m_args <<endl;m_temp_char
        m_temp_string = string( result );
        // cout << m_temp_string;
        *( get_ostream() ) << m_temp_string;
        ( get_ostream( "log" ) )->flush();
        return m_temp_string.length();
#endif
    }
};
}; // namespace Common_tools

#ifdef _MSC_VER
#include <Shlobj.h>
#ifndef _USE_WINSDKOS
#define _USE_WINSDKOS
#include <VersionHelpers.h>
#endif
#else
#include <sys/utsname.h>
#ifdef __APPLE__
#include <sys/sysctl.h>
#else
#include <sys/sysinfo.h>
#endif
#include <pwd.h>
#include <unistd.h>
#endif

#ifndef MAX_PATH
#define MAX_PATH 260
#endif

namespace Common_tools
{
using namespace std;

// Manage setting/removing bit flags
template < typename TYPE >
class TFlags
{
  public:
    typedef TYPE Type;

  public:
    inline TFlags() : flags( 0 ) {}
    inline TFlags( const TFlags &rhs ) : flags( rhs.flags ) {}
    inline TFlags( Type f ) : flags( f ) {}
    inline bool isSet( Type aFlag ) const { return ( flags & aFlag ) == aFlag; }
    inline bool isSet( Type aFlag, Type nF ) const { return ( flags & ( aFlag | nF ) ) == aFlag; }
    inline bool isSetExclusive( Type aFlag ) const { return flags == aFlag; }
    inline bool isAnySet( Type aFlag ) const { return ( flags & aFlag ) != 0; }
    inline bool isAnySet( Type aFlag, Type nF ) const
    {
        const Type m( flags & ( aFlag | nF ) );
        return m != 0 && ( m & nF ) == 0;
    }
    inline bool isAnySetExclusive( Type aFlag ) const { return ( flags & aFlag ) != 0 && ( flags & ~aFlag ) == 0; }
    inline void set( Type aFlag, bool bSet )
    {
        if ( bSet )
            set( aFlag );
        else
            unset( aFlag );
    }
    inline void set( Type aFlag ) { flags |= aFlag; }
    inline void unset( Type aFlag ) { flags &= ~aFlag; }
    inline void flip( Type aFlag ) { flags ^= aFlag; }
    inline void operator=( TFlags rhs ) { flags = rhs.flags; }
    inline operator Type() const { return flags; }
    inline operator Type &() { return flags; }
  protected:
    Type flags;
};
typedef class TFlags< uint32_t > Flags;

typedef struct CPUINFO_TYP
{
    bool bSSE;         // Streaming SIMD Extensions
    bool bSSE2;        // Streaming SIMD Extensions 2
    bool bSSE3;        // Streaming SIMD Extensions 3
    bool bSSE41;       // Streaming SIMD Extensions 4.1
    bool bSSE42;       // Streaming SIMD Extensions 4.2
    bool bAVX;         // Advanced Vector Extensions
    bool bFMA;         // Fused Multiply�Add
    bool b3DNOW;       // 3DNow! (vendor independent)
    bool b3DNOWEX;     // 3DNow! (AMD specific extensions)
    bool bMMX;         // MMX support
    bool bMMXEX;       // MMX (AMD specific extensions)
    bool bEXT;         // extended features available
    char vendor[ 13 ]; // vendor name
    char name[ 49 ];   // CPU name
} CPUINFO;
using String = std::string;
// F U N C T I O N S ///////////////////////////////////////////////

// Flags   InitCPU();
// CPUINFO GetCPUInfo();
// bool    OSSupportsSSE();
// bool    OSSupportsAVX();

#define PATH_SEPARATOR '/'
#define PATH_SEPARATOR_STR "/"
#define REVERSE_PATH_SEPARATOR '\\'

static String &trimUnifySlash( String &path )
{
    String::size_type start = 1;
    while ( ( start = path.find( PATH_SEPARATOR, start ) ) != String::npos )
        if ( path[ start - 1 ] == PATH_SEPARATOR )
            path.erase( start, 1 );
        else
            ++start;
    return path;
}
static String &ensureUnifySlash( String &path )
{
    String::size_type start = 0;
    while ( ( start = path.find( REVERSE_PATH_SEPARATOR, start ) ) != String::npos )
        path[ start ] = PATH_SEPARATOR;
    return trimUnifySlash( path );
}

static String formatBytes( int64_t aBytes )
{
    char buff[ 100 ];
    if ( aBytes < ( int64_t ) 1024 )
    {
        // return String::FormatString( "%dB", ( uint32_t ) aBytes & 0xffffffff );
        sprintf( buff, "%dB", ( uint32_t ) aBytes & 0xffffffff );
    }
    else if ( aBytes < ( int64_t ) 1024 * 1024 )
    {
        // return String::FormatString( "%.02fKB", ( double ) aBytes / ( 1024.0 ) );
        sprintf( buff, "%.02fKB", ( double ) aBytes / ( 1024.0 ) );
    }
    else if ( aBytes < ( int64_t ) 1024 * 1024 * 1024 )
    {
        // return String::FormatString( "%.02fMB", ( double ) aBytes / ( 1024.0 * 1024.0 ) );
        sprintf( buff, "%.02fMB", ( double ) aBytes / ( 1024.0 * 1024.0 ) );
    }
    else if ( aBytes < ( int64_t ) 1024 * 1024 * 1024 * 1024 )
    {
        // return String::FormatString( "%.02fGB", ( double ) aBytes / ( 1024.0 * 1024.0 * 1024.0 ) );
        sprintf( buff, "%.02fGB", ( double ) aBytes / ( 1024.0 * 1024.0 * 1024.0 ) );
    }
    else
    {
        // return String::FormatString( "%.02fTB", ( double ) aBytes / ( 1024.0 * 1024.0 * 1024.0 * 1024.0 ) );
        sprintf( buff, "%.02fTB", ( double ) aBytes / ( 1024.0 * 1024.0 * 1024.0 * 1024.0 ) );
    }
    std::string res_string = buff;
    return res_string;
}

inline String get_home_folder()
{
#ifdef _MSC_VER
    TCHAR homedir[ MAX_PATH ];
    if ( SHGetSpecialFolderPath( 0, homedir, CSIDL_PROFILE, TRUE ) != TRUE )
        return String();
#else
    const char *homedir;
    if ( ( homedir = getenv( "HOME" ) ) == NULL )
        homedir = getpwuid( getuid() )->pw_dir;
#endif // _MSC_VER
    String dir( String( homedir ) + PATH_SEPARATOR );
    return ensureUnifySlash( dir );
}

inline String get_application_folder()
{
#ifdef _MSC_VER
    TCHAR appdir[ MAX_PATH ];
    if ( SHGetSpecialFolderPath( 0, appdir, CSIDL_APPDATA, TRUE ) != TRUE )
        return String();
    String dir( String( appdir ) + PATH_SEPARATOR );
#else
    const char *homedir;
    if ( ( homedir = getenv( "HOME" ) ) == NULL )
        homedir = getpwuid( getuid() )->pw_dir;
    String dir( String( homedir ) + PATH_SEPARATOR + String( ".config" ) + PATH_SEPARATOR );
#endif // _MSC_VER
    return ensureUnifySlash( dir );
}

inline String get_current_folder()
{
    char pathname[ MAX_PATH + 1 ];
#ifdef _MSC_VER
    if ( !GetCurrentDirectory( MAX_PATH, pathname ) )
#else  // _MSC_VER
    if ( !getcwd( pathname, MAX_PATH ) )
#endif // _MSC_VER
        return String();
    String dir( String( pathname ) + PATH_SEPARATOR );
    return ensureUnifySlash( dir );
}

#ifdef _MSC_VER
#include <intrin.h>
inline void CPUID( int CPUInfo[ 4 ], int level ) { __cpuid( CPUInfo, level ); }
#else
#include <cpuid.h>
inline void CPUID( int CPUInfo[ 4 ], int level )
{
    unsigned *p( ( unsigned * ) CPUInfo );
    __get_cpuid( ( unsigned & ) level, p + 0, p + 1, p + 2, p + 3 );
}
#endif

inline CPUINFO GetCPUInfo_()
{
    CPUINFO info;

    // set all values to 0 (false)
    memset( &info, 0, sizeof( CPUINFO ) );

    int CPUInfo[ 4 ];

    // CPUID with an InfoType argument of 0 returns the number of
    // valid Ids in CPUInfo[0] and the CPU identification string in
    // the other three array elements. The CPU identification string is
    // not in linear order. The code below arranges the information
    // in a human readable form.
    CPUID( CPUInfo, 0 );
    *( ( int * ) info.vendor ) = CPUInfo[ 1 ];
    *( ( int * ) ( info.vendor + 4 ) ) = CPUInfo[ 3 ];
    *( ( int * ) ( info.vendor + 8 ) ) = CPUInfo[ 2 ];

    // Interpret CPU feature information.
    CPUID( CPUInfo, 1 );
    info.bMMX = ( CPUInfo[ 3 ] & 0x800000 ) != 0;            // test bit 23 for MMX
    info.bSSE = ( CPUInfo[ 3 ] & 0x2000000 ) != 0;           // test bit 25 for SSE
    info.bSSE2 = ( CPUInfo[ 3 ] & 0x4000000 ) != 0;          // test bit 26 for SSE2
    info.bSSE3 = ( CPUInfo[ 2 ] & 0x1 ) != 0;                // test bit 0 for SSE3
    info.bSSE41 = ( CPUInfo[ 2 ] & 0x80000 ) != 0;           // test bit 19 for SSE4.1
    info.bSSE42 = ( CPUInfo[ 2 ] & 0x100000 ) != 0;          // test bit 20 for SSE4.2
    info.bAVX = ( CPUInfo[ 2 ] & 0x18000000 ) == 0x18000000; // test bits 28,27 for AVX
    info.bFMA = ( CPUInfo[ 2 ] & 0x18001000 ) == 0x18001000; // test bits 28,27,12 for FMA

    // EAX=0x80000000 => CPUID returns extended features
    CPUID( CPUInfo, 0x80000000 );
    const unsigned nExIds = CPUInfo[ 0 ];
    info.bEXT = ( nExIds >= 0x80000000 );

    // must be greater than 0x80000004 to support CPU name
    if ( nExIds > 0x80000004 )
    {
        size_t idx( 0 );
        CPUID( CPUInfo, 0x80000002 ); // CPUID returns CPU name part1
        while ( ( ( uint8_t * ) CPUInfo )[ idx ] == ' ' )
            ++idx;
        memcpy( info.name, ( uint8_t * ) CPUInfo + idx, sizeof( CPUInfo ) - idx );
        idx = sizeof( CPUInfo ) - idx;

        CPUID( CPUInfo, 0x80000003 ); // CPUID returns CPU name part2
        memcpy( info.name + idx, CPUInfo, sizeof( CPUInfo ) );
        idx += 16;

        CPUID( CPUInfo, 0x80000004 ); // CPUID returns CPU name part3
        memcpy( info.name + idx, CPUInfo, sizeof( CPUInfo ) );
    }

    if ( ( strncmp( info.vendor, "AuthenticAMD", 12 ) == 0 ) && info.bEXT )
    {                                                       // AMD
        CPUID( CPUInfo, 0x80000001 );                       // CPUID will copy ext. feat. bits to EDX and cpu type to EAX
        info.b3DNOWEX = ( CPUInfo[ 3 ] & 0x40000000 ) != 0; // indicates AMD extended 3DNow+!
        info.bMMXEX = ( CPUInfo[ 3 ] & 0x400000 ) != 0;     // indicates AMD extended MMX
    }

    return info;
}

#if 1
inline String get_cpu_info()
{
    const CPUINFO info( GetCPUInfo_() );
    String        cpu( info.name[ 0 ] == 0 ? info.vendor : info.name );
#if 0
	if (info.bFMA)
		cpu += _T(" FMA");
	else if (info.bAVX)
		cpu += _T(" AVX");
	else if (info.bSSE42)
		cpu += _T(" SSE4.2");
	else if (info.bSSE41)
		cpu += _T(" SSE4.1");
	else if (info.bSSE3)
		cpu += _T(" SSE3");
	else if (info.bSSE2)
		cpu += _T(" SSE2");
	else if (info.bSSE)
		cpu += _T(" SSE");
	if (info.b3DNOWEX)
		cpu += _T(" 3DNOWEX");
	else if (info.b3DNOW)
		cpu += _T(" 3DNOW");
#endif
    return cpu;
}
#endif
/*----------------------------------------------------------------*/

inline String get_RAM_info()
{
#if defined( _MSC_VER )

#ifdef _WIN64
    MEMORYSTATUSEX memoryStatus;
    memset( &memoryStatus, sizeof( MEMORYSTATUSEX ), 0 );
    memoryStatus.dwLength = sizeof( memoryStatus );
    ::GlobalMemoryStatusEx( &memoryStatus );
    const size_t nTotalPhys( ( size_t ) memoryStatus.ullTotalPhys );
    const size_t nTotalVirtual( ( size_t ) memoryStatus.ullTotalVirtual );
#else
    MEMORYSTATUS memoryStatus;
    memset( &memoryStatus, sizeof( MEMORYSTATUS ), 0 );
    memoryStatus.dwLength = sizeof( MEMORYSTATUS );
    ::GlobalMemoryStatus( &memoryStatus );
    const size_t    nTotalPhys( ( size_t ) memoryStatus.dwTotalPhys );
    const size_t    nTotalVirtual( ( size_t ) memoryStatus.dwTotalVirtual );
#endif

#elif defined( __APPLE__ )

    int            mib[ 2 ] = { CTL_HW, HW_MEMSIZE };
    const unsigned namelen = sizeof( mib ) / sizeof( mib[ 0 ] );
    size_t         len = sizeof( size_t );
    size_t         nTotalPhys;
    sysctl( mib, namelen, &nTotalPhys, &len, NULL, 0 );
    const size_t nTotalVirtual( nTotalPhys );

#else // __GNUC__

    struct sysinfo info;
    sysinfo( &info );
    const size_t nTotalPhys( ( size_t ) info.totalram );
    const size_t nTotalVirtual( ( size_t ) info.totalswap );

#endif // _MSC_VER
    return formatBytes( nTotalPhys ) + " Physical Memory " + formatBytes( nTotalVirtual ) + " Virtual Memory";
}
/*----------------------------------------------------------------*/

inline String get_OS_info()
{
#ifdef _MSC_VER

    String os;
#ifdef _USE_WINSDKOS
#ifndef _WIN32_WINNT_WIN10
#define _WIN32_WINNT_WIN10 0x0A00
    if ( IsWindowsVersionOrGreater( HIBYTE( _WIN32_WINNT_WIN10 ), LOBYTE( _WIN32_WINNT_WIN10 ), 0 ) )
#else
    if ( IsWindows10OrGreater() )
#endif
        os = _T("Windows 10+");
    else if ( IsWindows8Point1OrGreater() )
        os = _T("Windows 8.1");
    else if ( IsWindows8OrGreater() )
        os = _T("Windows 8");
    else if ( IsWindows7SP1OrGreater() )
        os = _T("Windows 7 (SP1)");
    else if ( IsWindows7OrGreater() )
        os = _T("Windows 7");
    else if ( IsWindowsVistaSP2OrGreater() )
        os = _T("Windows Vista (SP2)");
    else if ( IsWindowsVistaSP1OrGreater() )
        os = _T("Windows Vista (SP1)");
    else if ( IsWindowsVistaOrGreater() )
        os = _T("Windows Vista");
    else if ( IsWindowsXPSP3OrGreater() )
        os = _T("Windows XP (SP3)");
    else if ( IsWindowsXPSP2OrGreater() )
        os = _T("Windows XP (SP2)");
    else if ( IsWindowsXPSP1OrGreater() )
        os = _T("Windows XP (SP1)");
    else if ( IsWindowsXPOrGreater() )
        os = _T("Windows XP");
    else
        os = _T("Windows (unknown version)");
#else
    OSVERSIONINFOEX ver;
    memset( &ver, 0, sizeof( OSVERSIONINFOEX ) );
    ver.dwOSVersionInfoSize = sizeof( OSVERSIONINFOEX );

    if ( !GetVersionEx( ( OSVERSIONINFO * ) &ver ) )
    {
        ver.dwOSVersionInfoSize = sizeof( OSVERSIONINFO );
        if ( !GetVersionEx( ( OSVERSIONINFO * ) &ver ) )
        {
            return "Windows (unknown version)";
        }
    }

    if ( ver.dwPlatformId != VER_PLATFORM_WIN32_NT )
    {
        os = "Win9x/ME";
    }
    else
    {
        switch ( ver.dwMajorVersion )
        {
        case 4:
            os = "WinNT4";
            break;

        case 5:
            switch ( ver.dwMinorVersion )
            {
            case 0:
                os = "Win2000";
                break;
            case 1:
                os = "WinXP";
                break;
            case 2:
                os = "Win2003";
                break;
            default:
                os = "Unknown WinNT5";
            }
            break;

        case 6:
            switch ( ver.dwMinorVersion )
            {
            case 0:
                os = ( ver.wProductType == VER_NT_WORKSTATION ? "WinVista" : "Win2008" );
                break;
            case 1:
                os = ( ver.wProductType == VER_NT_WORKSTATION ? "Win7" : "Win2008R2" );
                break;
            case 2:
                os = ( ver.wProductType == VER_NT_WORKSTATION ? "Win8" : "Win2012" );
                break;
            case 3:
                os = ( ver.wProductType == VER_NT_WORKSTATION ? "Win8.1" : "Win2012R2" );
                break;
            case 4:
                os = "Win10";
                break;
            default:
                os = "Unknown WinNT6";
            }
            break;

        default:
            os = "Windows (version unknown)";
        }
        if ( ver.wProductType & VER_NT_WORKSTATION )
            os += " Pro";
        else if ( ver.wProductType & VER_NT_SERVER )
            os += " Server";
        else if ( ver.wProductType & VER_NT_DOMAIN_CONTROLLER )
            os += " DC";
    }

    if ( ver.wServicePackMajor != 0 )
    {
        os += " (SP";
        os += String::ToString( ver.wServicePackMajor );
        if ( ver.wServicePackMinor != 0 )
        {
            os += '.';
            os += String::ToString( ver.wServicePackMinor );
        }
        os += ")";
    }
#endif

#ifdef _WIN64
    os += " x64";
#else
    typedef BOOL( WINAPI * LPFN_ISWOW64PROCESS )( HANDLE, PBOOL );
    const LPFN_ISWOW64PROCESS fnIsWow64Process = ( LPFN_ISWOW64PROCESS ) GetProcAddress( GetModuleHandle( "kernel32" ), "IsWow64Process" );
    BOOL                      bIsWow64 = FALSE;
    if ( fnIsWow64Process && fnIsWow64Process( GetCurrentProcess(), &bIsWow64 ) && bIsWow64 )
        os += " x64";
#endif

    return os;

#else // _MSC_VER

    utsname n;
    if ( uname( &n ) != 0 )
        return "linux (unknown version)";
    return String( n.sysname ) + " " + String( n.release ) + " (" + String( n.machine ) + ")";

#endif // _MSC_VER
}
/*----------------------------------------------------------------*/
};

#endif

inline String get_current_folder()
{
    char pathname[ MAX_PATH + 1 ];
#ifdef _MSC_VER
    if ( !GetCurrentDirectory( MAX_PATH, pathname ) )
#else  // _MSC_VER
    if ( !getcwd( pathname, MAX_PATH ) )
#endif // _MSC_VER
        return String();
    String dir( String( pathname ) + PATH_SEPARATOR );
    return ensureUnifySlash( dir );
}

#ifdef _MSC_VER
#include <intrin.h>
inline void CPUID( int CPUInfo[ 4 ], int level ) { __cpuid( CPUInfo, level ); }
#else
#include <cpuid.h>
inline void CPUID( int CPUInfo[ 4 ], int level )
{
    unsigned *p( ( unsigned * ) CPUInfo );
    __get_cpuid( ( unsigned & ) level, p + 0, p + 1, p + 2, p + 3 );
}
#endif

inline CPUINFO GetCPUInfo_()
{
    CPUINFO info;

    // set all values to 0 (false)
    memset( &info, 0, sizeof( CPUINFO ) );

    int CPUInfo[ 4 ];

    // CPUID with an InfoType argument of 0 returns the number of
    // valid Ids in CPUInfo[0] and the CPU identification string in
    // the other three array elements. The CPU identification string is
    // not in linear order. The code below arranges the information
    // in a human readable form.
    CPUID( CPUInfo, 0 );
    *( ( int * ) info.vendor ) = CPUInfo[ 1 ];
    *( ( int * ) ( info.vendor + 4 ) ) = CPUInfo[ 3 ];
    *( ( int * ) ( info.vendor + 8 ) ) = CPUInfo[ 2 ];

    // Interpret CPU feature information.
    CPUID( CPUInfo, 1 );
    info.bMMX = ( CPUInfo[ 3 ] & 0x800000 ) != 0;            // test bit 23 for MMX
    info.bSSE = ( CPUInfo[ 3 ] & 0x2000000 ) != 0;           // test bit 25 for SSE
    info.bSSE2 = ( CPUInfo[ 3 ] & 0x4000000 ) != 0;          // test bit 26 for SSE2
    info.bSSE3 = ( CPUInfo[ 2 ] & 0x1 ) != 0;                // test bit 0 for SSE3
    info.bSSE41 = ( CPUInfo[ 2 ] & 0x80000 ) != 0;           // test bit 19 for SSE4.1
    info.bSSE42 = ( CPUInfo[ 2 ] & 0x100000 ) != 0;          // test bit 20 for SSE4.2
    info.bAVX = ( CPUInfo[ 2 ] & 0x18000000 ) == 0x18000000; // test bits 28,27 for AVX
    info.bFMA = ( CPUInfo[ 2 ] & 0x18001000 ) == 0x18001000; // test bits 28,27,12 for FMA

    // EAX=0x80000000 => CPUID returns extended features
    CPUID( CPUInfo, 0x80000000 );
    const unsigned nExIds = CPUInfo[ 0 ];
    info.bEXT = ( nExIds >= 0x80000000 );

    // must be greater than 0x80000004 to support CPU name
    if ( nExIds > 0x80000004 )
    {
        size_t idx( 0 );
        CPUID( CPUInfo, 0x80000002 ); // CPUID returns CPU name part1
        while ( ( ( uint8_t * ) CPUInfo )[ idx ] == ' ' )
            ++idx;
        memcpy( info.name, ( uint8_t * ) CPUInfo + idx, sizeof( CPUInfo ) - idx );
        idx = sizeof( CPUInfo ) - idx;

        CPUID( CPUInfo, 0x80000003 ); // CPUID returns CPU name part2
        memcpy( info.name + idx, CPUInfo, sizeof( CPUInfo ) );
        idx += 16;

        CPUID( CPUInfo, 0x80000004 ); // CPUID returns CPU name part3
        memcpy( info.name + idx, CPUInfo, sizeof( CPUInfo ) );
    }

    if ( ( strncmp( info.vendor, "AuthenticAMD", 12 ) == 0 ) && info.bEXT )
    {                                                       // AMD
        CPUID( CPUInfo, 0x80000001 );                       // CPUID will copy ext. feat. bits to EDX and cpu type to EAX
        info.b3DNOWEX = ( CPUInfo[ 3 ] & 0x40000000 ) != 0; // indicates AMD extended 3DNow+!
        info.bMMXEX = ( CPUInfo[ 3 ] & 0x400000 ) != 0;     // indicates AMD extended MMX
    }

    return info;
}

#if 1
inline String get_cpu_info()
{
    const CPUINFO info( GetCPUInfo_() );
    String        cpu( info.name[ 0 ] == 0 ? info.vendor : info.name );
#if 0
	if (info.bFMA)
		cpu += _T(" FMA");
	else if (info.bAVX)
		cpu += _T(" AVX");
	else if (info.bSSE42)
		cpu += _T(" SSE4.2");
	else if (info.bSSE41)
		cpu += _T(" SSE4.1");
	else if (info.bSSE3)
		cpu += _T(" SSE3");
	else if (info.bSSE2)
		cpu += _T(" SSE2");
	else if (info.bSSE)
		cpu += _T(" SSE");
	if (info.b3DNOWEX)
		cpu += _T(" 3DNOWEX");
	else if (info.b3DNOW)
		cpu += _T(" 3DNOW");
#endif
    return cpu;
}
#endif
/*----------------------------------------------------------------*/

inline String get_RAM_info()
{
#if defined( _MSC_VER )

#ifdef _WIN64
    MEMORYSTATUSEX memoryStatus;
    memset( &memoryStatus, sizeof( MEMORYSTATUSEX ), 0 );
    memoryStatus.dwLength = sizeof( memoryStatus );
    ::GlobalMemoryStatusEx( &memoryStatus );
    const size_t nTotalPhys( ( size_t ) memoryStatus.ullTotalPhys );
    const size_t nTotalVirtual( ( size_t ) memoryStatus.ullTotalVirtual );
#else
    MEMORYSTATUS memoryStatus;
    memset( &memoryStatus, sizeof( MEMORYSTATUS ), 0 );
    memoryStatus.dwLength = sizeof( MEMORYSTATUS );
    ::GlobalMemoryStatus( &memoryStatus );
    const size_t    nTotalPhys( ( size_t ) memoryStatus.dwTotalPhys );
    const size_t    nTotalVirtual( ( size_t ) memoryStatus.dwTotalVirtual );
#endif

#elif defined( __APPLE__ )

    int            mib[ 2 ] = { CTL_HW, HW_MEMSIZE };
    const unsigned namelen = sizeof( mib ) / sizeof( mib[ 0 ] );
    size_t         len = sizeof( size_t );
    size_t         nTotalPhys;
    sysctl( mib, namelen, &nTotalPhys, &len, NULL, 0 );
    const size_t nTotalVirtual( nTotalPhys );

#else // __GNUC__

    struct sysinfo info;
    sysinfo( &info );
    const size_t nTotalPhys( ( size_t ) info.totalram );
    const size_t nTotalVirtual( ( size_t ) info.totalswap );

#endif // _MSC_VER
    return formatBytes( nTotalPhys ) + " Physical Memory " + formatBytes( nTotalVirtual ) + " Virtual Memory";
}

inline double get_total_phy_RAM_size_in_MB()
{
    struct sysinfo info;
    sysinfo( &info );
    return ( ( double ) ( ( size_t ) info.totalram ) / 1024.0 / 1024.0 );
}

inline double get_total_RAM_size_in_MB()
{
    struct sysinfo info;
    sysinfo( &info );
    return ( ( double ) ( ( size_t ) info.totalram + info.totalswap  ) / 1024.0 / 1024.0 );
}

inline double get_total_phy_RAM_size_in_GB()
{
    return ( ( double ) get_total_phy_RAM_size_in_MB() / 1024.0 );
}

inline double get_total_RAM_size_in_GB()
{
    return ( ( double ) get_total_RAM_size_in_MB() / 1024.0 );
}


inline String get_OS_info()
{
#ifdef _MSC_VER

    String os;
#ifdef _USE_WINSDKOS
#ifndef _WIN32_WINNT_WIN10
#define _WIN32_WINNT_WIN10 0x0A00
    if ( IsWindowsVersionOrGreater( HIBYTE( _WIN32_WINNT_WIN10 ), LOBYTE( _WIN32_WINNT_WIN10 ), 0 ) )
#else
    if ( IsWindows10OrGreater() )
#endif
        os = _T("Windows 10+");
    else if ( IsWindows8Point1OrGreater() )
        os = _T("Windows 8.1");
    else if ( IsWindows8OrGreater() )
        os = _T("Windows 8");
    else if ( IsWindows7SP1OrGreater() )
        os = _T("Windows 7 (SP1)");
    else if ( IsWindows7OrGreater() )
        os = _T("Windows 7");
    else if ( IsWindowsVistaSP2OrGreater() )
        os = _T("Windows Vista (SP2)");
    else if ( IsWindowsVistaSP1OrGreater() )
        os = _T("Windows Vista (SP1)");
    else if ( IsWindowsVistaOrGreater() )
        os = _T("Windows Vista");
    else if ( IsWindowsXPSP3OrGreater() )
        os = _T("Windows XP (SP3)");
    else if ( IsWindowsXPSP2OrGreater() )
        os = _T("Windows XP (SP2)");
    else if ( IsWindowsXPSP1OrGreater() )
        os = _T("Windows XP (SP1)");
    else if ( IsWindowsXPOrGreater() )
        os = _T("Windows XP");
    else
        os = _T("Windows (unknown version)");
#else
    OSVERSIONINFOEX ver;
    memset( &ver, 0, sizeof( OSVERSIONINFOEX ) );
    ver.dwOSVersionInfoSize = sizeof( OSVERSIONINFOEX );

    if ( !GetVersionEx( ( OSVERSIONINFO * ) &ver ) )
    {
        ver.dwOSVersionInfoSize = sizeof( OSVERSIONINFO );
        if ( !GetVersionEx( ( OSVERSIONINFO * ) &ver ) )
        {
            return "Windows (unknown version)";
        }
    }

    if ( ver.dwPlatformId != VER_PLATFORM_WIN32_NT )
    {
        os = "Win9x/ME";
    }
    else
    {
        switch ( ver.dwMajorVersion )
        {
        case 4:
            os = "WinNT4";
            break;

        case 5:
            switch ( ver.dwMinorVersion )
            {
            case 0:
                os = "Win2000";
                break;
            case 1:
                os = "WinXP";
                break;
            case 2:
                os = "Win2003";
                break;
            default:
                os = "Unknown WinNT5";
            }
            break;

        case 6:
            switch ( ver.dwMinorVersion )
            {
            case 0:
                os = ( ver.wProductType == VER_NT_WORKSTATION ? "WinVista" : "Win2008" );
                break;
            case 1:
                os = ( ver.wProductType == VER_NT_WORKSTATION ? "Win7" : "Win2008R2" );
                break;
            case 2:
                os = ( ver.wProductType == VER_NT_WORKSTATION ? "Win8" : "Win2012" );
                break;
            case 3:
                os = ( ver.wProductType == VER_NT_WORKSTATION ? "Win8.1" : "Win2012R2" );
                break;
            case 4:
                os = "Win10";
                break;
            default:
                os = "Unknown WinNT6";
            }
            break;

        default:
            os = "Windows (version unknown)";
        }
        if ( ver.wProductType & VER_NT_WORKSTATION )
            os += " Pro";
        else if ( ver.wProductType & VER_NT_SERVER )
            os += " Server";
        else if ( ver.wProductType & VER_NT_DOMAIN_CONTROLLER )
            os += " DC";
    }

    if ( ver.wServicePackMajor != 0 )
    {
        os += " (SP";
        os += String::ToString( ver.wServicePackMajor );
        if ( ver.wServicePackMinor != 0 )
        {
            os += '.';
            os += String::ToString( ver.wServicePackMinor );
        }
        os += ")";
    }
#endif

#ifdef _WIN64
    os += " x64";
#else
    typedef BOOL( WINAPI * LPFN_ISWOW64PROCESS )( HANDLE, PBOOL );
    const LPFN_ISWOW64PROCESS fnIsWow64Process = ( LPFN_ISWOW64PROCESS ) GetProcAddress( GetModuleHandle( "kernel32" ), "IsWow64Process" );
    BOOL                      bIsWow64 = FALSE;
    if ( fnIsWow64Process && fnIsWow64Process( GetCurrentProcess(), &bIsWow64 ) && bIsWow64 )
        os += " x64";
#endif

    return os;

#else // _MSC_VER

    utsname n;
    if ( uname( &n ) != 0 )
        return "linux (unknown version)";
    return String( n.sysname ) + " " + String( n.release ) + " (" + String( n.machine ) + ")";

#endif // _MSC_VER
}
/*----------------------------------------------------------------*/
};

#endif
