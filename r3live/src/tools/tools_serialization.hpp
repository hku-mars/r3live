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
#ifndef __TOOLS_SERIALIZATION_HPP__
#define __TOOLS_SERIALIZATION_HPP__
// Tools for serialization
// Developer: Jiarong Lin <ziv.lin.ljr@gmail.com>
// Reference:
// [1] https://github.com/artivis/boost_serialization_helper/blob/master/save_load_eigen.h
// [2] https://github.com/artivis/boost_serialization_helper/blob/master/save_load_cvmat.h
// [3] https://github.com/cdcseacave/openMVS/blob/master/libs/Common/Types.inl 

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/unordered_map.hpp>

#include <boost/serialization/vector.hpp>
#include <boost/serialization/array.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <boost/serialization/array.hpp>
#include <boost/serialization/split_free.hpp>

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include "tools_color_printf.hpp"
#ifndef SERIALIZATION_WITH_OPENCV
#define SERIALIZATION_WITH_OPENCV 1
#endif

#if SERIALIZATION_WITH_OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#endif

namespace Common_tools
{
    using std::cout;
    using std::endl;
template < typename T >
inline void dump_obj_to_file( T *obj, std::string file_name, int if_bin = 1 )
{
    // when set if_bin = 0, there might be crash with "terminate called after throwing an instance of 'boost::archive::archive_exception'"
    std::ofstream ofs( file_name );
    if ( ofs.is_open() )
    {
        if ( if_bin )
        {
            boost::archive::binary_oarchive oa( ofs );
            oa << *obj;
            ofs.close();
        }
        else
        {
            boost::archive::text_oarchive oa( ofs );
            oa << *obj;
            ofs.close();
        }
    }
    else
    {
        cout << ANSI_COLOR_RED_BOLD << "Dump obj to file [" << file_name << "] fail!, file can not open" << endl;
    }
}

template < typename T >
inline void load_obj_from_file( T *obj, std::string file_name, int if_bin = 1 )
{
    // when set if_bin = 0, there might be crash with "terminate called after throwing an instance of 'boost::archive::archive_exception'"
    std::ifstream ifs( file_name, std::ios_base::in  );
    if ( ifs.good() )
    {
        if ( if_bin )
        {
            boost::archive::binary_iarchive ia( ifs );
            ia >> *obj;
            ifs.close();
        }
        else
        {
            boost::archive::text_iarchive ia( ifs );
            ia >> *obj;
            ifs.close();
        }
    }
    else
    {
        // cout << ANSI_COLOR_RED_BOLD << "Load obj to file [" << file_name << "] fail!, file can not open" << endl;
    }
}
}

namespace boost
{
namespace serialization
{
template < class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols >
inline void save( Archive &ar, const Eigen::Matrix< _Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols > &M, const unsigned int /* file_version */ )
{
    typename Eigen::Matrix< _Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols >::Index rows = M.rows();
    typename Eigen::Matrix< _Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols >::Index cols = M.cols();

    ar << rows;
    ar << cols;

    ar << make_array( M.data(), M.size() );
}

template < class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols >
inline void load( Archive &ar, Eigen::Matrix< _Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols > &M, const unsigned int /* file_version */ )
{
    typename Eigen::Matrix< _Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols >::Index rows, cols;

    ar >> rows;
    ar >> cols;

    // if (rows=!_Rows) throw std::exception(/*"Unexpected number of rows"*/);
    // if (cols=!_Cols) throw std::exception(/*"Unexpected number of cols"*/);

    ar >> make_array( M.data(), M.size() );
}

template < class Archive, typename _Scalar, int _Cols, int _Options, int _MaxRows, int _MaxCols >
inline void load( Archive &ar, Eigen::Matrix< _Scalar, Eigen::Dynamic, _Cols, _Options, _MaxRows, _MaxCols > &M, const unsigned int /* file_version */ )
{
    typename Eigen::Matrix< _Scalar, Eigen::Dynamic, _Cols, _Options, _MaxRows, _MaxCols >::Index rows, cols;

    ar >> rows;
    ar >> cols;

    // if (cols=!_Cols) throw std::exception(/*"Unexpected number of cols"*/);

    M.resize( rows, Eigen::NoChange );

    ar >> make_array( M.data(), M.size() );
}

template < class Archive, typename _Scalar, int _Rows, int _Options, int _MaxRows, int _MaxCols >
inline void load( Archive &ar, Eigen::Matrix< _Scalar, _Rows, Eigen::Dynamic, _Options, _MaxRows, _MaxCols > &M, const unsigned int /* file_version */ )
{
    typename Eigen::Matrix< _Scalar, _Rows, Eigen::Dynamic, _Options, _MaxRows, _MaxCols >::Index rows, cols;

    ar >> rows;
    ar >> cols;

    // if (rows=!_Rows) throw std::exception(/*"Unexpected number of rows"*/);

    M.resize( Eigen::NoChange, cols );

    ar >> make_array( M.data(), M.size() );
}

template < class Archive, typename _Scalar, int _Options, int _MaxRows, int _MaxCols >
inline void load( Archive &ar, Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic, _Options, _MaxRows, _MaxCols > &M, const unsigned int /* file_version */ )
{
    typename Eigen::Matrix< _Scalar, Eigen::Dynamic, Eigen::Dynamic, _Options, _MaxRows, _MaxCols >::Index rows, cols;

    ar >> rows;
    ar >> cols;

    M.resize( rows, cols );

    ar >> make_array( M.data(), M.size() );
}

template < class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols >
inline void serialize( Archive &ar, Eigen::Matrix< _Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols > &M, const unsigned int file_version )
{
    split_free( ar, M, file_version );
}

template < class Archive, typename _Scalar, int _Dim, int _Mode, int _Options >
inline void serialize( Archive &ar, Eigen::Transform< _Scalar, _Dim, _Mode, _Options > &t, const unsigned int version )
{
    serialize( ar, t.matrix(), version );
}

template < class Archive, typename _Scalar >
void save( Archive &ar, const Eigen::Triplet< _Scalar > &m, const unsigned int /*version*/ )
{
    ar << m.row();
    ar << m.col();
    ar << m.value();
}

template < class Archive, typename _Scalar >
void load( Archive &ar, Eigen::Triplet< _Scalar > &m, const unsigned int /*version*/ )
{
    typename Eigen::Triplet< _Scalar >::Index row, col;
    _Scalar                                   value;

    ar >> row;
    ar >> col;
    ar >> value;

    m = Eigen::Triplet< _Scalar >( row, col, value );
}

template < class Archive, class _Scalar >
void serialize( Archive &ar, Eigen::Triplet< _Scalar > &m, const unsigned int version )
{
    split_free( ar, m, version );
}

template < class Archive, typename _Scalar, int _Options, typename _Index >
void save( Archive &ar, const Eigen::SparseMatrix< _Scalar, _Options, _Index > &m, const unsigned int /*version*/ )
{
    _Index innerSize = m.innerSize();
    _Index outerSize = m.outerSize();

    typedef typename Eigen::Triplet< _Scalar > Triplet;
    std::vector< Triplet >                     triplets;

    for ( _Index i = 0; i < outerSize; ++i )
        for ( typename Eigen::SparseMatrix< _Scalar, _Options, _Index >::InnerIterator it( m, i ); it; ++it )
            triplets.push_back( Triplet( it.row(), it.col(), it.value() ) );

    ar << innerSize;
    ar << outerSize;
    ar << triplets;
}

template < class Archive, typename _Scalar, int _Options, typename _Index >
void load( Archive &ar, Eigen::SparseMatrix< _Scalar, _Options, _Index > &m, const unsigned int /*version*/ )
{
    _Index innerSize;
    _Index outerSize;

    ar >> innerSize;
    ar >> outerSize;

    _Index rows = ( m.IsRowMajor ) ? outerSize : innerSize;
    _Index cols = ( m.IsRowMajor ) ? innerSize : outerSize;

    m.resize( rows, cols );

    typedef typename Eigen::Triplet< _Scalar > Triplet;
    std::vector< Triplet >                     triplets;

    ar >> triplets;

    m.setFromTriplets( triplets.begin(), triplets.end() );
}

template < class Archive, typename _Scalar, int _Options, typename _Index >
void serialize( Archive &ar, Eigen::SparseMatrix< _Scalar, _Options, _Index > &m, const unsigned int version )
{
    split_free( ar, m, version );
}

template < class Archive, typename _Scalar >
void serialize( Archive &ar, Eigen::Quaternion< _Scalar > &q, const unsigned int /*version*/ )
{
    ar &q.w();
    ar &q.x();
    ar &q.y();
    ar &q.z();
}

template < class Archive, typename _Scalar >
void serialize( Archive &ar, Eigen::Quaternion< _Scalar, Eigen::DontAlign > &q, const unsigned int /*version*/ )
{
    ar &q.w();
    ar &q.x();
    ar &q.y();
    ar &q.z();
}

#if SERIALIZATION_WITH_OPENCV

// Serialization support for cv::Mat
template < class Archive >
void save( Archive &ar, const cv::Mat &m, const unsigned int /*version*/ )
{
    const int    elem_type = m.type();
    const size_t elem_size = m.elemSize();

    ar &m.cols;
    ar &m.rows;
    ar &elem_type;
    ar &elem_size;

    const size_t data_size = elem_size * m.cols * m.rows;
    if ( m.isContinuous() )
    {
        ar &boost::serialization::make_array( m.ptr(), data_size );
    }
    else
    {
        cv::Mat m_cont;
        m.copyTo( m_cont );
        ar &boost::serialization::make_array( m_cont.ptr(), data_size );
    }
}
template < class Archive >
void load( Archive &ar, cv::Mat &m, const unsigned int /*version*/ )
{
    int    cols, rows, elem_type;
    size_t elem_size;

    ar &cols;
    ar &rows;
    ar &elem_type;
    ar &elem_size;

    m.create( rows, cols, elem_type );

    const size_t data_size = elem_size * m.cols * m.rows;
    ar &         boost::serialization::make_array( m.ptr(), data_size );
}
template < class Archive >
inline void serialize( Archive &ar, cv::Mat &m, const unsigned int version )
{
    split_free( ar, m, version );
}

// Serialization support for cv::Mat_
template < class Archive, typename _Tp >
void save( Archive &ar, const cv::Mat_< _Tp > &m, const unsigned int /*version*/ )
{
    ar &m.cols;
    ar &m.rows;

    const size_t data_size = m.cols * m.rows;
    if ( m.isContinuous() )
    {
        ar &boost::serialization::make_array( ( const _Tp * ) m.ptr(), data_size );
    }
    else
    {
        cv::Mat_< _Tp > m_cont;
        m.copyTo( m_cont );
        ar &boost::serialization::make_array( ( const _Tp * ) m_cont.ptr(), data_size );
    }
}
template < class Archive, typename _Tp >
void load( Archive &ar, cv::Mat_< _Tp > &m, const unsigned int /*version*/ )
{
    int cols, rows;
    ar &cols;
    ar &rows;

    m.create( rows, cols );

    const size_t data_size = m.cols * m.rows;
    for ( size_t n = 0; n < data_size; ++n )
        new ( ( _Tp * ) m.data + n ) _Tp;
    ar &boost::serialization::make_array( ( _Tp * ) m.ptr(), data_size );
}
template < class Archive, typename _Tp >
inline void serialize( Archive &ar, cv::Mat_< _Tp > &m, const unsigned int version )
{
    split_free( ar, m, version );
}

// Serialization support for cv::Matx
template < class Archive, typename _Tp, int m, int n >
void serialize( Archive &ar, cv::Matx< _Tp, m, n > &_m, const unsigned int /*version*/ )
{
    ar &_m.val;
}

// Serialization support for cv::Vec
template < class Archive, typename _Tp, int cn >
void serialize( Archive &ar, cv::Vec< _Tp, cn > &v, const unsigned int /*version*/ )
{
    ar &boost::serialization::base_object< cv::Matx< _Tp, cn, 1 > >( v );
}

// Serialization support for cv::Point_
template < class Archive, typename _Tp >
void serialize( Archive &ar, cv::Point_< _Tp > &pt, const unsigned int /*version*/ )
{
    ar &pt.x &pt.y;
}

// Serialization support for cv::Point3_
template < class Archive, typename _Tp >
void serialize( Archive &ar, cv::Point3_< _Tp > &pt, const unsigned int /*version*/ )
{
    ar &pt.x &pt.y &pt.z;
}

// Serialization support for cv::Size_
template < class Archive, typename _Tp >
void serialize( Archive &ar, cv::Size_< _Tp > &sz, const unsigned int /*version*/ )
{
    ar &sz.width &sz.height;
}

// Serialization support for cv::Rect_
template < class Archive, typename _Tp >
void serialize( Archive &ar, cv::Rect_< _Tp > &rc, const unsigned int /*version*/ )
{
    ar &rc.x &rc.y &rc.width &rc.height;
}

// Serialization support for cv::KeyPoint
template < class Archive >
void serialize( Archive &ar, cv::KeyPoint &k, const unsigned int /*version*/ )
{
    ar &k.pt;
    ar &k.size;
    ar &k.angle;
    ar &k.response;
    ar &k.octave;
    ar &k.class_id;
}

// Serialization support for cv::DMatch
template < class Archive >
void serialize( Archive &ar, cv::DMatch &m, const unsigned int /*version*/ )
{
    ar &m.queryIdx;
    ar &m.trainIdx;
    ar &m.imgIdx;
    ar &m.distance;
}
#endif
} // namespace serialization
} // namespace boost

#endif
