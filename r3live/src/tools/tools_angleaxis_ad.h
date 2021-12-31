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
#ifndef EIGEN_ANGLEAXIS_H_MINE
#define EIGEN_ANGLEAXIS_H_MINE
#ifndef EIGEN_ANGLEAXIS_H
#define EIGEN_ANGLEAXIS_H
#endif
#include <Eigen/Eigen>

namespace Eigen { 

/** \geometry_module \ingroup Geometry_Module
  *
  * \class AngleAxis
  *
  * \brief Represents a 3D rotation as a rotation angle around an arbitrary 3D axis
  *
  * \param _Scalar the scalar type, i.e., the type of the coefficients.
  *
  * \warning When setting up an AngleAxis object, the axis vector \b must \b be \b normalized.
  *
  * The following two typedefs are provided for convenience:
  * \li \c AngleAxisf for \c float
  * \li \c AngleAxisd for \c double
  *
  * Combined with MatrixBase::Unit{X,Y,Z}, AngleAxis can be used to easily
  * mimic Euler-angles. Here is an example:
  * \include AngleAxis_mimic_euler.cpp
  * Output: \verbinclude AngleAxis_mimic_euler.out
  *
  * \note This class is not aimed to be used to store a rotation transformation,
  * but rather to make easier the creation of other rotation (Quaternion, rotation Matrix)
  * and transformation objects.
  *
  * \sa class Quaternion, class Transform, MatrixBase::UnitX()
  */

namespace internal {
template<typename _Scalar> struct traits<AngleAxis<_Scalar> >
{
  typedef _Scalar Scalar;
};
}

template<typename _Scalar>
class AngleAxis : public RotationBase<AngleAxis<_Scalar>,3>
{
  typedef RotationBase<AngleAxis<_Scalar>,3> Base;

public:

  using Base::operator*;

  enum { Dim = 3 };
  /** the scalar type of the coefficients */
  typedef _Scalar Scalar;
  typedef Matrix<Scalar,3,3> Matrix3;
  typedef Matrix<Scalar,3,1> Vector3;
  typedef Quaternion<Scalar> QuaternionType;

protected:

  Vector3 m_axis;
  Scalar m_angle;

public:

  /** Default constructor without initialization. */
  EIGEN_DEVICE_FUNC AngleAxis() {}
  /** Constructs and initialize the angle-axis rotation from an \a angle in radian
    * and an \a axis which \b must \b be \b normalized.
    *
    * \warning If the \a axis vector is not normalized, then the angle-axis object
    *          represents an invalid rotation. */
  template<typename Derived>
  EIGEN_DEVICE_FUNC 
  inline AngleAxis(const Scalar& angle, const MatrixBase<Derived>& axis) : m_axis(axis), m_angle(angle) {}
  /** Constructs and initialize the angle-axis rotation from a quaternion \a q.
    * This function implicitly normalizes the quaternion \a q.
    */
  template<typename QuatDerived> 
  EIGEN_DEVICE_FUNC inline explicit AngleAxis(const QuaternionBase<QuatDerived>& q) { *this = q; }
  /** Constructs and initialize the angle-axis rotation from a 3x3 rotation matrix. */
  template<typename Derived>
  EIGEN_DEVICE_FUNC inline explicit AngleAxis(const MatrixBase<Derived>& m) { *this = m; }

  /** \returns the value of the rotation angle in radian */
  EIGEN_DEVICE_FUNC Scalar angle() const { return m_angle; }
  /** \returns a read-write reference to the stored angle in radian */
  EIGEN_DEVICE_FUNC Scalar& angle() { return m_angle; }

  /** \returns the rotation axis */
  EIGEN_DEVICE_FUNC const Vector3& axis() const { return m_axis; }
  /** \returns a read-write reference to the stored rotation axis.
    *
    * \warning The rotation axis must remain a \b unit vector.
    */
  EIGEN_DEVICE_FUNC Vector3& axis() { return m_axis; }

  /** Concatenates two rotations */
  EIGEN_DEVICE_FUNC inline QuaternionType operator* (const AngleAxis& other) const
  { return QuaternionType(*this) * QuaternionType(other); }

  /** Concatenates two rotations */
  EIGEN_DEVICE_FUNC inline QuaternionType operator* (const QuaternionType& other) const
  { return QuaternionType(*this) * other; }

  /** Concatenates two rotations */
  friend EIGEN_DEVICE_FUNC inline QuaternionType operator* (const QuaternionType& a, const AngleAxis& b)
  { return a * QuaternionType(b); }

  /** \returns the inverse rotation, i.e., an angle-axis with opposite rotation angle */
  EIGEN_DEVICE_FUNC AngleAxis inverse() const
  { return AngleAxis(-m_angle, m_axis); }

  template<class QuatDerived>
  EIGEN_DEVICE_FUNC AngleAxis& operator=(const QuaternionBase<QuatDerived>& q);
  template<typename Derived>
  EIGEN_DEVICE_FUNC AngleAxis& operator=(const MatrixBase<Derived>& m);

  template<typename Derived>
  EIGEN_DEVICE_FUNC AngleAxis& fromRotationMatrix(const MatrixBase<Derived>& m);
  EIGEN_DEVICE_FUNC Matrix3 toRotationMatrix(void) const;

  /** \returns \c *this with scalar type casted to \a NewScalarType
    *
    * Note that if \a NewScalarType is equal to the current scalar type of \c *this
    * then this function smartly returns a const reference to \c *this.
    */
  template<typename NewScalarType>
  EIGEN_DEVICE_FUNC inline typename internal::cast_return_type<AngleAxis,AngleAxis<NewScalarType> >::type cast() const
  { return typename internal::cast_return_type<AngleAxis,AngleAxis<NewScalarType> >::type(*this); }

  /** Copy constructor with scalar type conversion */
  template<typename OtherScalarType>
  EIGEN_DEVICE_FUNC inline explicit AngleAxis(const AngleAxis<OtherScalarType>& other)
  {
    m_axis = other.axis().template cast<Scalar>();
    m_angle = Scalar(other.angle());
  }

  EIGEN_DEVICE_FUNC static inline const AngleAxis Identity() { return AngleAxis(Scalar(0), Vector3::UnitX()); }

  /** \returns \c true if \c *this is approximately equal to \a other, within the precision
    * determined by \a prec.
    *
    * \sa MatrixBase::isApprox() */
  EIGEN_DEVICE_FUNC bool isApprox(const AngleAxis& other, const typename NumTraits<Scalar>::Real& prec = NumTraits<Scalar>::dummy_precision()) const
  { return m_axis.isApprox(other.m_axis, prec) && internal::isApprox(m_angle,other.m_angle, prec); }
};

/** \ingroup Geometry_Module
  * single precision angle-axis type */
typedef AngleAxis<float> AngleAxisf;
/** \ingroup Geometry_Module
  * double precision angle-axis type */
typedef AngleAxis<double> AngleAxisd;

/** Set \c *this from a \b unit quaternion.
  *
  * The resulting axis is normalized, and the computed angle is in the [0,pi] range.
  * 
  * This function implicitly normalizes the quaternion \a q.
  */
template<typename Scalar>
template<typename QuatDerived>
EIGEN_DEVICE_FUNC AngleAxis<Scalar>& AngleAxis<Scalar>::operator=(const QuaternionBase<QuatDerived>& q)
{
  EIGEN_USING_STD_MATH(atan2)
  EIGEN_USING_STD_MATH(abs)
  Scalar n = q.vec().norm();
  if(n<NumTraits<Scalar>::epsilon())
    n = q.vec().stableNorm();

  if (n != Scalar(0))
  {
    m_angle = Scalar(2)*atan2(n, abs(q.w()));
    if(q.w() < Scalar(0))
      n = -n;
    m_axis  = q.vec() / n;
  }
  else
  {
      // This part of modification is refer to ceres::QuaternionToAngleAxis(const T* quaternion, T* angle_axis)
      // Link: https://github.com/ceres-solver/ceres-solver/blob/master/include/ceres/rotation.h
      m_angle = Scalar( std::sqrt(1.0) );
      m_axis << Scalar(2.0) * q.x(), Scalar(2.0) * q.y(), Scalar(2.0) * q.z();
  }
  return *this;
}

/** Set \c *this from a 3x3 rotation matrix \a mat.
  */
template<typename Scalar>
template<typename Derived>
EIGEN_DEVICE_FUNC AngleAxis<Scalar>& AngleAxis<Scalar>::operator=(const MatrixBase<Derived>& mat)
{
  // Since a direct conversion would not be really faster,
  // let's use the robust Quaternion implementation:
  return *this = QuaternionType(mat);
}

/**
* \brief Sets \c *this from a 3x3 rotation matrix.
**/
template<typename Scalar>
template<typename Derived>
EIGEN_DEVICE_FUNC AngleAxis<Scalar>& AngleAxis<Scalar>::fromRotationMatrix(const MatrixBase<Derived>& mat)
{
  return *this = QuaternionType(mat);
}

/** Constructs and \returns an equivalent 3x3 rotation matrix.
  */
template<typename Scalar>
typename AngleAxis<Scalar>::Matrix3
EIGEN_DEVICE_FUNC AngleAxis<Scalar>::toRotationMatrix(void) const
{
  EIGEN_USING_STD_MATH(sin)
  EIGEN_USING_STD_MATH(cos)
  Matrix3 res;
  Vector3 sin_axis  = sin(m_angle) * m_axis;
  Scalar c = cos(m_angle);
  Vector3 cos1_axis = (Scalar(1)-c) * m_axis;

  Scalar tmp;
  tmp = cos1_axis.x() * m_axis.y();
  res.coeffRef(0,1) = tmp - sin_axis.z();
  res.coeffRef(1,0) = tmp + sin_axis.z();

  tmp = cos1_axis.x() * m_axis.z();
  res.coeffRef(0,2) = tmp + sin_axis.y();
  res.coeffRef(2,0) = tmp - sin_axis.y();

  tmp = cos1_axis.y() * m_axis.z();
  res.coeffRef(1,2) = tmp - sin_axis.x();
  res.coeffRef(2,1) = tmp + sin_axis.x();

  res.diagonal() = (cos1_axis.cwiseProduct(m_axis)).array() + c;

  return res;
}

} // end namespace Eigen

#endif // EIGEN_ANGLEAXIS_H
