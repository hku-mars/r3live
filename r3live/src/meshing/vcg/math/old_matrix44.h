/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2004                                                \/)\/    *
* Visual Computing Lab                                            /\/|      *
* ISTI - Italian National Research Council                           |      *
*                                                                    \      *
* All rights reserved.                                                      *
*                                                                           *
* This program is free software; you can redistribute it and/or modify      *
* it under the terms of the GNU General Public License as published by      *
* the Free Software Foundation; either version 2 of the License, or         *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
****************************************************************************/

#ifndef VCG_USE_EIGEN
#include "deprecated_matrix44.h"
#else

#ifndef __VCGLIB_MATRIX44
#define __VCGLIB_MATRIX44

#include "eigen.h"
#include <vcg/space/point3.h>
#include <vcg/space/point4.h>
#include <memory.h>
#include <vector>

namespace vcg{
template<class Scalar> class Matrix44;
}

namespace Eigen{
template<typename Scalar>
struct ei_traits<vcg::Matrix44<Scalar> > : ei_traits<Eigen::Matrix<Scalar,4,4,RowMajor> > {};
template<typename XprType> struct ei_to_vcgtype<XprType,4,4,RowMajor,4,4>
{ typedef vcg::Matrix44<typename XprType::Scalar> type; };
}

namespace vcg {

	/*
	Annotations:
Opengl stores matrix in  column-major order. That is, the matrix is stored as:

	a0  a4  a8  a12
	a1  a5  a9  a13
	a2  a6  a10 a14
	a3  a7  a11 a15

	Usually in opengl (see opengl specs) vectors are 'column' vectors
	so usually matrix are PRE-multiplied for a vector.
	So the command glTranslate generate a matrix that
	is ready to be premultipled for a vector:

	1 0 0 tx
	0 1 0 ty
	0 0 1 tz
	0 0 0  1

Matrix44 stores matrix in row-major order i.e.

	a0  a1  a2  a3
	a4  a5  a6  a7
	a8  a9  a10 a11
	a12 a13 a14 a15

So for the use of that matrix in opengl with their supposed meaning you have to transpose them before feeding to glMultMatrix.
This mechanism is hidden by the templated function defined in wrap/gl/math.h;
If your machine has the ARB_transpose_matrix extension it will use the appropriate;
The various gl-like command SetRotate, SetTranslate assume that you are making matrix
for 'column' vectors.

*/

// Note that we have to pass Dim and HDim because it is not allowed to use a template
// parameter to define a template specialization. To be more precise, in the following
// specializations, it is not allowed to use Dim+1 instead of HDim.
template< typename Other,
		  int OtherRows=Eigen::ei_traits<Other>::RowsAtCompileTime,
          int OtherCols=Eigen::ei_traits<Other>::ColsAtCompileTime>
struct ei_matrix44_product_impl;

/** \deprecated use Eigen::Matrix<Scalar,4,4> (or the typedef) you want a real 4x4 matrix, or use Eigen::Transform<Scalar,3> if you want a transformation matrix for a 3D space (a Eigen::Transform<Scalar,3> is internally a 4x4 col-major matrix)
  *
  * This class represents a 4x4 matrix. T is the kind of element in the matrix.
  */
template<typename _Scalar>
class Matrix44 : public Eigen::Matrix<_Scalar,4,4,Eigen::RowMajor> // FIXME col or row major !
{

	typedef Eigen::Matrix<_Scalar,4,4,Eigen::RowMajor> _Base;
public:

	using _Base::coeff;
	using _Base::coeffRef;
	using _Base::ElementAt;
	using _Base::setZero;

	_EIGEN_GENERIC_PUBLIC_INTERFACE(Matrix44,_Base);
	typedef _Scalar ScalarType;
	VCG_EIGEN_INHERIT_ASSIGNMENT_OPERATORS(Matrix44)

	Matrix44() : Base() {}
	~Matrix44() {}
	Matrix44(const Matrix44 &m) : Base(m) {}
	Matrix44(const Scalar * v ) : Base(Eigen::Map<Eigen::Matrix<Scalar,4,4,Eigen::RowMajor> >(v)) {}
	template<typename OtherDerived>
	Matrix44(const Eigen::MatrixBase<OtherDerived>& other) : Base(other) {}

	const typename Base::RowXpr operator[](int i) const { return Base::row(i); }
	typename Base::RowXpr operator[](int i) { return Base::row(i); }

	typename Base::ColXpr GetColumn4(const int& i) const { return Base::col(i); }
	const Eigen::Block<Base,3,1> GetColumn3(const int& i) const { return this->template block<3,1>(0,i); }

	typename Base::RowXpr GetRow4(const int& i) const { return Base::row(i); }
	Eigen::Block<Base,1,3> GetRow3(const int& i) const { return this->template block<1,3>(i,0); }

	template <class Matrix44Type>
	void ToMatrix(Matrix44Type & m) const { m = (*this).template cast<typename Matrix44Type::Scalar>(); }

	void ToEulerAngles(Scalar &alpha, Scalar &beta, Scalar &gamma);

	template <class Matrix44Type>
	void FromMatrix(const Matrix44Type & m) { for(int i = 0; i < 16; i++) Base::data()[i] = m.data()[i]; }

	void FromEulerAngles(Scalar alpha, Scalar beta, Scalar gamma);
	void SetDiagonal(const Scalar k);
	Matrix44 &SetScale(const Scalar sx, const Scalar sy, const Scalar sz);
	Matrix44 &SetScale(const Point3<Scalar> &t);
	Matrix44 &SetTranslate(const Point3<Scalar> &t);
	Matrix44 &SetTranslate(const Scalar sx, const Scalar sy, const Scalar sz);
	Matrix44 &SetShearXY(const Scalar sz);
	Matrix44 &SetShearXZ(const Scalar sy);
	Matrix44 &SetShearYZ(const Scalar sx);

	///use radiants for angle.
	Matrix44 &SetRotateDeg(Scalar AngleDeg, const Point3<Scalar> & axis);
	Matrix44 &SetRotateRad(Scalar AngleRad, const Point3<Scalar> & axis);

	/** taken from Eigen::Transform
	* \returns the product between the transform \c *this and a matrix expression \a other
	*
	* The right hand side \a other might be either:
	* \li a matrix expression with 4 rows
	* \li a 3D vector/point
	*/
	template<typename OtherDerived>
	inline const typename ei_matrix44_product_impl<OtherDerived>::ResultType
	operator * (const Eigen::MatrixBase<OtherDerived> &other) const
	{ return ei_matrix44_product_impl<OtherDerived>::run(*this,other.derived()); }

	void print() {std::cout << *this << "\n\n";}

};

//return NULL matrix if not invertible
template <class T> Matrix44<T> &Invert(Matrix44<T> &m);
template <class T> Matrix44<T> Inverse(const Matrix44<T> &m);

typedef Matrix44<short>  Matrix44s;
typedef Matrix44<int>    Matrix44i;
typedef Matrix44<float>  Matrix44f;
typedef Matrix44<double> Matrix44d;

template < class PointType , class T > void operator*=( std::vector<PointType> &vert, const Matrix44<T> & m ) {
	typename std::vector<PointType>::iterator ii;
	for(ii=vert.begin();ii!=vert.end();++ii)
		(*ii).P()=m * (*ii).P();
}

template <class T>
void Matrix44<T>::ToEulerAngles(Scalar &alpha, Scalar &beta, Scalar &gamma)
{
	alpha = atan2(coeff(1,2), coeff(2,2));
	beta = asin(-coeff(0,2));
	gamma = atan2(coeff(0,1), coeff(1,1));
}

template <class T>
void Matrix44<T>::FromEulerAngles(Scalar alpha, Scalar beta, Scalar gamma)
{
	this->SetZero();

	T cosalpha = cos(alpha);
	T cosbeta = cos(beta);
	T cosgamma = cos(gamma);
	T sinalpha = sin(alpha);
	T sinbeta = sin(beta);
	T singamma = sin(gamma);

	ElementAt(0,0) = cosbeta * cosgamma;
	ElementAt(1,0) = -cosalpha * singamma + sinalpha * sinbeta * cosgamma;
	ElementAt(2,0) = sinalpha * singamma + cosalpha * sinbeta * cosgamma;

	ElementAt(0,1) = cosbeta * singamma;
	ElementAt(1,1) = cosalpha * cosgamma + sinalpha * sinbeta * singamma;
	ElementAt(2,1) = -sinalpha * cosgamma + cosalpha * sinbeta * singamma;

	ElementAt(0,2) = -sinbeta;
	ElementAt(1,2) = sinalpha * cosbeta;
	ElementAt(2,2) = cosalpha * cosbeta;

	ElementAt(3,3) = 1;
}

template <class T> void Matrix44<T>::SetDiagonal(const Scalar k) {
	setZero();
	ElementAt(0, 0) = k;
	ElementAt(1, 1) = k;
	ElementAt(2, 2) = k;
	ElementAt(3, 3) = 1;
}

template <class T> Matrix44<T> &Matrix44<T>::SetScale(const Point3<Scalar> &t) {
	SetScale(t[0], t[1], t[2]);
	return *this;
}
template <class T> Matrix44<T> &Matrix44<T>::SetScale(const Scalar sx, const Scalar sy, const Scalar sz) {
	setZero();
	ElementAt(0, 0) = sx;
	ElementAt(1, 1) = sy;
	ElementAt(2, 2) = sz;
	ElementAt(3, 3) = 1;
	return *this;
}

template <class T> Matrix44<T> &Matrix44<T>::SetTranslate(const Point3<Scalar> &t) {
	SetTranslate(t[0], t[1], t[2]);
	return *this;
}
template <class T> Matrix44<T> &Matrix44<T>::SetTranslate(const Scalar tx, const Scalar ty, const Scalar tz) {
	Base::setIdentity();
	ElementAt(0, 3) = tx;
	ElementAt(1, 3) = ty;
	ElementAt(2, 3) = tz;
	return *this;
}

template <class T> Matrix44<T> &Matrix44<T>::SetRotateDeg(Scalar AngleDeg, const Point3<Scalar> & axis) {
	return SetRotateRad(math::ToRad(AngleDeg),axis);
}

template <class T> Matrix44<T> &Matrix44<T>::SetRotateRad(Scalar AngleRad, const Point3<Scalar> & axis) {
	//angle = angle*(T)3.14159265358979323846/180; e' in radianti!
	T c = math::Cos(AngleRad);
	T s = math::Sin(AngleRad);
	T q = 1-c;
	Point3<T> t = axis;
	t.Normalize();
	ElementAt(0,0) = t[0]*t[0]*q + c;
	ElementAt(0,1) = t[0]*t[1]*q - t[2]*s;
	ElementAt(0,2) = t[0]*t[2]*q + t[1]*s;
	ElementAt(0,3) = 0;
	ElementAt(1,0) = t[1]*t[0]*q + t[2]*s;
	ElementAt(1,1) = t[1]*t[1]*q + c;
	ElementAt(1,2) = t[1]*t[2]*q - t[0]*s;
	ElementAt(1,3) = 0;
	ElementAt(2,0) = t[2]*t[0]*q -t[1]*s;
	ElementAt(2,1) = t[2]*t[1]*q +t[0]*s;
	ElementAt(2,2) = t[2]*t[2]*q +c;
	ElementAt(2,3) = 0;
	ElementAt(3,0) = 0;
	ElementAt(3,1) = 0;
	ElementAt(3,2) = 0;
	ElementAt(3,3) = 1;
	return *this;
}

/* Shear Matrixes
XY
1 k 0 0   x    x+ky
0 1 0 0   y     y
0 0 1 0   z     z
0 0 0 1   1     1

1 0 k 0   x    x+kz
0 1 0 0   y     y
0 0 1 0   z     z
0 0 0 1   1     1

1 1 0 0   x     x
0 1 k 0   y     y+kz
0 0 1 0   z     z
0 0 0 1   1     1

*/

	template <class T> Matrix44<T> & Matrix44<T>::SetShearXY( const Scalar sh)	{// shear the X coordinate as the Y coordinate change
		Base::setIdentity();
		ElementAt(0,1) = sh;
		return *this;
	}

	template <class T> Matrix44<T> & Matrix44<T>::SetShearXZ( const Scalar sh)	{// shear the X coordinate as the Z coordinate change
		Base::setIdentity();
		ElementAt(0,2) = sh;
		return *this;
	}

	template <class T> Matrix44<T> &Matrix44<T>::SetShearYZ( const Scalar sh)	{// shear the Y coordinate as the Z coordinate change
		Base::setIdentity();
		ElementAt(1,2) = sh;
		return *this;
	}


/*
Given a non singular, non projective matrix (e.g. with the last row equal to [0,0,0,1] )
This procedure decompose it in a sequence of
	Scale,Shear,Rotation e Translation

- ScaleV and Tranv are obiviously scaling and translation.
- ShearV contains three scalars with, respectively
			ShearXY, ShearXZ e ShearYZ
- RotateV contains the rotations (in degree!) around the x,y,z axis
	The input matrix is modified leaving inside it a simple roto translation.

	To obtain the original matrix the above transformation have to be applied in the strict following way:

	OriginalMatrix =  Trn * Rtx*Rty*Rtz  * ShearYZ*ShearXZ*ShearXY * Scl

Example Code:
double srv() { return (double(rand()%40)-20)/2.0; } // small random value

	srand(time(0));
	Point3d ScV(10+srv(),10+srv(),10+srv()),ScVOut(-1,-1,-1);
	Point3d ShV(srv(),srv(),srv()),ShVOut(-1,-1,-1);
	Point3d RtV(10+srv(),srv(),srv()),RtVOut(-1,-1,-1);
	Point3d TrV(srv(),srv(),srv()),TrVOut(-1,-1,-1);

	Matrix44d Scl; Scl.SetScale(ScV);
	Matrix44d Sxy; Sxy.SetShearXY(ShV[0]);
	Matrix44d Sxz; Sxz.SetShearXZ(ShV[1]);
	Matrix44d Syz; Syz.SetShearYZ(ShV[2]);
	Matrix44d Rtx; Rtx.SetRotate(math::ToRad(RtV[0]),Point3d(1,0,0));
	Matrix44d Rty; Rty.SetRotate(math::ToRad(RtV[1]),Point3d(0,1,0));
	Matrix44d Rtz; Rtz.SetRotate(math::ToRad(RtV[2]),Point3d(0,0,1));
	Matrix44d Trn; Trn.SetTranslate(TrV);

	Matrix44d StartM =  Trn * Rtx*Rty*Rtz  * Syz*Sxz*Sxy *Scl;
	Matrix44d ResultM=StartM;
	Decompose(ResultM,ScVOut,ShVOut,RtVOut,TrVOut);

	Scl.SetScale(ScVOut);
	Sxy.SetShearXY(ShVOut[0]);
	Sxz.SetShearXZ(ShVOut[1]);
	Syz.SetShearYZ(ShVOut[2]);
	Rtx.SetRotate(math::ToRad(RtVOut[0]),Point3d(1,0,0));
	Rty.SetRotate(math::ToRad(RtVOut[1]),Point3d(0,1,0));
	Rtz.SetRotate(math::ToRad(RtVOut[2]),Point3d(0,0,1));
	Trn.SetTranslate(TrVOut);

	// Now Rebuild is equal to StartM
	Matrix44d RebuildM =  Trn * Rtx*Rty*Rtz  * Syz*Sxz*Sxy * Scl ;
*/
template <class T>
bool Decompose(Matrix44<T> &M, Point3<T> &ScaleV, Point3<T> &ShearV, Point3<T> &RotV,Point3<T> &TranV)
{
	if(!(M(3,0)==0 && M(3,1)==0 && M(3,2)==0 && M(3,3)==1) ) // the matrix is projective
		return false;
	if(math::Abs(M.Determinant())<1e-10) return false; // matrix should be at least invertible...

	// First Step recover the traslation
	TranV=M.GetColumn3(3);

	// Second Step Recover Scale and Shearing interleaved
	ScaleV[0]=Norm(M.GetColumn3(0));
	Point3<T> R[3];
	R[0]=M.GetColumn3(0);
	R[0].Normalize();

	ShearV[0]=R[0].dot(M.GetColumn3(1)); // xy shearing
	R[1]= M.GetColumn3(1)-R[0]*ShearV[0];
	assert(math::Abs(R[1].dot(R[0]))<1e-10);
	ScaleV[1]=Norm(R[1]);   // y scaling
	R[1]=R[1]/ScaleV[1];
	ShearV[0]=ShearV[0]/ScaleV[1];

	ShearV[1]=R[0].dot(M.GetColumn3(2)); // xz shearing
	R[2]= M.GetColumn3(2)-R[0]*ShearV[1];
	assert(math::Abs(R[2].dot(R[0]))<1e-10);

	R[2] = R[2]-R[1]*(R[2].dot(R[1]));
	assert(math::Abs(R[2].dot(R[1]))<1e-10);
	assert(math::Abs(R[2].dot(R[0]))<1e-10);

	ScaleV[2]=Norm(R[2]);
	ShearV[1]=ShearV[1]/ScaleV[2];
	R[2]=R[2]/ScaleV[2];
	assert(math::Abs(R[2].dot(R[1]))<1e-10);
	assert(math::Abs(R[2].dot(R[0]))<1e-10);

	ShearV[2]=R[1].dot(M.GetColumn3(2)); // yz shearing
	ShearV[2]=ShearV[2]/ScaleV[2];
	int i,j;
	for(i=0;i<3;++i)
		for(j=0;j<3;++j)
				M(i,j)=R[j][i];

	// Third and last step: Recover the rotation
	//now the matrix should be a pure rotation matrix so its determinant is +-1
	double det=M.Determinant();
	if(math::Abs(det)<1e-10) return false; // matrix should be at least invertible...
	assert(math::Abs(math::Abs(det)-1.0)<1e-10); // it should be +-1...
	if(det<0) {
		ScaleV  *= -1;
		M *= -1;
	}

	double alpha,beta,gamma; // rotations around the x,y and z axis
	beta=asin( M(0,2));
	double cosbeta=cos(beta);
	if(math::Abs(cosbeta) > 1e-5)
		{
			alpha=asin(-M(1,2)/cosbeta);
			if((M(2,2)/cosbeta) < 0 ) alpha=M_PI-alpha;
			gamma=asin(-M(0,1)/cosbeta);
			if((M(0,0)/cosbeta)<0) gamma = M_PI-gamma;
		}
	else
		{
			alpha=asin(-M(1,0));
			if(M(1,1)<0) alpha=M_PI-alpha;
			gamma=0;
		}

	RotV[0]=math::ToDeg(alpha);
	RotV[1]=math::ToDeg(beta);
	RotV[2]=math::ToDeg(gamma);

	return true;
}

/*
To invert a matrix you can
either invert the matrix inplace calling

vcg::Invert(yourMatrix);

or get the inverse matrix of a given matrix without touching it:

invertedMatrix = vcg::Inverse(untouchedMatrix);

*/
template <class T> Matrix44<T> & Invert(Matrix44<T> &m) {
	return m = m.lu().inverse();
}

template <class T> Matrix44<T> Inverse(const Matrix44<T> &m) {
	return m.lu().inverse();
}

template<typename Other,int OtherCols>
struct ei_matrix44_product_impl<Other, 4,OtherCols>
{
	typedef typename Other::Scalar Scalar;
	typedef typename Eigen::ProductReturnType<typename Matrix44<Scalar>::Base,Other>::Type ResultType;
	static ResultType run(const Matrix44<Scalar>& tr, const Other& other)
	{ return (static_cast<const typename Matrix44<Scalar>::Base&>(tr)) * other; }
};

template<typename Other>
struct ei_matrix44_product_impl<Other, 3,1>
{
	typedef typename Other::Scalar Scalar;
	typedef Eigen::Matrix<Scalar,3,1> ResultType;
	static ResultType run(const Matrix44<Scalar>& tr, const Other& p)
	{
		Scalar w;
		Eigen::Matrix<Scalar,3,1> s;
		s[0] = tr.ElementAt(0, 0)*p[0] + tr.ElementAt(0, 1)*p[1] + tr.ElementAt(0, 2)*p[2] + tr.ElementAt(0, 3);
		s[1] = tr.ElementAt(1, 0)*p[0] + tr.ElementAt(1, 1)*p[1] + tr.ElementAt(1, 2)*p[2] + tr.ElementAt(1, 3);
		s[2] = tr.ElementAt(2, 0)*p[0] + tr.ElementAt(2, 1)*p[1] + tr.ElementAt(2, 2)*p[2] + tr.ElementAt(2, 3);
			w = tr.ElementAt(3, 0)*p[0] + tr.ElementAt(3, 1)*p[1] + tr.ElementAt(3, 2)*p[2] + tr.ElementAt(3, 3);
		if(w!= 0) s /= w;
		return s;
	}
};

} //namespace
#endif

#endif
