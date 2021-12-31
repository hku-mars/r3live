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

#ifndef __VCGLIB_MATRIX44
#define __VCGLIB_MATRIX44

#include <memory.h>
#include <vcg/math/base.h>
#include <vcg/space/point3.h>
#include <vcg/space/point4.h>
#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/LU>

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

/** This class represent a 4x4 matrix. T is the kind of element in the matrix.
	*/
template <class T> class Matrix44 {
protected:
	T _a[16];

public:
	typedef T ScalarType;

	///@{

	/** $name Constructors
	*  No automatic casting and default constructor is empty
	*/
	Matrix44() {}
	~Matrix44() {}
	Matrix44(const Matrix44 &m);
	Matrix44(const T v[]);

	T &ElementAt(const int row, const int col);
	T ElementAt(const int row, const int col) const;
	//T &operator[](const int i);
	//const T &operator[](const int i) const;
	T *V();
	const T *V() const ;

	T *operator[](const int i);
	const T *operator[](const int i) const;

	// return a copy of the i-th column
	Point4<T> GetColumn4(const int& i)const{
		assert(i>=0 && i<4);
		return Point4<T>(ElementAt(0,i),ElementAt(1,i),ElementAt(2,i),ElementAt(3,i));
		//return Point4<T>(_a[i],_a[i+4],_a[i+8],_a[i+12]);
	}

	Point3<T> GetColumn3(const int& i)const{
		assert(i>=0 && i<4);
		return Point3<T>(ElementAt(0,i),ElementAt(1,i),ElementAt(2,i));
	}

	Point4<T> GetRow4(const int& i)const{
		assert(i>=0 && i<4);
		return Point4<T>(ElementAt(i,0),ElementAt(i,1),ElementAt(i,2),ElementAt(i,3));
		// return *((Point4<T>*)(&_a[i<<2])); alternativa forse + efficiente
	}

	Point3<T> GetRow3(const int& i)const{
		assert(i>=0 && i<4);
		return Point3<T>(ElementAt(i,0),ElementAt(i,1),ElementAt(i,2));
		// return *((Point4<T>*)(&_a[i<<2])); alternativa forse + efficiente
	}

	Matrix44 operator+(const Matrix44 &m) const;
	Matrix44 operator-(const Matrix44 &m) const;
	Matrix44 operator*(const Matrix44 &m) const;
	Point4<T> operator*(const Point4<T> &v) const;

	bool operator==(const  Matrix44 &m) const;
	bool operator!= (const  Matrix44 &m) const;

	Matrix44 operator-() const;
	Matrix44 operator*(const T k) const;
	void operator+=(const Matrix44 &m);
	void operator-=(const Matrix44 &m);
	void operator*=( const Matrix44 & m );
	void operator*=( const T k );

	template <class Matrix44Type>
	void ToMatrix(Matrix44Type & m) const {for(int i = 0; i < 16; i++) m.V()[i]=V()[i];}

	void ToEulerAngles(T &alpha, T &beta, T &gamma);

	template <class Matrix44Type>
	void FromMatrix(const Matrix44Type & m){for(int i = 0; i < 16; i++) V()[i]=m.V()[i];}

	template <class EigenMatrix44Type>
	void ToEigenMatrix(EigenMatrix44Type & m) const {
		for(int i = 0; i < 4; i++)
			for(int j = 0; j < 4; j++)
				m(i,j)=(*this)[i][j];
	}

	template <class EigenMatrix44Type>
	void FromEigenMatrix(const EigenMatrix44Type & m){
		for(int i = 0; i < 4; i++)
			for(int j = 0; j < 4; j++)
				ElementAt(i,j)=m(i,j);
	}

	void FromEulerAngles(T alpha, T beta, T gamma);
	void SetZero();
	void SetIdentity();
	void SetDiagonal(const T k);
	Matrix44 &SetScale(const T sx, const T sy, const T sz);
	Matrix44 &SetScale(const Point3<T> &t);
	Matrix44<T>& SetColumn(const unsigned int ii,const Point4<T> &t);
	Matrix44<T>& SetColumn(const unsigned int ii,const Point3<T> &t);
	Matrix44 &SetTranslate(const Point3<T> &t);
	Matrix44 &SetTranslate(const T sx, const T sy, const T sz);
	Matrix44 &SetShearXY(const T sz);
	Matrix44 &SetShearXZ(const T sy);
	Matrix44 &SetShearYZ(const T sx);

	///use radiants for angle.
	Matrix44 &SetRotateDeg(T AngleDeg, const Point3<T> & axis);
	Matrix44 &SetRotateRad(T AngleRad, const Point3<T> & axis);

	T Determinant() const;

	template <class Q> void Import(const Matrix44<Q> &m) {
		for(int i = 0; i < 16; i++)
			_a[i] = (T)(m.V()[i]);
	}
	template <class Q>
	static inline Matrix44 Construct( const Matrix44<Q> & b )
	{
		Matrix44<T> tmp; tmp.FromMatrix(b);
		return tmp;
	}

	static inline const Matrix44 &Identity( )
	{
		static Matrix44<T> tmp; tmp.SetIdentity();
		return tmp;
	}

	// for the transistion to eigen
	Matrix44 transpose() const
	{
		Matrix44 res = *this;
		Transpose(res);
		return res;
	}
	void transposeInPlace() { Transpose(*this); }

	void print() {
		unsigned int i, j, p;
		for (i=0, p=0; i<4; i++, p+=4)
		{
			std::cout << "[\t";
			for (j=0; j<4; j++)
				std::cout << _a[p+j] << "\t";

			std::cout << "]\n";
		}
		std::cout << "\n";
	}

};


/** Class for solving A * x = b. */
template <class T> class LinearSolve: public Matrix44<T> {
public:
	LinearSolve(const Matrix44<T> &m);
	Point4<T> Solve(const Point4<T> &b); // solve A � x = b
	///If you need to solve some equation you can use this function instead of Matrix44 one for speed.
	T Determinant() const;
protected:
	///Holds row permutation.
	int index[4]; //hold permutation
	///Hold sign of row permutation (used for determinant sign)
	T d;
	bool Decompose();
};

/*** Postmultiply */
//template <class T> Point3<T> operator*(const Point3<T> &p, const Matrix44<T> &m);

///Premultiply
template <class T> Point3<T> operator*(const Matrix44<T> &m, const Point3<T> &p);

template <class T> Matrix44<T> &Transpose(Matrix44<T> &m);
//return NULL matrix if not invertible
template <class T> Matrix44<T> Inverse(const Matrix44<T> &m);

typedef Matrix44<short>  Matrix44s;
typedef Matrix44<int>    Matrix44i;
typedef Matrix44<float>  Matrix44f;
typedef Matrix44<double> Matrix44d;



template <class T> Matrix44<T>::Matrix44(const Matrix44<T> &m) {
    memcpy((T *)_a, (const T *)m._a, 16 * sizeof(T));
}

template <class T> Matrix44<T>::Matrix44(const T v[]) {
	memcpy((T *)_a, v, 16 * sizeof(T));
}

template <class T> T &Matrix44<T>::ElementAt(const int row, const int col) {
	assert(row >= 0 && row < 4);
	assert(col >= 0 && col < 4);
	return _a[(row<<2) + col];
}

template <class T> T Matrix44<T>::ElementAt(const int row, const int col) const {
	assert(row >= 0 && row < 4);
	assert(col >= 0 && col < 4);
	return _a[(row<<2) + col];
}

//template <class T> T &Matrix44<T>::operator[](const int i) {
//	assert(i >= 0 && i < 16);
//	return ((T *)_a)[i];
//}
//
//template <class T> const T &Matrix44<T>::operator[](const int i) const {
//	assert(i >= 0 && i < 16);
//	return ((T *)_a)[i];
//}
template <class T> T *Matrix44<T>::operator[](const int i) {
	assert(i >= 0 && i < 4);
	return _a+i*4;
}

template <class T> const T *Matrix44<T>::operator[](const int i) const {
	assert(i >= 0 && i < 4);
	return _a+i*4;
}
template <class T>  T *Matrix44<T>::V()  { return _a;}
template <class T> const T *Matrix44<T>::V() const { return _a;}


template <class T> Matrix44<T> Matrix44<T>::operator+(const Matrix44 &m) const {
	Matrix44<T> ret;
	for(int i = 0; i < 16; i++)
		ret.V()[i] = V()[i] + m.V()[i];
	return ret;
}

template <class T> Matrix44<T> Matrix44<T>::operator-(const Matrix44 &m) const {
	Matrix44<T> ret;
	for(int i = 0; i < 16; i++)
		ret.V()[i] = V()[i] - m.V()[i];
	return ret;
}

template <class T> Matrix44<T> Matrix44<T>::operator*(const Matrix44 &m) const {
	Matrix44 ret;
	for(int i = 0; i < 4; i++)
		for(int j = 0; j < 4; j++) {
			T t = 0.0;
			for(int k = 0; k < 4; k++)
				t += ElementAt(i, k) * m.ElementAt(k, j);
			ret.ElementAt(i, j) = t;
		}
	return ret;
}

template <class T> Point4<T> Matrix44<T>::operator*(const Point4<T> &v) const {
	Point4<T> ret;
	for(int i = 0; i < 4; i++){
		T t = 0.0;
		for(int k = 0; k < 4; k++)
			t += ElementAt(i,k) * v[k];
		ret[i] = t;
	}
	return ret;
}


template <class T> bool Matrix44<T>::operator==(const  Matrix44 &m) const {
	for(int i = 0; i < 4; ++i)
		for(int j = 0; j < 4; ++j)
			if(ElementAt(i,j) != m.ElementAt(i,j))
				return false;
	return true;
}
template <class T> bool Matrix44<T>::operator!=(const  Matrix44 &m) const {
	for(int i = 0; i < 4; ++i)
		for(int j = 0; j < 4; ++j)
			if(ElementAt(i,j) != m.ElementAt(i,j))
				return true;
	return false;
}

template <class T> Matrix44<T> Matrix44<T>::operator-() const {
	Matrix44<T> res;
	for(int i = 0; i < 16; i++)
		res.V()[i] = -V()[i];
	return res;
}

template <class T> Matrix44<T> Matrix44<T>::operator*(const T k) const {
	Matrix44<T> res;
	for(int i = 0; i < 16; i++)
		res.V()[i] =V()[i] * k;
	return res;
}

template <class T> void Matrix44<T>::operator+=(const Matrix44 &m) {
	for(int i = 0; i < 16; i++)
		V()[i] += m.V()[i];
}
template <class T> void Matrix44<T>::operator-=(const Matrix44 &m) {
	for(int i = 0; i < 16; i++)
		V()[i] -= m.V()[i];
}
template <class T> void Matrix44<T>::operator*=( const Matrix44 & m ) {
	*this = *this *m;
}

template < class PointType , class T > void operator*=( std::vector<PointType> &vert, const Matrix44<T> & m ) {
	typename std::vector<PointType>::iterator ii;
	for(ii=vert.begin();ii!=vert.end();++ii)
		(*ii).P()=m * (*ii).P();
}

template <class T> void Matrix44<T>::operator*=( const T k ) {
	for(int i = 0; i < 16; i++)
		_a[i] *= k;
}

template <class T>
void Matrix44<T>::ToEulerAngles(T &alpha, T &beta, T &gamma)
{
	alpha = atan2(ElementAt(1,2), ElementAt(2,2));
	beta = asin(-ElementAt(0,2));
	gamma = atan2(ElementAt(0,1), ElementAt(0,0));
}

template <class T>
void Matrix44<T>::FromEulerAngles(T alpha, T beta, T gamma)
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

template <class T> void Matrix44<T>::SetZero() {
	memset((T *)_a, 0, 16 * sizeof(T));
}

template <class T> void Matrix44<T>::SetIdentity() {
	SetDiagonal(1);
}

template <class T> void Matrix44<T>::SetDiagonal(const T k) {
	SetZero();
	ElementAt(0, 0) = k;
	ElementAt(1, 1) = k;
	ElementAt(2, 2) = k;
	ElementAt(3, 3) = 1;
}

template <class T> Matrix44<T> &Matrix44<T>::SetScale(const Point3<T> &t) {
	SetScale(t[0], t[1], t[2]);
	return *this;
}
template <class T> Matrix44<T> &Matrix44<T>::SetScale(const T sx, const T sy, const T sz) {
	SetZero();
	ElementAt(0, 0) = sx;
	ElementAt(1, 1) = sy;
	ElementAt(2, 2) = sz;
	ElementAt(3, 3) = 1;
	return *this;
}

template <class T> Matrix44<T> &Matrix44<T>::SetTranslate(const Point3<T> &t) {
	SetTranslate(t[0], t[1], t[2]);
	return *this;
}
template <class T> Matrix44<T> &Matrix44<T>::SetTranslate(const T tx, const T ty, const T tz) {
	SetIdentity();
	ElementAt(0, 3) = tx;
	ElementAt(1, 3) = ty;
	ElementAt(2, 3) = tz;
	return *this;
}

template <class T> Matrix44<T> &Matrix44<T>::SetColumn(const unsigned int ii,const Point3<T> &t) {
	assert( ii < 4 );
	ElementAt(0, ii) = t.X();
	ElementAt(1, ii) = t.Y();
	ElementAt(2, ii) = t.Z();
	return *this;
}

template <class T> Matrix44<T> &Matrix44<T>::SetColumn(const unsigned int ii,const Point4<T> &t) {
	assert( ii < 4 );
	ElementAt(0, ii) = t[0];
	ElementAt(1, ii) = t[1];
	ElementAt(2, ii) = t[2];
	ElementAt(3, ii) = t[3];
	return *this;
}


template <class T> Matrix44<T> &Matrix44<T>::SetRotateDeg(T AngleDeg, const Point3<T> & axis) {
	return SetRotateRad(math::ToRad(AngleDeg),axis);
}

template <class T> Matrix44<T> &Matrix44<T>::SetRotateRad(T AngleRad, const Point3<T> & axis) {
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

/*
Given a non singular, non projective matrix (e.g. with the last row equal to [0,0,0,1] )
This procedure decompose it in a sequence of
- Scale,Shear,Rotation e Translation

- ScaleV and Tranv are obiviously scaling and translation.
- ShearV contains three scalars with, respectively,
      ShearXY, ShearXZ and ShearYZ
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
	if(!(M[3][0]==0 && M[3][1]==0 && M[3][2]==0 && M[3][3]==1) ) // the matrix is projective
		return false;
	if(math::Abs(M.Determinant())<1e-10) return false; // matrix should be at least invertible...

	// First Step recover the traslation
	TranV=M.GetColumn3(3);

	// Second Step Recover Scale and Shearing interleaved
	ScaleV[0]=Norm(M.GetColumn3(0));
	Point3<T> R[3];
	R[0]=M.GetColumn3(0);
	R[0].Normalize();

	ShearV[0]=R[0]*M.GetColumn3(1); // xy shearing
	R[1]= M.GetColumn3(1)-R[0]*ShearV[0];
	assert(math::Abs(R[1]*R[0])<1e-10);
	ScaleV[1]=Norm(R[1]);   // y scaling
	R[1]=R[1]/ScaleV[1];
	ShearV[0]=ShearV[0]/ScaleV[1];

	ShearV[1]=R[0]*M.GetColumn3(2); // xz shearing
	R[2]= M.GetColumn3(2)-R[0]*ShearV[1];
	assert(math::Abs(R[2]*R[0])<1e-10);

	R[2] = R[2]-R[1]*(R[2]*R[1]);
	assert(math::Abs(R[2]*R[1])<1e-10);
	assert(math::Abs(R[2]*R[0])<1e-10);

	ScaleV[2]=Norm(R[2]);
	ShearV[1]=ShearV[1]/ScaleV[2];
	R[2]=R[2]/ScaleV[2];
	assert(math::Abs(R[2]*R[1])<1e-10);
	assert(math::Abs(R[2]*R[0])<1e-10);

	ShearV[2]=R[1]*M.GetColumn3(2); // yz shearing
	ShearV[2]=ShearV[2]/ScaleV[2];
	int i,j;
	for(i=0;i<3;++i)
		for(j=0;j<3;++j)
			M[i][j]=R[j][i];

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
	beta=asin( M[0][2]);
	double cosbeta=cos(beta);
	if(math::Abs(cosbeta) > 1e-5)
	{
		alpha=asin(-M[1][2]/cosbeta);
		if((M[2][2]/cosbeta) < 0 ) alpha=M_PI-alpha;
		gamma=asin(-M[0][1]/cosbeta);
		if((M[0][0]/cosbeta)<0) gamma = M_PI-gamma;
	}
	else
	{
		alpha=asin(-M[1][0]);
		if(M[1][1]<0) alpha=M_PI-alpha;
		gamma=0;
	}

	RotV[0]=math::ToDeg(alpha);
	RotV[1]=math::ToDeg(beta);
	RotV[2]=math::ToDeg(gamma);

	return true;
}




template <class T> T Matrix44<T>::Determinant() const {
	Eigen::Matrix4d mm;
	this->ToEigenMatrix(mm);
	return mm.determinant();
}


template <class T> Point3<T> operator*(const Matrix44<T> &m, const Point3<T> &p) {
	T w;
	Point3<T> s;
	s[0] = m.ElementAt(0, 0)*p[0] + m.ElementAt(0, 1)*p[1] + m.ElementAt(0, 2)*p[2] + m.ElementAt(0, 3);
	s[1] = m.ElementAt(1, 0)*p[0] + m.ElementAt(1, 1)*p[1] + m.ElementAt(1, 2)*p[2] + m.ElementAt(1, 3);
	s[2] = m.ElementAt(2, 0)*p[0] + m.ElementAt(2, 1)*p[1] + m.ElementAt(2, 2)*p[2] + m.ElementAt(2, 3);
	w = m.ElementAt(3, 0)*p[0] + m.ElementAt(3, 1)*p[1] + m.ElementAt(3, 2)*p[2] + m.ElementAt(3, 3);
	if(w!= 0) s /= w;
	return s;
}

template <class T> Matrix44<T> &Transpose(Matrix44<T> &m) {
	for(int i = 1; i < 4; i++)
		for(int j = 0; j < i; j++) {
			std::swap(m.ElementAt(i, j), m.ElementAt(j, i));
		}
	return m;
}

template <class T> Matrix44<T> Inverse(const Matrix44<T> &m) {
	Eigen::Matrix4d mm,mmi;
	m.ToEigenMatrix(mm);
	mmi=mm.inverse();
	Matrix44<T> res;
	res.FromEigenMatrix(mmi);
	return res;
}

} //namespace
#endif


