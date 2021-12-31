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
#ifndef __VCGLIB_MATRIX33_H
#define __VCGLIB_MATRIX33_H

#include <stdio.h>
#include <vcg/math/matrix44.h>
#include <vcg/space/point3.h>
#include <vector>

namespace vcg {

template<class S>
/** @name Matrix33
    Class Matrix33.
    This is the class for definition of a matrix 3x3.
    @param S (Template Parameter) Specifies the ScalarType field.
*/
class Matrix33
{
public:
    typedef S ScalarType;

    /// Default constructor
    inline Matrix33() {}

    /// Copy constructor
    Matrix33( const Matrix33 & m )
    {
        for(int i=0;i<9;++i)
            a[i] = m.a[i];
    }

    /// create from array
    Matrix33( const S * v )
    {
        for(int i=0;i<9;++i) a[i] = v[i];
    }

    /// create from Matrix44 excluding row and column k
    Matrix33 (const Matrix44<S> & m, const int & k)
    {
        int i,j, r=0, c=0;
        for(i = 0; i< 4;++i)if(i!=k){c=0;
            for(j=0; j < 4;++j)if(j!=k)
            { (*this)[r][c] = m[i][j]; ++c;}
            ++r;
        }
    }

    template <class EigenMatrix33Type>
    void ToEigenMatrix(EigenMatrix33Type & m) const {
      for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
          m(i,j)=(*this)[i][j];
    }

    template <class EigenMatrix33Type>
    void FromEigenMatrix(const EigenMatrix33Type & m){
      for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
          (*this)[i][j]=m(i,j);
    }

    ///	Number of columns
    inline unsigned int ColumnsNumber() const
    {
        return 3;
    };

    /// Number of rows
    inline unsigned int RowsNumber() const
    {
        return 3;
    };

    /// Assignment operator
    Matrix33 & operator = ( const Matrix33 & m )
    {
        for(int i=0;i<9;++i)
            a[i] = m.a[i];
        return *this;
    }



    /// Operatore di indicizzazione
    inline S * operator [] ( const int i )
    {
        return a+i*3;
    }
    /// Operatore const di indicizzazione
    inline const S * operator [] ( const int i ) const
    {
        return a+i*3;
    }


    /// Modificatore somma per matrici 3x3
    Matrix33 & operator += ( const Matrix33  &m )
    {
        for(int i=0;i<9;++i)
            a[i] += m.a[i];
        return *this;
    }


    /// Modificatore sottrazione per matrici 3x3
    Matrix33 & operator -= ( const Matrix33 &m )
    {
        for(int i=0;i<9;++i)
            a[i] -= m.a[i];
        return *this;
    }

     /// Modificatore divisione per scalare
    Matrix33 & operator /= ( const S &s )
    {
        for(int i=0;i<9;++i)
            a[i] /= s;
        return *this;
    }


    /// Modificatore prodotto per matrice
    Matrix33 operator * ( const Matrix33< S> & t ) const
    {
        Matrix33<S> r;

        int i,j;
        for(i=0;i<3;++i)
            for(j=0;j<3;++j)
                    r[i][j] = (*this)[i][0]*t[0][j] + (*this)[i][1]*t[1][j] + (*this)[i][2]*t[2][j];

        return r;
    }

    /// Modificatore prodotto per matrice
    void operator *=( const Matrix33< S> & t )
    {
                Matrix33<S> r;
        int i,j;
        for(i=0;i<3;++i)
            for(j=0;j<3;++j)
                                        r[i][j] = (*this)[i][0]*t[0][j] + (*this)[i][1]*t[1][j] + (*this)[i][2]*t[2][j];
                for(i=0;i<9;++i) this->a[i] = r.a[i];
    }

    /// Modificatore prodotto per costante
    Matrix33 & operator *= ( const S t )
    {
        for(int i=0;i<9;++i)
            a[i] *= t;
        return *this;
    }

    /// Operatore prodotto per costante
    Matrix33 operator * ( const S t ) const
    {
        Matrix33<S> r;
        for(int i=0;i<9;++i)
            r.a[i] = a[i]* t;

        return r;
    }

    /// Operatore sottrazione per matrici 3x3
    Matrix33  operator - ( const Matrix33 &m ) const
    {
        Matrix33<S> r;
        for(int i=0;i<9;++i)
            r.a[i] = a[i] - m.a[i];

        return r;
    }
    /// Operatore sottrazione per matrici 3x3
    Matrix33  operator + ( const Matrix33 &m ) const
    {
        Matrix33<S> r;
        for(int i=0;i<9;++i)
            r.a[i] = a[i] + m.a[i];

        return r;
    }
    /** Operatore per il prodotto matrice-vettore.
        @param v A point in $R^{3}$
        @return Il vettore risultante in $R^{3}$
    */
    Point3<S> operator * ( const Point3<S> & v ) const
    {
        Point3<S> t;

        t[0] = a[0]*v[0] + a[1]*v[1] + a[2]*v[2];
        t[1] = a[3]*v[0] + a[4]*v[1] + a[5]*v[2];
        t[2] = a[6]*v[0] + a[7]*v[1] + a[8]*v[2];
        return t;
    }

    void OuterProduct(Point3<S> const &p0, Point3<S> const &p1) {
        Point3<S> row;
        row = p1*p0[0];
        a[0] = row[0];a[1] = row[1];a[2] = row[2];
        row = p1*p0[1];
        a[3] = row[0]; a[4] = row[1]; a[5] = row[2];
        row = p1*p0[2];
        a[6] = row[0];a[7] = row[1];a[8] = row[2];
    }

    Matrix33 & SetZero()	{
        for(int i=0;i<9;++i) a[i] =0;
        return (*this);
    }
    Matrix33 & SetIdentity()	{
        for(int i=0;i<9;++i) a[i] =0;
        a[0]=a[4]=a[8]=1.0;
        return (*this);
    }

    Matrix33 & SetRotateRad(S angle, const Point3<S> & axis )
    {
        S c = cos(angle);
        S s = sin(angle);
        S q = 1-c;
        Point3<S> t = axis;
        t.Normalize();
        a[0] = t[0]*t[0]*q + c;
        a[1] = t[0]*t[1]*q - t[2]*s;
        a[2] = t[0]*t[2]*q + t[1]*s;
        a[3] = t[1]*t[0]*q + t[2]*s;
        a[4] = t[1]*t[1]*q + c;
        a[5] = t[1]*t[2]*q - t[0]*s;
        a[6] = t[2]*t[0]*q -t[1]*s;
        a[7] = t[2]*t[1]*q +t[0]*s;
        a[8] = t[2]*t[2]*q +c;
        return (*this);
    }
    Matrix33 & SetRotateDeg(S angle, const Point3<S> & axis ){
        return SetRotateRad(math::ToRad(angle),axis);
    }

    /// Funzione per eseguire la trasposta della matrice
    Matrix33 & Transpose()
    {
        std::swap(a[1],a[3]);
        std::swap(a[2],a[6]);
        std::swap(a[5],a[7]);
        return *this;
    }

    // for the transistion to eigen
    Matrix33 transpose() const
    {
        Matrix33 res = *this;
        res.Transpose();
        return res;
    }

    void transposeInPlace() { this->Transpose(); }

    /// Funzione per costruire una matrice diagonale dati i tre elem.
    Matrix33 & SetDiagonal(S *v)
    {int i,j;
        for(i=0;i<3;i++)
      for(j=0;j<3;j++)
        if(i==j) (*this)[i][j] = v[i];
        else     (*this)[i][j] = 0;
    return *this;
    }


    /// Assegna l'n-simo vettore colonna
    void SetColumn(const int n, S* v){
        assert( (n>=0) && (n<3) );
        a[n]=v[0]; a[n+3]=v[1]; a[n+6]=v[2];
    };

    /// Assegna l'n-simo vettore riga
    void SetRow(const int n, S* v){
        assert( (n>=0) && (n<3) );
        int m=n*3;
        a[m]=v[0]; a[m+1]=v[1]; a[m+2]=v[2];
    };

    /// Assegna l'n-simo vettore colonna
    void SetColumn(const int n, const Point3<S> v){
        assert( (n>=0) && (n<3) );
        a[n]=v[0]; a[n+3]=v[1]; a[n+6]=v[2];
    };

    /// Assegna l'n-simo vettore riga
    void SetRow(const int n, const Point3<S> v){
        assert( (n>=0) && (n<3) );
        int m=n*3;
        a[m]=v[0]; a[m+1]=v[1]; a[m+2]=v[2];
    };

    /// Restituisce l'n-simo vettore colonna
    Point3<S> GetColumn(const int n) const {
        assert( (n>=0) && (n<3) );
        Point3<S> t;
        t[0]=a[n]; t[1]=a[n+3]; t[2]=a[n+6];
        return t;
    };

    /// Restituisce l'n-simo vettore riga
    Point3<S> GetRow(const int n) const {
        assert( (n>=0) && (n<3) );
        Point3<S> t;
        int m=n*3;
        t[0]=a[m]; t[1]=a[m+1]; t[2]=a[m+2];
        return t;
    };



    /// Funzione per il calcolo del determinante
    S Determinant() const
    {
        return a[0]*(a[4]*a[8]-a[5]*a[7]) -
                 a[1]*(a[3]*a[8]-a[5]*a[6]) +
                     a[2]*(a[3]*a[7]-a[4]*a[6]) ;
    }

// return the Trace of the matrix i.e. the sum of the diagonal elements
S Trace() const
{
    return a[0]+a[4]+a[8];
}

/*
compute the matrix generated by the product of a * b^T
*/
void ExternalProduct(const Point3<S> &a, const Point3<S> &b)
{
    for(int i=0;i<3;++i)
        for(int j=0;j<3;++j)
             (*this)[i][j] = a[i]*b[j];
}

/* Compute the Frobenius Norm of the Matrix
*/
ScalarType Norm()
{
    ScalarType SQsum=0;
    for(int i=0;i<3;++i)
        for(int j=0;j<3;++j)
             SQsum += a[i]*a[i];
    return (math::Sqrt(SQsum));
}


/*
It compute the  covariance matrix of a set of 3d points. Returns the barycenter
*/
template <class STLPOINTCONTAINER >
void Covariance(const STLPOINTCONTAINER &points, Point3<S> &bp) {
    assert(!points.empty());
    typedef typename  STLPOINTCONTAINER::const_iterator PointIte;
    // first cycle: compute the barycenter
    bp.SetZero();
    for( PointIte pi = points.begin(); pi != points.end(); ++pi) bp+= (*pi);
    bp/=points.size();
    // second cycle: compute the covariance matrix
    this->SetZero();
    vcg::Matrix33<ScalarType> A;
    for( PointIte pi = points.begin(); pi != points.end(); ++pi) {
             Point3<S> p = (*pi)-bp;
             A.OuterProduct(p,p);
            (*this)+= A;
    }
}



/*
It compute the cross covariance matrix of two set of 3d points P and X;
it returns also the barycenters of P and X.
fonte:

Besl, McKay
A method for registration o f 3d Shapes
IEEE TPAMI Vol 14, No 2 1992

*/
template <class STLPOINTCONTAINER >
void CrossCovariance(const STLPOINTCONTAINER &P, const STLPOINTCONTAINER &X,
                                         Point3<S> &bp, Point3<S> &bx)
{
    SetZero();
    assert(P.size()==X.size());
    bx.SetZero();
    bp.SetZero();
    Matrix33<S> tmp;
    typename std::vector <Point3<S> >::const_iterator pi,xi;
    for(pi=P.begin(),xi=X.begin();pi!=P.end();++pi,++xi){
        bp+=*pi;
        bx+=*xi;
        tmp.ExternalProduct(*pi,*xi);
        (*this)+=tmp;
    }
    bp/=P.size();
    bx/=X.size();
    (*this)/=P.size();
    tmp.ExternalProduct(bp,bx);
    (*this)-=tmp;
}

template <class STLPOINTCONTAINER, class STLREALCONTAINER>
void WeightedCrossCovariance(const STLREALCONTAINER &  weights,
                             const STLPOINTCONTAINER &P,
                             const STLPOINTCONTAINER &X,
                             Point3<S> &bp,
                             Point3<S> &bx)
{
    SetZero();
    assert(P.size()==X.size());
    bx.SetZero();
    bp.SetZero();
    Matrix33<S> tmp;
    typename std::vector <Point3<S> >::const_iterator pi,xi;
    typename STLREALCONTAINER::const_iterator pw;

    for(pi=P.begin(),xi=X.begin();pi!=P.end();++pi,++xi){
        bp+=(*pi);
        bx+=(*xi);
    }
    bp/=P.size();
    bx/=X.size();

    for(pi=P.begin(),xi=X.begin(),pw = weights.begin();pi!=P.end();++pi,++xi,++pw){

        tmp.ExternalProduct(((*pi)-(bp)),((*xi)-(bp)));

        (*this)+=tmp*(*pw);
    }
}

private:
    S a[9];
};

///return the tranformation matrix to transform
///to the frame specified by the three vectors
template <class S>
vcg::Matrix33<S> TransformationMatrix(const vcg::Point3<S> dirX,
                                        const vcg::Point3<S> dirY,
                                        const vcg::Point3<S> dirZ)
{
    vcg::Matrix33<S> Trans;

    ///it must have right orientation cause of normal
    Trans[0][0]=dirX[0];
    Trans[0][1]=dirX[1];
    Trans[0][2]=dirX[2];
    Trans[1][0]=dirY[0];
    Trans[1][1]=dirY[1];
    Trans[1][2]=dirY[2];
    Trans[2][0]=dirZ[0];
    Trans[2][1]=dirZ[1];
    Trans[2][2]=dirZ[2];

    /////then find the inverse
    return (Trans);
}
template <class S>
Matrix33<S> Inverse(const Matrix33<S>&m)
    {
  Eigen::Matrix3d mm,mmi;
  m.ToEigenMatrix(mm);
  mmi=mm.inverse();
  Matrix33<S> res;
  res.FromEigenMatrix(mmi);
  return res;
}

///given 2 vector centered into origin calculate the rotation matrix from first to the second
template <class S>
Matrix33<S> RotationMatrix(vcg::Point3<S> v0,vcg::Point3<S> v1,bool normalized=true)
    {
        typedef typename vcg::Point3<S> CoordType;
        Matrix33<S> rotM;
        const S epsilon=0.00001;
        if (!normalized)
        {
            v0.Normalize();
            v1.Normalize();
        }
        S dot=(v0*v1);
        ///control if there is no rotation
        if (dot>((S)1-epsilon))
        {
            rotM.SetIdentity();
            return rotM;
        }

        ///find the axis of rotation
        CoordType axis;
        axis=v0^v1;
        axis.Normalize();

        ///construct rotation matrix
        S u=axis.X();
        S v=axis.Y();
        S w=axis.Z();
        S phi=acos(dot);
        S rcos = cos(phi);
        S rsin = sin(phi);

        rotM[0][0] =      rcos + u*u*(1-rcos);
        rotM[1][0] =  w * rsin + v*u*(1-rcos);
        rotM[2][0] = -v * rsin + w*u*(1-rcos);
        rotM[0][1] = -w * rsin + u*v*(1-rcos);
        rotM[1][1] =      rcos + v*v*(1-rcos);
        rotM[2][1] =  u * rsin + w*v*(1-rcos);
        rotM[0][2] =  v * rsin + u*w*(1-rcos);
        rotM[1][2] = -u * rsin + v*w*(1-rcos);
        rotM[2][2] =      rcos + w*w*(1-rcos);

        return rotM;
    }

///return the rotation matrix along axis
template <class S>
Matrix33<S> RotationMatrix(const vcg::Point3<S> &axis,
                           const float &angleRad)
    {
        vcg::Matrix44<S> matr44;
        vcg::Matrix33<S> matr33;
        matr44.SetRotate(angleRad,axis);
        for (int i=0;i<3;i++)
            for (int j=0;j<3;j++)
                matr33[i][j]=matr44[i][j];
        return matr33;
    }

/// return a random rotation matrix, from the paper:
/// Fast Random Rotation Matrices, James Arvo
/// Graphics Gems III pp. 117-120
template <class S>
 Matrix33<S> RandomRotation(){
    S x1,x2,x3;
    Matrix33<S> R,H,M,vv;
    Point3<S> v;
    R.SetIdentity();
    H.SetIdentity();
    x1 = rand()/S(RAND_MAX);
    x2 = rand()/S(RAND_MAX);
    x3 = rand()/S(RAND_MAX);

    R[0][0] =		cos(S(2)*M_PI*x1);
    R[0][1] =		sin(S(2)*M_PI*x1);
    R[1][0] = -	R[0][1];
    R[1][1] =		R[0][0];

    v[0] = cos(2.0 * M_PI * x2)*sqrt(x3);
    v[1] = sin(2.0 * M_PI * x2)*sqrt(x3);
    v[2] = sqrt(1-x3);

    vv.OuterProduct(v,v);
    H -= vv*S(2);
    M = H*R*S(-1);
    return M;
}

///
typedef Matrix33<short>  Matrix33s;
typedef Matrix33<int>	 Matrix33i;
typedef Matrix33<float>  Matrix33f;
typedef Matrix33<double> Matrix33d;

} // end of namespace

#endif
