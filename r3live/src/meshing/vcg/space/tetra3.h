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
/****************************************************************************
  History

$Log: not supported by cvs2svn $
Revision 1.15  2007/07/31 12:35:42  ganovelli
added ScalarType to tetra3

Revision 1.14  2006/07/06 12:39:51  ganovelli
adde barycenter()

Revision 1.13  2006/06/06 14:35:31  zifnab1974
Changes for compilation on linux AMD64. Some remarks: Linux filenames are case-sensitive. _fileno and _filelength do not exist on linux

Revision 1.12  2006/03/01 15:59:34  pietroni
added InterpolationParameters function

Revision 1.11  2005/12/12 11:15:26  ganovelli
modifications to compile with gcc

Revision 1.10  2005/11/29 16:20:33  pietroni
added IsInside() function

Revision 1.9  2004/10/13 12:45:51  cignoni
Better Doxygen documentation

Revision 1.8  2004/09/01 12:21:11  pietroni
minor changes to comply gcc compiler (typename's )

Revision 1.7  2004/07/09 10:08:21  ganovelli
ComputeVOlume moved outside the class and other
 minor corrections

Revision 1.6  2004/06/25 18:17:03  ganovelli
minor changes

Revision 1.5  2004/05/13 12:51:00  turini
Changed SolidAngle : table EV in table EofV
Changed DiedralAngle : tables FE and FV in tables FofE and FofV

Revision 1.4  2004/05/13 08:42:36  pietroni
the relation between entities functions are in tetra class (don't neeed template argoument)

Revision 1.3  2004/04/28 16:31:17  turini
Changed :
in SolidAngle(vind) :
double da0=DiedralAngle(EV(vind,0));
double da1=DiedralAngle(EV(vind,1));
double da2=DiedralAngle(EV(vind,2));
in
double da0=DiedralAngle(EofV(vind,0));
double da1=DiedralAngle(EofV(vind,1));
double da2=DiedralAngle(EofV(vind,2));

Changed :
in DiedralAngle(edgeind) :
int f1=FE(edgeind,0);
int f2=FE(edgeind,1);
in
int f1=FofE(edgeind,0);
int f2=FofE(edgeind,1);

Changed :
in DiedralAngle(edgeind) :
Point3d p0=FV(f1,0)->P();
Point3d p1=FV(f1,1)->P();
Point3d p2=FV(f1,2)->P();
in
Point3d p0=_v[FofV(f1,0)];
Point3d p1=_v[FofV(f1,1)];
Point3d p2=_v[FofV(f1,2)];

Changed :
in DiedralAngle(edgeind) :
p0=FV(f2,0)->P();
p1=FV(f2,1)->P();
p2=FV(f2,2)->P();
in
p0=_v[FofV(f2,0)];
p1=_v[FofV(f2,1)];
p2=_v[FofV(f2,2)];

Revision 1.2  2004/04/28 11:37:15  pietroni
*** empty log message ***

Revision 1.1  2004/04/22 13:19:12  ganovelli
first version

Revision 1.2  2004/04/20 16:26:48  pietroni
*** empty log message ***

Revision 1.1  2004/04/15 08:54:20  pietroni
*** empty log message ***

Revision 1.1  2004/04/08 01:13:31  pietroni
Initial commit


****************************************************************************/
#ifndef __VCG_TETRA3
#define __VCG_TETRA3

#include <vcg/space/point3.h>
#include <vcg/math/matrix44.h>
#include <vcg/math/matrix33.h>

namespace vcg {
/** \addtogroup space */
/*@{*/
/** 
		Templated class for storing a generic tetrahedron


 */
class Tetra
{
public:

 
//Tatrahedron Functions to retrieve information about relation between faces of tetrahedron(faces,adges,vertices).

  static int VofE(const int &indexE,const int &indexV)
  {	assert ((indexE<6)&&(indexV<2));
   static int edgevert[6][2] ={{0,1},
					{0,2},
					{0,3},
					{1,2},
					{1,3},
					{2,3}};
   return (edgevert[indexE][indexV]);
  }

	static int VofF(const int &indexF,const int &indexV)
	{	assert ((indexF<4)&&(indexV<3));
    static int facevert[4][3]={{0,1,2},
					{0,3,1},
					{0,2,3},
					{1,3,2}};
		return (facevert[indexF][indexV]);
	}

  static int EofV(const int &indexV,const int &indexE) 
	{	
    assert ((indexE<3)&&(indexV<4));
		static int vertedge[4][3]={{0,1,2},
											{0,3,4},
											{5,1,3},
											{4,5,2}};
		return vertedge[indexV][indexE];
	}

 static int EofF(const int &indexF,const int &indexE) 
	{	assert ((indexF<4)&&(indexE<3));
    static int faceedge[4][3]={{0,3,1},
					{2,4,0},
					{1,5,2},
					{4,5,3}
					};
		return faceedge [indexF][indexE];
	}

	static int FofV(const int &indexV,const int &indexF)
	{	
    assert ((indexV<4)&&(indexF<3));
    static int vertface[4][3]={{0,1,2},
					{0,3,1},
					{0,2,3},
					{1,3,2}};
		return vertface[indexV][indexF];
	}
  
  static int FofE(const int &indexE,const int &indexSide) 
	{	assert ((indexE<6)&&(indexSide<2));
    static int edgeface[6][2]={{0,1},
					{0,2},
					{1,2},
					{0,3},
					{1,3},
					{2,3}};
		return edgeface [indexE][indexSide];
	}

static int VofEE(const int &indexE0,const int &indexE1)
{
  assert ((indexE0<6)&&(indexE0>=0));
  assert ((indexE1<6)&&(indexE1>=0));
  static int edgesvert[6][6]={{-1,0,0,1,1,-1},
  {0,-1,0,2,-1,2},
  {0,0,-1,-1,3,3},
  {1,2,-1,-1,1,2},
  {1,-1,3,1,-1,3},
  {-1,2,3,2,3,-1}};
return (edgesvert[indexE0][indexE1]);
}

static int VofFFF(const int &indexF0,const int &indexF1,const int &indexF2)
{
  assert ((indexF0<4)&&(indexF0>=0));
  assert ((indexF1<4)&&(indexF1>=0));
  assert ((indexF2<4)&&(indexF2>=0));
  static int facesvert[4][4][4]={
		{//0
			{-1,-1,-1,-1},{-1,-1,0,1},{-1,0,-1,2},{-1,1,2,-1}
		},
		{//1
			{-1,-1,0,1},{-1,-1,-1,-1},{0,-1,-1,3},{1,-1,3,-1}
		},
		{//2
			{-1,0,-1,2},{0,-1,-1,3},{-1,-1,-1,-1},{2,3,-1,-1}
		},
		{//3
			{-1,1,2,-1},{1,-1,3,-1},{2,3,-1,-1},{-1,-1,-1,-1}
		}
	};
  return facesvert[indexF0][indexF1][indexF2];
}

static int EofFF(const int &indexF0,const int &indexF1)
{
  assert ((indexF0<4)&&(indexF0>=0));
  assert ((indexF1<4)&&(indexF1>=0));
  static	int facesedge[4][4]={{-1,  0,  1,  3},
						  { 0, -1,  2,  4},
						  { 1,  2, -1,  5},
						  { 3,  4,  5, -1}};
  return (facesedge[indexF0][indexF1]);
}

static int EofVV(const int &indexV0,const int &indexV1) 
{
      assert ((indexV0<4)&&(indexV0>=0));
      assert ((indexV1<4)&&(indexV1>=0));
      static	int verticesedge[4][4]={{-1,  0,  1,  2},
						  { 0, -1,  3,  4},
						  { 1,  3, -1,  5},
						  { 2,  4,  5, -1}};
			
			return verticesedge[indexV0][indexV1];
}

static int FofVVV(const int &indexV0,const int &indexV1,const int &indexV2)
{

assert ((indexV0<4)&&(indexV0>=0));
assert ((indexV1<4)&&(indexV1>=0));
assert ((indexV2<4)&&(indexV2>=0));

static int verticesface[4][4][4]={
		{//0
			{-1,-1,-1,-1},{-1,-1,0,1},{-1,0,-1,2},{-1,1,2,-1}
		},
		{//1
			{-1,-1,0,1},{-1,-1,-1,-1},{0,-1,-1,3},{1,-1,3,-1}
		},
		{//2
			{-1,0,-1,2},{0,-1,-1,3},{-1,-1,-1,-1},{2,3,-1,-1}
		},
		{//3
			{-1,1,2,-1},{1,-1,3,-1},{2,3,-1,-1},{-1,-1,-1,-1}
		}
	};
return verticesface[indexV0][indexV1][indexV2];
}

static int FofEE(const int &indexE0,const int &indexE1)
{
  assert ((indexE0<6)&&(indexE0>=0));
  assert ((indexE1<6)&&(indexE1>=0));
  static	int edgesface[6][6]={{-1,0,1,0,1,-1},
    {0,-1,2,0,-1,2},
    {1,2,-1,-1,1,2},
    {0,0,-1,-1,3,3},
    {1,-1,1,3,-1,3},
    {-1,2,2,3,3,-1}};
						  
			return edgesface[indexE0][indexE1];
}
};

/** 
		Templated class for storing a generic tetrahedron in a 3D space.
    Note the relation with the Face class of TetraMesh complex, both classes provide the P(i) access functions to their points and therefore they share the algorithms on it (e.g. area, normal etc...)
 */
 template <class ScalarType> 
 class Tetra3:public Tetra
{
public:
  typedef  Point3< ScalarType > CoordType;
  //typedef typename ScalarType ScalarType;

/*********************************************
  
**/

private:
	/// Vector of the 4 points that defines the tetrahedron
	CoordType _v[4];

public:

/// Shortcut per accedere ai punti delle facce
	inline CoordType & P( const int j ) { return _v[j];}
	inline CoordType const & cP( const int j )const { return _v[j];}

	inline CoordType & P0( const int j ) { return _v[j];}
	inline CoordType & P1( const int j ) { return _v[(j+1)%4];}
	inline CoordType & P2( const int j ) { return _v[(j+2)%4];}
  inline CoordType & P3( const int j ) { return _v[(j+3)%4];}

	inline const CoordType &  P0( const int j ) const { return _v[j];}
	inline const CoordType &  P1( const int j ) const { return _v[(j+1)%4];}
	inline const CoordType &  P2( const int j ) const { return _v[(j+2)%4];}
  inline const CoordType &  P3( const int j ) const { return _v[(j+3)%4];}

	inline const CoordType & cP0( const int j ) const { return _v[j];}
	inline const CoordType & cP1( const int j ) const { return _v[(j+1)%4];}
	inline const CoordType & cP2( const int j ) const { return _v[(j+2)%4];}
  inline const CoordType & cP3( const int j ) const { return _v[(j+3)%4];}

/// compute and return the barycenter of a tetrahedron
 CoordType ComputeBarycenter()
	{	
			return((_v[0] + _v[1] + _v[2]+ _v[3])/4);
	}

/// compute and return the solid angle on a vertex
double SolidAngle(int vind)
	{	
		double da0=DiedralAngle(EofV(vind,0));
		double da1=DiedralAngle(EofV(vind,1));
		double da2=DiedralAngle(EofV(vind,2));

			return((da0 + da1 + da2)- M_PI);
	}

/// compute and return the diadedral angle on an edge
	double DiedralAngle(int edgeind)
  {
		int f1=FofE(edgeind,0);
		int f2=FofE(edgeind,1);
		CoordType p0=_v[FofV(f1,0)];
		CoordType p1=_v[FofV(f1,1)];
		CoordType p2=_v[FofV(f1,2)];
		CoordType norm1=((p1-p0)^(p2-p0));
		p0=_v[FofV(f2,0)];
		p1=_v[FofV(f2,1)];
		p2=_v[FofV(f2,2)];
		CoordType norm2=((p1-p0)^(p2-p0));
		norm1.Normalize();
		norm2.Normalize();
	 return (M_PI-acos(double(norm1*norm2)));
	}

/// compute and return the aspect ratio of the tetrahedron 
ScalarType ComputeAspectRatio()
	{	
		double a0=SolidAngle(0);
		double a1=SolidAngle(1);
		double a2=SolidAngle(2);
		double a3=SolidAngle(3);
		return (ScalarType)math::Min(a0,math::Min(a1,math::Min(a2,a3)));
	}

	///return true of p is inside tetrahedron's volume
	bool IsInside(const CoordType &p)
	{
		//bb control
		vcg::Box3<typename CoordType::ScalarType> bb;
		for (int i=0;i<4;i++)
			bb.Add(_v[i]);

		if (!bb.IsIn(p))
			return false;

		vcg::Matrix44<ScalarType> M0;
		vcg::Matrix44<ScalarType> M1;
		vcg::Matrix44<ScalarType> M2;
		vcg::Matrix44<ScalarType> M3;
		vcg::Matrix44<ScalarType> M4;

		CoordType p1=_v[0];
		CoordType p2=_v[1];
		CoordType p3=_v[2];
		CoordType p4=_v[3];

		M0[0][0]=p1.V(0);
		M0[0][1]=p1.V(1);
		M0[0][2]=p1.V(2);
		M0[1][0]=p2.V(0);
		M0[1][1]=p2.V(1);
		M0[1][2]=p2.V(2);
		M0[2][0]=p3.V(0);
		M0[2][1]=p3.V(1);
		M0[2][2]=p3.V(2);
		M0[3][0]=p4.V(0);
		M0[3][1]=p4.V(1);
		M0[3][2]=p4.V(2);
		M0[0][3]=1;
		M0[1][3]=1;
		M0[2][3]=1;
		M0[3][3]=1;

		M1=M0;
		M1[0][0]=p.V(0);
		M1[0][1]=p.V(1);
		M1[0][2]=p.V(2);

		M2=M0;
		M2[1][0]=p.V(0);
		M2[1][1]=p.V(1);
		M2[1][2]=p.V(2);

		M3=M0;
		M3[2][0]=p.V(0);
		M3[2][1]=p.V(1);
		M3[2][2]=p.V(2);

		M4=M0;
		M4[3][0]=p.V(0);
		M4[3][1]=p.V(1);
		M4[3][2]=p.V(2);

		ScalarType d0=M0.Determinant();
		ScalarType d1=M1.Determinant();
		ScalarType d2=M2.Determinant();
		ScalarType d3=M3.Determinant();
		ScalarType d4=M4.Determinant();

		// all determinant must have same sign
		return (((d0>0)&&(d1>0)&&(d2>0)&&(d3>0)&&(d4>0))||((d0<0)&&(d1<0)&&(d2<0)&&(d3<0)&&(d4<0)));
	}

	void InterpolationParameters(const CoordType & bq, ScalarType &a, ScalarType &b, ScalarType &c ,ScalarType &d)
	{
		const ScalarType EPSILON = ScalarType(0.000001);
		Matrix33<ScalarType> M;
		
		CoordType v0=P(0)-P(2);
		CoordType v1=P(1)-P(2);
		CoordType v2=P(3)-P(2);
		CoordType v3=bq-P(2);

		M[0][0]=v0.X();
		M[1][0]=v0.Y();
		M[2][0]=v0.Z();

		M[0][1]=v1.X();
		M[1][1]=v1.Y();
		M[2][1]=v1.Z();

		M[0][2]=v2.X();
		M[1][2]=v2.Y();
		M[2][2]=v2.Z();

		Matrix33<ScalarType> inv_M =vcg::Inverse<ScalarType>(M);

		CoordType Barycentric=inv_M*v3;

		a=Barycentric.V(0);
		b=Barycentric.V(1);
		c=Barycentric.V(2);
		d=1-(a+b+c);

	}

}; //end Class

// compute the barycenter
template<class ScalarType>
Point3<ScalarType> Barycenter(const Tetra3<ScalarType> &t) 
{
	return ((t.cP(0)+t.cP(1)+t.cP(2)+t.cP(3))/(ScalarType) 4.0);
}

// compute and return the volume of a tetrahedron
 template<class TetraType>
typename TetraType::ScalarType ComputeVolume( const TetraType &  t){
	return (typename TetraType::ScalarType)((( t.cP(2)-t.cP(0))^(t.cP(1)-t.cP(0) ))*(t.cP(3)-t.cP(0))/6.0);
}
		
/// Returns the normal to the face face of the tetrahedron t
template<class TetraType>
Point3<typename TetraType::ScalarType> Normal( const TetraType &t,const int &face)
{
  return(((t.cP(Tetra::VofF(face,1))-t.cP(Tetra::VofF(face,0)))^(t.cP(Tetra::VofF(face,2))-t.cP(Tetra::VofF(face,0)))).Normalize());
}
/*@}*/
}	 // end namespace


#endif

