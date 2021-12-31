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
Revision 1.14  2006/12/18 16:02:57  matteodelle
minor eroor correction on variable names

Revision 1.13  2006/12/18 14:28:07  matteodelle
*** empty log message ***

Revision 1.12  2006/12/18 09:46:39  callieri
camera+shot revamp: changed field names to something with more sense, cleaning of various functions, correction of minor bugs/incongruences, removal of the infamous reference in shot.

Revision 1.11  2006/01/10 12:22:34  spinelli
add namespace vcg::

Revision 1.10  2005/10/24 14:42:57  spinelli
add namespace vcg:: to GetFrustum(...)

Revision 1.9  2005/06/29 15:02:29  spinelli
aggiunto:
- static void CavalieriProj( .. )
- static void IsometricProj( .. )

modificato:
- static void TransformGL( .. )
- static void SetSubView( .. )

Revision 1.8  2005/02/22 10:57:05  tommyfranken
corrected some syntax errors in GetFrustum

Revision 1.7  2005/02/21 18:11:47  ganovelli
GetFrustum moved from gl/camera to math/camera.h

Revision 1.6  2004/12/16 14:41:36  ricciodimare
*** empty log message ***

Revision 1.5  2004/12/16 11:08:35  ricciodimare
Cambiato il nome del costruttore era rimasto quello vecchio... e tolti alcune righe di codice commentate

Revision 1.4  2004/12/15 18:45:06  tommyfranken
*** empty log message ***

Revision 1.3  2004/11/03 09:38:21  ganovelli
added SetSubView, some comment and put the class back(!)

Revision 1.2  2004/10/05 19:04:44  ganovelli
changed from classes to functions

Revision 1.1  2004/09/15 22:59:13  ganovelli
creation

****************************************************************************/


#ifndef __GL_CAMERA
#define __GL_CAMERA
// VCG
#include <vcg/math/camera.h>

// opengl
#include <GL/glew.h>

template <class CameraType>
struct GlCamera{

	typedef typename CameraType::ScalarType ScalarType;
	typedef typename CameraType::ScalarType S;


/// returns the OpenGL 4x4 PROJECTION matrix that describes the camera (intrinsics)
static vcg::Matrix44<ScalarType>
MatrixGL(vcg::Camera<S> & cam, vcg::Matrix44<S> &m)
{
	glPushAttrib(GL_TRANSFORM_BIT);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	TransformGL(cam);
	glGetv(GL_PROJECTION_MATRIX,&m[0][0]);
	glPopMatrix();
	glPopAttrib();
	return m;
}

/// set the OpenGL PROJECTION matrix for the Cavalieri projection
static void SetGLCavalieriProj(float x1, float x2, float y1, float y2, float z1, float z2)
{
	GLfloat cavalieri[16];

	cavalieri[0]  = 2.0f/(x2-x1);                   cavalieri[4] = 0;            
	cavalieri[8]  = (0.707106f * -2.0f)/(x2-x1);    cavalieri[12] = (x2+x1)/(x2-x1);
	cavalieri[1]  = 0;                              cavalieri[5] = 2.0/(y2-y1);    
	cavalieri[9]  = (0.707106f * -2.0f)/(y2-y1);    cavalieri[13] = (y2+y1)/(y2-y1);
	cavalieri[2]  = 0;                              cavalieri[6] = 0;            
	cavalieri[10] = -2.0f/(z2-z1);                  cavalieri[14] = (z2+z1)/(z2-z1);
	cavalieri[3]  = 0;                              cavalieri[7] = 0;            
	cavalieri[11] = 0;                              cavalieri[15] = 1.0f;

	glLoadMatrixf(cavalieri);
}

/// set the OpenGL PROJECTION matrix for the Isometric projection
static void SetGLIsometricProj(float x1, float x2, float y1, float y2, float z1, float z2)
{
	GLfloat isometric[16];

	isometric[0]  = 1.6f/(x2-x1);     isometric[4]  = 0;            
	isometric[8]  = -1.6f/(x2-x1);    isometric[12] = (x2+x1)/(x2-x1);
	isometric[1]  = -1.0f/(y2-y1);    isometric[5]  = 2.0f/(y2-y1);    
	isometric[9]  = -1.0f/(y2-y1);    isometric[13] = (y2+y1)/(y2-y1);
	isometric[2]  = 0;                isometric[6]  = 0;            
	isometric[10] = -2.0f/(z2-z1);    isometric[14] = (z2+z1)/(z2-z1);
	isometric[3]  = 0;                isometric[7]  = 0;            
	isometric[11] = 0;                isometric[15] = 1.0f;

	glLoadMatrixf(isometric);
}

/// get OpenGL-like frustum from a vcg camera (intrinsics)
static void GetFrustum(vcg::Camera<S> & intrinsics, S & sx,S & dx,S & bt,S & tp,S & f)
{
	intrinsics.GetFrustum(sx,dx,bt,tp,f);
}

/// set the OpenGL PROJECTION matrix to match the camera (intrinsics). requires near and far plane
static void TransformGL(vcg::Camera<S> & camera, S nearDist, S farDist ) 
{
	S sx,dx,bt,tp,nr;
	camera.GetFrustum(sx,dx,bt,tp,nr);

  if(camera.cameraType == CameraType::PERSPECTIVE) {
    S ratio = nearDist/nr;
    sx *= ratio;
    dx *= ratio;
    bt *= ratio;
    tp *= ratio;
  }

	assert(glGetError()==0);
	
	switch(camera.cameraType) 
	{
   case CameraType::PERSPECTIVE: glFrustum(sx,dx,bt,tp,nearDist,farDist);	break;
   case CameraType::ORTHO:       glOrtho(sx,dx,bt,tp,nearDist,farDist); break;
   case CameraType::ISOMETRIC:   SetGLIsometricProj(sx,dx,bt,tp,nearDist,farDist); 	break;
   case CameraType::CAVALIERI:   SetGLCavalieriProj(sx,dx,bt,tp,nearDist,farDist); 	break;
	}
       
	assert(glGetError()==0);
};


static void GetViewSize(vcg::Camera<S> & camera, S &width, S &height) {
	S sx,dx,bt,tp,nr,fr;
	GetFrustum(camera,sx,dx,bt,tp,nr,fr);	
	width = dx-sx;	//right - left = width
	height = tp-bt;  //top - bottom = height
};


static void SetSubView(vcg::Camera<S> & camera,vcg::Point2<S> p0,S nearDist, S farDist,vcg::Point2<S> p1){
	//typedef typename CameraType::ScalarType S;
	S sx,dx,bt,tp,f;
	GetFrustum(camera,sx,dx,bt,tp,f);	
	S width = dx-sx;	//right - left = width
	S height = tp-bt;  //top - bottom = height
	/*glFrustum(
				width* p0[0]+ sx, width* p1[0]+ sx,	
				height* p0[1]+ bt, height* p1[1]+ bt,
				nr,fr);*/

	

	switch(camera.cameraType) 
	{
   case CameraType::PERSPECTIVE: glFrustum(	width* p0[0]+ sx, width* p1[0]+ sx,		height* p0[1]+ bt, height* p1[1]+bt,nearDist,farDist);	break;
   case CameraType::ORTHO:       glOrtho(width* p0[0]+sx, width* p1[0]+sx,			height* p0[1]+ bt, height* p1[1]+bt,nearDist,farDist); break;
	 //case vcg::ISOMETRIC:   IsometricProj(dx-width* p1[0], dx-width* p0[0],		tp-height* p1[1], tp-height* p0[1],nearDist,farDist);	break;
	 //case vcg::CAVALIERI:   CavalieriProj(dx-width* p1[0], dx-width* p0[0],		tp-height* p1[1], tp-height* p0[1],nearDist,farDist);	break;
	}


	assert(glGetError()==0);
};
};
#endif






//private:
//	
//	static inline S SQRT( S x) { return sqrt(fabs(x)); }
//	static inline S CBRT ( S x )
//	{
//		if (x == 0) return 0;
//		else if (x > 0) return pow (x, 1.0 / 3.0);
//		else return -pow (-x, 1.0 / 3.0);
//	}
//	static inline void SINCOS( S x, S & s, S & c)
//	{
//		s=sin(x);
//		c=cos(x);
//	}
//	static inline void SINCOSd( double x, double & s, double & c)
//	{
//		s=sin(x);
//		c=cos(x);
//	}
//	static inline S CUB( S x ) { return x*x*x; }
//	static inline S SQR( S x ) { return x*x; }
//
//public:
//	void undistorted_to_distorted_sensor_coord (S Xu, S Yu, S & Xd, S & Yd) const
//	{
//		const S SQRT3 = S(1.732050807568877293527446341505872366943);
//		S Ru,Rd,lambda,c,d,Q,R,D,S,T,sinT,cosT;
//
//		if((Xu==0 && Yu==0) || k[0] == 0)
//		{
//			Xd = Xu;
//			Yd = Yu;
//			return;
//		}
//
//		Ru = hypot (Xu, Yu);	/* SQRT(Xu*Xu+Yu*Yu) */
//		c = 1 / k[0];
//		d = -c * Ru;
//
//		Q = c / 3;
//		R = -d / 2;
//		D = CUB (Q) + SQR (R);
//
//		if (D >= 0)		/* one real root */
//		{
//			D = SQRT (D);
//			S = CBRT (R + D);
//			T = CBRT (R - D);
//			Rd = S + T;
//
//			if (Rd < 0)
//				Rd = SQRT (-1 / (3 * k[0]));	
//		}
//		else			/* three real roots */
//		{
//			D = SQRT (-D);
//			S = CBRT (hypot (R, D));
//			T = atan2 (D, R) / 3;
//			SINCOS (T, sinT, cosT);
//
//			/* the larger positive root is    2*S*cos(T)                   */
//			/* the smaller positive root is   -S*cos(T) + SQRT(3)*S*sin(T) */
//			/* the negative root is           -S*cos(T) - SQRT(3)*S*sin(T) */
//			Rd = -S * cosT + SQRT3 * S * sinT;	/* use the smaller positive root */
//		}
//
//		lambda = Rd / Ru;
//
//		Xd = Xu * lambda;
//		Yd = Yu * lambda;
//	}
//
//
//void correction(double k, float i, float j, float &disi, float &disj)
//{
//	// (i,j)		punto nell'immagine distorta
//	// (disi,disj)	punto nell'immagine corretta (undistorted)
//	float hyp;
//	float I,J,ni,nj;
//	float ratio = 1;
//
//	ni = i-viewport[0]/2;
//	nj = j-viewport[1]/2;
//	hyp = ni*ni + nj*nj;
//
//	I = ni * (1+ k * hyp);
//	J = nj * (1+ k * hyp);
//
//	disi = (I*ratio+viewport[0]/2);
//	disj = (J*ratio+viewport[1]/2);
//}
//
//
//
//void distorsion( float k ,float i, float j,double & disi, double &disj)
//{
//		// (i,j)		punto nell'immagine corretta (undistorted)
//	// (disi,disj)	punto nell'immagine distorta
//	float hyp;
//	int _I,_J;
//	float I,J,ni,nj;
//	I = i-viewport[0]/2;
//	J = j-viewport[1]/2;
//	hyp = sqrt(I*I + J*J);
//	if((k==0.0) || (hyp <0.001))
//		{
//		disi = i;
//		disj = j;
//		}
//	else
//		{
//		undistorted_to_distorted_sensor_coord (I, J, disi, disj);
//		disi += viewport[0]/2;
//		disj += viewport[1]/2;
//
//
////	hyp = (viewport[0]*viewport[0] + viewport[1]*viewport[1])/4;
////	ni = SX/2 + SX/2 * cam.k[0] * hyp;
/////	nj = SY/2 + SY/2 * cam.k[0] * hyp;
////	float ratio = sqrt(hyp/(ni*ni + nj*nj));
//	float ratio=1;
//
//
//
//	//----------- Maple 
//	//  float t0,t1,t2,sol;
//
//	//t0 = 1/k*pow((108.0*hyp+12.0*sqrt(3.0)*sqrt((4.0+27.0*hyp*hyp*k)/k))*k*k,0.3333333333333333)/6.0-2.0/pow((108.0*hyp+12.0*sqrt(3.0)*sqrt((4.0+27.0*hyp*hyp*k)/k))*k*k,0.3333333333333333);
//
//
//	//t1 = -1/k*pow((108.0*hyp+12.0*sqrt(3.0)*sqrt((4.0+27.0*hyp*hyp*k)/k))*k*k,0.3333333333333333)/12.0+1/pow((108.0*hyp+12.0*sqrt(3.0)*sqrt((4.0+27.0*hyp*
//	//hyp*k)/k))*k*k,0.3333333333333333)+sqrt(-1.0)*sqrt(3.0)*(1/k*pow((108.0*hyp+
//	//12.0*sqrt(3.0)*sqrt((4.0+27.0*hyp*hyp*k)/k))*k*k,0.3333333333333333)/6.0+2.0/
//	//pow((108.0*hyp+12.0*sqrt(3.0)*sqrt((4.0+27.0*hyp*hyp*k)/k))*k*k,
//	//0.3333333333333333))/2.0;
//
//	//t2 = -1/k*pow((108.0*hyp+12.0*sqrt(3.0)*sqrt((4.0+27.0*hyp*hyp*k)/k))*k*k,0.3333333333333333)/12.0+1/pow((108.0*hyp+12.0*sqrt(3.0)*sqrt((4.0+27.0*hyp*
//	//hyp*k)/k))*k*k,0.3333333333333333)-sqrt(-1.0)*sqrt(3.0)*(1/k*pow((108.0*hyp+
//	//12.0*sqrt(3.0)*sqrt((4.0+27.0*hyp*hyp*k)/k))*k*k,0.3333333333333333)/6.0+2.0/
//	//pow((108.0*hyp+12.0*sqrt(3.0)*sqrt((4.0+27.0*hyp*hyp*k)/k))*k*k,
//	//0.3333333333333333))/2.0;
//
//	//sol = (t0>t1)?t0:t1;
//	//sol = (sol<t2)?t2:sol;
//	//sol = t0;
//	//ni = sol*I/hyp;
//	//nj = sol*J/hyp;
//	////----------- 
//
//	//disi = (ni*ratio+viewport[0]/2);
//	//disj = (nj*ratio+viewport[1]/2);
//		}
//}
//	void ResizeGridMap(const int & si,const int & sj ){
//			int j;
//			gridMap.resize(sj+1);
//			for(j=0; j < sj+1; j++)
//					gridMap[j].resize(si+1);
//		}
//	void UpdateGridMap(){
//		int sj = gridMap.size();
//		int si = gridMap[0].size();
//		int i,j;
//		for(j=0; j < sj; j++)
//			for(i=0; i < gridMap[0].size(); i++)
//			//		gridMap[i][j] = Point2<scalar> (i/(double)(si-1),j/(double)(sj-1));
//				{
//					double disi,disj;
//					distorsion( k[0] ,(i/(double)(si-1))*viewport[0], (j/(double)(sj-1))*viewport[1],disi,disj);
//					gridMap[i][j] = Point2<scalar> (disi/viewport[0],disj/viewport[1]);
//				}
//		}
//
//	inline Camera()
//	{
//		k[0]=k[1]=k[2]=k[3]=0.0;
//		valid = false;
//		ortho = false;
//		ResizeGridMap(100,100);// da spostare altrove
//	}
//
//	inline bool IsValid()
//	{
//		return valid;
//	}
//
//	inline bool IsOrtho() const
//	{
//		return ortho;
//	}
//
//	inline void SetInvalid()
//	{
//		valid = false;
//	}
//
//	inline void SetOrtho(bool isOrtho=true) 
//	{
//		ortho = isOrtho;
//	}
//
//		// Genera una camera standard
//	void Standard()
//	{
//		valid = true;
//		ortho = false;
//		view_p  = vectorial(0,0,0);
//		x_axis  = vectorial(1,0,0);
//		y_axis  = vectorial(0,1,0);
//		z_axis  = vectorial(0,0,1);
//		f       = 25.75;
//		s       = Point2<S>(0.0074,0.0074);
//		c       = Point2<S>(320,240);
//		viewport[0] = 640;
//		viewport[1] = 480;
//		k[0]    = 0;
//		k[1]    = 0;
//		k[2]    = 0;
//		k[3]    = 0;
//	}
//
//		// Trasla la camera (world coordinate)
//	inline void Translate( const vectorial & t )
//	{
//		view_p += t;
//	}
//
//		// Trasla la camera (camera coordinate)
//	inline void Move( const vectorial & t )
//	{
//		view_p+= x_axis * t[0]+y_axis * t[1] + z_axis * t[2];		
//	}
//
//	// scala la camera
//	inline void Scale(const scalar & sc){
//		view_p *=sc;
//		s[0]*=sc;
//		s[1]*=sc;
//		f*=sc;
//		//printf("sc\n");
//	}
//
//	
//
//		// NOTA funziona solo se l'ultima colonna di m e' 0,0,0,1
//	void Apply( const Matrix44<S> & m )
//	{
//			// Passo 1: calcolo pseudo inversa di m
//		S s11,s12,s13;
//		S s21,s22,s23;
//		S s31,s32,s33;
//		S s41,s42,s43;
//
//		{
//			S t4  = m[0][0]*m[1][1];
//			S t6  = m[0][0]*m[2][1];
//			S t8  = m[1][0]*m[0][1];
//			S t10 = m[1][0]*m[2][1];
//			S t12 = m[2][0]*m[0][1];
//			S t14 = m[2][0]*m[1][1];
//			S t17 = 1/(t4*m[2][2]-t6*m[1][2]-t8*m[2][2]+t10*m[0][2]+t12*m[1][2]-t14*m[0][2]);
//			S t27 = m[1][0]*m[2][2];
//			S t28 = m[2][0]*m[1][2];
//			S t31 = m[0][0]*m[2][2];
//			S t32 = m[2][0]*m[0][2];
//			S t35 = m[0][0]*m[1][2];
//			S t36 = m[1][0]*m[0][2];
//			S t49 = m[3][0]*m[1][1];
//			S t51 = m[3][0]*m[2][1];
//			S t59 = m[3][0]*m[0][1];
//			s11 = -(-m[1][1]*m[2][2]+m[2][1]*m[1][2])*t17;
//			s12 = -( m[0][1]*m[2][2]-m[2][1]*m[0][2])*t17;
//			s13 =  ( m[0][1]*m[1][2]-m[1][1]*m[0][2])*t17;
//			s21 =  (-t27+t28)*t17;
//			s22 = -(-t31+t32)*t17;
//			s23 = -( t35-t36)*t17;
//			s31 = -(-t10+t14)*t17;
//			s32 =  (-t6 +t12)*t17;
//			s33 =  ( t4 -t8 )*t17;
//			s41 = -(t10*m[3][2]-t27*m[3][1]-t14*m[3][2]+t28*m[3][1]+t49*m[2][2]-t51*m[1][2])*t17;
//			s42 = -(-t6*m[3][2]+t31*m[3][1]+t12*m[3][2]-t32*m[3][1]-t59*m[2][2]+t51*m[0][2])*t17;
//			s43 =  (-t4*m[3][2]+t35*m[3][1]+t8 *m[3][2]-t36*m[3][1]-t59*m[1][2]+t49*m[0][2])*t17;
//			1.0;
//		}
//
//		//Matrix44<S> t2 = tt*m;
//		//print(t2);
//			// Fase 2: Calcolo nuovo punto di vista
//		{
//			S t1  = view_p[2]*s31;
//			S t3  = view_p[2]*s21;
//			S t5  = s43*s21;
//			S t7  = s43*s31;
//			S t9  = view_p[1]*s31;
//			S t11 = view_p[1]*s21;
//			S t13 = s42*s31;
//			S t15 = s42*s21;
//			S t17 = view_p[0]*s32;
//			S t19 = view_p[0]*s22;
//			S t21 = s41*s32;
//			S t23 = s41*s22;
//			S t25 = -t1*s22+t3*s32-t5*s32+t7*s22+t9*s23-t11*s33-t13*s23+t15*s33-t17*s23+t19*s33+t21*s23-t23*s33;
//			S t39 = 1/(s11*s22*s33-s11*s32*s23-s21*s12*s33+s21*s32*s13+s31*s12*s23-s31*s22*s13);
//			S t41 = view_p[0]*s12;
//			S t45 = s41*s12;
//			S t47 = view_p[2]*s11;
//			S t50 = s43*s11;
//			S t53 = view_p[1]*s11;
//			S t56 = s42*s11;
//			S t59 = t41*s33-t17*s13+t21*s13-t45*s33+t47*s32-t1*s12-t50*s32+t7*s12-t53*s33+t9*s13+t56*s33-t13*s13;
//			S t73 = t15*s13-t56*s23+t19*s13-t41*s23-t23*s13+t45*s23-t11*s13+t53*s23+t3*s12-t47*s22-t5*s12+t50*s22;
//
//			view_p[0] =  t25*t39;
//			view_p[1] = -t59*t39;
//			view_p[2] = -t73*t39;
//		}
//
//			// Fase 3: Calcol nuovo sistema di riferimento
//		{
//			S A00 = s11*x_axis[0]+s12*x_axis[1]+s13*x_axis[2];
//			S A01 = s11*y_axis[0]+s12*y_axis[1]+s13*y_axis[2];
//			S A02 = s11*z_axis[0]+s12*z_axis[1]+s13*z_axis[2];
//		//	S A03 = 0.0;
//			S A10 = s21*x_axis[0]+s22*x_axis[1]+s23*x_axis[2];
//			S A11 = s21*y_axis[0]+s22*y_axis[1]+s23*y_axis[2];
//			S A12 = s21*z_axis[0]+s22*z_axis[1]+s23*z_axis[2];
//		//	S A13 = 0.0;
//			S A20 = s31*x_axis[0]+s32*x_axis[1]+s33*x_axis[2];
//			S A21 = s31*y_axis[0]+s32*y_axis[1]+s33*y_axis[2];
//			S A22 = s31*z_axis[0]+s32*z_axis[1]+s33*z_axis[2];
//
//			x_axis[0] = A00; x_axis[1] = A10; x_axis[2] = A20;
//			y_axis[0] = A01; y_axis[1] = A11; y_axis[2] = A21;
//			z_axis[0] = A02; z_axis[1] = A12; z_axis[2] = A22;
//		//	S A1[2][3] = 0.0;
//		//	S A1[3][0] = 0.0;
//		//	S A1[3][1] = 0.0;
//		//	S A1[3][2] = 0.0;
//		//	S A1[3][3] = 1.0;
//		}
//	}
//
//	/*
//		// Applica una trasformazione
//	void Apply( const Matrix44<S> & m )
//	{
//		Point3<S> tx = view_p+x_axis;
//		Point3<S> ty = view_p+y_axis;
//		Point3<S> tz = view_p+z_axis;
//
//		view_p = m.Apply(view_p);
//		
//		x_axis = m.Apply(tx) - view_p;
//		y_axis = m.Apply(ty) - view_p;
//		z_axis = m.Apply(tz) - view_p;
//	}
//
//			// Applica una trasformazione ma bene!
//	void Stable_Apply( const Matrix44<S> & m )
//	{
//		Point3<S> tx = view_p+x_axis;
//		Point3<S> ty = view_p+y_axis;
//		Point3<S> tz = view_p+z_axis;
//
//		view_p = m.Stable_Apply(view_p);
//		
//		x_axis = m.Stable_Apply(tx) - view_p;
//		y_axis = m.Stable_Apply(ty) - view_p;
//		z_axis = m.Stable_Apply(tz) - view_p;
//	}
//
//	*/
//
//	void Project( const vectorial & p, Point2<S> & q ) const
//	{
//		vectorial dp = p - view_p;
//		S  dx = dp*x_axis;
//		S  dy = dp*y_axis;
//		S  dz = dp*z_axis;
//		
//		S  tx = dx;
//		S  ty = -dy;
//		S  qx,qy;
//
//		// nota: per le camere ortogonali viewportM vale 1
//		if(!IsOrtho())
//		{
//			tx *= f/dz;
//			ty *= f/dz;
//
//			undistorted_to_distorted_sensor_coord(tx,ty,qx,qy);
//
//			q[0] = qx/s[0]+c[0];
//			q[1] = qy/s[1]+c[1];
//		}
//		else
//		{
//			q[0] = tx/(s[0]*viewportM)+c[0];
//			q[1] = ty/(s[1]*viewportM)+c[1];
//		}	
//	}
//
//#if 1
//	void Show( FILE * fp )
//	{
//		if(valid)
//			fprintf(fp,
//				"posiz.: %g %g %g\n"
//				"x axis: %g %g %g\n"
//				"y axis: %g %g %g\n"
//				"z axis: %g %g %g\n"
//				"focal : %g  scale: %g %g  center: %g %g\n"
//				"viewp.: %d %d  distorsion: %g %g %g %g\n"
//				,view_p[0],view_p[1],view_p[2]
//				,x_axis[0],x_axis[1],x_axis[2]
//				,y_axis[0],y_axis[1],y_axis[2]
//				,z_axis[0],z_axis[1],z_axis[2]
//				,f,s[0],s[1],c[0],c[1]
//				,viewport[0],viewport[1],k[0],k[1],k[2],k[3]
//			);
//		else
//			fprintf(fp,"Invalid\n");
//	}
//#endif
//
//		// Legge una camera in descrizione tsai binario
//	static void load_tsai_bin (FILE *fp, tsai_camera_parameters *cp, tsai_calibration_constants *cc)
//	{
//		double    sa,
//				  ca,
//				  sb,
//				  cb,
//				  sg,
//				  cg;
//
//		fread(&(cp->Ncx),sizeof(double),1,fp);
//		fread(&(cp->Nfx),sizeof(double),1,fp);
//		fread(&(cp->dx),sizeof(double),1,fp);
//		fread(&(cp->dy),sizeof(double),1,fp);
//		fread(&(cp->dpx),sizeof(double),1,fp);
//		fread(&(cp->dpy),sizeof(double),1,fp);
//		fread(&(cp->Cx),sizeof(double),1,fp);
//		fread(&(cp->Cy),sizeof(double),1,fp);
//		fread(&(cp->sx),sizeof(double),1,fp);
//
//		fread(&(cc->f),sizeof(double),1,fp);
//		fread(&(cc->kappa1),sizeof(double),1,fp);
//		fread(&(cc->Tx),sizeof(double),1,fp);
//		fread(&(cc->Ty),sizeof(double),1,fp);
//		fread(&(cc->Tz),sizeof(double),1,fp);
//		fread(&(cc->Rx),sizeof(double),1,fp);
//		fread(&(cc->Ry),sizeof(double),1,fp);
//		fread(&(cc->Rz),sizeof(double),1,fp);
//    
//
//		SINCOSd (cc->Rx, sa, ca);
//		SINCOSd (cc->Ry, sb, cb);
//		SINCOSd (cc->Rz, sg, cg);
//
//		cc->r1 = cb * cg;
//		cc->r2 = cg * sa * sb - ca * sg;
//		cc->r3 = sa * sg + ca * cg * sb;
//		cc->r4 = cb * sg;
//		cc->r5 = sa * sb * sg + ca * cg;
//		cc->r6 = ca * sb * sg - cg * sa;
//		cc->r7 = -sb;
//		cc->r8 = cb * sa;
//		cc->r9 = ca * cb;
//
//		fread(&(cc->p1),sizeof(double),1,fp);
//		fread(&(cc->p2),sizeof(double),1,fp);
//	}
//
//	void load_tsai (FILE *fp, tsai_camera_parameters *cp, tsai_calibration_constants *cc)
//	{
//		double    sa,
//				  ca,
//				  sb,
//				  cb,
//				  sg,
//				  cg;
//
//		fscanf (fp, "%lf", &(cp->Ncx));
//		fscanf (fp, "%lf", &(cp->Nfx));
//		fscanf (fp, "%lf", &(cp->dx));
//		fscanf (fp, "%lf", &(cp->dy));
//		fscanf (fp, "%lf", &(cp->dpx));
//		fscanf (fp, "%lf", &(cp->dpy));
//		fscanf (fp, "%lf", &(cp->Cx));
//		fscanf (fp, "%lf", &(cp->Cy));
//		fscanf (fp, "%lf", &(cp->sx));
//
//		fscanf (fp, "%lf", &(cc->f));
//		fscanf (fp, "%lf", &(cc->kappa1));
//		fscanf (fp, "%lf", &(cc->Tx));
//		fscanf (fp, "%lf", &(cc->Ty));
//		fscanf (fp, "%lf", &(cc->Tz));
//		fscanf (fp, "%lf", &(cc->Rx));
//		fscanf (fp, "%lf", &(cc->Ry));
//		fscanf (fp, "%lf", &(cc->Rz));
//
//		SINCOSd (cc->Rx, sa, ca);
//		SINCOSd (cc->Ry, sb, cb);
//		SINCOSd (cc->Rz, sg, cg);
//
//		cc->r1 = cb * cg;
//		cc->r2 = cg * sa * sb - ca * sg;
//		cc->r3 = sa * sg + ca * cg * sb;
//		cc->r4 = cb * sg;
//		cc->r5 = sa * sb * sg + ca * cg;
//		cc->r6 = ca * sb * sg - cg * sa;
//		cc->r7 = -sb;
//		cc->r8 = cb * sa;
//		cc->r9 = ca * cb;
//
//		fscanf (fp, "%lf", &(cc->p1));
//		fscanf (fp, "%lf", &(cc->p2));
//	}
//	
//		// Importa una camera dal formato tsai
//	void import( const tsai_camera_parameters & cp,
//		         const tsai_calibration_constants & cc,
//				 const int image_viewport[2]
//			   )
//	{
//		assert(!IsOrtho());
//		valid = true;
//		x_axis[0] = cc.r1; x_axis[1] = cc.r2; x_axis[2] = cc.r3;
//		y_axis[0] = cc.r4; y_axis[1] = cc.r5; y_axis[2] = cc.r6;
//		z_axis[0] = cc.r7; z_axis[1] = cc.r8; z_axis[2] = cc.r9;
//
//		view_p[0] = - (cc.Tx * x_axis[0] + cc.Ty * y_axis[0] + cc.Tz * z_axis[0]);
//		view_p[1] = - (cc.Tx * x_axis[1] + cc.Ty * y_axis[1] + cc.Tz * z_axis[1]);
//		view_p[2] = - (cc.Tx * x_axis[2] + cc.Ty * y_axis[2] + cc.Tz * z_axis[2]);
//
//		s[0] = cp.dpx/cp.sx;
//		s[1] = cp.dpy;
//		c[0] = cp.Cx;
//		c[1] = cp.Cy;
//
//		f = cc.f;
//		viewport[0] = image_viewport[0];
//		viewport[1] = image_viewport[1];
//
//		k[0] = cc.kappa1;
//		k[1] = cc.kappa1;
//		k[2] = 0;
//		k[2] = 0;
//	}
//
//		// Esporta una camera in formato tsai
//	void export( tsai_camera_parameters & cp,
//		         tsai_calibration_constants & cc,
//				 int image_viewport[2]
//			   )
//	{
//		assert(!IsOrtho());
//		cc.r1 = x_axis[0];  cc.r2 = x_axis[1]; cc.r3= x_axis[2] ;
//		cc.r4 = y_axis[0];  cc.r5 = y_axis[1]; cc.r6= y_axis[2] ;
//		cc.r7 = z_axis[0];  cc.r8 = z_axis[1]; cc.r9= z_axis[2] ;
//
//		cc.Tx = - (view_p[0] * x_axis[0] + view_p[1] * x_axis[1] + view_p[2] * x_axis[2]);
//		cc.Ty = - (view_p[0] * y_axis[0] + view_p[1] * y_axis[1] + view_p[2] * y_axis[2]);
//		cc.Tz = - (view_p[0] * z_axis[0] + view_p[1] * z_axis[1] + view_p[2] * z_axis[2]);
//
//		cp.dpx = s[0];
//		cp.dpy = s[1];
//
//		cp.Cx=  c[0] ;
//		cp.Cy=  c[1] ;
//		cp.sx= 1; 
//
//		cc.f= f ;
//
//		image_viewport[0] = viewport[0];
//		image_viewport[1] = viewport[1];
//
//		cc.kappa1= k[0] ;
//		cc.kappa1= k[1] ;
//	}
//
//
//	void Save(FILE * out)
//	{
//		fprintf(out,"VIEW_POINT %f %f %f\n",	view_p[0],view_p[1],view_p[2]);
//		fprintf(out,"X_AXIS		%f %f %f\n",	x_axis[0],x_axis[1],x_axis[2]);
//		fprintf(out,"Y_AXIS		%f %f %f\n",	y_axis[0],y_axis[1],y_axis[2]);
//		fprintf(out,"Z_AXIS		%f %f %f\n",	z_axis[0],z_axis[1],z_axis[2]);
//		fprintf(out,"FOCUS_LENGHT  %f \n",	 f);
//		fprintf(out,"SCALE  %f %f \n",	 s[0], s[1]);
//		fprintf(out,"VIEWPORT  %d %d \n",	 viewport[0], viewport[1]);
//		fprintf(out,"VIEWPORTM %f\n",	 viewportM);
//		fprintf(out,"RADIAL_DISTORSION %.10g %.10g \n",	 k[0],k[1]);
//		fprintf(out,"CENTER  %f %f \n",	 c[0], c[1]);
//		fprintf(out,"IS_VALID %d\n", IsValid());
//		fprintf(out,"END_CAMERA\n");
//	}
//
//	void Load(FILE * in)
//	{
//		char row[255];
//		Standard();
//	while(!feof(in))
//			{
//				fscanf(in,"%s",row);
//				if(strcmp(row,"VIEW_POINT")==0)
//	  				fscanf(in,"%lg %lg %lg",&view_p[0],&view_p[1],&view_p[2]);
//				else
//				if(strcmp(row,"X_AXIS")==0)
//					fscanf(in,"%lg %lg %lg",&	x_axis[0],&x_axis[1],&x_axis[2]);
//				else
//				if(strcmp(row,"Y_AXIS")==0)
//					fscanf(in,"%lg %lg %lg",&	y_axis[0],&y_axis[1],&y_axis[2]);
//				else
//				if(strcmp(row,"Z_AXIS")==0)
//					fscanf(in,"%lg %lg %lg",&	z_axis[0],&z_axis[1],&z_axis[2]);
//				else
//				if(strcmp(row,"FOCUS_LENGHT")==0)
//					fscanf(in,"%lg",&f);
//				else
//				if(strcmp(row,"SCALE")==0)
//					fscanf(in,"%lg %lg",&s[0],&s[1]);
//				else
//				if(strcmp(row,"VIEWPORT")==0)
//					fscanf(in,"%d %d",	&viewport[0],&viewport[1]);
//				else
//				if(strcmp(row,"VIEWPORTM")==0)
//					fscanf(in,"%f",	&viewportM);
//				else
//				if(strcmp(row,"CENTER")==0)
//					fscanf(in,"%lg %lg",	&c[0],&c[1]);
//				else
//				if(strcmp(row,"RADIAL_DISTORSION")==0)
//					fscanf(in,"%lg %lg",	&k[0],&k[1]);
//				else
//				if(strcmp(row,"IS_VALID")==0)
//					fscanf(in,"%d",&valid);
//				if(strcmp(row,"END_CAMERA")==0)
//					break;
//			}
//	}
//
//#ifdef __GL_H__
//
//// Prende in ingresso il bounding box dell'oggetto da inquadrare e setta projection e modelmatrix
//// in modo da matchare il piu' possibile quelle della camera. Ovviamente (?) si ignora le distorsioni radiali.
//// Nota che bb viene utilizzato solo per settare i near e far plane in maniera sensata.
//void SetGL(const Box3<scalar> &bb,scalar subx0=0, scalar subx1=1,scalar suby0=0,scalar suby1=1)
//{
//	scalar _,__;
//	SetGL(_,__,bb,subx0, subx1, suby0, suby1);
//
//}
//
//void SetGL(scalar &znear, scalar &zfar,const Box3<scalar> &bb,scalar subx0=0, 
//		   scalar subx1=1,scalar suby0=0,scalar suby1=1)
//{
//	glMatrixMode(GL_PROJECTION);
//	glLoadIdentity();
//	scalar left,right;
//	scalar bottom, top;
//	scalar w,h;
//
//	// La lunghezza focale <f> e' la distanza del piano immagine dal centro di proiezione. 
//	// il che mappa direttamente nella chiamata glFrustum che prende in ingresso 
//	// le coordinate del piano immagine posto a znear.
//
//	float imleft   =-c[0]*s[0];
//	float imright  =(viewport[0]-c[0])*s[0];
//	float imbottom =-c[1]*s[1];
//	float imtop    =(viewport[1]-c[1])*s[1];
//	znear = Distance(view_p, bb.Center())-bb.Diag();
//	zfar  = Distance(view_p, bb.Center())+bb.Diag();
//	
//	w=imright-imleft;
//	h=imtop-imbottom;
//	
//	// Quindi il frustum giusto sarebbe questo, 
//	//            glFrustum(imleft, imright, imbottom, imtop, f, zfar);
//  // ma per amor di opengl conviene spostare il near plane fino ad essere vicino all'oggetto da inquadrare.
//	// Cambiare f significa amplificare in maniera proporzionale anche i left right ecc.
//	
//	// 8/5/02 Nota che il near plane va spostato verso l'oggetto solo se quello calcolato sopra e' maggiore di 'f' 
//	// nota che potrebbe anche succedere che znear <0 (viewpoint vicino ad un oggetto con il bb allungato);
//	if(znear<f) znear=f;
//
//	float nof = znear/f; 
//	if(subx0==0 && subx1 == 1 && suby0==0 && suby1 == 1) 
//	{
//		if(!IsOrtho())
//			glFrustum(imleft*nof, imright*nof, imbottom*nof, imtop*nof, znear, zfar);
//		else
//			glOrtho(imleft*viewportM, imright*viewportM, imbottom*viewportM, imtop*viewportM, znear, zfar);
//	}
//	else {// nel caso si voglia fare subboxing 
//		left   = imleft+w*subx0;
//		right  = imleft+w*subx1;
//		bottom = imbottom +h*suby0;
//		top    = imbottom +h*suby1;
//		{
//		if(!IsOrtho())
//			glFrustum(left*nof, right*nof, bottom*nof, top*nof, znear, zfar);
//		else
//			glOrtho(left*viewportM, right*viewportM, bottom*viewportM, top*viewportM, znear, zfar);
//		}
//	}
//
//	glMatrixMode(GL_MODELVIEW);
//	glLoadIdentity();
//	scalar l=max(scalar(1.0),view_p.Norm());
//	gluLookAt(view_p[0], view_p[1], view_p[2],
//						view_p[0] + (z_axis[0]*l), 
//						view_p[1] + (z_axis[1]*l), 
//						view_p[2] + (z_axis[2]*l),
//						y_axis[0],y_axis[1],y_axis[2]);
//}
//// Sposta la camera a caso di in maniera che l'angolo di variazione rispetto al punt c passato sia inferiore a RadAngle
////
//void Jitter(Point3<scalar> c, scalar RadAngle)
//{
//	Point3<scalar> rnd(1.0 - 2.0*scalar(rand())/RAND_MAX, 
//		                 1.0 - 2.0*scalar(rand())/RAND_MAX, 
//										 1.0 - 2.0*scalar(rand())/RAND_MAX);
//	rnd.Normalize();
//	Matrix44<scalar> m,t0,t1,tr;
//	Point3<scalar> axis = rnd ^ (view_p-c).Normalize();
//	scalar RadRandAngle=RadAngle*(1.0 - 2.0*scalar(rand())/RAND_MAX);
//	t0.Translate(c);
//	t1.Translate(-c);
//	m.Rotate(ToDeg(RadRandAngle),axis);
//  tr=t1*m*t0;
//  Apply(tr);
//}
//
//
//
//
//void glTexGen(int offx =0, // angolo basso sinistra della
//			  int offy=0,  // subtexture per la quale si vogliono settare le coordinate	
//			  int sx=1,    // Dimensioni in Texel
//			  int sy=1, 
//			  int Tx=1,	   // Dimensioni della texture	
//			  int Ty=1)
//{
//	// prendi la rototraslazione che
//	// trasforma la coordinata nel
//	// sistema di coordinate della camera
//	Matrix44d M;
//	M[0][0] =  x_axis[0];
//	M[0][1] =  x_axis[1];
//	M[0][2] =  x_axis[2];
//	M[0][3] = -view_p* x_axis ;
//
//	M[1][0] =  y_axis[0];
//	M[1][1] =  y_axis[1];
//	M[1][2] =  y_axis[2];
//	M[1][3] = -view_p* y_axis;
//
//	M[2][0] =  z_axis[0];
//	M[2][1] =  z_axis[1];
//	M[2][2] =  z_axis[2];
//	M[2][3] = -view_p* z_axis;
//
//	M[3][0] = 0.0;
//	M[3][1] = 0.0;
//	M[3][2] = 0.0;
//	M[3][3] = 1.0;
//
//	//	prendi la matrice di proiezione
//	Matrix44d P;
//	P.SetZero();
//
//	if(!IsOrtho())//  prospettica
//	{
//		
//		P[0][0] = sx/(s[0]*viewport[0]*Tx);
//		P[0][2] = (1/f)*(offx+0.5*sx)/Tx; 
//
//		P[1][1] = sy/(s[1]* viewport[1]*Ty);
//		P[1][2] = (1/f)*(offy+0.5*sy)/Ty; 
//
//		P[2][2] = 1;
//		P[3][2] = 1/f;
//	}
//	else //  ortogonale
//	{
//		P[0][0] = sx/(s[0]*viewport[0]*viewportM*Tx);
//		P[0][3] =  (offx+0.5*sx)/Tx; // l'effetto e' una traslazione di +1/2
//
//		P[1][1] = sy/(s[1]* viewport[1]*viewportM*Ty);
//		P[1][3] =  (offy+0.5*sy)/Ty; // l'effetto e' una traslazione di +1/2
//
//		P[2][2] = 1;
//		P[3][3] = 1;
//	}
//	// componi
//	Matrix44d PM = P*M;
//
//	glTexGend(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
//	glTexGend(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
//	glTexGend(GL_Q, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
//
//	glTexGendv(GL_S,GL_OBJECT_PLANE,&PM[0][0]);
//	glTexGendv(GL_T,GL_OBJECT_PLANE,&PM[1][0]);
//	glTexGendv(GL_Q,GL_OBJECT_PLANE,&PM[3][0]);
//
//	glEnable(GL_TEXTURE_GEN_S);
//	glEnable(GL_TEXTURE_GEN_T);
//	glDisable(GL_TEXTURE_GEN_R);
//	glEnable(GL_TEXTURE_GEN_Q);
//}
//
//// versione per le texture rettangolare NV_TEXTURE_RECTANGLE
//// la differenza da glTexGen e' che il mapping e' in [0..sx]X[0..sy]
//void glTexGen_NV(int sx,    // Texture Size
//				 int sy)
//{
//	// prendi la rototraslazione che
//	// trasforma la coordinata nel
//	// sistema di coordinate della camera
//	Matrix44d M;
//	M[0][0] =  x_axis[0];
//	M[0][1] =  x_axis[1];
//	M[0][2] =  x_axis[2];
//	M[0][3] = -view_p* x_axis ;
//
//	M[1][0] =  y_axis[0];
//	M[1][1] =  y_axis[1];
//	M[1][2] =  y_axis[2];
//	M[1][3] = -view_p* y_axis;
//
//	M[2][0] =  z_axis[0];
//	M[2][1] =  z_axis[1];
//	M[2][2] =  z_axis[2];
//	M[2][3] = -view_p* z_axis;
//
//	M[3][0] = 0.0;
//	M[3][1] = 0.0;
//	M[3][2] = 0.0;
//	M[3][3] = 1.0;
//
//	//	prendi la matrice di proiezione
//	Matrix44d P;
//	P.SetZero();
//
//	if(!IsOrtho())//  prospettica
//	{
//		
//		P[0][0] = sx/(s[0]*viewport[0]);
//		P[0][2] = sx*(1/f)*( 0.5); 
//
//		P[1][1] = sy/(s[1]* viewport[1] );
//		P[1][2] = sy*(1/f)*( 0.5); 
//
//		P[2][2] = 1;
//		P[3][2] = 1/f;
//	}
//	else //  ortogonale
//	{
//		P[0][0] = sx/(s[0]*viewport[0]*viewportM);
//		P[0][3] = sx*  0.5 ; // l'effetto e' una traslazione di +1/2
//
//		P[1][1] = sy/(s[1]* viewport[1]*viewportM);
//		P[1][3] = sy*  0.5 ; // l'effetto e' una traslazione di +1/2
//
//		P[2][2] = 1;
//		P[3][3] = 1;
//	}
//	// componi
//	Matrix44d PM = P*M;
//
//	glTexGend(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
//	glTexGend(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
//	glTexGend(GL_Q, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
//
//	glTexGendv(GL_S,GL_OBJECT_PLANE,&PM[0][0]);
//	glTexGendv(GL_T,GL_OBJECT_PLANE,&PM[1][0]);
//	glTexGendv(GL_Q,GL_OBJECT_PLANE,&PM[3][0]);
//
//	glEnable(GL_TEXTURE_GEN_S);
//	glEnable(GL_TEXTURE_GEN_T);
//	glDisable(GL_TEXTURE_GEN_R);
//	glEnable(GL_TEXTURE_GEN_Q);
////	glDisable(GL_TEXTURE_GEN_Q);
//
//}
//#endif // __GL_H__
//
//};
//}	// End namespace vcg
