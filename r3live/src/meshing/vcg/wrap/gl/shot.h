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
Revision 1.12  2006/12/18 16:02:57  matteodelle
minor eroor correction on variable names

Revision 1.11  2006/12/18 15:26:24  callieri
added a function to approximate a far plane value given a shot and the mesh bbox

Revision 1.10  2006/12/18 14:28:07  matteodelle
*** empty log message ***

Revision 1.9  2006/12/18 09:46:39  callieri
camera+shot revamp: changed field names to something with more sense, cleaning of various functions, correction of minor bugs/incongruences, removal of the infamous reference in shot.

Revision 1.8  2006/01/11 16:06:25  matteodelle
*** empty log message ***


Revision 1.8  2005/01/11 17:06:30  dellepiane
FromTrackball() coorected (similarity->Extrinsics

Revision 1.7  2005/11/25 10:33:33  spinelli
shot.camera  -> shot.Intrinsics
shot.similarity.Matrix() -> shot.Extrinsics.Matrix()

Revision 1.6  2005/02/22 11:15:01  ganovelli
added vcg namespace

Revision 1.5  2005/02/11 11:43:09  tommyfranken
FromTrackball() corrected

Revision 1.4  2004/12/15 18:45:06  tommyfranken
*** empty log message ***

Revision 1.3  2004/11/03 09:41:57  ganovelli
added FromTrackball and fixed include names (Poiint to point)

Revision 1.2  2004/10/05 19:04:45  ganovelli
changed from classes to functions

Revision 1.1  2004/09/15 22:59:13  ganovelli
creation

Revision 1.2  2004/09/06 21:41:30  ganovelli
*** empty log message ***

Revision 1.1  2004/09/03 13:01:51  ganovelli
creation

****************************************************************************/


#ifndef __VCGLIB_GLSHOT
#define __VCGLIB_GLSHOT

// include vcg stuff
#include <vcg/space/point2.h>
#include <vcg/space/point3.h>
#include <vcg/math/similarity.h>

#include <vcg/math/shot.h>

// include wrap stuff
#include <wrap/gui/trackball.h>
#include <wrap/gl/math.h>
#include <wrap/gl/camera.h>

template <class ShotType>
struct GlShot {

	typedef typename ShotType::ScalarType ScalarType;
	typedef GlCamera<typename ShotType::CameraType> GlCameraType;

/// returns the OpenGL 4x4 MODELVIEW matrix that describes the shot position and orientation (extrinsics)
static void MatrixGL(ShotType & shot,vcg::Matrix44<ScalarType> & m) 
{
	m = shot.GetWorldToExtrinsicsMatrix();
}

/// set the OpenGL MODELVIEW matrix to match the shot (extrinsics)
static void TransformGL(vcg::Shot<ScalarType> & shot)
{
	vcg::Matrix44<ScalarType> m;
	MatrixGL(shot,m);
	glMultMatrix(m);
}

/// set the OpenGL PROJECTION and MODELVIEW matrix to match camera+shot. requires near and far plane
static void SetView(vcg::Shot<ScalarType> & shot, ScalarType nearDist, ScalarType farDist)
{
	assert(glGetError() == 0);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	assert(glGetError() == 0);
	GlCameraType::TransformGL(shot.Intrinsics, nearDist, farDist); // apply camera/projection transformation
	assert(glGetError() == 0);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	GlShot<ShotType>::TransformGL(shot);	// apply similarity/modelview transformation
	assert(glGetError() == 0);
}

/// restore the previous OpenGL modelview and projection state. to be called AFTER a SetView
static void	UnsetView()
{
	glPushAttrib(GL_TRANSFORM_BIT);
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
}

/// given a shot and the mesh bounding box, returns an approximate far plane distance
/// distance is always approximated by excess
static ScalarType GetFarPlane(vcg::Shot<ScalarType> & shot, vcg::Box3<ScalarType> bbox)
{
	ScalarType farDist;

	vcg::Point3<ScalarType> farcorner;
    vcg::Point3<ScalarType> campos = shot.Extrinsics.Tra();
	 
	 if (abs(campos.X() - bbox.max.X()) > abs(campos.X() - bbox.min.X()))
		 farcorner.X() = bbox.max.X();
	 else
		 farcorner.X() = bbox.min.X();

	 if (abs(campos.Y() - bbox.max.Y()) > abs(campos.Y() - bbox.min.Y()))
		 farcorner.Y() = bbox.max.Y();
	 else
		 farcorner.Y() = bbox.min.Y();

	 if (abs(campos.Z() - bbox.max.Z()) > abs(campos.Z() - bbox.min.Z()))
		 farcorner.Z() = bbox.max.Z();
	 else
		 farcorner.Z() = bbox.min.Z();

	 farDist = (campos - farcorner).Norm();

	return farDist;
}


/// given a shot and the mesh bounding box, return near and far plane (exact)
static void GetNearFarPlanes(vcg::Shot<ScalarType> & shot, vcg::Box3<ScalarType> bbox, ScalarType &nr, ScalarType &fr)
{
  vcg::Point3<ScalarType> zaxis = shot.Axis(2); 
  ScalarType offset = zaxis * shot.GetViewPoint();
  bool first = true;
  for(int i = 0; i < 8; i++) {
    vcg::Point3<ScalarType> c = bbox.P(i);
    ScalarType d = -(zaxis * c - offset);
    if(first || d < nr)  
      nr = d;
    if(first || d > fr)   
      fr = d;
    first = false;
  }
}



static void SetSubView(vcg::Shot<ScalarType> & shot,
					   vcg::Point2<ScalarType> p1,
					   vcg::Point2<ScalarType> p2)
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	assert(glGetError() == 0);
	GlCameraType::SetSubView(shot.Intrinsics,p1,0,1000,p2);
	assert(glGetError() == 0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	GlShot<ShotType>::TransformGL(shot);							// apply similarity/modelview transformation
	assert(glGetError() == 0);
}


	
/**********************************
DEFINE SHOT FROM TRACKBALL
Adds to a given shot the trackball transformations.
After this operation the trackball should be resetted, to avoid
multiple apply of the same transformation.
***********************************/
static void FromTrackball(const vcg::Trackball & tr, 
						  vcg::Shot<ScalarType> & sShot, 
						  vcg::Shot<ScalarType> & shot )
{
	vcg::Point3<ScalarType>		cen; cen.Import(tr.center);
	vcg::Point3<ScalarType>		tra; tra.Import(tr.track.tra);
	vcg::Matrix44<ScalarType>	trM; trM.FromMatrix(tr.track.Matrix());

	vcg::Point3<ScalarType>		vp = Inverse(trM)*(sShot.GetViewPoint()-cen) +cen;// +tra;

	shot.SetViewPoint(vp);
	shot.Extrinsics.SetRot(sShot.Extrinsics.Rot()*trM);
//	shot.Extrinsics.sca	=	sShot.Extrinsics.sca*(ScalarType)tr.track.sca;
}
};
#endif





