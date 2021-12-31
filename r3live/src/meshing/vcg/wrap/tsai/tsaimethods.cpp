/***************************************************************************
 *   Copyright (C) 2008 by Luca Baronti   				   *
 *   lbaronti@gmail.com   						   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/***************************************************************************
  History
Modified by sottile on July 2009
****************************************************************************/

/****************************************************************************
TSAI METHODS
  Interface class between the tsai.lib and the project structure.
  It is sufficient calling <alignImage> with correct parameters, to
  get a calibrated shot.
  -
  Comment:
  Sometimes the Tsai Lib returns a irregular shifted translation values.
  Since now no solution found.
*****************************************************************************/
extern "C" {
#include <../meshlab/src/external/tsai-30b3/cal_main.h>
}

#include "tsaimethods.h"		//base header

#include <vcg/math/camera.h>

/*********************************************
...
*********************************************/
bool TsaiMethods::calibrate( vcg::Shot<double>* shot,std::list<TsaiCorrelation>* corr, bool p_foc)
{
  bool my_ret_val=false;

  if(corr->size() >= MIN_POINTS_FOR_CALIBRATE){

    //initialize_photometrics_parms ();
    Shot2Tsai(shot);

    if(createDataSet(corr,shot))
    {
      if (p_foc)
        //noncoplanar_calibration();
        noncoplanar_calibration_with_full_optimization ();
      else
        noncoplanar_extrinsic_parameter_estimation ();

      Tsai2Shot(shot);
      my_ret_val = true;
    }
    else my_ret_val = false;
  }
  return my_ret_val;
}

/*********************************************
CREATE DATA SET
*********************************************/
//TOGLIERE SHOT DAI PARAMETRI!
bool TsaiMethods::createDataSet(std::list<TsaiCorrelation>* corr,vcg::Shot<double>* s)
{
  bool my_ret_val=false;

  vcg::Point3d *p1;
  vcg::Point2d *p2;
        int count=0;

        std::list<TsaiCorrelation>::iterator it_c;
  TsaiCorrelation* c;

        double ratio = s->Intrinsics.ViewportPx.X()/(double) s->Intrinsics.ViewportPx.Y();

        for ( it_c= corr->begin() ; it_c !=corr->end(); it_c++ ){
                c=&*it_c;
                p1=&(c->point3d);
    p2=&(c->point2d);

    if(p1!=NULL && p2!=NULL)
    {
                        cd.xw[count] = p1->X();
                        cd.yw[count] = p1->Y();
                        cd.zw[count] = p1->Z();
//                        cd.Xf[count] = (p2->X()+1)/2.0 * cp.Cx*2.0;
//                        cd.Yf[count] = ((-p2->Y())+1)/2.0 * cp.Cy*2.0;

                        cd.Xf[count] = ((p2->X()/ratio) +1)/2.0 * cp.Cx*2.0;
                        cd.Yf[count] = ((-p2->Y())+1)/2.0 * cp.Cy*2.0;
      count++;
    }
    if(count>=MAX_POINTS) break;

  }//all corrs
        assert(count==corr->size());
        cd.point_count = count;
        //qDebug("all points: %i",cd.point_count);
        if(count>0 && count<MAX_POINTS && count>10) my_ret_val = true;

        return my_ret_val;
}


/*********************************************
SHOT 2 TSAI
Transformation of the camera data between tsai structure and vcg structure
*********************************************/
void TsaiMethods::Shot2Tsai(vcg::Shot<double>* shot){
  vcg::Matrix44d mat = shot->Extrinsics.Rot();

  vcg::Point3d view_p = shot->Extrinsics.Tra();

  cc.r1 = mat[0][0];  cc.r2 = mat[0][1]; cc.r3 = mat[0][2];
  cc.r4 = -mat[1][0];  cc.r5 = -mat[1][1]; cc.r6 = -mat[1][2];
  cc.r7 = -mat[2][0];  cc.r8 = -mat[2][1]; cc.r9 = -mat[2][2];


  vcg::Point3<vcg::Shot<double>::ScalarType> tl;//(-cc.Tx,cc.Ty,cc.Tz);
  tl = mat* shot->Extrinsics.Tra();
  cc.Tx = -tl[0];
  cc.Ty = tl[1];
  cc.Tz = tl[2];

  //cc.Tx = view_p.X();
  //cc.Ty = view_p.Y();
  //cc.Tz = view_p.Z();

  Cam2Tsai(shot);
}

/*********************************************
TSAI 2 SHOT
Transformation of the camera data between tsai structure and vcg structure
*********************************************/
void TsaiMethods::Tsai2Shot(vcg::Shot<double>* shot, bool p_foc ) {

  if(p_foc)
    shot->Intrinsics.FocalMm =	cc.f;//*cp.sx;// *SCALE_FACTOR;
  /* old ones
  shot->Intrinsics.DistorCenterPx[0] = cc.p1;
  shot->Intrinsics.DistorCenterPx[1] = cc.p2;

  shot->Intrinsics.DistorCenterPx[0] = shot->Intrinsics.CenterPx.X()+(cc.p1/shot->Intrinsics.PixelSizeMm.X());
  shot->Intrinsics.DistorCenterPx[1] = shot->Intrinsics.CenterPx.Y()+(cc.p2/shot->Intrinsics.PixelSizeMm.Y());
  */
  shot->Intrinsics.DistorCenterPx[0] = cp.Cx;
  shot->Intrinsics.DistorCenterPx[1] = cp.Cy;

  shot->Intrinsics.k[0]=cc.kappa1;

  /* ROTATION */
  vcg::Matrix44<vcg::Shot<double>::ScalarType> mat;
  vcg::Matrix44<vcg::Shot<double>::ScalarType> s_mat=shot->Extrinsics.Rot();
  mat.SetIdentity();
  mat[0][0]=cc.r1; 	mat[0][1]=cc.r2; 	mat[0][2]=cc.r3;
  mat[1][0]=-cc.r4; 	mat[1][1]=-cc.r5; 	mat[1][2]=-cc.r6;
  mat[2][0]=-cc.r7; 	mat[2][1]=-cc.r8; 	mat[2][2]=-cc.r9;

  shot->Extrinsics.SetRot(mat);

  /* TRANSLATION */
  vcg::Point3d tl = shot->Extrinsics.Tra();

  tl = vcg::Inverse(shot->Extrinsics.Rot())* vcg::Point3d(-cc.Tx,cc.Ty,cc.Tz);

        shot->Extrinsics.SetTra(tl);
}

void TsaiMethods::Cam2Tsai(vcg::Shot<double> *s){

  cp.Ncx = s->Intrinsics.ViewportPx.X();	// [sel]     Number of sensor elements in camera's x direction //
  cp.Nfx = s->Intrinsics.ViewportPx.X();	// [pix]     Number of pixels in frame grabber's x direction   //
  cp.dx  = s->Intrinsics.PixelSizeMm.X();//*SCALE_FACTOR;	// [mm/sel]  X dimension of camera's sensor element (in mm)    //
  cp.dy  = s->Intrinsics.PixelSizeMm.Y();//*SCALE_FACTOR;	// [mm/sel]  Y dimension of camera's sensor element (in mm)    //

  cp.dpx = cp.dx * cp.Nfx/cp.Ncx;	// [mm/pix]  Effective X dimension of pixel in frame grabber   //
  cp.dpy = cp.dy;	// [mm/pix]  Effective Y dimension of pixel in frame grabber   //

  cp.Cx  = s->Intrinsics.CenterPx.X();	// [pix]     Z axis intercept of camera coordinate system      //
  cp.Cy  = s->Intrinsics.CenterPx.Y();	// [pix]     Z axis intercept of camera coordinate system      //

  cp.sx  = 1.0;	// []        Scale factor to compensate for any error in dpx   //

  cc.f = s->Intrinsics.FocalMm;// *SCALE_FACTOR;
  cc.kappa1 = s->Intrinsics.k[0];
}
