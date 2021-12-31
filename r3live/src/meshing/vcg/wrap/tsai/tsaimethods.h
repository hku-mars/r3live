/***************************************************************************
 *   Copyright (C) 2008 by Luca Baronti  				   *
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


/** <b>Tsai Methods -- Interface with the tsai lib </b>

The class is static, so a simple call to <code>calibrate()</code> is
sufficient to get a calibrated shot.<br>

<i>Comment:</i><br>
Sometimes the Tsai Lib returns a irregular shifted translation values.<br>
Since now no solution found.

*/


/****************************************************************************\
*										*
* Camera parameters are usually the fixed parameters of the given camera 	*
* system, typically obtained from manufacturers specifications.			*
*										*
* Cy and Cy (the center of radial lens distortion), may be calibrated		*
* separately or as part of the coplanar/noncoplanar calibration.		*
* The same with sx (x scale uncertainty factor).				*
*										*
\*******************************************************************************/

#ifndef _TSAI_METHODS_
#define _TSAI_METHODS_

#include <vcg/math/matrix44.h>
#include <vcg/space/point3.h>
#include <vcg/space/point2.h>
#include <vcg/math/shot.h>

#include <list>

// minimum points that need tsai for a proper calibration
static const int MIN_POINTS_FOR_CALIBRATE = 13;

struct TsaiCorrelation {
	vcg::Point3d point3d;
	vcg::Point2d point2d;
};

class TsaiMethods
{
public:
	//Calibration of the shot according to the 2D and 3D points
    static bool calibrate( vcg::Shot<double>* shot,std::list<TsaiCorrelation>* corr, bool p_foc); /// send all the valid binded points of the image at the calibration lib, then set the intinsic param of the image

	///Transformation of the camera data between tsai structure and vcg structure
	static void Shot2Tsai(vcg::Shot<double>*);
	///Transformation of the camera data between tsai structure and vcg structure
	static void Tsai2Shot(vcg::Shot<double>*, bool p_foc=true);
	
private:
	static void Cam2Tsai(vcg::Shot<double>*);
        static bool createDataSet(std::list<TsaiCorrelation>* corr,vcg::Shot<double>* s);

};

#endif //_TSAI_METHODS_
