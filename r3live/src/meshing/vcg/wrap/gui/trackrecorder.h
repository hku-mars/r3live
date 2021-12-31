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
Revision 1.1  2005/04/14 21:23:39  ganovelli
*** empty log message ***



****************************************************************************/
#ifndef VCGLIB_TRACKRECORDER
#define VCGLIB_TRACKRECORDER


#include <wrap/gui/trackball.h>
#include <stdio.h>
#include <time.h>

namespace vcg{
struct TrackRecorder{

	TrackRecorder(){Stop();}
	
	FILE * trackfile;
	
	enum { PLAY,REC,OFF } mode;
	int nextTime,   
			startTime;  
 
	void StartPlaying(char * namefile){
		if(trackfile != NULL) return;

		trackfile = fopen(namefile,"rb");
		startTime = clock();
		mode = PLAY;
		fread(&nextTime,4,1,trackfile);
	} 

	void UpdateTrackball(Trackball & t){

		while( ( clock()-startTime > nextTime)&& !feof(trackfile)){
				fread(&t.track,sizeof(float)*4 + sizeof(float)*5,1,trackfile);
				fread(&nextTime,4,1,trackfile);
		}
		if(feof(trackfile))
			Stop();
	}

	void  StartRecording(char * namefile){
		if(trackfile != NULL) return;
		trackfile = fopen(namefile,"wb");	
		startTime = clock();
		mode = REC;
	} 

	void  RecordTrackball(Trackball & t){
		nextTime = clock()-startTime;
		fwrite(&nextTime,4,1,trackfile);
		fwrite(&t.track,sizeof(float)*4 + sizeof(float)*5,1,trackfile);
	}

	void  Stop(){mode = OFF; trackfile = NULL;};

};
}
#endif