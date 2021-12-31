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
Revision 1.3  2004/05/12 10:18:55  ganovelli
new line added at the end of file

Revision 1.2  2004/05/10 14:26:33  ganovelli
argument name removed to avoid "unreference variable"warning

Revision 1.1  2004/03/03 15:00:51  cignoni
Initial commit

****************************************************************************/

#ifndef __VCGLIB_CALLBACK
#define __VCGLIB_CALLBACK

namespace vcg {
// Generic Callback function:
// Used to make algorithms interumpable
// Return value: true continue, false break
// The second callback is to know where we are (useful for progress bar)
typedef bool CallBack( const char * str );
typedef bool CallBackPos(const int pos, const char * str );

inline bool DummyCallBack( const char * ) {return true;}
inline bool DummyCallBackPos(const int ,  const char * ) {return true;}

/// interruptible function 
typedef void TaskFunc(int t,void * contest); 
	
}	// End namespace


#endif
