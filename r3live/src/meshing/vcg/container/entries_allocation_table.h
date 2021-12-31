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

#ifndef __VCGLIB_ENTRIES__
#define __VCGLIB_ENTRIES__


namespace vcg {

// EntryCATBase: base class for the entry of the allocation table
// templated over the container type
template <class STL_CONT>
class EntryCATBase{
public:
EntryCATBase(STL_CONT & _c):c(_c){};
typename STL_CONT::value_type * Start() const;
virtual bool Empty(){return true;};
const STL_CONT *  C();
virtual void Push_back(const int &){};

virtual void Reserve(const int & s){};
virtual void Resize(const int & s){};

const bool operator < (const EntryCATBase<STL_CONT> & other) const;

private:
	STL_CONT & c;
};

//EntryCAT: entry for the case of optional core types (matches with CAT)
template <class STL_CONT,class ATTR_TYPE >
struct EntryCAT: public EntryCATBase<STL_CONT>{
typedef ATTR_TYPE attr_type;
EntryCAT(STL_CONT & _c) : EntryCATBase<STL_CONT>(_c){};
std::vector<ATTR_TYPE> & Data(){return data;}
void Push_back(const int & n){ for(int i = 0; i < n ; ++i) data.push_back(ATTR_TYPE());}
virtual void Reserve(const int & s){data.reserve(s);};
virtual void Resize(const int & s){data.resize(s);};


private:
std::vector<ATTR_TYPE> data;
};

//----------------------EntryCAT: implementation ----------------------------------------
template <class STL_CONT>
const bool EntryCATBase<STL_CONT>:: operator < (const EntryCATBase<STL_CONT> & other) const{
	return (Start() < other.Start());
}

template <class STL_CONT>
 typename STL_CONT::value_type  * EntryCATBase<STL_CONT>::Start()const {
	return  c.Pointer2begin();
	}

template <class STL_CONT>
	const STL_CONT * EntryCATBase<STL_CONT>::C(){
	return &c;
	}



}; // end namespace vcg

#endif
