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

#ifndef __VCGLIB_SIMPLE__
#define __VCGLIB_SIMPLE__

#include <string.h>

namespace vcg {

class SimpleTempDataBase{
public:
  virtual ~SimpleTempDataBase() {}
  SimpleTempDataBase() {}
    virtual void Resize(const int & sz) = 0;
    virtual void Reorder(std::vector<size_t> & newVertIndex)=0;
    virtual int SizeOf() const  = 0;
    virtual void * DataBegin() = 0;
    virtual void * At(unsigned int i ) = 0;
};

template <class TYPE>
class VectorNBW: public std::vector<TYPE> {};

template <>
class VectorNBW<bool>{
public:
    VectorNBW():data(0),datasize(0),datareserve(0){}
    ~VectorNBW() { delete[] data;}
    bool * data;

    void reserve (const int & sz)	{
        if(sz<=datareserve) return;
        bool * newdataLoc = new bool[ sz ];
        if(datasize!=0) memcpy(newdataLoc,data,sizeof(datasize));
        std::swap(data,newdataLoc);
        if(newdataLoc != 0) delete[] newdataLoc;
        datareserve = sz;
    }

    void resize  (const int & sz)	{
        int oldDatasize = datasize;
        if(sz <= oldDatasize) return;
        if(sz > datareserve)
            reserve(sz);
        datasize = sz;
        memset(&data[oldDatasize],0,datasize-oldDatasize);
        }
    void push_back(const bool & v)	{ resize(datasize+1); data[datasize] = v;}

    void clear(){ datasize = 0;}

    unsigned int  size() const { return datasize;}

    bool empty() const {return datasize==0;}

    bool * begin() const {return data;}

    bool  & operator [](const int & i){return data[i];}

private:
    int datasize;
    int datareserve;
};

template <class STL_CONT, class ATTR_TYPE>
class SimpleTempData:public SimpleTempDataBase{

    public:
    typedef SimpleTempData<STL_CONT,ATTR_TYPE> SimpTempDataType;
    typedef ATTR_TYPE AttrType;

    STL_CONT& c;
    VectorNBW<ATTR_TYPE> data;
    int padding;

    SimpleTempData(STL_CONT  &_c):c(_c),padding(0){data.reserve(c.capacity());data.resize(c.size());};
    SimpleTempData(STL_CONT  &_c, const ATTR_TYPE &val):c(_c){
        data.reserve(c.capacity());data.resize(c.size());
        Init(val);
    };

    ~SimpleTempData(){data.clear();}

    void Init(const ATTR_TYPE &val)
    {
        std::fill(data.begin(),data.end(),val);
    }
    // access to data
    ATTR_TYPE & operator[](const typename STL_CONT::value_type & v){return data[&v-&*c.begin()];}
    ATTR_TYPE & operator[](const typename STL_CONT::value_type * v){return data[v-&*c.begin()];}
    ATTR_TYPE & operator[](const typename STL_CONT::iterator & cont){return data[&(*cont)-&*c.begin()];}
    ATTR_TYPE & operator[](const int & i){return data[i];}

    void * At(unsigned int i ) {return &(*this)[i];};

    // update temporary data size
    bool UpdateSize(){
            if(data.size() != c.size())
                {
                    data.resize(c.size());
                    return false;
                }
            return true;
        }

    void Resize(const int & sz){
        data.resize(sz);
    }

    void Reorder(std::vector<size_t> & newVertIndex){
        for(unsigned int i = 0 ; i < data.size(); ++i){
            if( newVertIndex[i] != (std::numeric_limits<size_t>::max)())
                data[newVertIndex[i]] = data[i];
        }
    }

    int SizeOf() const {return sizeof(ATTR_TYPE);}
    void * DataBegin() {return data.empty()?NULL:&(*data.begin());}
};

template <class ATTR_TYPE>
class Attribute: public SimpleTempDataBase   {
public:
    typedef ATTR_TYPE AttrType;
    AttrType * attribute;
    Attribute(){attribute = new ATTR_TYPE();}
    ~Attribute(){delete attribute;}
    int SizeOf()const {return sizeof(ATTR_TYPE);}
    void * DataBegin(){return attribute;}

    void Resize(const int &  ) {assert(0);}
    void Reorder(std::vector<size_t> &  ){assert(0);}
    void * At(unsigned int ) {assert(0);return (void*)0;}
};

} // end namespace vcg

#endif
