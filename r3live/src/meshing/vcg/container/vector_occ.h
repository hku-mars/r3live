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

#ifndef __VCGLIB_TRACED_VECTOR__
#define __VCGLIB_TRACED_VECTOR__ 


#include <vcg/container/container_allocation_table.h>
#include <vcg/container/entries_allocation_table.h>

#include <assert.h>

namespace vcg {
	/*@{*/
/*!
 * This class represent a vector_occ. A vector_occ is derived by a std::vector.
 * The characteristic of a vector_occ is that you can add (at run time) new attributes
 * to the container::value_type elements contained in the vector. (see the example..)
 * The position in memory of a traced vector is kept by the Container Allocation Table,
 * which is a (unique) list of vector_occ positions.
 */

template <class VALUE_TYPE>
class vector_occ: public std::vector<VALUE_TYPE>{
	typedef  vector_occ<VALUE_TYPE> ThisType;
	typedef  std::vector<VALUE_TYPE> TT;	

public:
	vector_occ():std::vector<VALUE_TYPE>(){id = ID(); ID()=ID()+1; reserve(1);}
	~vector_occ();
	
	VALUE_TYPE * Pointer2begin(){
		if(TT::empty()) return (VALUE_TYPE *)id; else return &*std::vector<VALUE_TYPE>::begin();
	}
	std::list < CATBase<ThisType>* > attributes;
	// override di tutte le funzioni che possono spostare 
	// l'allocazione in memoria del container
	void push_back(const VALUE_TYPE & v);
	void pop_back();
	void resize(const unsigned int & size);
	void reserve(const unsigned int & size);

	/// this function enable the use of an optional attribute (see...)
	template <class ATTR_TYPE>
		void Enable(){
			CAT<ThisType,ATTR_TYPE> * cat = CAT<ThisType,ATTR_TYPE>::New();
			cat->Insert(*this);
			attributes.push_back(cat);
			}

	/// this function returns true if the attribute in the template parameter is enabled
  /// Note: once an attribute is disabled, its data is lost (the memory freed)
	template <class ATTR_TYPE>
		bool IsEnabled() const{
				typename std::list < CATBase<ThisType> * >::const_iterator ia; 
				for(ia = attributes.begin(); ia != attributes.end(); ++ia)
					if((*ia)->Id() == CAT<ThisType,ATTR_TYPE>::Id())
						return true;
				return false;
				}


	/// this function disable the use of an optional attribute (see...)
  /// Note: once an attribute is disabled, its data is lost (the memory freed)
	template <class ATTR_TYPE>
		void Disable(){
				typename std::list < CATBase<ThisType> * >::iterator ia; 
				for(ia = attributes.begin(); ia != attributes.end(); ++ia)
					if((*ia)->Id() == CAT<ThisType,ATTR_TYPE>::Id())
						{
							(*ia)->Remove(*this);
							//delete (*ia);
							attributes.erase(ia);
							break;
						}
				}

private:	
	VALUE_TYPE * old_start;
	int id;
	static int & ID(){static int id; return id;} 
	void Update();
};

	/*@}*/

template <class VALUE_TYPE>
void vector_occ<VALUE_TYPE>::push_back(const VALUE_TYPE & v){
	std::vector<VALUE_TYPE>::push_back(v);
	Update();	
	typename std::list < CATBase<ThisType> * >::iterator ia; 
	for(ia = attributes.begin(); ia != attributes.end(); ++ia)
		(*ia)->AddDataElem(&(*(this->begin())),1);

}
template <class VALUE_TYPE>
void vector_occ<VALUE_TYPE>::pop_back(){
	std::vector<VALUE_TYPE>::pop_back();
	Update();
}

template <class VALUE_TYPE>
void vector_occ<VALUE_TYPE>::resize(const unsigned int & size){
	std::vector<VALUE_TYPE>::resize(size);
	Update();
	typename std::list < CATBase<ThisType> * >::iterator ia; 
	for(ia = attributes.begin(); ia != attributes.end(); ++ia)
		(*ia)->
	Resize(&(*(this->begin())),size);
}

template <class VALUE_TYPE>
void vector_occ<VALUE_TYPE>::reserve(const unsigned int & size){
	std::vector<VALUE_TYPE>::reserve(size);
	Update();
}

template <class VALUE_TYPE>
	void vector_occ<VALUE_TYPE>::
		Update(){
		typename std::list < CATBase<ThisType> * >::iterator ia; 
		if(Pointer2begin() != old_start)
			for(ia = attributes.begin(); ia != attributes.end(); ++ia)
				(*ia)->Resort(old_start,Pointer2begin());

		old_start = Pointer2begin();
	}



template <class VALUE_TYPE>
vector_occ<VALUE_TYPE>::~vector_occ(){
		typename std::list < CATBase<ThisType> * >::iterator ia; 
		for(ia = attributes.begin(); ia != attributes.end(); ++ia)
			{	
				(*ia)->Remove(*this);
				delete *ia;
			}
		}

}; // end namespace
#endif
