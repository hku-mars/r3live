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

****************************************************************************/

#ifndef VCG_BMT_H
#define VCG_BMT_H

#include <string>
#include <vector>

#include <vcg/space/sphere3.h>
#include <wrap/mt/mt.h>

namespace vcg {

/** \addtogroup bmt */
/*@{*/
    /**
        The class for representing a batched mt structure.
        See bmt.cpp for details on the structure of the file.
     */

class Bmt {
public:    
  ///Cell structure for the mt representation
  class Cell {
  public:
    //this effectively limits databases to 4Gb in size.
    unsigned int offset; 
    float error;
    Sphere3f sphere;
    float Error() { return error; }
  };

  ///tells what is inside each cell.
  unsigned int signature;
  ///Bounding sphere
  Sphere3f sphere;

  std::vector<Cell> index;
  std::vector< MT<Bmt::Cell>::Update > history;
  


  Bmt();
  ~Bmt();
  bool Load(const std::string &filename);      
  char *GetData(unsigned int &size, unsigned int offset);  
  
private:
  FILE *fp;
  unsigned int index_offset;
  unsigned int index_size;
  unsigned int history_offset;
  unsigned int history_size;  
};

class BmtBuilder {
public:
  BmtBuilder();
  ~BmtBuilder();

  ///tells what is inside each cell.
  unsigned int signature;
  ///Bounding sphere
  Sphere3f sphere;
  std::vector<Bmt::Cell> index;
  std::vector<std::vector<unsigned int > > creation;
  std::vector<std::vector<unsigned int > > deletion;
  
  bool Create(unsigned int signature);

  unsigned int AddCell(Bmt::Cell cell, unsigned int size, char *data);
  void AddUpdate(std::vector<unsigned int> &created, std::vector<unsigned int> &erased);

  bool Save(const std::string &filename);
private:
  FILE *ftmp;
  FILE *fout;
};

float Distance(Bmt::Cell &cell, Point3f &p);

}//end namespace

#endif