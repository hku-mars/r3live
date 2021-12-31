/****************************************************************************
* MeshLab                                                           o o     *
* An extendible mesh processor                                    o     o   *
*                                                                _   O  _   *
* Copyright(C) 2005, 2009                                          \/)\/    *
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


#include <wrap/bmt/bmt.h>

using namespace std;
using namespace vcg;

Bmt::Bmt(): fp(NULL) {}
Bmt::~Bmt() {}

bool Bmt::Load(const std::string &filename) {
  fp = fopen(filename.c_str(), "rb");
  if(!fp) return false;
  unsigned int magic;
  fread(&magic, sizeof(unsigned int), 1, fp);
  if(magic != 0x62647421) //bmt!
    return false;
  //signature
  fread(&signature, sizeof(unsigned int), 1, fp);
  //sphere
  fread(&sphere, sizeof(Sphere3f), 1, fp);
  //index and history offsets and size;  
  fread(&index_offset, sizeof(unsigned int), 1, fp);
  fread(&index_size, sizeof(unsigned int), 1, fp);
  fread(&history_offset, sizeof(unsigned int), 1, fp);
  fread(&history_size, sizeof(unsigned int), 1, fp);

  //index
  index.resize(index_size);
  fread(&index[0], sizeof(Bmt::Cell), index.size(), fp);

  //history
  history.resize(history_size);  
  for(unsigned int i = 0; i < history.size(); i++) {
    vector<Bmt::Cell *> &created = history[i].created;
    vector<Bmt::Cell *> &erased = history[i].erased;    

    unsigned int created_size;
    fread(&created_size, sizeof(unsigned int), 1, fp);
    created.resize(created_size);
    fread(&*created.begin(), sizeof(unsigned int), created.size(), fp);
    for(unsigned int k = 0; k < created_size; k++) 
      created[k] = &(index[(unsigned int)created[k]]);
    
    unsigned int erased_size;
    fread(&erased_size, sizeof(unsigned int), 1, fp);
    erased.resize(erased_size);
    fread(&*erased.begin(), sizeof(unsigned int), erased.size(), fp);
    for(unsigned int k = 0; k < erased_size; k++) 
      erased[k] = &(index[(unsigned int)erased[k]]);        
  }  
  return true;
}
      
char *Bmt::GetData(unsigned int &size, unsigned int offset) {
  fseek(fp, offset, SEEK_SET);
  fread(&size, sizeof(unsigned int), 1, fp);
  assert(size < 64000);
  char *data = new char[size];
  fread(data, 1, size, fp);
  return data;
}

/*unsigned int &Bmt::IndexOffset();
unsigned int &Bmt::IndexSize();
Bmt::Cell *Bmt::IndexBegin();

unsigned int &Bmt::HistoryOffset();  
unsigned int &Bmt::HistorySize();
unsigned int *Bmt::HistoryBegin();        */

/**** BMT BUILDER ****/

BmtBuilder::BmtBuilder(): ftmp(NULL), fout(NULL) {}

BmtBuilder::~BmtBuilder() {}
  
bool BmtBuilder::Create(unsigned int sign) {
  signature = sign;
  ftmp = fopen("tmp.bmt", "wb+");
  if(!ftmp)
    return false;
  return true;
}

unsigned int BmtBuilder::AddCell(Bmt::Cell cell, unsigned int size, char *data) {  
  sphere.Add(cell.sphere);
  cell.offset = ftell(ftmp);
  index.push_back(cell);
  //TODO: add padding to 4 or 8 bytes.
  fwrite(&size, sizeof(unsigned int), 1, ftmp);
  fwrite(data, 1, size, ftmp);    
  return index.size()-1;
}

void BmtBuilder::AddUpdate(std::vector<unsigned int> &created, std::vector<unsigned int> &erased) {
  creation.push_back(created);
  deletion.push_back(erased);
}

bool BmtBuilder::Save(const std::string &filename) {  
  assert(ftmp);
  
  //TODO: reorganize data to be spatially coherent (both index and related data.

  fout = fopen(filename.c_str(), "wb+");
  if(!fout) {
    fclose(ftmp);
    return false;
  }
  //first thing we write a magic constant "bmt!"
  unsigned int magic = 0x62647421; //bmt!  
  fwrite(&magic, sizeof(unsigned int), 1, fout);

  //signature:
  fwrite(&signature, sizeof(unsigned int), 1, fout);
  //sphere
  fwrite(&sphere, sizeof(Sphere3f), 1, fout);

  //next we write index and history offset and size
  unsigned int index_offset = 5 * sizeof(unsigned int) + sizeof(Sphere3f);
  unsigned int index_size = index.size();
  fwrite(&index_offset, sizeof(unsigned int), 1, fout);
  fwrite(&index_size, sizeof(unsigned int), 1, fout);

  unsigned int history_offset = index_offset + index_size * sizeof(Bmt::Cell);
  unsigned int history_size = creation.size();
  fwrite(&history_offset, sizeof(unsigned int), 1, fout);
  fwrite(&history_size, sizeof(unsigned int), 1, fout);

  unsigned int index_start = ftell(fout);
  //writing index (but its a fake... it will need to be rewritten late5r with correct offsets
  fwrite(&index[0], sizeof(Bmt::Cell), index.size(), fout);

  //writing history
  for(unsigned int i = 0; i < creation.size(); i++) {
    vector<unsigned int> &created = creation[i];
    vector<unsigned int> &erased = deletion[i];
    unsigned int created_size = created.size();
    fwrite(&created_size, sizeof(unsigned int), 1, fout);
    fwrite(&*created.begin(), sizeof(unsigned int), created.size(), fout);
    unsigned int erased_size = erased.size();
    fwrite(&erased_size, sizeof(unsigned int), 1, fout);
    fwrite(&*created.begin(), sizeof(unsigned int), erased.size(), fout);
  }
  //writing data
  vector<Bmt::Cell>::iterator k;
  for(k = index.begin(); k != index.end(); k++) {
    Bmt::Cell &cell = *k;    
    fseek(ftmp, cell.offset, SEEK_SET);
    unsigned int size;
    fread(&size, sizeof(unsigned int), 1, ftmp);
    char *data = new char[size];
    fread(data, 1, size, ftmp);
    cell.offset = ftell(fout);
    fwrite(&size, sizeof(unsigned int), 1, fout);
    fwrite(data, 1, size, fout);
    delete []data;
  }
  //writing index again
  fseek(fout, index_start, SEEK_SET);
  fwrite(&index[0], sizeof(Bmt::Cell), index.size(), fout);

  fclose(fout);
  return true;
}
