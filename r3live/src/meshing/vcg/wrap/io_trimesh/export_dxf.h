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

#ifndef __VCGLIB_EXPORT_DXF
#define __VCGLIB_EXPORT_DXF

namespace vcg {
namespace tri {
namespace io {

template <class SaveMeshType>
/**
This class encapsulate a filter for save dxf meshes.
*/
class ExporterDXF
{
public:
  ///Standard call for saving a mesh
  static int Save(SaveMeshType &m, const char * filename)
  {
    if(m.fn==0 && m.en != 0) return SaveEdge(m,filename);

    FILE * o = fopen(filename,"w");
    if(o==NULL)	return 1;
    fprintf(o,"0\n");
    fprintf(o,"SECTION\n");
    fprintf(o,"2\n");
    fprintf(o,"ENTITIES\n");

    typename SaveMeshType::FaceIterator fi;
    for(fi=m.face.begin(); fi!=m.face.end(); ++fi)
    {
      if (!fi->IsD())
      {
        typename SaveMeshType::CoordType v0 = (*fi).V(0)->P();
        typename SaveMeshType::CoordType v1 = (*fi).V(1)->P();
        typename SaveMeshType::CoordType v2 = (*fi).V(2)->P();
        fprintf(o,"0\n");  fprintf(o,"3DFACE\n");  fprintf(o,"8\n");     fprintf(o,"0\n");
        fprintf(o,"10\n"); fprintf(o,"%f\n", v0[0]);     //X
        fprintf(o,"20\n"); fprintf(o,"%f\n", v0[1]);     //Y
        fprintf(o,"30\n"); fprintf(o,"%f\n", v0[2]);     //Z

        fprintf(o,"11\n"); fprintf(o,"%f\n", v1[0]);     //X
        fprintf(o,"21\n"); fprintf(o,"%f\n", v1[1]);     //Y
        fprintf(o,"31\n"); fprintf(o,"%f\n", v1[2]);     //Z

        fprintf(o,"12\n"); fprintf(o,"%f\n", v2[0]);     //X
        fprintf(o,"22\n"); fprintf(o,"%f\n", v2[1]);     //Y
        fprintf(o,"32\n"); fprintf(o,"%f\n", v2[2]);     //Z

        fprintf(o,"13\n"); fprintf(o,"%f\n", v2[0]);     //X
        fprintf(o,"23\n"); fprintf(o,"%f\n", v2[1]);     //Y
        fprintf(o,"33\n"); fprintf(o,"%f\n", v2[2]);     //Z
      }
    }

    fprintf(o,"0\n");
    fprintf(o,"ENDSEC\n");
    fprintf(o,"0\n");
    fprintf(o,"EOF\n");
    fclose(o);
    return 0;
  }
  /// Standard call for knowing the meaning of an error code
  static const char *ErrorMsg(int error)
  {
    static std::vector<std::string> dxf_error_msg;
    if(dxf_error_msg.empty())
    {
      dxf_error_msg.resize(2 );
      dxf_error_msg[0]="No errors";
      dxf_error_msg[1]="Can't open file";
    }

    if(error>1 || error<0) return "Unknown error";
    else return dxf_error_msg[error].c_str();
  }


  static bool SaveEdge(SaveMeshType  &mp, const char * filename)
  {
    FILE * o = fopen(filename,"w");
    if(o==NULL)	return 1;
    fprintf(o,"0\n");
    fprintf(o,"SECTION\n");
    fprintf(o,"2\n");
    fprintf(o,"ENTITIES\n");

    typename SaveMeshType::EdgeIterator ei;
    for(ei=mp.edge.begin(); ei!=mp.edge.end();++ei)
    {
      Point3f p1 = (*ei).V(0)->P();
      Point3f p2 = (*ei).V(1)->P();

      fprintf(o,"0\n");
      fprintf(o,"LINE\n");
      fprintf(o,"8\n");
      fprintf(o,"0\n");
      fprintf(o,"10\n");

      fprintf(o,"%f\n", p1[0]);     //X
      fprintf(o,"20\n");
      fprintf(o,"%f\n", p1[1]);     //Y
      fprintf(o,"30\n");
      fprintf(o,"%f\n", p1[2]);     //Z

      fprintf(o,"11\n");
      fprintf(o,"%f\n", p2[0]);     //X
      fprintf(o,"21\n");
      fprintf(o,"%f\n", p2[1]);     //Y
      fprintf(o,"31\n");
      fprintf(o,"%f\n", p2[2]);     //Z
    }

    fprintf(o,"0\n");
    fprintf(o,"ENDSEC\n");
    fprintf(o,"0\n");
    fprintf(o,"EOF\n");
    fclose(o);
    return true;
  }


}; // end class

} // end Namespace io
} // end Namespace tri
} // end Namespace vcg

#endif
