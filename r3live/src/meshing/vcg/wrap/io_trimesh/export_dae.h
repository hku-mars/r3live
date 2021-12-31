/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2004-2008                                           \/)\/    *
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

#ifndef EXPORTER_DAE_H
#define EXPORTER_DAE_H

#include <wrap/dae/xmldocumentmanaging.h>
#include <wrap/dae/colladaformat.h>
#include <wrap/dae/util_dae.h>

namespace vcg
{
namespace tri
{
namespace io
{
	template<typename MESHMODEL>
	class ExporterDAE
	{
	public:
		static int Save(const MESHMODEL& model,const char* filename,const int mask,const QDomDocument* doc = NULL)
		{
			XMLDocumentWriter stream(filename);
			if (stream.isReliable())
			{
				XMLDocument* document = Collada::DocumentManager::createColladaDocument(model,mask);
				stream.write(*document);
				Collada::DocumentManager::destroyColladaDocument(document);
				return UtilDAE::E_NOERROR;
			}
			else 
				return UtilDAE::E_CANTSAVE;
		}

		static int GetExportMaskCapability()
		{
			int capability = 0;

			//camera
			//capability |= MeshModel::IOM_CAMERA;

			//vert
			capability |= Mask::IOM_VERTCOORD;
			capability |= Mask::IOM_VERTNORMAL;
			capability |= Mask::IOM_VERTTEXCOORD;
			capability |= Mask::IOM_VERTCOLOR;
			//face

			////wedg
			capability |= Mask::IOM_WEDGNORMAL;
			capability |= Mask::IOM_WEDGTEXCOORD;

			return capability;
		}
	};
}
}
}
#endif
