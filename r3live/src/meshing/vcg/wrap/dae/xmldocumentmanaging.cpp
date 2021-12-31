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
#include <wrap/dae/xmldocumentmanaging.h>
#include <cassert>

XMLNode::XMLNode(XMLTag* tag)
:_tag(tag)
{
}

XMLNode::~XMLNode()
{
	delete _tag;
}

XMLLeafNode::XMLLeafNode(XMLLeafTag* leaftag)
:XMLNode(leaftag)
{
}

XMLLeafNode::~XMLLeafNode()
{

}


void XMLLeafNode::applyProcedure(XMLVisitor& v)
{
	v(*this);
}

XMLInteriorNode::XMLInteriorNode(XMLTag* tag)
:XMLNode(tag)
{
}

XMLNode* XMLInteriorNode::son(int ii)
{
	assert((ii > 0) && (ii < _sons.size()));
	return _sons[ii];
}

QVector< XMLNode* > XMLInteriorNode::sons()
{
	return _sons;
}

XMLInteriorNode::~XMLInteriorNode()
{
	for(QVector< XMLNode* >::iterator it = _sons.begin();it != _sons.end();++it)
		delete (*it);
}

void XMLInteriorNode::applyProcedure(XMLVisitor& v)
{
	v(*this);
}
