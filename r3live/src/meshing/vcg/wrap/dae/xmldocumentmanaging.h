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
#ifndef _XML_DOCUMENT_MANAGING_H
#define _XML_DOCUMENT_MANAGING_H

#include <vector>
#include <set>
#include <QDomDocument>
#include<QFile>
#include<QVector>
#include <QXmlStreamWriter>
#include <QString>

class XMLTag
{
public:
	typedef std::pair<QString,QString> TagAttribute;
	typedef QVector<TagAttribute> TagAttributes;
	QString _tagname;
	TagAttributes _attributes;

	XMLTag(const QString& tagname = QString(),const TagAttributes& attr = TagAttributes())
		:_tagname(tagname),_attributes(attr)
	{
	}

	virtual ~XMLTag()
	{
	}
};

class XMLLeafTag : public XMLTag
{
public:
		QVector<QString> _text;

		XMLLeafTag(const QString& tagname = QString(),const QVector<QString>& text = QVector<QString>())
			:XMLTag(tagname),_text(text)
		{
		}

		virtual ~XMLLeafTag()
		{
		}
};

class XMLDocumentWriter;

class XMLVisitor;

class XMLNode
{
public:
	XMLNode(XMLTag* tag);
	virtual ~XMLNode();

	virtual void applyProcedure(XMLVisitor& v) = 0;

	XMLTag* _tag;
};

class XMLInteriorNode : public XMLNode
{
public:
	XMLInteriorNode(XMLTag* tag);

	XMLNode* son(int ii);

	QVector< XMLNode* > sons();

	void applyProcedure(XMLVisitor& v);

	~XMLInteriorNode();

	QVector< XMLNode* > _sons; 
};

class XMLLeafNode : public XMLNode
{
public:
	XMLLeafNode(XMLLeafTag* leaftag);

	void applyProcedure(XMLVisitor& v);
	virtual ~XMLLeafNode();
};

class XMLDocument
{
public:
	XMLDocument(XMLInteriorNode* root)
		:_root(root)
	{
	}

	~XMLDocument()
	{
		delete (_root);
	}

	XMLInteriorNode* _root;
};


class XMLVisitor
{
public:
	virtual void operator()(XMLLeafNode& leaf) = 0;
	virtual void operator()(XMLInteriorNode& intnode) = 0;
};


class XMLDocumentWriter : public XMLVisitor
{
private:
	QXmlStreamWriter _stream;
	QFile _file;
	bool _error;

	void writeText(XMLLeafNode& node)
	{
		XMLLeafTag* leaftag = static_cast<XMLLeafTag*>(node._tag);
		for(QVector<QString>::iterator it = leaftag->_text.begin();it != leaftag->_text.end();++it)
		{
			QString tmp = "";
			if (it != leaftag->_text.begin()) 
				tmp = QString(" ");
			_stream.writeCharacters(tmp + *it);
		}
	}

	void writeAttributes(XMLNode& node)
	{
		QXmlStreamAttributes attr;
		for(XMLTag::TagAttributes::iterator it = node._tag->_attributes.begin();it != node._tag->_attributes.end();++it)
			attr.append(it->first,it->second);
		_stream.writeAttributes(attr);
	}

	void recursiveStep(XMLInteriorNode& intnode)
	{
		QVector< XMLNode* > sons = intnode.sons();
		for(QVector< XMLNode* >::iterator its = sons.begin();its != sons.end();++its)
			(*its)->applyProcedure(*this);
	}

public:

	void operator()(XMLLeafNode& node)
	{
		_stream.writeStartElement(node._tag->_tagname);
		writeAttributes(node);
		writeText(node);
		_stream.writeEndElement();
	}

	void operator()(XMLInteriorNode& intnode)
	{
		_stream.writeStartElement(intnode._tag->_tagname);
		writeAttributes(intnode);
		recursiveStep(intnode);
		_stream.writeEndElement();
	}

	void operator()(XMLDocument& doc)
	{
		_stream.writeStartDocument();
		operator()(*(doc._root));
		_stream.writeEndDocument();
	}

	void write(XMLDocument& doc)
	{
		(*this)(doc);
	}

	bool isReliable() const 
	{
		return !_error;
	}

	XMLDocumentWriter(const char* filename,const bool autoformatting = true)
		:_stream(),_file(filename),_error(false)
	{
		if (!_file.open(QIODevice::WriteOnly | QIODevice::Text))
			_error = true;
		_stream.setDevice(&_file);
		_stream.setAutoFormatting(autoformatting);
	}

  virtual ~XMLDocumentWriter()
	{
		_file.close();
	}
};

#endif
