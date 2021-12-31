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

#ifndef __GL_OBJECT_H__
#define __GL_OBJECT_H__

class GLObject
{
public:
	GLObject(void)
	{
		this->objectID = 0;
	}

	virtual ~GLObject(void)
	{
	}

	GLuint ObjectID(void) const
	{
		return this->objectID;
	}

	bool ValidObject(void) const
	{
		return (this->objectID != 0);
	}

	virtual void Gen(void) = 0;
	virtual void Del(void) = 0;

protected:
	GLuint objectID;
};

class Bindable
{
public:
	Bindable(void)
	{
		this->bound = false;
	}

	void Bind(void)
	{
		this->bound = true;
		this->DoBind();
	}

	void Unbind(void)
	{
		this->DoUnbind();
		this->bound = false;
	}

	bool IsBound(void) const
	{
		return this->bound;
	}

protected:
	bool bound;

	virtual void DoBind(void) = 0;
	virtual void DoUnbind(void) = 0;
};

#endif //__GL_OBJECT_H__
