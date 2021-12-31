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

#ifndef __SHADERS_H__
#define __SHADERS_H__

#include <GL/glew.h>
#include <stdio.h>
#include <set>

#include "gl_object.h"
#include "../../vcg/space/point2.h"
#include "../../vcg/space/point3.h"
#include "../../vcg/space/point4.h"

class Shader : public GLObject, public Bindable
{
public:
	typedef enum
	{
		VERTEX,
		FRAGMENT,
		GEOMETRY
	} ShaderType;

	Shader(void) : GLObject(), Bindable()
	{
		this->flags = 0;
		this->flags |= SOURCE_DIRTY;
		this->compiled = false;
	}

	void Gen(void)
	{
		this->Del();
		GLenum t;
		switch (this->Type())
		{
			case Shader::VERTEX   : t = GL_VERTEX_SHADER;   break;
			case Shader::FRAGMENT : t = GL_FRAGMENT_SHADER; break;
			case Shader::GEOMETRY : t = GL_GEOMETRY_SHADER_EXT; break;
			default: return;
		};
		this->objectID = glCreateShader(t);
	}

	void Del(void)
	{
		if (this->objectID == 0) return;
		glDeleteShader(this->objectID);
		this->objectID = 0;
	}

	virtual ShaderType Type(void) const = 0;

	void SetSource(const char * src)
	{
		if (this->objectID==0)
			Gen();

		this->flags |= SOURCE_DIRTY;
		this->compiled = false;
		this->source = src;

		const char * pSrc = this->source.c_str();
		glShaderSource(this->objectID, 1, &pSrc, 0);
	}

	bool LoadSource(const char * fileName)
	{
		if (this->objectID==0)
			Gen();

		this->flags |= SOURCE_DIRTY;
		this->compiled = false;
		FILE * f = fopen(fileName, "rb");
		if (f == 0)
		{
			this->source = "";
			return false;
		}
		fseek(f, 0, SEEK_END);
		const size_t sz = (size_t)ftell(f);
		rewind(f);
		char * buff = new char[sz + 1];
		fread(buff, sizeof(char), sz, f);
		fclose(f);
		buff[sz] = '\0';

		this->source = buff;
		delete [] buff;

		const char * pSrc = this->source.c_str();
		glShaderSource(this->objectID, 1, &pSrc, 0);

		return true;
	}

	bool Compile(void)
	{
		glCompileShader(this->objectID);
		GLint cm = 0;
		glGetShaderiv(this->objectID, GL_COMPILE_STATUS, &cm);
		this->compiled = (cm != GL_FALSE);
		this->flags = 0;
		return this->compiled;
	}

	bool IsCompiled(void)
	{
		return this->compiled;
	}

	std::string InfoLog(void)
	{
		GLint len = 0;
		glGetShaderiv(this->objectID, GL_INFO_LOG_LENGTH, &len);
		char * ch = new char[len + 1];
		glGetShaderInfoLog(this->objectID, len, &len, ch);
		std::string infoLog = ch;
		delete [] ch;
		return infoLog;
	}

protected:
	enum
	{
		SOURCE_DIRTY
	};

	std::string source;
	unsigned int flags;
	bool compiled;

	void DoBind(void)
	{
	}

	void DoUnbind(void)
	{
	}
};

class VertexShader : public Shader
{
public:
	VertexShader(void) : Shader()
	{
	}

	ShaderType Type(void) const
	{
		return Shader::VERTEX;
	}
};

class FragmentShader : public Shader
{
public:
	FragmentShader(void) : Shader()
	{
	}

	ShaderType Type(void) const
	{
		return Shader::FRAGMENT;
	}
};

class GeometryShader : public Shader
{
public:
	GeometryShader(void) : Shader()
	{
	}

	ShaderType Type(void) const
	{
		return Shader::GEOMETRY;
	}
};

#if 0
class Program;

class Uniform
{
friend class Program;

public:
	/*
	typedef enum
	{
		U_BOOL,
		U_BVEC2,
		U_BVEC3,
		U_BVEC4,
		U_BMAT2,
		U_BMAT3,
		U_BMAT4,

		U_INT,
		U_IVEC2,
		U_IVEC3,
		U_IVEC4,
		U_IMAT2,
		U_IMAT3,
		U_IMAT4,

		U_FLOAT,
		U_FVEC2,
		U_FVEC3,
		U_FVEC4,
		U_FMAT2,
		U_FMAT3,
		U_FMAT4,

		U_SAMPLER1D,
		U_SAMPLER2D,
		U_SAMPLER3D,
		U_SAMPLERCUBE,
		U_SAMPLER1DSHADOW,
		U_SAMPLER2DSHADOW
	} UniformType;
	*/

	const std::string & Name(void) const
	{
		return this->name;
	}

	virtual GLenum Type(void) const = 0;

protected:
	Program * prog;
	GLint location;
	std::string name;

	Uniform(Program * prog, GLint location, const std::string & name)
	{
		this->prog = prog;
		this->location = location;
		this->name = name;
	}

	virtual void Apply(void) = 0;
};

class Uniform1b : public Uniform;
{
public:

	void SetValue(GLboolean x)
	{
		this->value[0] = x;
	}

	GLboolean GetValue(void) const
	{
		return this->value[0];
	}

protected:
	Program * prog;
	GLboolean value[1];

	Uniform(Program * prog, GLint location, const std::string & name) : Uniform(prog, location, name)
	{
		this->value = GL_FALSE;
	}
};

class Uniform2b : public Uniform;
{
public:

	void SetValue(GLboolean x, GLboolean y)
	{
		this->value[0] = x;
		this->value[1] = y;
	}

	GLboolean GetValueX(void) const
	{
		return this->value[0];
	}

	GLboolean GetValueY(void) const
	{
		return this->value[1];
	}

protected:
	Program * prog;
	GLboolean value[2];

	Uniform(Program * prog, GLint location, const std::string & name) : Uniform(prog, location, name)
	{
		this->value[0] = GL_FALSE;
		this->value[1] = GL_FALSE;
	}
};

class Uniform3b : public Uniform;
{
public:

	void SetValue(GLboolean x, GLboolean y, GLboolean z)
	{
		this->value[0] = x;
		this->value[1] = y;
		this->value[2] = z;
	}

	GLboolean GetValueX(void) const
	{
		return this->value[0];
	}

	GLboolean GetValueY(void) const
	{
		return this->value[1];
	}

	GLboolean GetValueZ(void) const
	{
		return this->value[2];
	}

protected:
	Program * prog;
	GLboolean value[2];

	Uniform(Program * prog, GLint location, const std::string & name) : Uniform(prog, location, name)
	{
		this->value[0] = GL_FALSE;
		this->value[1] = GL_FALSE;
	}
};

class Uniform1i : public Uniform;
{
public:

	void SetValue(GLint v)
	{
		this->value = v;
	}

	GLint GetValue(void) const
	{
		return this->value;
	}

protected:
	Program * prog;
	GLint value;

	Uniform(Program * prog, GLint location, const std::string & name) : Uniform(prog, location, name)
	{
		this->value = 0;
	}
};
#endif

class Program : public GLObject, public Bindable
{
public:

	Program(void)
	{
		this->linked = false;
	}

	void Gen(void)
	{
		this->Del();
		this->objectID = glCreateProgram();
	}

	void Del(void)
	{
		if (this->objectID == 0) return;
		glDeleteProgram(this->objectID);
		this->objectID = 0;
	}

	void Attach(Shader * shd)
	{
		if (this->objectID==0)
			Gen();
		this->shaders.insert(shd);
		this->linked = false;
		glAttachShader(this->objectID, shd->ObjectID());
	}

	void Detach(Shader * shd)
	{
		this->shaders.erase(shd);
		this->linked = false;
		glDetachShader(this->objectID, shd->ObjectID());
	}

	GLsizei AttachedShaders(void) const
	{
		return ((GLsizei)(this->shaders.size()));
	}

	Shader * AttachedShader(int i)
	{
		Shader * shd = 0;
		int cnt = 0;
		for (std::set<Shader *>::iterator it=this->shaders.begin(); (cnt < i) && (it!=this->shaders.end()); ++it)
		{
			shd = (*it);
		}
		return shd;
	}

	bool Link(void)
	{
		bool ok = true;
		for (std::set<Shader *>::iterator it=this->shaders.begin(); it!=this->shaders.end(); ++it)
		{
			Shader * shd = (*it);
			if (!shd->IsCompiled())
			{
				ok = shd->Compile() && ok;
			}
		}

		if (!ok)
			return false;

		glLinkProgram(this->objectID);

		GLint cm = 0;
		glGetProgramiv(this->objectID, GL_LINK_STATUS, &cm);
		this->linked = (cm != GL_FALSE);

		return this->linked;
	}

	bool IsLinked(void) const
	{
		return this->linked;
	}

	std::string InfoLog(void)
	{
		GLint len = 0;
		glGetProgramiv(this->objectID, GL_INFO_LOG_LENGTH, &len);
		char * ch = new char[len + 1];
		glGetProgramInfoLog(this->objectID, len, &len, ch);
		std::string infoLog = ch;
		delete [] ch;
		return infoLog;
	}

	void Uniform(const char * name, GLint x)
	{
		const GLint loc = glGetUniformLocation(this->objectID, name);
		glUniform1i(loc, x);
	}

	void Uniform(const char * name, GLint x, GLint y)
	{
		const GLint loc = glGetUniformLocation(this->objectID, name);
		glUniform2i(loc, x, y);
	}

	void Uniform(const char * name, GLint x, GLint y, GLint z)
	{
		const GLint loc = glGetUniformLocation(this->objectID, name);
		glUniform3i(loc, x, y, z);
	}

	void Uniform(const char * name, GLint x, GLint y, GLint z, GLint w)
	{
		const GLint loc = glGetUniformLocation(this->objectID, name);
		glUniform4i(loc, x, y, z, w);
	}

	void Uniform(const char * name, GLfloat x)
	{
		const GLint loc = glGetUniformLocation(this->objectID, name);
		glUniform1f(loc, x);
	}

	void Uniform(const char * name, GLfloat x, GLfloat y)
	{
		const GLint loc = glGetUniformLocation(this->objectID, name);
		glUniform2f(loc, x, y);
	}

	void Uniform(const char * name, GLfloat x, GLfloat y, GLfloat z)
	{
		const GLint loc = glGetUniformLocation(this->objectID, name);
		glUniform3f(loc, x, y, z);
	}

	void Uniform(const char * name, GLfloat x, GLfloat y, GLfloat z, GLfloat w)
	{
		const GLint loc = glGetUniformLocation(this->objectID, name);
		glUniform4f(loc, x, y, z, w);
	}

	void Uniform(const char * name, const vcg::Point2f& p)
	{
		const GLint loc = glGetUniformLocation(this->objectID, name);
		glUniform2fv(loc, 1, p.V());
	}

	void Uniform(const char * name, const vcg::Point3f& p)
	{
		const GLint loc = glGetUniformLocation(this->objectID, name);
		glUniform3fv(loc, 1, p.V());
	}

	void Uniform(const char * name, const vcg::Point4f& p)
	{
		const GLint loc = glGetUniformLocation(this->objectID, name);
		glUniform4fv(loc, 1, p.V());
	}

	void Parameter(GLenum pname, int value)
	{
		glProgramParameteriEXT(this->objectID, pname, value);
	}

	void Attribute(int index, GLfloat x, GLfloat y, GLfloat z, GLfloat w)
	{
		glVertexAttrib4f(index, x, y, z, w);
	}

	void BindAttribute(int index, const char * name)
	{
		glBindAttribLocation(this->objectID, index, name);
	}

protected:
	std::set<Shader *> shaders;
	bool linked;

	void DoBind(void)
	{
		if (!this->IsLinked())
		{
			this->Link();
		}
		glUseProgram(this->objectID);
	}

	void DoUnbind(void)
	{
		glUseProgram(0);
	}
};

class ProgramVF : public Bindable
{
public:
	Program prog;
	VertexShader vshd;
	FragmentShader fshd;

	ProgramVF(void) : Bindable()
	{
	}

	void SetSources(const char * vsrc, const char * fsrc)
	{
		if (vsrc) {
			this->vshd.SetSource(vsrc);
			this->prog.Attach(&(this->vshd));
		}
		if (fsrc) {
			this->fshd.SetSource(fsrc);
			this->prog.Attach(&(this->fshd));
		}
	}

	void LoadSources(const char * vfile, const char * ffile)
	{
		if (vfile) {
			this->vshd.LoadSource(vfile);
			this->prog.Attach(&(this->vshd));
		}
		if (ffile) {
			this->fshd.LoadSource(ffile);
			this->prog.Attach(&(this->fshd));
		}
	}

protected:
	void DoBind(void)
	{
		this->prog.Bind();
	}

	void DoUnbind(void)
	{
		this->prog.Unbind();
	}
};

#endif // __SHADERS_H__
