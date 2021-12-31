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

#ifndef __FBO_H__
#define __FBO_H__

#pragma warning(disable : 4250)

#include <map>
#include <vector>

#include <GL/glew.h>
#include <GL/glut.h>

#include "gl_object.h"

class FrameBufferSemantic
{
public:
	typedef enum
	{
		COLOR,
		DEPTH,
		STENCIL
	} FBSType;

	virtual FBSType Semantic(void) const = 0;
	virtual bool ValidateFormat(GLenum format) const = 0;

	static bool ValidateFormat(FBSType type, GLenum format)
	{
		switch (type)
		{
			case COLOR   : return FrameBufferSemantic::ValidateColor(format);
			case DEPTH   : return FrameBufferSemantic::ValidateDepth(format);
			case STENCIL : return FrameBufferSemantic::ValidateStencil(format);
			default      : return false;
		}
	}

	static bool ValidateColor(GLenum type)
	{
		return true;
	}

	static bool ValidateDepth(GLenum type)
	{
		return  true;
	}

	static bool ValidateStencil(GLenum type)
	{
		return true;
	}
};

class Texture : public GLObject, public Bindable, public FrameBufferSemantic
{
public:
	Texture(void) : GLObject(), Bindable(), FrameBufferSemantic()
	{
		this->format = GL_NONE;
	}

	void Gen(void)
	{
		this->Del();
		glGenTextures(1, &(this->objectID));
	}

	void Del(void)
	{
		if (this->objectID == 0) return;
		glDeleteTextures(1, &(this->objectID));
		this->objectID = 0;
	}

	GLenum Format(void) const
	{
		return this->format;
	}

	virtual GLint Dimensions(void) const = 0;

	virtual GLsizei Size(const unsigned int i) const = 0;

	virtual GLenum Target(void) const = 0;

protected:
	GLenum format;

	void DoBind(void)
	{
		glBindTexture(this->Target(), this->objectID);
	}

	void DoUnbind(void)
	{
		glBindTexture(this->Target(), 0);
	}
};

class ColorTexture : public virtual Texture
{
public:
	ColorTexture(void) : Texture()
	{
	}

	FrameBufferSemantic::FBSType Semantic(void) const
	{
		return FrameBufferSemantic::COLOR;
	}

	bool ValidateFormat(GLenum format) const
	{
		return FrameBufferSemantic::ValidateColor(format);
	}
};

class DepthTexture : public virtual Texture
{
public:
	DepthTexture(void) : Texture()
	{
	}

	FrameBufferSemantic::FBSType Semantic(void) const
	{
		return FrameBufferSemantic::DEPTH;
	}

	bool ValidateFormat(GLenum format) const
	{
		return FrameBufferSemantic::ValidateDepth(format);
	}
};

class StencilTexture : public virtual Texture
{
public:
	StencilTexture(void) : Texture()
	{
	}

	FrameBufferSemantic::FBSType Semantic(void) const
	{
		return FrameBufferSemantic::STENCIL;
	}

	bool ValidateFormat(GLenum format) const
	{
		return FrameBufferSemantic::ValidateStencil(format);
	}
};

class Texture1D : public virtual Texture
{
public:
	Texture1D(void) : Texture()
	{
		this->dims[0] = 0;
		this->wraps[0] = GL_CLAMP_TO_EDGE;
	}

	GLsizei Width(void) const
	{
		return this->dims[0];
	}

	GLint Dimensions(void) const
	{
		return 1;
	}

	GLsizei Size(const unsigned int i) const
	{
		if (i > 0) return 0;
		return this->dims[0];
	}

	GLenum Target(void) const
	{
		return GL_TEXTURE_1D;
	}

	bool Set(GLint level, GLint internalFormat, GLsizei width, GLint border, GLenum format, GLenum type, const GLvoid * pixels)
	{
		if (!this->ValidateFormat(internalFormat)) return false;

		this->Bind();

		glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexImage1D(GL_TEXTURE_1D, level, internalFormat, width, border, format, type, pixels);

		this->Unbind();

		this->format = internalFormat;
		this->dims[0] = width;

		return true;
	}

protected:
	GLsizei dims[1];
	GLenum wraps[1];
};

class Texture2D : public virtual Texture
{
public:
	Texture2D(void) : Texture()
	{
		this->dims[0] = 0;
		this->dims[1] = 0;
	}

	GLsizei Width(void) const
	{
		return this->dims[0];
	}

	GLsizei Height(void) const
	{
		return this->dims[1];
	}

	GLint Dimensions(void) const
	{
		return 2;
	}

	GLsizei Size(const unsigned int i) const
	{
		if (i > 1) return 0;
		return this->dims[i];
	}

	GLenum Target(void) const
	{
		return GL_TEXTURE_2D;
	}

	bool Set(GLint level, GLint internalFormat, GLsizei width, GLsizei height, GLint border, GLenum format, GLenum type, const GLvoid * pixels)
	{
		if (!this->ValidateFormat(internalFormat)) return false;

		this->Bind();

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, width, height, border, format, type, pixels);

		this->Unbind();

		this->format = internalFormat;
		this->dims[0] = width;
		this->dims[1] = height;

		return true;
	}

protected:
	GLsizei dims[2];

	void DoBind(void)
	{
		glBindTexture(GL_TEXTURE_2D, this->objectID);
	}

	void DoUnbind(void)
	{
		glBindTexture(GL_TEXTURE_2D, 0);
	}
};

class Texture3D : public virtual Texture
{
public:
	Texture3D(void) : Texture()
	{
		this->dims[0] = 0;
		this->dims[1] = 0;
		this->dims[2] = 0;
	}

	GLsizei Width(void) const
	{
		return this->dims[0];
	}

	GLsizei Height(void) const
	{
		return this->dims[1];
	}

	GLsizei Depth(void) const
	{
		return this->dims[2];
	}

	GLint Dimensions(void) const
	{
		return 3;
	}

	GLsizei Size(const unsigned int i) const
	{
		if (i > 2) return 0;
		return this->dims[i];
	}

	GLenum Target(void) const
	{
		return GL_TEXTURE_3D;
	}

	bool Set(GLint level, GLint internalFormat, GLsizei width, GLsizei height, GLsizei depth, GLint border, GLenum format, GLenum type, const GLvoid * pixels)
	{
		if (!this->ValidateFormat(internalFormat)) return false;

		this->Bind();

		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexImage3D(GL_TEXTURE_3D, 0, internalFormat, width, height, depth, border, format, type, pixels);

		this->Unbind();

		this->format = internalFormat;
		this->dims[0] = width;
		this->dims[1] = height;
		this->dims[2] = depth;

		return true;
	}

protected:
	GLsizei dims[3];

	void DoBind(void)
	{
		glBindTexture(GL_TEXTURE_3D, this->objectID);
	}

	void DoUnbind(void)
	{
		glBindTexture(GL_TEXTURE_3D, 0);
	}
};

class ColorTexture1D : public virtual ColorTexture, public virtual Texture1D
{
public:
	ColorTexture1D(void) : ColorTexture(), Texture1D()
	{
	}
};

class ColorTexture2D : public virtual ColorTexture, public virtual Texture2D
{
public:
	ColorTexture2D(void) : ColorTexture(), Texture2D()
	{
	}
};

class ColorTexture3D : public virtual ColorTexture, public virtual Texture3D
{
public:
	ColorTexture3D(void) : ColorTexture(), Texture3D()
	{
	}
};

class DepthTexture2D : public virtual DepthTexture, public virtual Texture2D
{
public:
	DepthTexture2D(void) : DepthTexture(), Texture2D()
	{
	}
};

class StencilTexture2D : public virtual StencilTexture, public virtual Texture2D
{
public:
	StencilTexture2D(void) : StencilTexture(), Texture2D()
	{
	}
};

class FrameBuffer;

class RenderTarget : public GLObject, public Bindable, public FrameBufferSemantic
{
friend class FrameBuffer;

public:
	typedef enum
	{
		BUFFER,
		TEXTURE
	} RTStorageType;

	RenderTarget(void) : GLObject(), Bindable(), FrameBufferSemantic()
	{
		this->frameBuffer = 0;
	}

	bool Attach(FrameBuffer * fb);

	bool Detach(void);

	FrameBuffer * GetFrameBuffer(void)
	{
		return this->frameBuffer;
	}

	const FrameBuffer * GetFrameBuffer(void) const
	{
		return this->frameBuffer;
	}

	virtual GLsizei Width(void) const = 0;
	virtual GLsizei Height(void) const = 0;
	virtual GLenum Format(void) const = 0;

	virtual GLenum Attachment(void) const = 0;

	virtual bool ValidateAttachment(GLenum attachment) const = 0;

	virtual RTStorageType StorageType(void) const = 0;

protected:
	FrameBuffer * frameBuffer;

	virtual bool BindToFB(void) = 0;
};

class BufferRenderTarget : public virtual RenderTarget
{
public:
	BufferRenderTarget(void) : RenderTarget()
	{
		this->width = 0;
		this->height = 0;
		this->format = GL_NONE;
	}

	void Gen(void)
	{
		this->Del();
		glGenRenderbuffersEXT(1, &(this->objectID));
	}

	void Del(void)
	{
		if (this->objectID == 0) return;
		glDeleteRenderbuffersEXT(1, &(this->objectID));
		this->objectID = 0;
	}

	GLsizei Width(void) const
	{
		return this->width;
	}

	GLsizei Height(void) const
	{
		return this->height;
	}

	GLenum Format(void) const
	{
		return this->format;
	}

	RTStorageType StorageType(void) const
	{
		return RenderTarget::BUFFER;
	}

	bool Set(GLenum format, GLsizei width, GLsizei height)
	{
		if (!this->ValidateFormat(format)) return false;

		this->Bind();

		glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, format, width, height);

		this->Unbind();

		this->format = format;
		this->width = width;
		this->height = height;

		return true;
	}

	bool BindToFB(void)
	{
		if (this->frameBuffer == 0) return false;
		glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, this->Attachment(), GL_RENDERBUFFER_EXT, this->objectID);

		return true;
	}

protected:
	GLenum format;
	GLsizei width;
	GLsizei height;

	void DoBind(void)
	{
		glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, this->objectID);
	}

	void DoUnbind(void)
	{
		glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);
	}
};

class TextureRenderTarget : public virtual RenderTarget
{
public:
	TextureRenderTarget(void) : RenderTarget()
	{
		this->tex = 0;
		this->level = 0;
	}

	void Gen(void)
	{
	}

	void Del(void)
	{
	}

	GLsizei Width(void) const
	{
		if (this->tex == 0) return 0;
		return this->tex->Width();
	}

	GLsizei Height(void) const
	{
		if (this->tex == 0) return 0;
		return this->tex->Height();
	}

	GLenum Format(void) const
	{
		if (this->tex == 0) return GL_NONE;
		return this->tex->Format();
	}

	RTStorageType StorageType(void) const
	{
		return RenderTarget::TEXTURE;
	}

	void SetLevel(GLint level)
	{
		if (level < 0) level = 0;
		this->level = level;
	}

	bool Set(Texture2D * tex)
	{
		this->Unset();

		if (tex == 0) return true;
		if (this->Semantic() != tex->Semantic()) return false;

		this->tex = tex;

		return true;
	}

	bool Unset(void)
	{
		this->tex = 0;

		return true;
	}

	Texture2D * GetTexture(void)
	{
		return (this->tex);
	}

	bool BindToFB(void)
	{
		if (this->frameBuffer == 0) return false;
		if (this->tex == 0) return false;

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, this->Attachment(), GL_TEXTURE_2D, this->tex->ObjectID(), this->level);

		return true;
	}

protected:

	Texture2D * tex;
	GLint level;

	void DoBind(void)
	{
	}

	void DoUnbind(void)
	{
	}
};

class ColorRenderTarget : public virtual RenderTarget
{
public:
	ColorRenderTarget(void) : RenderTarget()
	{
		this->attachment = GL_COLOR_ATTACHMENT0_EXT;
	}

	FrameBufferSemantic::FBSType Semantic(void) const
	{
		return FrameBufferSemantic::COLOR;
	}

	bool ValidateFormat(GLenum format) const
	{
		return FrameBufferSemantic::ValidateColor(format);
	}

	bool ValidateAttachment(GLenum attachment) const
	{
		return (((GL_COLOR_ATTACHMENT0_EXT) <= attachment) && (attachment <= (GL_COLOR_ATTACHMENT0_EXT + 3)));
	}

	void SetAttachment(GLenum attachment)
	{
		if (!this->ValidateAttachment(attachment)) return;
		this->attachment = attachment;
	}

	GLenum Attachment(void) const
	{
		return this->attachment;
	}

protected:
	GLenum attachment;
};

class DepthRenderTarget : public virtual RenderTarget
{
public:
	DepthRenderTarget(void) : RenderTarget()
	{
	}

	FrameBufferSemantic::FBSType Semantic(void) const
	{
		return FrameBufferSemantic::DEPTH;
	}

	bool ValidateFormat(GLenum format) const
	{
		return FrameBufferSemantic::ValidateDepth(format);
	}

	bool ValidateAttachment(GLenum attachment) const
	{
		return (attachment == GL_DEPTH_ATTACHMENT_EXT);
	}

	GLenum Attachment(void) const
	{
		return GL_DEPTH_ATTACHMENT_EXT;
	}
};

class StencilRenderTarget : public virtual RenderTarget
{
public:
	StencilRenderTarget(void) : RenderTarget()
	{
	}

	FrameBufferSemantic::FBSType Semantic(void) const
	{
		return FrameBufferSemantic::STENCIL;
	}

	bool ValidateFormat(GLenum format) const
	{
		return FrameBufferSemantic::ValidateStencil(format);
	}

	bool ValidateAttachment(GLenum attachment) const
	{
		return (attachment == GL_STENCIL_ATTACHMENT_EXT);
	}

	GLenum Attachment(void) const
	{
		return GL_STENCIL_ATTACHMENT_EXT;
	}
};

class ColorRenderBuffer : public virtual ColorRenderTarget, public virtual BufferRenderTarget
{
public:
	ColorRenderBuffer(void) : ColorRenderTarget(), BufferRenderTarget()
	{
	}
};

class ColorRenderTexture : public virtual ColorRenderTarget, public virtual TextureRenderTarget
{
public:
	ColorRenderTexture(void) : ColorRenderTarget(), TextureRenderTarget()
	{
	}

	ColorRenderTexture(Texture2D * tex) : ColorRenderTarget(), TextureRenderTarget()
	{
		this->Set(tex);
	}
};

class DepthRenderBuffer : public virtual DepthRenderTarget, public virtual BufferRenderTarget
{
public:
	DepthRenderBuffer(void) : DepthRenderTarget(), BufferRenderTarget()
	{
	}
};

class DepthRenderTexture : public virtual DepthRenderTarget, public virtual TextureRenderTarget
{
public:
	DepthRenderTexture(void) : DepthRenderTarget(), TextureRenderTarget()
	{
	}

	DepthRenderTexture(Texture2D * tex) : DepthRenderTarget(), TextureRenderTarget()
	{
		this->Set(tex);
	}
};

class StencilRenderBuffer : public virtual StencilRenderTarget, public virtual BufferRenderTarget
{
public:
	StencilRenderBuffer(void) : StencilRenderTarget(), BufferRenderTarget()
	{
	}
};

class StencilRenderTexture : public virtual StencilRenderTarget, public virtual TextureRenderTarget
{
public:
	StencilRenderTexture(void) : StencilRenderTarget(), TextureRenderTarget()
	{
	}

	StencilRenderTexture(Texture2D * tex) : StencilRenderTarget(), TextureRenderTarget()
	{
		this->Set(tex);
	}
};

class FrameBuffer : public GLObject, public Bindable
{
friend class RenderTarget;

public:
	FrameBuffer(void) : GLObject(), Bindable()
	{
	}

	void Gen(void)
	{
		this->Del();
		glGenFramebuffersEXT(1, &(this->objectID));
	}

	void Del(void)
	{
		if (this->objectID == 0) return;
		glDeleteFramebuffersEXT(1, &(this->objectID));
		this->objectID = 0;
	}

	bool DetachAll(void)
	{
		return false;
	}

	bool IsValid(void) const
	{
		const GLenum s = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
		//return (s == GL_FRAMEBUFFER_COMPLETE_EXT);

		switch (s)
		{
			case GL_FRAMEBUFFER_COMPLETE_EXT:
				printf("ok\n");
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT:
				printf("i a\n");
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT:
				printf("i m a\n");
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_DUPLICATE_ATTACHMENT_EXT:
				printf("i d a\n");
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT:
				printf("i d\n");
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT:
				printf("i f\n");
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT:
				printf("i d b\n");
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER_EXT:
				printf("i r b\n");
				break;
			case GL_FRAMEBUFFER_UNSUPPORTED_EXT:
				printf("u\n");
				break;
			default:
				printf("def\n");
				break;
		}

		return (s == GL_FRAMEBUFFER_COMPLETE_EXT);
	}

protected:
	typedef std::map<GLenum, RenderTarget *> RTMap;
	typedef RTMap::iterator RTMap_i;
	typedef RTMap::const_iterator RTMap_ci;

	RTMap renderTargets;

	void DoBind(void)
	{
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, this->objectID);

		std::vector<GLenum> colorDrawBuffers;
		colorDrawBuffers.reserve(this->renderTargets.size());

		for (RTMap_i rt=this->renderTargets.begin(); rt!=this->renderTargets.end(); ++rt)
		{
			RenderTarget * prt = (*rt).second;
			if (prt->Semantic() == FrameBufferSemantic::COLOR)
			{
				colorDrawBuffers.push_back(prt->Attachment());
			}
			prt->BindToFB();
		}

		const GLsizei sz = (GLsizei)(colorDrawBuffers.size());
		if (sz > 0)
		{
			glDrawBuffers(sz, &(colorDrawBuffers[0]));
		}
	}

	void DoUnbind(void)
	{
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	}

	bool AddRT(RenderTarget * rt)
	{
		if (rt == 0) return false;
		RTMap_i it = this->renderTargets.find(rt->Attachment());
		if (it == this->renderTargets.end())
		{
			this->renderTargets.insert(std::make_pair(rt->Attachment(), rt));
			return true;
		}
		return false;
	}

	bool RemoveRT(RenderTarget * rt)
	{
		if (rt == 0) return false;
		RTMap_i it = this->renderTargets.find(rt->Attachment());
		if ((*it).second == rt)
		{
			this->renderTargets.erase(it);
			return true;
		}
		return false;
	}
};



bool RenderTarget::Attach(FrameBuffer * fb)
{
	this->Detach();
	if (fb == 0) return true;

	if (fb->AddRT(this))
	{
		this->frameBuffer = fb;
		return true;
	}

	return false;
}

bool RenderTarget::Detach(void)
{
	if (this->frameBuffer == 0) return false;
	this->frameBuffer->RemoveRT(this);
	this->frameBuffer = 0;

	return true;
}

#endif  // __FBO_H__
