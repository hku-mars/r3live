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
Revision 1.1  2007/07/26 16:22:47  m_di_benedetto
First Commit.



****************************************************************************/

#ifndef VCGLIB_GL_SURFACE_H
#define VCGLIB_GL_SURFACE_H

#include <vector>
#include <GL/glew.h>

namespace vcg
{

/****************************************************************************
The gl_surface class simplify the render-to-texture OpenGL functionality.

It provides a framebuffer composed of single or multiple color buffers and
a depth buffer; color-only and depth-only framebuffers can also be created.

Sample usage:

****************************************************************
// *** declaration

gl_surface my_srf;


***********************
// *** initialization: single color render target, with depth buffer
{
	std::vector<GLenum> color_formats;
	color_formats.push_back(GL_RGBA8);

	my_srf.set(width, height, color_formats, GL_DEPTH_COMPONENT);
}
***********************


***********************
//  *** initialization: two color render targets, without depth buffer
// NOTE: the maximum number of color targets is implementation dependent.
// NOTE: DX10 class hardware allows different formats for each color target.
{
	std::vector<GLenum> color_formats;
	color_formats.push_back(GL_RGBA8);
	color_formats.push_back(GL_RGBA8);

	my_srf.set(width, height, color_formats, GL_NONE);
}
***********************


***********************
// *** usage: render-to-targets
{
	my_srf.begin_write();
		application_draw_code();
	my_srf.end_write();
}
***********************


***********************
// *** usage: using rendered textures
{
	// 2 buffers

	// bind the second
	glActiveTexture(GL_TEXTURE1);
	my_srf.begin_read_color(1); // actually does a glBindTexture();

	// bind the first
	glActiveTexture(GL_TEXTURE0);
	my_srf.begin_read_color(0);

		application_draw_code();

	// unbind the second
	glActiveTexture(GL_TEXTURE1);
	my_srf.end_read_color(1);

	// unbind first
	glActiveTexture(GL_TEXTURE0);
	my_srf.end_read_color(0);
}

***********************
// *** usage: use depth map
{
	my_srf.begin_read_depth();

		// use depth map here
		application_draw_code();

	my_srf.end_read_depth();
}

***********************
// *** usage: cleanup
{
	my_srf.clear();
	// clear() is also safely called in the destructor.
}
***********************

Other commodity methods for getting/setting pixels are also provided.

****************************************************************************/

class gl_surface
{
	public:

		typedef gl_surface this_type;

		gl_surface(void) : width(0), height(0), depth_tex(0), fb(0)
		{
			;
		}

		~gl_surface(void)
		{
			this->clear();
		}

		bool set(int width, int height, const std::vector<GLenum> & color_formats, GLenum depth_format)
		{
			this->clear();

			this->width  = width;
			this->height = height;

			this->color_formats = color_formats;

			this->color_texs.resize(color_formats.size());

			for (size_t i=0; i<this->color_texs.size(); ++i)
			{
				glGenTextures   (1, &(this->color_texs[i]));
				glBindTexture   (GL_TEXTURE_2D, this->color_texs[i]);
				glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,     GL_CLAMP_TO_EDGE);
				glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,     GL_CLAMP_TO_EDGE);
				glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
				glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
				glTexImage2D    (GL_TEXTURE_2D, 0, this->color_formats[i], this->width, this->height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, 0);
			}

			this->depth_format = depth_format;
			if (this->depth_format != GL_NONE)
			{
				glGenTextures   (1, &(this->depth_tex));
				glBindTexture   (GL_TEXTURE_2D, this->depth_tex);
				glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,           GL_CLAMP_TO_EDGE);
				glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,           GL_CLAMP_TO_EDGE);
				glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,       GL_NEAREST);
				glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,       GL_NEAREST);
				glTexParameteri (GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE_ARB,   GL_LUMINANCE);
				glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE_ARB, GL_COMPARE_R_TO_TEXTURE_ARB);
				glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC_ARB, GL_LEQUAL);
				glTexImage2D    (GL_TEXTURE_2D, 0, this->depth_format, this->width, this->height, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, 0);
			}

			glGenFramebuffersEXT(1, &(this->fb));
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, this->fb);

			std::vector<GLenum> sites(this->color_texs.size());
			for (size_t i=0; i<this->color_texs.size(); ++i)
			{
				sites[i] = GL_COLOR_ATTACHMENT0_EXT + ((GLenum)i);
				glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, sites[i], GL_TEXTURE_2D, this->color_texs[i], 0);
			}

			if (this->depth_format != GL_NONE)
			{
				glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, this->depth_tex, 0);
			}

			if (!sites.empty())
			{
				glDrawBuffers((GLsizei)(sites.size()), &(sites[0]));
			}

			const GLenum s = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
			const bool res = (s == GL_FRAMEBUFFER_COMPLETE_EXT);

			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

			if (!res)
			{
				glDeleteFramebuffersEXT(1, &(this->fb));
				this->fb = 0;

				for (size_t i=0; i<this->color_texs.size(); ++i)
				{
					glDeleteTextures(1, &(this->color_texs[i]));
				}
				this->color_texs.clear();

				this->color_formats.clear();

				if (this->depth_tex != 0)
				{
					glDeleteTextures(1, &(this->depth_tex));
					this->depth_tex = 0;
				}

				this->width  = 0;
				this->height = 0;

				return false;
			}

			return true;
		}

		bool set_simple(int width, int height)
		{
			std::vector<GLenum> c_formats;
			c_formats.push_back(GL_RGBA8);

			return this->set(width, height, c_formats, GL_DEPTH_COMPONENT);
		}

		bool set_color_only(int width, int height, GLenum color_format)
		{
			std::vector<GLenum> c_formats;
			c_formats.push_back(color_format);

			return this->set(width, height, c_formats, GL_NONE);
		}

		bool set_color_only(int width, int height, const std::vector<GLenum> & color_formats)
		{
			return this->set(width, height, color_formats, GL_NONE);
		}

		bool set_depth_only(int width, int height, GLenum depth_format)
		{
			std::vector<GLenum> c_formats;

			return this->set(width, height, c_formats, depth_format);
		}

		bool clear(void)
		{
			if (!this->is_valid()) return false;

			glDeleteFramebuffersEXT(1, &(this->fb));
			this->fb = 0;

			for (size_t i=0; i<this->color_texs.size(); ++i)
			{
				glDeleteTextures(1, &(this->color_texs[i]));
			}
			this->color_texs.clear();

			this->color_formats.clear();

			if (this->depth_tex != 0)
			{
				glDeleteTextures(1, &(this->depth_tex));
				this->depth_tex = 0;
			}

			this->width  = 0;
			this->height = 0;

			return true;
		}

		bool is_valid(void) const
		{
			return (this->fb != 0);
		}

		int get_width(void) const
		{
			return this->width;
		}

		int get_height(void) const
		{
			return this->height;
		}

		int color_attachments_count(void) const
		{
			return ((int)(this->color_texs.size()));
		}

		GLenum get_color_attachment_format(int attachment) const
		{
			if (!this->is_valid()) return GL_NONE;
			if ((attachment < 0) || (attachment >= this->color_attachments_count())) return GL_NONE;

			return this->color_formats[attachment];
		}

		bool has_depth_attachment(void) const
		{
			return (this->depth_tex != 0);
		}

		GLenum get_depth_attachment_format(void) const
		{
			if (!this->is_valid()) return GL_NONE;
			if (!this->has_depth_attachment()) return GL_NONE;

			return this->depth_format;
		}

		bool set_color_pixels(int attachment, GLenum format, GLenum type, const void * pixels)
		{
			if (!this->begin_read_color(attachment)) return false;

			glTexImage2D(GL_TEXTURE_2D, 0, this->color_formats[attachment], this->width, this->height, 0, format, type, pixels);

			this->end_read_color(attachment);

			return true;
		}

		bool get_color_pixels(int attachment, GLenum format, GLenum type, void * pixels)
		{
			if (!this->begin_read_color(attachment)) return false;

			glGetTexImage(GL_TEXTURE_2D, 0, format, type, pixels);

			this->end_read_color(attachment);

			return true;
		}

		bool set_depth_pixels(GLenum format, GLenum type, const void * pixels)
		{
			if (!this->is_valid()) return false;
			if (!this->has_depth_attachment()) return false;

			glTexImage2D(GL_TEXTURE_2D, 0, this->depth_format, this->width, this->height, 0, format, type, pixels);

			return true;
		}

		bool get_depth_pixels(GLenum format, GLenum type, void * pixels)
		{
			if (!this->begin_read_depth()) return false;

			glGetTexImage(GL_TEXTURE_2D, 0, format, type, pixels);

			this->end_read_depth();

			return true;
		}

		bool begin_read_color(int attachment)
		{
			if (!this->is_valid()) return false;
			if ((attachment < 0) || (attachment >= this->color_attachments_count())) return false;

			glBindTexture(GL_TEXTURE_2D, this->color_texs[attachment]);

			return true;
		}

		bool end_read_color(int attachment)
		{
			if (!this->is_valid()) return false;
			if ((attachment < 0) || (attachment >= this->color_attachments_count())) return false;

			glBindTexture(GL_TEXTURE_2D, 0);

			return true;
		}

		bool begin_read_depth(void)
		{
			if (!this->is_valid()) return false;
			if (!this->has_depth_attachment()) return false;

			glBindTexture(GL_TEXTURE_2D, this->depth_tex);

			return true;
		}

		bool end_read_depth(void)
		{
			if (!this->is_valid()) return false;
			if (!this->has_depth_attachment()) return false;

			glBindTexture(GL_TEXTURE_2D, 0);

			return true;
		}

		bool begin_write(void)
		{
			if (!this->is_valid()) return false;

			glPushAttrib(GL_VIEWPORT_BIT);

			glViewport(0, 0, this->width, this->height);

			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, this->fb);

			return true;
		}

		bool end_write(void)
		{
			if (!this->is_valid()) return false;

			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

			glPopAttrib();

			return true;
		}

		bool draw_color_attachment(int x, int y, int width, int height, int attachment)
		{
			if (!this->is_valid()) return false;
			if ((attachment < 0) || (attachment >= this->color_attachments_count())) return false;

			glPushAttrib(GL_ALL_ATTRIB_BITS);

			glViewport(x, y, width, height);

			glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadIdentity();

			this->begin_read_color(attachment);
				glBegin(GL_QUADS);
					glTexCoord2f(0.0f, 0.0f);    glVertex2f(-1.0f, -1.0f);
					glTexCoord2f(1.0f, 0.0f);    glVertex2f( 1.0f, -1.0f);
					glTexCoord2f(1.0f, 1.0f);    glVertex2f( 1.0f,  1.0f);
					glTexCoord2f(0.0f, 1.0f);    glVertex2f(-1.0f,  1.0f);
				glEnd();
			this->end_read_color(attachment);

			glMatrixMode(GL_PROJECTION);
			glPopMatrix();

			glMatrixMode(GL_MODELVIEW);
			glPopMatrix();

			glPopAttrib();
      return true;
		}

		bool draw_depth_attachment(int x, int y, int width, int height)
		{
			if (!this->is_valid()) return false;
			if (!this->has_depth_attachment()) return false;

			glPushAttrib(GL_ALL_ATTRIB_BITS);

			glViewport(x, y, width, height);

			glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadIdentity();

			this->begin_read_depth();
				glBegin(GL_QUADS);
					glTexCoord2f(0.0f, 0.0f);    glVertex2f(-1.0f, -1.0f);
					glTexCoord2f(1.0f, 0.0f);    glVertex2f( 1.0f, -1.0f);
					glTexCoord2f(1.0f, 1.0f);    glVertex2f( 1.0f,  1.0f);
					glTexCoord2f(0.0f, 1.0f);    glVertex2f(-1.0f,  1.0f);
				glEnd();
			this->end_read_depth();

			glMatrixMode(GL_PROJECTION);
			glPopMatrix();

			glMatrixMode(GL_MODELVIEW);
			glPopMatrix();

			glPopAttrib();
      return true;
		}

	protected:

		int width;
		int height;
		std::vector<GLenum> color_formats;
		std::vector<GLuint> color_texs;
		GLenum depth_format;
		GLuint depth_tex;
		GLuint fb;
};

} // end namespace vcg

#endif // VCGLIB_GL_SURFACE_H
