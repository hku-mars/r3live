#ifndef GLW_UTILITY_H
#define GLW_UTILITY_H

#include <stdio.h>
#include <stddef.h>

#include <string>
#include <sstream>
#include <map>
#include <vector>

#include "./context.h"

#define GLW_STRINGIFY(S)             #S
#define GLW_OFFSET_OF(TYPE, MEMBER)  ((const void *)(offsetof(TYPE, MEMBER)))

namespace glw
{

class ShaderHeaderBuilder
{
	public:

		typedef void                BaseType;
		typedef ShaderHeaderBuilder ThisType;

		void clear(void)
		{
			this->m_version    .clear();
			this->m_extensions .clear();
			this->m_defines    .clear();
			this->m_texts      .clear();
		}

		ThisType & version(const std::string & v)
		{
			this->m_version = v;
			return (*this);
		}

		ThisType & enableExtension(const std::string & ext)
		{
			this->m_extensions[ext] = ThisType::Enable;
			return (*this);
		}

		ThisType & disableExtension(const std::string & ext)
		{
			this->m_extensions[ext] = ThisType::Disable;
			return (*this);
		}

		ThisType & requireExtension(const std::string & ext)
		{
			this->m_extensions[ext] = ThisType::Require;
			return (*this);
		}

		ThisType & define(const std::string & name, const std::string & value)
		{
			this->m_defines[name] = value;
			return (*this);
		}

		ThisType & text(const std::string & txt)
		{
			this->m_texts.push_back(txt);
			return (*this);
		}

		std::string toString(void) const
		{
			const char * extModeMap[] =
			{
				"",
				"enable",
				"disable",
				"require"
			};

			std::ostringstream res;

			if (!this->m_version.empty())
			{
				res << "#version " << this->m_version << std::endl;
				res << std::endl;
			}

			if (!this->m_extensions.empty())
			{
				for (ExtensionMapConstIterator it=this->m_extensions.begin(); it!=this->m_extensions.end(); ++it)
				{
					if (it->second == ThisType::DontCare) continue;
					res << "#extension " << it->first << " : " << extModeMap[it->second] << std::endl;
				}
				res << std::endl;
			}

			if (!this->m_defines.empty())
			{
				for (DefineMapConstIterator it=this->m_defines.begin(); it!=this->m_defines.end(); ++it)
				{
					res << "#define " << it->first << " " << it->second << std::endl;
				}
				res << std::endl;
			}

			if (!this->m_texts.empty())
			{
				for (size_t i=0; i<this->m_texts.size(); ++i)
				{
					res << this->m_texts[i] << std::endl;
				}
				res << std::endl;
			}

			return res.str();
		}

	protected:

		enum ExtensionMode
		{
			DontCare = 0,
			Enable,
			Disable,
			Require
		};

		typedef std::map<std::string, ExtensionMode> ExtensionMap;
		typedef ExtensionMap::const_iterator         ExtensionMapConstIterator;
		typedef ExtensionMap::iterator               ExtensionMapIterator;
		typedef ExtensionMap::value_type             ExtensionMapValue;

		typedef std::map<std::string, std::string>   DefineMap;
		typedef DefineMap::const_iterator            DefineMapConstIterator;
		typedef DefineMap::iterator                  DefineMapIterator;
		typedef DefineMap::value_type                DefineMapValue;

		typedef std::vector<std::string>             TextVector;

		std::string  m_version;
		ExtensionMap m_extensions;
		DefineMap    m_defines;
		TextVector   m_texts;
};

inline std::string readTextFile(const char * fileName)
{
	std::string r;
	if (fileName == 0) return r;

	FILE * f = fopen(fileName, "rb");
	if (f == 0) return r;

	fseek(f, 0, SEEK_END);
	const size_t sz = size_t(ftell(f));
	rewind(f);

	char * str = new char [sz + 1];
	fread(str, sizeof(char), sz / sizeof(char), f);
	fclose(f);

	str[sz] = '\0';
	r = str;
	delete [] str;

	return r;
}

inline std::string readTextFile(const std::string & fileName)
{
	return readTextFile(fileName.c_str());
}

inline BufferHandle createBuffer(Context & ctx, GLsizeiptr size, const void * data = 0, GLenum usage = GL_STATIC_DRAW)
{
	BufferArguments args;
	args.size  = size;
	args.usage = usage;
	args.data  = data;
	return ctx.createBuffer(args);
}

template <typename TValue, typename TAllocator>
inline BufferHandle createBuffer(Context & ctx, const std::vector<TValue, TAllocator> & data, GLenum usage = GL_STATIC_DRAW)
{
	return createBuffer(ctx, GLsizeiptr(sizeof(TValue) * data.size()), ((!data.empty()) ? (&(data[0])) : (0)), usage);
}

inline RenderbufferHandle createRenderbuffer(Context & ctx, GLenum format, GLsizei width, GLsizei height)
{
	RenderbufferArguments args;
	args.format = format;
	args.width  = width;
	args.height = height;
	return ctx.createRenderbuffer(args);
}

inline Texture2DHandle createTexture2D(Context & ctx, GLenum format, GLsizei width, GLsizei height, GLenum dataFormat, GLenum dataType, const void * data = 0, const TextureSampleMode & sampler = TextureSampleMode())
{
	Texture2DArguments args;
	args.format     = format;
	args.width      = width;
	args.height     = height;
	args.dataFormat = dataFormat;
	args.dataType   = dataType;
	args.data       = data;
	args.sampler    = sampler;
	return ctx.createTexture2D(args);
}

inline TextureCubeHandle createTextureCube(Context & ctx, GLenum format, GLsizei size, GLenum dataFormat, GLenum dataType, const void * data = 0, const TextureSampleMode & sampler = TextureSampleMode())
{
	TextureCubeArguments args;
	args.format     = format;
	args.size       = size;
	args.dataFormat = dataFormat;
	args.dataType   = dataType;
	args.data       = data;
	args.sampler    = sampler;
	return ctx.createTextureCube(args);
}

inline FramebufferHandle createFramebufferWithDepthStencil
(
	Context & ctx,
	const RenderTarget & depthTarget,
	const RenderTarget & stencilTarget,
	const RenderTarget & colorTarget0 = RenderTarget(),
	const RenderTarget & colorTarget1 = RenderTarget(),
	const RenderTarget & colorTarget2 = RenderTarget(),
	const RenderTarget & colorTarget3 = RenderTarget(),
	const RenderTarget & colorTarget4 = RenderTarget(),
	const RenderTarget & colorTarget5 = RenderTarget(),
	const RenderTarget & colorTarget6 = RenderTarget(),
	const RenderTarget & colorTarget7 = RenderTarget()
)
{
	FramebufferArguments args;

	args.depthTarget   = depthTarget;
	args.stencilTarget = stencilTarget;

	if (colorTarget0.target) { args.colorTargets[0] = colorTarget0; args.targetInputs[0] = 0; }
	if (colorTarget1.target) { args.colorTargets[1] = colorTarget1; args.targetInputs[1] = 1; }
	if (colorTarget2.target) { args.colorTargets[2] = colorTarget2; args.targetInputs[2] = 2; }
	if (colorTarget3.target) { args.colorTargets[3] = colorTarget3; args.targetInputs[3] = 3; }
	if (colorTarget4.target) { args.colorTargets[4] = colorTarget4; args.targetInputs[4] = 4; }
	if (colorTarget5.target) { args.colorTargets[5] = colorTarget5; args.targetInputs[5] = 5; }
	if (colorTarget6.target) { args.colorTargets[6] = colorTarget6; args.targetInputs[6] = 6; }
	if (colorTarget7.target) { args.colorTargets[7] = colorTarget7; args.targetInputs[7] = 7; }

	return ctx.createFramebuffer(args);
}

inline FramebufferHandle createFramebuffer
(
	Context & ctx,
	const RenderTarget & depthTarget,
	const RenderTarget & colorTarget0 = RenderTarget(),
	const RenderTarget & colorTarget1 = RenderTarget(),
	const RenderTarget & colorTarget2 = RenderTarget(),
	const RenderTarget & colorTarget3 = RenderTarget(),
	const RenderTarget & colorTarget4 = RenderTarget(),
	const RenderTarget & colorTarget5 = RenderTarget(),
	const RenderTarget & colorTarget6 = RenderTarget(),
	const RenderTarget & colorTarget7 = RenderTarget()
)
{
	RenderTarget nullTarget;
	return createFramebufferWithDepthStencil(ctx, depthTarget, nullTarget, colorTarget0, colorTarget1, colorTarget2, colorTarget3, colorTarget4, colorTarget5, colorTarget6, colorTarget7);
}

inline ProgramHandle createProgram(Context & ctx, const std::string & srcPrefix, const std::string & vertexSrc, const std::string & geometrySrc, const std::string & fragmentSrc, const ProgramArguments & args = ProgramArguments())
{
	ProgramArguments pArgs = args;

	if (!vertexSrc.empty())
	{
		VertexShaderArguments args;
		args.source = srcPrefix + vertexSrc;
		pArgs.shaders.push_back(ctx.createVertexShader(args));
	}

	if (!geometrySrc.empty())
	{
		GeometryShaderArguments args;
		args.source = srcPrefix + geometrySrc;
		pArgs.shaders.push_back(ctx.createGeometryShader(args));
	}

	if (!fragmentSrc.empty())
	{
		FragmentShaderArguments args;
		args.source = srcPrefix + fragmentSrc;
		pArgs.shaders.push_back(ctx.createFragmentShader(args));
	}

	return ctx.createProgram(pArgs);
}

inline ProgramHandle createProgram(Context & ctx, const std::string & srcPrefix, const std::string & vertexSrc, const std::string & fragmentSrc, const ProgramArguments & args = ProgramArguments())
{
	return createProgram(ctx, srcPrefix, vertexSrc, "", fragmentSrc, args);
}

inline ProgramHandle loadProgram(Context & ctx, const std::string & srcPrefix, const std::string & vertexFile, const std::string & geometryFile, const std::string & fragmentFile, const ProgramArguments & args = ProgramArguments())
{
	return createProgram(ctx, srcPrefix, readTextFile(vertexFile), readTextFile(geometryFile), readTextFile(fragmentFile), args);
}

inline ProgramHandle loadProgram(Context & ctx, const std::string & srcPrefix, const std::string & vertexFile, const std::string & fragmentFile, const ProgramArguments & args = ProgramArguments())
{
	return loadProgram(ctx, srcPrefix, vertexFile, "", fragmentFile.c_str(), args);
}

};

#endif // GLW_UTILITY_H
