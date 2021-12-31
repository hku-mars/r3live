#ifndef GLW_TEXTURECUBE_H
#define GLW_TEXTURECUBE_H

#include "./texture.h"

namespace glw
{

class TextureCubeArguments : public TextureArguments
{
	public:

		typedef TextureArguments   BaseType;
		typedef TextureCubeArguments ThisType;

		GLsizei           size;
		GLenum            dataFormat;
		GLenum            dataType;
		const void *      data;
		TextureSampleMode sampler;

		TextureCubeArguments(void)
		{
			this->clear();
		}

		void clear(void)
		{
			BaseType::clear();
			this->size       = 0;
			this->dataFormat = GL_NONE;
			this->dataType   = GL_NONE;
			this->data       = 0;
			this->sampler.clear();
		}
};

class TextureCube : public Texture
{
	friend class Context;

	public:

		typedef Texture   BaseType;
		typedef TextureCube ThisType;

		virtual Type type(void) const
		{
			return TextureCubeType;
		}

		virtual int imageDimensions(void) const
		{
			return 2;
		}

		virtual bool isArray(void) const
		{
			return false;
		}

		GLsizei size(void) const
		{
			return this->m_size;
		}

		GLsizei width(void) const
		{
			return this->size();
		}

		GLsizei height(void) const
		{
			return this->size();
		}

		void setImage(GLenum target, GLint unit, GLint level, GLsizei size, GLenum dataFormat, GLenum dataType, const void * data)
		{
			(void)unit;
			GLW_ASSERT(this->isValid());
			glTexImage2D(target, level, this->m_format, size, size, 0, dataFormat, dataType, data);
		}

		void getImage(GLenum target, GLint unit, GLint level, GLenum dataFormat, GLenum dataType, void * data)
		{
			(void)unit;
			GLW_ASSERT(this->isValid());
			glGetTexImage(target, level, dataFormat, dataType, data);
		}

		void setSubImage(GLenum target, GLint unit, GLint level, GLint xoffset, GLint yoffset, GLsizei width, GLsizei height, GLenum dataFormat, GLenum dataType, const void * data)
		{
			(void)unit;
			GLW_ASSERT(this->isValid());
			glTexSubImage2D(target, level, xoffset, yoffset, width, height, dataFormat, dataType, data);
		}

		void generateMipmap(GLenum target, GLint unit)
		{
			(void)unit;
			GLW_ASSERT(this->isValid());
			glGenerateMipmap(target);
		}

		void setSampleMode(GLenum target, GLint unit, const TextureSampleMode & sampler)
		{
			(void)unit;
			GLW_ASSERT(this->isValid());
			if (GLW_CARE_OF(sampler.minFilter)) glTexParameteri(target, GL_TEXTURE_MIN_FILTER, sampler.minFilter);
			if (GLW_CARE_OF(sampler.magFilter)) glTexParameteri(target, GL_TEXTURE_MAG_FILTER, sampler.magFilter);
			if (GLW_CARE_OF(sampler.wrapS    )) glTexParameteri(target, GL_TEXTURE_WRAP_S,     sampler.wrapS    );
			if (GLW_CARE_OF(sampler.wrapT    )) glTexParameteri(target, GL_TEXTURE_WRAP_T,     sampler.wrapT    );
		}

	protected:

		GLsizei m_size;

		TextureCube(Context * ctx)
			: BaseType (ctx)
			, m_size   (0)
		{
			;
		}

		bool create(const TextureCubeArguments & args)
		{
			this->destroy();
			GLint boundName = 0;
			glGetIntegerv(GL_TEXTURE_BINDING_CUBE_MAP, &boundName);
			glGenTextures(1, &(this->m_name));
			glBindTexture(GL_TEXTURE_CUBE_MAP, this->m_name);
			for (int f=0; f<6; ++f)
			{
				glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + f, 0, args.format, args.size, args.size, 0, args.dataFormat, args.dataType, args.data);
			}
			this->m_format = args.format;
			this->m_size   = args.size;
			this->setSampleMode(GL_TEXTURE_CUBE_MAP, 0, args.sampler);
			glBindTexture(GL_TEXTURE_CUBE_MAP, boundName);
			return true;
		}

		virtual void doDestroy(void)
		{
			BaseType::doDestroy();
			this->m_format = GL_NONE;
			this->m_size   = 0;
		}

		virtual bool doIsValid(void) const
		{
			return ((this->m_format != GL_NONE) && (this->m_size > 0));
		}
};

namespace detail { template <> struct BaseOf <TextureCube> { typedef Texture Type; }; };
typedef   detail::ObjectSharedPointerTraits  <TextureCube> ::Type TextureCubePtr;

class SafeTextureCube : public SafeTexture
{
	friend class Context;
	friend class BoundTextureCube;

	public:

		typedef SafeTexture   BaseType;
		typedef SafeTextureCube ThisType;

		SafeTextureCube(void)
			: BaseType()
		{
			;
		}

		GLsizei size(void) const
		{
			return this->object()->size();
		}

		GLsizei width(void) const
		{
			return this->object()->width();
		}

		GLsizei height(void) const
		{
			return this->object()->height();
		}

	protected:

		SafeTextureCube(const TextureCubePtr & texture2D)
			: BaseType(texture2D)
		{
			;
		}

		const TextureCubePtr & object(void) const
		{
			return static_cast<const TextureCubePtr &>(BaseType::object());
		}

		TextureCubePtr & object(void)
		{
			return static_cast<TextureCubePtr &>(BaseType::object());
		}
};

namespace detail { template <> struct BaseOf     <SafeTextureCube> { typedef SafeTexture   Type; }; };
namespace detail { template <> struct ObjectBase <SafeTextureCube> { typedef TextureCube     Type; }; };
namespace detail { template <> struct ObjectSafe <TextureCube    > { typedef SafeTextureCube Type; }; };
typedef   detail::ObjectSharedPointerTraits      <SafeTextureCube> ::Type TextureCubeHandle;

class TextureCubeBindingParams : public TextureBindingParams
{
	public:

		typedef TextureBindingParams   BaseType;
		typedef TextureCubeBindingParams ThisType;

		TextureCubeBindingParams(void)
			: BaseType()
		{
			;
		}

		TextureCubeBindingParams(GLenum aUnit)
			: BaseType(GL_TEXTURE_CUBE_MAP, aUnit)
		{
			;
		}
};

class BoundTextureCube : public BoundTexture
{
	friend class Context;

	public:

		typedef BoundTexture   BaseType;
		typedef BoundTextureCube ThisType;

		BoundTextureCube(void)
			: BaseType()
		{
			;
		}

		const TextureCubeHandle & handle(void) const
		{
			return static_cast<const TextureCubeHandle &>(BaseType::handle());
		}

		TextureCubeHandle & handle(void)
		{
			return static_cast<TextureCubeHandle &>(BaseType::handle());
		}

		void setSampleMode(const TextureSampleMode & sampler)
		{
			this->object()->setSampleMode(this->m_target, this->m_unit, sampler);
		}

		void setImage(GLenum face, GLint level, GLsizei size, GLenum dataFormat, GLenum dataType, const void * data)
		{
			this->object()->setImage(face, this->m_unit, level, size, dataFormat, dataType, data);
		}

		void getImage(GLenum face, GLint level, GLenum dataFormat, GLenum dataType, void * data)
		{
			this->object()->getImage(face, this->m_unit, level, dataFormat, dataType, data);
		}

		void setSubImage(GLenum face, GLint level, GLint xoffset, GLint yoffset, GLsizei width, GLsizei height, GLenum dataFormat, GLenum dataType, const void * data)
		{
			this->object()->setSubImage(face, this->m_unit, level, xoffset, yoffset, width, height, dataFormat, dataType, data);
		}

		void generateMipmap(void)
		{
			this->object()->generateMipmap(this->m_target, this->m_unit);
		}

	protected:

		BoundTextureCube(const TextureCubeHandle & handle, const TextureCubeBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}

		const TextureCubePtr & object(void) const
		{
			return this->handle()->object();
		}

		TextureCubePtr & object(void)
		{
			return this->handle()->object();
		}
};

namespace detail { template <> struct ParamsOf    <BoundTextureCube> { typedef TextureCubeBindingParams Type; }; };
namespace detail { template <> struct BaseOf      <BoundTextureCube> { typedef BoundObject Type; }; };
namespace detail { template <> struct ObjectBase  <BoundTextureCube> { typedef TextureCube      Type; }; };
namespace detail { template <> struct ObjectBound <TextureCube     > { typedef BoundTextureCube Type; }; };
typedef   detail::ObjectSharedPointerTraits       <BoundTextureCube> ::Type  BoundTextureCubeHandle;

};

#endif // GLW_TEXTURECUBE_H
