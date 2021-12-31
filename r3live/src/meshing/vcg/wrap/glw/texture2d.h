#ifndef GLW_TEXTURE2D_H
#define GLW_TEXTURE2D_H

#include "./texture.h"

namespace glw
{

class Texture2DArguments : public TextureArguments
{
	public:

		typedef TextureArguments   BaseType;
		typedef Texture2DArguments ThisType;

		GLsizei           width;
		GLsizei           height;
		GLenum            dataFormat;
		GLenum            dataType;
		const void *      data;
		TextureSampleMode sampler;

		Texture2DArguments(void)
		{
			this->clear();
		}

		void clear(void)
		{
			BaseType::clear();
			this->width      = 0;
			this->height     = 0;
			this->dataFormat = GL_NONE;
			this->dataType   = GL_NONE;
			this->data       = 0;
			this->sampler.clear();
		}
};

class Texture2D : public Texture
{
	friend class Context;

	public:

		typedef Texture   BaseType;
		typedef Texture2D ThisType;

		virtual Type type(void) const
		{
			return Texture2DType;
		}

		virtual int imageDimensions(void) const
		{
			return 2;
		}

		virtual bool isArray(void) const
		{
			return false;
		}

		GLsizei width(void) const
		{
			return this->m_width;
		}

		GLsizei height(void) const
		{
			return this->m_height;
		}

		void setImage(GLenum target, GLint unit, GLint level, GLsizei width, GLsizei height, GLenum dataFormat, GLenum dataType, const void * data)
		{
			(void)unit;
			GLW_ASSERT(this->isValid());
			if (level == 0)
			{
				this->m_width  = width;
				this->m_height = height;
			}
			glTexImage2D(target, level, this->m_format, width, height, 0, dataFormat, dataType, data);
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

		GLsizei m_width;
		GLsizei m_height;

		Texture2D(Context * ctx)
			: BaseType (ctx)
			, m_width  (0)
			, m_height (0)
		{
			;
		}

		bool create(const Texture2DArguments & args)
		{
			this->destroy();
			GLint boundName = 0;
			glGetIntegerv(GL_TEXTURE_BINDING_2D, &boundName);
			glGenTextures(1, &(this->m_name));
			glBindTexture(GL_TEXTURE_2D, this->m_name);
			glTexImage2D(GL_TEXTURE_2D, 0, args.format, args.width, args.height, 0, args.dataFormat, args.dataType, args.data);
			this->m_format = args.format;
			this->m_width  = args.width;
			this->m_height = args.height;
			this->setSampleMode(GL_TEXTURE_2D, 0, args.sampler);
			glBindTexture(GL_TEXTURE_2D, boundName);
			return true;
		}

		virtual void doDestroy(void)
		{
			BaseType::doDestroy();
			this->m_format = GL_NONE;
			this->m_width  = 0;
			this->m_height = 0;
		}

		virtual bool doIsValid(void) const
		{
			return ((this->m_format != GL_NONE) && (this->m_width > 0) && (this->m_height > 0));
		}
};

namespace detail { template <> struct BaseOf <Texture2D> { typedef Texture Type; }; };
typedef   detail::ObjectSharedPointerTraits  <Texture2D> ::Type Texture2DPtr;

class SafeTexture2D : public SafeTexture
{
	friend class Context;
	friend class BoundTexture2D;

	public:

		typedef SafeTexture   BaseType;
		typedef SafeTexture2D ThisType;

		SafeTexture2D(void)
			: BaseType()
		{
			;
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

		SafeTexture2D(const Texture2DPtr & texture2D)
			: BaseType(texture2D)
		{
			;
		}

		const Texture2DPtr & object(void) const
		{
			return static_cast<const Texture2DPtr &>(BaseType::object());
		}

		Texture2DPtr & object(void)
		{
			return static_cast<Texture2DPtr &>(BaseType::object());
		}
};

namespace detail { template <> struct BaseOf     <SafeTexture2D> { typedef SafeTexture   Type; }; };
namespace detail { template <> struct ObjectBase <SafeTexture2D> { typedef Texture2D     Type; }; };
namespace detail { template <> struct ObjectSafe <Texture2D    > { typedef SafeTexture2D Type; }; };
typedef   detail::ObjectSharedPointerTraits      <SafeTexture2D> ::Type Texture2DHandle;

class Texture2DBindingParams : public TextureBindingParams
{
	public:

		typedef TextureBindingParams   BaseType;
		typedef Texture2DBindingParams ThisType;

		Texture2DBindingParams(void)
			: BaseType()
		{
			;
		}

		Texture2DBindingParams(GLenum aUnit)
			: BaseType(GL_TEXTURE_2D, aUnit)
		{
			;
		}
};

class BoundTexture2D : public BoundTexture
{
	friend class Context;

	public:

		typedef BoundTexture   BaseType;
		typedef BoundTexture2D ThisType;

		BoundTexture2D(void)
			: BaseType()
		{
			;
		}

		const Texture2DHandle & handle(void) const
		{
			return static_cast<const Texture2DHandle &>(BaseType::handle());
		}

		Texture2DHandle & handle(void)
		{
			return static_cast<Texture2DHandle &>(BaseType::handle());
		}

		void setSampleMode(const TextureSampleMode & sampler)
		{
			this->object()->setSampleMode(this->m_target, this->m_unit, sampler);
		}

		void setImage(GLint level, GLsizei width, GLsizei height, GLenum dataFormat, GLenum dataType, const void * data)
		{
			this->object()->setImage(this->m_target, this->m_unit, level, width, height, dataFormat, dataType, data);
		}

		void getImage(GLint level, GLenum dataFormat, GLenum dataType, void * data)
		{
			this->object()->getImage(this->m_target, this->m_unit, level, dataFormat, dataType, data);
		}

		void setSubImage(GLint level, GLint xoffset, GLint yoffset, GLsizei width, GLsizei height, GLenum dataFormat, GLenum dataType, const void * data)
		{
			this->object()->setSubImage(this->m_target, this->m_unit, level, xoffset, yoffset, width, height, dataFormat, dataType, data);
		}

		void generateMipmap(void)
		{
			this->object()->generateMipmap(this->m_target, this->m_unit);
		}

	protected:

		BoundTexture2D(const Texture2DHandle & handle, const Texture2DBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}

		const Texture2DPtr & object(void) const
		{
			return this->handle()->object();
		}

		Texture2DPtr & object(void)
		{
			return this->handle()->object();
		}
};

namespace detail { template <> struct ParamsOf    <BoundTexture2D> { typedef Texture2DBindingParams Type; }; };
namespace detail { template <> struct BaseOf      <BoundTexture2D> { typedef BoundObject Type; }; };
namespace detail { template <> struct ObjectBase  <BoundTexture2D> { typedef Texture2D      Type; }; };
namespace detail { template <> struct ObjectBound <Texture2D     > { typedef BoundTexture2D Type; }; };
typedef   detail::ObjectSharedPointerTraits       <BoundTexture2D> ::Type  BoundTexture2DHandle;

};

#endif // GLW_TEXTURE2D_H
