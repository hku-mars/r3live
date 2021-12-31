#ifndef GLW_TEXTURE_H
#define GLW_TEXTURE_H

#include "./renderable.h"

namespace glw
{

class TextureSampleMode
{
	public:

		typedef void              BaseType;
		typedef TextureSampleMode ThisType;

		GLenum minFilter;
		GLenum magFilter;
		GLenum wrapS;
		GLenum wrapT;
		GLenum wrapR;

		TextureSampleMode(void)
		{
			this->clear();
		}

		TextureSampleMode(GLenum rMinFilter, GLenum rMagFilter, GLenum rWrapS, GLenum rWrapT, GLenum rWrapR)
			: minFilter (rMinFilter)
			, magFilter (rMagFilter)
			, wrapS     (rWrapS)
			, wrapT     (rWrapT)
			, wrapR     (rWrapR)
		{
			;
		}

		void clear(void)
		{
			this->minFilter = GLW_DONT_CARE;
			this->magFilter = GLW_DONT_CARE;
			this->wrapS     = GLW_DONT_CARE;
			this->wrapT     = GLW_DONT_CARE;
			this->wrapR     = GLW_DONT_CARE;
		}

		static TextureSampleMode dontCare(void)
		{
			return ThisType();
		}

		static TextureSampleMode texelFetch(void)
		{
			return ThisType(GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE);
		}
};

inline TextureSampleMode textureSampleMode(GLenum minFilter = GLW_DONT_CARE, GLenum magFilter = GLW_DONT_CARE, GLenum wrapS = GLW_DONT_CARE, GLenum wrapT = GLW_DONT_CARE, GLenum wrapR = GLW_DONT_CARE)
{
	return TextureSampleMode(minFilter, magFilter, wrapS, wrapT, wrapR);
}

class TextureArguments : public RenderableArguments
{
	public:

		typedef RenderableArguments BaseType;
		typedef TextureArguments    ThisType;

		TextureArguments(void)
		{
			this->clear();
		}

		void clear(void)
		{
			BaseType::clear();
		}
};

class Texture : public Renderable
{
	friend class Context;

	public:

		typedef Renderable  BaseType;
		typedef Texture     ThisType;

		virtual ~Texture(void)
		{
			this->destroy();
		}

	protected:

		Texture(Context * ctx)
			: BaseType(ctx)
		{
			;
		}

		virtual void doDestroy(void)
		{
			glDeleteTextures(1, &(this->m_name));
		}
};

namespace detail { template <> struct BaseOf <Texture> { typedef Renderable Type; }; };
typedef   detail::ObjectSharedPointerTraits  <Texture> ::Type TexturePtr;

class SafeTexture : public SafeRenderable
{
	friend class Context;
	friend class BoundTexture;

	public:

		typedef SafeRenderable   BaseType;
		typedef SafeTexture ThisType;

		SafeTexture(void)
			: BaseType()
		{
			;
		}

	protected:

		SafeTexture(const TexturePtr & texture)
			: BaseType(texture)
		{
			;
		}

		const TexturePtr & object(void) const
		{
			return static_cast<const TexturePtr &>(BaseType::object());
		}

		TexturePtr & object(void)
		{
			return static_cast<TexturePtr &>(BaseType::object());
		}
};

namespace detail { template <> struct BaseOf     <SafeTexture> { typedef SafeRenderable   Type; }; };
namespace detail { template <> struct ObjectBase <SafeTexture> { typedef Texture     Type; }; };
namespace detail { template <> struct ObjectSafe <Texture    > { typedef SafeTexture Type; }; };
typedef   detail::ObjectSharedPointerTraits      <SafeTexture> ::Type TextureHandle;

class TextureBindingParams : public RenderableBindingParams
{
	public:

		typedef RenderableBindingParams   BaseType;
		typedef TextureBindingParams ThisType;

		TextureBindingParams(void)
			: BaseType()
		{
			;
		}

		TextureBindingParams(GLenum aTarget, GLenum aUnit)
			: BaseType(aTarget, aUnit)
		{
			;
		}
};

class BoundTexture : public BoundRenderable
{
	friend class Context;

	public:

		typedef BoundRenderable   BaseType;
		typedef BoundTexture ThisType;

		BoundTexture(void)
			: BaseType()
		{
			;
		}

		const TextureHandle & handle(void) const
		{
			return static_cast<const TextureHandle &>(BaseType::handle());
		}

		TextureHandle & handle(void)
		{
			return static_cast<TextureHandle &>(BaseType::handle());
		}

	protected:

		BoundTexture(const TextureHandle & handle, const TextureBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}

		const TexturePtr & object(void) const
		{
			return this->handle()->object();
		}

		TexturePtr & object(void)
		{
			return this->handle()->object();
		}

		virtual void bind(void)
		{
			glActiveTexture(GL_TEXTURE0 + this->m_unit);
			glBindTexture(this->m_target, this->object()->name());
		}

		virtual void unbind(void)
		{
			glActiveTexture(GL_TEXTURE0 + this->m_unit);
			glBindTexture(this->m_target, 0);
		}
};

namespace detail { template <> struct ParamsOf    <BoundTexture> { typedef TextureBindingParams Type; }; };
namespace detail { template <> struct BaseOf      <BoundTexture> { typedef BoundObject Type; }; };
namespace detail { template <> struct ObjectBase  <BoundTexture> { typedef Texture      Type; }; };
namespace detail { template <> struct ObjectBound <Texture     > { typedef BoundTexture Type; }; };
typedef   detail::ObjectSharedPointerTraits       <BoundTexture> ::Type  BoundTextureHandle;

};

#endif // GLW_TEXTURE_H
