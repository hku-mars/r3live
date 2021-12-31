#ifndef GLW_RENDERABLE_H
#define GLW_RENDERABLE_H

#include <string>

#include "./object.h"

namespace glw
{

class RenderableArguments : public ObjectArguments
{
	public:

		typedef ObjectArguments     BaseType;
		typedef RenderableArguments ThisType;

		GLenum format;

		RenderableArguments(void)
			: BaseType ()
			, format   (GL_RGBA8)
		{
			;
		}

		RenderableArguments(GLenum aFormat)
			: BaseType ()
			, format   (aFormat)
		{
			;
		}

		void clear(void)
		{
			BaseType::clear();
			this->format = GL_RGBA8;
		}
};

class Renderable : public Object
{
	friend class Context;

	public:

		typedef Object     BaseType;
		typedef Renderable ThisType;

		GLenum format(void) const
		{
			return this->m_format;
		}

		virtual int  imageDimensions (void) const = 0;
		virtual bool isArray         (void) const = 0;

	protected:

		GLenum m_format;

		Renderable(Context * ctx)
			: BaseType (ctx)
			, m_format (GL_NONE)
		{
			;
		}
};

namespace detail { template <> struct BaseOf <Renderable> { typedef Object Type; }; };
typedef   detail::ObjectSharedPointerTraits  <Renderable> ::Type RenderablePtr;

class SafeRenderable : public SafeObject
{
	friend class Context;
	friend class BoundRenderable;

	public:

		typedef SafeObject     BaseType;
		typedef SafeRenderable ThisType;

		SafeRenderable(void)
			: BaseType()
		{
			;
		}

		GLenum format(void) const
		{
			return this->object()->format();
		}

		int imageDimensions(void) const
		{
			return this->object()->imageDimensions();
		}

		bool isArray(void) const
		{
			return this->object()->isArray();
		}

	protected:

		SafeRenderable(const RenderablePtr & renderable)
			: BaseType(renderable)
		{
			;
		}

		const RenderablePtr & object(void) const
		{
			return static_cast<const RenderablePtr &>(BaseType::object());
		}

		RenderablePtr & object(void)
		{
			return static_cast<RenderablePtr &>(BaseType::object());
		}
};

namespace detail { template <> struct BaseOf     <SafeRenderable> { typedef SafeObject     Type; }; };
namespace detail { template <> struct ObjectBase <SafeRenderable> { typedef Renderable     Type; }; };
namespace detail { template <> struct ObjectSafe <Renderable    > { typedef SafeRenderable Type; }; };
typedef   detail::ObjectSharedPointerTraits      <SafeRenderable> ::Type RenderableHandle;

class RenderableBindingParams : public ObjectBindingParams
{
	public:

		typedef ObjectBindingParams BaseType;
		typedef RenderableBindingParams ThisType;

		RenderableBindingParams(void)
			: BaseType()
		{
			;
		}

		RenderableBindingParams(GLenum aTarget, GLenum aUnit)
			: BaseType(aTarget, aUnit)
		{
			;
		}
};

class BoundRenderable : public BoundObject
{
	friend class Context;

	public:

		typedef BoundObject     BaseType;
		typedef BoundRenderable ThisType;

		BoundRenderable(void)
			: BaseType()
		{
			;
		}

		const RenderableHandle & handle(void) const
		{
			return static_cast<const RenderableHandle &>(BaseType::handle());
		}

		RenderableHandle & handle(void)
		{
			return static_cast<RenderableHandle &>(BaseType::handle());
		}

	protected:

		BoundRenderable(const RenderableHandle & handle, const RenderableBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}

		const RenderablePtr & object(void) const
		{
			return this->handle()->object();
		}

		RenderablePtr & object(void)
		{
			return this->handle()->object();
		}
};

namespace detail { template <> struct ParamsOf    <BoundRenderable> { typedef RenderableBindingParams Type; }; };
namespace detail { template <> struct BaseOf      <BoundRenderable> { typedef BoundObject Type; }; };
namespace detail { template <> struct ObjectBase  <BoundRenderable> { typedef Renderable      Type; }; };
namespace detail { template <> struct ObjectBound <Renderable     > { typedef BoundRenderable Type; }; };
typedef   detail::ObjectSharedPointerTraits       <BoundRenderable> ::Type  BoundRenderableHandle;

};

#endif // GLW_RENDERABLE_H
