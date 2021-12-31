#ifndef GLW_RENDERBUFFER_H
#define GLW_RENDERBUFFER_H

#include "./renderable.h"

namespace glw
{

class RenderbufferArguments : public RenderableArguments
{
	public:

		typedef RenderableArguments   BaseType;
		typedef RenderbufferArguments ThisType;

		GLsizei width;
		GLsizei height;

		RenderbufferArguments(void)
			: BaseType ()
			, width    (0)
			, height   (0)
		{
		}

		RenderbufferArguments(GLenum aFormat, GLsizei aWidth, GLsizei aHeight)
			: BaseType (aFormat)
			, width    (aWidth)
			, height   (aHeight)
		{
			;
		}

		void clear(void)
		{
			BaseType::clear();
			this->width  = 0;
			this->height = 0;
		}
};

class Renderbuffer : public Renderable
{
	friend class Context;

	public:

		typedef Renderable   BaseType;
		typedef Renderbuffer ThisType;

		virtual ~Renderbuffer(void)
		{
			this->destroy();
		}

		virtual Type type(void) const
		{
			return RenderbufferType;
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

		void setStorage(GLenum target, GLint unit, GLenum format, GLsizei width, GLsizei height)
		{
			(void)unit;
			GLW_ASSERT(this->isValid());
			glRenderbufferStorage(target, format, width, height);
			this->m_format = format;
			this->m_width  = width;
			this->m_height = height;
		}

	protected:

		GLsizei m_width;
		GLsizei m_height;

		Renderbuffer(Context * ctx)
			: BaseType (ctx)
			, m_width  (0)
			, m_height (0)
		{
			;
		}

		bool create(const RenderbufferArguments & args)
		{
			this->destroy();
			GLint boundName = 0;
			glGetIntegerv(GL_RENDERBUFFER_BINDING, &boundName);
			glGenRenderbuffers(1, &(this->m_name));
			glBindRenderbuffer(GL_RENDERBUFFER, this->m_name);
			glRenderbufferStorage(GL_RENDERBUFFER, args.format, args.width, args.height);
			glBindRenderbuffer(GL_RENDERBUFFER, boundName);
			this->m_format = args.format;
			this->m_width  = args.width;
			this->m_height = args.height;
			return true;
		}

		virtual void doDestroy(void)
		{
			glDeleteRenderbuffers(1, &(this->m_name));
			this->m_format = GL_NONE;
			this->m_width  = 0;
			this->m_height = 0;
		}

		virtual bool doIsValid(void) const
		{
			return ((this->m_format != GL_NONE) && (this->m_width > 0) && (this->m_height > 0));
		}
};

namespace detail { template <> struct BaseOf <Renderbuffer> { typedef Renderable Type; }; };
typedef   detail::ObjectSharedPointerTraits  <Renderbuffer> ::Type RenderbufferPtr;

class SafeRenderbuffer : public SafeRenderable
{
	friend class Context;
	friend class BoundRenderbuffer;

	public:

		typedef SafeRenderable   BaseType;
		typedef SafeRenderbuffer ThisType;

		SafeRenderbuffer(void)
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

		SafeRenderbuffer(const RenderbufferPtr & renderbuffer)
			: BaseType(renderbuffer)
		{
			;
		}

		const RenderbufferPtr & object(void) const
		{
			return static_cast<const RenderbufferPtr &>(BaseType::object());
		}

		RenderbufferPtr & object(void)
		{
			return static_cast<RenderbufferPtr &>(BaseType::object());
		}
};

namespace detail { template <> struct BaseOf     <SafeRenderbuffer> { typedef SafeRenderable   Type; }; };
namespace detail { template <> struct ObjectBase <SafeRenderbuffer> { typedef Renderbuffer     Type; }; };
namespace detail { template <> struct ObjectSafe <Renderbuffer    > { typedef SafeRenderbuffer Type; }; };
typedef   detail::ObjectSharedPointerTraits      <SafeRenderbuffer> ::Type RenderbufferHandle;

class RenderbufferBindingParams : public RenderableBindingParams
{
	public:

		typedef RenderableBindingParams   BaseType;
		typedef RenderbufferBindingParams ThisType;

		RenderbufferBindingParams(void)
			: BaseType(GL_RENDERBUFFER, 0)
		{
			;
		}
};

class BoundRenderbuffer : public BoundRenderable
{
	friend class Context;

	public:

		typedef BoundRenderable   BaseType;
		typedef BoundRenderbuffer ThisType;

		BoundRenderbuffer(void)
			: BaseType()
		{
			;
		}

		const RenderbufferHandle & handle(void) const
		{
			return static_cast<const RenderbufferHandle &>(BaseType::handle());
		}

		RenderbufferHandle & handle(void)
		{
			return static_cast<RenderbufferHandle &>(BaseType::handle());
		}

		void setStorage(GLenum format, GLsizei width, GLsizei height)
		{
			this->object()->setStorage(this->m_target, this->m_unit, format, width, height);
		}

	protected:

		BoundRenderbuffer(const RenderbufferHandle & handle, const RenderbufferBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}

		const RenderbufferPtr & object(void) const
		{
			return this->handle()->object();
		}

		RenderbufferPtr & object(void)
		{
			return this->handle()->object();
		}

		virtual void bind(void)
		{
			glBindRenderbuffer(this->m_target, this->object()->name());
		}

		virtual void unbind(void)
		{
			glBindRenderbuffer(this->m_target, 0);
		}
};

namespace detail { template <> struct ParamsOf    <BoundRenderbuffer> { typedef RenderbufferBindingParams Type; }; };
namespace detail { template <> struct BaseOf      <BoundRenderbuffer> { typedef BoundObject Type; }; };
namespace detail { template <> struct ObjectBase  <BoundRenderbuffer> { typedef Renderbuffer      Type; }; };
namespace detail { template <> struct ObjectBound <Renderbuffer     > { typedef BoundRenderbuffer Type; }; };
typedef   detail::ObjectSharedPointerTraits       <BoundRenderbuffer> ::Type  BoundRenderbufferHandle;

};

#endif // GLW_RENDERBUFFER_H
