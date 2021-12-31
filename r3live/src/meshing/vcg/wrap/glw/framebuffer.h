#ifndef GLW_FRAMEBUFFER_H
#define GLW_FRAMEBUFFER_H

#include "./texture2d.h"
#include "./renderbuffer.h"

#include <vector>
#include <map>
#include <set>

namespace glw
{

class RenderTarget
{
	public:

		typedef void         BaseType;
		typedef RenderTarget ThisType;

		RenderableHandle target;
		GLint            level;
		GLint            layer;
		GLenum           face;

		RenderTarget(void)
		{
			this->clear();
		}

		RenderTarget(RenderableHandle & rTarget, GLint rLevel, GLint rLayer, GLenum rFace)
			: target (rTarget)
			, level  (rLevel)
			, layer  (rLayer)
			, face   (rFace)
		{
			;
		}

		RenderTarget(RenderableHandle & rTarget)
			: target (rTarget)
			, level  (0)
			, layer  (0)
			, face   (GL_TEXTURE_CUBE_MAP_POSITIVE_X)
		{
			;
		}

		void clear(void)
		{
			this->target.setNull();
			this->level = 0;
			this->layer = -1;
			this->face  = GL_TEXTURE_CUBE_MAP_POSITIVE_X;
		}

		bool isNull(void) const
		{
			return this->target.isNull();
		}
};

typedef std::vector<RenderTarget> RenderTargetVector;

inline RenderTarget texture2DTarget(Texture2DHandle & handle, GLint level = 0)
{
	return RenderTarget(handle, level, 0, GL_NONE);
}

inline RenderTarget textureCubeTarget(TextureCubeHandle & handle, GLenum face = GL_TEXTURE_CUBE_MAP_POSITIVE_X, GLint level = 0)
{
	return RenderTarget(handle, level, 0, face);
}

inline RenderTarget renderbufferTarget(RenderbufferHandle & handle)
{
	return RenderTarget(handle, 0, 0, GL_NONE);
}

class RenderTargetMapping
{
	public:

		typedef void                BaseType;
		typedef RenderTargetMapping ThisType;

		typedef std::map<GLuint, RenderTarget> Map;
		typedef Map::const_iterator            ConstIterator;
		typedef Map::iterator                  Iterator;
		typedef Map::value_type                Value;

		Map bindings;

		RenderTargetMapping(void)
		{
			this->clear();
		}

		void clear(void)
		{
			this->bindings.clear();
		}

		const RenderTarget & operator [] (GLuint attachmentIndex) const
		{
			return this->bindings.find(attachmentIndex)->second;
		}

		RenderTarget & operator [] (GLuint attachmentIndex)
		{
			return this->bindings[attachmentIndex];
		}
};

class RenderTargetBinding
{
	public:

		typedef void                BaseType;
		typedef RenderTargetBinding ThisType;

		typedef std::map<GLuint, GLuint> Map;
		typedef Map::const_iterator      ConstIterator;
		typedef Map::iterator            Iterator;
		typedef Map::value_type          Value;

		Map bindings;

		RenderTargetBinding(void)
		{
			this->clear();
		}

		void clear(void)
		{
			this->bindings.clear();
		}

		GLuint operator [] (GLuint attachmentIndex) const
		{
			return this->bindings.find(attachmentIndex)->second;
		}

		GLuint & operator [] (GLuint attachmentIndex)
		{
			return this->bindings[attachmentIndex];
		}
};

class FramebufferArguments : public ObjectArguments
{
	public:

		typedef ObjectArguments      BaseType;
		typedef FramebufferArguments ThisType;

		RenderTargetMapping  colorTargets;
		RenderTarget         depthTarget;
		RenderTarget         stencilTarget;
		RenderTargetBinding  targetInputs;

		FramebufferArguments(void)
			: BaseType()
		{
			;
		}

		void clear(void)
		{
			BaseType::clear();
			this->colorTargets  .clear();
			this->depthTarget   .clear();
			this->stencilTarget .clear();
			this->targetInputs  .clear();
		}
};

class Framebuffer : public Object
{
	friend class Context;

	public:

		typedef Object      BaseType;
		typedef Framebuffer ThisType;

		virtual ~Framebuffer(void)
		{
			this->destroy();
		}

		virtual Type type(void) const
		{
			return FramebufferType;
		}

		const FramebufferArguments & arguments(void) const
		{
			return this->m_config;
		}

		bool setColorTarget(GLenum target, GLint unit, GLint index, const RenderTarget & renderTarget)
		{
			(void)unit;
			GLW_ASSERT(this->isValid());
			this->m_config.colorTargets[index].clear();
			const bool r = this->attachTarget(target, GL_COLOR_ATTACHMENT0 + index, renderTarget);
			if (!r) return false;
			this->m_config.colorTargets[index] = renderTarget;
			return true;
		}

		bool removeColorTarget(GLenum target, GLint unit, GLint index)
		{
			(void)unit;
			GLW_ASSERT(this->isValid());
			glFramebufferRenderbuffer(target, GL_COLOR_ATTACHMENT0 + index, GL_RENDERBUFFER, 0);
			this->m_config.colorTargets[index].clear();
			return true;
		}

		bool removeAllColorTargets(GLenum target, GLint unit)
		{
			(void)unit;
			for (RenderTargetMapping::ConstIterator it=this->m_config.colorTargets.bindings.begin(); it!=this->m_config.colorTargets.bindings.end(); ++it)
			{
				glFramebufferRenderbuffer(target, GL_COLOR_ATTACHMENT0 + it->first, GL_RENDERBUFFER, 0);
			}
			this->m_config.colorTargets.clear();
			return true;
		}

		bool setDepthTarget(GLenum target, GLint unit, const RenderTarget & renderTarget)
		{
			(void)unit;
			GLW_ASSERT(this->isValid());
			this->m_config.depthTarget.clear();
			const bool r = this->attachTarget(target, GL_DEPTH_ATTACHMENT, renderTarget);
			if (!r) return false;
			this->m_config.depthTarget = renderTarget;
			return true;
		}

		bool removeDepthTarget(GLenum target, GLint unit)
		{
			(void)unit;
			GLW_ASSERT(this->isValid());
			glFramebufferRenderbuffer(target, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
			this->m_config.depthTarget.clear();
			return true;
		}

		bool setStencilTarget(GLenum target, GLint unit, const RenderTarget & renderTarget)
		{
			(void)unit;
			GLW_ASSERT(this->isValid());
			this->m_config.stencilTarget.clear();
			const bool r = this->attachTarget(target, GL_STENCIL_ATTACHMENT, renderTarget);
			if (!r) return false;
			this->m_config.stencilTarget = renderTarget;
			return true;
		}

		bool removeStencilTarget(GLenum target, GLint unit)
		{
			(void)unit;
			GLW_ASSERT(this->isValid());
			glFramebufferRenderbuffer(target, GL_STENCIL_ATTACHMENT, GL_RENDERBUFFER, 0);
			this->m_config.stencilTarget.clear();
			return true;
		}

		bool removeAllTargets(GLenum target, GLint unit)
		{
			this->removeAllColorTargets (target, unit);
			this->removeDepthTarget     (target, unit);
			this->removeStencilTarget   (target, unit);
			return true;
		}

		bool setTargetInputs(GLenum target, GLint unit, const RenderTargetBinding & targetInputs)
		{
			(void)target;
			(void)unit;
			GLW_ASSERT(this->isValid());
			this->configureTargetInputs(targetInputs);
			return true;
		}

		bool readColorPixels(GLenum target, GLint unit, GLint index, GLint x, GLint y, GLsizei width, GLsizei height, GLenum format, GLenum type, GLvoid * data)
		{
			(void)target;
			(void)unit;
			GLW_ASSERT(this->isValid());
			if (this->m_config.colorTargets.bindings.count(index) <= 0)
			{
				GLW_ASSERT(0);
				return false;
			}
			glReadBuffer(GL_COLOR_ATTACHMENT0 + index);
			glReadPixels(x, y, width, height, format, type, data);
			return true;
		}

		bool readDepthPixels(GLenum target, GLint unit, GLint x, GLint y, GLsizei width, GLsizei height, GLenum type, GLvoid * data)
		{
			(void)target;
			(void)unit;
			GLW_ASSERT(this->isValid());
			if (this->m_config.depthTarget.isNull())
			{
				GLW_ASSERT(0);
				return false;
			}
			glReadPixels(x, y, width, height, GL_DEPTH_COMPONENT, type, data);
			return true;
		}

		bool readStencilPixels(GLenum target, GLint unit, GLint x, GLint y, GLsizei width, GLsizei height, GLenum type, GLvoid * data)
		{
			(void)target;
			(void)unit;
			GLW_ASSERT(this->isValid());
			if (this->m_config.stencilTarget.isNull())
			{
				GLW_ASSERT(0);
				return false;
			}
			glReadPixels(x, y, width, height, GL_STENCIL_INDEX, type, data);
			return true;
		}

	protected:

		Framebuffer(Context * ctx)
			: BaseType(ctx)
		{
			;
		}

		bool create(const FramebufferArguments & args)
		{
			this->destroy();

			GLint boundNameDraw = 0;
			glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &boundNameDraw);

			GLint boundNameRead = 0;
			glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &boundNameRead);

			glGenFramebuffers(1, &(this->m_name));
			glBindFramebuffer(GL_FRAMEBUFFER, this->m_name);
			this->configure(GL_FRAMEBUFFER, args);
			glBindFramebuffer(GL_FRAMEBUFFER, 0);

			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, boundNameDraw);
			glBindFramebuffer(GL_READ_FRAMEBUFFER, boundNameRead);

			return true;
		}

		virtual void doDestroy(void)
		{
			glDeleteFramebuffers(1, &(this->m_name));
			this->m_config.clear();
		}

		virtual bool doIsValid(void) const
		{
			return true;
		}

	private:

		FramebufferArguments m_config;

		void configure(GLenum target, const FramebufferArguments & args)
		{
			this->m_config.clear();

			for (RenderTargetMapping::ConstIterator it=args.colorTargets.bindings.begin(); it!=args.colorTargets.bindings.end(); ++it)
			{
				const bool colorAttached = this->attachTarget(target, GL_COLOR_ATTACHMENT0 + it->first, it->second);
				if (!colorAttached) continue;
				this->m_config.colorTargets[it->first] = it->second;
			}

			const bool depthAttached = this->attachTarget(target, GL_DEPTH_ATTACHMENT, args.depthTarget);
			if (depthAttached) this->m_config.depthTarget = args.depthTarget;

			const bool stencilAttached = this->attachTarget(target, GL_STENCIL_ATTACHMENT, args.stencilTarget);
			if (stencilAttached) this->m_config.stencilTarget = args.stencilTarget;

			this->configureTargetInputs(args.targetInputs);
		}

		bool attachTarget(GLenum target, GLenum attachment, const RenderTarget & renderTarget)
		{
			const RenderableHandle & handle = renderTarget.target;

			if (!handle)
			{
				glFramebufferRenderbuffer(target, attachment, GL_RENDERBUFFER, 0);
				return false;
			}

			switch (handle->type())
			{
				case RenderbufferType : glFramebufferRenderbuffer (target, attachment, GL_RENDERBUFFER,   handle->name()                    ); break;
				case Texture2DType    : glFramebufferTexture2D    (target, attachment, GL_TEXTURE_2D,     handle->name(), renderTarget.level); break;
				case TextureCubeType  : glFramebufferTexture2D    (target, attachment, renderTarget.face, handle->name(), renderTarget.level); break;
				default               : GLW_ASSERT(0);                                                                                         break;
			}

			return true;
		}

		void configureTargetInputs(const RenderTargetBinding & targetInputs)
		{
			if (this->m_config.colorTargets.bindings.empty() && targetInputs.bindings.empty())
			{
				glDrawBuffer(GL_NONE);
				glReadBuffer(GL_NONE);
				return;
			}

			std::vector<GLenum> drawBuffers;
			drawBuffers.reserve(targetInputs.bindings.size());
			for (RenderTargetBinding::ConstIterator it=targetInputs.bindings.begin(); it!=targetInputs.bindings.end(); ++it)
			{
				const GLuint fragOutput      = it->second;
				const GLuint attachmentIndex = GL_COLOR_ATTACHMENT0 + it->first;
				if (drawBuffers.size() <= size_t(fragOutput))
				{
					drawBuffers.resize(size_t(fragOutput + 1), GL_NONE);
				}
				drawBuffers[fragOutput] = attachmentIndex;
				this->m_config.targetInputs[it->first] = fragOutput;
			}
			glDrawBuffers(GLsizei(drawBuffers.size()), &(drawBuffers[0]));
			glReadBuffer(drawBuffers[0]);
		}
};

namespace detail { template <> struct BaseOf <Framebuffer> { typedef Object Type; }; };
typedef   detail::ObjectSharedPointerTraits  <Framebuffer> ::Type FramebufferPtr;

class SafeFramebuffer : public SafeObject
{
	friend class Context;
	friend class BoundFramebuffer;

	public:

		typedef SafeObject  BaseType;
		typedef SafeFramebuffer ThisType;

		const FramebufferArguments & arguments(void) const
		{
			return this->object()->arguments();
		}

	protected:

		SafeFramebuffer(const FramebufferPtr & program)
			: BaseType(program)
		{
			;
		}

		const FramebufferPtr & object(void) const
		{
			return static_cast<const FramebufferPtr &>(BaseType::object());
		}

		FramebufferPtr & object(void)
		{
			return static_cast<FramebufferPtr &>(BaseType::object());
		}
};

namespace detail { template <> struct BaseOf     <SafeFramebuffer> { typedef SafeObject Type; }; };
namespace detail { template <> struct ObjectBase <SafeFramebuffer> { typedef Framebuffer     Type; }; };
namespace detail { template <> struct ObjectSafe <Framebuffer    > { typedef SafeFramebuffer Type; }; };
typedef   detail::ObjectSharedPointerTraits      <SafeFramebuffer> ::Type FramebufferHandle;

class FramebufferBindingParams : public ObjectBindingParams
{
	public:

		typedef ObjectBindingParams      BaseType;
		typedef FramebufferBindingParams ThisType;

		FramebufferBindingParams(void)
			: BaseType()
		{
			;
		}

		FramebufferBindingParams(GLenum target)
			: BaseType(target, 0)
		{
			;
		}
};

class BoundFramebuffer : public BoundObject
{
	friend class Context;

	public:

		typedef BoundObject BaseType;
		typedef BoundFramebuffer ThisType;

		BoundFramebuffer(void)
			: BaseType()
		{
			;
		}

		const FramebufferHandle & handle(void) const
		{
			return static_cast<const FramebufferHandle &>(BaseType::handle());
		}

		FramebufferHandle & handle(void)
		{
			return static_cast<FramebufferHandle &>(BaseType::handle());
		}

		GLenum getStatus(void) const
		{
			return glCheckFramebufferStatus(this->m_target);
		}

		bool isComplete(void) const
		{
			return (this->getStatus() == GL_FRAMEBUFFER_COMPLETE);
		}

		bool setColorTarget(GLint index, const RenderTarget & renderTarget)
		{
			return this->object()->setColorTarget(this->m_target, this->m_unit, index, renderTarget);
		}

		bool removeColorTarget(GLint index)
		{
			return this->object()->removeColorTarget(this->m_target, this->m_unit, index);
		}

		bool removeAllColorTargets(void)
		{
			return this->object()->removeAllColorTargets(this->m_target, this->m_unit);
		}

		bool setDepthTarget(const RenderTarget & renderTarget)
		{
			return this->object()->setDepthTarget(this->m_target, this->m_unit, renderTarget);
		}

		bool removeDepthTarget(void)
		{
			return this->object()->removeDepthTarget(this->m_target, this->m_unit);
		}

		bool setStencilTarget(const RenderTarget & renderTarget)
		{
			return this->object()->setStencilTarget(this->m_target, this->m_unit, renderTarget);
		}

		bool removeStencilTarget(void)
		{
			return this->object()->removeStencilTarget(this->m_target, this->m_unit);
		}

		bool removeAllTargets(void)
		{
			return this->object()->removeAllTargets(this->m_target, this->m_unit);
		}

		bool setTargetInputs(const RenderTargetBinding & targetInputs)
		{
			return this->object()->setTargetInputs(this->m_target, this->m_unit, targetInputs);
		}

	protected:

		BoundFramebuffer(const FramebufferHandle & handle, const FramebufferBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}

		const FramebufferPtr & object(void) const
		{
			return this->handle()->object();
		}

		FramebufferPtr & object(void)
		{
			return this->handle()->object();
		}

		virtual void bind(void)
		{
			glBindFramebuffer(this->m_target, this->object()->name());
		}

		virtual void unbind(void)
		{
			glBindFramebuffer(this->m_target, 0);
		}
};

namespace detail { template <> struct ParamsOf    <BoundFramebuffer> { typedef FramebufferBindingParams Type; }; };
namespace detail { template <> struct BaseOf      <BoundFramebuffer> { typedef BoundObject Type; }; };
namespace detail { template <> struct ObjectBase  <BoundFramebuffer> { typedef Framebuffer      Type; }; };
namespace detail { template <> struct ObjectBound <Framebuffer     > { typedef BoundFramebuffer Type; }; };
typedef   detail::ObjectSharedPointerTraits       <BoundFramebuffer> ::Type  BoundFramebufferHandle;

class ReadFramebufferBindingParams : public FramebufferBindingParams
{
	public:

		typedef FramebufferBindingParams     BaseType;
		typedef ReadFramebufferBindingParams ThisType;

		ReadFramebufferBindingParams(void)
			: BaseType(GL_READ_FRAMEBUFFER)
		{
			;
		}
};

class BoundReadFramebuffer : public BoundFramebuffer
{
	friend class Context;

	public:

		typedef BoundFramebuffer     BaseType;
		typedef BoundReadFramebuffer ThisType;

		BoundReadFramebuffer(void)
			: BaseType()
		{
			;
		}

		bool readColorPixels(GLint index, GLint x, GLint y, GLsizei width, GLsizei height, GLenum format, GLenum type, GLvoid * data)
		{
			return this->object()->readColorPixels(this->m_target, this->m_unit, index, x, y, width, height, format, type, data);
		}

		bool readDepthPixels(GLint x, GLint y, GLsizei width, GLsizei height, GLenum type, GLvoid * data)
		{
			return this->object()->readDepthPixels(this->m_target, this->m_unit, x, y, width, height, type, data);
		}

		bool readStencilPixels(GLint x, GLint y, GLsizei width, GLsizei height, GLenum type, GLvoid * data)
		{
			return this->object()->readStencilPixels(this->m_target, this->m_unit, x, y, width, height, type, data);
		}

	protected:

		BoundReadFramebuffer(const FramebufferHandle & handle, const ReadFramebufferBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}
};

namespace detail { template <> struct ParamsOf   <BoundReadFramebuffer> { typedef ReadFramebufferBindingParams Type; }; };
namespace detail { template <> struct BaseOf     <BoundReadFramebuffer> { typedef BoundFramebuffer Type; }; };
namespace detail { template <> struct ObjectBase <BoundReadFramebuffer> { typedef Framebuffer      Type; }; };
typedef   detail::ObjectSharedPointerTraits      <BoundReadFramebuffer> ::Type BoundReadFramebufferHandle;

class DrawFramebufferBindingParams : public FramebufferBindingParams
{
	public:

		typedef FramebufferBindingParams     BaseType;
		typedef DrawFramebufferBindingParams ThisType;

		DrawFramebufferBindingParams(void)
			: BaseType(GL_DRAW_FRAMEBUFFER)
		{
			;
		}
};

class BoundDrawFramebuffer : public BoundFramebuffer
{
	friend class Context;

	public:

		typedef BoundFramebuffer     BaseType;
		typedef BoundDrawFramebuffer ThisType;

		BoundDrawFramebuffer(void)
			: BaseType()
		{
			;
		}

	protected:

		BoundDrawFramebuffer(const FramebufferHandle & handle, const DrawFramebufferBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}
};

namespace detail { template <> struct ParamsOf   <BoundDrawFramebuffer> { typedef DrawFramebufferBindingParams Type; }; };
namespace detail { template <> struct BaseOf     <BoundDrawFramebuffer> { typedef BoundFramebuffer Type; }; };
namespace detail { template <> struct ObjectBase <BoundDrawFramebuffer> { typedef Framebuffer      Type; }; };
typedef   detail::ObjectSharedPointerTraits      <BoundDrawFramebuffer> ::Type BoundDrawFramebufferHandle;

class ReadDrawFramebufferBindingParams : public FramebufferBindingParams
{
	public:

		typedef FramebufferBindingParams     BaseType;
		typedef ReadDrawFramebufferBindingParams ThisType;

		ReadDrawFramebufferBindingParams(void)
			: BaseType(GL_FRAMEBUFFER)
		{
			;
		}
};

class BoundReadDrawFramebuffer : public BoundFramebuffer
{
	friend class Context;

	public:

		typedef BoundFramebuffer     BaseType;
		typedef BoundReadDrawFramebuffer ThisType;

		BoundReadDrawFramebuffer(void)
			: BaseType()
		{
			;
		}

		bool readColorPixels(GLint index, GLint x, GLint y, GLsizei width, GLsizei height, GLenum format, GLenum type, GLvoid * data)
		{
			return this->object()->readColorPixels(this->m_target, this->m_unit, index, x, y, width, height, format, type, data);
		}

		bool readDepthPixels(GLint x, GLint y, GLsizei width, GLsizei height, GLenum type, GLvoid * data)
		{
			return this->object()->readDepthPixels(this->m_target, this->m_unit, x, y, width, height, type, data);
		}

		bool readStencilPixels(GLint x, GLint y, GLsizei width, GLsizei height, GLenum type, GLvoid * data)
		{
			return this->object()->readStencilPixels(this->m_target, this->m_unit, x, y, width, height, type, data);
		}

	protected:

		BoundReadDrawFramebuffer(const FramebufferHandle & handle, const ReadDrawFramebufferBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}
};

namespace detail { template <> struct ParamsOf   <BoundReadDrawFramebuffer> { typedef ReadDrawFramebufferBindingParams Type; }; };
namespace detail { template <> struct BaseOf     <BoundReadDrawFramebuffer> { typedef BoundFramebuffer Type; }; };
namespace detail { template <> struct ObjectBase <BoundReadDrawFramebuffer> { typedef Framebuffer      Type; }; };
typedef   detail::ObjectSharedPointerTraits      <BoundReadDrawFramebuffer> ::Type BoundReadDrawFramebufferHandle;

};

#endif // GLW_FRAMEBUFFER_H
