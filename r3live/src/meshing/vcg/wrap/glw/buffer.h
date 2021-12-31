#ifndef GLW_BUFFER_H
#define GLW_BUFFER_H

#include "./object.h"

namespace glw
{

class BufferArguments : public ObjectArguments
{
	public:

		typedef ObjectArguments BaseType;
		typedef BufferArguments ThisType;

		GLsizeiptr   size;
		GLenum       usage;
		const void * data;

		BufferArguments(void)
			: BaseType ()
			, size     (0)
			, usage    (GL_STATIC_DRAW)
			, data     (0)
		{
			this->clear();
		}

		BufferArguments(GLsizeiptr aSize, GLenum aUsage = GL_STATIC_DRAW, const void * aData = 0)
			: BaseType ()
			, size     (aSize)
			, usage    (aUsage)
			, data     (aData)
		{
			;
		}

		void clear(void)
		{
			BaseType::clear();
			this->size  = 0;
			this->usage = GL_STATIC_DRAW;
			this->data  = 0;
		}
};

class Buffer : public Object
{
	friend class Context;

	public:

		typedef Object BaseType;
		typedef Buffer ThisType;

		virtual ~Buffer(void)
		{
			this->destroy();
		}

		virtual Type type(void) const
		{
			return BufferType;
		}

		GLsizeiptr size(void) const
		{
			return this->m_size;
		}

		GLenum usage(void) const
		{
			return this->m_usage;
		}

		void setData(GLenum target, GLint unit, const GLsizeiptr size, GLenum usage, const GLvoid * data)
		{
			(void)unit;
			GLW_ASSERT(this->isValid());
			glBufferData(target, size, data, usage);
			this->m_size  = size;
			this->m_usage = usage;
		}

		void setSubData(GLenum target, GLint unit, GLintptr offset, GLsizeiptr size, const GLvoid * data)
		{
			(void)unit;
			GLW_ASSERT(this->isValid());
			glBufferSubData(target, offset, size, data);
		}

		void getSubData(GLenum target, GLint unit, GLintptr offset, GLsizeiptr size, GLvoid * data)
		{
			(void)unit;
			GLW_ASSERT(this->isValid());
			glGetBufferSubData(target, offset, size, data);
		}

		void * map(GLenum target, GLint unit, GLenum access)
		{
			(void)unit;
			GLW_ASSERT(this->isValid());
			GLW_ASSERT(!this->isMapped(target, unit));
			void * ptr = glMapBuffer(target, access);
			if (ptr == 0) return 0;
			this->m_mapAccess  = access;
			this->m_mapPointer = ptr;
			return ptr;
		}

		void unmap(GLenum target, GLint unit)
		{
			(void)unit;
			GLW_ASSERT(this->isValid());
			GLW_ASSERT(this->isMapped(target, unit));
			glUnmapBuffer(target);
			this->m_mapAccess  = GL_NONE;
			this->m_mapPointer = 0;
		}

		GLenum mapAccess(GLenum target, GLint unit) const
		{
			(void)target;
			(void)unit;
			return this->m_mapAccess;
		}

		bool isMapped(GLenum target, GLint unit) const
		{
			(void)target;
			(void)unit;
			return (this->m_mapAccess != GL_NONE);
		}

		void * mapPointer(GLenum target, GLint unit) const
		{
			(void)target;
			(void)unit;
			return this->m_mapPointer;
		}

		void vertexAttribPointer(GLenum target, GLint unit, GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const GLvoid * offset)
		{
			(void)target;
			(void)unit;
			GLW_ASSERT(this->isValid());
			glVertexAttribPointer(index, size, type, normalized, stride, offset);
		}

		void drawElements(GLenum target, GLint unit, GLenum mode, GLsizei count, GLenum type, const GLvoid * indices)
		{
			(void)target;
			(void)unit;
			GLW_ASSERT(this->isValid());
			glDrawElements(mode, count, type, indices);
		}

	protected:

		GLsizeiptr m_size;
		GLenum     m_usage;
		GLenum     m_mapAccess;
		void *     m_mapPointer;

		Buffer(Context * ctx)
			: BaseType     (ctx)
			, m_size       (0)
			, m_usage      (GL_NONE)
			, m_mapAccess  (GL_NONE)
			, m_mapPointer (0)
		{
			;
		}

		bool create(const BufferArguments & args)
		{
			this->destroy();
			GLint boundName = 0;
			glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &boundName);
			glGenBuffers(1, &(this->m_name));
			glBindBuffer(GL_ARRAY_BUFFER, this->m_name);
			glBufferData(GL_ARRAY_BUFFER, args.size, args.data, args.usage);
			glBindBuffer(GL_ARRAY_BUFFER, boundName);
			this->m_size  = args.size;
			this->m_usage = args.usage;
			return true;
		}

		virtual void doDestroy(void)
		{
			glDeleteBuffers(1, &(this->m_name));
			this->m_size       = 0;
			this->m_usage      = GL_NONE;
			this->m_mapAccess  = GL_NONE;
			this->m_mapPointer = 0;
		}

		virtual bool doIsValid(void) const
		{
			return (this->m_size > 0);
		}
};

namespace detail { template <> struct BaseOf <Buffer> { typedef Object Type; }; };
typedef   detail::ObjectSharedPointerTraits  <Buffer> ::Type BufferPtr;

class SafeBuffer : public SafeObject
{
	friend class Context;
	friend class BoundBuffer;

	public:

		typedef SafeObject BaseType;
		typedef SafeBuffer ThisType;

		SafeBuffer(void)
			: BaseType()
		{
			;
		}

		GLsizeiptr size(void) const
		{
			return this->object()->size();
		}

		GLenum usage(void) const
		{
			return this->object()->usage();
		}

	protected:

		SafeBuffer(const BufferPtr & buffer)
			: BaseType(buffer)
		{
			;
		}

		const BufferPtr & object(void) const
		{
			return static_cast<const BufferPtr &>(BaseType::object());
		}

		BufferPtr & object(void)
		{
			return static_cast<BufferPtr &>(BaseType::object());
		}
};

namespace detail { template <> struct BaseOf     <SafeBuffer> { typedef SafeObject Type; }; };
namespace detail { template <> struct ObjectBase <SafeBuffer> { typedef Buffer     Type; }; };
namespace detail { template <> struct ObjectSafe <Buffer    > { typedef SafeBuffer Type; }; };
typedef   detail::ObjectSharedPointerTraits      <SafeBuffer> ::Type BufferHandle;

class BufferBindingParams : public ObjectBindingParams
{
	public:

		typedef ObjectBindingParams BaseType;
		typedef BufferBindingParams ThisType;

		BufferBindingParams(void)
			: BaseType()
		{
			;
		}

		BufferBindingParams(GLenum aTarget, GLenum aUnit)
			: BaseType(aTarget, aUnit)
		{
			;
		}
};

class BoundBuffer : public BoundObject
{
	friend class Context;

	public:

		typedef BoundObject BaseType;
		typedef BoundBuffer ThisType;

		BoundBuffer(void)
			: BaseType()
		{
			;
		}

		const BufferHandle & handle(void) const
		{
			return static_cast<const BufferHandle &>(BaseType::handle());
		}

		BufferHandle & handle(void)
		{
			return static_cast<BufferHandle &>(BaseType::handle());
		}

		void setData(const GLsizeiptr size, GLenum usage, const GLvoid * data)
		{
			this->object()->setData(this->m_target, this->m_unit, size, usage, data);
		}

		void setSubData(GLintptr offset, GLsizeiptr size, const GLvoid * data)
		{
			this->object()->setSubData(this->m_target, this->m_unit, offset, size, data);
		}

		void getSubData(GLintptr offset, GLsizeiptr size, GLvoid * data)
		{
			this->object()->getSubData(this->m_target, this->m_unit, offset, size, data);
		}

		void * map(GLenum access)
		{
			return this->object()->map(this->m_target, this->m_unit, access);
		}

		void unmap(void)
		{
			this->object()->unmap(this->m_target, this->m_unit);
		}

		GLenum mapAccess(void) const
		{
			return this->object()->mapAccess(this->m_target, this->m_unit);
		}

		bool isMapped(void) const
		{
			return this->object()->isMapped(this->m_target, this->m_unit);
		}

		void * mapPointer(void) const
		{
			return this->object()->mapPointer(this->m_target, this->m_unit);
		}

	protected:

		BoundBuffer(const BufferHandle & handle, const BufferBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}

		const BufferPtr & object(void) const
		{
			return this->handle()->object();
		}

		BufferPtr & object(void)
		{
			return this->handle()->object();
		}

		virtual void bind(void)
		{
			glBindBuffer(this->m_target, this->object()->name());
		}

		virtual void unbind(void)
		{
			glBindBuffer(this->m_target, 0);
		}
};

namespace detail { template <> struct ParamsOf    <BoundBuffer> { typedef BufferBindingParams Type; }; };
namespace detail { template <> struct BaseOf      <BoundBuffer> { typedef BoundObject Type; }; };
namespace detail { template <> struct ObjectBase  <BoundBuffer> { typedef Buffer      Type; }; };
namespace detail { template <> struct ObjectBound <Buffer     > { typedef BoundBuffer Type; }; };
typedef   detail::ObjectSharedPointerTraits       <BoundBuffer> ::Type  BoundBufferHandle;

class VertexBufferBindingParams : public BufferBindingParams
{
	public:

		typedef BufferBindingParams       BaseType;
		typedef VertexBufferBindingParams ThisType;

		VertexBufferBindingParams(void)
			: BaseType(GL_ARRAY_BUFFER, 0)
		{
			;
		}
};

class BoundVertexBuffer : public BoundBuffer
{
	friend class Context;

	public:

		typedef BoundBuffer       BaseType;
		typedef BoundVertexBuffer ThisType;

		BoundVertexBuffer(void)
			: BaseType()
		{
			;
		}

		void vertexAttribPointer(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const GLvoid * offset)
		{
			this->object()->vertexAttribPointer(this->m_target, this->m_unit, index, size, type, normalized, stride, offset);
		}

	protected:

		BoundVertexBuffer(const BufferHandle & handle, const VertexBufferBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}
};

namespace detail { template <> struct ParamsOf   <BoundVertexBuffer> { typedef VertexBufferBindingParams Type; }; };
namespace detail { template <> struct BaseOf     <BoundVertexBuffer> { typedef BoundBuffer Type; }; };
namespace detail { template <> struct ObjectBase <BoundVertexBuffer> { typedef Buffer      Type; }; };
typedef   detail::ObjectSharedPointerTraits      <BoundVertexBuffer> ::Type BoundVertexBufferHandle;

class IndexBufferBindingParams : public BufferBindingParams
{
	public:

		typedef BufferBindingParams      BaseType;
		typedef IndexBufferBindingParams ThisType;

		IndexBufferBindingParams(void)
			: BaseType(GL_ELEMENT_ARRAY_BUFFER, 0)
		{
			;
		}
};

class BoundIndexBuffer : public BoundBuffer
{
	friend class Context;

	public:

		typedef BoundBuffer       BaseType;
		typedef BoundIndexBuffer ThisType;

		BoundIndexBuffer(void)
			: BaseType()
		{
			;
		}

		void drawElements(GLenum mode, GLsizei count, GLenum type, const GLvoid * indices)
		{
			this->object()->drawElements(this->m_target, this->m_unit, mode, count, type, indices);
		}

	protected:

		BoundIndexBuffer(const BufferHandle & handle, const IndexBufferBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}
};

namespace detail { template <> struct ParamsOf   <BoundIndexBuffer> { typedef IndexBufferBindingParams Type; }; };
namespace detail { template <> struct BaseOf     <BoundIndexBuffer> { typedef BoundBuffer Type; }; };
namespace detail { template <> struct ObjectBase <BoundIndexBuffer> { typedef Buffer      Type; }; };
typedef   detail::ObjectSharedPointerTraits      <BoundIndexBuffer> ::Type BoundIndexBufferHandle;

class PixelPackBufferBindingParams : public BufferBindingParams
{
	public:

		typedef BufferBindingParams      BaseType;
		typedef PixelPackBufferBindingParams ThisType;

		PixelPackBufferBindingParams(void)
			: BaseType(GL_PIXEL_PACK_BUFFER, 0)
		{
			;
		}
};

class BoundPixelPackBuffer : public BoundBuffer
{
	friend class Context;

	public:

		typedef BoundBuffer          BaseType;
		typedef BoundPixelPackBuffer ThisType;

		BoundPixelPackBuffer(void)
			: BaseType()
		{
			;
		}

	protected:

		BoundPixelPackBuffer(const BufferHandle & handle, const PixelPackBufferBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}
};

namespace detail { template <> struct ParamsOf   <BoundPixelPackBuffer> { typedef PixelPackBufferBindingParams Type; }; };
namespace detail { template <> struct BaseOf     <BoundPixelPackBuffer> { typedef BoundBuffer Type; }; };
namespace detail { template <> struct ObjectBase <BoundPixelPackBuffer> { typedef Buffer      Type; }; };
typedef   detail::ObjectSharedPointerTraits      <BoundPixelPackBuffer> ::Type BoundPixelPackBufferHandle;

class PixelUnpackBufferBindingParams : public BufferBindingParams
{
	public:

		typedef BufferBindingParams      BaseType;
		typedef PixelUnpackBufferBindingParams ThisType;

		PixelUnpackBufferBindingParams(void)
			: BaseType(GL_PIXEL_UNPACK_BUFFER, 0)
		{
			;
		}
};

class BoundPixelUnpackBuffer : public BoundBuffer
{
	friend class Context;

	public:

		typedef BoundBuffer          BaseType;
		typedef BoundPixelUnpackBuffer ThisType;

		BoundPixelUnpackBuffer(void)
			: BaseType()
		{
			;
		}

	protected:

		BoundPixelUnpackBuffer(const BufferHandle & handle, const PixelUnpackBufferBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}
};

namespace detail { template <> struct ParamsOf   <BoundPixelUnpackBuffer> { typedef PixelUnpackBufferBindingParams Type; }; };
namespace detail { template <> struct BaseOf     <BoundPixelUnpackBuffer> { typedef BoundBuffer Type; }; };
namespace detail { template <> struct ObjectBase <BoundPixelUnpackBuffer> { typedef Buffer      Type; }; };
typedef   detail::ObjectSharedPointerTraits      <BoundPixelUnpackBuffer> ::Type BoundPixelUnpackBufferHandle;

class UniformBufferBindingParams : public BufferBindingParams
{
	public:

		typedef BufferBindingParams      BaseType;
		typedef UniformBufferBindingParams ThisType;

		GLintptr   offset;
		GLsizeiptr size;

		UniformBufferBindingParams(void)
			: BaseType (GL_UNIFORM_BUFFER, 0)
			, offset   (0)
			, size     (0)
		{
			;
		}

		UniformBufferBindingParams(GLuint aIndex, GLintptr aOffset, GLsizeiptr aSize)
			: BaseType (GL_UNIFORM_BUFFER, GLint(aIndex))
			, offset   (aOffset)
			, size     (aSize)
		{
			;
		}
};

class BoundUniformBuffer : public BoundBuffer
{
	friend class Context;

	public:

		typedef BoundBuffer          BaseType;
		typedef BoundUniformBuffer ThisType;

		BoundUniformBuffer(void)
			: BaseType()
		{
			;
		}

	protected:

		BoundUniformBuffer(const BufferHandle & handle, const UniformBufferBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}

		virtual void bind(void)
		{
			glBindBufferRange(this->m_target, GLuint(this->m_unit), this->object()->name(), this->m_offset, this->m_size);
		}

		virtual void unbind(void)
		{
			glBindBufferRange(this->m_target, GLuint(this->m_unit), 0, 0, 0);
		}

	private:

		GLintptr   m_offset;
		GLsizeiptr m_size;
};

namespace detail { template <> struct ParamsOf   <BoundUniformBuffer> { typedef UniformBufferBindingParams Type; }; };
namespace detail { template <> struct BaseOf     <BoundUniformBuffer> { typedef BoundBuffer Type; }; };
namespace detail { template <> struct ObjectBase <BoundUniformBuffer> { typedef Buffer      Type; }; };
typedef   detail::ObjectSharedPointerTraits      <BoundUniformBuffer> ::Type BoundUniformBufferHandle;

class FeedbackBufferBindingParams : public BufferBindingParams
{
	public:

		typedef BufferBindingParams      BaseType;
		typedef FeedbackBufferBindingParams ThisType;

		GLintptr   offset;
		GLsizeiptr size;

		FeedbackBufferBindingParams(void)
			: BaseType (GL_TRANSFORM_FEEDBACK_BUFFER, 0)
			, offset   (0)
			, size     (0)
		{
			;
		}

		FeedbackBufferBindingParams(GLuint aIndex, GLintptr aOffset, GLsizeiptr aSize)
			: BaseType (GL_TRANSFORM_FEEDBACK_BUFFER, GLint(aIndex))
			, offset   (aOffset)
			, size     (aSize)
		{
			;
		}
};

class BoundFeedbackBuffer : public BoundBuffer
{
	friend class Context;

	public:

		typedef BoundBuffer          BaseType;
		typedef BoundFeedbackBuffer ThisType;

		BoundFeedbackBuffer(void)
			: BaseType()
		{
			;
		}

	protected:

		BoundFeedbackBuffer(const BufferHandle & handle, const FeedbackBufferBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}

		virtual void bind(void)
		{
			glBindBufferRange(this->m_target, GLuint(this->m_unit), this->object()->name(), this->m_offset, this->m_size);
		}

		virtual void unbind(void)
		{
			glBindBufferRange(this->m_target, GLuint(this->m_unit), 0, 0, 0);
		}

	private:

		GLintptr   m_offset;
		GLsizeiptr m_size;
};

namespace detail { template <> struct ParamsOf   <BoundFeedbackBuffer> { typedef FeedbackBufferBindingParams Type; }; };
namespace detail { template <> struct BaseOf     <BoundFeedbackBuffer> { typedef BoundBuffer Type; }; };
namespace detail { template <> struct ObjectBase <BoundFeedbackBuffer> { typedef Buffer      Type; }; };
typedef   detail::ObjectSharedPointerTraits      <BoundFeedbackBuffer> ::Type BoundFeedbackBufferHandle;

};

#endif // GLW_BUFFER_H
