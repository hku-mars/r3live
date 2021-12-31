#ifndef GLW_OBJECT_H
#define GLW_OBJECT_H

#include "./bookkeeping.h"
#include "./noncopyable.h"
#include "./objectdeleter.h"
#include "./type.h"
#include "./glheaders.h"

namespace glw
{

class Context;

class ObjectArguments
{
	public:

		typedef void            BaseType;
		typedef ObjectArguments ThisType;

		ObjectArguments(void)
		{
			;
		}

		void clear(void)
		{
			;
		}
};

class Object : public detail::NonCopyable
{
	friend class Context;

	public:

		typedef detail::NonCopyable BaseType;
		typedef Object              ThisType;

		virtual ~Object(void)
		{
			this->destroy();
		}

		bool isValid(void) const
		{
			return ((this->m_name != 0) && this->doIsValid());
		}

		const Context * context(void) const
		{
			return this->m_context;
		}

		Context * context(void)
		{
			return this->m_context;
		}

		GLuint name(void) const
		{
			return this->m_name;
		}

		virtual Type type(void) const = 0;

	protected:

		GLuint    m_name;
		Context * m_context;

		Object(Context * ctx)
			: m_name    (0)
			, m_context (ctx)
		{
			;
		}

		void destroy(void)
		{
			if (this->m_name == 0) return;
			this->doDestroy();
			this->m_name    = 0;
			this->m_context = 0;
		}

		virtual void doDestroy(void)       = 0;
		virtual bool doIsValid(void) const = 0;
};

namespace detail { template <typename T> struct ObjectBase  { typedef NoType        Type; }; };
namespace detail { template <typename T> struct ObjectSafe  { typedef NoType        Type; }; };
namespace detail { template <typename T> struct ObjectBound { typedef NoType        Type; }; };

namespace detail { template <> struct BaseOf    <Object> { typedef NoType        Type; }; };
namespace detail { template <> struct DeleterOf <Object> { typedef ObjectDeleter Type; }; };
typedef  detail::ObjectSharedPointerTraits      <Object> ::Type ObjectPtr;

class SafeObject : public detail::NonCopyable
{
	friend class Context;
	friend class BoundObject;

	public:

		typedef detail::NonCopyable BaseType;
		typedef SafeObject          ThisType;

		SafeObject(void)
			: m_object(0)
		{
			;
		}

		virtual ~SafeObject(void)
		{
			;
		}

		bool isNull(void) const
		{
			return this->m_object.isNull();
		}

		bool isValid(void) const
		{
			return this->m_object->isValid();
		}

		const Context * context(void) const
		{
			if (this->isNull()) return 0;
			return this->m_object->context();
		}

		Context * context(void)
		{
			if (this->isNull()) return 0;
			return this->m_object->context();
		}

		GLuint name(void) const
		{
			if (this->isNull()) return 0;
			return this->m_object->name();
		}

		Type type(void) const
		{
			if (this->isNull()) return InvalidType;
			return this->m_object->type();
		}

	protected:

		SafeObject(const ObjectPtr & object)
			: m_object(object)
		{
			;
		}

		const ObjectPtr & object(void) const
		{
			return this->m_object;
		}

		ObjectPtr & object(void)
		{
			return this->m_object;
		}

	private:

		ObjectPtr m_object;
};

namespace detail { template <> struct BaseOf     <SafeObject> { typedef NoType                     Type; }; };
namespace detail { template <> struct DeleterOf  <SafeObject> { typedef DefaultDeleter<SafeObject> Type; }; };
namespace detail { template <> struct ObjectBase <SafeObject> { typedef Object                     Type; }; };
namespace detail { template <> struct ObjectSafe <Object    > { typedef SafeObject                 Type; }; };
typedef  detail::ObjectSharedPointerTraits       <SafeObject> ::Type  ObjectHandle;

class ObjectBindingParams
{
	public:

		typedef void                BaseType;
		typedef ObjectBindingParams ThisType;

		GLenum target;
		GLint  unit;

		ObjectBindingParams(void)
			: target (GL_NONE)
			, unit   (0)
		{
			;
		}

		ObjectBindingParams(GLenum aTarget, GLenum aUnit)
			: target (aTarget)
			, unit   (aUnit)
		{
			;
		}
};

class BoundObject : public detail::NonCopyable
{
	friend class Context;

	public:

		typedef detail::NonCopyable   BaseType;
		typedef BoundObject           ThisType;

		BoundObject(void)
			: m_handle (0)
			, m_target (GL_NONE)
			, m_unit   (0)
		{
			;
		}

		virtual ~BoundObject(void)
		{
			;
		}

		bool isNull(void) const
		{
			return this->m_handle.isNull();
		}

		const ObjectHandle & handle(void) const
		{
			return this->m_handle;
		}

		ObjectHandle & handle(void)
		{
			return this->m_handle;
		}

		GLenum target(void) const
		{
			return this->m_target;
		}

		GLint unit(void) const
		{
			return this->m_unit;
		}

	protected:

		ObjectHandle m_handle;
		GLenum       m_target;
		GLint        m_unit;

		BoundObject(const ObjectHandle & handle, const ObjectBindingParams & params)
			: m_handle (handle)
			, m_target (params.target)
			, m_unit   (params.unit)
		{
			;
		}

		const ObjectPtr & object(void) const
		{
			return this->handle()->object();
		}

		ObjectPtr & object(void)
		{
			return this->handle()->object();
		}

		virtual void bind   (void) = 0;
		virtual void unbind (void) = 0;
};

namespace detail { template <typename T> struct ParamsOf { typedef NoType Type; }; };

namespace detail { template <> struct ParamsOf    <BoundObject> { typedef ObjectBindingParams         Type; }; };
namespace detail { template <> struct BaseOf      <BoundObject> { typedef NoType                      Type; }; };
namespace detail { template <> struct DeleterOf   <BoundObject> { typedef DefaultDeleter<BoundObject> Type; }; };
namespace detail { template <> struct ObjectBase  <BoundObject> { typedef Object                      Type; }; };
namespace detail { template <> struct ObjectBound <Object     > { typedef BoundObject                 Type; }; };
typedef   detail::ObjectSharedPointerTraits       <BoundObject> ::Type  BoundObjectHandle;

};

#endif // GLW_OBJECT_H
