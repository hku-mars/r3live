#ifndef GLW_BOOKKEEPING_H
#define GLW_BOOKKEEPING_H

#include "./common.h"

namespace glw
{

namespace detail
{

                                  struct NoType                    {                                                                         };
template <typename T>             struct DefaultDeleter            { void operator () (T * t) { delete t; }                                  };

template <typename T>             struct BaseOf                    { typedef NoType Type;                                                    };

template <typename T, typename B> struct RootOfType                { typedef typename RootOfType<B,  typename BaseOf<B>::Type>::Type  Type;  };
template <typename T>             struct RootOfType<T, NoType>     { typedef T                                                        Type;  };
template <typename T>             struct RootOf                    { typedef typename RootOfType<T, typename BaseOf<T>::Type>::Type   Type;  };

template <typename T, typename B> struct DeleterOfType             { typedef typename DeleterOfType<B, typename BaseOf<B>::Type>::Type Type; };
template <typename T>             struct DeleterOfType<T, NoType>  { typedef DefaultDeleter<T>                                         Type; };
template <typename T>             struct DeleterOf                 { typedef typename DeleterOfType<T, typename BaseOf<T>::Type>::Type Type; };

template <typename TObject, typename TDeleter, typename TBaseObject>
class RefCountedObject : public RefCountedObject<TBaseObject, TDeleter, typename BaseOf<TBaseObject>::Type>
{
	public:

		typedef RefCountedObject<TBaseObject, TDeleter, typename BaseOf<TBaseObject>::Type> BaseType;
		typedef RefCountedObject<TObject, TDeleter, TBaseObject>                            ThisType;
		typedef TObject                                                                     ObjectType;
		typedef TDeleter                                                                    DeleterType;
		typedef TBaseObject                                                                 BaseObjectType;

		RefCountedObject(ObjectType * object, const DeleterType & deleter)
			: BaseType(object, deleter)
		{
			;
		}

		const ObjectType * object(void) const
		{
			return static_cast<const ObjectType *>(BaseType::object());
		}

		ObjectType * object(void)
		{
			return static_cast<ObjectType *>(BaseType::object());
		}
};

template <typename TObject, typename TDeleter>
class RefCountedObject<TObject, TDeleter, NoType>
{
	public:

		typedef void                                        BaseType;
		typedef RefCountedObject<TObject, TDeleter, NoType> ThisType;
		typedef TObject                                     ObjectType;
		typedef TDeleter                                    DeleterType;
		typedef NoType                                      BaseObjectType;

		RefCountedObject(ObjectType * object, const DeleterType & deleter)
			: m_object   (object)
			, m_refCount (0)
			, m_deleter  (deleter)
		{
			GLW_ASSERT(this->m_object  != 0);
		}

		~RefCountedObject(void)
		{
			this->destroyObject();
		}

		bool isNull(void) const
		{
			return (this->m_object == 0);
		}

		void setNull(bool deleteObject)
		{
			if (deleteObject)
			{
				this->destroyObject();
			}
			this->m_object = 0;
		}

		const ObjectType * object(void) const
		{
			return this->m_object;
		}

		ObjectType * object(void)
		{
			return this->m_object;
		}

		const DeleterType & deleter(void) const
		{
			return this->m_deleter;
		}

		DeleterType & deleter(void)
		{
			return this->m_deleter;
		}

		void ref(void)
		{
			this->m_refCount++;
		}

		void unref(void)
		{
			GLW_ASSERT(this->m_refCount > 0);
			this->m_refCount--;
			if (this->m_refCount == 0)
			{
				delete this;
			}
		}

		int refCount(void) const
		{
			return this->m_refCount;
		}

	private:

		ObjectType * m_object;
		int          m_refCount;
		DeleterType  m_deleter;

		RefCountedObject(const ThisType & other);
		ThisType & operator = (const ThisType & other);

		void destroyObject(void)
		{
			if (this->m_object == 0) return;
			this->m_deleter(this->m_object);
			this->m_object = 0;
		}
};

template <typename T> struct RefCountedObjectTraits { typedef RefCountedObject<T, typename DeleterOf<T>::Type, typename BaseOf<T>::Type> Type; };

template <typename TObject, typename TDeleter, typename TBaseObject>
class ObjectSharedPointer : public ObjectSharedPointer<TBaseObject, TDeleter, typename BaseOf<TBaseObject>::Type>
{
	public:

		typedef ObjectSharedPointer<TBaseObject, TDeleter, typename BaseOf<TBaseObject>::Type>  BaseType;
		typedef ObjectSharedPointer<TObject, TDeleter, TBaseObject>                             ThisType;
		typedef TObject                                                                         ObjectType;
		typedef TDeleter                                                                        DeleterType;
		typedef TBaseObject                                                                     BaseObjectType;
		typedef RefCountedObject<ObjectType, DeleterType, BaseObjectType>                       RefCountedObjectType;

		ObjectSharedPointer(void)
			: BaseType()
		{
			;
		}

		ObjectSharedPointer(const ThisType & other)
			: BaseType(other)
		{
			;
		}

		ObjectSharedPointer(RefCountedObjectType * refObject)
			: BaseType(refObject)
		{
			;
		}

		const ObjectType & operator * (void) const
		{
			return (*(this->object()));
		}

		ObjectType & operator * (void)
		{
			return (*(this->object()));
		}

		const ObjectType * operator -> (void) const
		{
			return this->object();
		}

		ObjectType * operator -> (void)
		{
			return this->object();
		}

	protected:

		const ObjectType * object(void) const
		{
			return static_cast<const ObjectType *>(BaseType::object());
		}

		ObjectType * object(void)
		{
			return static_cast<ObjectType *>(BaseType::object());
		}

		RefCountedObjectType * refObject(void) const
		{
			return static_cast<RefCountedObjectType *>(BaseType::refObject());
		}
};

template <typename TObject, typename TDeleter>
class ObjectSharedPointer<TObject, TDeleter, NoType>
{
	public:

		typedef void                                              BaseType;
		typedef ObjectSharedPointer<TObject, TDeleter, NoType>    ThisType;
		typedef TObject                                           ObjectType;
		typedef TDeleter                                          DeleterType;
		typedef NoType                                            BaseObjectType;
		typedef RefCountedObject<ObjectType, DeleterType, NoType> RefCountedObjectType;

		ObjectSharedPointer(void)
			: m_refObject(0)
		{
			;
		}

		ObjectSharedPointer(const ThisType & other)
			: m_refObject(0)
		{
			this->attach(other.refObject());
		}

		ObjectSharedPointer(RefCountedObjectType * refObject)
			: m_refObject(0)
		{
			this->attach(refObject);
		}

		~ObjectSharedPointer(void)
		{
			this->detach();
		}

		bool isNull(void) const
		{
			if (this->m_refObject == 0) return true;
			return this->m_refObject->isNull();
		}

		void setNull(void)
		{
			this->detach();
		}

		const ObjectType & operator * (void) const
		{
			return (*(this->object()));
		}

		ObjectType & operator * (void)
		{
			return (*(this->object()));
		}

		const ObjectType * operator -> (void) const
		{
			return this->object();
		}

		ObjectType * operator -> (void)
		{
			return this->object();
		}

		operator bool (void) const
		{
			return (!this->isNull());
		}

		ThisType & operator = (const ThisType & other)
		{
			this->attach(other.refObject());
			return (*this);
		}

	protected:

		const ObjectType * object(void) const
		{
			GLW_ASSERT(!this->isNull());
			return this->m_refObject->object();
		}

		ObjectType * object(void)
		{
			GLW_ASSERT(!this->isNull());
			return this->m_refObject->object();
		}

		RefCountedObjectType * refObject(void) const
		{
			return this->m_refObject;
		}

	private:

		RefCountedObjectType * m_refObject;

		void attach(RefCountedObjectType * reObject)
		{
			this->detach();
			this->m_refObject = reObject;
			if (this->m_refObject != 0)
			{
				this->m_refObject->ref();
			}
		}

		void detach(void)
		{
			if (this->m_refObject == 0) return;
			this->m_refObject->unref();
			this->m_refObject = 0;
		}
};

template <typename T> struct ObjectSharedPointerTraits { typedef ObjectSharedPointer<T, typename DeleterOf<typename RootOf<T>::Type>::Type, typename BaseOf<T>::Type> Type; };

};

};

#endif // GLW_BOOKKEEPING_H
