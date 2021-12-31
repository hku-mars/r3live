#ifndef GLW_CONTEXT_H
#define GLW_CONTEXT_H

#include "./noncopyable.h"
#include "./objectdeleter.h"
#include "./buffer.h"
#include "./vertexshader.h"
#include "./geometryshader.h"
#include "./fragmentshader.h"
#include "./program.h"
#include "./renderbuffer.h"
#include "./texture2d.h"
#include "./texturecube.h"
#include "./framebuffer.h"

#include <string>
#include <set>
#include <map>

namespace glw
{

class Context : public detail::NonCopyable
{
	friend class detail::ObjectDeleter;

	public:

		typedef detail::NonCopyable BaseType;
		typedef Context             ThisType;

		Context(void)
			: m_acquired           (false)
			, m_maxUniformBuffers  (0)
			, m_maxFeedbackBuffers (0)
			, m_maxTextureUnits    (0)
		{
			;
		}

		virtual ~Context(void)
		{
			this->release();
		}

		bool acquire(void)
		{
			//
			this->release();
			//
			this->initializeTargets();
			//
			this->m_acquired = true;
			(void)glGetError();
			return this->m_acquired;
		}

		void release(void)
		{
			if (!this->isAcquired()) return;
			this->m_acquired = false;
			this->terminateTargets();
			this->invalidateReferencesToAllObjects();
			(void)glGetError();
		}

		bool isAcquired(void) const
		{
			return this->m_acquired;
		}

		bool isValid(void) const
		{
			return this->isAcquired();
		}

		BufferHandle createBuffer(const BufferArguments & args)
		{
			BufferHandle handle = this->createHandle<Buffer>();
			handle->object()->create(args);
			return handle;
		}

		BoundVertexBufferHandle bindVertexBuffer(BufferHandle & handle)
		{
			return this->bind<BoundVertexBuffer>(handle);
		}

		void unbindVertexBuffer(void)
		{
			BufferHandle nullHandle;
			this->bindVertexBuffer(nullHandle);
		}

		BoundIndexBufferHandle bindIndexBuffer(BufferHandle & handle)
		{
			return this->bind<BoundIndexBuffer>(handle);
		}

		void unbindIndexBuffer(void)
		{
			BufferHandle nullHandle;
			this->bindIndexBuffer(nullHandle);
		}

		BoundPixelPackBufferHandle bindPixelPackBuffer(BufferHandle & handle)
		{
			return this->bind<BoundPixelPackBuffer>(handle);
		}

		void unbindPixelPackBuffer(void)
		{
			BufferHandle nullHandle;
			this->bindPixelPackBuffer(nullHandle);
		}

		BoundPixelUnpackBufferHandle bindPixelUnpackBuffer(BufferHandle & handle)
		{
			return this->bind<BoundPixelUnpackBuffer>(handle);
		}

		void unbindPixelUnpackBuffer(void)
		{
			BufferHandle nullHandle;
			this->bindPixelUnpackBuffer(nullHandle);
		}

		BoundUniformBufferHandle bindUniformBuffer(BufferHandle & handle, GLuint index, GLintptr offset, GLsizeiptr size)
		{
			return this->bind<BoundUniformBuffer>(handle, UniformBufferBindingParams(index, offset, size));
		}

		void unbindUniformBuffer(GLuint index)
		{
			BufferHandle nullHandle;
			this->bindUniformBuffer(nullHandle, index, 0, 0);
		}

		BoundFeedbackBufferHandle bindFeedbackBuffer(BufferHandle & handle, GLuint index, GLintptr offset, GLsizeiptr size)
		{
			return this->bind<BoundFeedbackBuffer>(handle, FeedbackBufferBindingParams(index, offset, size));
		}

		void unbindFeedbackBuffer(GLuint index)
		{
			BufferHandle nullHandle;
			this->bindFeedbackBuffer(nullHandle, index, 0, 0);
		}

		RenderbufferHandle createRenderbuffer(const RenderbufferArguments & args)
		{
			RenderbufferHandle handle = this->createHandle<Renderbuffer>();
			handle->object()->create(args);
			return handle;
		}

		BoundRenderbufferHandle bindRenderbuffer(RenderbufferHandle & handle)
		{
			return this->bind<BoundRenderbuffer>(handle);
		}

		void unbindRenderbuffer(void)
		{
			RenderbufferHandle nullHandle;
			this->bindRenderbuffer(nullHandle);
		}

		VertexShaderHandle createVertexShader(const VertexShaderArguments & args)
		{
			VertexShaderHandle handle = this->createHandle<VertexShader>();
			handle->object()->create(args);
			return handle;
		}

		BoundVertexShaderHandle bindVertexShader(VertexShaderHandle & handle)
		{
			return this->bind<BoundVertexShader>(handle, VertexShaderBindingParams());
		}

		void unbindVertexShader(void)
		{
			VertexShaderHandle nullHandle;
			this->bindVertexShader(nullHandle);
		}

		GeometryShaderHandle createGeometryShader(const GeometryShaderArguments & args)
		{
			GeometryShaderHandle handle = this->createHandle<GeometryShader>();
			handle->object()->create(args);
			return handle;
		}

		BoundGeometryShaderHandle bindGeometryShader(GeometryShaderHandle & handle)
		{
			return this->bind<BoundGeometryShader>(handle, GeometryShaderBindingParams());
		}

		void unbindGeometryShader(void)
		{
			GeometryShaderHandle nullHandle;
			this->bindGeometryShader(nullHandle);
		}

		FragmentShaderHandle createFragmentShader(const FragmentShaderArguments & args)
		{
			FragmentShaderHandle handle = this->createHandle<FragmentShader>();
			handle->object()->create(args);
			return handle;
		}

		BoundFragmentShaderHandle bindFragmentShader(FragmentShaderHandle & handle)
		{
			return this->bind<BoundFragmentShader>(handle, FragmentShaderBindingParams());
		}

		void unbindFragmentShader(void)
		{
			FragmentShaderHandle nullHandle;
			this->bindFragmentShader(nullHandle);
		}

		ProgramHandle createProgram(const ProgramArguments & args)
		{
			ProgramHandle handle = this->createHandle<Program>();
			handle->object()->create(args);
			return handle;
		}

		BoundProgramHandle bindProgram(ProgramHandle & handle)
		{
			return this->bind<BoundProgram>(handle, ProgramBindingParams());
		}

		void unbindProgram(void)
		{
			ProgramHandle nullHandle;
			this->bindProgram(nullHandle);
		}

		Texture2DHandle createTexture2D(const Texture2DArguments & args)
		{
			Texture2DHandle handle = this->createHandle<Texture2D>();
			handle->object()->create(args);
			return handle;
		}

		BoundTexture2DHandle bindTexture2D(Texture2DHandle & handle, GLint unit)
		{
			return this->bind<BoundTexture2D>(handle, Texture2DBindingParams(unit));
		}

		void unbindTexture2D(GLint unit)
		{
			Texture2DHandle nullHandle;
			this->bindTexture2D(nullHandle, unit);
		}

		TextureCubeHandle createTextureCube(const TextureCubeArguments & args)
		{
			TextureCubeHandle handle = this->createHandle<TextureCube>();
			handle->object()->create(args);
			return handle;
		}

		BoundTextureCubeHandle bindTextureCube(TextureCubeHandle & handle, GLint unit)
		{
			return this->bind<BoundTextureCube>(handle, TextureCubeBindingParams(unit));
		}

		void unbindTextureCube(GLint unit)
		{
			TextureCubeHandle nullHandle;
			this->bindTextureCube(nullHandle, unit);
		}

		FramebufferHandle createFramebuffer(const FramebufferArguments & args)
		{
			FramebufferHandle handle = this->createHandle<Framebuffer>();
			handle->object()->create(args);
			return handle;
		}

		BoundReadFramebufferHandle bindReadFramebuffer(FramebufferHandle & handle)
		{
			FramebufferHandle nullHandle;
			this->bind<BoundReadDrawFramebuffer>(nullHandle, ReadDrawFramebufferBindingParams());
			return this->bind<BoundReadFramebuffer>(handle, ReadFramebufferBindingParams());
		}

		void unbindReadFramebuffer(void)
		{
			FramebufferHandle nullHandle;
			this->bindReadFramebuffer(nullHandle);
		}

		BoundDrawFramebufferHandle bindDrawFramebuffer(FramebufferHandle & handle)
		{
			FramebufferHandle nullHandle;
			this->bind<BoundReadDrawFramebuffer>(nullHandle, ReadDrawFramebufferBindingParams());
			return this->bind<BoundDrawFramebuffer>(handle, DrawFramebufferBindingParams());
		}

		void unbindDrawFramebuffer(void)
		{
			FramebufferHandle nullHandle;
			this->bindDrawFramebuffer(nullHandle);
		}

		BoundReadDrawFramebufferHandle bindReadDrawFramebuffer(FramebufferHandle & handle)
		{
			FramebufferHandle nullHandle;
			this->bind<BoundReadFramebuffer>(nullHandle, ReadFramebufferBindingParams());
			this->bind<BoundDrawFramebuffer>(nullHandle, DrawFramebufferBindingParams());
			return this->bind<BoundReadDrawFramebuffer>(handle, ReadDrawFramebufferBindingParams());
		}

		void unbindReadDrawFramebuffer(void)
		{
			FramebufferHandle nullHandle;
			this->bindReadDrawFramebuffer(nullHandle);
		}

	private:

		template <typename TObject>
		struct ObjectSafeFromObject
		{
			typedef typename detail::ObjectSafe<TObject>::Type Type;
		};

		template <typename TObject>
		struct ObjectBoundFromObject
		{
			typedef typename detail::ObjectBound<TObject>::Type Type;
		};

		template <typename TBinding>
		struct ObjectFromBinding
		{
			typedef typename detail::ObjectBase<TBinding>::Type Type;
		};

		template <typename TObject>
		struct RefCountedPtrFromObject
		{
			typedef detail::RefCountedObject<TObject, typename detail::DeleterOf<typename detail::RootOf<TObject>::Type>::Type, typename detail::BaseOf<TObject>::Type> Type;
		};

		template <typename TObject>
		struct RefCountedSafeHandleFromObject
		{
			typedef typename ObjectSafeFromObject<TObject>::Type ObjectSafeType;
			typedef detail::RefCountedObject<ObjectSafeType, typename detail::DeleterOf<typename detail::RootOf<ObjectSafeType>::Type>::Type, typename detail::BaseOf<ObjectSafeType>::Type> Type;
		};

		template <typename TObject>
		struct RefCountedBindingHandleFromObject
		{
			typedef typename ObjectBoundFromObject<TObject>::Type ObjectBoundType;
			typedef detail::RefCountedObject<ObjectBoundType, typename detail::DeleterOf<typename detail::RootOf<ObjectBoundType>::Type>::Type, typename detail::BaseOf<ObjectBoundType>::Type> Type;
		};

		template <typename TBinding>
		struct RefCountedBindingHandleFromBinding
		{
			typedef detail::RefCountedObject<TBinding, typename detail::DeleterOf<typename detail::RootOf<TBinding>::Type>::Type, typename detail::BaseOf<TBinding>::Type> Type;
		};

		template <typename TObject>
		struct PtrFromObject
		{
			typedef detail::ObjectSharedPointer<TObject, typename detail::DeleterOf<typename detail::RootOf<TObject>::Type>::Type, typename detail::BaseOf<TObject>::Type> Type;
		};

		template <typename TObject>
		struct SafeHandleFromObject
		{
			typedef typename ObjectSafeFromObject<TObject>::Type ObjectSafeType;
			typedef detail::ObjectSharedPointer<ObjectSafeType, typename detail::DeleterOf<typename detail::RootOf<ObjectSafeType>::Type>::Type, typename detail::BaseOf<ObjectSafeType>::Type> Type;
		};

		template <typename TObject>
		struct BindingHandleFromObject
		{
			typedef typename ObjectBoundFromObject<TObject>::Type ObjectBoundType;
			typedef detail::ObjectSharedPointer<ObjectBoundType, typename detail::DeleterOf<typename detail::RootOf<ObjectBoundType>::Type>::Type, typename detail::BaseOf<ObjectBoundType>::Type> Type;
		};

		template <typename TBinding>
		struct SafeHandleFromBinding
		{
			typedef typename SafeHandleFromObject<typename ObjectFromBinding<TBinding>::Type>::Type Type;
		};

		template <typename TBinding>
		struct BindingHandleFromBinding
		{
			typedef detail::ObjectSharedPointer<TBinding, typename detail::DeleterOf<typename detail::RootOf<TBinding>::Type>::Type, typename detail::BaseOf<TBinding>::Type> Type;
		};

		typedef Object                                                 ObjectType;
		typedef RefCountedPtrFromObject<ObjectType>::Type              RefCountedPtrType;
		typedef std::map<Object *, RefCountedPtrType *>                RefCountedPtrPtrMap;
		typedef RefCountedPtrPtrMap::const_iterator                    RefCountedPtrPtrMapConstIterator;
		typedef RefCountedPtrPtrMap::iterator                          RefCountedPtrPtrMapIterator;
		typedef RefCountedPtrPtrMap::value_type                        RefCountedPtrPtrMapValue;

		typedef std::pair<GLenum, GLint>                               BindingTarget;
		typedef BoundObjectHandle::RefCountedObjectType                RefCountedBindingType;
		typedef std::map<BindingTarget, RefCountedBindingType *>       RefCountedBindingPtrMap;
		typedef RefCountedBindingPtrMap::const_iterator                RefCountedBindingPtrMapConstIterator;
		typedef RefCountedBindingPtrMap::iterator                      RefCountedBindingPtrMapIterator;
		typedef RefCountedBindingPtrMap::value_type                    RefCountedBindingPtrMapValue;

		bool                    m_acquired;
		int                     m_maxUniformBuffers;
		int                     m_maxFeedbackBuffers;
		int                     m_maxTextureUnits;
		RefCountedPtrPtrMap     m_objects;
		RefCountedBindingPtrMap m_bindings;

		template <typename TBinding, typename TBindingParams>
		void initializeTarget(const TBindingParams & params)
		{
			typedef TBinding                                                       BindingType;
			typedef typename RefCountedBindingHandleFromBinding<BindingType>::Type RefCountedBindingHandleType;

			const BindingTarget bt = BindingTarget(params.target, params.unit);
			RefCountedBindingHandleType * binding = 0;
			this->m_bindings.insert(RefCountedBindingPtrMapValue(bt, binding));
		}

		template <typename TBinding, typename TBindingParams>
		void terminateTarget(const TBindingParams & params)
		{
			typedef TBinding                                          BindingType;
			typedef typename SafeHandleFromBinding<BindingType>::Type SafeHandleType;

			SafeHandleType nullHandle;
			this->bind<BindingType>(nullHandle, params);
		}

		void initializeTargets(void)
		{	
			this->initializeTarget<BoundVertexBuffer,        VertexBufferBindingParams        >(VertexBufferBindingParams        ()       );
			this->initializeTarget<BoundIndexBuffer,         IndexBufferBindingParams         >(IndexBufferBindingParams         ()       );			
			this->initializeTarget<BoundPixelPackBuffer,     PixelPackBufferBindingParams     >(PixelPackBufferBindingParams     ()       );			
			this->initializeTarget<BoundPixelUnpackBuffer,   PixelUnpackBufferBindingParams   >(PixelUnpackBufferBindingParams   ()       );			
			this->initializeTarget<BoundRenderbuffer,        RenderbufferBindingParams        >(RenderbufferBindingParams        ()       );			
			this->initializeTarget<BoundVertexShader,        VertexShaderBindingParams        >(VertexShaderBindingParams        ()       );			
			this->initializeTarget<BoundGeometryShader,      GeometryShaderBindingParams      >(GeometryShaderBindingParams      ()       );			
			this->initializeTarget<BoundFragmentShader,      FragmentShaderBindingParams      >(FragmentShaderBindingParams      ()       );			
			this->initializeTarget<BoundProgram,             ProgramBindingParams             >(ProgramBindingParams             ()       );			
			this->initializeTarget<BoundReadFramebuffer,     ReadFramebufferBindingParams     >(ReadFramebufferBindingParams     ()       );			
			this->initializeTarget<BoundDrawFramebuffer,     DrawFramebufferBindingParams     >(DrawFramebufferBindingParams     ()       );			
			this->initializeTarget<BoundReadDrawFramebuffer, ReadDrawFramebufferBindingParams >(ReadDrawFramebufferBindingParams ()       );

			{
				this->m_maxUniformBuffers = 0;
				if (GLEW_ARB_uniform_buffer_object)
				{
					GLint uniformBuffers = 0;
					glGetIntegerv(GL_MAX_UNIFORM_BUFFER_BINDINGS, &uniformBuffers);
					this->m_maxUniformBuffers = int(uniformBuffers);
					for (int i=0; i<this->m_maxUniformBuffers; ++i)
					{
						
						this->initializeTarget<BoundUniformBuffer, UniformBufferBindingParams>(UniformBufferBindingParams(GLuint(i), 0, 0));
						
					}
				}
			}

			{
				this->m_maxFeedbackBuffers = 0;
				if (GLEW_EXT_transform_feedback)
				{
					GLint feedbackBuffers = 0;
					glGetIntegerv(GL_MAX_TRANSFORM_FEEDBACK_SEPARATE_ATTRIBS, &feedbackBuffers);
					this->m_maxFeedbackBuffers = int(feedbackBuffers);
					for (int i=0; i<this->m_maxFeedbackBuffers; ++i)
					{
						this->initializeTarget<BoundFeedbackBuffer, FeedbackBufferBindingParams>(FeedbackBufferBindingParams(GLuint(i), 0, 0));
					}
				}
			}

			{
				GLint texUnits = 0;
				glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &texUnits);
				this->m_maxTextureUnits = int(texUnits);
				for (int i=0; i<this->m_maxTextureUnits; ++i)
				{
					this->initializeTarget<BoundTexture2D  >(Texture2DBindingParams   (GLint(i)));
					this->initializeTarget<BoundTextureCube>(TextureCubeBindingParams (GLint(i)));
				}
			}
		}

		void terminateTargets(void)
		{
			this->terminateTarget<BoundVertexBuffer,        VertexBufferBindingParams        >(VertexBufferBindingParams        ()       );
			this->terminateTarget<BoundIndexBuffer,         IndexBufferBindingParams         >(IndexBufferBindingParams         ()       );
			this->terminateTarget<BoundPixelPackBuffer,     PixelPackBufferBindingParams     >(PixelPackBufferBindingParams     ()       );
			this->terminateTarget<BoundPixelUnpackBuffer,   PixelUnpackBufferBindingParams   >(PixelUnpackBufferBindingParams   ()       );
			this->terminateTarget<BoundRenderbuffer,        RenderbufferBindingParams        >(RenderbufferBindingParams        ()       );
			this->terminateTarget<BoundVertexShader,        VertexShaderBindingParams        >(VertexShaderBindingParams        ()       );
			this->terminateTarget<BoundGeometryShader,      GeometryShaderBindingParams      >(GeometryShaderBindingParams      ()       );
			this->terminateTarget<BoundFragmentShader,      FragmentShaderBindingParams      >(FragmentShaderBindingParams      ()       );
			this->terminateTarget<BoundProgram,             ProgramBindingParams             >(ProgramBindingParams             ()       );
			this->terminateTarget<BoundReadFramebuffer,     ReadFramebufferBindingParams     >(ReadFramebufferBindingParams     ()       );
			this->terminateTarget<BoundDrawFramebuffer,     DrawFramebufferBindingParams     >(DrawFramebufferBindingParams     ()       );
			this->terminateTarget<BoundReadDrawFramebuffer, ReadDrawFramebufferBindingParams >(ReadDrawFramebufferBindingParams ()       );

			{
				for (int i=0; i<this->m_maxUniformBuffers; ++i)
				{
					this->terminateTarget<BoundUniformBuffer, UniformBufferBindingParams>(UniformBufferBindingParams(GLuint(i), 0, 0));
				}
				this->m_maxUniformBuffers = 0;
			}

			{
				for (int i=0; i<this->m_maxFeedbackBuffers; ++i)
				{
					this->terminateTarget<BoundFeedbackBuffer, FeedbackBufferBindingParams>(FeedbackBufferBindingParams(GLuint(i), 0, 0));
				}
				this->m_maxFeedbackBuffers = 0;
			}

			{
				for (int i=0; i<this->m_maxTextureUnits; ++i)
				{
					this->terminateTarget<BoundTexture2D  >(Texture2DBindingParams   (GLint(i)));
					this->terminateTarget<BoundTextureCube>(TextureCubeBindingParams (GLint(i)));
				}
				this->m_maxTextureUnits = 0;
			}
		}

		template <typename TObject>
		TObject * createObject(void)
		{
			typedef TObject ObjectType;
			ObjectType * object = new ObjectType(this);
			return object;
		}

		void destroyObject(Object * object)
		{
			GLW_ASSERT(object != 0);
			object->destroy();
			delete object;
		}

		template <typename TObject>
		typename SafeHandleFromObject<TObject>::Type createHandle(void)
		{
			typedef TObject                                                   ObjectType;
			typedef typename RefCountedPtrFromObject<ObjectType>::Type        RefCountedPtrType;
			typedef typename PtrFromObject<ObjectType>::Type                  PtrType;

			typedef typename ObjectSafeFromObject<ObjectType>::Type           ObjectSafeType;
			typedef typename RefCountedSafeHandleFromObject<ObjectType>::Type RefCountedSafeHandleType;
			typedef typename SafeHandleFromObject<TObject>::Type              SafeHandleType;

			ObjectType *               object            = this->createObject<ObjectType>();
			RefCountedPtrType *        refCountedPtr     = new RefCountedPtrType(object, typename detail::DeleterOf<Object>::Type());
			PtrType                    ptr               = PtrType(refCountedPtr);

			ObjectSafeType *           objecSafe         = new ObjectSafeType(ptr);
			RefCountedSafeHandleType * refCountedHandle  = new RefCountedSafeHandleType(objecSafe, typename detail::DeleterOf<ObjectSafeType>::Type());
			SafeHandleType             handle            = SafeHandleType(refCountedHandle);

			this->m_objects.insert(RefCountedPtrPtrMapValue(object, refCountedPtr));

			return handle;
		}

		void noMoreReferencesTo(Object * object)
		{
			GLW_ASSERT(object != 0);
			RefCountedPtrPtrMapIterator it = this->m_objects.find(object);
			GLW_ASSERT(it != this->m_objects.end());
			this->m_objects.erase(it);
			this->destroyObject(object);
		}

		void invalidateReferencesToAllObjects(void)
		{
			for (RefCountedPtrPtrMapIterator it=this->m_objects.begin(); it!=this->m_objects.end(); ++it)
			{
				Object *            object = it->first;
				RefCountedPtrType * refPtr = it->second;
				refPtr->setNull(false);
				this->destroyObject(object);
			}
		}

		template <typename TBinding>
		typename BindingHandleFromBinding<TBinding>::Type bind(typename SafeHandleFromBinding<TBinding>::Type & h, const typename detail::ParamsOf<TBinding>::Type & params = typename detail::ParamsOf<TBinding>::Type())
		{
			typedef TBinding                                                       BindingType;
			typedef typename detail::ParamsOf<TBinding>::Type                      BindingParamsType;
			typedef typename BindingHandleFromBinding<BindingType>::Type           BindingHandleType;
			typedef typename RefCountedBindingHandleFromBinding<BindingType>::Type RefCountedBindingHandleType;

			const BindingTarget bt = BindingTarget(params.target, params.unit);

			RefCountedBindingPtrMapIterator it = this->m_bindings.find(bt);
			GLW_ASSERT(it != this->m_bindings.end());

			RefCountedBindingHandleType * currentBinding = static_cast<RefCountedBindingHandleType *>(it->second);
			if (currentBinding != 0)
			{
				GLW_ASSERT(!currentBinding->isNull());
				// WARNING: as state could have been changed outside GLW, uncommenting the following line may prevent correct binding.
				//if (currentBinding->object() == Object) return UnsafeObjectType(currentBinding);
				if (h.isNull()) currentBinding->object()->unbind();
				currentBinding->setNull(true);
				currentBinding->unref();
				currentBinding = 0;
				it->second = 0;
			}

			if (h.isNull()) return BindingHandleType();

			BindingType *                 binding    = new BindingType(h, params);
			RefCountedBindingHandleType * newBinding = new RefCountedBindingHandleType(binding, typename detail::DeleterOf<BindingType>::Type());
			newBinding->ref();
			newBinding->object()->bind();
			it->second = newBinding;

			return BindingHandleType(newBinding);
		}
};

namespace detail
{

inline void ObjectDeleter :: operator () (Object * object) const
{
	if (object == 0) return;
	object->context()->noMoreReferencesTo(object);
}

};

};

#endif // GLW_CONTEXT_H
