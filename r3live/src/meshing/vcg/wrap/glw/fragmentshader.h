#ifndef GLW_FRAGMENTSHADER_H
#define GLW_FRAGMENTSHADER_H

#include "./shader.h"

namespace glw
{

class FragmentShaderArguments : public ShaderArguments
{
	public:

		typedef ShaderArguments       BaseType;
		typedef FragmentShaderArguments ThisType;

		FragmentShaderArguments(void)
			: BaseType()
		{
			;
		}

		void clear(void)
		{
			BaseType::clear();
		}
};

class FragmentShader : public Shader
{
	friend class Context;

	public:

		typedef Shader       BaseType;
		typedef FragmentShader ThisType;

		virtual Type type(void) const
		{
			return FragmentShaderType;
		}

	protected:

		FragmentShader(Context * ctx)
			: BaseType(ctx)
		{
			;
		}

		virtual GLenum shaderType(void) const
		{
			return GL_FRAGMENT_SHADER;
		}

		bool create(const FragmentShaderArguments & args)
		{
			return BaseType::create(args);
		}
};

namespace detail { template <> struct BaseOf <FragmentShader> { typedef Shader Type; }; };
typedef   detail::ObjectSharedPointerTraits  <FragmentShader> ::Type FragmentShaderPtr;

class SafeFragmentShader : public SafeShader
{
	friend class Context;
	friend class BoundFragmentShader;

	public:

		typedef SafeShader       BaseType;
		typedef SafeFragmentShader ThisType;

	protected:

		SafeFragmentShader(const FragmentShaderPtr & fragmentShader)
			: BaseType(fragmentShader)
		{
			;
		}

		const FragmentShaderPtr & object(void) const
		{
			return static_cast<const FragmentShaderPtr &>(BaseType::object());
		}

		FragmentShaderPtr & object(void)
		{
			return static_cast<FragmentShaderPtr &>(BaseType::object());
		}
};

namespace detail { template <> struct BaseOf     <SafeFragmentShader> { typedef SafeShader Type; }; };
namespace detail { template <> struct ObjectBase <SafeFragmentShader> { typedef FragmentShader     Type; }; };
namespace detail { template <> struct ObjectSafe <FragmentShader    > { typedef SafeFragmentShader Type; }; };
typedef   detail::ObjectSharedPointerTraits      <SafeFragmentShader> ::Type FragmentShaderHandle;

class FragmentShaderBindingParams : public ShaderBindingParams
{
	public:

		typedef ShaderBindingParams       BaseType;
		typedef FragmentShaderBindingParams ThisType;

		FragmentShaderBindingParams(void)
			: BaseType(GL_FRAGMENT_SHADER, 0)
		{
			;
		}
};

class BoundFragmentShader : public BoundShader
{
	friend class Context;

	public:

		typedef BoundShader       BaseType;
		typedef BoundFragmentShader ThisType;

		const FragmentShaderHandle & handle(void) const
		{
			return static_cast<const FragmentShaderHandle &>(BaseType::handle());
		}

		FragmentShaderHandle & handle(void)
		{
			return static_cast<FragmentShaderHandle &>(BaseType::handle());
		}

	protected:

		BoundFragmentShader(const FragmentShaderHandle & handle, const ShaderBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}

		const FragmentShaderPtr & object(void) const
		{
			return this->handle()->object();
		}

		FragmentShaderPtr & object(void)
		{
			return this->handle()->object();
		}
};

namespace detail { template <> struct ParamsOf    <BoundFragmentShader> { typedef FragmentShaderBindingParams Type; }; };
namespace detail { template <> struct BaseOf      <BoundFragmentShader> { typedef BoundShader Type; }; };
namespace detail { template <> struct ObjectBase  <BoundFragmentShader> { typedef FragmentShader      Type; }; };
namespace detail { template <> struct ObjectBound <FragmentShader     > { typedef BoundFragmentShader Type; }; };
typedef   detail::ObjectSharedPointerTraits       <BoundFragmentShader> ::Type  BoundFragmentShaderHandle;

};

#endif // GLW_FRAGMENTSHADER_H
