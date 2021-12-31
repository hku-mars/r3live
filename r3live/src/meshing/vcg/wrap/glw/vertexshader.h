#ifndef GLW_VERTEXSHADER_H
#define GLW_VERTEXSHADER_H

#include "./shader.h"

namespace glw
{

class VertexShaderArguments : public ShaderArguments
{
	public:

		typedef ShaderArguments       BaseType;
		typedef VertexShaderArguments ThisType;

		VertexShaderArguments(void)
			: BaseType()
		{
			;
		}

		void clear(void)
		{
			BaseType::clear();
		}
};

class VertexShader : public Shader
{
	friend class Context;

	public:

		typedef Shader       BaseType;
		typedef VertexShader ThisType;

		virtual Type type(void) const
		{
			return VertexShaderType;
		}

	protected:

		VertexShader(Context * ctx)
			: BaseType(ctx)
		{
			;
		}

		virtual GLenum shaderType(void) const
		{
			return GL_VERTEX_SHADER;
		}

		bool create(const VertexShaderArguments & args)
		{
			return BaseType::create(args);
		}
};

namespace detail { template <> struct BaseOf <VertexShader> { typedef Shader Type; }; };
typedef   detail::ObjectSharedPointerTraits  <VertexShader> ::Type VertexShaderPtr;

class SafeVertexShader : public SafeShader
{
	friend class Context;
	friend class BoundVertexShader;

	public:

		typedef SafeShader       BaseType;
		typedef SafeVertexShader ThisType;

	protected:

		SafeVertexShader(const VertexShaderPtr & vertexShader)
			: BaseType(vertexShader)
		{
			;
		}

		const VertexShaderPtr & object(void) const
		{
			return static_cast<const VertexShaderPtr &>(BaseType::object());
		}

		VertexShaderPtr & object(void)
		{
			return static_cast<VertexShaderPtr &>(BaseType::object());
		}
};

namespace detail { template <> struct BaseOf     <SafeVertexShader> { typedef SafeShader Type; }; };
namespace detail { template <> struct ObjectBase <SafeVertexShader> { typedef VertexShader     Type; }; };
namespace detail { template <> struct ObjectSafe <VertexShader    > { typedef SafeVertexShader Type; }; };
typedef   detail::ObjectSharedPointerTraits      <SafeVertexShader> ::Type VertexShaderHandle;

class VertexShaderBindingParams : public ShaderBindingParams
{
	public:

		typedef ShaderBindingParams       BaseType;
		typedef VertexShaderBindingParams ThisType;

		VertexShaderBindingParams(void)
			: BaseType(GL_VERTEX_SHADER, 0)
		{
			;
		}
};

class BoundVertexShader : public BoundShader
{
	friend class Context;

	public:

		typedef BoundShader       BaseType;
		typedef BoundVertexShader ThisType;

		const VertexShaderHandle & handle(void) const
		{
			return static_cast<const VertexShaderHandle &>(BaseType::handle());
		}

		VertexShaderHandle & handle(void)
		{
			return static_cast<VertexShaderHandle &>(BaseType::handle());
		}

	protected:

		BoundVertexShader(const VertexShaderHandle & handle, const ShaderBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}

		const VertexShaderPtr & object(void) const
		{
			return this->handle()->object();
		}

		VertexShaderPtr & object(void)
		{
			return this->handle()->object();
		}
};

namespace detail { template <> struct ParamsOf    <BoundVertexShader> { typedef VertexShaderBindingParams Type; }; };
namespace detail { template <> struct BaseOf      <BoundVertexShader> { typedef BoundShader Type; }; };
namespace detail { template <> struct ObjectBase  <BoundVertexShader> { typedef VertexShader      Type; }; };
namespace detail { template <> struct ObjectBound <VertexShader     > { typedef BoundVertexShader Type; }; };
typedef   detail::ObjectSharedPointerTraits       <BoundVertexShader> ::Type  BoundVertexShaderHandle;

};

#endif // GLW_VERTEXSHADER_H
