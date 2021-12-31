#ifndef GLW_GEOMETRYSHADER_H
#define GLW_GEOMETRYSHADER_H

#include "./shader.h"

namespace glw
{

class GeometryShaderArguments : public ShaderArguments
{
	public:

		typedef ShaderArguments       BaseType;
		typedef GeometryShaderArguments ThisType;

		GeometryShaderArguments(void)
			: BaseType()
		{
			;
		}

		void clear(void)
		{
			BaseType::clear();
		}
};

class GeometryShader : public Shader
{
	friend class Context;

	public:

		typedef Shader       BaseType;
		typedef GeometryShader ThisType;

		virtual Type type(void) const
		{
			return GeometryShaderType;
		}

	protected:

		GeometryShader(Context * ctx)
			: BaseType(ctx)
		{
			;
		}

		virtual GLenum shaderType(void) const
		{
			return GL_GEOMETRY_SHADER;
		}

		bool create(const GeometryShaderArguments & args)
		{
			return BaseType::create(args);
		}
};

namespace detail { template <> struct BaseOf <GeometryShader> { typedef Shader Type; }; };
typedef   detail::ObjectSharedPointerTraits  <GeometryShader> ::Type GeometryShaderPtr;

class SafeGeometryShader : public SafeShader
{
	friend class Context;
	friend class BoundGeometryShader;

	public:

		typedef SafeShader       BaseType;
		typedef SafeGeometryShader ThisType;

	protected:

		SafeGeometryShader(const GeometryShaderPtr & geometryShader)
			: BaseType(geometryShader)
		{
			;
		}

		const GeometryShaderPtr & object(void) const
		{
			return static_cast<const GeometryShaderPtr &>(BaseType::object());
		}

		GeometryShaderPtr & object(void)
		{
			return static_cast<GeometryShaderPtr &>(BaseType::object());
		}
};

namespace detail { template <> struct BaseOf     <SafeGeometryShader> { typedef SafeShader Type; }; };
namespace detail { template <> struct ObjectBase <SafeGeometryShader> { typedef GeometryShader     Type; }; };
namespace detail { template <> struct ObjectSafe <GeometryShader    > { typedef SafeGeometryShader Type; }; };
typedef   detail::ObjectSharedPointerTraits      <SafeGeometryShader> ::Type GeometryShaderHandle;

class GeometryShaderBindingParams : public ShaderBindingParams
{
	public:

		typedef ShaderBindingParams       BaseType;
		typedef GeometryShaderBindingParams ThisType;

		GeometryShaderBindingParams(void)
			: BaseType(GL_GEOMETRY_SHADER, 0)
		{
			;
		}
};

class BoundGeometryShader : public BoundShader
{
	friend class Context;

	public:

		typedef BoundShader       BaseType;
		typedef BoundGeometryShader ThisType;

		const GeometryShaderHandle & handle(void) const
		{
			return static_cast<const GeometryShaderHandle &>(BaseType::handle());
		}

		GeometryShaderHandle & handle(void)
		{
			return static_cast<GeometryShaderHandle &>(BaseType::handle());
		}

	protected:

		BoundGeometryShader(const GeometryShaderHandle & handle, const ShaderBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}

		const GeometryShaderPtr & object(void) const
		{
			return this->handle()->object();
		}

		GeometryShaderPtr & object(void)
		{
			return this->handle()->object();
		}
};

namespace detail { template <> struct ParamsOf    <BoundGeometryShader> { typedef GeometryShaderBindingParams Type; }; };
namespace detail { template <> struct BaseOf      <BoundGeometryShader> { typedef BoundShader Type; }; };
namespace detail { template <> struct ObjectBase  <BoundGeometryShader> { typedef GeometryShader      Type; }; };
namespace detail { template <> struct ObjectBound <GeometryShader     > { typedef BoundGeometryShader Type; }; };
typedef   detail::ObjectSharedPointerTraits       <BoundGeometryShader> ::Type  BoundGeometryShaderHandle;

};

#endif // GLW_GEOMETRYSHADER_H
