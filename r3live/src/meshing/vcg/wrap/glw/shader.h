#ifndef GLW_SHADER_H
#define GLW_SHADER_H

#include <string>
#include <iostream>

#include "./object.h"

namespace glw
{

class ShaderArguments : public ObjectArguments
{
	public:

		typedef ObjectArguments BaseType;
		typedef ShaderArguments ThisType;

		std::string source;

		ShaderArguments(void)
			: BaseType()
		{
			;
		}

		void clear(void)
		{
			BaseType::clear();
			this->source.clear();
		}
};

class Shader : public Object
{
	friend class Context;

	public:

		typedef Object BaseType;
		typedef Shader ThisType;

		virtual ~Shader(void)
		{
			this->destroy();
		}

		const std::string & source(void) const
		{
			return this->m_source;
		}

		const std::string & log(void) const
		{
			return this->m_log;
		}

		bool isCompiled(void) const
		{
			return this->m_compiled;
		}

	protected:

		std::string  m_source;
		std::string  m_log;
		bool         m_compiled;

		Shader(Context * ctx)
			: BaseType   (ctx)
			, m_compiled (false)
		{
			;
		}

		virtual GLenum shaderType(void) const = 0;

		bool create(const ShaderArguments & args)
		{
			this->destroy();
			const GLenum shType = this->shaderType();
			this->m_name = glCreateShader(shType);
			this->compile(args.source);
			return this->m_compiled;
		}

		virtual void doDestroy(void)
		{
			glDeleteShader(this->m_name);
			this->m_source.clear();
			this->m_log.clear();
			this->m_compiled = false;
		}

		virtual bool doIsValid(void) const
		{
			return this->m_compiled;
		}

		void compile(const std::string & source)
		{
			const char * src = source.c_str();
			glShaderSource(this->m_name, 1, &src, 0);
			glCompileShader(this->m_name);

			GLint compileStatus = 0;
			glGetShaderiv(this->m_name, GL_COMPILE_STATUS, &compileStatus);

			this->m_source   = source;
			this->m_log      = ThisType::getInfoLog(this->m_name);
			this->m_compiled = (compileStatus != GL_FALSE);

#if GLW_PRINT_LOG_TO_STDERR
			std::cerr << "---------------------------" << std::endl;
			std::cerr << "[";
			switch (this->shaderType())
			{
				case GL_VERTEX_SHADER   : std::cerr << "Vertex ";   break;
				case GL_GEOMETRY_SHADER : std::cerr << "Geometry "; break;
				case GL_FRAGMENT_SHADER : std::cerr << "Fragment "; break;
				default: break;
			}
			std::cerr << "Shader Compile Log]: " << ((this->m_compiled) ? ("OK") : ("FAILED")) << std::endl;
			std::cerr << this->m_log << std::endl;
			std::cerr << "---------------------------" << std::endl;
#endif
		}

	private:

		static std::string getInfoLog(GLuint Shader)
		{
			std::string log;

			GLint logLen = 0;
			glGetShaderiv(Shader, GL_INFO_LOG_LENGTH, &logLen);
			if (logLen > 0)
			{
				char * sLog = new char[logLen + 1];
				glGetShaderInfoLog(Shader, logLen, &logLen, sLog);
				if (logLen > 0)
				{
					if (sLog[0] != '\0')
					{
						sLog[logLen - 1] = '\0';
						log = sLog;
					}
				}
				delete [] sLog;
			}
			return log;
		}
};

namespace detail { template <> struct BaseOf <Shader> { typedef Object Type; }; };
typedef   detail::ObjectSharedPointerTraits  <Shader> ::Type ShaderPtr;

class SafeShader : public SafeObject
{
	friend class Context;
	friend class BoundShader;

	public:

		typedef SafeObject BaseType;
		typedef SafeShader ThisType;

		SafeShader(void)
			: BaseType()
		{
			;
		}

		const std::string & source(void) const
		{
			return this->object()->source();
		}

		const std::string & log(void) const
		{
			return this->object()->log();
		}

		bool isCompiled(void) const
		{
			return this->object()->isCompiled();
		}

	protected:

		SafeShader(const ShaderPtr & shader)
			: BaseType(shader)
		{
			;
		}

		const ShaderPtr & object(void) const
		{
			return static_cast<const ShaderPtr &>(BaseType::object());
		}

		ShaderPtr & object(void)
		{
			return static_cast<ShaderPtr &>(BaseType::object());
		}
};

namespace detail { template <> struct BaseOf     <SafeShader> { typedef SafeObject Type; }; };
namespace detail { template <> struct ObjectBase <SafeShader> { typedef Shader     Type; }; };
namespace detail { template <> struct ObjectSafe <Shader    > { typedef SafeShader Type; }; };
typedef   detail::ObjectSharedPointerTraits      <SafeShader> ::Type ShaderHandle;

class ShaderBindingParams : public ObjectBindingParams
{
	public:

		typedef ObjectBindingParams BaseType;
		typedef ShaderBindingParams ThisType;

		ShaderBindingParams(void)
			: BaseType()
		{
			;
		}

		ShaderBindingParams(GLenum aTarget, GLenum aUnit)
			: BaseType(aTarget, aUnit)
		{
			;
		}
};

class BoundShader : public BoundObject
{
	friend class Context;

	public:

		typedef BoundObject BaseType;
		typedef BoundShader ThisType;

		BoundShader(void)
			: BaseType()
		{
			;
		}

		const ShaderHandle & handle(void) const
		{
			return static_cast<const ShaderHandle &>(BaseType::handle());
		}

		ShaderHandle & handle(void)
		{
			return static_cast<ShaderHandle &>(BaseType::handle());
		}

	protected:

		BoundShader(const ShaderHandle & handle, const ShaderBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}

		const ShaderPtr & object(void) const
		{
			return this->handle()->object();
		}

		ShaderPtr & object(void)
		{
			return this->handle()->object();
		}

		virtual void bind(void)
		{
			;
		}

		virtual void unbind(void)
		{
			;
		}
};

namespace detail { template <> struct ParamsOf    <BoundShader> { typedef ShaderBindingParams Type; }; };
namespace detail { template <> struct BaseOf      <BoundShader> { typedef BoundObject Type; }; };
namespace detail { template <> struct ObjectBase  <BoundShader> { typedef Shader      Type; }; };
namespace detail { template <> struct ObjectBound <Shader     > { typedef BoundShader Type; }; };
typedef   detail::ObjectSharedPointerTraits       <BoundShader> ::Type  BoundShaderHandle;

};

#endif // GLW_SHADER_H
