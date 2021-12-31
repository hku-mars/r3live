#ifndef GLW_PROGRAM_H
#define GLW_PROGRAM_H

#include <memory.h>

#include <string>
#include <vector>
#include <map>

#include "./vertexshader.h"
#include "./geometryshader.h"
#include "./fragmentshader.h"

namespace glw
{

typedef std::vector<ShaderHandle> ShaderHandleVector;

class VertexAttributeBinding
{
	public:

		typedef void                   BaseType;
		typedef VertexAttributeBinding ThisType;

		typedef std::map<std::string, GLuint> Map;
		typedef Map::const_iterator           ConstIterator;
		typedef Map::iterator                 Iterator;
		typedef Map::value_type               Value;

		Map bindings;

		VertexAttributeBinding(void)
		{
			this->clear();
		}

		void clear(void)
		{
			this->bindings.clear();
		}

		GLuint operator [] (const std::string & attributeName) const
		{
			return this->bindings.find(attributeName)->second;
		}

		GLuint & operator [] (const std::string & attributeName)
		{
			return this->bindings[attributeName];
		}
};

class GeometryStage
{
	public:

		typedef void          BaseType;
		typedef GeometryStage ThisType;

		/*
		GLenum inputPrimitiveType;
		GLenum outputPrimitiveType;
		GLint  maxOutputVertices;
		*/

		GeometryStage(void)
		{
			this->clear();
		}

		void clear(void)
		{
			/*
			this->inputPrimitiveType  = GLW_DONT_CARE;
			this->outputPrimitiveType = GLW_DONT_CARE;
			this->maxOutputVertices   = GLW_DONT_CARE;
			*/
		}
};

class TransformFeedbackStream
{
	public:

		typedef void                    BaseType;
		typedef TransformFeedbackStream ThisType;

		typedef std::vector<std::string> VaryingVector;

		VaryingVector varyings;
		GLenum        bufferMode;

		TransformFeedbackStream(void)
		{
			this->clear();
		}

		void clear(void)
		{
			this->varyings.clear();
			this->bufferMode = GL_INTERLEAVED_ATTRIBS;
		}
};

class RasterizerSettings
{
	public:

		typedef void               BaseType;
		typedef RasterizerSettings ThisType;

		enum RasterizerExecution
		{
			DontCare = 0,
			Autodetect,
			ForceEnabled,
			ForceDisabled
		};

		// TODO
		//RasterizerExecution execution;

		RasterizerSettings(void)
		{
			this->clear();
		}

		void clear(void)
		{
			//this->execution = ThisType::Autodetect;
		}
};

class FragmentOutputBinding
{
	public:

		typedef void                  BaseType;
		typedef FragmentOutputBinding ThisType;

		typedef std::map<std::string, GLuint> Map;
		typedef Map::const_iterator           ConstIterator;
		typedef Map::iterator                 Iterator;
		typedef Map::value_type               Value;

		Map bindings;

		FragmentOutputBinding(void)
		{
			this->clear();
		}

		void clear(void)
		{
			this->bindings.clear();
		}

		GLuint operator [] (const std::string & outName) const
		{
			return this->bindings.find(outName)->second;
		}

		GLuint & operator [] (const std::string & outName)
		{
			return this->bindings[outName];
		}
};

class ProgramArguments : public ObjectArguments
{
	public:

		typedef ObjectArguments    BaseType;
		typedef ProgramArguments   ThisType;

		ShaderHandleVector         shaders;
		VertexAttributeBinding     vertexInputs;
		GeometryStage              geometryStage;
		TransformFeedbackStream    feedbackStream;
		RasterizerSettings         rasterSettings;
		FragmentOutputBinding      fragmentOutputs;

		ProgramArguments(void)
			: BaseType()
		{
			;
		}

		void clear(void)
		{
			BaseType::clear();
			this->shaders         .clear();
			this->vertexInputs    .clear();
			this->geometryStage   .clear();
			this->feedbackStream  .clear();
			this->rasterSettings  .clear();
			this->fragmentOutputs .clear();
		}
};

class Program : public Object
{
	friend class Context;

	public:

		typedef Object  BaseType;
		typedef Program ThisType;

		virtual ~Program(void)
		{
			this->destroy();
		}

		virtual Type type(void) const
		{
			return ProgramType;
		}

		const ProgramArguments & arguments(void) const
		{
			return this->m_arguments;
		}

		const std::string & log(void) const
		{
			return this->m_log;
		}

		const std::string & fullLog(void) const
		{
			return this->m_fullLog;
		}

		bool isLinked(void) const
		{
			return this->m_linked;
		}

		GLint getUniformLocation(const std::string & name) const
		{
#if GLW_ASSERT_UNIFORM_LOCATION
			GLW_ASSERT(this->m_uniforms.count(name) > 0);
#endif
			UniformMapConstIterator it = this->m_uniforms.find(name);
			if (it == this->m_uniforms.end()) return -1;
			return it->second.location;
		}

#define _GLW_IMPLEMENT_SCALAR_UNIFORM_(TYPE, FUNCION_SUFFIX) \
	void setUniform    (const std::string & name, TYPE x                                                       ) { glUniform1         ## FUNCION_SUFFIX     (this->getUniformLocation(name),                                       x         ); } \
	void setUniform    (const std::string & name, TYPE x, TYPE y                                               ) { glUniform2         ## FUNCION_SUFFIX     (this->getUniformLocation(name),                                       x, y      ); } \
	void setUniform    (const std::string & name, TYPE x, TYPE y, TYPE z                                       ) { glUniform3         ## FUNCION_SUFFIX     (this->getUniformLocation(name),                                       x, y, z   ); } \
	void setUniform    (const std::string & name, TYPE x, TYPE y, TYPE z, TYPE w                               ) { glUniform4         ## FUNCION_SUFFIX     (this->getUniformLocation(name),                                       x, y, z, w); }

#define _GLW_IMPLEMENT_VECTOR_UNIFORM_(TYPE, FUNCION_SUFFIX) \
	void setUniform1   (const std::string & name, const TYPE * v,                                 int count = 1) { glUniform1         ## FUNCION_SUFFIX ## v (this->getUniformLocation(name), GLsizei(count),                       v         ); } \
	void setUniform2   (const std::string & name, const TYPE * v,                                 int count = 1) { glUniform2         ## FUNCION_SUFFIX ## v (this->getUniformLocation(name), GLsizei(count),                       v         ); } \
	void setUniform3   (const std::string & name, const TYPE * v,                                 int count = 1) { glUniform3         ## FUNCION_SUFFIX ## v (this->getUniformLocation(name), GLsizei(count),                       v         ); } \
	void setUniform4   (const std::string & name, const TYPE * v,                                 int count = 1) { glUniform4         ## FUNCION_SUFFIX ## v (this->getUniformLocation(name), GLsizei(count),                       v         ); }

#define _GLW_IMPLEMENT_MATRIX_UNIFORM_(TYPE, FUNCION_SUFFIX) \
	void setUniform2x2 (const std::string & name, const TYPE * m,                 bool transpose, int count = 1) { glUniformMatrix2   ## FUNCION_SUFFIX ## v (this->getUniformLocation(name), GLsizei(count), GLboolean(transpose), m         ); } \
	void setUniform2x3 (const std::string & name, const TYPE * m,                 bool transpose, int count = 1) { glUniformMatrix2x3 ## FUNCION_SUFFIX ## v (this->getUniformLocation(name), GLsizei(count), GLboolean(transpose), m         ); } \
	void setUniform2x4 (const std::string & name, const TYPE * m,                 bool transpose, int count = 1) { glUniformMatrix2x4 ## FUNCION_SUFFIX ## v (this->getUniformLocation(name), GLsizei(count), GLboolean(transpose), m         ); } \
	void setUniform3x2 (const std::string & name, const TYPE * m,                 bool transpose, int count = 1) { glUniformMatrix3x2 ## FUNCION_SUFFIX ## v (this->getUniformLocation(name), GLsizei(count), GLboolean(transpose), m         ); } \
	void setUniform3x3 (const std::string & name, const TYPE * m,                 bool transpose, int count = 1) { glUniformMatrix3   ## FUNCION_SUFFIX ## v (this->getUniformLocation(name), GLsizei(count), GLboolean(transpose), m         ); } \
	void setUniform3x4 (const std::string & name, const TYPE * m,                 bool transpose, int count = 1) { glUniformMatrix3x4 ## FUNCION_SUFFIX ## v (this->getUniformLocation(name), GLsizei(count), GLboolean(transpose), m         ); } \
	void setUniform4x2 (const std::string & name, const TYPE * m,                 bool transpose, int count = 1) { glUniformMatrix4x2 ## FUNCION_SUFFIX ## v (this->getUniformLocation(name), GLsizei(count), GLboolean(transpose), m         ); } \
	void setUniform4x3 (const std::string & name, const TYPE * m,                 bool transpose, int count = 1) { glUniformMatrix4x3 ## FUNCION_SUFFIX ## v (this->getUniformLocation(name), GLsizei(count), GLboolean(transpose), m         ); } \
	void setUniform4x4 (const std::string & name, const TYPE * m,                 bool transpose, int count = 1) { glUniformMatrix4   ## FUNCION_SUFFIX ## v (this->getUniformLocation(name), GLsizei(count), GLboolean(transpose), m         ); }

		_GLW_IMPLEMENT_SCALAR_UNIFORM_(int,          i )
		_GLW_IMPLEMENT_SCALAR_UNIFORM_(unsigned int, ui)
		_GLW_IMPLEMENT_SCALAR_UNIFORM_(float,        f )
		_GLW_IMPLEMENT_VECTOR_UNIFORM_(int,          i )
		_GLW_IMPLEMENT_VECTOR_UNIFORM_(unsigned int, ui)
		_GLW_IMPLEMENT_VECTOR_UNIFORM_(float,        f )
		_GLW_IMPLEMENT_MATRIX_UNIFORM_(float,        f )

		GLW_IMPLEMENT_CUSTOM_UNIFORMS;

#undef _GLW_IMPLEMENT_SCALAR_UNIFORM_
#undef _GLW_IMPLEMENT_VECTOR_UNIFORM_
#undef _GLW_IMPLEMENT_MATRIX_UNIFORM_

	protected:

		Program(Context * ctx)
			: BaseType (ctx)
			, m_linked (false)
		{
			;
		}

		bool create(const ProgramArguments & args)
		{
			this->destroy();

			this->m_arguments = args;

			GLint boundName = 0;
			glGetIntegerv(GL_CURRENT_PROGRAM, &boundName);

			this->m_name = glCreateProgram();
			this->m_fullLog = "";

			// shaders
			{
				for (size_t i=0; i<this->m_arguments.shaders.size(); ++i)
				{
					const ShaderHandle & shader = this->m_arguments.shaders[i];
					if (!shader) continue;
					this->m_fullLog += shader->log();
					if (!shader->isCompiled()) continue;
					glAttachShader(this->m_name, shader->name());
				}
			}

			// vertex
			{
				for (VertexAttributeBinding::ConstIterator it=this->m_arguments.vertexInputs.bindings.begin(); it!=this->m_arguments.vertexInputs.bindings.end(); ++it)
				{
					glBindAttribLocation(this->m_name, it->second, it->first.c_str());
				}
			}

			// geometry
			{
				;
			}

			// transform feedback
			{
				const size_t count = this->m_arguments.feedbackStream.varyings.size();
				if (count > 0)
				{
					const char ** varyings = new const char * [count];
					for (size_t i=0; i<count; ++i)
					{
						varyings[i]  = this->m_arguments.feedbackStream.varyings[i].c_str();
					}
					glTransformFeedbackVaryings(this->m_name, GLsizei(count), varyings, this->m_arguments.feedbackStream.bufferMode);
					delete [] varyings;
				}
			}

			// TODO
			// rasterizer
			{
				;
			}

			// fragment
			{
				for (FragmentOutputBinding::ConstIterator it=this->m_arguments.fragmentOutputs.bindings.begin(); it!=this->m_arguments.fragmentOutputs.bindings.end(); ++it)
				{
					glBindFragDataLocation(this->m_name, it->second, it->first.c_str());
				}
			}

			glLinkProgram(this->m_name);

			GLint linkStatus = 0;
			glGetProgramiv(this->m_name, GL_LINK_STATUS, &linkStatus);

			this->m_log      = ThisType::getInfoLog(this->m_name);
			this->m_fullLog += this->m_log;
			this->m_linked   = (linkStatus != GL_FALSE);

#if GLW_PRINT_LOG_TO_STDERR
			std::cerr << "---------------------------" << std::endl;
			std::cerr << "[Program Link Log]: " << ((this->m_linked) ? ("OK") : ("FAILED")) << std::endl;
			std::cerr << this->m_log << std::endl;
			std::cerr << "---------------------------" << std::endl;
#endif

			if (this->m_linked)
			{
				this->postLink();
			}

			glUseProgram(boundName);

			return this->m_linked;
		}

		virtual void doDestroy()
		{
			glDeleteProgram(this->m_name);
			this->m_arguments.clear();
			this->m_log.clear();
			this->m_fullLog.clear();
			this->m_linked = false;
		}

		virtual bool doIsValid(void) const
		{
			return this->m_linked;
		}

	private:

		class UniformInfo
		{
			public:

				typedef void        BaseType;
				typedef UniformInfo ThisType;

				std::string name;
				GLint       location;
				GLenum      type;
				GLint       size;

				UniformInfo(void)
					: location (-1)
					, type     (GL_NONE)
					, size     (0)
				{
					;
				}
		};

		typedef std::map<std::string, UniformInfo> UniformMap;
		typedef UniformMap::const_iterator         UniformMapConstIterator;
		typedef UniformMap::iterator               UniformMapIterator;
		typedef UniformMap::value_type             UniformMapValue;

		ProgramArguments m_arguments;
		UniformMap       m_uniforms;
		std::string      m_log;
		std::string      m_fullLog;
		bool             m_linked;

		static std::string getInfoLog(GLuint Program)
		{
			std::string log;
			GLint logLen = 0;
			glGetProgramiv(Program, GL_INFO_LOG_LENGTH, &logLen);
			if (logLen > 0)
			{
				char * sLog = new char[logLen + 1];
				glGetProgramInfoLog(Program, logLen, &logLen, sLog);
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

		void setupUniforms(void)
		{
			this->m_uniforms.clear();

			GLint ucount = 0;
			glGetProgramiv(this->m_name,  GL_ACTIVE_UNIFORMS, &ucount);
			if (ucount <= 0) return;

			GLint ulen = 0;
			glGetProgramiv(this->m_name, GL_ACTIVE_UNIFORM_MAX_LENGTH, &ulen);
			ulen++; // according to specs, +1 (for null) is already accounted, but some implementations are broken.
			if (ulen <= 0) return;

			UniformInfo info;
			GLchar * uname = new GLchar [ulen + 1];
			for (int i=0; i<int(ucount); ++i)
			{
				GLsizei length = 0;
				glGetActiveUniform(this->m_name, GLuint(i), GLsizei(ulen), &length, &(info.size), &(info.type), uname);
				info.name     = uname;
				info.location = glGetUniformLocation(this->m_name, uname);
				this->m_uniforms.insert(UniformMapValue(info.name, info));
			}
			delete [] uname;
		}

		void postLink(void)
		{
			this->setupUniforms();
		}
};

namespace detail { template <> struct BaseOf <Program> { typedef Object Type; }; };
typedef   detail::ObjectSharedPointerTraits  <Program> ::Type ProgramPtr;

class SafeProgram : public SafeObject
{
	friend class Context;
	friend class BoundProgram;

	public:

		typedef SafeObject  BaseType;
		typedef SafeProgram ThisType;

		const ProgramArguments & arguments(void) const
		{
			return this->object()->arguments();
		}

		const std::string & log(void) const
		{
			return this->object()->log();
		}

		const std::string & fullLog(void) const
		{
			return this->object()->fullLog();
		}

		bool isLinked(void) const
		{
			return this->object()->isLinked();
		}

	protected:

		SafeProgram(const ProgramPtr & program)
			: BaseType(program)
		{
			;
		}

		const ProgramPtr & object(void) const
		{
			return static_cast<const ProgramPtr &>(BaseType::object());
		}

		ProgramPtr & object(void)
		{
			return static_cast<ProgramPtr &>(BaseType::object());
		}
};

namespace detail { template <> struct BaseOf     <SafeProgram> { typedef SafeObject Type; }; };
namespace detail { template <> struct ObjectBase <SafeProgram> { typedef Program     Type; }; };
namespace detail { template <> struct ObjectSafe <Program    > { typedef SafeProgram Type; }; };
typedef   detail::ObjectSharedPointerTraits      <SafeProgram> ::Type ProgramHandle;

class ProgramBindingParams : public ObjectBindingParams
{
	public:

		typedef ObjectBindingParams BaseType;
		typedef ProgramBindingParams ThisType;

		ProgramBindingParams(void)
			: BaseType(GL_CURRENT_PROGRAM, 0)
		{
			;
		}
};

class BoundProgram : public BoundObject
{
	friend class Context;

	public:

		typedef BoundObject BaseType;
		typedef BoundProgram ThisType;

		BoundProgram(void)
			: BaseType()
		{
			;
		}

		const ProgramHandle & handle(void) const
		{
			return static_cast<const ProgramHandle &>(BaseType::handle());
		}

		ProgramHandle & handle(void)
		{
			return static_cast<ProgramHandle &>(BaseType::handle());
		}

#define _GLW_FORWARD_SCALAR_UNIFORM_(TYPE) \
	void setUniform    (const std::string & name, TYPE x                                                       ) { this->object()->setUniform(name, x         ); } \
	void setUniform    (const std::string & name, TYPE x, TYPE y                                               ) { this->object()->setUniform(name, x, y      ); } \
	void setUniform    (const std::string & name, TYPE x, TYPE y, TYPE z                                       ) { this->object()->setUniform(name, x, y, z   ); } \
	void setUniform    (const std::string & name, TYPE x, TYPE y, TYPE z, TYPE w                               ) { this->object()->setUniform(name, x, y, z, w); }

#define _GLW_FORWARD_VECTOR_UNIFORM_(TYPE) \
	void setUniform1   (const std::string & name, const TYPE * v,                                 int count = 1) { this->object()->setUniform1(name, v, count); } \
	void setUniform2   (const std::string & name, const TYPE * v,                                 int count = 1) { this->object()->setUniform2(name, v, count); } \
	void setUniform3   (const std::string & name, const TYPE * v,                                 int count = 1) { this->object()->setUniform3(name, v, count); } \
	void setUniform4   (const std::string & name, const TYPE * v,                                 int count = 1) { this->object()->setUniform4(name, v, count); }

#define _GLW_FORWARD_MATRIX_UNIFORM_(TYPE) \
	void setUniform2x2 (const std::string & name, const TYPE * m,                 bool transpose, int count = 1) { this->object()->setUniform2x2(name, m, transpose, count); } \
	void setUniform2x3 (const std::string & name, const TYPE * m,                 bool transpose, int count = 1) { this->object()->setUniform2x3(name, m, transpose, count); } \
	void setUniform2x4 (const std::string & name, const TYPE * m,                 bool transpose, int count = 1) { this->object()->setUniform2x4(name, m, transpose, count); } \
	void setUniform3x2 (const std::string & name, const TYPE * m,                 bool transpose, int count = 1) { this->object()->setUniform3x2(name, m, transpose, count); } \
	void setUniform3x3 (const std::string & name, const TYPE * m,                 bool transpose, int count = 1) { this->object()->setUniform3x3(name, m, transpose, count); } \
	void setUniform3x4 (const std::string & name, const TYPE * m,                 bool transpose, int count = 1) { this->object()->setUniform3x4(name, m, transpose, count); } \
	void setUniform4x2 (const std::string & name, const TYPE * m,                 bool transpose, int count = 1) { this->object()->setUniform4x2(name, m, transpose, count); } \
	void setUniform4x3 (const std::string & name, const TYPE * m,                 bool transpose, int count = 1) { this->object()->setUniform4x3(name, m, transpose, count); } \
	void setUniform4x4 (const std::string & name, const TYPE * m,                 bool transpose, int count = 1) { this->object()->setUniform4x4(name, m, transpose, count); }

		_GLW_FORWARD_SCALAR_UNIFORM_(int)
		_GLW_FORWARD_SCALAR_UNIFORM_(unsigned int)
		_GLW_FORWARD_SCALAR_UNIFORM_(float)
		_GLW_FORWARD_VECTOR_UNIFORM_(int)
		_GLW_FORWARD_VECTOR_UNIFORM_(unsigned int)
		_GLW_FORWARD_VECTOR_UNIFORM_(float)
		_GLW_FORWARD_MATRIX_UNIFORM_(float)

#undef _GLW_FORWARD_SCALAR_UNIFORM_
#undef _GLW_FORWARD_VECTOR_UNIFORM_
#undef _GLW_FORWARD_MATRIX_UNIFORM_

	protected:

		BoundProgram(const ProgramHandle & handle, const ProgramBindingParams & params)
			: BaseType(handle, params)
		{
			;
		}

		const ProgramPtr & object(void) const
		{
			return this->handle()->object();
		}

		ProgramPtr & object(void)
		{
			return this->handle()->object();
		}

		virtual void bind(void)
		{
			glUseProgram(this->object()->name());
		}

		virtual void unbind(void)
		{
			glUseProgram(0);
		}
};

namespace detail { template <> struct ParamsOf    <BoundProgram> { typedef ProgramBindingParams Type; }; };
namespace detail { template <> struct BaseOf      <BoundProgram> { typedef BoundObject Type; }; };
namespace detail { template <> struct ObjectBase  <BoundProgram> { typedef Program      Type; }; };
namespace detail { template <> struct ObjectBound <Program     > { typedef BoundProgram Type; }; };
typedef   detail::ObjectSharedPointerTraits       <BoundProgram> ::Type  BoundProgramHandle;

};

#endif // GLW_PROGRAM_H
