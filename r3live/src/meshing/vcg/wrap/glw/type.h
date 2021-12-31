#ifndef GLW_TYPE_H
#define GLW_TYPE_H

#include "./common.h"

namespace glw
{

enum Type
{
	InvalidType = 0,
	BufferType,
	RenderbufferType,
	VertexShaderType,
	GeometryShaderType,
	FragmentShaderType,
	ProgramType,
	Texture2DType,
	TextureCubeType,
	FramebufferType
};

};

#endif // GLW_TYPE_H
