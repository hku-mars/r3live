#ifndef GLW_COMMON_H
#define GLW_COMMON_H

#include <assert.h>

#include "./config.h"

#define GLW_DONT_CARE                                (0xFFFFFFFF)
#define GLW_CARE_OF(X)                               ((X) != GLW_DONT_CARE)

#define GLW_CHECK_GL_ERROR                           GLW_ASSERT(glGetError() == GL_NO_ERROR)
#define GLW_CHECK_GL_READ_FRAMEBUFFER_STATUS         GLW_ASSERT(glCheckFramebufferStatus(GL_READ_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE)
#define GLW_CHECK_GL_DRAW_FRAMEBUFFER_STATUS         GLW_ASSERT(glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE)
#define GLW_CHECK_GL_READ_DRAW_FRAMEBUFFER_STATUS    GLW_ASSERT(glCheckFramebufferStatus(GL_FRAMEBUFFER)      == GL_FRAMEBUFFER_COMPLETE)

namespace glw
{

namespace detail
{

}

};

#endif // GLW_COMMON_H
