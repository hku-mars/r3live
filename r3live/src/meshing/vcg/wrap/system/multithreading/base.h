#ifndef MT_BASE_H
#define MT_BASE_H

#include <assert.h>

#define MT_PREVENT_COPY(CLASS_NAME) \
	private: \
		CLASS_NAME (const CLASS_NAME &); \
		CLASS_NAME & operator = (const CLASS_NAME &);

#define MT_ASSERT  assert

namespace mt
{

}

#endif // MT_BASE_H
