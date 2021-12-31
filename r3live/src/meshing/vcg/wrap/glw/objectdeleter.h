#ifndef GLW_OBJECTDELETER_H
#define GLW_OBJECTDELETER_H

#include "./common.h"

namespace glw
{

class Object;

namespace detail
{

class ObjectDeleter
{
	public:

		typedef void          BaseType;
		typedef ObjectDeleter ThisType;

		inline void operator () (Object * object) const;
};

};

};

#endif // GLW_OBJECTDELETER_H
