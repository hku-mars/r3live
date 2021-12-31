#ifndef GLW_NONCOPYABLE_H
#define GLW_NONCOPYABLE_H

#include "./common.h"

namespace glw
{

namespace detail
{

class NonCopyable
{
	public:

		typedef void        BaseType;
		typedef NonCopyable ThisType;

		NonCopyable(void)
		{
			;
		}

	private:

		NonCopyable(const ThisType & that)
		{
			(void)that;
		}

		ThisType & operator = (const ThisType & that)
		{
			(void)that;
			return (*this);
		}
};

};

};

#endif // GLW_NONCOPYABLE_H
