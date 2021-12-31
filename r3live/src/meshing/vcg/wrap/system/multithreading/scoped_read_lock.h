#ifndef MT_SCOPED_READ_LOCK_H
#define MT_SCOPED_READ_LOCK_H

#include "rw_lock.h"

namespace mt
{

class scoped_read_lock
{
	MT_PREVENT_COPY(scoped_read_lock)

	public:

		typedef scoped_read_lock this_type;
		typedef void             base_type;

		scoped_read_lock(rw_lock & rwl) : rw(rwl)
		{
			this->rw.lock_read();
		}

		~scoped_read_lock(void)
		{
			this->rw.unlock_read();
		}

	protected:

		rw_lock & rw;
};

}

#endif // MT_SCOPED_READ_LOCK_H
