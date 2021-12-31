#ifndef MT_SCOPED_WRITE_LOCK_H
#define MT_SCOPED_WRITE_LOCK_H

#include "rw_lock.h"

namespace mt
{

class scoped_write_lock
{
	MT_PREVENT_COPY(scoped_write_lock)

	public:

		typedef scoped_write_lock this_type;
		typedef void             base_type;

		scoped_write_lock(rw_lock & rwl) : rw(rwl)
		{
			this->rw.lock_write();
		}

		~scoped_write_lock(void)
		{
			this->rw.unlock_write();
		}

	protected:

		rw_lock & rw;
};

}

#endif // MT_SCOPED_WRITE_LOCK_H
