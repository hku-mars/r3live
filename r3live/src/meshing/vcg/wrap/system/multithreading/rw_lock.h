#ifndef MT_RW_LOCK_H
#define MT_RW_LOCK_H

#include "base.h"

#include <pthread.h>

namespace mt
{

class rw_lock
{
	MT_PREVENT_COPY(rw_lock)

	public:

		typedef rw_lock this_type;
		typedef void    base_type;

		rw_lock(void)
		{
			pthread_rwlock_init(&(this->rw), 0);
		}

		~rw_lock(void)
		{
			pthread_rwlock_destroy(&(this->rw));
		}

		void lock_read(void)
		{
			pthread_rwlock_rdlock(&(this->rw));
		}

		void unlock_read(void)
		{
			pthread_rwlock_unlock(&(this->rw));
		}

		void lock_write(void)
		{
			pthread_rwlock_wrlock(&(this->rw));
		}

		void unlock_write(void)
		{
			pthread_rwlock_unlock(&(this->rw));
		}

	private:

		pthread_rwlock_t rw;
};

}

#endif // MT_RW_LOCK_H
