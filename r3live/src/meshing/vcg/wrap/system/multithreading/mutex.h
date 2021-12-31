#ifndef MT_MUTEX_H
#define MT_MUTEX_H

#include "base.h"

#include <pthread.h>

namespace mt
{

class condition;

class mutex
{
	MT_PREVENT_COPY(mutex)

	public:

		typedef mutex this_type;
		typedef void  base_type;

		mutex(void)
		{
			pthread_mutex_init(&(this->m), 0);
		}

		~mutex(void)
		{
			pthread_mutex_destroy(&(this->m));
		}

		void lock(void)
		{
			pthread_mutex_lock(&(this->m));
		}

		void unlock(void)
		{
			pthread_mutex_unlock(&(this->m));
		}

	private:

		friend class condition;

		pthread_mutex_t m;
};

}

#endif // MT_MUTEX_H
