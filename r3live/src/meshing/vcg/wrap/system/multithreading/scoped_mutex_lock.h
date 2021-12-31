#ifndef MT_SCOPED_MUTEX_LOCK_H
#define MT_SCOPED_MUTEX_LOCK_H

#include "mutex.h"

namespace mt
{

class scoped_mutex_lock
{
    MT_PREVENT_COPY(scoped_mutex_lock)

    public:

            typedef scoped_mutex_lock this_type;
            typedef void              base_type;

            scoped_mutex_lock(mutex & m) : mtx(m)
            {
                    this->mtx.lock();
            }

            ~scoped_mutex_lock(void)
            {
                    this->mtx.unlock();
            }

            //jnoguera 14-12-2011
            //method added to mime QMutexLocker
            scoped_mutex_lock(mutex * m) : mtx( *m )
            {
                    this->mtx.lock();
            }


    protected:

            mutex & mtx;
};

}

#endif // MT_SCOPED_MUTEX_LOCK_H





/*
#ifndef MT_SCOPED_MUTEX_LOCK_H
#define MT_SCOPED_MUTEX_LOCK_H

#include "mutex.h"

namespace mt
{

class scoped_mutex_lock
{
    MT_PREVENT_COPY(scoped_mutex_lock)

    public:

            typedef scoped_mutex_lock this_type;
            typedef void              base_type;

            scoped_mutex_lock(mutex & m) : mtx(m)
            {
                    this->mtx.lock();
            }

            ~scoped_mutex_lock(void)
            {
                    this->mtx.unlock();
            }

    protected:

            mutex & mtx;
};

}

#endif // MT_SCOPED_MUTEX_LOCK_H

*/


