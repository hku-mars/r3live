#ifndef MT_THREAD_H
#define MT_THREAD_H

#include <memory.h>

#include "base.h"

#include <pthread.h>

namespace mt
{

class thread
{
    MT_PREVENT_COPY(thread)

    public:

            typedef thread this_type;
            typedef void   base_type;

            thread(void) : flags(0)
            {
                    ;
            }

            virtual ~thread(void)
            {
                    ;
            }

            virtual bool start(void)
            {
                    if ((this->flags & thread_started) != 0) return false;

                    pthread_create(&(this->tid), 0, this_type::thread_func, reinterpret_cast<void *>(this));

                    /*
                    sched_param sp;
                    memset(&sp, 0, sizeof(sched_param));
                    sp.sched_priority = sched_get_priority_min(SCHED_OTHER);
                    sp.sched_priority = 0; // normal
                    pthread_setschedparam(this->tid, SCHED_OTHER, &sp);
                    */

                    this->flags |= thread_started;

                    return true;
            }

            virtual bool wait(void)
            {
                    if ((this->flags & thread_started) == 0) return false;
                    pthread_join(this->tid, 0);
                    this->flags &= ~thread_started;
                    return true;
            }

            virtual bool kill(void)
            {
                    if ((this->flags & thread_started) == 0) return false;
                    pthread_kill(this->tid, 0);
                    this->flags &= ~(thread_started | thread_running);
                    return true;
            }

            bool is_started(void) const
            {
                    return ((this->flags & thread_started) != 0);
            }

            bool is_running(void) const
            {
                    return ((this->flags & thread_running) != 0);
            }

    protected:

            virtual void run(void)
            {
                    ;
            }

    private:

            enum thread_flags
            {
                    thread_none    = (     0),
                    thread_started = (1 << 0),
                    thread_running = (1 << 1)
            };

            volatile unsigned int flags;
            pthread_t tid;

            static void * thread_func(void * param)
            {
                    this_type * p_this = reinterpret_cast<this_type *>(param);
                    MT_ASSERT(p_this != 0);

                    p_this->flags |= thread_running;
                    p_this->run();
                    p_this->flags &= ~thread_running;

                    return 0;
            }
};

}

#endif // MT_THREAD_H
