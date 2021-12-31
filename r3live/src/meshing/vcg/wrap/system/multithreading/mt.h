#ifndef MT_MT_H
#define MT_MT_H


#ifdef QT_CORE_LIB

#include <QThread>
#include <QMutex>
#include <QSemaphore>

namespace mt{
  typedef QThread thread;
  typedef QMutex mutex;
  typedef QMutexLocker mutexlocker;
  typedef QSemaphore semaphore;

//cache.h, token.h
//QAtomicInt

}//namespace


#else

#include "base.h"
#include "mutex.h"
#include "rw_lock.h"
#include "semaphore.h"
#include "thread.h"
#include "scoped_mutex_lock.h"
#include "scoped_read_lock.h"
#include "scoped_write_lock.h"

namespace mt{
  typedef scoped_mutex_lock mutexlocker;
}
#endif

#endif // MT_MT_H
