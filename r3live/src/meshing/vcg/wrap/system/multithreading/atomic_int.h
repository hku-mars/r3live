#ifndef _ATOMIC_INT_H
#define _ATOMIC_INT_H

#ifdef QT_CORE_LIB
  #include <QAtomicInt>
  namespace mt{
  	typedef QAtomicInt atomicInt;
  }

#elif defined(__APPLE__)
#  include "atomic_int_apple.h"


//generic implementation using mutexes
#else
#  include "atomic_int_generic.h"
#endif

/*
#elif defined(_WIN32)
#  include "atomic_int_win32.h"
#endif

#elif defined(__linux__)
#  include "atomic_int_linux.h"
#endif
*/
/*
__linux__
__unix__
__posix__
*/

#endif // _ATOMIC_INT_H

