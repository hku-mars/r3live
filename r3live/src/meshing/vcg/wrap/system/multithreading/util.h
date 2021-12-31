#ifndef MT_UTIL_H
#define MT_UTIL_H

#if (defined(_MSC_VER))
#include <windows.h>
#elif (defined(__MINGW32__))
#include <windows.h>
#elif (defined(__linux__))
#include <unistd.h>
#else
#include <unistd.h>
//#error "mt utils.h : unrecognized environment."
#endif

namespace mt
{

inline void sleep_ms(unsigned int msecs)
{
#if (defined(_MSC_VER))
	Sleep(DWORD(msecs));
#elif (defined(__MINGW32__))
	Sleep(DWORD(msecs));
#elif (defined(__linux__))
	const unsigned int  secs  = ((unsigned int)(msecs / 1000));
	const unsigned long usecs = ((unsigned long)((msecs % 1000) * 1000));

	sleep(secs);
	usleep(usecs);
#else
	const unsigned int  secs  = ((unsigned int)(msecs / 1000));
	const unsigned long usecs = ((unsigned long)((msecs % 1000) * 1000));

	sleep(secs);
	usleep(usecs);
#endif
}

}

#endif // MT_UTIL_H
