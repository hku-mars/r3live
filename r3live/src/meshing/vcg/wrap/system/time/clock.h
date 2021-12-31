//*******************************************************************
// Clock.h -- Header for Clock.cpp
// Copyright (c) 2011 Jose Maria Noguera
// Dec 9, 2011
//
// Jose Maria Noguera Rozua http://wwwdi.ujaen.es/~jnoguera/
//
//*******************************************************************

#ifndef _CHRONOMETER_H_
#define _CHRONOMETER_H_

#ifdef QT_CORE_LIB

#include <QTime>

namespace mt{
  typedef QTime Clock;
}//namespace

#else

#ifdef _WIN32 
	#include <windows.h>
#else
	#include <sys/time.h>
#endif

namespace mt{

/** @brief Clock used to measure performance of algoritms.
Behaves similarly to QTime (QT).*/

class Clock{

public:
  /** Initializes the Clock */
  Clock()
  {
  #ifdef _WIN32
    QueryPerformanceFrequency(&freq);
  #endif
  }

  /** Returns the current time as reported by the system clock. */
  static Clock currentTime()
  {
    Clock c;
    c.start();
    return c;
  }

  /** Sets this time to the current time. This is practical for timing. */
  inline void start()
  {
  #ifdef _WIN32
    QueryPerformanceCounter(&tini);
  #else
    gettimeofday (&tini, 0);
  #endif
  }

  /** Sets this time to the current time and returns the number of
  milliseconds that have elapsed since the last time start()
  or restart() was called.*/
  inline int restart()
  {
    return elapsedAux(true);
  }

  /** Returns the number of milliseconds that have elapsed since the
  last time start() or restart() was called. */
  inline int elapsed()
  {
    return elapsedAux(false);
  }


  /* Returns the number of milliseconds from this time to t. If t is earlier
  than this time, the number of milliseconds returned is negative.*/
  int msecsTo ( const Clock & t ) const
  {
  #ifdef _WIN32
    LARGE_INTEGER tacum;
    tacum.QuadPart = (t.tini.QuadPart - this->tini.QuadPart);
    return (int) tacum.QuadPart / ((int)freq.QuadPart / 1000);
  #else
    long int sec = t.tini.tv_sec - this->tini.tv_sec;
    long int usec = t.tini.tv_usec - this->tini.tv_usec;
    if (usec < 0) {
      sec --;
      usec += 1000000u;
    }
    return (int) sec * 1000.0f + (int) usec / 1000.0f;
  #endif
  }

protected:

#ifdef _WIN32
  /* Initial time */
  LARGE_INTEGER tini;
  /* Frequency */
  LARGE_INTEGER freq;
#else
  /** Initial time */
  timeval tini;
#endif

  /** Returns the number of milliseconds that have elapsed since the
  last time start() or restart() was called. 
  If restart is true, it also sets this time to the current time.*/
  int elapsedAux( bool restart )
  {
  #ifdef _WIN32
    LARGE_INTEGER tnow;
    LARGE_INTEGER tacum;

    QueryPerformanceCounter(&tnow);
    tacum.QuadPart = (tnow.QuadPart - tini.QuadPart);

    if(restart)
      tini.QuadPart = tnow.QuadPart;

    return (int) tacum.QuadPart / ((int)freq.QuadPart / 1000);

  #else
    timeval tnow;
    gettimeofday (&tnow, 0);

    long int sec = tnow.tv_sec - tini.tv_sec;
    long int usec = tnow.tv_usec - tini.tv_usec;
    if (usec < 0) {
      sec --;
      usec += 1000000u;
    }
    if(restart)
      tini = tnow;

    return (int) sec * 1000.0f + (int) usec / 1000.0f;

  #endif
  }
};

}//namespace

#endif //ifdef QT_CORE_LIB
#endif //ifndef _CHRONOMETER_H_
