#ifndef _ATOMIC_INT_GENERIC_H

#define _ATOMIC_INT_GENERIC_H

#include "mt.h"

namespace mt{

class atomicInt
{
public:
  atomicInt()
  {
    _q_value = 0;
  }

  atomicInt( int value )
  {
      _q_value = value;
  }

// atomic API

/**
Reads the current _q_value of this QAtomicInt and then adds valueToAdd
to the current _q_value, returning the original _q_value.
*/
  inline int fetchAndAddAcquire( int valueToAdd )
  {
    mutexlocker lock(&m);
    int originalValue = _q_value;
    _q_value += valueToAdd;
    return originalValue;
  }

/**
Atomically increments the _q_value of this atomicInt.
Returns true if the new _q_value is non-zero, false otherwise.*/
  inline bool ref()
  {
    mutexlocker lock(&m);
    return ++_q_value != 0;
  }

/*
Atomically decrements the _q_value of this QAtomicInt.
Returns true if the new _q_value is non-zero, false otherwise.*/
  inline bool deref()
  {
    mutexlocker lock(&m);
    return --_q_value != 0;
  }

/*
If the current _q_value of this QAtomicInt is the expectedValue,
the test-and-set functions assign the newValue to this QAtomicInt
and return true. If the values are not the same, this function
does nothing and returns false.
*/
  inline bool testAndSetOrdered(int expectedValue, int newValue)
  {
    mutexlocker lock(&m);
    if (_q_value == expectedValue) {
      _q_value = newValue;
      return true;
    }
    return false;
  }

// Non-atomic API
  inline bool operator==(int value) const
  {
    return _q_value == value;
  }

  inline bool operator!=(int value) const
  {
    return _q_value != value;
  }

  inline bool operator!() const
  {
    return _q_value == 0;
  }

  inline operator int() const
  {
    return _q_value;
  }

  inline atomicInt &operator=(int value)
  {
    _q_value = value;
    return *this;
  }

  inline bool operator>(int value) const
  {
    return _q_value > value;
  }

  inline bool operator<(int value) const
  {
    return _q_value < value;
  }

private:
  volatile int _q_value;
  mutex m;
};

}//namespace

#endif

