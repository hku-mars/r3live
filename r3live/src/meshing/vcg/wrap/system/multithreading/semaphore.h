#ifndef MT_SEMAPHORE_H
#define MT_SEMAPHORE_H

#include "base.h"

#include <iostream>
#include <semaphore.h>

namespace mt
{

class semaphore
{
  MT_PREVENT_COPY(semaphore)

  public:

  typedef semaphore this_type;
  typedef void      base_type;

  semaphore(void)
  {
    sem_init(&(this->s), 0, 0);
  }

  semaphore(int value)
  {
    sem_init(&(this->s), 0, value);
  }

  ~semaphore(void)
  {
    sem_destroy(&(this->s));
  }

  void post(void)
  {
    sem_post(&(this->s));
  }

  /*
  void post(int n)
  {
    sem_post_multiple(&(this->s), n);
  }
  */

  void wait(void)
  {
    sem_wait(&(this->s));
  }

  bool trywait(void)
  {
    return (sem_trywait(&(this->s)) == 0);
  }

  //methods added for conforming to the QT implementation
  //jnoguera 14-12-2011

  void release(int n=1)
  {
    if(n != 1)
      std::cout << "Error, mt::semaphore.release() not supported\n";
    sem_post(&(this->s));
  }

  void acquire(int n=1)
  {
    if(n != 1)
      std::cout << "Error, mt::semaphore.tryAcquire() not supported\n";
    sem_wait(&(this->s));
  }

  bool tryAcquire(int n=1)
  {
  if(n != 1)
    std::cout << "Error, mt::semaphore.tryAcquire() not supported\n";
  return (sem_trywait(&(this->s)) == 0);
  }

  int available()
  {
    int value;
    sem_getvalue( &(this->s), &value );
    return value;
  }

private:

sem_t s;
};

}

#endif // MT_SEMAPHORE_H
