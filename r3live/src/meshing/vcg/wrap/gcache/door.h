/****************************************************************************
* GCache                                                                    *
* Author: Federico Ponchio                                                  *
*                                                                           *
* Copyright(C) 2011                                                         *
* Visual Computing Lab                                                      *
* ISTI - Italian National Research Council                                  *
*                                                                           *
* All rights reserved.                                                      *
*                                                                           *
* This program is free software; you can redistribute it and/or modify      *
* it under the terms of the GNU General Public License as published by      *
* the Free Software Foundation; either version 2 of the License, or         *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
****************************************************************************/


#ifndef CACHE_DOOR_H
#define CACHE_DOOR_H

#include <wrap/system/multithreading/mt.h>
#include <wrap/system/multithreading/atomic_int.h>

#ifdef NEXUS_USE_QT
#include <QWaitCondition>
#endif

#define METHOD_2

#ifdef METHOD_1

class QDoor {
 private:
  mt::semaphore door;
  mt::mutex room;     //lock when entering. unlock when exiting
  QAtomicInt key; //keep tracks of door status

 public:
  QDoor():key(0) {}
  void open() {
    if(key.testAndSetOrdered(0, 1))
      door.release(1);
  }

  void enter() {
    door.acquire(1); //here I am sure that key is 1
    //if here a open appends will have no effect.
    key.testAndSetOrdered(1, 0);
    room.lock();
  }
  void leave() {
    room.unlock();
  }
  void lock() {
    int r = key.fetchAndStoreOrdered(-1);
    if(r == 1) //if the door was open
      door.tryAcquire(1); //might file if whe are between enter acquire and key = 0.
  }
  void unlock() {
    key = 0;
  }
};
#endif

#ifdef METHOD_2

//a door needs to be open for the thread to continue,
//if it is open the thread enter and closes the door
//this mess is to avoid [if(!open.available()) open.release(1)]

class QDoor {
 private:
  mt::semaphore _open;
  mt::semaphore _close;

 public:
  mt::mutex room;
  QDoor(): _open(0), _close(1) {} //this means closed

  void open() {
    if(_close.tryAcquire(1)) //check it is not open
      _open.release(1); //open
  }
  void close() {
    if(_open.tryAcquire(1)) //check not already closed
      _close.release(1);
  }
  void enter(bool close = false) {
    _open.acquire(1);
    if(close)
      _close.release(1); //close door behind
    else
      _open.release(1);  //leave door opened
   room.lock();
  }
  void leave() { room.unlock(); }

  void lock() {
    //door might be open or closed, but we might happen just in the middle
    //of someone opening, closing or entering it.
    while(!_open.tryAcquire(1) && !_close.tryAcquire(1)) {}
    //no resources left, door is locked
  }
  void unlock(bool open = false) {
    if(open)
      _open.release(1);
    else
      _close.release(1);
  }
  bool isWaiting() {
    if(_open.tryAcquire(1)) {
      _close.release(1);
      return false;
    }
    return true;
  }
};


#endif


#ifdef METHOD_3
/**
  A wait condition class that works as a door.
  Should check if the semaphore version is faster.
*/

class QDoor {
 public:

  QDoor(void) : doorOpen(false), waiting(false) {}

  ///opens the door. Threads trying to enter will be awakened
  void open(void) {
    m.lock();
    doorOpen = true;
    m.unlock();
    c.wakeAll(); arglebargle
  }

  ///attempt to enter the door. if the door is closed the thread will wait until the door is opened.
  /// if close is true, the door will be closed after the thread is awakened, this allows to
  ///   have only one thread entering the door each time open() is called
  void enter(bool close = false) {
    m.lock();
    waiting = true;
    while (!doorOpen)
      c.wait(&(m));

    if(close)
      doorOpen = false;
    waiting = false;
    m.unlock();
  }
  void leave() {}
  bool isWaiting() {
    m.lock();
    bool w = waiting;
    m.unlock();
    return w;
  }
  void lock() { //prevend door opening and entering
    m.lock();
  }
  void unlock(bool open = false) { //reverse effect of lock
    doorOpen = open;
    m.unlock();
  }
 private:
  mt::mutex m;
  QWaitCondition c;
  bool doorOpen;
  bool waiting;
};

#endif


#endif //CACHE_DOOR_H
