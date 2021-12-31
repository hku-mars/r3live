#ifndef GCACHE_TOKEN_H
#define GCACHE_TOKEN_H

#include <wrap/system/multithreading/atomic_int.h>

/* QAtomic int count keep trak of token status:
   >0: locked (possibly multiple times)
   0: data ready in last cache
   -1: not in last cache
   -2: to removed from all caches
   -3: out of caches
   */

/** Holds the resources to be cached.
    The Priority template argument can simply be a floating point number
    or something more complex, (frame and error in pixel); the only
    requirement is the existence of a < comparison operator */
namespace vcg {

template <typename Priority>
class Token {
public:
    ///Resource loading status
    /*** - LOCKED: resource in the higher cache and locked
       - READY: resource in the higher cache
       - CACHE: resource in some cache (not the highest)
       - REMOVE: resource in some cache and scheduled for removal
       - OUTSIDE: resource not in the cache system */
    enum Status { LOCKED = 1, READY = 0, CACHE = -1, REMOVE = -2, OUTSIDE = -3 };
    ///Do not access these members directly. Will be moved to private shortly.
    ///used by various cache threads to sort objects [do not use, should be private]
    Priority priority;
    ///set in the main thread   [do not use, should be private]
    Priority new_priority;
    ///swap space used in updatePriorities [do not use, should be private]
    mt::atomicInt count;

public:
    Token(): count(OUTSIDE) {}

    ///the new priority will be effective only after a call to Controller::updatePriorities()
    void setPriority(const Priority &p) {
        new_priority = p;
    }

    //set and get are safe to call in the controller thread.
    Priority getPriority() {
        return new_priority;
    }

    ///return false if resource not in highest query. remember to unlock when done
    bool lock() {
        if(count.fetchAndAddAcquire(1) >= 0) return true;
        count.deref();
        return false;
    }

    ///assumes it was locked first and 1 unlock for each lock.
    bool unlock() {
        return count.deref();
    }

    ///can't be removed if locked and will return false
    bool remove() {
        count.testAndSetOrdered(READY, REMOVE);
        count.testAndSetOrdered(CACHE, REMOVE);
#if(QT_VERSION < 0x050000)
        return count <= REMOVE;
#else
        return count.load() <= REMOVE;                   //might have become OUSIDE in the meanwhile
#endif
    }

    bool isLocked() {
#if(QT_VERSION < 0x050000)
        return count > 0;
#else
        return count.load() > 0;
#endif
    }
    bool isInCache() { return count != OUTSIDE; }  //careful, can be used only when provider thread is locked.

    ///copy priority to swap space [do not use, should be private]
    void pushPriority() {
        priority = new_priority;
    }

    bool operator<(const Token &a) const {
#if(QT_VERSION < 0x050000)
        if(count == a.count)
            return priority < a.priority;
        return count < a.count;
#else
        if(count.load() == a.count.load())
            return priority < a.priority;
        return count.load() < a.count.load();
#endif
    }
    bool operator>(const Token &a) const {
#if(QT_VERSION < 0x050000)
        if(count == a.count)
            return priority > a.priority;
        return count > a.count;
#else
        if(count.load() == a.count.load())
            return priority > a.priority;
        return count.load() > a.count.load();
#endif
    }
};

} //namespace
#endif // GCACHE_H
