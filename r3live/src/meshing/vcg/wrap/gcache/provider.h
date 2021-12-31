#ifndef GCACHE_PROVIDER_H
#define GCACHE_PROVIDER_H

#include <wrap/system/multithreading/mt.h>
#include "dheap.h"
#include "door.h"

#include "token.h"

/* this cache system enforce the rule that the items in a cache are always in all the cache below */
/* two mechanism to remove tokens from the cache:
      1) set token count to something low
      2) set maximum number of tokens in the provider
*/

/** Base class for Cache and last cache in the GCache system.
    You should never interact with this class.
*/

namespace vcg {

template <typename Token>
class Provider: public mt::thread {
 public:
  ///holds the resources in this cache but not in the cache above
  PtrDHeap<Token> heap;
  ///tokens above this number will be scheduled for deletion
  int max_tokens;
  ///signals we need to rebuild heap.
  bool heap_dirty;
  ///lock this before manipulating heap.
  mt::mutex heap_lock;
  ///signals (to next cache!) priorities have changed or something is available
  QDoor check_queue;

  Provider(): max_tokens(-1), heap_dirty(false) {}
  virtual ~Provider() {}

  /// [should be protected, do not use] called in controller thread!
  void pushPriorities() {
    mt::mutexlocker locker(&heap_lock);
    for(int i = 0; i < heap.size(); i++)
      heap[i].pushPriority();

    heap_dirty = true;
    check_queue.open();
  }
  /// assumes heap lock is locked, runs in cache thread [should be protected, do not use]
  void rebuild() {
    if(!this->heap_dirty) return;

    this->heap.rebuild();
    this->heap_dirty = false;

    //remove OUTSIDE tokens from bottom of heap
    if(max_tokens != -1) {
      while(this->heap.size() > max_tokens) {
        Token &t = this->heap.min();
        t.count = Token::OUTSIDE;
        this->heap.popMin();
      }
    }
  }

  ///ensure no locked item are to be removed [should be protected, do not use]
  template <class FUNCTOR> void flush(FUNCTOR functor) {
    int count = 0;
    mt::mutexlocker locker(&(this->heap_lock));
    for(int k = 0; k < this->heap.size(); k++) {
      Token *token = &this->heap[k];
      if(functor(token)) { //drop it
        token->count = Token::OUTSIDE;
      } else
        this->heap.at(count++) = token;
    }
    this->heap.resize(count);
    this->heap_dirty = true;
  }
};

} //namespace
#endif
