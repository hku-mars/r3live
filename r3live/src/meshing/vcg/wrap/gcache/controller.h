#ifndef GCACHE_CONTROLLER_H
#define GCACHE_CONTROLLER_H

#include "cache.h"

/** Allows to insert tokens, update priorities and generally control the cache.
*/
namespace vcg {

template <class Token>
class Controller {
 public:
  ///tokens waiting to be added, should be private
  std::vector<Token *> tokens;
  /// threads still running, but no door is open in caches,
  ///transfers might still be going on!
  bool paused;
  ///all cache threads are stopped
  bool stopped;

 public:
  ///should be protected
  Provider<Token> provider;
  ///should be protected
  std::vector<Cache<Token> *> caches;

  Controller(): paused(false), stopped(true) {}
  ~Controller() { if(!stopped) finish(); }

  ///called before the cache is started to add a cache in the chain
  /** The order in which the caches are added is from the lowest to the highest. */
  void addCache(Cache<Token> *cache) {
    if(caches.size() == 0)
      cache->setInputCache(&provider);
    else
      cache->setInputCache(caches.back());
    assert(cache->input);
    caches.push_back(cache);
  }

  ///insert the token in the cache if not already present (actual insertion is done on updatePriorities)
  bool addToken(Token *token) {
    if(token->count.testAndSetOrdered(Token::OUTSIDE, Token::CACHE)) {
      tokens.push_back(token);
      return true;
    }
    return false;
  }

  ///WARNING: migh stall for the time needed to drop tokens from cache.
  //FUNCTOR has bool operator(Token *) and return true to remove
  template<class FUNCTOR> void removeTokens(FUNCTOR functor) {
    pause(); //this might actually be unnecessary if you mark tokens to be removed
    for(int i = (int)caches.size()-1; i >= 0; i--)
      caches[i]->flush(functor);
    provider.flush(functor);

    resume();
  }

  ///if more tokens than m present in the provider, lowest priority ones will be removed
  void setMaxTokens(int m) {
    mt::mutexlocker l(&provider.heap_lock);
    provider.max_tokens = m;
  }

  ///ensure that added tokens are processed and existing ones have their priority updated.
  ///potential bug! update is done on the heaps, if something is in transit...
  void updatePriorities() {
    if(tokens.size()) {
      mt::mutexlocker l(&provider.heap_lock);
      for(unsigned int i = 0; i < tokens.size(); i++)
        provider.heap.push(tokens[i]);
      tokens.clear();
    }

    provider.pushPriorities();
    for(unsigned int i = 0; i < caches.size(); i++)
      caches[i]->pushPriorities();
  }

  ///start the various cache threads.
  void start() {
    assert(stopped);
    assert(!paused);
    assert(caches.size() > 1);
    caches.back()->final = true;
    for(unsigned int i = 0; i < caches.size(); i++) //cache 0 is a provider, and his thread is not running.
      caches[i]->start();
    stopped = false;
  }

  ///stops the cache threads
  void stop() {
    if(stopped) return;
    assert(!paused);

    //signal al caches to quit
    for(unsigned int i = 0; i < caches.size(); i++)
      caches[i]->quit = true;

    //abort current gets
    for(unsigned int i = 0; i < caches.size(); i++)
      caches[i]->abort();

    //make sure all caches actually run a cycle.
    for(unsigned int i = 0; i < caches.size(); i++)
      caches[i]->input->check_queue.open();

    for(unsigned int i = 0; i < caches.size(); i++)
      caches[i]->wait();

    stopped = true;
  }

  void finish() {
    stop();
    flush();
  }

  void pause() {
    assert(!stopped);
    assert(!paused);

    //lock all doors.
    for(unsigned int i = 0; i < caches.size(); i++)
      caches[i]->input->check_queue.lock();

    //abort all pending calls
    for(unsigned int i = 0; i < caches.size(); i++)
      caches[i]->abort();

    //make sure no cache is running (must be done after abort! otherwise we have to wait for the get)
    for(unsigned int i = 0; i < caches.size()-1; i++)
      caches[i]->input->check_queue.room.lock();

    paused = true;
  }

  void resume() {
    assert(!stopped);
    assert(paused);
    cout << "Resume" << endl;

    //unlock and open all doors
    for(unsigned int i = 0; i < caches.size(); i++) {
      caches[i]->input->check_queue.unlock();
      caches[i]->input->check_queue.open();
    }

    //allow all cache to enter again.
    for(unsigned int i = 0; i < caches.size()-1; i++)
      caches[i]->input->check_queue.room.unlock();

    paused = false;
  }
  ///empty all caches AND REMOVES ALL TOKENS!
  void flush() {
    for(int i = (int)caches.size()-1; i >= 0; i--)
      caches[i]->flush();
    provider.heap.clear();
  }

  bool newData() {
    bool c = false;
    for(int i = (int)caches.size() -1; i >= 0; i--) {
      c |= caches[i]->newData();
    }
    return c;
  }

  bool isWaiting() {
    bool waiting = true;
    for(int i = (int)caches.size() -1; i >= 0; i--) {
      waiting &= caches[i]->input->check_queue.isWaiting();
    }
    return waiting;
  }
};

} //namespace
#endif // CONTROLLER_H
