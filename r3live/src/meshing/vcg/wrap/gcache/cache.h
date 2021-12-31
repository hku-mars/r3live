#ifndef GCACHE_CACHE_H
#define GCACHE_CACHE_H

#ifdef _MSC_VER

typedef __int16 int16_t;
typedef unsigned __int16 uint16_t;
typedef __int32 int32_t;
typedef unsigned __int32 uint32_t;
typedef __int64 int64_t;
typedef unsigned __int64 uint64_t;

#else
#include <stdint.h>
#endif

#include <iostream>
#include <limits.h>
#include <vector>
#include <list>

#include "token.h"

#include <wrap/system/multithreading/mt.h>
#include <wrap/system/multithreading/atomic_int.h>

#include "provider.h"

using namespace std;
/* this cache system enforce the rule that the items in a cache are always in all the cache below */
/* two mechanism to remove tokens from the cache:
      1) set token count to something low
      2) set maximum number of tokens in the provider
*/

/** Cache virtual base class. You are required to implement the pure virtual functions get, drop and size.
*/

namespace vcg {

template <typename Token> class Transfer;

template <typename Token>
class Cache: public Provider<Token> {

public:
    ///true if this is the last cache (the one we use the data from)
    bool final;
    //if true the cache will exit at the first opportunity
    bool quit;
    ///keeps track of changes (if 1 then something was loaded or dropped
    mt::atomicInt new_data;
    ///callback for new_data
    void (*callback)(void *data);

    ///data is fetched from here
    Provider<Token> *input;

    ///threads running over cache...
    std::vector<Transfer<Token> *> transfers;

protected:
    ///max space available
    uint64_t s_max;
    ///current space used
    uint64_t s_curr;

public:
    Cache(uint64_t _capacity = INT_MAX):
        final(false), quit(false), new_data(false), input(NULL), s_max(_capacity), s_curr(0) {}
    virtual ~Cache() {}

    void setInputCache(Provider<Token> *p) { input = p; }
    uint64_t capacity() { return s_max; }
    uint64_t size() { return s_curr; }
    void setCapacity(uint64_t c) { s_max = c; }

    ///return true if the cache is waiting for priority to change
    bool newData() {
        bool r = new_data.testAndSetOrdered(1, 0); //if changed is 1, r is true
        return r;
    }

    ///empty the cache. Make sure no resource is locked before calling this.
    /// Require pause or stop before. Ensure there no locked item
    void flush() {
        //std::vector<Token *> tokens;
        {
            for(int i = 0; i < this->heap.size(); i++) {
                Token *token = &(this->heap[i]);
                //tokens.push_back(token);
                s_curr -= drop(token);
                //assert(!(token->count.load() >= Token::LOCKED));
                if(final)
                    token->count.testAndSetOrdered(Token::READY, Token::CACHE);
                input->heap.push(token);
            }
            this->heap.clear();
        }
        if(!s_curr == 0) {
            std::cerr << "Cache size after flush is not ZERO!\n";
            s_curr = 0;
        }
    }

    ///empty the cache. Make sure no resource is locked before calling this.
    /// Require pause or stop before. Ensure there no locked item
    template <class FUNCTOR> void flush(FUNCTOR functor) {
        std::vector<Token *> tokens;
        {
            int count = 0;
            mt::mutexlocker locker(&(this->heap_lock));
            for(int k = 0; k < this->heap.size(); k++) {
                Token *token = &this->heap[k];
                if(functor(token)) { //drop it
                    tokens.push_back(token);
                    s_curr -= drop(token);
                    //assert(token->count.load() < Token::LOCKED);
                    if(final)
                        token->count.testAndSetOrdered(Token::READY, Token::CACHE);
                } else
                    this->heap.at(count++) = token;
            }
            this->heap.resize(count);
            this->heap_dirty = true;
        }
        {
            mt::mutexlocker locker(&(input->heap_lock));
            for(unsigned int i = 0; i < tokens.size(); i++) {
                input->heap.push(tokens[i]);
            }
        }
    }

    virtual void abort() {}

protected:
    ///return the space used in the cache by the loaded resource
    virtual int size(Token *token) = 0;
    ///returns amount of space used in cache -1 for failed transfer
    virtual int get(Token *token) = 0;
    ///return amount removed
    virtual int drop(Token *token) = 0;
    ///make sure the get function do not access token after abort is returned.




    ///called in as first thing in run()
    virtual void begin() {}
    virtual void middle() {}
    ///called in as last thing in run()
    virtual void end() {}

    ///[should be protected]
    void run() {
        assert(input);
        /* basic operation of the cache:
          1) make room until eliminating an element would leave empty space.
          2) transfer first element of input_cache if
          cache has room OR first element in input has higher priority of last element */
        begin();
        while(!this->quit) {
            input->check_queue.enter();     //wait for cache below to load something or priorities to change
            if(this->quit) break;

            middle();

            if(unload() || load()) {
                new_data.testAndSetOrdered(0, 1);  //if not changed, set as changed
                input->check_queue.open();        //we signal ourselves to check again
                cout << "loaded or unloaded\n";
            }
            input->check_queue.leave();
        }
        this->quit = false;                   //in case someone wants to restart;
        end();
    }



    /** Checks wether we need to make room in the cache because of:
     size() - sizeof(lowest priority item) > capacity()
  **/
    bool unload() {
        Token *remove = NULL;
        //make room int the cache checking that:
        //1 we need to make room (capacity < current)
        if(size() > capacity()) {
            mt::mutexlocker locker(&(this->heap_lock));

            //2 we have some element not in the upper caches (heap.size()  > 0
            if(this->heap.size()) {
                Token &last = this->heap.min();
                int itemsize = size(&last);

                //3 after removing the item, we are still full (avoids bouncing items)
                if(size() - itemsize > capacity()) {

                    //4 item to remove is not locked. (only in last cache. you can't lock object otherwise)
                    if(!final) { //not final we can drop when we want
                        remove = this->heap.popMin();
                    } else {
                        last.count.testAndSetOrdered(Token::READY, Token::CACHE);
#if(QT_VERSION < 0x050000)
                int last_count = last.count;
#else
                int last_count = last.count.load();
#endif
                        if(last_count <= Token::CACHE) { //was not locked and now can't be locked, remove it.
                            remove = this->heap.popMin();
                        } else { //last item is locked need to reorder stack
                            remove = this->heap.popMin();
                            this->heap.push(remove);
                            cout << "Reordering stack something (what?)\n";
                            return true;
                        }
                    }
                }
            }
        }

        if(remove) {
            {
                mt::mutexlocker input_locker(&(input->heap_lock));
                int size = drop(remove);
                assert(size >= 0);
                s_curr -= size;
                input->heap.push(remove);
            }
            return true;
        }
        return false;
    }

    ///should be protected
    bool load() {
        Token *insert = NULL;
        Token *last = NULL;              //we want to lock only one heap at once to avoid deadlocks.

        /* check wether we have room (curr < capacity) or heap is empty.
       empty heap is bad: we cannot drop anything to make room, and cache above has nothing to get.
       this should not happen if we set correct cache sizes, but if it happens.... */
        {
            mt::mutexlocker locker(&(this->heap_lock));
            this->rebuild();
            if(size() > capacity() && this->heap.size() > 0) {
                last = &(this->heap.min()); //no room, set last so we might check for a swap.
            }
        }

        {
            mt::mutexlocker input_locker(&(input->heap_lock));
            input->rebuild();                                  //if dirty rebuild
            if(input->heap.size()) {                           //we need something in input to tranfer.
                Token &first = input->heap.max();
#if(QT_VERSION < 0x050000)
                int first_count = first.count;
#else
                int first_count = first.count.load();
#endif
                if(first_count > Token::REMOVE &&
                        (!last || first.priority > last->priority)) { //if !last we already decided we want a transfer., otherwise check for a swap
                    insert = input->heap.popMax();                 //remove item from heap, while we transfer it.
                }
            }
        }

        if(insert) {                                        //we want to fetch something

            int size = get(insert);

            if(size >= 0) {                                   //success
                s_curr += size;
                {
                    mt::mutexlocker locker(&(this->heap_lock));
                    if(final)
                        insert->count.ref();                       //now lock is 0 and can be locked

                    this->heap.push(insert);
                }
                this->check_queue.open();                      //we should signal the parent cache that we have a new item
                return true;

            } else {                                         //failed transfer put it back, we will keep trying to transfer it...
                mt::mutexlocker input_locker(&(input->heap_lock));
                input->heap.push(insert);
                return false;
            }
        }
        return false;
    }
};

} //namespace

#endif // GCACHE_H
