/* 
This code is the implementation of our paper "R3LIVE: A Robust, Real-time, RGB-colored, 
LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package".

Author: Jiarong Lin   < ziv.lin.ljr@gmail.com >

If you use any code of this repo in your academic research, please cite at least
one of our papers:
[1] Lin, Jiarong, and Fu Zhang. "R3LIVE: A Robust, Real-time, RGB-colored, 
    LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package." 
[2] Xu, Wei, et al. "Fast-lio2: Fast direct lidar-inertial odometry."
[3] Lin, Jiarong, et al. "R2LIVE: A Robust, Real-time, LiDAR-Inertial-Visual
     tightly-coupled state Estimator and mapping." 
[4] Xu, Wei, and Fu Zhang. "Fast-lio: A fast, robust lidar-inertial odometry 
    package by tightly-coupled iterated kalman filter."
[5] Cai, Yixi, Wei Xu, and Fu Zhang. "ikd-Tree: An Incremental KD Tree for 
    Robotic Applications."
[6] Lin, Jiarong, and Fu Zhang. "Loam-livox: A fast, robust, high-precision 
    LiDAR odometry and mapping package for LiDARs of small FoV."

For commercial use, please contact me < ziv.lin.ljr@gmail.com > and
Dr. Fu Zhang < fuzhang@hku.hk >.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from this
    software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/

// Tools for maintaining a multiple thread pool for parallel operations.
// Developer: Jiarong Lin <ziv.lin.ljr@gmail.com>
// Reference:
// [1] https://github.com/progschj/ThreadPool


#ifndef __THREAD_POOL_HPP__
#define __THREAD_POOL_HPP__

#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>
#include <sstream>
#include <atomic>
#include <mutex>
#include <unistd.h>
#include <sched.h>   //cpu_set_t , CPU_SET
#include <sys/resource.h>
#include <pthread.h> //pthread_t
// #include "tools_logger.hpp"
using std::cout;
using std::endl;

namespace Common_tools
{

class Process
{
public:
	enum Priority {
		IDLE = -3,
		LOW = -2,
		BELOWNORMAL = -1,
		NORMAL = 0,
		ABOVENORMAL = 1,
		HIGH = 2,
		REALTIME = 3
	};

	#ifdef _MSC_VER

	typedef HANDLE process_t;

	static inline process_t getCurrentProcessID() { return ::GetCurrentProcess(); }
	static inline void setProcessPriority(process_t id, Priority p) { ::SetPriorityClass(id, convertPriority(p)); }
	static inline Priority getProcessPriority(process_t id) { return convertPriority((PriorityOS)::GetPriorityClass(id)); }

	#else //_MSC_VER

	typedef id_t process_t;

	static inline process_t getCurrentProcessID() { return ::getpid(); }
	static inline void setProcessPriority(process_t id, Priority p) { ::setpriority(PRIO_PROCESS, id, convertPriority(p)); }
	static inline Priority getProcessPriority(process_t id) { return convertPriority((PriorityOS)::getpriority(PRIO_PROCESS, id)); }

	#endif //_MSC_VER

	static inline void setCurrentProcessPriority(Priority p) { setProcessPriority(getCurrentProcessID(), p); }
	static inline Priority getCurrentProcessPriority() { return getProcessPriority(getCurrentProcessID()); }

protected:

	#ifdef _MSC_VER

	enum PriorityOS {
		OS_IDLE = IDLE_PRIORITY_CLASS,
		OS_LOW = PROCESS_MODE_BACKGROUND_BEGIN,
		OS_BELOWNORMAL = BELOW_NORMAL_PRIORITY_CLASS,
		OS_NORMAL = NORMAL_PRIORITY_CLASS,
		OS_ABOVENORMAL = ABOVE_NORMAL_PRIORITY_CLASS,
		OS_HIGH = HIGH_PRIORITY_CLASS,
		OS_REALTIME = REALTIME_PRIORITY_CLASS
	};

	#else //_MSC_VER

	enum PriorityOS {
		OS_IDLE = 19,
		OS_LOW = 15,
		OS_BELOWNORMAL = 10,
		OS_NORMAL = 0,
		OS_ABOVENORMAL = -10,
		OS_HIGH = -15,
		OS_REALTIME = -20
	};

	#endif //_MSC_VER

	static inline PriorityOS convertPriority(Priority p) {
		switch (p) {
		case IDLE:				return OS_IDLE;
		case LOW:				return OS_LOW;
		case BELOWNORMAL:		return OS_BELOWNORMAL;
		case NORMAL:			return OS_NORMAL;
		case ABOVENORMAL:		return OS_ABOVENORMAL;
		case HIGH:				return OS_HIGH;
		case REALTIME:			return OS_REALTIME;
		}
		return OS_NORMAL;
	}
	static inline Priority convertPriority(PriorityOS p) {
		switch (p) {
		case OS_IDLE:			return IDLE;
		case OS_LOW:			return LOW;
		case OS_BELOWNORMAL:	return BELOWNORMAL;
		case OS_NORMAL:			return NORMAL;
		case OS_ABOVENORMAL:	return ABOVENORMAL;
		case OS_HIGH:			return HIGH;
		case OS_REALTIME:		return REALTIME;
		}
		return NORMAL;
	}
};
/*----------------------------------------------------------------*/


    static uint64_t get_thread_id()
    {
        //https://stackoverflow.com/questions/7432100/how-to-get-integer-thread-id-in-c11
        static_assert(sizeof(std::thread::id) == sizeof(uint64_t), "this function only works if size of thead::id is equal to the size of uint_64");
        auto id = std::this_thread::get_id();
        uint64_t *ptr = (uint64_t *)&id;
        return (*ptr);
    }

    static void set_thread_as_highest_priority()
    {
        pthread_t      thId = pthread_self();
        pthread_attr_t thAttr;
        int            policy = 0;
        int            max_prio_for_policy = 0;

        pthread_attr_init( &thAttr );
        pthread_attr_getschedpolicy( &thAttr, &policy );
        max_prio_for_policy = sched_get_priority_max( policy );

        pthread_setschedprio( thId, max_prio_for_policy );
        pthread_attr_destroy( &thAttr );
    }

    class ThreadPool
    {
    public:
        ThreadPool(size_t threads = 0, bool if_set_cpu_affinity = true, bool if_set_highest_priority = true);
        template <class F, class... Args>
        auto commit_task(F &&f, Args &&...args)
            -> std::future<typename std::result_of<F(Args...)>::type>;
        ~ThreadPool();
        std::atomic<int> thread_on_cpu_count;
    private:
        // need to keep track of threads so we can join them
        std::vector<std::thread> workers;
        // the task queue
        std::queue<std::function<void()>> tasks;

        // synchronization
        std::mutex queue_mutex;
        std::condition_variable condition;
        bool stop;
    };

    // the constructor just launches some amount of workers
    inline ThreadPool::ThreadPool(size_t threads, bool if_set_cpu_affinity, bool if_set_highest_priority)
        : stop(false)
    {
        thread_on_cpu_count = 0;
        int number_of_cpus = std::thread::hardware_concurrency();
        if(threads <= 0) 
        {
            threads = number_of_cpus;
        }
        // cout << "Number of processors: " << number_of_cpus << endl;
        if(threads <= 0 )
        {
            // threads = number_of_cpus;
            threads = 10;
        }
        for (size_t i = 0; i < threads; ++i)
        {
            workers.emplace_back(
                [this, number_of_cpus, if_set_highest_priority]
                {
                    cpu_set_t cpuset;
                    //the CPU we want to use
                    int cpu_id = (thread_on_cpu_count++) % number_of_cpus;
                    CPU_ZERO(&cpuset);        //clears the cpuset
                    CPU_SET(cpu_id, &cpuset); //set CPU

                    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == -1)
                    {
                        printf("Warning!!!,Thread_id [%ld] set CPU-%d affinity Fail!!!\r\n", get_thread_id(), cpu_id);
                    }
                    else
                    {
                        printf("Thread_id [%ld] set CPU-%d affinity success.\r\n", get_thread_id(), cpu_id);
                    }
                    if (if_set_highest_priority)
                    {
                        Common_tools::Process::setCurrentProcessPriority(Common_tools::Process::Priority::REALTIME);
                        set_thread_as_highest_priority();
                    }
                    while(1)
                    {
                        std::function<void()> task;
                        {
                            std::unique_lock<std::mutex> lock(this->queue_mutex);
                            this->condition.wait(lock,
                                                 [this]
                                                 { return this->stop || !this->tasks.empty(); });
                            if (this->stop && this->tasks.empty())
                                return;
                            task = std::move(this->tasks.front());
                            this->tasks.pop();
                        }
                        task();
                        std::this_thread::yield();
                    }
                });
        }
    }

    // add new work item to the pool
    template <class F, class... Args>
    auto ThreadPool::commit_task(F && f, Args && ...args)->std::future<typename std::result_of<F(Args...)>::type>
    {
        using return_type = typename std::result_of<F(Args...)>::type;

        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...));

        std::future<return_type> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            // don't allow enqueueing after stopping the pool
            if (stop)
                throw std::runtime_error("enqueue on stopped ThreadPool");
            tasks.emplace([task]()
                          { (*task)(); });
        }
        // condition.notify_one();
        condition.notify_all();
        return res;
    }

    // the destructor joins all threads
    inline ThreadPool::~ThreadPool()
    {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true;
        }
        condition.notify_all();
        for (std::thread &worker : workers)
        {
            worker.join();
        }
    }
}
#endif