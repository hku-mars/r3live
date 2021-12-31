#ifndef __COMMON_TOOLS_H__
#define __COMMON_TOOLS_H__

#include <stdio.h>
#include <stdlib.h>
#include <future>
#include <vector>
#include <set>
#include <list>
#include <thread>
#include "tools_logger.hpp"
#include "tools_timer.hpp"
#include "tools_random.hpp"
#include "tools_json.hpp"
#include "os_compatible.hpp"

namespace Common_tools
{
    template <typename T>
    static  std::vector<T>  vector_2d_to_1d(std::vector<std::vector<T>> & pt_vec_vec)
    {
        std::vector<T> pt_vec;
        for (unsigned int i = 0; i < pt_vec_vec.size(); i++)
        {
            pt_vec.insert(pt_vec.end(), pt_vec_vec[i].begin(), pt_vec_vec[i].end());
        }
        return pt_vec;
    };

    template<typename T>
    std::vector<T> std_set_to_vector (const std::set<T> & input_set)
    {
        std::vector<T> output_vector;
        for(auto it = input_set.begin() ; it!= input_set.end(); it++)
        {
            output_vector.push_back(*(it));
        }
        return output_vector;
    };

    template<typename T>
    std::set<T> std_vector_to_set (const std::vector<T> & input_vector)
    {
        std::set<T> output_set;
        for(auto it = input_vector.begin() ; it!= input_vector.end(); it++)
        {
            output_set.insert(*(it));
        }
        return output_set;
    };

    template <typename T>
    void maintain_maximum_thread_pool(std::list<T> & thread_pool, size_t maximum_parallel_thread)
    {
      if(thread_pool.size() >= (size_t) maximum_parallel_thread)
      {
          //printf("******** Maximum thread number reach, current size = %d, wait finish... *****\r\n", thread_pool.size());
          while ( 1 )
          {
              //printf_line;
              for ( auto it = thread_pool.begin(); it != thread_pool.end(); it++ )
              {

                  auto status = (*it)->wait_for(std::chrono::nanoseconds(1));
                  if ( status == std::future_status::ready )
                  {
                      //cout << "status == std::future_status::ready" << endl;
                      delete *it;
                      thread_pool.erase( it );
                      break;
                  }
              }
              if(thread_pool.size() < (size_t) maximum_parallel_thread)
              {
                break;
              }
              std::this_thread::sleep_for( std::chrono::nanoseconds( 1 ) );
          }

      }

    };

    inline bool if_file_exist( const std::string &name )
    {
        //Copy from: https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exist-using-standard-c-c11-c
        struct stat buffer;
        return ( stat( name.c_str(), &buffer ) == 0 );
    };
    }
#endif
