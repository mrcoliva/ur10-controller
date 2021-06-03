/*! \file
 *
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author J. Rogelio Guadarrama-Olvera
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 14.02.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#ifndef OPEN_WALKER_CORE_THREAD_H_
#define OPEN_WALKER_CORE_THREAD_H_

#include <ow_core/utilities/type_not_assignable.h>

#include <thread>
#include <sys/syscall.h>
#include <sys/types.h>

/*! \file thread.h
 *  \brief Contains Thread Wrapper interface
 */

#include <iostream>

namespace ow
{

  /**
   * @brief The thread class wrappes a std::thread object.
   * 
   * This Class can be used as a base class to execute code in another thread.
   * After the start() is called, the run() function will be executed in
   * a new thread.
   */
  class Thread
  {
    OW_TYPE_NOT_ASSIGNABLE(Thread)

  protected:
    std::thread thread;
    bool started_;

  public:
    /**
     * @brief Construct a new Thread object
     * 
     */
    Thread() : started_(false)
    {
    }

    /**
     * @brief Destroy the Thread object
     * 
     */
    virtual ~Thread()
    {
      if (thread.joinable())
      {
        thread.join();
      }
    }

    /**
     * @brief returns the thread id
     * 
     * @return std::thread::id 
     */
    std::thread::id getID()
    {
      return thread.get_id();
    }

    /**
     * @brief checks if the thread is stopped
     * 
     * @return true 
     * @return false 
     */
    bool isFinished()
    {
      return !started_;
    }

    /**
     * @brief checks if the thread is running
     * 
     * @return true 
     * @return false 
     */
    bool isRunning()
    {
      return started_;
    }

    /**
     * @brief start the thread
     * 
     * @return true 
     * @return false 
     */
    bool start()
    {
      if (!isRunning())
      {
        thread = std::thread(&Thread::runInternal, this);
        return true;
      }
      return false;
    }

  private:
    virtual void runInternal()
    {
      started_ = true;
      run();
      started_ = false;
    }

  protected:
    /*!
    * @brief The thread execution function.
    */
    virtual void run() = 0;
  };

} // namespace ow

#endif // OPEN_WALKER_CORE_THREAD_H_