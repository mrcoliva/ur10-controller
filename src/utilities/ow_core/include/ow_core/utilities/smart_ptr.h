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

#ifndef OPEN_WALKER_CORE_SMART_POINTER_H
#define OPEN_WALKER_CORE_SMART_POINTER_H

/*! \file smart_ptr.h
 *  \brief Contains smart pointer functions.
 */
namespace ow
{
  /**
   * @brief make a unique ptr function needed for c++11 standard
   * 
   * @tparam T
   * @tparam Args 
   * @param args 
   * @return std::unique_ptr<T> 
   */
  template <typename T, typename... Args>
  std::unique_ptr<T> make_unique(Args &&... args)
  {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
  }

} // namespace ow

#endif // OPEN_WALKER_CORE_SMART_POINTER_H