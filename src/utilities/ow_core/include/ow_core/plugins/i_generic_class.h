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


#ifndef OPEN_WALKER_CORE_PLUGINS_I_GENERIC_CLASS_H
#define OPEN_WALKER_CORE_PLUGINS_I_GENERIC_CLASS_H

#include <string>

namespace ow_core
{

/*!
 * \brief The IGenericClass class.
 *
 * The interface class for dynamically created classes
 * from a runtime loaded shared library.
 *
 * \note If you want to create a class dynamically from a runtime
 * loaded shared library, then that class needs to have a virtual destructor.
 *
 * All OpenWalker plugins derive from this class to be loadable at
 * runtime.
 */
class IGenericClass
{
public:
  /*!
   * \brief The class type of this class.
   *
   * Since demangling and mangling are compiler specific we use
   * this function to store the type of a class.
   * This function should be overloaded in all classes that
   * derive from this class.
   */
  static const std::string& ClassType()
  {
    static const std::string s = "ow_core::IGenericClass";
    return s;
  }

public:
  /*!
   * \brief Virtual destructor.
   */
  virtual ~IGenericClass()
  {}

  /*!
   * \brief Get the class type.
   *
   * \note This identifier can be used to determine the class
   * that this interface abstracts. Then the pointer of this
   * class can be up casted to the correct type.
   *
   * \note This function should be overwritten by all classes
   * that derive from this class.
   */
  virtual std::string classType() const
  {
    return ClassType();
  }
};

} // namespace ow_core

#endif // OPEN_WALKER_CORE_PLUGINS_I_GENERIC_CLASS_H
