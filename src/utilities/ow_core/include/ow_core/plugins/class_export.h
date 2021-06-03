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


#ifndef OPEN_WALKER_CORE_PLUGINS_CLASS_EXPORT_H
#define OPEN_WALKER_CORE_PLUGINS_CLASS_EXPORT_H

/*!
 *  \file class_export.h
 *  \brief Contains the macros for classes to load them on runtime.
 *
 *  This macros have to be added to the .cpp file of the class that
 *  will be loaded at runtime. Make sure that you have only one loadable class
 *  with this macros per .so file.
 */


/*! \def OW_PLUGIN_CLASS_CREATE(Class)
    \brief A macro that creates the create factory function for the \a Class.

 * \note The static_cast from 'Class*' to 'IGenericClass*' is
 * essential. The void* is casted back with reinterpret_cast to
 * 'IGenericClass*'. This fails with seg fault when not previously
 * using static_cast.
 *
 * \note The upcasting from 'IGenericClass*' to 'Class*' must
 * be again done with static_cast!
 *
 * \note Use this macro only once per .so file!
 */
#define OW_PLUGIN_CLASS_CREATE(Class) \
extern "C" \
{ \
  void* create() \
  { \
    Class* ptr = new Class; \
    return static_cast<ow_core::IGenericClass*>(ptr); \
  } \
}


/*! \def OW_PLUGIN_CLASS_DESTROY(Class)
 *  \brief A macro that creates the destroy factory function for the \a Class.
 *
 * \note The destruction should run from top to down
 * (should be less error prone than down to top).
 *
 * \note Since classes should be created by the factory they
 * should also be deleted by the factory to avoid memory
 * corruptions when the .so file has been compiled with
 * a different compiler (version) then the loader
 * of the .so file.
 *
 * \note Use this macro only once per .so file!
 */
#define OW_PLUGIN_CLASS_DESTROY(Class) \
extern "C" \
{ \
  void destroy(void* ptr) \
  { \
    ow_core::IGenericClass* g = \
      reinterpret_cast<ow_core::IGenericClass*>(ptr); \
    Class* c = static_cast<Class*>(g); \
    delete c; \
  } \
}

/*! \def OW_PLUGIN_CLASS(Class)
 * \brief A macro that creates the create and destroy factory functions
 * for the \a Class.
 *
 * \note Use this macro only once per .so file!
 */
#define OW_PLUGIN_CLASS(Class) \
  OW_PLUGIN_CLASS_CREATE(Class) \
  OW_PLUGIN_CLASS_CREATE(Class)


#endif // OPEN_WALKER_CORE_PLUGINS_CLASS_EXPORT_H
