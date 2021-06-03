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


#ifndef OPEN_WALKER_CORE_SUPPORT_TEMPLATES_H
#define OPEN_WALKER_CORE_SUPPORT_TEMPLATES_H

/*! \file support_templates.h
 *  \brief Contains support templates.
 *
 * Mostly inspired by Eigen.
 */


// the namespace for the project
namespace ow
{

template <class T> struct remove_const { typedef T type; };
template <class T> struct remove_const<const T> { typedef T type; };

template <typename T> struct add_const { typedef const T type; };
template <typename T> struct add_const<T&> { typedef T& type; };

template <typename T> struct is_const { enum { value = 0 }; };
template <typename T> struct is_const<T const> { enum { value = 1 }; };

template<bool Condition, typename Then, typename Else>
struct conditional { typedef Then type; };

template<typename Then, typename Else>
struct conditional <false, Then, Else> { typedef Else type; };



template<typename T> struct traits;

// here we say once and for all that traits<const T> == traits<T>
// When constness must affect traits, it has to be constness on template
//  parameters on which T itself depends.
// For example, traits<Map<const T> > != traits<Map<T> >, but
//              traits<const Map<T> > == traits<Map<T> >
template<typename T> struct traits<const T> : traits<T> {};

}

#endif // OPEN_WALKER_CORE_SUPPORT_TEMPLATES_H
