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


#ifndef OPEN_WALKER_CORE_VECTOR_BASE_H
#define OPEN_WALKER_CORE_VECTOR_BASE_H

#include <Eigen/Dense>
#include <ow_core/utilities/forward_declarations.h>

#include <ow_msgs/Vector.h>

namespace ow_core
{
/*!
 * \brief The VectorBase class.
 */
template <typename _Derived>
class VectorBase
{
public:
  typedef _Derived Derived;

public:

  /*!
   * \brief Get accces to top level class.
   */
  Derived& derived()
  {
    return *static_cast<Derived*>(this);
  }

  /*!
   * \brief Get accces to top level class.
   */
  const Derived& derived() const
  {
    return *static_cast<const Derived*>(this);
  }

  /*!
   * \brief Assignment of ow_msgs::Vector.
   */
  VectorBase& operator=(const ow_msgs::Vector& vector)
  {
    vectorMsgToEigen(vector ,derived());
    return *this;
  }

  /*!
   * \brief Conversion to ow_msgs::Vector.
   */
  operator ow_msgs::Vector() const
  {
    ow_msgs::Vector vector;
    eigenToVectorMsg(derived(), vector);
    return vector;
  }

  /*!
   * \brief Conversion to ow_msgs::Vector.
   */
  ow_msgs::Vector toVectorMsg() const
  {
    return static_cast<ow_msgs::Vector>(derived());
  }

  /*!
   * \brief Assignment of std::vector<double>..
   */
  VectorBase& operator=(const std::vector<double>& vector)
  {
    stdVectorToEigen(vector, derived());
    return *this;
  }

  /*!
   * \brief Conversion to std::vector<double>.
   */
  operator std::vector<double>() const
  {
    std::vector<double> vector;
    eigenToStdVector(derived(), vector);
    return vector;
  }

  /*!
   * \brief Conversion to std::vector<double>.
   */
  std::vector<double> toStdVector() const
  {
    return static_cast<std::vector<double> >(derived());
  }

  /*!
   * \brief Conversion to std::string.
   */
  std::string toString() const
  {
    std::ostringstream out;
    out << derived().transpose();
    return out.str();
  }

};

}  // namespace ow_core

#endif  // OPEN_WALKER_CORE_VECTOR_REF_H
