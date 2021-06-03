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


#ifndef OPEN_WALKER_CORE_Force_TORQUE_SENSOR_H
#define OPEN_WALKER_CORE_Force_TORQUE_SENSOR_H

#include <ow_core/types/wrench.h>

namespace ow_core{

/*!
 * \brief The ForceTorqueSensor class.
 *
 *  This class is a container for:
 *    - Wrench measured by the sensor
 *    - Offset Wrench
 */
template <typename _Scalar>
class ForceTorqueSensor 
{
public:
  typedef _Scalar Scalar;
  typedef Wrench<Scalar> ForceTorque;

  /*!
   * \brief Construct as Zero.
   */
  static const ForceTorqueSensor& Zero()
  {
    static const ForceTorqueSensor v(ForceTorque::Zero(),"");
    return v;
  }

  /*!
   * \brief Construct as Default
   */
  static const ForceTorqueSensor& Default()
  {
    static const ForceTorqueSensor v(ForceTorque::Zero(), "");
    return v;
  }

protected:
  ForceTorque W_;
  std::string name_;
  ForceTorque W_offset_;

public:
  /*!
    * \brief Default Constructor.
    */
  ForceTorqueSensor()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  ForceTorqueSensor(const ForceTorqueSensor& other) :
    W_(other.W_),
    name_(other.name_),
    W_offset_(other.W_offset_)
  {
  }

  /*!
    * \brief Constructor from sub elements
   */
  explicit ForceTorqueSensor(
    const ForceTorque& W,
    const std::string& name = "",
    const ForceTorque& W_offset = ForceTorque::Zero()) :
    W_(W),
    name_(name),
    W_offset_(W_offset)
  {
  }

  ForceTorque& W()
  {
    return W_;
  }

  ForceTorque& wrench()
  {
    return W_;
  }

  const ForceTorque& W() const
  {
    return W_;
  }

  const ForceTorque& wrench() const
  {
    return W_;
  }

  std::string& name()
  {
    return name_;
  }

  const std::string& name() const
  {
    return name_;
  }

  ForceTorque& WOffset()
  {
    return W_offset_;
  }

  ForceTorque& wrenchOffset()
  {
    return W_offset_;
  }

  const ForceTorque& WOffset() const
  {
    return W_offset_;
  }

  const ForceTorque& wrenchOffset() const
  {
    return W_offset_;
  }

};

}

#endif // OPEN_WALKER_CORE_Force_TORQUE_SENSOR_H
