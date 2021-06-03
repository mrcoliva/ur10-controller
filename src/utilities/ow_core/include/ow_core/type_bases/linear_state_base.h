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


#ifndef OPEN_WALKER_CORE_LINEAR_STATE_BASE_H
#define OPEN_WALKER_CORE_LINEAR_STATE_BASE_H

#include <ow_core/type_bases/state_base.h>
#include <ow_msgs/LinearState.h>

namespace ow_core{

/*!
 * \brief The LinearStateBase class.
 * 
 * \note this Class simply passes _Derived through to the StateBase
 */
template <typename _Derived>
class LinearStateBase :
  public StateBase<_Derived>
{
public:
  typedef _Derived Derived;
  typedef StateBase<Derived> Base;

public:
  using Base::operator=;

  /*!
   * \brief Assignment of ow_msgs::LinearState.
   */
  LinearStateBase& operator=(const ow_msgs::LinearState& msg)
  {
    Base::derived().pos() = msg.position;
    Base::derived().vel() = msg.velocity;
    Base::derived().acc() = msg.acceleration;
    Base::derived().effort() = msg.force;
    return *this;
  }

  /*!
   * \brief Conversion to ow_msgs::LinearState.
   */
  operator ow_msgs::LinearState() const
  {
    ow_msgs::LinearState msg;
    msg.position = Base::derived().pos();
    msg.velocity = Base::derived().vel();
    msg.acceleration = Base::derived().acc();
    msg.force = Base::derived().effort();
    return msg;
  }  

  /*!
   * \brief Conversion to ow_msgs::LinearState.
   */
  ow_msgs::LinearState toLinearStateMsg() const
  {
    return static_cast<ow_msgs::LinearState>(Base::derived());
  }
};

}  // namespace ow_core

#endif  // OPEN_WALKER_CORE_LINEAR_STATE_BASE_H
