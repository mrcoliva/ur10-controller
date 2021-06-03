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


#ifndef OPEN_WALKER_CORE_CARTESIAN_STATE_H
#define OPEN_WALKER_CORE_CARTESIAN_STATE_H

#include <ow_core/types/cartesian_position.h>
#include <ow_core/types/cartesian_velocity.h>
#include <ow_core/types/cartesian_acceleration.h>
#include <ow_core/types/wrench.h>

#include <ow_core/type_bases/state_base.h>

#include <ow_msgs/CartesianState.h>

namespace ow{

/*!
 * \brief The traits class for the CartesianState class.
 *
 * This class contains the typedefs and enums for
 * the CartesianState class.
 *
 * These typedefs and enums are used to define and describe
 * the properties (traits) of this class.
 */
template<typename _Scalar>
struct traits<ow_core::CartesianState<_Scalar> >
{
  typedef _Scalar Scalar;
  typedef ow_core::CartesianPosition<Scalar> Pos;
  typedef ow_core::CartesianVelocity<Scalar> Vel;
  typedef ow_core::CartesianAcceleration<Scalar> Acc;
  typedef ow_core::Wrench<Scalar> Effort;
  enum
  {
    IsRef = 0,
  };
};

} // namespace ow

namespace ow_core{

/*!
 * \brief The CartesianState class.
 *
 *  This class is a container for:
 *    - CartesianPosition
 *    - CartesianVelocity
 *    - CartesianAcceleration
 *    - Wrench
 */
template<typename _Scalar>
class CartesianState :
  public StateBase<CartesianState<_Scalar> >
{
public:
  typedef _Scalar Scalar;
  typedef StateBase<CartesianState<Scalar> > Base;

  typedef CartesianPosition<Scalar> Pos;
  typedef CartesianVelocity<Scalar> Vel;
  typedef CartesianAcceleration<Scalar> Acc;
  typedef Wrench<Scalar> Effort;

protected:
  Pos X_;
  Vel XP_;
  Acc XPP_;
  Effort W_;

public:
  /*!
    * \brief Default Constructor.
    */
  CartesianState()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  CartesianState(const CartesianState& other) :
    X_(other.X_),
    XP_(other.XP_),
    XPP_(other.XPP_),
    W_(other.W_)
  {}

  /*!
   * \brief Copy constructor.
   */
  CartesianState(const Base& other) : 
    X_(other.pos()),
    XP_(other.vel()),
    XPP_(other.acc()),
    W_(other.effort())
  {
  }

  /*!
    * \brief Constructor from sub elements
    */
  explicit CartesianState(
    const Pos& X, 
    const Vel& XP = Vel::Zero(), 
    const Acc& XPP = Acc::Zero(), 
    const Effort& W = Effort::Zero()) : 
    X_(X),
    XP_(XP),
    XPP_(XPP),
    W_(W)
  {
  }

  Pos& X()
  {
    return X_;
  }

  Pos& pos()
  {
    return X_;
  }

  const Pos& X() const
  {
    return X_;
  }

  const Pos& pos() const
  {
    return X_;
  }

  Vel& XP()
  {
    return XP_;
  }

  Vel& vel()
  {
    return XP_;
  }

  const Vel& XP() const
  {
    return XP_;
  }

  const Vel& vel() const
  {
    return XP_;
  }
  
  Acc& XPP()
  {
    return XPP_;
  }

  Acc& acc()
  {
    return XPP_;
  }

  const Acc& XPP() const
  {
    return XPP_;
  }

  const Acc& acc() const
  {
    return XPP_;
  }

  Effort& W()
  {
    return W_;
  }

  Effort& effort()
  {
    return W_;
  }

  const Effort& W() const
  {
    return W_;
  }

  const Effort& effort() const
  {
    return W_;
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;

  /*!
   * \brief Assignment operator.
   */
  CartesianState& operator=(const CartesianState& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment of ow_msgs::CartesianState.
   */
  CartesianState& operator=(const ow_msgs::CartesianState& msg)
  {
    pos() = msg.position;
    vel() = msg.velocity;
    acc() = msg.acceleration;
    effort() = msg.wrench;
    return *this;
  }

  /*!
   * \brief Conversion to ow_msgs::CartesianState.
   */
  operator ow_msgs::CartesianState() const
  {
    ow_msgs::CartesianState msg;
    msg.position = pos();
    msg.velocity = vel();
    msg.acceleration = acc();
    msg.wrench = effort();
    return msg;
  }  

  /*!
   * \brief Conversion to ow_msgs::CartesianState.
   */
  ow_msgs::CartesianState toCartesianStateMsg() const
  {
    return static_cast<ow_msgs::CartesianState>(*this);
  }

};

}

#endif // OPEN_WALKER_CORE_LINEAR_STATE_H
