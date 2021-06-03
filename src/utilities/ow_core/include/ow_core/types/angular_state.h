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


#ifndef OPEN_WALKER_CORE_ANGULAR_STATE_H
#define OPEN_WALKER_CORE_ANGULAR_STATE_H

#include <ow_core/types/angular_position.h>
#include <ow_core/types/angular_velocity.h>
#include <ow_core/types/angular_acceleration.h>
#include <ow_core/types/moment.h>

#include <ow_core/type_bases/state_base.h>

#include <ow_msgs/AngularState.h>

namespace ow{

/*!
 * \brief The traits class for the AngularState class.
 *
 * This class contains the typedefs and enums for
 * the AngularState class.
 *
 * These typedefs and enums are used to define and describe
 * the properties (traits) of this class.
 */
template<typename _Scalar>
struct traits<ow_core::AngularState<_Scalar> >
{
  typedef _Scalar Scalar;
  typedef ow_core::AngularPosition<Scalar> Pos;
  typedef ow_core::AngularVelocity<Scalar> Vel;
  typedef ow_core::AngularAcceleration<Scalar> Acc;
  typedef ow_core::Moment<Scalar> Effort;
  
  enum
  {
    IsRef = 0,
  };
};

} // namespace ow


namespace ow_core{

/*!
 * \brief The AngularState class.
 *
 *  This class is a container for:
 *    - AngularPosition
 *    - AngularVelocity
 *    - AngularAcceleration
 *    - Moment
 */
template<typename _Scalar>
class AngularState : 
  public StateBase<AngularState<_Scalar> >
{
public:
  typedef _Scalar Scalar;
  typedef StateBase<AngularState<Scalar> > Base;

  typedef AngularPosition<Scalar> Pos;
  typedef AngularVelocity<Scalar> Vel;
  typedef AngularAcceleration<Scalar> Acc;
  typedef Moment<Scalar> Effort;

protected:
  Pos Q_;
  Vel omega_;
  Acc alpha_;
  Effort mu_;

public:
  /*!
    * \brief Default Constructor.
    */
  AngularState()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  AngularState(const AngularState& other) :
    Q_(other.Q_),
    omega_(other.omega_),
    alpha_(other.alpha_),
    mu_(other.mu_)
  {
  }

  /*!
   * \brief Copy constructor.
   */
  AngularState(const Base& other) : 
    Q_(other.pos()),
    omega_(other.vel()),
    alpha_(other.acc()),
    mu_(other.effort())
  {
  }

  /*!
    * \brief Constructor from sub elements
    */
  explicit AngularState(const Pos& Q, const Vel& omega = Vel::Zero(), const Acc& alpha = Acc::Zero(), const Effort& mu = Effort::Zero()) : 
    Q_(Q),
    omega_(omega),
    alpha_(alpha),
    mu_(mu)
  {
  }

  /*!
    * \brief access to orientation part.
    */
  Pos& Q()
  {
    return Q_;
  }

  /*!
    * \brief access to orientation part.
    */
  Pos& pos()
  {
    return Q_;
  }

  /*!
    * \brief access to orientation part.
    */
  const Pos& Q() const
  {
    return Q_;
  }

  /*!
    * \brief access to orientation part.
    */
  const Pos& pos() const
  {
    return Q_;
  }

  /*!
    * \brief access to velocity part.
    */
  Vel& omega()
  {
    return omega_;
  }

  /*!
    * \brief access to velocity part.
    */
  Vel& vel()
  {
    return omega_;
  }

  /*!
    * \brief access to velocity part.
    */
  const Vel& omega() const
  {
    return omega_;
  }

  /*!
    * \brief access to velocity part.
    */
  const Vel& vel() const
  {
    return omega_;
  }

  /*!
    * \brief access to acceleration part.
    */
  Acc& alpha()
  {
    return alpha_;
  }

  /*!
    * \brief access to acceleration part.
    */
  Acc& acc()
  {
    return alpha_;
  }

  /*!
    * \brief access to acceleration part.
    */
  const Acc& alpha() const
  {
    return alpha_;
  }

  /*!
    * \brief access to acceleration part.
    */
  const Acc& acc() const
  {
    return alpha_;
  }

  /*!
    * \brief access to acceleration part.
    */
  Effort& mu()
  {
    return mu_;
  }

  /*!
    * \brief access to moment part.
    */
  Effort& effort()
  {
    return mu_;
  }

  /*!
    * \brief access to moment part.
    */
  const Effort& mu() const
  {
    return mu_;
  }

  /*!
    * \brief access to moment part.
    */
  const Effort& effort() const
  {
    return mu_;
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;

  /*!
   * \brief Assignment operator.
   */
  AngularState& operator=(const AngularState& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment of ow_msgs::AngularState.
   */
  AngularState& operator=(const ow_msgs::AngularState& msg)
  {
    pos() = msg.position;
    vel() = msg.velocity;
    acc() = msg.acceleration;
    effort() = msg.torque;
    return *this;
  }

  /*!
   * \brief Conversion to ow_msgs::AngularState.
   */
  operator ow_msgs::AngularState() const
  {
    ow_msgs::AngularState msg;
    msg.position = pos();
    msg.velocity = vel();
    msg.acceleration = acc();
    msg.torque = effort();
    return msg;
  }  

  /*!
   * \brief Conversion to ow_msgs::AngularState.
   */
  ow_msgs::AngularState toAngularStateMsg() const
  {
    return static_cast<ow_msgs::AngularState>(*this);
  }

};

} // namespace ow_core

#endif // OPEN_WALKER_CORE_ANGULAR_STATE_H
