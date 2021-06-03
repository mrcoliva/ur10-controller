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


#ifndef OPEN_WALKER_CORE_STATE_BASE_H
#define OPEN_WALKER_CORE_STATE_BASE_H

#include <ow_core/utilities/support_templates.h>
#include <ow_core/utilities/forward_declarations.h>

namespace ow_core
{

template <typename _Derived, int _IsRef>
struct StateBaseTraits;

template <typename _Derived>
struct StateBaseTraits<_Derived, 0>
{
  typedef typename ow::traits<_Derived>::Pos& Pos;
  typedef typename ow::traits<_Derived>::Vel& Vel;
  typedef typename ow::traits<_Derived>::Acc& Acc;
  typedef typename ow::traits<_Derived>::Effort& Effort;
  typedef const typename ow::traits<_Derived>::Pos& CPos;
  typedef const typename ow::traits<_Derived>::Vel& CVel;
  typedef const typename ow::traits<_Derived>::Acc& CAcc;
  typedef const typename ow::traits<_Derived>::Effort& CEffort;
};

template <typename _Derived>
struct StateBaseTraits<_Derived, 1>
{
  typedef typename ow::traits<_Derived>::Pos Pos;
  typedef typename ow::traits<_Derived>::Vel Vel;
  typedef typename ow::traits<_Derived>::Acc Acc;
  typedef typename ow::traits<_Derived>::Effort Effort;
  typedef const typename ow::traits<_Derived>::CPos CPos;
  typedef const typename ow::traits<_Derived>::CVel CVel;
  typedef const typename ow::traits<_Derived>::CAcc CAcc;
  typedef const typename ow::traits<_Derived>::CEffort CEffort;
};

/*!
 * \brief The StateBase class.
 *
 * Template interface class for the group of state type classes:
 * JointState, LinearState, AngularState, CartesianState.
 *
 * This class is important when implementing operations that
 * should be available for all the state classes.
 */
template <typename _Derived>
class StateBase
{
public:
  typedef _Derived Derived;

  typedef typename ow::traits<Derived>::Pos Pos;
  typedef typename ow::traits<Derived>::Vel Vel;
  typedef typename ow::traits<Derived>::Acc Acc;
  typedef typename ow::traits<Derived>::Effort Effort;
  enum
  {
    IsRef = ow::traits<Derived>::IsRef,
  };

  typedef typename StateBaseTraits<Derived, IsRef>::Pos RetPos;
  typedef typename StateBaseTraits<Derived, IsRef>::Vel RetVel;
  typedef typename StateBaseTraits<Derived, IsRef>::Acc RetAcc;
  typedef typename StateBaseTraits<Derived, IsRef>::Effort RetEffort;
  typedef typename StateBaseTraits<Derived, IsRef>::CPos CRetPos;
  typedef typename StateBaseTraits<Derived, IsRef>::CVel CRetVel;
  typedef typename StateBaseTraits<Derived, IsRef>::CAcc CRetAcc;
  typedef typename StateBaseTraits<Derived, IsRef>::CEffort CRetEffort;

public:

  /*!
   * \brief Get Zero State for static Sizes
   */
  static const Derived& Zero()
  {
    static Derived v;
    static bool once = false;
    if(!once)
    {
      v.pos() = Pos::Zero();
      v.vel() = Vel::Zero();
      v.acc() = Acc::Zero();
      v.effort() = Effort::Zero();
      once = true;
    }
    return v;
  }

  /*!
   * \brief Get Zero State for dynamic Sizes
   */
  template<typename T>
  static const Derived& Zero(const T& x)
  {
    static Derived v;
    v.pos() = Pos::Zero(x);
    v.vel() = Vel::Zero(x);
    v.acc() = Acc::Zero(x);
    v.effort() = Effort::Zero(x);
    return v;
  }

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
   * \brief Assignment operator.
   */
  StateBase& operator=(const StateBase& other)
  {
    derived().pos() = other.pos();
    derived().vel() = other.vel();
    derived().acc() = other.acc();
    derived().effort() = other.effort();
    return *this;
  }

  /*!
   * \brief set everything to zero.
   */
  StateBase& setZero()
  {
    derived().pos().setZero();
    derived().vel().setZero();
    derived().acc().setZero();
    derived().effort().setZero();
    return *this;
  }

  /*!
   * \brief set everything to zero.
   */
  template<typename T>
  StateBase& setZero(const T& x)
  {
    derived().pos().setZero(x);
    derived().vel().setZero(x);
    derived().acc().setZero(x);
    derived().effort().setZero(x);
    return *this;
  }

  /*!
   * \brief resize the vector.
   */
  template<typename T>
  void resize(const T& x)
  {
    derived().pos().resize(x);
    derived().vel().resize(x);
    derived().acc().resize(x);
    derived().effort().resize(x);
  }

  /*!
   * \brief General Position value.
   */
  RetPos pos()
  {
    return derived().pos();
  }

  /*!
   * \brief General Position value.
   */
  CRetPos pos() const
  {
    return derived().pos();
  }

  /*!
   * \brief General Velocity value.
   */
  RetVel vel()
  {
    return derived().vel();
  }

  /*!
   * \brief General Velocity value.
   */
  CRetVel vel() const
  {
    return derived().vel();
  }

  /*!
   * \brief General Acceleration value.
   */
  RetAcc acc()
  {
    return derived().acc();
  }

  /*!
   * \brief General Acceleration value.
   */
  CRetAcc acc() const
  {
    return derived().acc();
  }

  /*!
   * \brief General Force value.
   */
  RetEffort effort()
  {
    return derived().effort();
  }

  /*!
   * \brief General Force value.
   */
  CRetEffort effort() const
  {
    return derived().effort();
  }

  /*!
   * \brief General Print Function.
   */
  std::string toString() const
  {
    std::ostringstream out;
    out << "pos    = [" << pos().toString() << "]\n";
    out << "vel    = [" << vel().toString() << "]\n";
    out << "acc    = [" << acc().toString() << "]\n";
    out << "effort = [" << effort().toString() << "]\n";

    return out.str();
  }
  
};

} // namespace ow_core

#endif  // OPEN_WALKER_CORE_STATE_BASE_H
