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


#ifndef OPEN_WALKER_CORE_LINEAR_VELOCITY_H
#define OPEN_WALKER_CORE_LINEAR_VELOCITY_H

#include <ow_core/utilities/type_guard.h>
#include <ow_core/type_bases/linear_velocity_base.h>


namespace ow_core{

/*!
 * \brief The LinearVelocity class.
 *
 * The LinearVelocity is of type Eigen::Vector3 and is
 * represented by the math symbol \f$\dot{\mathbf{x}}\f$.
 *
 */
template <typename _Scalar>
class LinearVelocity :
    public Eigen::Matrix<_Scalar,3,1>,
    public LinearVelocityBase<LinearVelocity<_Scalar> >,
    TypeGuard
{
  OW_TYPE_GUARD(LinearVelocity)

public:
  typedef _Scalar Scalar;
  typedef Eigen::Matrix<_Scalar,3,1> Base;
  typedef LinearVelocityBase<LinearVelocity<_Scalar> > LVBase;

protected:

public:
  /*!
   * \brief Default Constructor.
   */
  LinearVelocity()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  LinearVelocity(const LinearVelocity& other) :
    Base(other)
  {
  }

  /*!
   * \brief Assignment from Scalar values
   *
   */
  LinearVelocity(const Scalar& xP, const Scalar& yP, const Scalar& zP) :
    Base(xP,yP,zP)
  {
  }

  /*!
   *  \brief Copy constructor.
   *
   * This copy constructor not only works with Eigen matrices
   * but also with their expressions.
   */
  template <typename OtherDerived>
  LinearVelocity(const Eigen::EigenBase<OtherDerived>& other) :
    Base(other)
  {
  }

  /*!
   * \brief Copy constructor.
   *
   * This copy constructor allows construction from
   * the reference class.
   */
  template <typename OtherDerived>
  LinearVelocity(const LinearVelocityRef<OtherDerived>& other) :
    Base(other)
  {
  }


  /*!
   * \brief Use assignment operators of the base class.
   */
  using Base::operator=;
  using LVBase::operator=;

  /*!
   * \brief Assignment operator.
   */
  LinearVelocity& operator=(const LinearVelocity& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment operator.
   */
  template<typename OtherDerived>
  LinearVelocity& operator=(const LinearVelocityRef<OtherDerived>& other)
  {
    Base::operator=(other);
    return *this;
  }

};

}  // namespace ow_core

#endif  // OPEN_WALKER_CORE_LINEAR_VELOCITY_H
