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


#ifndef OPEN_WALKER_CORE_ANGULAR_ACCELERATION_H
#define OPEN_WALKER_CORE_ANGULAR_ACCELERATION_H

#include <ow_core/utilities/type_guard.h>
#include <ow_core/type_bases/angular_acceleration_base.h>

namespace ow_core
{
/*!
 * \brief The AngularAcceleration class.
 *
 * The AngularAcceleration is of type Eigen::Vector3 and is
 * represented by the math symbol \f$\mathbf{\alpha}\f$.
 *
 */
template <typename _Scalar>
class AngularAcceleration :
    public Eigen::Matrix<_Scalar,3,1>,
    public AngularAccelerationBase<AngularAcceleration<_Scalar> >,
    TypeGuard
{
  OW_TYPE_GUARD(AngularAcceleration)

public:
  typedef _Scalar Scalar;
  typedef Eigen::Matrix<Scalar,3,1> Base;
  typedef AngularAccelerationBase<AngularAcceleration<Scalar> > ABase;

protected:

public:
  /*!
   * \brief Default Constructor.
   */
  AngularAcceleration()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  AngularAcceleration(const AngularAcceleration& other) :
    Base(other)
  {
  }

  /*!
   * \brief Assignment from Scalar values
   *
   */
  AngularAcceleration(const Scalar& x, const Scalar& y, const Scalar& z) :
    Base(x,y,z)
  {
  }

  /*!
   * \brief Copy constructor.
   *
   * This copy constructor not only works with Eigen matrices
   * but also with their expressions.
   */
  template <typename OtherDerived>
  AngularAcceleration(const Eigen::EigenBase<OtherDerived>& other) :
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
  AngularAcceleration(const AngularAccelerationRef<OtherDerived>& other) :
    Base(other)
  {
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;
  using ABase::operator=;

  /*!
   * \brief Assignment operator.
   */
  AngularAcceleration& operator=(const AngularAcceleration& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment operator.
   */
  template<typename OtherDerived>
  AngularAcceleration& operator=(
    const AngularAccelerationRef<OtherDerived>& other)
  {
    Base::operator=(other);
    return *this;
  }

  

};

}  // namespace ow_core

#endif  // OPEN_WALKER_CORE_ANGULAR_ACCELERATION_H
