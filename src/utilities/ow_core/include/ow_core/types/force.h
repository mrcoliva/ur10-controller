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


#ifndef OPEN_WALKER_CORE_FORCE_H
#define OPEN_WALKER_CORE_FORCE_H

#include <ow_core/utilities/type_guard.h>
#include <ow_core/type_bases/force_base.h>

namespace ow_core{

/*!
 * \brief The Force class.
 *
 * The Force is of type Eigen::Vector3 and is
 * represented by the math symbol \f$\mathVec{f}\f$.
 */
template <typename _Scalar>
class Force :
    public Eigen::Matrix<_Scalar,3,1>,
    public ForceBase<Force<_Scalar> >,
    TypeGuard
{
  OW_TYPE_GUARD(Force)

public:
  typedef _Scalar Scalar;
  typedef Eigen::Matrix<_Scalar,3,1> Base;
  typedef ForceBase<Force<_Scalar> > FBase;

protected:

public:
  /*!
   * \brief Default Constructor.
   */
  Force()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  Force(const Force& other) :
    Base(other)
  {
  }

  /*!
   * \brief Assignment from Scalar values
   *
   */
  Force(const Scalar& x, const Scalar& y, const Scalar& z) :
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
  Force(const Eigen::EigenBase<OtherDerived>& other) :
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
  Force(const ForceRef<OtherDerived>& other) :
    Base(other)
  {
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;
  using FBase::operator=;

  /*!
   * \brief Assignment operator.
   */
  Force& operator=(const Force& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment operator.
   */
  template<typename OtherDerived>
  Force& operator=(const ForceRef<OtherDerived>& other)
  {
    Base::operator=(other);
    return *this;
  }

};

}  // namespace ow_core

#endif  // OPEN_WALKER_CORE_FORCE_H
