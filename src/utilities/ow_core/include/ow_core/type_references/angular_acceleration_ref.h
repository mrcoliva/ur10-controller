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


#ifndef OPEN_WALKER_CORE_ANGULAR_ACCELERATION_REF_H
#define OPEN_WALKER_CORE_ANGULAR_ACCELERATION_REF_H

#include <ow_core/utilities/type_guard.h>
#include <ow_core/type_bases/angular_acceleration_base.h>

namespace ow_core
{

/*!
 * \brief The AngularAccelerationRef class.
 *
 * The AngularAcceleration is of type Eigen::Vector3 and is
 * represented by the math symbol \f$\mathbf{\alpha}\f$.
 *
 * References the data of another Eigen type class
 * via Eigen::Block.
 *
 */
template <typename _Derived>
class AngularAccelerationRef :
    public Eigen::Block<_Derived, 3, 1>,
    public AngularAccelerationBase<AngularAccelerationRef<_Derived> >,
    TypeGuard
{
  OW_TYPE_GUARD(AngularAccelerationRef)

public:
  typedef _Derived Derived;
  typedef typename Derived::Scalar Scalar;
  typedef Eigen::Block<Derived, 3, 1> Base;
  typedef AngularAccelerationBase<AngularAccelerationRef<Derived> > ABase;

public:
  /*!
   * \brief Default Constructor.
   *
   * \param ref
   *      the reference to storage Eigen object to access the
   *      elements of the AngularVelocity via Eigen::Block.
   *
   * \param start_row
   *      the start index of the row for Eigen::Block.
   *
   * \param start_col
   *      the start index of the column for Eigen::Block.
   */
  explicit AngularAccelerationRef(Derived& ref, 
                                  int start_row = 0, 
                                  int start_col = 0) :
    Base(ref, start_row, start_col)
  {
  }

  /*!
   * \brief Assignment operator.
   */
  AngularAccelerationRef operator=(const AngularAcceleration<Scalar>& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;
  using ABase::operator=;

private:
  /*!
   * \brief No null reference.
   */
  AngularAccelerationRef();
};

}

#endif  // OPEN_WALKER_CORE_ANGULAR_ACCELERATION_REF_H
