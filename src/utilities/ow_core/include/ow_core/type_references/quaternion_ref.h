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


#ifndef OPEN_WALKER_CORE_QUATERNION_REF_H
#define OPEN_WALKER_CORE_QUATERNION_REF_H

#include <ow_core/utilities/type_guard.h>
#include <ow_core/type_bases/quaternion_base.h>
#include <ow_core/eigen/quaternion_ref.h>

namespace ow{

/*!
 * \brief The traits class for the AngularPosition class.
 *
 * This class contains the typedefs and enums for
 * the QuaternionRef class.
 *
 * These typedefs and enums are used to define and describe
 * the properties (traits) of this class.
 */
template<typename _Derived>
struct traits<ow_core::QuaternionRef<_Derived> >
{
  typedef typename _Derived::Scalar Scalar;
  typedef int Index;

  enum {
    RowsAtCompileTime = 4,
    ColsAtCompileTime = 1,
    SizeAtCompileTime = 4,
  };
};

} // namespace ow

namespace ow_core
{
/*!
 * \brief The QuaternionRef class.
 *
 * The QuaternionRef is of type Eigen::Quaternion and is
 * represented by the math symbol \f$\mathbf{Q}\f$.
 *
 * References the data of another Eigen type class
 * via Eigen::QuaternionRef.
 *
 */
template <typename _Derived>
class QuaternionRef : 
  public ow_core::QuaternionBase<_Derived>,
  public Eigen::QuaternionRef<_Derived>
{
public:
  typedef _Derived Derived;
  typedef Eigen::QuaternionRef<Derived> Base;
  typedef ow_core::QuaternionBase<Derived> QBase;

public:
  /*!
   * \brief Default Constructor.
   *
   * \param ref
   *      the reference to storage Eigen object to access the
   *      elements of the quaternion via Eigen::Block.
   *
   * \param start_row
   *      the start index of the row for Eigen::Block.
   *
   * \param start_col
   *      the start index of the column for Eigen::Block.
   */
  explicit QuaternionRef(Derived& ref, int start_row = 0, int start_col = 0) : 
    Base(ref, start_row, start_col)
  {
  }

  /*!
   * \brief Assignment operator.
   *
  QuaternionRef operator=(const Quaternion<Scalar>& other)
  {
    Base::operator=(other);
    return *this;
  }*/

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;
  using QBase::operator=;

private:
  /*!
   * \brief No null reference.
   */
  QuaternionRef();
};

}  // namespace ow_core

#endif  // OPEN_WALKER_CORE_QUATERNION_REF_H
