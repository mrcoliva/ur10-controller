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


#ifndef OPEN_WALKER_CORE_JACOBIAN_REF_H
#define OPEN_WALKER_CORE_JACOBIAN_REF_H

#include <ow_core/utilities/type_guard.h>
#include <ow_core/type_bases/jacobian_base.h>

namespace ow{
/*!
 * \brief The traits class for the Jacobian class.
 *
 * This class contains the typedefs and enums for
 * the Jacobian class.
 *
 * These typedefs and enums are used to define and describe
 * the properties (traits) of this class.
 */
template <typename _Derived, int _Cols>
struct traits<ow_core::JacobianRef<_Derived, _Cols> >
{
  typedef typename _Derived::Scalar Scalar;
  enum
  {
    RowsAtCompileTime = 3,
    ColsAtCompileTime = _Cols
  };
};

} // namespace ow

namespace ow_core{

/*!
 * \brief The ForceRef class.
 *
 * The ForceRef is of type Eigen::Vector3 and is
 * represented by the math symbol \f$\mathbf{f}\f$.
 *
 * References the data of another Eigen type class
 * via Eigen::Block.
 *
 */
template <typename _Derived, int _Cols>
class JacobianRef :
  public Eigen::Block<_Derived, 3, _Cols>,
  public JacobianBase<JacobianRef<_Derived, _Cols> >,
  TypeGuard
{
  OW_TYPE_GUARD(JacobianRef)

public:
  typedef _Derived Derived;
  typedef typename Derived::Scalar Scalar;
  typedef Eigen::Block<Derived, 3, _Cols> Base;
  typedef JacobianBase<JacobianRef<Derived, _Cols> > JBase;

  enum
  {
    ColsAtCompileTime = _Cols
  };

public:
  /*!
   * \brief Default Constructor.
   *
   * \param ref
   *      the reference to storage Eigen object.
   *
   * \param start_row
   *      the start index of the row for Eigen::Block.
   *
   * \param start_col
   *      the start index of the column for Eigen::Block.
   */
  explicit JacobianRef(Derived& ref, int start_row = 0, int start_col = 0) : 
    Base(ref, start_row, start_col)
  {
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;
  using JBase::operator=;

private:
  /*!
   * \brief No null reference.
   */
  JacobianRef();
};

}  // namespace ow_core

#endif  // OPEN_WALKER_CORE_FORCE_REF_H
