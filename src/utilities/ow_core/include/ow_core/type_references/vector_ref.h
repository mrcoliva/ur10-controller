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


#ifndef OPEN_WALKER_CORE_VECTOR_REF_H
#define OPEN_WALKER_CORE_VECTOR_REF_H

#include <ow_core/utilities/type_guard.h>
#include <ow_core/type_bases/vector_base.h>

namespace ow_core
{
/*!
 * \brief The VectorRef class.
 *
 * References the data of another Eigen type class
 * via Eigen:Block.
 *
 * We need this special type to get the behavior of Eigen::Matrix
 * when defining new references.
 */
template <typename _Derived, int _Rows = Eigen::Dynamic>
class VectorRef : 
  public Eigen::Block<_Derived, _Rows, 1>,
  public VectorBase<VectorRef<_Derived, _Rows> >,
  TypeGuard
{
  OW_TYPE_GUARD(VectorRef)

public:
  typedef _Derived Derived;
  typedef typename Derived::Scalar Scalar;

  enum
  {
    RowsAtCompileTime = _Rows
  };

  typedef Eigen::Block<Derived, RowsAtCompileTime, 1> Base;
  typedef VectorBase<VectorRef<Derived, RowsAtCompileTime> > VBase;

public:
  /*!
   * \brief Default Constructor, Fixed Sized Vector.
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
  explicit VectorRef(Derived& ref, 
                     int start_row = 0, 
                     int start_col = 0) : 
    Base(ref, start_row, start_col)
  {
  }

  /*!
   * \brief Default Constructor, Dynamically Sized Vector.
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
   * 
   * \param block_rows
   *      the start index of the column for Eigen::Block.
   */
  VectorRef(Derived& ref, 
            int start_row, 
            int start_col,
            int block_rows) : 
    Base(ref, start_row, start_col, block_rows, 1)
  {
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;
  using VBase::operator=;

private:
  /*!
   * \brief No null reference.
   */
  VectorRef();
};

}  // namespace ow_core

#endif  // OPEN_WALKER_CORE_VECTOR_REF_H
