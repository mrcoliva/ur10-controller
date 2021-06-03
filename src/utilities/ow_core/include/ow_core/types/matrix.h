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


#ifndef OPEN_WALKER_CORE_MATRIX_H
#define OPEN_WALKER_CORE_MATRIX_H

#include <ow_core/utilities/type_guard.h>
#include <ow_core/type_bases/matrix_base.h>

namespace ow_core
{
/*!
 * \brief The Matrix class.
 *
 * The Matrix is of type Eigen::Matrix.
 */
template <typename _Scalar, int _Rows, int _Cols>
class Matrix :
  public Eigen::Matrix<_Scalar, _Rows, _Cols>,
  public MatrixBase<Matrix<_Scalar, _Rows, _Cols> >
{
public:

  enum
  {
      RowsAtCompileTime = _Rows,
      ColsAtCompileTime = _Cols,
  };

  typedef _Scalar Scalar;
  typedef Eigen::Matrix<_Scalar,RowsAtCompileTime,ColsAtCompileTime> Base;
  typedef MatrixBase<Matrix<Scalar,RowsAtCompileTime,ColsAtCompileTime> > MBase;

protected:

public:
  /*!
   * \brief Default Constructor.
   */
  explicit Matrix(Scalar& ref, int start_row = 0, int start_col = 0) : 
    Base(ref, start_row, start_col)
  {
  }

  /*!
   * \brief Default Constructor.
   */
  Matrix()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  Matrix(const Matrix& other) :
    Base(other)
  {
  }

  /*!
   *  \brief Copy constructor.
   *
   * This copy constructor not only works with Eigen matrices
   * but also with their expressions.
   */
  template<typename OtherDerived>
  Matrix(const Eigen::EigenBase<OtherDerived>& other) :
    Base(other)
  {
  }

  /*!
   * \brief Use assignment operators of MBase class.
   */
  using Base::operator=;
  using MBase::operator=;

  /*!
   * \brief Assignment operator.
   */
  Matrix& operator=(const Matrix& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment operator.
   */
  template<typename OtherDerived, int OtherRows, int OtherCols>
  Matrix& operator=(const MatrixRef<OtherDerived, OtherRows, OtherCols>& other)
  {
    Base::operator=(other);
    return *this;
  }

};

}  // namespace ow_core


#endif  // OPEN_WALKER_CORE_MATRIX_H
