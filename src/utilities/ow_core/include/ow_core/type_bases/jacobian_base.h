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


#ifndef OPEN_WALKER_CORE_JACOBIAN_BASE_H
#define OPEN_WALKER_CORE_JACOBIAN_BASE_H

#include <ow_core/math/pseudo_inverse.h>
#include <ow_core/type_bases/matrix_base.h>

namespace ow_core{

/*!
 * \brief The JacobianBase class.
 */
template <typename _Derived>
class JacobianBase :
    public MatrixBase<_Derived>
{
public:
  typedef _Derived Derived;
  typedef MatrixBase<Derived> Base;

  typedef typename ow::traits<Derived>::Scalar Scalar;
  enum {
    RowsAtCompileTime = ow::traits<Derived>::RowsAtCompileTime,
    ColsAtCompileTime = ow::traits<Derived>::ColsAtCompileTime,
  };

  typedef Eigen::Matrix<Scalar,RowsAtCompileTime,ColsAtCompileTime> Matrix;
  typedef Eigen::Matrix<Scalar,ColsAtCompileTime,RowsAtCompileTime> Inverse;
  typedef Eigen::Matrix<Scalar,ColsAtCompileTime,ColsAtCompileTime> Nullspace;

public:
  using Base::operator=;

  /**
   * @brief compute the manipulability of the jacobain
   * 
   * @return manipulability 
   */
  Scalar manipulability() const
  {
    return std::sqrt((Base::derived()*Base::derived().transpose()).determinant());
  }

  /**
   * @brief Compute the damped pseudo inverse of the jacobian.
   * 
   * @param lamda 
   * @param tol 
   * @return Inverse 
   */
  Inverse dampedPinv(Scalar lamda = Scalar(1e-8), Scalar tol = Scalar(1e-8)) const
  {
    Matrix m = Base::derived();
    return ow::dampedPinvSVD<Matrix>(m, lamda, tol);
  }

  /**
   * @brief Compute the pseudo inverse of the jacobian.
   * 
   * @param lamda 
   * @param tol 
   * @return Inverse 
   */
  Inverse pinv(Scalar lamda = Scalar(1e-8), Scalar tol = Scalar(1e-8)) const
  {
    Matrix m = Base::derived();
    return ow::pinvSVD<Matrix>(m, tol);
  }

  /**
   * @brief computes the nullspace for joint velocites (without damping).
   * 
   * @note The nullspace for torques is the transpose of this function.
   * 
   * @return Nullspace 
   */
  Nullspace nullspace()
  {
    static const Nullspace I = Nullspace::Identity();
    return I - pinv()*Base::derived();
  }
};

}  // namespace ow_core

#endif  // OPEN_WALKER_CORE_JACOBIAN_BASE_H
