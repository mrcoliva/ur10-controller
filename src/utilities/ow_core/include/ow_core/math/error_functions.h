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

#ifndef OPEN_WALKER_CORE_MATH_ERROR_FUNTIONS_H
#define OPEN_WALKER_CORE_MATH_ERROR_FUNTIONS_H

#include <ow_core/math/quaternion.h>
#include <ow_core/math/quaternion.h>

// the namespace for the project
namespace ow
{

  /*!
  * \brief CartesianTransformation Error 
  * 
  * Computes the Error between a desired CartesianPosition Xd_base
  * and the current CartesianPosition Xcur_base.
  * 
  * Important:
  * The resulting error vector is expressed within the base coordinate system.
  * 
  * \return cartesian error w.r.t base
  * e_b = cartesianError(Xd_b, X_b)
  */
  inline CartesianVector& cartesianError(
    CartesianVector& e,
    const ow::CartesianPosition& Xd,
    const ow::CartesianPosition& X)
  {
    e.linear() = Xd.linear() - X.linear();
    e.angular() = quaternionLogError(Xd.angular(), X.angular());

    return e;
  };

  /*!
  * \brief CartesianTransformation Error 
  * 
  * Computes the Error between a desired CartesianPosition Xd_base
  * and the current CartesianPosition Xcur_base.
  * 
  * Important:
  * The resulting error vector is expressed within the base coordinate system.
  * 
  * \return cartesian error w.r.t base
  * e_b = cartesianError(Xd_b, X_b)
  */
  inline CartesianVector cartesianError(
    const ow::CartesianPosition& Xd,
    const ow::CartesianPosition& X)
  {
    CartesianVector e;
    return cartesianError(e, Xd, X);
  };

  /*!
  * \brief CartesianTransformation Error 
  * 
  * Computes the Error between a desired HomogeneousTransformation Td_base
  * and the current HomogeneousTransformation Tcur_base.
  * 
  * Important:
  * The resulting error vector is expressed within the base coordinate system.
  * 
  * \return cartesian error w.r.t base
  * e_b = cartesianError(Td_b, T_b)
  */
  inline CartesianVector& cartesianError(
    CartesianVector& e,
    const ow::HomogeneousTransformation& Td,
    const ow::HomogeneousTransformation& T)
  {
    ow_core::AngularPosition<Scalar> Qd(Td.orien());
    ow_core::AngularPosition<Scalar> Q(T.orien());
    e.linear() = Td.pos() - T.pos();
    e.angular() = quaternionLogError(Qd, Q);
    return e;
  };

  /*!
  * \brief CartesianTransformation Error 
  * 
  * Computes the Error between a desired HomogeneousTransformation Td_base
  * and the current HomogeneousTransformation Tcur_base.
  * 
  * Important:
  * The resulting error vector is expressed within the base coordinate system.
  * 
  * \return cartesian error w.r.t base
  * e_b = cartesianError(Td_b, T_b)
  */
  inline CartesianVector cartesianError(
    const ow::HomogeneousTransformation& Td,
    const ow::HomogeneousTransformation& T)
  {
    CartesianVector e;
    return cartesianError(e, Td, T);
  };


  /*!
   * \brief Compute the shortest connection between two Quaternions.
   * 
   * \return modified desired quaternion.
   */
  template <typename Derived1, typename Derived2>
  inline ow::AngularPosition shortestPath(
      const ow_core::QuaternionBase<Derived1> &Qd,
      const ow_core::QuaternionBase<Derived2> &Q)
  {
    ow::AngularPosition Qe = Qd.derived() * Q.derived().inverse();
    checkFlipQuaternionSign(Qe);
    return Qe * Q.derived();
  }

  /*!
   * \brief Compute the shortest connection between two CartesianPosition.
   * 
   * \return modified desired CartesianPosition.
   */
  inline ow::CartesianPosition shortestPath(
      const ow::CartesianPosition &Xd,
      const ow::CartesianPosition &X)
  {
    ow::CartesianPosition Xd_mod;
    Xd_mod.linear() = Xd.linear();
    Xd_mod.angular() = shortestPath(Xd.angular(), X.angular());
    return Xd_mod;
  };

  /*!
   * \brief Compute the shortest connection between two Vectors.
   * 
   * \return the desired Vector.
   */
  template <typename Derived1, typename Derived2>
  inline Eigen::Matrix<typename Derived1::Scalar, Derived1::RowsAtCompileTime, Derived1::ColsAtCompileTime>
  shortestPath(
      const Eigen::MatrixBase<Derived1> &xd,
      const Eigen::MatrixBase<Derived2> &x)
  {
    return xd;
  };

} // namespace ow

#endif
