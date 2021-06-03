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

#ifndef OPEN_WALKER_CORE_MATH_CROSS_MAPS_H
#define OPEN_WALKER_CORE_MATH_CROSS_MAPS_H

#include <Eigen/Geometry>

// the namespace for the project
namespace ow
{

 /*!
  * \brief Construct a Quaternion from an element in its tangential space.
  * 
  * This can used to map an angular velocity omega from to an lie element in S3.
  */
 template<typename Derived>
  inline Eigen::Quaternion<typename Derived::Scalar> 
    expMapS3(const Eigen::MatrixBase<Derived>& omega)
  {
    typedef typename Derived::Scalar Scalar;

    Scalar omega_norm = omega.norm();
    if(omega_norm <= std::numeric_limits<Scalar>::epsilon())
    {
      return Eigen::Quaternion<Scalar>::Identity();
    }
    Scalar th = Scalar(0.5)*omega_norm;

    Eigen::Quaternion<Scalar> Q;
    Q.coeffs() << std::sin(th)*omega/omega_norm, std::cos(th);
    return Q;
  }

  /*!
   * \brief get the element in the tangential vector space of this Lie element 
   * in cartesian vector form. 
   * 
   * The vector in tangential space is a 3d vector for a quaternions.
   * 
   * \note To calculate the error between two quaternions the signs of the 
   * coefficients need to be flipped if scalar part w < 0
   */
  template<typename Derived>
  Eigen::Matrix<typename Derived::Scalar,3,1> logMapS3(const Eigen::QuaternionBase<Derived>& Q)
  {
    typedef typename Derived::Scalar Scalar;

    const Scalar one = Scalar(1) - std::numeric_limits<Scalar>::epsilon();

    Scalar theta 
      = Scalar(2)*std::atan2(Q.vec().norm(), Q.w());

    // avoid division by 0
    Scalar d = Q.w()*Q.w();
    if(std::abs(d) >= one)
    {
      // linear interpolation
      return Scalar(2)*(Scalar(1) + Scalar(1)/Scalar(6)*theta*theta)*Q.vec();
    }
    return theta/std::sin(theta/Scalar(2))*Q.vec();
  }

} // namespace ow

#endif