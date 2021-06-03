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


#ifndef OPEN_WALKER_CORE_MATH_QUATERNION_H
#define OPEN_WALKER_CORE_MATH_QUATERNION_H

#include <ow_core/types.h>

/*! 
 * \file quaternion.h
 * \brief Contains functions for quaternion related math
 */


// the namespace for the project
namespace ow{

/*!
 * \brief Flip Quaternions sign if scalar part w < 0.
 *
 * Flip Quaternion if scalar part sign smaller than zero, to prevent
 * pi/2 ambiguity in computation of the quaternion log map.
 */
template <typename Derived1>
bool checkFlipQuaternionSign(ow_core::QuaternionBase<Derived1>& Q)
{
  if(Q.derived().w() < 0.0)
  {
    Q.derived().coeffs() *= -1.0;
    return true;
  }
  return false;
}

/*!
 * \brief Computes the error between two quaternions.
 *
 * Computes the error between a desired quaternion Qd
 * and the current quaterion Q measured in the tangential vector space of S3.
 * 
 * The error is expressed within the base frame of Qe and Q.
 * e_b = quaternionLogError(Qe_b, Q_b)
 */
template <typename Derived1, typename Derived2>
inline Vector3& quaternionLogError(
    Vector3& e,
    const ow_core::QuaternionBase<Derived1>& Qd,
    const ow_core::QuaternionBase<Derived2>& Q)
{
  typedef ow::Scalar Scalar;

  // Relative quaternion between desired Q and current Q
  ow_core::AngularPosition<Scalar> Qe = Qd.derived()*Q.derived().inverse();
  checkFlipQuaternionSign(Qe);

  // return the log map of the delta quaternion
  e = Qe.logMap();

  return e;
}

/*!
 * \brief Computes the error between two quaternions.
 *
 * Computes the error between a desired quaternion Qd
 * and the current quaterion Q measured in the tangential vector space of S3.
 * 
 * The error is expressed within the base frame of Qe and Q.
 * e_b = quaternionLogError(Qe_b, Q_b)
 */
template <typename Derived1, typename Derived2>
inline Vector3 quaternionLogError(
    const ow_core::QuaternionBase<Derived1>& Qd,
    const ow_core::QuaternionBase<Derived2>& Q)
{
  Vector3 e;
  return quaternionLogError(e, Qd, Q);
}

/*!
 * \brief Computes the angular velocity omega.
 * 
 * Computes the angular velocity of a body given the time derivative of its 
 * quaternion coefficients QP and its current orientation Q.
 * 
 * The angular velocity is expressed in the fixed frame base  of Q_b.
 * omega_b = quaternion2AngularVelocityInertial(omega_b, QP_b, Q_b)
 * 
 * See: https://www.astro.rug.nl/software/kapteyn-beta/_downloads/attitude.pdf
 */
template <typename Derived1, typename Derived2>
inline AngularVelocity& quaternion2AngularVelocityInertial(
    AngularVelocity& omega,
    const ow_core::QuaternionBase<Derived1>& QP,
    const ow_core::QuaternionBase<Derived2>& Q)
{
  typedef ow::Scalar Scalar;

  Eigen::Matrix<Scalar,3,4> H;
  H << Q(3), -Q(2),  Q(1), -Q(0),
      Q(2),  Q(3), -Q(0), -Q(1),
      -Q(1),  Q(0),  Q(3), -Q(2);

  omega = Scalar(2)*H*QP.derived().coeffs();
  return omega;
}

/*!
 * \brief Computes the angular velocity omega.
 * 
 * Computes the angular velocity of a body given the time derivative of its 
 * quaternion coefficients QP and its current orientation Q.
 * 
 * The angular velocity is expressed in the fixed frame base  of Q_b.
 * omega_b = quaternion2AngularVelocityInertial(omega_b, QP_b, Q_b)
 * 
 * See: https://www.astro.rug.nl/software/kapteyn-beta/_downloads/attitude.pdf
 */
template <typename Derived1, typename Derived2, typename Derived3>
inline ow_core::AngularVelocityRef<Derived1> quaternion2AngularVelocityInertial(
    ow_core::AngularVelocityRef<Derived1> omega,
    const ow_core::QuaternionBase<Derived2>& QP,
    const ow_core::QuaternionBase<Derived3>& Q)
{
  typedef ow::Scalar Scalar;

  Eigen::Matrix<Scalar,3,4> H;
  H << Q(3), -Q(2),  Q(1), -Q(0),
       Q(2),  Q(3), -Q(0), -Q(1),
      -Q(1),  Q(0),  Q(3), -Q(2);

  omega = Scalar(2)*H*QP.derived().coeffs();
  return omega;
}

/*!
 * \brief Computes the angular acceleration alpha.
 * 
 * Computes the angular acceleration of a body given the sec. time derivative of 
 * its quaternion coefficients QPP and its current orientation Q.
 * 
 * The angular velocity is expressed in the fixed frame base  of Q_b.
 * omega_b = quaternion2AngularAccelerationInertial(omega_b, QPP_b, Q_b)
 * 
 * See: https://www.astro.rug.nl/software/kapteyn-beta/_downloads/attitude.pdf
 */
template <typename Derived1, typename Derived2>
inline AngularAcceleration& quaternion2AngularAccelerationInertial(
    AngularAcceleration& alpha,
    const ow_core::QuaternionBase<Derived1>& QPP,
    const ow_core::QuaternionBase<Derived2>& Q)
{
  typedef ow::Scalar Scalar;

  Eigen::Matrix<Scalar,3,4> H;
  H << Q(3), -Q(2),  Q(1), -Q(0),
       Q(2),  Q(3), -Q(0), -Q(1),
      -Q(1),  Q(0),  Q(3), -Q(2);

  alpha = Scalar(2)*H*QPP.derived().coeffs();
  return alpha;
}

/*!
 * \brief Computes the angular acceleration alpha.
 * 
 * Computes the angular acceleration of a body given the sec. time derivative of 
 * its quaternion coefficients QPP and its current orientation Q.
 * 
 * The angular velocity is expressed in the fixed frame base  of Q_b.
 * omega_b = quaternion2AngularAccelerationInertial(omega_b, QPP_b, Q_b)
 * 
 * See: https://www.astro.rug.nl/software/kapteyn-beta/_downloads/attitude.pdf
 */
template <typename Derived1, typename Derived2, typename Derived3>
inline ow_core::AngularAccelerationRef<Derived1> quaternion2AngularAccelerationInertial(
    ow_core::AngularAccelerationRef<Derived1> alpha,
    const ow_core::QuaternionBase<Derived2>& QPP,
    const ow_core::QuaternionBase<Derived3>& Q)
{
  typedef ow::Scalar Scalar;

  Eigen::Matrix<Scalar,3,4> H;
  H << Q(3), -Q(2),  Q(1), -Q(0),
      Q(2),  Q(3), -Q(0), -Q(1),
      -Q(1),  Q(0),  Q(3), -Q(2);

  alpha = Scalar(2)*H*QPP.derived().coeffs();
  return alpha;
}

/*!
 * \brief Computes the numeric integration of an AngularVelocity omega(t)
 *
 * Integrates AngularVelocity to AngularPosition over the
 * time interval dt by assuming a constant velocity over dt.
 *
 * q_[n+1] = Exp(w)*q_[n]
 * Exp(w) = [cos(||w||*dt/2), sin(||w||*dt/2) * w / ||w||]
 * 
 * The orientation Q[n+1] is expressed in the fixed frame base of Q_b.
 * Q[n+1]_b = quaternionIntegration(Q[n+1]_b, Q[n]_b, omega_b)
 */
template <typename Derived1, typename Derived2>
inline AngularPosition& quaternionIntegration(
  AngularPosition& Qr,
  const ow_core::QuaternionBase<Derived1>& Q,
  const ow_core::AngularVelocityBase<Derived2>& omega,
  ow::Scalar dt)
{
  Qr = AngularPosition::ExpMap(omega.derived()*dt)*Q.derived();
  return Qr;
}

/*!
 * \brief Computes the numeric integration of an AngularVelocity omega(t)
 *
 * Integrates AngularVelocity to AngularPosition over the
 * time interval dt by assuming a constant velocity over dt.
 *
 * q_[n+1] = Exp(w)*q_[n]
 * Exp(w) = [cos(||w||*dt/2), sin(||w||*dt/2) * w / ||w||]
 * 
 * The orientation Q[n+1] is expressed in the fixed frame base of Q_b.
 * Q[n+1]_b = quaternionIntegration(Q[n+1]_b, Q[n]_b, omega_b)
 */
template <typename Derived1, typename Derived2, typename Derived3>
inline ow_core::AngularPositionRef<Derived1> quaternionIntegration(
  ow_core::AngularPositionRef<Derived1> Qr,
  const ow_core::QuaternionBase<Derived2>& Q,
  const ow_core::AngularVelocityBase<Derived3>& omega,
  ow::Scalar dt)
{
  Qr = AngularPosition::ExpMap(omega.derived()*dt)*Q.derived();
  return Qr;
}

/*!
 * \brief SLERP interpolation for AngularPosition
 *
 * Interpolate between intial Qi and final Qf AngularPosition.
 *
 * See: https://en.wikipedia.org/wiki/Slerp
 */
template <typename Derived1, typename Derived2>
inline AngularPosition slerp(
  const ow_core::QuaternionBase<Derived1>& Qi,
  const ow_core::QuaternionBase<Derived2>& Qf,
  ow::Scalar s)
{
  typedef ow::Scalar Scalar;
  const Scalar one = Scalar(1) - std::numeric_limits<Scalar>::epsilon();

  ow::AngularPosition Qs;
  Scalar d = Qi.derived().dot(Qf.derived());
  Scalar d_abs = std::abs(d);

  Scalar scale_0;
  Scalar scale_1;

  if(d_abs >= one)
  {
    // linear interpolation
    scale_0 = Scalar(1) - s;
    scale_1 = s;
  }
  else
  {
    // spherical interpolation
    Scalar th = std::acos(d_abs);
    Scalar th_sin = std::sin(th);

    scale_0 = std::sin((Scalar(1) - s)*th)/th_sin;
    scale_1 = std::sin(s*th)/th_sin;
  }
  if(d < Scalar(0))
  {
    // take the shortest path
    scale_1 = -scale_1;
  }
  Qs.coeffs() = scale_0*Qi.derived().coeffs()
      + scale_1*Qf.derived().coeffs();
  return Qs;
}

/*!
 * \brief First derivative of SLERP
 *
 * First Order Derivate of interpolation between
 * intial and final AngularPosition wrt to parameterization s.
 * 
 * \note This is the derivative of the quaternion coefficients, not 
 * the time derivative (angular velocities).
 *
 * QsP = d slerp(Qi,Qf,s) / ds
 *
 * See: https://en.wikipedia.org/wiki/Slerp
 */
template <typename Derived1, typename Derived2>
inline AngularPosition slerpP(
    const ow_core::QuaternionBase<Derived1>& Qi,
    const ow_core::QuaternionBase<Derived2>& Qf,
    ow::Scalar s)
{
  typedef ow::Scalar Scalar;
  const Scalar one = Scalar(1) - std::numeric_limits<Scalar>::epsilon();

  AngularPosition QsP;
  Scalar d = Qi.derived().dot(Qf.derived());
  Scalar d_abs = std::abs(d);

  Scalar scale_0;
  Scalar scale_1;

  if(d_abs >= one)
  {
    // linear interpolation
    scale_0 = -1;
    scale_1 = 1;
  }
  else
  {
    // spherical interpolation
    Scalar th = std::acos(d_abs);
    Scalar th_sin = std::sin(th);

    scale_0 = -th*std::cos((Scalar(1) - s)*th)/th_sin;
    scale_1 = th*std::cos(s*th)/th_sin;
  }
  if(d < Scalar(0))
  {
    // take the shortest path
    scale_1 = -scale_1;
  }
  QsP.coeffs() = scale_0*Qi.derived().coeffs()
      + scale_1*Qf.derived().coeffs();
  return QsP;
}

/*!
 * \brief Second derivative of SLERP
 *
 * Second Order Derivate of interpolation between
 * intial and final AngularPosition wrt to parameterization s.
 *
 * QsPP = d2 slerp(Qi,Qf,s) / ds2
 *
 * See: https://en.wikipedia.org/wiki/Slerp
 */
template <typename Derived1, typename Derived2>
inline AngularPosition slerpPP(
    const ow_core::QuaternionBase<Derived1>& Qi,
    const ow_core::QuaternionBase<Derived2>& Qf,
    ow::Scalar s)
{
  typedef ow::Scalar Scalar;
  const Scalar one = Scalar(1) - std::numeric_limits<Scalar>::epsilon();

  AngularPosition QsPP;
  Scalar d = Qi.derived().dot(Qf.derived());
  Scalar d_abs = std::abs(d);

  Scalar scale_0;
  Scalar scale_1;

  if(d_abs >= one)
  {
    // linear interpolation
    scale_0 = 0;
    scale_1 = 0;
  }
  else
  {
    // spherical interpolation
    Scalar th = std::acos(d_abs);
    Scalar th_sin = std::sin(th);

    scale_0 = -th*th*std::sin((Scalar(1) - s)*th)/th_sin;
    scale_1 = -th*th*std::sin(s*th)/th_sin;
  }
  if(d < Scalar(0))
  {
    // take the shortest path
    scale_1 = -scale_1;
  }
  QsPP.coeffs() = scale_0*Qi.derived().coeffs()
      + scale_1*Qf.derived().coeffs();
  return QsPP;
}

} // namespace ow

#endif // OPEN_WALKER_CORE_MATH_QUATERNION_H
