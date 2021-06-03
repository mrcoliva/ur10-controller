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


#ifndef OPEN_WALKER_CORE_MATH_TRANSFORM_H
#define OPEN_WALKER_CORE_MATH_TRANSFORM_H

#include <ow_core/types.h>

/*! \file transform.h
 *  \brief Contains functions for change the reference frames of type classes.
 */

// the namespace for the project
namespace ow{

/*!
 * \brief Transform a Vector3 to a different coordinate frame.
 *
 * \param v_child
 *      Vector3 wrt original frame.
 *
 * \param T_parent_child
 *      HomogeneousTransformation from original frame to new coordinate frame.
 *
 * \return
 *      Transformed Vector3 wrt new frame.
 */
template <typename Derived>
inline Vector3 changeRefFrame(
    const ow_core::Vector3Base<Derived>& v_child,
    const HomogeneousTransformation& T_parent_child)
{
  return T_parent_child.rotation()*v_child.derived();
}

template <typename Derived>
inline Vector3 changeRefFrame(
    const ow_core::Vector3Base<Derived>& v_child,
    const CartesianPosition& X_parent_child)
{
  return X_parent_child.angular()*v_child.derived();
}

/*!
 * \brief Transform a AngularAcceleration to a different coordinate frame.
 *
 * \param xPP_child
 *      AngularAcceleration wrt original frame.
 *
 * \param T_parent_child
 *      HomogeneousTransformation from original frame to new coordinate frame.
 *
 * \return
 *      Transformed AngularAcceleration wrt new frame.
 */
template <typename Derived>
inline AngularAcceleration changeRefFrame(
    const ow_core::AngularAccelerationBase<Derived>& xPP_child,
    const HomogeneousTransformation& T_parent_child)
{
  return T_parent_child*xPP_child.derived();
}

template <typename Derived>
inline AngularAcceleration changeRefFrame(
    const ow_core::AngularAccelerationBase<Derived>& xPP_child,
    const CartesianPosition& X_parent_child)
{
  return X_parent_child.angular()*xPP_child.derived();
}

/*!
 * \brief Transform a AngularPosition to a different coordinate frame.
 *
 * \param Q_child
 *      AngularPosition wrt original frame.
 *
 * \param T_parent_child
 *      HomogeneousTransformation from original frame to new coordinate frame.
 *
 * \return
 *      Transformed AngularPosition wrt new frame.
 */
template <typename Derived>
inline AngularPosition changeRefFrame(
    const ow_core::AngularPositionBase<Derived>& Q_child,
    const HomogeneousTransformation& T_parent_child)
{
  return T_parent_child.orientation()*Q_child.derived();
}

template <typename Derived>
inline AngularPosition changeRefFrame(
    const ow_core::AngularPositionBase<Derived>& Q_child,
    const CartesianPosition& X_parent_child)
{
  return X_parent_child.angular()*Q_child.derived();
}

/*!
 * \brief Transform a AngularVelocity to a different coordinate frame.
 *
 * \param xP_child
 *      AngularVelocity wrt original frame.
 *
 * \param T_parent_child
 *      HomogeneousTransformation from original frame to new coordinate frame.
 *
 * \return
 *      Transformed AngularVelocity wrt new frame.
 */
template <typename Derived>
inline AngularVelocity changeRefFrame(
    const ow_core::AngularVelocityBase<Derived>& xP_child,
    const HomogeneousTransformation& T_parent_child)
{
  return T_parent_child*xP_child.derived();
}

template <typename Derived>
inline AngularVelocity changeRefFrame(
    const ow_core::AngularVelocityBase<Derived>& xP_child,
    const CartesianPosition& X_parent_child)
{
  return X_parent_child.angular()*xP_child.derived();
}

/*!
 * \brief Transform a Force to a different coordinate frame.
 *
 * \param f_child
 *      Force wrt original frame.
 *
 * \param T_parent_child
 *      HomogeneousTransformation from original frame to new coordinate frame.
 *
 * \return
 *      Transformed Force wrt new frame.
 */
template <typename Derived>
inline Force changeRefFrame(
    const ow_core::ForceBase<Derived>& f_child,
    const HomogeneousTransformation& T_parent_child)
{
  return T_parent_child*f_child.derived();
}

template <typename Derived>
inline Force changeRefFrame(
    const ow_core::ForceBase<Derived>& f_child,
    const CartesianPosition& X_parent_child)
{
  return X_parent_child.angular()*f_child.derived();
}

/*!
 * \brief Transform a LinearAcceleration to a different coordinate frame.
 *
 * \param xPP_child
 *      LinearAcceleration wrt original frame.
 *
 * \param T_parent_child
 *      HomogeneousTransformation from original frame to new coordinate frame.
 *
 * \return
 *      Transformed LinearAcceleration wrt new frame.
 */
template <typename Derived>
inline LinearAcceleration changeRefFrame(
    const ow_core::LinearAccelerationBase<Derived>& xPP_child,
    const HomogeneousTransformation& T_parent_child)
{
  return T_parent_child*xPP_child.derived();
}

template <typename Derived>
inline LinearAcceleration changeRefFrame(
    const ow_core::LinearAccelerationBase<Derived>& xPP_child,
    const CartesianPosition& X_parent_child)
{
  return X_parent_child.angular()*xPP_child.derived();
}

/*!
 * \brief Transform a LinearPosition to a different coordinate frame.
 *
 * \param x_child
 *      LinearPosition wrt original frame.
 *
 * \param T_parent_child
 *      HomogeneousTransformation from original frame to new coordinate frame.
 *
 * \return
 *      Transformed LinearPosition wrt new frame.
 */
template <typename Derived>
inline LinearPosition changeRefFrame(
    const ow_core::LinearPositionBase<Derived>& x_child,
    const HomogeneousTransformation& T_parent_child)
{
  return T_parent_child*x_child.derived();
}

template <typename Derived>
inline LinearPosition changeRefFrame(
    const ow_core::LinearPositionBase<Derived>& x_child,
    const CartesianPosition& X_parent_child)
{
  // same as X_parent_child*x_child.derived()
  return X_parent_child.angular()*x_child.derived() + X_parent_child.linear();
}

/*!
 * \brief Transform a LinearVelocity to a different coordinate frame.
 *
 * \param xP_child
 *      LinearVelocity wrt original frame.
 *
 * \param T_parent_child
 *      HomogeneousTransformation from original frame to new coordinate frame.
 *
 * \return
 *      Transformed LinearVelocity wrt new frame.
 */
template <typename Derived>
inline LinearVelocity changeRefFrame(
    const ow_core::LinearVelocityBase<Derived>& xP_child,
    const HomogeneousTransformation& T_parent_child)
{
  return T_parent_child*xP_child.derived();
}

template <typename Derived>
inline LinearVelocity changeRefFrame(
    const ow_core::LinearVelocityBase<Derived>& xP_child,
    const CartesianPosition& X_parent_child)
{
  return X_parent_child.angular()*xP_child.derived();
}

/*!
 * \brief Transform a Moment to a different coordinate frame.
 *
 * \param mu_child
 *      Moment wrt original frame.
 *
 * \param T_parent_child
 *      HomogeneousTransformation from original frame to new coordinate frame.
 *
 * \return
 *      Transformed Moment wrt new frame.
 */
template <typename Derived>
inline Moment changeRefFrame(
    const ow_core::MomentBase<Derived>& mu_child,
    const HomogeneousTransformation& T_parent_child)
{
  return T_parent_child.rotation()*mu_child.derived();
}

template <typename Derived>
inline Moment changeRefFrame(
    const ow_core::MomentBase<Derived>& mu_child,
    const CartesianPosition& X_parent_child)
{
  return X_parent_child.angular()*mu_child.derived();
}

/*!
 * \brief Transform Cartesian vector to a different coordinate frame.
 *
 * \param v_child
 *      Cartesian vector wrt original frame.
 *
 * \param T_parent_child
 *      HomogeneousTransformation from original frame to new coordinate frame.
 * 
 * \return
 *      Transformed Cartesian vector wrt new coordinate frame.
 */
template<typename Derived>
inline Derived changeRefFrame(
    const ow_core::CartesianBase<Derived>& X_child,
    const HomogeneousTransformation& T_parent_child)
{
  Derived X_new;
  X_new.angular() = changeRefFrame(X_child.angular(), T_parent_child);
  X_new.linear() = changeRefFrame(X_child.linear(), T_parent_child);
  return X_new;
}

template<typename Derived>
inline Derived changeRefFrame(
    const ow_core::CartesianBase<Derived>& X_child,
    const CartesianPosition& X_parent_child)
{
  Derived X_new;
  X_new.angular() = changeRefFrame(X_child.angular(), X_parent_child);
  X_new.linear() = changeRefFrame(X_child.linear(), X_parent_child);
  return X_new;
}

/*!
 * \brief Transform Wrench to a different coordinate frame.
 *
 * \param Worg
 *      Wrench wrt original frame.
 *
 * \param T_parent_child
 *      HomogeneousTransformation from original frame to new coordinate frame.
 * 
 * \return
 *      Transformed Wrench wrt new coordinate frame.
 */
inline Wrench changeRefFrame(const Wrench& Worg,
                      const HomogeneousTransformation& T_parent_child)
{
  Wrench W_new;
  W_new.force() = changeRefFrame(Worg.force(),T_parent_child);
  W_new.moment() = changeRefFrame(Worg.moment(),T_parent_child);
  return W_new;
}

inline Wrench changeRefFrame(const Wrench& Worg,
                      const CartesianPosition& X_parent_child)
{
  Wrench W_new;
  W_new.force() = changeRefFrame(Worg.force(), X_parent_child);
  W_new.moment() = changeRefFrame(Worg.moment(), X_parent_child);
  return W_new;
}

/*!
 * \brief Transform State to a different coordinate frame.
 *
 * \param X_child
 *      State wrt original frame.
 *
 * \param T_parent_child
 *      HomogeneousTransformation from original frame to new coordinate frame.
 *
 * \return
 *      Transformed State wrt new coordinate frame.
 */
template <typename Derived>
inline Derived changeRefFrame(
  const ow_core::StateBase<Derived>& X_child,
  const HomogeneousTransformation& T_parent_child)
{
  Derived X_new;
  X_new.pos() = changeRefFrame(X_child.pos(), T_parent_child);
  X_new.vel() = changeRefFrame(X_child.vel(), T_parent_child);
  X_new.acc() = changeRefFrame(X_child.acc(), T_parent_child);
  X_new.effort() = changeRefFrame(X_child.effort(), T_parent_child);
  return X_new;
}

/*!
 * \brief Transform State to a different coordinate frame.
 *
 * \param X_child
 *      State wrt original frame.
 *
 * \param T_parent_child
 *      HomogeneousTransformation from original frame to new coordinate frame.
 *
 * \return
 *      Transformed State wrt new coordinate frame.
 */
template <typename Derived>
inline Derived changeRefFrame(
  const ow_core::StateBase<Derived>& X_child,
  const CartesianPosition& X_parent_child)
{
  Derived X_new;
  X_new.pos() = changeRefFrame(X_child.pos(), X_parent_child);
  X_new.vel() = changeRefFrame(X_child.vel(), X_parent_child);
  X_new.acc() = changeRefFrame(X_child.acc(), X_parent_child);
  X_new.effort() = changeRefFrame(X_child.effort(), X_parent_child);
  return X_new;
}

} // namespace ow

#endif // OPEN_WALKER_CORE_MATH_TRANSFORM_LINEAR_ACCELERATION_H
