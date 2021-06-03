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


#ifndef OPEN_WALKER_CORE_STATE_TRAJECTORY_H
#define OPEN_WALKER_CORE_STATE_TRAJECTORY_H

#include <ros/assert.h>

#include <ow_core/utilities/type_not_assignable.h>
#include <ow_core/types.h>
#include <ow_core/math/quaternion.h>
#include <ow_core/trajectory/polynomial_trajectory.h>

namespace ow_core{


/*!
 * \brief The StateTrajectory class.
 *
 * This class interpolates a state through a set of support points
 * The result is a trajectory with state(t)=[pos(t), vel(t), acc(t)].
 *
 * The _Derived template parameter has to be derived from
 * ow_core::StateBase thus belong to the group of
 * state type classes.
 * 
 * Usage:
 * 
 *  // define time segments between 0 and 10 sec
 *  std::vector<ow::Scalar> segments = {0, 1, 3, 5, 6, 9, 10};  
 * 
 *  // define support points (here Quaternion Orientations)
 *  std::vector<ow::AngularPosition> c_samples = {
 *    ow::AngularPosition(1, 0, 0, 0),
 *    ow::AngularPosition::Zero(),  // support point
 *    ow::AngularPosition(0.7042004, 0, 0.7100013, 0),
 *    ow::AngularPosition(0.4388107, 0, 0.6353917, 0.6353917),
 *    ow::AngularPosition(0.37845, 0.3052338, 0.6179029, 0.6179029),
 *    ow::AngularPosition::Zero(),  // support point
 *    ow::AngularPosition(1, 0, 0, 0)
 *  };
 * 
 *  // create state spline
 *  ow_core::StateTrajectory<ow::AngularState> 
 *     state_spline(ow_core::CubicSpline(segments, c_samples));
 * 
 *  // interpolate the state
 *  std::vector<ow::AngularState> result = state_traj.evaluate(time_vec);
 * 
 */
template <typename _Derived>
class StateTrajectory
{
  OW_TYPE_NOT_ASSIGNABLE(StateTrajectory)

public:
  typedef _Derived Derived;
  typedef std::vector<Derived> DerivedVec;

  typedef typename ow::traits<Derived>::Scalar Scalar;
  typedef std::vector<Scalar> ScalarVec;

  typedef typename ow::traits<Derived>::Pos V;
  typedef typename ow::traits<Derived>::Vel VP;
  typedef typename ow::traits<Derived>::Acc VPP;
  typedef typename ow::traits<Derived>::Effort VF;

  typedef PolynomialTrajectory<V, Scalar> Trajectory;
  typedef typename Trajectory::ValueVec VVec;
  typedef typename Trajectory::ValueMatrix VMatrix;

protected:
  /*!
   * \brief The trajectory with position type.
   */
  Trajectory trajectory_;

public:
  /*!
   * \brief Constructor.
   *
   * Takes a constructed trajectory object as input.
   */
  StateTrajectory(const Trajectory& trajectory) :
    trajectory_(trajectory)
  {
  }

  /*!
   * \brief Deconstructor.
   */
  ~StateTrajectory()
  {
  }

  /*!
   * \brief get end time of the trajectory
   */
  Scalar startTime() const
  {
    return trajectory_.startTime();
  }

  /*!
   * \brief get end time of a segment
   */
  Scalar endTime() const
  {
    return trajectory_.endTime();
  }

  /*!
   * \brief evaluate trajectory at a single time t
   *  
   * \return The state [pos, vel, acc] at time t
   */
  Derived evaluate(const Scalar& t) const
  {
    // compute the two derivatives
    VVec vv = trajectory_.evaluateAll(t, 2);

    Derived state;

    convertToPosition(state.pos(), vv[2]);

    convertToAcceleration(
      state.acc(),
      vv[0],          // acceleration
      state.pos());   // position

    convertToVelocity(
      state.vel(),
      vv[1],          // velocity
      state.pos());   // position

    return state;
  }

  /*!
   * \brief evaluate trajectory at at all times t_vec
   * 
   * \return A vector of values with the solution at all t in t_vec.
   */
  DerivedVec evaluate(const ScalarVec& t_vec)
  {
    VMatrix vmat = trajectory_.evaluateAll(t_vec, 2);
    DerivedVec states(vmat.size());

    for(int i = 0; i < states.size(); ++i)
    {
      Derived& state = states[i];
      VVec& vv = vmat[i];

      convertToPosition(state.pos(), vv[2]);

      convertToAcceleration(
        state.acc(),
        vv[0],          // acceleration
        state.pos());   // position

      convertToVelocity(
        state.vel(),
        vv[1],          // velocity
        state.pos());   // position
    }
    return states;
  }

protected:
  /*!
  * \brief Conversion to Position Type from Interpolated values.
  * 
  * This function covers the general case for which the position type is 
  * directly given by the interpolated values
  */ 
  template <typename Derived1, typename Derived2>
  Eigen::MatrixBase<Derived1>& convertToPosition(
    Eigen::MatrixBase<Derived1>& pos,
    const Eigen::MatrixBase<Derived2>& value) const
  {
    pos = value;
    return pos;
  };

  /*!
  * \brief Conversion to Position Type from Interpolated values.
  * 
  * This function covers the AngularPosition case for which the quaternion is 
  * given by the normalized interpolated values.
  */ 
  template <typename Derived1>
  ow::AngularPosition& convertToPosition(
    ow::AngularPosition& Q,
    const ow_core::AngularPositionBase<Derived1>& value) const
  {
    Q = value;
    Q.normalize();
    return Q;
  };

  /*!
  * \brief Conversion of Position Type to Velocity Type
  * 
  * This function covers the general case for which the velocity is 
  * directly given by the time derivative of the position.
  */
  template <typename Derived1, typename Derived2, typename Derived3>
  Eigen::MatrixBase<Derived1>& convertToVelocity(
    Eigen::MatrixBase<Derived1>& vel,
    const Eigen::MatrixBase<Derived2>& posP,
    const Eigen::MatrixBase<Derived3>& pos) const
  {
    vel = posP;
    return vel;
  };

  /*!
  * \brief Conversion of Position Type to Velocity Type
  * 
  * This function covers the special case for AngularPositions to 
  * AngularVelocities. This requires the time derivative of the
  * AngularPosition and the current AngularPosition.
  */
  template <typename Derived1, typename Derived2>
  ow::AngularVelocity& convertToVelocity(
    ow::AngularVelocity& omega,
    const ow_core::AngularPositionBase<Derived1>& QP,
    const ow_core::AngularPositionBase<Derived2>& Q) const
  {
    ow::quaternion2AngularVelocityInertial(omega, QP, Q);
    return omega;
  };

  /*!
  * \brief Conversion of Position Type to Velocity Type
  * 
  * This function covers the special case for CartesianPositions to 
  * CartesianVelocities. This requires the time derivative of the
  * CartesianPosition and the current CartesianPosition.
  */
  ow::CartesianVelocity& convertToVelocity(
    ow::CartesianVelocity& vel,
    const ow::CartesianPosition& XP,
    const ow::CartesianPosition& X) const
  {
    vel.linear() = static_cast<const ow_core::Vector3<Scalar>& >(
      XP.position());
    ow::quaternion2AngularVelocityInertial(
      vel.angular(), 
      XP.orientation(), 
      X.orientation());
    return vel;
  };

  /*!
  * \brief Conversion of Velocity Type to Acceleration Type
  * 
  * This function covers the general case for which the accelearation is 
  * directly given by the time derivative of the velocity.
  */
  template <typename Derived1, typename Derived2, typename Derived3>
  inline Eigen::MatrixBase<Derived1>& convertToAcceleration(
    Eigen::MatrixBase<Derived1>& acc,
    const Eigen::MatrixBase<Derived2>& velP,
    const Eigen::MatrixBase<Derived3>& pos) const
  {
    acc = velP;
    return acc;
  };

  /*!
  * \brief Conversion of Velocity Type to Acceleration Type
  * 
  * This function covers the special case for AngularPositions to 
  * AngularVelocities. This requires the time derivative of the
  * AngularPosition and the current AngularPosition.
  */
  template <typename Derived1, typename Derived2>
  ow::AngularAcceleration& convertToAcceleration(
    ow::AngularAcceleration& alpha,
    const ow_core::AngularPositionBase<Derived1>& QPP,
    const ow_core::AngularPositionBase<Derived2>& Q) const
  {
    ow::quaternion2AngularAccelerationInertial(alpha, QPP, Q);
    return alpha;
  };

  /*!
  * \brief Conversion of Velocity Type to Acceleration Type
  * 
  * This function covers the special case for CartesianPositions to 
  * CartesianVelocities. This requires the time derivative of the
  * CartesianPosition and the current CartesianPosition.
  */
  inline ow::CartesianAcceleration& convertToAcceleration(
    ow::CartesianAcceleration& acc,
    const ow::CartesianPosition& XPP,
    const ow::CartesianPosition& X) const
  {
    acc.linear() = static_cast<const ow_core::Vector3<Scalar>& >(
      XPP.position());
    ow::quaternion2AngularAccelerationInertial(
      acc.angular(), 
      XPP.orientation(), 
      X.orientation());
    return acc;
  };

};

}

#endif // OPEN_WALKER_CORE_STATE_DERIVATIVES_UPDATER_H
