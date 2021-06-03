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


#ifndef OPEN_WALKER_CORE_TYPES_H
#define OPEN_WALKER_CORE_TYPES_H

/*! \file types.h
 *  \brief Contains all the types.
 */

// get the configurations to correctly define all types
#include <ow_core/configuration.h>

#include <ow_core/types/matrix.h>
#include <ow_core/types/rotation3.h>
#include <ow_core/types/vector3.h>

#include <ow_core/types/joint_state.h>
#include <ow_core/types/angular_state.h>
#include <ow_core/types/linear_state.h>
#include <ow_core/types/cartesian_state.h>

#include <ow_core/types/spatial_vector.h>
#include <ow_core/types/cartesian_vector.h>

#include <ow_core/types/jacobian.h>

#include <ow_core/types/homogeneous_transformation.h>

#include <ow_core/types/inertial_measurment_unit_sensor.h>
#include <ow_core/types/force_torque_sensor.h>

#include <ow_core/utilities/eigen_typedef_macros.h>

/*!
 * \brief The namespace for the configured types.
 */
namespace ow
{
  /*! Type for Scalars */
  typedef OW_TYPES_SCALAR Scalar;

  /*! Basic Types */
  typedef ow_core::Rotation3<Scalar> Rotation3;
  typedef ow_core::Vector3<Scalar> Vector3;

  typedef ow_core::AngularPosition<Scalar> AngularPosition;
  typedef ow_core::AngularVelocity<Scalar> AngularVelocity;
  typedef ow_core::AngularAcceleration<Scalar> AngularAcceleration;
  typedef ow_core::Moment<Scalar> Moment;
  typedef ow_core::AngularState<Scalar> AngularState;

  typedef ow_core::LinearPosition<Scalar> LinearPosition;
  typedef ow_core::LinearVelocity<Scalar> LinearVelocity;
  typedef ow_core::LinearAcceleration<Scalar> LinearAcceleration;
  typedef ow_core::Force<Scalar> Force;
  typedef ow_core::LinearState<Scalar> LinearState;

  typedef ow_core::HomogeneousTransformation<Scalar> HomogeneousTransformation;

  typedef ow_core::CartesianPosition<Scalar> CartesianPosition;
  typedef ow_core::CartesianVelocity<Scalar> CartesianVelocity;
  typedef ow_core::CartesianAcceleration<Scalar> CartesianAcceleration;
  typedef ow_core::Wrench<Scalar> Wrench;
  typedef ow_core::CartesianState<Scalar> CartesianState;

  typedef ow_core::SpatialVector<Scalar> SpatialVector;
  typedef ow_core::CartesianVector<Scalar> CartesianVector;

  typedef ow_core::InertialMeasurmentUnitSensor<Scalar> ImuSensor;
  typedef ow_core::ForceTorqueSensor<Scalar> FTSensor;

  /* Robot Specific Types */
  typedef ow_core::Matrix<OW_VECTOR_DOF_SCALAR,OW_VECTOR_DOF_ROWS,1> VectorDof;
  typedef ow_core::Matrix<OW_VECTOR_DOF_SCALAR,OW_VECTOR_DOF_ROWS,OW_VECTOR_DOF_ROWS> MatrixDof;
  typedef ow_core::JointPosition<OW_VECTOR_DOF_SCALAR,OW_VECTOR_DOF_ROWS> JointPosition;
  typedef ow_core::JointVelocity<OW_VECTOR_DOF_SCALAR,OW_VECTOR_DOF_ROWS> JointVelocity;
  typedef ow_core::JointAcceleration<OW_VECTOR_DOF_SCALAR,OW_VECTOR_DOF_ROWS> JointAcceleration;
  typedef ow_core::JointEffort<OW_VECTOR_DOF_SCALAR,OW_VECTOR_DOF_ROWS> JointEffort;
  typedef ow_core::JointState<OW_VECTOR_DOF_SCALAR,OW_VECTOR_DOF_ROWS> JointState;

  // typedef Eigen::Matrix<Scalar,6,OW_VECTOR_DOF_ROWS> Jacobian;
  typedef ow_core::Jacobian<Scalar, OW_VECTOR_DOF_ROWS> Jacobian;

  typedef ow_core::JointState<OW_VECTOR_DOF_SCALAR> JointStateX;
  typedef ow_core::JointPosition<OW_VECTOR_DOF_SCALAR> JointPositionX;
  typedef ow_core::JointVelocity<OW_VECTOR_DOF_SCALAR> JointVelocityX;
  typedef ow_core::JointAcceleration<OW_VECTOR_DOF_SCALAR> JointAccelerationX;
  typedef ow_core::JointEffort<OW_VECTOR_DOF_SCALAR> JointEffortX;

  /*! Eigen specific types */
  OPEN_WALKER_CORE_TYPEDEFS_ALL_SIZES(unsigned int, ui)
  OPEN_WALKER_CORE_TYPEDEFS_ALL_SIZES(int, i)

  typedef ow_core::Matrix<Scalar, 6, 1> Vector6;
  typedef ow_core::Matrix<Scalar, 7, 1> Vector7;
  typedef ow_core::Matrix<Scalar, 2, 1> Vector2;

  typedef ow_core::Matrix<Scalar, 3, 3> Matrix3;
  typedef ow_core::Matrix<Scalar, 4, 4> Matrix4;
  typedef ow_core::Matrix<Scalar, 6, 6> Matrix6;

  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
}

#endif  // OPEN_WALKER_CORE_TYPES_H
