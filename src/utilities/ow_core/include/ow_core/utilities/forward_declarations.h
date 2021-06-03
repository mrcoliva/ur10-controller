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


#ifndef OPEN_WALKER_CORE_FORWARD_DECLARATIONS_H
#define OPEN_WALKER_CORE_FORWARD_DECLARATIONS_H


/*! \file forward_declarations.h
 * \brief Contains all the forward declarations needed in the ow_core package.
 */



// Forward declarations for type classes in alphabetical order
//  for all classes in the types folder
namespace ow_core
{

template <typename _Scalar>
class AngularAcceleration;

template <typename _Scalar>
class AngularPosition;

template<typename _Scalar>
class AngularState;

template <typename _Scalar>
class AngularVelocity;

template <typename _Scalar>
class CartesianAcceleration;

template <typename _Scalar>
class CartesianPosition;

template<typename _Scalar>
class CartesianState;

template <typename _Scalar>
class CartesianVelocity;

template <typename _Scalar>
class SpatialVector;

template <typename _Scalar>
class CartesianVector;

template <typename _Scalar>
class Force;

template <typename _Scalar>
class HomogeneousTransformation;

template <typename _Scalar, int _Rows>
class JointAcceleration;

template <typename _Scalar, int _Rows>
class JointEffort;

template <typename _Scalar, int _Rows>
class JointPosition;

template<typename _Scalar, int _Rows>
class JointState;

template <typename _Scalar, int _Rows>
class JointVelocity;

template <typename _Scalar>
class LinearAcceleration;

template <typename _Scalar>
class LinearPosition;

template<typename _Scalar>
class LinearState;

template <typename _Scalar>
class LinearVelocity;

template <typename _Scalar>
class Moment;

template <typename _Scalar>
class Rotation3;

template <typename _Derived>
class StateBase;

template <typename _Scalar>
class Vector3;

template <typename _Scalar, int _Rows>
class VectorDof;

template <typename _Scalar, int _Rows, int _Cols>
class MatrixDof;

template <typename _Scalar, int _Cols>
class Jacobian;
template <typename _Derived, int _Cols>
class JacobianRef;

template <typename _Scalar>
class Wrench;

template <typename _Scalar>
class ZeroMomentPoint;

template <typename _Scalar>
class ZeroMomentPointAcceleration;

template<typename _Scalar>
class ZeroMomentPointState;

template <typename _Scalar>
class ZeroMomentPointVelocity;

template <typename _Derived>
class LinearPositionRef;

template <typename _Derived>
class LinearVelocityRef;

template <typename _Derived>
class LinearAccelerationRef;

template <typename _Derived>
class ForceRef;

template <typename _Derived>
class AngularPositionRef;

template <typename _Derived>
class AngularVelocityRef;

template <typename _Derived>
class AngularAccelerationRef;

template <typename _Derived>
class MomentRef;

template <typename _Derived, int _Rows, int _Cols>
class MatrixRef;

template <typename _Derived, int _Rows>
class VectorRef; 

template <typename _Derived, int _Rows>
class JointRef; 

template <typename _Derived>
class Rotation3Ref;

template <typename _Derived>
class QuaternionRef;

template <typename _Derived>
class JointStateRef;

template<typename _Scalar>
class DivergentComponentOfMotionState;

}




#endif // OPEN_WALKER_CORE_FORWARD_DECLARATIONS_H
