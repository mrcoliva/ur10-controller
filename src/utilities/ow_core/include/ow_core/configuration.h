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


#ifndef OPEN_WALKER_CORE_CONFIGURATION_H
#define OPEN_WALKER_CORE_CONFIGURATION_H


/*! \file configuration.h
 *  \brief Contains the global configurations.
 */

/*! \def OW_ROBOT_DOF
 *  \brief A definition to set degrees of freedom of the robot.
 */
#define OW_ROBOT_DOF                6

/*! \def OW_TYPES_SCALAR
 *  \brief A definition to set the scalar type for all type classes.
 */
#define OW_TYPES_SCALAR             double

/*! \def OW_VECTOR_DOF_SCALAR
 *  \brief A definition to set the scalar type of the VectorDof.
 *
 *  Either set it to OW_TYPES_SCALAR or a builtin type.
 *
 *  \note For now the system only supports a common type
 *  for OW_TYPES_SCALAR and OW_VECTOR_DOF_SCALAR.
 */
#define OW_VECTOR_DOF_SCALAR        OW_TYPES_SCALAR


/*! \def OW_VECTOR_DOF_ROWS
 *  \brief A definition to set the rows of the VectorDof type.
 *
 *  Either set it to OW_ROBOT_DOF or Eigen::Dynamic.
*/
#define OW_VECTOR_DOF_ROWS          OW_ROBOT_DOF

/*! \def OW_GRAVITY
 *  \brief Magnitude of gravity [m/s^2].
 */
#define OW_GRAVITY                  9.81


#endif  // OPEN_WALKER_CORE_CONFIGURATION_H
