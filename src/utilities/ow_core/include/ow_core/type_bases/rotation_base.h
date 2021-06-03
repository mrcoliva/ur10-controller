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


#ifndef OPEN_WALKER_CORE_RotationBase_BASE_H
#define OPEN_WALKER_CORE_RotationBase_BASE_H

#include <ow_core/conversions.h>

namespace ow_core
{
/*!
 * \brief The RotationBase class.
 */
template <typename _Derived>
class RotationBase
{
public:
  typedef _Derived Derived;

public:

 /*!
   * \brief Get accces to top level class.
   */
  Derived& derived()
  {
    return *static_cast<Derived*>(this);
  }

  /*!
   * \brief Get accces to top level class.
   */
  const Derived& derived() const
  {
    return *static_cast<const Derived*>(this);
  }

  /*!
   * \brief Assignment of Eigen::RotationBase
   *
  template <typename OtherDerived>
  RotationBase& operator=(const Eigen::RotationBase<OtherDerived, 3>& other)
  {
    derived() = other.toRotationMatrix();
    return *this;
  }*/

  /*!
   * \brief Assignment of tf::Quaternion.
   */
  RotationBase& operator=(const tf::Quaternion& q)
  {
    quaternionTFToEigen(q, derived());
    return *this;
  }

  /*!
   * \brief Assignment of geometry_msgs::Quaternion.
   */
  RotationBase& operator=(const geometry_msgs::Quaternion& q)
  {
    quaternionMsgToEigen(q, derived());
    return *this;
  }

  /*!
   * \brief Assignment of tf::Matrix3x3.
   */
  RotationBase& operator=(const tf::Matrix3x3& R)
  {
    matrixTFToEigen(R, derived());
    return *this; 
  }

  /*!
   * \brief Conversion to tf::Quaternion.
   */
  operator tf::Quaternion() const
  {
    tf::Quaternion q;
    eigenToQuaternionTF(derived(), q);
    return q;
  }

  /*!
   * \brief Conversion to geometry_msgs::Quaternion.
   */
  operator geometry_msgs::Quaternion() const
  {
    geometry_msgs::Quaternion q;
    eigenToQuaternionMsg(derived(), q);
    return q;
  }

  /*!
   * \brief Conversion to tf::Matrix3x3.
   */
  operator tf::Matrix3x3() const
  {
    tf::Matrix3x3 m;
    eigenToMatrixTF(derived(), m);
    return m;
  }

  /*!
   * \brief Conversion to tf::Quaternion.
   */
  tf::Quaternion toQuaternionTF()
  {
    return static_cast<tf::Quaternion>(derived());
  }

  /*!
   * \brief Conversion to geometry_msgs::Quaternion.
   */
  geometry_msgs::Quaternion toQuaternionMsg()
  {
    return static_cast<geometry_msgs::Quaternion>(derived());
  }

  /*!
   * \brief Conversion to tf::Matrix3x3.
   *
   */
  tf::Matrix3x3 toMatrixTF()
  {
    tf::Matrix3x3 m;
    eigenToMatrixTF(derived(), m);
    return m;
  }

};

}  // namespace ow_core

#endif  // OPEN_WALKER_CORE_MATRIX_BASE_H
