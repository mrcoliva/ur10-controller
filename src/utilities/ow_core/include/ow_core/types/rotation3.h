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


#ifndef OPEN_WALKER_CORE_ROTATION3_H
#define OPEN_WALKER_CORE_ROTATION3_H

#include <ow_core/utilities/type_guard.h>
#include <ow_core/type_bases/rotation3_base.h>

#include <ow_core/math/random_orientation.h>

namespace ow_core
{
/*!
 * \brief The Rotation3 class.
 *
 * The Rotation3 is of type Eigen::Matrix3 and is
 * represented by the math symbol \f$\mathbf{R}\f$.
 *
 */
template <typename _Scalar>
class Rotation3 :
  public Eigen::Matrix<_Scalar, 3, 3>,
  public Rotation3Base<Rotation3<_Scalar> >,
  TypeGuard
{
  OW_TYPE_GUARD(Rotation3)

public:
  typedef _Scalar Scalar;
  typedef Eigen::Matrix<Scalar, 3, 3> Base;
  typedef Rotation3Base<Rotation3<Scalar> > RBase;
  typedef Eigen::AngleAxis<Scalar> AngleAxis;

  /*!
   *  \brief Rotation around x-axis for given angle in radian.
   * 
   *  Returns a Rotation Matrix constructed from roll angle.
   */
  static Rotation3 Rx(const Scalar& angle = 0.0)
  {
    return Rotation3(AngleAxis(angle,Eigen::Vector3d::UnitX()));
  }

  /*!
   *  \brief Rotation around y-axis for given angle in radian.
   * 
   *  Returns a Rotation Matrix constructed from pitch angle.
   */
  static Rotation3 Ry(const Scalar& angle = 0.0)
  {
    return Rotation3(AngleAxis(angle,Eigen::Vector3d::UnitY()));
  }

  /*!
   *  \brief Rotation around y-axis for given angle in radian.
   * 
   *  Returns a Rotation Matrix constructed from yaw angle.
   */
  static Rotation3 Rz(const Scalar& angle = 0.0)
  {
    return Rotation3(AngleAxis(angle,Eigen::Vector3d::UnitZ()));
  }

  /*!
   *  \brief Rotation from RPY angles.
   * 
   *  Returns a Rotation Matrix constructed from roll pitch and yaw euler angles
   */
  static Rotation3 RPY(const Scalar& roll = 0.0,
                       const Scalar& pitch = 0.0,
                       const Scalar& yaw = 0.0)
  {
    return Rotation3(
      AngleAxis(roll, Eigen::Vector3d::UnitX())*
      AngleAxis(pitch, Eigen::Vector3d::UnitY())*
      AngleAxis(yaw, Eigen::Vector3d::UnitZ()));
  }

  /*!
   *  \brief Rotation from RPY angles.
   * 
   *  Returns a Rotation Matrix constructed from roll pitch and yaw euler angles
   */
  static Rotation3 RPY(const ow_core::Vector3<Scalar>& rpy =
      ow_core::Vector3<Scalar>::Zero())
  {
    return RPY(rpy.x(),rpy.y(), rpy.z());
  }

  /*!
  * \brief Generate a random uniformly sampled Rotation3 in SO(3).
  * 
  * Usage: 
  *   std::random_device rd;
  *   ow::Rotation3 R = ow::Rotation3::Random(rd)
  */
  template<typename Generator>
  static Rotation3 Random(Generator& generator)
  {
    return ow::uniformRandomRotation<Scalar>(generator);
  }

protected:

public:
  /*!
   * \brief Default Constructor.
   */
  Rotation3()
  {
  }

  /*!
   * \brief Copy Constructor.
   */
  Rotation3(const Rotation3& other) :
    Base(other)
  {
  }

  /*!
   *  \brief Copy constructor.
   *
   * This copy constructor not only works with Eigen matrices
   * but also with their expressions.
   */
  template <typename OtherDerived>
  Rotation3(const Eigen::EigenBase<OtherDerived>& other) :
    Base(other)
  {
  }

  /*!
   *  \brief Copy constructor.
   *
   * This copy constructor works with Eigen Rotation representations.
   */
  template <typename OtherDerived>
  explicit Rotation3(const Eigen::RotationBase<OtherDerived, 3>& other) : 
    Base(other.toRotationMatrix())
  {
  }

  /*!
   * \brief Copy from AngularPosition
   */
  explicit Rotation3(const ow_core::AngularPosition<Scalar>& other) :
    Base(other)
  {
  }

  /*!
   * \brief Copy from AngularPositionRef
   */
  template <typename OtherDerived>
  explicit Rotation3(const ow_core::AngularPositionRef<OtherDerived>& other) :
    Base(other)
  {
  }

  /*!
   * \brief Use assignment operators of RBase class.
   */
  using Base::operator=;
  using RBase::operator=;
  
  /*!
   * \brief Assignment operator.
   */
  Rotation3& operator=(const Rotation3& other)
  {
    RBase::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment operator.
   */
  template<typename OtherDerived>
  Rotation3& operator=(const Rotation3Ref<OtherDerived>& other)
  {
    Base::operator=(other);
    return *this;
  }

  Rotation3& operator=(const ow_core::AngularPosition<Scalar>& other)
  {
    Base::operator=(other);
    return *this;
  }

  template <typename OtherDerived>
  Rotation3& operator=(const ow_core::AngularPositionRef<OtherDerived>& other)
  {
    Base::operator=(other);
    return *this;
  }

};

}  // namespace ow_core

#endif  // OPEN_WALKER_CORE_ROTATION3_H
