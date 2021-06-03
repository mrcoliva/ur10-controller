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


#ifndef OPEN_WALKER_CORE_CARTESIAN_POSITION_REF_H
#define OPEN_WALKER_CORE_CARTESIAN_POSITION_REF_H

#include <ow_core/utilities/type_guard.h>
#include <ow_core/type_references/angular_position_ref.h>
#include <ow_core/type_references/linear_position_ref.h>
#include <geometry_msgs/Pose.h>

#include <ow_core/type_bases/cartesian_base.h>

namespace ow{

/*!
 * \brief The traits class for the CartesianAcceleration class.
 *
 * This class contains the typedefs and enums for
 * the CartesianAcceleration class.
 *
 * These typedefs and enums are used to define and describe
 * the properties (traits) of this class.
 */
template<typename _Scalar>
struct traits<ow_core::CartesianPosition<_Scalar> >
{
  typedef Eigen::Matrix<_Scalar, 7, 1> Base;
  typedef ow_core::LinearPositionRef<Base> LinearRef;
  typedef ow_core::AngularPositionRef<Base> AngularRef;
  typedef ow_core::LinearPositionRef<const Base> CLinearRef;
  typedef ow_core::AngularPositionRef<const Base> CAngularRef;
  enum {
    LinearPartAtCompileTime = 0,
    AngularPartCompileTime = 3,
  };
};

} // namespace ow

namespace ow_core
{
/*!
 * \brief The CartesianPosition class.
 *
 * The CartesianPosition is of type Eigen::Vector7 and is
 * represented by the math symbol \f$\mathbf{X}\f$.
 *
 * Stores the position and orientation information
 * in a 7 dimensional vector. The position part is represented
 * by the first three elements. The orientation as a quaternion
 * by the last four elements.
 *
 */
template <typename _Scalar>
class CartesianPosition :
    public Eigen::Matrix<_Scalar, 7, 1>,
    public CartesianBase<CartesianPosition<_Scalar> >,
    TypeGuard
{
  OW_TYPE_GUARD(CartesianPosition)
public:
  typedef _Scalar Scalar;
  typedef Eigen::Matrix<Scalar, 7, 1> Base;
  typedef CartesianBase<CartesianPosition<_Scalar> > CBase;
  typedef Eigen::Transform<Scalar, 3, Eigen::Affine> Transform;

  /*!
   * \brief Construct as Identity.
   * 
   * \note CartesianPosition saved as [x,y,z,qx,qy,qz,qw]
   */
  static const CartesianPosition& Identity()
  {
    static const CartesianPosition v = (Base() << 0,0,0,0,0,0,1).finished();
    return v;
  }

  /*!
   * \brief Construct as Zero.
   * 
   * \note zero means identity orientation part
   * \note CartesianPosition saved as [x,y,z,qx,qy,qz,qw]
   */
  static const CartesianPosition& Zero()
  {
    static const CartesianPosition v = (Base() << 0,0,0,0,0,0,1).finished();
    return v;
  }

  /*!
   * \brief Construct as Default
   *
   * Default is Identity
   */
  static const CartesianPosition& Default()
  {
    return Identity();
  }

public:
  /*!
   * \brief Default Constructor
   */
  CartesianPosition()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  CartesianPosition(const CartesianPosition& other) :
    Base(other)
  {
  }

  /*!
   * \brief Copy Constructor
   */
  CartesianPosition(const CBase& other) :
    CBase(other)
  {
  }

  /*!
   * \brief Constructor from Scalars
   */
  CartesianPosition(const Scalar& x, const Scalar& y, const Scalar& z, 
                    const Scalar& qw, const Scalar& qx, 
                    const Scalar& qy, const Scalar& qz)
  {
    position().x() = x;
    position().y() = y;
    position().z() = z;
    orientation().w() = qw;
    orientation().x() = qx;
    orientation().y() = qy;
    orientation().z() = qz;
  }

  /*!
   * \brief Copy constructor.
   *
   * This copy constructor not only works with Eigen matrices
   * but also with their expressions.
   */
  template <typename OtherDerived>
  CartesianPosition(const Eigen::EigenBase<OtherDerived>& other) :
    Base(other.derived())
  {
  }

  /*!
   * \brief Copy constructor from Eigen::Transform.
   */
  CartesianPosition(const Transform& other)
  {
    operator=(other);
  }

  /*!
   * \brief Copy constructor HomogeneousTransformation.
   */
  explicit CartesianPosition(const HomogeneousTransformation<Scalar>& other)
  {
    operator=(other);
  }

  /*!
   * \brief Copy constructor from Eigen::Translation.
   *
   * This Constructor sets the rotational part to identity.
   */
  explicit CartesianPosition(const typename Transform::TranslationType& other)
  {
    operator=(other);
  }

  /*!
   * \brief Copy constructor from Eigen::RotationBase.
   *
   * This Constructor sets the translational part to zero.
   */
  template <typename OtherDerived>
  explicit CartesianPosition(const Eigen::RotationBase<OtherDerived, 3>& other)
  {
    operator=(other);
  }

  /*!
   * \brief Copy constructor from tf::Transform.
   *
   */
  explicit CartesianPosition(const tf::Transform& other)
  {
    operator=(other);
  }

  /*!
   * \brief Copy constructor from geometry_msgs::Pose.
   *
   */
  explicit CartesianPosition(const geometry_msgs::Pose& other)
  {
    operator=(other);
  }

  /*!
   * \brief set to identity.
   */
  void setIdentity()
  {
    CBase::linear().setZero();
    CBase::angular().setIdentity();
  }

  /*!
   * \brief set to identity.
   */
  void setZero()
  {
    setIdentity();
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;
  using CBase::operator=;

  /*!
   * \brief Assignment operator.
   */
  CartesianPosition& operator=(const CartesianPosition& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment from ow::HomogenousTransformation.
   */
  void operator=(const HomogeneousTransformation<Scalar>& T)
  {
    // use: operator=(const Transform& T)
    operator=(static_cast<const Transform&>(T));
  }

  /*!
   * \brief Assignment from Eigen::Transformation.
   *
   */
  void operator=(const Transform& T)
  {
    CBase::linear() = T.translation();
    CBase::angular() = T.linear();
  }

  /*!
   * \brief Assignment from tf::Transform.
   *
   */
  void operator=(const tf::Transform& T)
  {
    CBase::linear() = T.getOrigin();
    CBase::angular() = T.getRotation();
  }

  /*!
   * \brief Assignment from geometry_msgs::Pose
   *
   */
  void operator=(const geometry_msgs::Pose& T)
  {
    CBase::linear() = T.position;
    CBase::angular() = T.orientation;
  }

  /*!
   * \brief Assignment from Eigen::RotationBase.
   *
   */
  template <typename OtherDerived>
  void operator=(const Eigen::RotationBase<OtherDerived, 3>& R)
  {
    CBase::linear().setZero();
    CBase::angular() = R.toRotationMatrix();
  }

  /*!
   * \brief Assignment from Eigen::Translation.
   *
   */
  void operator=(const typename Transform::TranslationType& t)
  {
    CBase::linear() = t.vector();
    CBase::angular().setIdentity();
  }

  /*!
   * \brief Conversion to Eigen::Transformation.
   *
   */
  operator Transform() const
  {
    return typename 
      Transform::TranslationType(CBase::linear())*CBase::angular();
  }

  /*!
   * \brief Conversion to tf::Transform.
   *
   */
  operator tf::Transform() const
  {
    return 
      tf::Transform(CBase::angular().toQuaternionTF(), CBase::linear());
  }

  /*!
   * \brief Conversion to geometry_msgs::Pose.
   *
   */
  operator geometry_msgs::Pose() const
  {
    geometry_msgs::Pose X;
    X.position = CBase::linear();
    X.orientation = CBase::angular();
    return X;
  }

  /*!
   * \brief Conversion to Eigen::Transformation.
   *
   */
  Transform toTransformEigen() const
  {
    return static_cast<Transform>(*this);
  }

  /*!
   * \brief Conversion to tf::Transform.
   *
   */
  tf::Transform toTransformTf() const
  {
    return static_cast<tf::Transform>(*this);
  }

  /*!
   * \brief Conversion to geometry_msgs::Pose.
   *
   */
  geometry_msgs::Pose toPoseMsg() const
  {
    return static_cast<geometry_msgs::Pose>(*this);
  }

  /*!
   * \brief access to position part
   */
  LinearPositionRef<Base> position()
  {
    return CBase::linear();
  }

  /*!
   * \brief access to position part
   */
  LinearPositionRef<Base> pos()
  {
    return CBase::linear();
  }

  /*!
   * \brief const access to position part
   */
  LinearPositionRef<const Base> position() const
  {
    return CBase::linear();
  }

  /*!
   * \brief const access to position part
   */
  LinearPositionRef<const Base> pos() const
  {
    return CBase::linear();
  }

  /*!
   * \brief access to orientation part
   */
  AngularPositionRef<Base> orientation()
  {
    return CBase::angular();
  }

  /*!
   * \brief const access to orientation part
   */
  AngularPositionRef<const Base> orientation() const
  {
    return CBase::angular();
  }

  /*!
   * \brief access to orientation part
   */
  AngularPositionRef<Base> orien()
  {
    return CBase::angular();
  }

  /*!
   * \brief const access to orientation part
   */
  AngularPositionRef<const Base> orien() const
  {
    return CBase::angular();
  }

  /*!
   * \brief return the inverse cartesian position.
   */
  CartesianPosition inverse() const
  {
    CartesianPosition res;
    res.angular() = CBase::angular().inverse();
    res.linear() = Scalar(-1)*(res.angular()*CBase::linear());
    return res;
  }

  /*!
   * \brief Multiply two cartesian positions.
   */
  CartesianPosition& operator*=(const CartesianPosition& other) 
  { 
    return *this = *this * other; 
  }

  /*!
   * \brief Multiply two cartesian positions.
   */
  CartesianPosition operator*(const CartesianPosition& other) const
  {
    CartesianPosition res;
    res.angular() = CBase::angular()*other.angular();
    res.position() = CBase::angular()*other.linear() + CBase::linear();
    return res;
  }

  /*!
   * \brief Multiply with point to transform between frames.
   * 
   * Rotates and translates a point according to this cartesian position.
   */
  template <typename OtherDerived>
  ow_core::Vector3<Scalar> operator*(const ow_core::Vector3Base<OtherDerived>& other) const
  {
    return CBase::angular()*other.derived() + CBase::linear();
  }

private:
  using Base::operator+;
  using Base::operator+=;
  using Base::operator-;
  using Base::operator-=;

};

}  // namespace ow_core

#endif  // OPEN_WALKER_CORE_CARTESIAN_POSITION_REF_H
