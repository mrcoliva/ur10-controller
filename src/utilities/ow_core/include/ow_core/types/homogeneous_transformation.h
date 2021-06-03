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


#ifndef OPEN_WALKER_CORE_HOMOGENEOUS_TRANSFORMATION_H
#define OPEN_WALKER_CORE_HOMOGENEOUS_TRANSFORMATION_H

#include <ow_core/utilities/type_guard.h>
#include <ow_core/type_references/rotation3_ref.h>
#include <ow_core/type_references/linear_position_ref.h>

namespace ow_core
{
/*!
 * \brief The HomogeneousTransformation class.
 *
 * The HomogeneousTransformation is of type Eigen::Transform and is
 * represented by the math symbol \f$\mathbf{T}\f$.
 *
 * Stores the position and orientation information
 * in a 4x4 dimensional matrix. The orientation part is represented as a 3x3 matrix
 * in the upper left corner. The position part is represented as a 3 dimensional vector
 * in the upper right corner.
 *
 */
template <typename _Scalar>
class HomogeneousTransformation : 
    public Eigen::Transform<_Scalar, 3, Eigen::Affine>,
    TypeGuard
{
  OW_TYPE_GUARD(HomogeneousTransformation)

public:
  typedef _Scalar Scalar;
  typedef Eigen::Transform<_Scalar, 3, Eigen::Affine> Base;
  typedef typename Eigen::Transform<_Scalar, 3, Eigen::Affine>::MatrixType Matrix;

  /*!
   * \brief Default HomogeneousTransformation.
   *
   *  constructs as identity
   */
  static const HomogeneousTransformation<Scalar>& Default()
  {
    static const HomogeneousTransformation v = Matrix::Identity().matrix();
    return v;
  }

public:
  /*!
   * \brief Default Constructor
   */
  HomogeneousTransformation() :
    Base()
  {
  }

  /*! 
   * \brief Copy constructor from Eigen::Affine.
   */
  HomogeneousTransformation(const HomogeneousTransformation& other) :
    Base(other)
  {
  }

  /*! 
   * \brief Copy constructor from Eigen::Affine.
   *
   * \todo removed explicit for: HomogeneousTransformation T = Eigen::Affine()
   */
  HomogeneousTransformation(const Base& other) :
    Base(other)
  {
  }

  /*!
   * \brief Copy constructor OtherDerived.
   *
   * This copy constructor not only works with Eigen matrices
   * but also with their expressions.
   */
  template <typename OtherDerived>
  HomogeneousTransformation(const Eigen::EigenBase<OtherDerived>& other) :
    Base(other.derived())
  {
  }

  /*!
   * \brief Copy constructor from Eigen::Transform::Translation.
   *
   */
  explicit HomogeneousTransformation(const CartesianPosition<Scalar>& other)
  {
    operator=(other);
  }

  /*!
   * \brief Copy constructor from Eigen::Transform::Translation.
   *
   */
  explicit HomogeneousTransformation(const typename Base::TranslationType& other) :
    Base(other)
  {
  }

  /*!
   * \brief Copy constructor from Eigen::RotationBase.
   *
   */
  template <typename OtherDerived>
  explicit HomogeneousTransformation(const Eigen::RotationBase<OtherDerived, 3>& other) :
    Base(other)
  {
  }

  /*!
   * \brief Copy constructor from tf::Transform.
   *
   */
  explicit HomogeneousTransformation(const tf::Transform& other)
  {
    operator=(other);
  }

  /*!
   * \brief Copy constructor from geometry_msgs::Pose.
   *
   */
  explicit HomogeneousTransformation(const geometry_msgs::Pose& other)
  {
    operator=(other);
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;

  /*!
   * \brief Assignment from CartesianPosition.
   */
  void operator=(const CartesianPosition<Scalar>& other)
  {
    position() = other.position();
    orientation() = other.orientation().toRotationMatrix();
  }

  /*!
   * \brief Assignment from tf::Transform.
   */
  void operator=(const tf::Transform& other)
  {
    position() = other.getOrigin();
    orientation() = other.getRotation();
  }

  /*!
   * \brief Assignment from geometry_msgs::Pose.
   */
  void operator=(const geometry_msgs::Pose& other)
  {
    position() = other.position;
    orientation() = other.orientation;
  }

  /*!
   * \brief Conversion to tf::Transform.
   */
  operator tf::Transform() const
  {
    return tf::Transform(orientation().toQuaternionTF(), position());
  }

  /*!
   * \brief Conversion to geometry_msgs::Pose.
   */
  operator geometry_msgs::Pose() const
  {
    geometry_msgs::Pose X;
    X.position = position();
    X.orientation = orientation();
    return X;
  }

  /*!
   * \brief Conversion to tf::Transform.
   */
  tf::Transform toTransformTf()
  {
    return static_cast<tf::Transform>(*this);
  }

  /*!
   * \brief Conversion to geometry_msgs::Pose.
   */
  geometry_msgs::Pose toPoseMsg()
  {
    return static_cast<geometry_msgs::Pose>(*this);
  }

  /*!
   * \brief access to position part
   */
  LinearPositionRef<Matrix> position()
  {
    return LinearPositionRef<Matrix>(Base::matrix(),0,3);
  }

  /*!
   * \brief access to position part
   */
  LinearPositionRef<Matrix> pos()
  {
    return LinearPositionRef<Matrix>(Base::matrix(),0,3);
  }

  /*!
   * \brief const access to position part
   */
  LinearPositionRef<const Matrix> position() const
  {
    return LinearPositionRef<const Matrix>(Base::matrix(),0,3);
  }

  /*!
   * \brief const access to position part
   */
  LinearPositionRef<const Matrix> pos() const
  {
    return LinearPositionRef<const Matrix>(Base::matrix(),0,3);
  }

  /*!
   * \brief access to orientation part
   */
  Rotation3Ref<Matrix> orientation()
  {
    return Rotation3Ref<Matrix>(Base::matrix(),0,0);
  }

  /*!
   * \brief access to orientation part
   */
  Rotation3Ref<Matrix> orien()
  {
    return Rotation3Ref<Matrix>(Base::matrix(),0,0);
  }

  /*!
   * \brief const access to orientation part
   */
  Rotation3Ref<const Matrix> orientation() const
  {
    return Rotation3Ref<const Matrix>(Base::matrix(),0,0);
  }

  /*!
   * \brief const access to orientation part
   */
  Rotation3Ref<const Matrix> orien() const
  {
    return Rotation3Ref<const Matrix>(Base::matrix(),0,0);
  }

  /*!
   * \brief Conversion to std::string.
   */
  std::string toString() const
  {
    std::ostringstream out;
    out << Base::matrix();
    return out.str();
  }
};

}  // namespace ow_core

#endif  // OPEN_WALKER_CORE_HOMOGENEOUS_TRANSFORMATION_H
