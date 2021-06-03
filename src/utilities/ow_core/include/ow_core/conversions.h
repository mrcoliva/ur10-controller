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


#ifndef OPEN_WALKER_CORE_CONVERSIONS_H
#define OPEN_WALKER_CORE_CONVERSIONS_H

#include <Eigen/Dense>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

#include <ow_msgs/Vector.h>

/*! \file conversions.h
 *  \brief Contains global conversion functions.
 *
 *  Contains all the global conversion functions to convert between:
 *      - the OpenWalker types
 *      - the Eigen types
 *      - the ROS tf types
 *      - the ROS geometry_msgs types
 */
namespace ow_core
{

/*!
 *  \brief Converts an Eigen::Matrix to a std::string.
 */
template <typename _Derived>
std::string eigenToString(const Eigen::MatrixBase<_Derived>& e)
{
  std::ostringstream out;
  out << e;
  return out.str();
}

/*!
 *  \brief Converts a tf::Matrix3x3 into an Eigen::Quaterniond.
 */
void matrixTFToEigen(const tf::Matrix3x3& t, Eigen::Quaterniond& e);

/*!
 *  \brief Converts a tf::Matrix3x3 into an Eigen::QuaternionBase.
 */
template <typename _Derived>
void matrixTFToEigen(const tf::Matrix3x3& t, Eigen::QuaternionBase<_Derived>& e)
{
  typedef typename Eigen::internal::traits<_Derived>::Scalar Scalar;
  Eigen::Quaterniond ed;
  matrixTFToEigen(t, ed);
  e = ed.cast<Scalar>();
}

/*!
 *  \brief Converts a tf::Matrix3x3 into an Eigen::Matrix3.
 */
template <typename _Derived>
void matrixTFToEigen(const tf::Matrix3x3& t, Eigen::MatrixBase<_Derived>& e)
{
  typedef typename Eigen::internal::traits<_Derived>::Scalar Scalar;
  typedef Eigen::Matrix<double, _Derived::ColsAtCompileTime, _Derived::RowsAtCompileTime> Base;
  Base ed;
  tf::matrixTFToEigen(t, ed);
  e = ed.template cast<Scalar>();
}

/*!
 *  \brief Converts an Eigen::Quaterniond into a tf::Matrix3x3.
 */
void eigenToMatrixTF(const Eigen::Quaterniond& e, tf::Matrix3x3& t);

/*!
 *  \brief Converts an Eigen::QuaternionBase into a tf::Matrix3x3.
 */
template <typename _Derived>
void eigenToMatrixTF(const Eigen::QuaternionBase<_Derived>& e, tf::Matrix3x3& t)
{
  Eigen::Quaterniond ed;
  ed = e.template cast<double>();
  eigenToMatrixTF(ed, t);
}

/*!
 *  \brief Converts an Eigen::MatrixBase into a tf::Matrix3x3.
 */
template <typename _Derived>
void eigenToMatrixTF(const Eigen::MatrixBase<_Derived>& e, tf::Matrix3x3& t)
{
  typedef Eigen::Matrix<double, _Derived::ColsAtCompileTime, _Derived::RowsAtCompileTime> Base;
  Base ed;
  ed = e.template cast<double>();
  tf::matrixEigenToTF(ed, t);
}

/*!
 *  \brief Converts a tf::Quaternion into an Eigen::QuaternionBase.
 */
template <typename _Derived>
void quaternionTFToEigen(const tf::Quaternion& t, Eigen::QuaternionBase<_Derived>& e)
{
  typedef typename Eigen::internal::traits<_Derived>::Scalar Scalar;
  Eigen::Quaterniond ed;
  tf::quaternionTFToEigen(t, ed);
  e = ed.cast<Scalar>();
}

/*!
 *  \brief Converts a tf::Quaternion into an Eigen::MatrixBase.
 */
template <typename _Derived>
void quaternionTFToEigen(const tf::Quaternion& t, Eigen::MatrixBase<_Derived>& e)
{
  typedef typename Eigen::internal::traits<_Derived>::Scalar Scalar;
  Eigen::Quaternion<Scalar> eq;
  quaternionTFToEigen(t, eq);
  e = eq.toRotationMatrix();
}

/*!
 *  \brief Converts a tf::Quaternion into an Eigen::QuaternionBase.
 */
template <typename _Derived>
void eigenToQuaternionTF(const Eigen::QuaternionBase<_Derived>& e, tf::Quaternion& t)
{
  //    typedef typename Eigen::internal::traits<_Derived>::Scalar Scalar;
  Eigen::Quaterniond ed;
  ed = e.template cast<double>();
  tf::quaternionEigenToTF(ed, t);
}

template <typename _Derived>
void eigenToQuaternionTF(const Eigen::MatrixBase<_Derived>& e, tf::Quaternion& t)
{
  typedef typename Eigen::internal::traits<_Derived>::Scalar Scalar;
  Eigen::Quaternion<Scalar> eq(e);
  eigenToQuaternionTF(eq, t);
}

/*!
 *  \brief Converts a geometry_msgs::Quaternion into an Eigen::Quaterniond.
 */
void quaternionMsgToEigen(const geometry_msgs::Quaternion& t, Eigen::Quaterniond& e);

/*!
 *  \brief Converts a geometry_msgs::Quaternion into an Eigen::QuaternionBase.
 */
template <typename _Derived>
void quaternionMsgToEigen(const geometry_msgs::Quaternion& t, Eigen::QuaternionBase<_Derived>& e)
{
  typedef typename Eigen::internal::traits<_Derived>::Scalar Scalar;
  Eigen::Quaterniond ed;
  quaternionMsgToEigen(t, ed);
  e = ed.cast<Scalar>();
}

/*!
 *  \brief Converts a geometry_msgs::Quaternion into an Eigen::Matrix3.
 */
template <typename _Derived>
void quaternionMsgToEigen(const geometry_msgs::Quaternion& t, Eigen::MatrixBase<_Derived>& e)
{
  typedef typename Eigen::internal::traits<_Derived>::Scalar Scalar;
  Eigen::Quaternion<Scalar> eq;
  quaternionMsgToEigen(t, eq);
  e = eq.toRotationMatrix();
}

/*!
 *  \brief Converts an Eigen::Quaterniond into a geometry_msgs::Quaternion.
 */
void eigenToQuaternionMsg(const Eigen::Quaterniond& e, geometry_msgs::Quaternion& t);

/*!
 *  \brief Converts an Eigen::QuaternionBase to a geometry_msgs::Quaternion.
 */
template <typename _Derived>
void eigenToQuaternionMsg(const Eigen::QuaternionBase<_Derived>& e, geometry_msgs::Quaternion& t)
{
  //    typedef typename Eigen::internal::traits<_Derived>::Scalar Scalar;
  Eigen::Quaterniond ed;
  ed = e.template cast<double>();
  eigenToQuaternionMsg(ed, t);
}

/*!
 *  \brief Converts an Eigen::Matrix3 to a geometry_msgs::Quaternion.
 */
template <typename _Derived>
void eigenToQuaternionMsg(const Eigen::MatrixBase<_Derived>& e, geometry_msgs::Quaternion& t)
{
  typedef typename Eigen::internal::traits<_Derived>::Scalar Scalar;
  Eigen::Quaternion<Scalar> eq(e);
  eigenToQuaternionMsg(eq, t);
}

/*!
 *  \brief Converts an Eigen::Vector3d into a geometry_msgs::Point.
 */
void eigenToPointMsg(const Eigen::Vector3d& e, geometry_msgs::Point& t);

/*!
 *  \brief Converts an Eigen::EigenBase to a geometry_msgs::Point.
 */
template <typename _Derived>
void vector3EigenToPointMsg(const Eigen::MatrixBase<_Derived>& e, geometry_msgs::Point& t)
{
  typedef Eigen::Matrix<double, _Derived::ColsAtCompileTime, _Derived::RowsAtCompileTime> Base;
  Base ed = e.template cast<double>();
  eigenToPointMsg(ed, t);
}

/*!
 *  \brief Converts an Eigen::Vector3d into a geometry_msgs::Point.
 */
void eigenToVector3Msg(const Eigen::Vector3d& e, geometry_msgs::Vector3& t);

/*!
 *  \brief Converts an Eigen::EigenBase to a geometry_msgs::Point.
 */
template <typename _Derived>
void vector3EigenToVector3Msg(const Eigen::MatrixBase<_Derived>& e, geometry_msgs::Vector3& t)
{
  typedef Eigen::Matrix<double, _Derived::ColsAtCompileTime, _Derived::RowsAtCompileTime> Base;
  Base ed = e.template cast<double>();
  vector3dEigenToVector3Msg(ed, t);
}

/*!
 *  \brief Converts an Eigen::VectorXd into a std::vector<double>.
 */
void eigenToStdVector(const Eigen::VectorXd& e, std::vector<double>& t);

/*!
 *  \brief Converts an Eigen::EigenBase to a ow_msgs::Vector.
 */
template <typename _Derived>
void eigenToVectorMsg(const Eigen::MatrixBase<_Derived>& e, ow_msgs::Vector& t)
{
  typedef Eigen::Matrix<double, _Derived::ColsAtCompileTime, _Derived::RowsAtCompileTime> Base;
  Base ed = e.template cast<double>();
  eigenToStdVector(ed, t.data);
}

/*!
 *  \brief Converts a geometry_msgs::Point into an Eigen::Vector3d.
 */
void pointMsgToEigen(const geometry_msgs::Point& t, Eigen::Vector3d& e);

/*!
 *  \brief Converts a geometry_msgs::Point into an Eigen::EigenBase.
 */
template <typename _Derived>
void pointMsgToEigen(const geometry_msgs::Point& t, Eigen::MatrixBase<_Derived>& e)
{
  typedef typename Eigen::MatrixBase<_Derived>::Scalar Scalar;
  Eigen::Vector3d ed;
  pointMsgToEigen(t, ed);
  e = ed.cast<Scalar>();
}

/*!
 *  \brief Converts a geometry_msgs::Vector3 into an Eigen::Vector3d.
 */
void vector3MsgToEigen(const geometry_msgs::Vector3& t, Eigen::Vector3d& e);

/*!
 *  \brief Converts a geometry_msgs::Vector3 into an Eigen::EigenBase.
 */
template <typename _Derived>
void vector3MsgToEigen(const geometry_msgs::Vector3& t, Eigen::MatrixBase<_Derived>& e)
{
  typedef typename Eigen::MatrixBase<_Derived>::Scalar Scalar;
  Eigen::Vector3d ed;
  vector3MsgToEigen(t, ed);
  e = ed.cast<Scalar>();
}

/*!
 *  \brief Converts an std::vector<double> into Eigen::VectorXd.
 */
void stdVectorToEigen(const std::vector<double>& t, Eigen::VectorXd& e);

/*!
 *  \brief Converts a geometry_msgs::Vector3 into an Eigen::EigenBase.
 */
template <typename _Derived>
void vectorMsgToEigen(const ow_msgs::Vector& t, Eigen::MatrixBase<_Derived>& e)
{
  typedef typename Eigen::MatrixBase<_Derived>::Scalar Scalar;
  Eigen::VectorXd ed;
  stdVectorToEigen(t.data, ed);
  e = ed.cast<Scalar>();
}

/*!
 *  \brief Converts an Eigen::EigenBase into a tf::Vector3.
 */
template <typename _Derived>
void eigenToVector3TF(const Eigen::MatrixBase<_Derived>& e, tf::Vector3& t)
{
  Eigen::Vector3d ed;
  ed = e.template cast<double>();
  tf::vectorEigenToTF(ed, t);
}

/*!
 *  \brief Converts an tf::Vector3 into Eigen::EigenBase.
 */
template <typename _Derived>
void vector3TFToEigen(const tf::Vector3& t, Eigen::MatrixBase<_Derived>& e)
{
  typedef typename Eigen::MatrixBase<_Derived>::Scalar Scalar;
  Eigen::Vector3d ed;
  tf::vectorTFToEigen(t, ed);
  e = ed.cast<Scalar>();
}

}  // namespace ow_core

#endif  // OPEN_WALKER_CORE_CONVERSIONS_H
