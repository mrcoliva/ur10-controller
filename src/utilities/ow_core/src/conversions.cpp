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


#include <ow_core/conversions.h>
#include <tf_conversions/tf_eigen.h>

namespace ow_core
{
void matrixTFToEigen(const tf::Matrix3x3& t, Eigen::Quaterniond& e)
{
  Eigen::Matrix3d md;
  tf::matrixTFToEigen(t, md);
  e = md;
}

void quaternionMsgToEigen(const geometry_msgs::Quaternion& m, Eigen::Quaterniond& e)
{
  e.x() = m.x;
  e.y() = m.y;
  e.z() = m.z;
  e.w() = m.w;
}

void eigenToMatrixTF(const Eigen::Quaterniond& e, tf::Matrix3x3& t)
{
  Eigen::Matrix3d md = e.toRotationMatrix();
  tf::matrixEigenToTF(md, t);
}

void eigenToQuaternionMsg(const Eigen::Quaterniond& e, geometry_msgs::Quaternion& t)
{
  t.x = e.x();
  t.y = e.y();
  t.z = e.z();
  t.w = e.w();
}

void eigenToPointMsg(const Eigen::Vector3d& e, geometry_msgs::Point& t)
{
  t.x = e.x();
  t.y = e.y();
  t.z = e.z();
}

void eigenToVector3Msg(const Eigen::Vector3d& e, geometry_msgs::Vector3& t)
{
  t.x = e.x();
  t.y = e.y();
  t.z = e.z();
}

void eigenToStdVector(const Eigen::VectorXd& e, std::vector<double>& t)
{
  t.resize(e.size());
  for(size_t i = 0; i < t.size(); ++i)
  {
    t[i] = e[i];
  }
}

void pointMsgToEigen(const geometry_msgs::Point& t, Eigen::Vector3d& e)
{
  e.x() = t.x;
  e.y() = t.y;
  e.z() = t.z;
}

void vector3MsgToEigen(const geometry_msgs::Vector3& t, Eigen::Vector3d& e)
{
  e.x() = t.x;
  e.y() = t.y;
  e.z() = t.z;
}

void stdVectorToEigen(const std::vector<double>& t, Eigen::VectorXd& e)
{
  e.resize(t.size());
  for(size_t i = 0; i < t.size(); ++i)
  {
    e[i] = t[i];
  }
}

}  // namespace ow_core
