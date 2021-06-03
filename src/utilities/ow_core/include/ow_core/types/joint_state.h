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


#ifndef OPEN_WALKER_CORE_JOINT_STATE_H
#define OPEN_WALKER_CORE_JOINT_STATE_H

#include <ow_core/utilities/type_guard.h>

#include <ow_core/types/joint_position.h>
#include <ow_core/types/joint_velocity.h>
#include <ow_core/types/joint_acceleration.h>
#include <ow_core/types/joint_effort.h>

#include <ow_core/type_references/joint_state_ref.h>
#include <ow_core/type_bases/state_base.h>

#include <ow_msgs/JointState.h>
#include <sensor_msgs/JointState.h>

namespace ow{

/*!
 * \brief The traits class for the AngularState class.
 *
 * This class contains the typedefs and enums for
 * the AngularState class.
 *
 * These typedefs and enums are used to define and describe
 * the properties (traits) of this class.
 */
template<typename _Scalar, int _Rows>
struct traits<ow_core::JointState<_Scalar,_Rows> >
{
  enum
  {
    RowsAtCompileTime = _Rows,
    IsRef = 0,
  };
  typedef _Scalar Scalar;
  typedef ow_core::JointPosition<Scalar,RowsAtCompileTime> Pos;
  typedef ow_core::JointVelocity<Scalar,RowsAtCompileTime> Vel;
  typedef ow_core::JointAcceleration<Scalar,RowsAtCompileTime> Acc;
  typedef ow_core::JointEffort<Scalar,RowsAtCompileTime> Effort;
};

} // namespace ow

namespace ow_core{

/*!
 * \brief The JointState class.
 *
 *  This class is a container for:
 *    - JointPosition
 *    - JointVelocity
 *    - JointAcceleration
 *    - JointEffort
 */
template<typename _Scalar, int _Rows = Eigen::Dynamic>
class JointState :
  public StateBase<JointState<_Scalar, _Rows> >,
  TypeGuard
{
  OW_TYPE_GUARD(JointState)
  
public:

  enum
  {
    RowsAtCompileTime = _Rows,
  };

  typedef _Scalar Scalar;
  typedef StateBase<JointState<Scalar, RowsAtCompileTime> > Base;

  typedef JointPosition<Scalar,RowsAtCompileTime> Pos;
  typedef JointVelocity<Scalar,RowsAtCompileTime> Vel;
  typedef JointAcceleration<Scalar,RowsAtCompileTime> Acc;
  typedef JointEffort<Scalar,RowsAtCompileTime> Effort;

protected:
  Pos q_;
  Vel qP_;
  Acc qPP_;
  Effort tau_;

public:
  /*!
    * \brief Default Constructor.
    */
  JointState()
  {
  }

  /*!
   * \brief This constructor is for both 1x1 matrices and dynamic vectors
   */
  template<typename T>
  explicit JointState(const T& x) :
    q_(x),
    qP_(x),
    qPP_(x),
    tau_(x)
  {
  }

  /*!
   * \brief Copy constructor.
   */
  JointState(const JointState& other):
    q_(other.q_),
    qP_(other.qP_),
    qPP_(other.qPP_),
    tau_(other.tau_)
  {
  }

  /*!
    * \brief Constructor from sub elements
    * 
    * \note This we resize qP, qPP, tau in the dynamic case to the same size
    * as q.
    */
  explicit JointState(
    const Pos& q) : 
    q_(q)
  {
    qP_.setZero(q_.rows());
    qPP_.setZero(q_.rows());
    tau_.setZero(q_.rows());
  }

  Pos& q()
  {
    return q_;
  }

  Pos& pos()
  {
    return q_;
  }

  const Pos& q() const
  {
    return q_;
  }

  const Pos& pos() const
  {
    return q_;
  }

  Vel& qP()
  {
    return qP_;
  }

  Vel& vel()
  {
    return qP_;
  }

  const Vel& qP() const
  {
    return qP_;
  }

  const Vel& vel() const
  {
    return qP_;
  }

  Acc& qPP()
  {
    return qPP_;
  }

  Acc& acc()
  {
    return qPP_;
  }

  const Acc& qPP() const
  {
    return qPP_;
  }

  const Acc& acc() const
  {
    return qPP_;
  }

  Effort& tau()
  {
    return tau_;
  }

  Effort& effort()
  {
    return tau_;
  }

  const Effort& tau() const
  {
    return tau_;
  }

  const Effort& effort() const
  {
    return tau_;
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;

  /*!
   * \brief Assignment operator.
   */
  JointState& operator=(const JointState& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Get a reference to sub vector
   */
  JointStateRef<JointState<Scalar, RowsAtCompileTime> > ref(
    int start_row,
    int block_rows) 
  {
    return JointStateRef<JointState<Scalar, RowsAtCompileTime> >(
      *this, start_row, block_rows);
  }

  /*!
   * \brief Get a reference to sub vector
   */
  JointStateRef<const JointState<Scalar, RowsAtCompileTime> > ref(
    int start_row,
    int block_rows) const
  {
    return JointStateRef<const JointState<Scalar, RowsAtCompileTime> >(
      *this, start_row, block_rows);
  }

  /*!
   * \brief Assignment of ow_msgs::JointState.
   */
  JointState& operator=(const ow_msgs::JointState& msg)
  {
    pos() = msg.position;
    vel() = msg.velocity;
    acc() = msg.acceleration;
    effort() = msg.effort;
    return *this;
  }

  /*!
   * \brief Conversion to ow_msgs::JointState.
   */
  operator ow_msgs::JointState() const
  {
    ow_msgs::JointState msg;
    msg.position = pos();
    msg.velocity = vel();
    msg.acceleration = acc();
    msg.effort = effort();
    return msg;
  }  

  /*!
   * \brief Conversion to ow_msgs::JointState.
   */
  ow_msgs::JointState toJointStateMsg() const
  {
    return static_cast<ow_msgs::JointState>(*this);
  }

  /*!
   * \brief Assignment of ow_msgs::JointState.
   */
  JointState& operator=(const sensor_msgs::JointState& msg)
  {
    pos() = msg.position;
    vel() = msg.velocity;
    acc().setZero();
    effort() = msg.effort;
    return *this;
  }

  /*!
   * \brief Conversion to ow_msgs::JointState.
   */
  operator sensor_msgs::JointState() const
  {
    sensor_msgs::JointState msg;
    msg.position = pos();
    msg.velocity = vel();
    msg.effort = effort();
    return msg;
  }  

  /*!
   * \brief Conversion to ow_msgs::JointState.
   */
  ow_msgs::JointState toJointStateSensorMsg() const
  {
    return static_cast<ow_msgs::JointState>(*this);
  }

};


} // namespace ow_core

#endif // TUM_ICS_OPEN_WALKER_JOINT_STATE_H
