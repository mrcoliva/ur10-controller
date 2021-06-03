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


#ifndef OPEN_WALKER_CORE_IMU_SENSOR_H
#define OPEN_WALKER_CORE_IMU_SENSOR_H

#include <ow_core/types/linear_acceleration.h>
#include <ow_core/types/angular_velocity.h>
#include <ow_core/types/angular_position.h>

#include <sensor_msgs/Imu.h>

namespace ow_core{

/*!
 * \brief The InertialMeasurmentUnitSensor class.
 *
 *  This class is a container for:
 *    - linear acceleration xPP measured by the acceleration sensor
 *    - Angular velocity omega measured by the gyro sensor
 *    - AngularPosition Q computed by from the two
 */
template <typename _Scalar>
class InertialMeasurmentUnitSensor 
{
public:
  typedef _Scalar Scalar;
  typedef LinearAcceleration<Scalar> LinearAcc;
  typedef AngularVelocity<Scalar> AngularVel;
  typedef AngularPosition<Scalar> AngularPos;
  typedef CartesianPosition<Scalar> CartesianPos;

  /*!
   * \brief Construct as Zero.
   */
  static const InertialMeasurmentUnitSensor& Zero()
  {
    static const InertialMeasurmentUnitSensor v(
      LinearAcc::Zero(),
      AngularVel::Zero(),
      "",
      AngularPos::Identity()
    );
    return v;
  }

  /*!
   * \brief Construct as Default
   */
  static const InertialMeasurmentUnitSensor& Default()
  {
    static const InertialMeasurmentUnitSensor v(
      LinearAcc::Zero(),
      AngularVel::Zero(),
      "",
      AngularPos::Identity()
    );
    return v;
  }

protected:
  std::string name_;        //!< sensor name
  LinearAcc xPP_;           //!< accelerometer ouput
  AngularVel omega_;        //!< gyroscope ouput    
  AngularPos Q_;            //!< orientation wrt to robot base
  CartesianPos X_imu_base_; //!< imu mounting wrt robot base

public:
  /*!
    * \brief Default Constructor.
    */
  InertialMeasurmentUnitSensor()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  InertialMeasurmentUnitSensor(const InertialMeasurmentUnitSensor& other) :
    xPP_(other.xPP_),
    omega_(other.omega_),
    name_(other.name_),
    Q_(other.Q_),
    X_imu_base_(other.X_imu_base_)
  {
  }

  /*!
    * \brief Constructor from sub elements
   */
  explicit InertialMeasurmentUnitSensor(
    const LinearAcc& xPP,
    const AngularVel& omega,
    const std::string& name = "",
    const AngularPos& Q = AngularPos::Identity(),
    const CartesianPos& X_imu_base_ = CartesianPos::Zero()) :
    xPP_(xPP),
    omega_(omega),
    name_(name),
    Q_(Q),
    X_imu_base_(X_imu_base_)
  {
  }

  std::string& name()
  {
    return name_;
  }

  const std::string& name() const
  {
    return name_;
  }

  LinearAcc& xPP()
  {
    return xPP_;
  }

  LinearAcc& linearAcc()
  {
    return xPP_;
  }

  const LinearAcc& xPP() const
  {
    return xPP_;
  }

  const LinearAcc& linearAcc() const
  {
    return xPP_;
  }

  AngularVel& omega()
  {
    return omega_;
  }

  AngularVel& angularVel()
  {
    return omega_;
  }

  const AngularVel& omega() const
  {
    return omega_;
  }

  const AngularVel& angularVel() const
  {
    return omega_;
  }

  CartesianPos& X_imu_base()
  {
    return X_imu_base_;
  }

  const CartesianPos& X_imu_base() const
  {
    return X_imu_base_;
  }

  AngularPos& Q()
  {
    return Q_;
  }

  AngularPos& angularPos()
  {
    return Q_;
  }

  const AngularPos& Q() const
  {
    return Q_;
  }

  const AngularPos& angularPos() const
  {
    return Q_;
  }

  /*!
  * \brief Assignment of sensor_msgs::Imu.
  */
  InertialMeasurmentUnitSensor& operator=(const sensor_msgs::Imu& msg)
  {
    angularPos() = msg.orientation;
    angularVel() = msg.angular_velocity;
    linearAcc() = msg.linear_acceleration;    
    return *this;
  }

  /*!
  * \brief Conversion to sensor_msgs::Imu.
  */
  operator sensor_msgs::Imu() const
  {
    sensor_msgs::Imu msg;
    msg.orientation = angularPos();
    msg.angular_velocity = angularVel();
    msg.linear_acceleration = linearAcc();
    return msg;
  }  

  /*!
  * \brief Conversion to sensor_msgs::Imu.
  */
  sensor_msgs::Imu toImuMsg() const
  {
    return static_cast<sensor_msgs::Imu>(*this);
  }

};

}

#endif // OPEN_WALKER_CORE_IMU_SENSOR_H
