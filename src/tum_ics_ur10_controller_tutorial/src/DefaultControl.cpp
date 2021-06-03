#include <tum_ics_ur10_controller_tutorial/DefaultControl.h>
#include <tum_ics_ur_robot_msgs/ControlData.h>
#include <math.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <ur10_robot_model/model_ur10.h>
#include <ow_core/math/maps.h>
#include <boost/qvm/quat_operations.hpp>
#include <ow_core/trajectory/trajectories.h>
#include <tum_ics_ur10_controller_tutorial/CartesianTrajectory.h>
#include <object_msgs/Objects.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <ow_core/math.h>
#include <ow_core/algorithms/state_integrator.h>
#include <std_msgs/Float32.h>

namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    DefaultControl::DefaultControl(double weight, const QString &name) : 
      ControlEffort(name, SPLINE_TYPE, JOINT_SPACE, weight),
      m_startFlag(false),
      m_jKp(Matrix6d::Zero()),
      m_jKd(Matrix6d::Zero()),
      m_jKi(Matrix6d::Zero()),
      m_cKp(Matrix6d::Zero()),
      m_cKd(Matrix6d::Zero()),
      m_cKi(Matrix6d::Zero()),
      m_Kf(Vector6d::Zero()),
      m_iDeltaX(Vector6d::Zero()),
      m_jointSpaceGoal(Vector6d::Zero()),
      m_jointTrajectoryDuration(100.0),
      m_targetTrajectoryOffset(Vector3d(0, 0, 0.5)),
      m_stateIntegrator(ow::Scalar(0.002)),
      m_desiredPVA(Matrix3d::Zero())
    {
      m_desiredPositionPublisher = n.advertise<geometry_msgs::PoseStamped>("/desired_position", 100);
      m_angularErrorPublisher = n.advertise<std_msgs::Float32>("/angular_error", 100);
      m_controlDataPublisher = n.advertise<tum_ics_ur_robot_msgs::ControlData>("/control_data", 100);
      m_circlePathPublisher = n.advertise<nav_msgs::Path>("/cartesian_path", 100);
      m_posePublisher = n.advertise<geometry_msgs::PoseStamped>("/target_pose", 100);
      m_pathPublisher = n.advertise<nav_msgs::Path>("/ef_traj", 100);
      
      m_obstacleSubscriber = n.subscribe("/obstacles", 100, &DefaultControl::obstacleCallback, this);
      m_targetSubscriber = n.subscribe("/target", 100, &DefaultControl::targetCallback, this);

      m_controlPeriod = 0.002;
      m_theta_hat = m_robotModel.parameterInitalGuess();

      m_continueTrajectoryTimestamp = 0;
      m_lastStateUpdateTimeStep = 0;

      m_currentSlerpStartTimeStep = 0;
      m_currentSlerpEndTimeStep = 0;

      m_targetPosition = Vector3d::Zero();
      m_path.header.frame_id = "world";
      m_circlePath.header.frame_id = "world";

      m_prevState = State::none;
      m_state = State::none;
    }

    DefaultControl::~DefaultControl() {}

    void DefaultControl::setQInit(const JointState &qinit) {
      m_qInit = qinit;
    }

    void DefaultControl::setQHome(const JointState &qhome) {
      m_qHome = qhome;
    }

    void DefaultControl::setQPark(const JointState &qpark) {
      m_qPark = qpark;
    }

    bool DefaultControl::init() {
      ROS_WARN_STREAM("DefaultControl::init");

      if(!m_robotModel.initRequest(n)) {
        ROS_ERROR_STREAM("Error initalizing model");
        return false;
      }

      readParameters(true);
      generateCircularTrajectoryPath();

      return !m_error;
    }

    bool DefaultControl::start() {
      ROS_WARN_STREAM("DefaultControl::start");
      return true;
    }

    bool DefaultControl::stop() {
      return true;
    }

    void DefaultControl::targetCallback(const object_msgs::ObjectsConstPtr& msg) {
      auto position = msg->objects[0].position.position;
      m_targetPosition << position.x, position.y, position.z;
    }

    void DefaultControl::obstacleCallback(const object_msgs::ObjectsConstPtr& msg) {
      int numObstacles = msg->objects.size();
      m_obstaclePositions.resize(3, numObstacles);

      // transform obstacle position into base frame and store in m_obstaclePosition
      for (int i = 0; i < (msg->objects.size()); i++) {
        auto position = msg->objects[i].position.position;

        Vector3d posWorldFrame(position.x, position.y, position.z);
        m_obstaclePositions.col(i) = applyTransformation(m_robotModel.T_0_B(), posWorldFrame);
      }
    }

    /*! \brief The state machine. */
    State DefaultControl::nextState(const RobotTime &time, const JointState &current) {
      if (time.tD() < 6) {
        return State::leaveSingularity;
      }

      m_prevState = m_state;

      if (m_prevState == State::leaveSingularity && time.tD() > 6) {
        m_state = State::returnToTrajectory;
      } 
      else if (shouldAvoidObstacles(current)) {
        m_state = State::avoidObstacles;
      }
      else if (m_prevState == State::avoidObstacles && !shouldAvoidObstacles(current)) {
        m_state = State::returnToTrajectory;
      }
      else if (time.tD() >= m_continueTrajectoryTimestamp) {
        m_state = State::followTrajectory;
      }

      if (m_prevState != m_state) {
        m_lastStateUpdateTimeStep = time.tD();
        m_stateIntegrator.reset();
        m_iDeltaX.setZero();
      }

      return m_state;
    }

    Vector6d DefaultControl::update(const RobotTime &time, const JointState &current)
    {
      if (!m_startFlag)
      {
        m_qStart = current.q;
        ROS_WARN_STREAM("START [DEG]: \n" << m_qStart.transpose());
        m_time_prev = time.tRc();
        m_startFlag = true;
      }

      publishVisualizationTopics(time, current);

      // time
      ros::Duration dt = time.tRc() - m_time_prev;
      m_time_prev = time.tRc();

      m_state = nextState(time, current);
      
      switch (m_state)
      {
        case State::none:
          return Vector6d::Zero();
  
        case State::leaveSingularity: {
          VVector6d vQd = getJointPVT5(m_qStart, m_jointSpaceGoal, time.tD(), m_jointTrajectoryDuration);
          return jointSpaceController(time, dt, current, vQd);
        }

        case State::followTrajectory: {
          return cartesianSpaceController(time, dt, current, circularTrajectory(time.tD()));
        }

        case State::avoidObstacles: {
          Matrix3d M = Matrix3d::Zero();
          M.col(0) = m_robotModel.T_tool_0(current.q).translation();

          // impedance control for obstacle avoidance (only first three joints)
          auto obstacleAvoidance = impedanceController(current);
          obstacleAvoidance.tail(3).setZero();
          
          // disable position control, only use orientation control and model compensation
          auto gazeControl = cartesianSpaceController(time, dt, current, M, 0.0, 1.0);

          return obstacleAvoidance + gazeControl;
        }

        case State::returnToTrajectory: {
          if (m_prevState != State::returnToTrajectory) {
            double duration = returnTrajectoryDuration(current, time.tD());

            m_continueTrajectoryTimestamp = time.tD() + duration;
            m_returnInitPVA = getPVA(current);
          }

          Matrix3d goalPVA = circularTrajectory(m_continueTrajectoryTimestamp);

          // create trajectory from current PVA to circlePVA
          CartesianTrajectory traj;
          traj.init(
            m_lastStateUpdateTimeStep, 
            m_continueTrajectoryTimestamp, 
            m_returnInitPVA.col(0),
            goalPVA.col(0),
            m_returnInitPVA.col(1),
            goalPVA.col(1),
            m_returnInitPVA.col(2), 
            goalPVA.col(2));

          Matrix3d desiredPVA = traj.evaluate(time.tD());

          return cartesianSpaceController(time, dt, current, desiredPVA);
        }
      }
    }

    Vector6d DefaultControl::cartesianSpaceController(
      const RobotTime &time,
      const ros::Duration &dt, 
      const JointState &current, 
      const Matrix3d &desiredStates,
      const double &positionControlWeight,
      const double &orientationControlWeight)
    {
      MatrixDOFd J = m_robotModel.J_tool_0(current.q);
      MatrixDOFd Jp = m_robotModel.Jp_tool_0(current.q, current.qp);
      MatrixDOFd J_inverse = inverseJacobian(J);

      // current pose
      ow::HomogeneousTransformation transform = m_robotModel.T_tool_0(current.q);
      Vector3d currentPosition = transform.translation();
      Quaterniond currentQuaternion(transform.rotation());

      // current linear and angular velocity and acceleration
      Vector6d x;
      x << currentPosition, Vector3d::Zero(); // orientation not relevant yet
      Vector6d xp = J * current.qp;
      Vector6d xpp = Jp * current.qp + J * current.qpp;

      // desired linear velocity and acceleration
      Vector6d xd;
      Vector6d xpd;      
      Vector6d xppd;

      xd << desiredStates.col(0), Vector3d::Zero(); // orientation part is set below
      xpd << desiredStates.col(1), Vector3d::Zero();
      xppd << desiredStates.col(2), Vector3d::Zero();

      // position, velocity and acceleration errors (only the linear part is relevant here)
      Vector6d deltaX = x - xd;
      Vector6d deltaXp = xp - xpd;
      Vector6d deltaXpp = xpp - xppd;

      Matrix3d orientationErrors = computeOrientationErrors(current, xp.tail(3), xpp.tail(3), time);

      // correct orientation and angular velocity/acceleration errors with transformed values
      deltaX.tail(3) = orientationErrors.row(0);
      deltaXp.tail(3) = orientationErrors.row(1);
      deltaXpp.tail(3) = orientationErrors.row(2);

      applyErrorWeighting(deltaX, deltaXp, deltaXpp, positionControlWeight, orientationControlWeight);

      // error integral
      m_iDeltaX += dt.toSec() * deltaX;

      // use lower gains when interpolating the orientation 
      // to avoid joint velocity limits
      auto Kp(m_cKp);
      auto Kd(m_cKd);

      if (time.tD() < m_currentSlerpEndTimeStep) {
        Kp.bottomRightCorner(3, 3) *= 0.7;
        Kd.bottomRightCorner(3, 3) *= 0.7;
      }

      // compute references in cartesian space
      Vector6d xpr = xpd - Kp * deltaX - m_cKi * m_iDeltaX;
      Vector6d xppr = xppd - m_cKp * deltaXp;

      // compute references in joint space
      JointState reference;
      reference.qp = J_inverse * xpr;
      reference.qpp = J_inverse * (xppr - Jp * current.qp);

      // compute torques
      auto Sq = current.qp - reference.qp;
      auto compensationTerm = modelCompensation(current, reference, Sq, dt);
      
      Vector6d tau = -Kd * Sq + compensationTerm;

      return tau;
    }

    Vector6d DefaultControl::impedanceController(const JointState &current) {
      Vector6d tau;
      tau.setZero();

      Vector6d obstacleToJointDistances = Vector6d::Zero();

      // repulsive potentials for each joint
      for (int joint = 0; joint < STD_DOF; joint++) {
        Vector3d jointPosition = m_robotModel.T_j_0(current.q, joint).translation();

        Vector3d repulsion;
        repulsion.setZero();
        
        // sum up repulsive potentials for all objects
        for (int j = 0; j < m_obstaclePositions.cols(); j++) {
          repulsion += repulsivePotential(jointPosition - m_obstaclePositions.col(j));
        }
        
        auto linearJointJacobian = m_robotModel.J_j_0(current.q, joint).topRows(3);
        tau += linearJointJacobian.transpose() * m_Kf(joint) * repulsion;
      }

      return tau;
    }

    /*! \brief Calculates and returns a duration for a trajecory back to the
     * the moving target position on the circular path. */
    double DefaultControl::returnTrajectoryDuration(
      const JointState &current,
      const double t0)
    {
      Vector3d currentPosition = getPVA(current).col(0);
    
      // first, naively estimate a good duration of the trajectory
      // by using the target position on the circle at t0
      Vector3d naiveTargetPosition = circularTrajectory(t0).col(0);
    
      double naiveDistance = (currentPosition - naiveTargetPosition).norm();
      double duration = m_trajectoryDurationErrorGain * naiveDistance;
    
      // get actual target position on the circle position after the duration
      Vector3d targetPosition = circularTrajectory(t0 + duration).col(0);
    
      double distance = (currentPosition - targetPosition).norm();
      duration = m_trajectoryDurationErrorGain * distance;

      return duration;
    }

    /*! \brief Returns the current tool position col(0), velocity col(1)
    * and acceleration col(2). */
    Matrix3d DefaultControl::getPVA(const JointState &current) {
      MatrixDOFd J = m_robotModel.J_tool_0(current.q);
      MatrixDOFd Jp = m_robotModel.Jp_tool_0(current.q, current.qp);

      Matrix3d pva;

      pva.col(0) = m_robotModel.T_tool_0(current.q).translation();
      pva.col(1) = m_robotModel.J_tool_0(current.q).topRows(3) * current.qp;
      pva.col(2) = Jp.topRows(3) * current.qp + J.topRows(3) * current.qpp;

      return pva;
    }

    /*! \brief Returns a repulsive protential as a function of the given distance. */
    Vector3d DefaultControl::repulsivePotential(const Vector3d distance) {
      if (distance.norm() > m_obstacleInfluenceRadius) {
        return Vector3d::Zero();
      }

      double magnitude = pow(1 - distance.norm() / m_obstacleInfluenceRadius, 2);
      Vector3d direction = distance.normalized();

      return magnitude * direction;
    }

    /*! \brief Whether at least one obstacle is influencing at least one joint. */
    bool DefaultControl::shouldAvoidObstacles(const JointState &current) {
      for (int i = 0; i < STD_DOF; i++) {
        Vector3d jointPosition = m_robotModel.T_j_0(current.q, i).translation();

        for (int j = 0; j < m_obstaclePositions.cols(); j++) {
          auto distance = (jointPosition - m_obstaclePositions.col(j)).norm();

          if (distance < m_obstacleInfluenceRadius) {
            return true;
          }
        }
      }

      return false;
    }

    Matrix3d DefaultControl::computeOrientationErrors(
      const JointState &current,
      const Vector3d &omega,
      const Vector3d &omegaP,
      const RobotTime &time)
    {
      Quaterniond currentQuaternion(m_robotModel.T_tool_0(current.q).rotation());
      Quaterniond desiredQuaternion = rotationToGazeTarget(current);

      double angularError = abs(currentQuaternion.angularDistance(desiredQuaternion));
      auto s = slerpInterpolationParameter(angularError, time);

      auto orientation = ow_core::AngularPosition<double>(currentQuaternion);
      auto desiredOrientation = ow_core::AngularPosition<double>(desiredQuaternion);

      auto Qd = ow::slerp(orientation, desiredOrientation, s);
      auto Qdp = ow::slerpP(orientation, desiredOrientation, s);
      auto Qdpp = ow::slerpPP(orientation, desiredOrientation, s);

      ow_core::AngularPosition<double> Qe = Qd.normalized() * currentQuaternion.normalized().inverse();
      ow::checkFlipQuaternionSign(Qe);
      
      ow::AngularVelocity wQd;
      ow::AngularAcceleration wQdp;
      ow::quaternion2AngularVelocityInertial(wQd, Qdp, Qd);
      ow::quaternion2AngularAccelerationInertial(wQdp, Qdpp, Qd);

      Vector3d orientationError = -Qe.logMap();
      Vector3d angularVelocityError = omega - wQd;
      Vector3d angularAccelerationError = omegaP - wQdp;

      // TODO: find out reason and fix this
      if (orientationError.hasNaN()) {
        orientationError << Vector3d::Zero();
      }
      
      Matrix3d errors;
      errors.row(0) = orientationError;
      errors.row(1) = angularVelocityError;
      errors.row(2) = angularAccelerationError;

      return errors;
    }

    /*! \brief Evaluates the trajectory of the slerp interpolation parameter, returns s. */
    ow::Scalar DefaultControl::slerpInterpolationParameter(
      const double angularError, 
      const RobotTime &time)
    {
      ow::Scalar s;

      // if deviation is small enough, no need to interpolate
      if (angularError < m_slerpAngularErrorThreshold && time.tD() > m_currentSlerpEndTimeStep) {
        s = 1;

        // stop current slerp id needed
        m_currentSlerpEndTimeStep = time.tD();
      }
      // interpolate to the target orientation using slerp
      else {
        CartesianTrajectory poly;
        Vector6d sConstraints;
        sConstraints << 0.2, 1, 0, 0, 0, 0;

        // no interpolation currently in progress -> start new one
        if (time.tD() > m_currentSlerpEndTimeStep) {
          m_currentSlerpStartTimeStep = time.tD();
          m_currentSlerpEndTimeStep = m_currentSlerpStartTimeStep + m_slerpDurationErrorGain * angularError;
        }

        poly.init(m_currentSlerpStartTimeStep, m_currentSlerpEndTimeStep, Vector3d::Zero(), Vector3d::Zero());
        s = poly.evaluateScalar(sConstraints, time.tD())[0];

        m_iDeltaX.setZero();
      }

      return s;
    }

    /*! \brief  Multiplies the positional and angular parts of the error vectors
     * with the given weights. */
    void DefaultControl::applyErrorWeighting(
      Vector6d &deltaX,
      Vector6d &deltaXp,
      Vector6d &deltaXpp,
      const double positionWeight, 
      const double orientationWeight)
    {
      deltaX.head(3) *= positionWeight;
      deltaXp.head(3) *= positionWeight;
      deltaXpp.head(3) *= positionWeight;

      deltaX.tail(3) *= orientationWeight;
      deltaXp.tail(3) *= orientationWeight;
      deltaXpp.tail(3) *= orientationWeight;
    }

    Vector6d DefaultControl::integratedPositionError(
      const Vector6d deltaX,
      const Vector6d deltaXp,
      const Vector6d deltaXpp,
      const Quaterniond rotationError)
    {
      ow::CartesianState state;
      state.pos().linear() = deltaX.head(3);
      state.pos().angular() = rotationError.coeffs();
      state.vel() = deltaXp;
      state.acc() = deltaXpp;

      ow::CartesianState stateIntegral = m_stateIntegrator.update(state);

      Vector6d iDeltaX;
      iDeltaX.head(3) = stateIntegral.pos().linear();
      iDeltaX.tail(3) = ow::logMapS3(Quaterniond(stateIntegral.pos().angular()));

      return iDeltaX;
    }

    Matrix3d DefaultControl::circularTrajectory(const double t) {
      return CartesianTrajectory::circle(
        m_circularTrajectoryAnchorPosition,
        m_circularTrajectoryRadius, 
        m_circularTrajectoryPeriod, 
        t);
    }

    /*! \brief Returns the rotation that points the end-effector towards the currently
     * active target. */
    Quaterniond DefaultControl::rotationToGazeTarget(const JointState &current) {
      // transform target into end-effector frame
      auto targetToEf = m_robotModel.T_tool_B(current.q).inverse();
      Vector3d target = applyTransformation(targetToEf, m_targetPosition).normalized();
      Vector3d z(0, 0, 1);

      // calculate angle and axis of rotation
      Vector3d axis = z.cross(target).normalized();
      auto angle = acos(z.dot(target));

      // if angle is too small, avoid numerically unstable rotation
      if (abs(angle) < 0.01) {
        if (angle < 0.0) { angle = -0.01; } else { angle = 0.01; } 
      }
      
      // make sure to use the shortest angle
      if (axis.cross(z).dot(target) < 0.0) {
        angle = -angle;
      }

      // convert angle and axis to quaternion
      Vector4d coeffs;
      coeffs << sin(angle/2) * axis, cos(angle/2);

      Quaterniond q;
      q.coeffs() = coeffs;
      q.normalize();

      // rotate desired orientation from end-effector to base frame
      auto effectorToBase = m_robotModel.T_tool_0(current.q);
      auto qDesired = Quaterniond(effectorToBase.rotation()) * q;

      return qDesired;
    }

    MatrixDOFd DefaultControl::inverseJacobian(const MatrixDOFd J, const double wThreshold, const double psi) {      
      // compute manipulability
      double w = sqrt((J * J.transpose()).determinant());

      // if near singularity, return a damped pseudo-inverse
      if (w < wThreshold) {
        Matrix6d I = Matrix6d::Identity();
        return J.transpose() * (J * J.transpose() + psi * I).inverse();
      }

      return J.inverse();
    }

    Vector3d DefaultControl::applyTransformation(const ow::HomogeneousTransformation transformation, Vector3d point) {
      Vector4d homogeneousTransformedPoint = transformation.matrix() * point.homogeneous();
      return homogeneousTransformedPoint.head(3);
    }

    Vector6d DefaultControl::jointSpaceController(
      const RobotTime &time, 
      const ros::Duration &dt, 
      const JointState &current, 
      const VVector6d& vQd)
    {
      auto deltaQ = current.q - vQd[0];
      auto deltaQp = current.qp - vQd[1];

      JointState reference;
      reference = current;
      reference.qp = vQd[1] - m_jKp * deltaQ;
      reference.qpp = vQd[2] - m_jKp * deltaQp;

      auto Sq = current.qp - reference.qp;
      auto compensationTerm = modelCompensation(current, reference, Sq, dt);

      return -m_jKd * Sq + compensationTerm;
    }

    /*! \brief Returns Yr * theta. */
    VectorDOFd DefaultControl::modelCompensation(
        const JointState current, 
        const JointState reference, 
        const VectorDOFd Sq, 
        const ros::Duration dt)
    {
      ur::UR10Model::Regressor Yr = m_robotModel.regressor(current.q, current.qp, reference.qp, reference.qpp);
      m_theta_hat -= 0.05 * Yr.transpose() * Sq * dt.toSec();
      return Yr * m_theta_hat;
    }

    void DefaultControl::publishVisualizationTopics(const RobotTime &time, const JointState &current) {
      m_circlePathPublisher.publish(m_circlePath);

      // visualize pose, trajectory and desired position
      publishPose(m_posePublisher, m_robotModel.T_ef_B(current.q));
      publishPath(m_pathPublisher, m_path, m_robotModel.T_tool_B(current.q));

      Vector3d desiredPosition = circularTrajectory(time.tD()).col(0);
      ow::HomogeneousTransformation desiredTransform;
      desiredTransform.position() = applyTransformation(m_robotModel.T_B_0(), desiredPosition);
      desiredTransform.orientation().setIdentity();
      publishPose(m_desiredPositionPublisher, desiredTransform);
    }

    void DefaultControl::publishPath(
      const ros::Publisher publisher,
      nav_msgs::Path &path,
      ow::HomogeneousTransformation transform, 
      const int maxLength)
    {
      // only keep the most recent poses
      if (path.poses.size() >= maxLength) {
        path.poses.erase(path.poses.begin());
      }

      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.header.stamp = ros::Time::now();
      pose.pose = transform.toPoseMsg();
      path.poses.push_back(pose);
      
      publisher.publish(path);
    }

    void DefaultControl::publishPose(
      const ros::Publisher publisher, 
      ow::HomogeneousTransformation transform)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "world";
      pose.pose = transform.toPoseMsg();

      publisher.publish(pose);
    }

    void DefaultControl::publishControlData(
      const RobotTime &time,
      const JointState &current,
      const Vector6d &deltaX,
      const Vector6d &deltaXp)
    {
      tum_ics_ur_robot_msgs::ControlData msg;
      msg.header.stamp = ros::Time::now();
      msg.time = time.tD();

      auto T = m_robotModel.T_tool_0(current.q);
      Vector6d Xef0;
      Xef0 << T.translation(), T.rotation().eulerAngles(0, 1, 2);

      for (int i = 0; i < STD_DOF; i++)
      {
        msg.q[i] = current.q(i);
        msg.qp[i] = current.qp(i);
        msg.qpp[i] = current.qpp(i);

        msg.DX[i] = deltaX(i);
        msg.DXp[i] = deltaXp(i);

        msg.Xef_0[i] = Xef0(i);

        msg.torques[i] = current.tau(i);
      }

      m_controlDataPublisher.publish(msg);
    }

    /*! \brief Stores poses forming a nav_msgs::Path of the circular trajectory
     * in m_circlePath for visualization purposes. */
    void DefaultControl::generateCircularTrajectoryPath(const int n) {
      auto baseToWorldTransform = m_robotModel.T_B_0();
      auto orientation = Quaterniond::Identity();

      for (int t = 0; t < n; t++) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.header.stamp.sec = 0;
        pose.header.stamp.nsec = t;

        double tt = t/double(n) * m_circularTrajectoryPeriod;
        Vector3d position = applyTransformation(baseToWorldTransform, circularTrajectory(tt).col(0));

        pose.pose.position.x = position.x();
        pose.pose.position.y = position.y();
        pose.pose.position.z = position.z();
        pose.pose.orientation.x = orientation.x();
        pose.pose.orientation.y = orientation.y();
        pose.pose.orientation.z = orientation.z();
        pose.pose.orientation.w = orientation.w();

        m_circlePath.poses.push_back(pose);
      }
    }

    void DefaultControl::broadcastTransform(
      const std::__cxx11::string name, 
      const Vector3d translation,
      const Quaterniond rotation,
      const std::__cxx11::string referenceFrame)
    {
      static tf2_ros::TransformBroadcaster br;
      geometry_msgs::TransformStamped transformStamped;

      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = referenceFrame;
      transformStamped.child_frame_id = name;
      transformStamped.transform.translation.x = translation[0];
      transformStamped.transform.translation.y = translation[1];
      transformStamped.transform.translation.z = translation[2];

      transformStamped.transform.rotation.x = rotation.x();
      transformStamped.transform.rotation.y = rotation.y();
      transformStamped.transform.rotation.z = rotation.z();
      transformStamped.transform.rotation.w = rotation.w();

      br.sendTransform(transformStamped);
    }

    void DefaultControl::readParameters(const bool verbose) {
      std::vector<double> vec;

      // check namespace
      std::string ns = "~simple_effort_ctrl";
      if (!ros::param::has(ns)) {
        ROS_ERROR_STREAM("Parameters are not defined in:" << ns);
        m_error = true;
      }

      readScalarParameter("/joint_trajectory_duration", m_jointTrajectoryDuration, 100.0, ns, verbose);
      readScalarParameter("/trajectoryDurationErrorGain", m_trajectoryDurationErrorGain, 5, ns, verbose);
      readScalarParameter("/slerpAngularErrorThreshold", m_slerpAngularErrorThreshold, 0.25, ns, verbose);
      readScalarParameter("/slerpDurationErrorGain", m_slerpDurationErrorGain, 5, ns, verbose);
      readScalarParameter("/obstacleInfluenceRadius", m_obstacleInfluenceRadius, 0.5, ns, verbose);
      readScalarParameter("/circularTrajectoryPeriod", m_circularTrajectoryPeriod, 7, ns, verbose);
      readScalarParameter("/circularTrajectoryRadius", m_circularTrajectoryRadius, 0.15, ns), verbose; 
      
      readDiagonalMatrixDOFdParameter("/joint_space_gains_p", m_jKp, ns, verbose);
      readDiagonalMatrixDOFdParameter("/joint_space_gains_d", m_jKd, ns, verbose);
      readDiagonalMatrixDOFdParameter("/task_space_gains_p", m_cKp, ns, verbose);
      readDiagonalMatrixDOFdParameter("/task_space_gains_d", m_cKd, ns, verbose);
      readDiagonalMatrixDOFdParameter("/task_space_gains_i", m_cKi, ns, verbose);

      ros::param::get(ns + "/force_control_gains", vec);
      for (int i = 0; i < STD_DOF; i++) {
        m_Kf(i) = vec[i];
      }

      if (verbose) {
        ROS_WARN_STREAM("Kf: \n" << m_Kf.transpose());
      }

      ros::param::get(ns + "/circularTrajectoryAnchorPosition", vec);
      for (int i = 0; i < 3; i++) {
        m_circularTrajectoryAnchorPosition(i) = vec[i];
      }

      // GOAL
      ros::param::get(ns + "/joint_space_goal", vec);
      for (int i = 0; i < STD_DOF; i++) {
        m_jointSpaceGoal(i) = vec[i];
      }

      m_jointSpaceGoal = DEG2RAD(m_jointSpaceGoal);
    }

    void DefaultControl::readDiagonalMatrixDOFdParameter(
      const std::__cxx11::string &key, 
      MatrixDOFd &M, 
      const std::string ns,
      const bool verbose)
    {
      std::vector<double> vec;
      ros::param::get(ns + key, vec);

      for (int i = 0; i < STD_DOF; i++) {
        M(i, i) = vec[i];
      }
      
      if (verbose) {
        ROS_WARN_STREAM(key + ": \n" << M);
      }
    }

    void DefaultControl::readScalarParameter(
      const std::__cxx11::string &key, 
      double &s, double fallback, 
      const std::string ns,
      const bool verbose)
    {
      ros::param::get(ns + key, s);
      
      if (s <= 0) {
        ROS_ERROR_STREAM(key + " must be positive, but is " << s);
        s = fallback;
      }

      if (verbose) {
        ROS_WARN_STREAM(key + ": " << s);
      }
    }

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
