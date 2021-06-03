#ifndef UR_ROBOT_LLI_DEFAULTCONTROL_H
#define UR_ROBOT_LLI_DEFAULTCONTROL_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <ur10_robot_model/model_ur10.h>
#include <object_msgs/Objects.h>
#include <nav_msgs/Path.h>
#include <ow_core/algorithms/state_integrator.h>

namespace tum_ics_ur_robot_lli
{

  enum class State {
    none,
    leaveSingularity, 
    followTrajectory, 
    avoidObstacles,
    returnToTrajectory
  };

  namespace RobotControllers
  {

    class DefaultControl : public ControlEffort
    {
    private:
      bool m_startFlag;

      Vector6d m_qStart;
      JointState m_qInit;
      JointState m_qHome;
      JointState m_qPark;

      ros::NodeHandle n;
      
      // publishers
      ros::Publisher m_controlDataPublisher;
      ros::Publisher m_pathPublisher;
      ros::Publisher m_circlePathPublisher;
      ros::Publisher m_posePublisher;
      ros::Publisher m_desiredPositionPublisher;
      ros::Publisher m_angularErrorPublisher;

      ros::Subscriber m_targetSubscriber;
      ros::Subscriber m_obstacleSubscriber;
      
      // joint space control goal configuration
      Vector6d m_jointSpaceGoal;

      // joint space gains
      Matrix6d m_jKp;
      Matrix6d m_jKd;
      Matrix6d m_jKi;

      // cartesian space gains
      Matrix6d m_cKp;
      Matrix6d m_cKd;
      Matrix6d m_cKi;

      // repulsive force gains
      Vector6d m_Kf;

      ow_core::StateIntegrator<ow::CartesianState> m_stateIntegrator;

      Vector3d m_circularTrajectoryAnchorPosition;
      double m_circularTrajectoryPeriod;
      double m_circularTrajectoryRadius;
      double m_continueTrajectoryTimestamp;
      double m_trajectoryDurationErrorGain;
      double m_lastStateUpdateTimeStep;
      double m_currentSlerpStartTimeStep;
      double m_currentSlerpEndTimeStep;
      double m_slerpDurationErrorGain;
      double m_slerpAngularErrorThreshold;

      Matrix3d m_desiredPVA;
      Matrix3d m_returnInitPVA;

      State m_prevState;
      State m_state;

      Vector6d m_iDeltaX;

      double m_jointTrajectoryDuration;
      ros::Time m_time_prev;

      Matrix<double, 81, 81> m_Gamma;
      Matrix<double, 81, 1> m_theta_hat;

      ur::UR10Model m_robotModel;

      nav_msgs::Path m_path;
      nav_msgs::Path m_circlePath;

      double m_obstacleInfluenceRadius;
      
      MatrixXd m_obstaclePositions;
      Vector3d m_targetPosition;
      Vector3d m_targetTrajectoryOffset;

      double m_controlPeriod; // [s]

    public:
      DefaultControl(double weight = 1.0, const QString &name = "DefaultControl");

      ~DefaultControl();

      void setQInit(const JointState &qinit);
      void setQHome(const JointState &qhome);
      void setQPark(const JointState &qpark);

    private:
      bool init();
      bool start();
      bool stop();

      Vector6d update(
        const RobotTime &time, 
        const JointState &current);
      
      // controllers
      Vector6d jointSpaceController(
        const RobotTime &time, 
        const ros::Duration &dt, 
        const JointState &current, 
        const VVector6d& vQd);

      Vector6d cartesianSpaceController(
        const RobotTime &time, 
        const ros::Duration &dt, 
        const JointState &current, 
        const Matrix3d &desiredStates, 
        const double &positionControlWeight = 1.0,
        const double &orientationControlWeight = 1.0);
      
      Vector6d impedanceController(
        const JointState &current);

      void applyErrorWeighting(
        Vector6d &deltaX,
        Vector6d &deltaXp,
        Vector6d &deltaXpp,
        const double positionWeight, 
        const double orientationWeight);

      Vector6d integratedPositionError(
        const Vector6d deltaX,
        const Vector6d deltaXp,
        const Vector6d deltaXpp,
        const Quaterniond rotationError);

      double returnTrajectoryDuration(
        const JointState &current,
        const double t0);
      
      // publishing
      void publishVisualizationTopics(
        const RobotTime &time, 
        const JointState &current);

      void publishPose(
        const ros::Publisher publisher, 
        ow::HomogeneousTransformation transform);
      
      void publishPath(
        const ros::Publisher publisher,
        nav_msgs::Path &path,
        ow::HomogeneousTransformation transform, 
        const int maxLength = 5000);

      void publishControlData(
        const RobotTime &time,
        const JointState &current,
        const Vector6d &deltaX,
        const Vector6d &deltaXp);

      void broadcastTransform(
        const std::__cxx11::string name, 
        const Vector3d translation = Vector3d::Zero(),
        const Quaterniond rotation = Quaterniond::Identity(),
        const std::__cxx11::string referenceFrame = "world");

      // topic callbacks
      void targetCallback(const object_msgs::ObjectsConstPtr& msg);
      void obstacleCallback(const object_msgs::ObjectsConstPtr& msg);

      // state machine
      State nextState(const RobotTime &time, const JointState &current);
      bool shouldAvoidObstacles(const JointState &current);

      // Utils
      Matrix3d getPVA(const JointState &current);
      Vector3d repulsivePotential(const Vector3d distance);
      
      void readParameters(const bool verbose = false);
      
      void readScalarParameter(
        const std::__cxx11::string &key, 
        double &s, 
        double fallback,
        const std::string ns = "~simple_effort_ctrl",
        const bool verbose = false);

      void readDiagonalMatrixDOFdParameter(
        const std::__cxx11::string &key, 
        MatrixDOFd &M,
        const std::string ns = "~simple_effort_ctrl",
        const bool verbose = false);

      MatrixDOFd inverseJacobian(
        const MatrixDOFd J, 
        const double wThreshold = 0.1, 
        const double psi = 0.05);

      VectorDOFd modelCompensation(
        const JointState current, 
        const JointState reference, 
        const VectorDOFd Sq, 
        const ros::Duration dt);

      // geometry
      Matrix3d computeOrientationErrors(
        const JointState &current,
        const Vector3d &omega,
        const Vector3d &omegaP,
        const RobotTime &time);

      ow::Scalar slerpInterpolationParameter(
        const double angularError, 
        const RobotTime &time);

      Quaterniond rotationToGazeTarget(const JointState &current);
      
      Matrix3d circularTrajectory(const double t);
      void generateCircularTrajectoryPath(const int n = 100);

      Vector3d applyTransformation(
        const ow::HomogeneousTransformation transformation,
        Vector3d point);

    };
  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli

#endif // UR_ROBOT_LLI_DEFAULTCONTROL_H
