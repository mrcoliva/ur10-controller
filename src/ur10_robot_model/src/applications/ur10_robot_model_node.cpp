#include <ur10_robot_model/model_ur10.h>

int main(int argc, char **argv)
{
  // setup ros
  ros::init(argc,argv,"ur10_robot_model_test");
  ros::NodeHandle nh("~");

  // initalize the models parameter
  ur::UR10Model model("ur10_model");
  if(!model.initRequest(nh))
  {
    ROS_ERROR_STREAM("Error initalizing model");
    return -1;
  }

  // call functions
  ow::VectorDof q, qP, qrP, qrPP;
  q << 2.82, -2.03, -1.43, -0.67, -1.0, 0.16;
  qP.setZero();
  qrP.setZero();
  qrPP.setZero();

  ur::UR10Model::Regressor Y = model.regressor(q, qP, qrP, qrPP);

  ur::UR10Model::Parameters th = model.parameterInitalGuess(); 

  Eigen::Affine3d T_ef_0 = model.T_ef_0(q);

  Eigen::Matrix<double,6,6> J_ef_0 = model.J_ef_0(q);

  // print something
  // ROS_WARN_STREAM("T_ef_0=\n" << T_ef_0.matrix());
  // ROS_WARN_STREAM("J_ef_0=\n" << J_ef_0);

  return 0;
}