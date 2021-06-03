#ifndef MATRIX_YR_H_
#define MATRIX_YR_H_

#include <ur10_robot_model/model_ur10.h>
#include <ow_core/common/parameter.h>
#include <string>

namespace ur
{

  UR10Model::UR10Model(const std::string &name) : Base(name),
                                                  M_(ow::MatrixDof::Zero()),
                                                  C_(ow::MatrixDof::Zero()),
                                                  g_(ow::VectorDof::Zero())
  {
    // initalize correct size
    theta_.setZero(81,1);
    Yr_.setZero(6,81);
  }

  UR10Model::~UR10Model()
  {
  }

  const ow::MatrixDof& UR10Model::inertiaMatrix(const ow::VectorDof &q)
  {
    matrix_M(M_, q);
    return M_;
  }

  const ow::MatrixDof& UR10Model::centripetalMatrix(const ow::VectorDof &q, const ow::VectorDof &qP)
  {
    matrix_C(C_, q, qP);
    return C_;
  }

  const ow::VectorDof& UR10Model::gravityVector(const ow::VectorDof &q)
  {
    matrix_G(g_, q);
    return g_;
  }

  const UR10Model::Regressor& UR10Model::regressor(const ow::VectorDof &q, const ow::VectorDof &qP, const ow::VectorDof &qrP, const ow::VectorDof &qrPP)
  {
    matrix_Y(Yr_, q, qP, qrP, qrPP);
    return Yr_;
  }

  const UR10Model::Parameters& UR10Model::parameterInitalGuess()
  {
    return theta_;
  }

  ow::HomogeneousTransformation UR10Model::T_0_B() const
  {
    return T_0_b_;
  }

  ow::HomogeneousTransformation UR10Model::T_B_0() const
  {
    return T_b_0_;
  }

  ow::HomogeneousTransformation UR10Model::T_Tool_Ef() const
  {
    return T_tool_ef_;
  }

  ow::Vector3 UR10Model::g_B() const
  {
    return g_b_;
  }

  ow::Vector3 UR10Model::g_0() const
  {
    return g_0_;
  }

  std::string UR10Model::getBaseFrame() const
  {
    return base_frame_;
  }

  std::string UR10Model::get0Frame() const
  {
    return robot_0_frame_;
  }

  std::string UR10Model::getToolFrame() const
  {
    return tool_frame_;
  }

  ow::HomogeneousTransformation UR10Model::T_ef_0(const ow::VectorDof &q) const
  {
    ow::HomogeneousTransformation T;
    matrix_T6_0(T, q);
    return T;
  }

  ow::HomogeneousTransformation UR10Model::T_ef_B(const ow::VectorDof &q) const
  {
    ow::HomogeneousTransformation T;
    matrix_T6_0(T, q);
    T = T_0_b_ * T;
    return T;
  }

  ow::HomogeneousTransformation UR10Model::T_tool_0(const ow::VectorDof &q) const
  {
    ow::HomogeneousTransformation T;
    matrix_T6_0(T, q);
    T = T * T_tool_ef_;
    return T;
  }

  ow::HomogeneousTransformation UR10Model::T_tool_B(const ow::VectorDof &q) const
  {
    ow::HomogeneousTransformation T;
    matrix_T6_0(T, q);
    T = T_0_b_ * T * T_tool_ef_;
    return T;
  }

  /* joint transformation */
  ow::HomogeneousTransformation UR10Model::T_j_0(const ow::VectorDof &q, int j) const
  {
    ow::HomogeneousTransformation T;
    switch (j)
    {
    case 0:
      matrix_T1_0(T, q);
      break;
    case 1:
      matrix_T2_0(T, q);
      break;
    case 2:
      matrix_T3_0(T, q);
      break;
    case 3:
      matrix_T4_0(T, q);
      break;
    case 4:
      matrix_T5_0(T, q);
      break;
    case 5:
      matrix_T6_0(T, q);
      break;
    }
    return T;
  }

  ow::MatrixDof UR10Model::J_ef_0(const ow::VectorDof &q) const
  {
    ow::MatrixDof J = ow::MatrixDof::Zero();
    matrix_J6_0(J, q);
    return J;
  }

  ow::MatrixDof UR10Model::J_tool_0(const ow::VectorDof &q) const
  {
    ow::MatrixDof J = ow::MatrixDof::Zero();
    matrix_J6_0(J, q);
    matrix_Jt6_0(J, T_tool_ef_.pos(), q);
    return J;
  }

  ow::MatrixDof UR10Model::J_j_0(const ow::VectorDof &q, int j) const
  {
    ow::MatrixDof J = ow::MatrixDof::Zero();
    switch (j)
    {
    case 0:
      matrix_J1_0(J, q);
      break;
    case 1:
      matrix_J2_0(J, q);
      break;
    case 2:
      matrix_J3_0(J, q);
      break;
    case 3:
      matrix_J4_0(J, q);
      break;
    case 4:
      matrix_J5_0(J, q);
      break;
    case 5:
      matrix_J6_0(J, q);
      break;
    }
    return J;
  }

  ow::MatrixDof UR10Model::Jt_j_0(const ow::Vector3 &tj, const ow::VectorDof &q, int j) const
  {
    ow::MatrixDof J = ow::MatrixDof::Zero();
    switch (j)
    {
    case 0:
      matrix_J1_0(J, q);
      matrix_Jt1_0(J, tj, q);
      break;
    case 1:
      matrix_J2_0(J, q);
      matrix_Jt2_0(J, tj, q);
      break;
    case 2:
      matrix_J3_0(J, q);
      matrix_Jt3_0(J, tj, q);
      break;
    case 3:
      matrix_J4_0(J, q);
      matrix_Jt4_0(J, tj, q);
      break;
    case 4:
      matrix_J5_0(J, q);
      matrix_Jt5_0(J, tj, q);
      break;
    case 5:
      matrix_J6_0(J, q);
      matrix_Jt6_0(J, tj, q);
      break;
    }
    return J;
  }

  ow::MatrixDof UR10Model::Jp_ef_0(const ow::VectorDof &q, const ow::VectorDof &qP) const
  {
    ow::MatrixDof Jp = ow::MatrixDof::Zero();
    matrix_J6_0p(Jp, q, qP);
    return Jp;
  }

  ow::MatrixDof UR10Model::Jp_tool_0(const ow::VectorDof &q, const ow::VectorDof &qP) const
  {
    ow::MatrixDof Jp = ow::MatrixDof::Zero();
    matrix_J6_0p(Jp, q, qP);
    matrix_Jt6_0p(Jp, T_tool_ef_.pos(), q, qP);
    return Jp;
  }

  ow::MatrixDof UR10Model::Jp_j_0(const ow::VectorDof &q, const ow::VectorDof &qP, int j) const
  {
    ow::MatrixDof Jp = ow::MatrixDof::Zero();
    switch (j)
    {
    case 0:
      matrix_J1_0p(Jp, q, qP);
      break;
    case 1:
      matrix_J2_0p(Jp, q, qP);
      break;
    case 2:
      matrix_J3_0p(Jp, q, qP);
      break;
    case 3:
      matrix_J4_0p(Jp, q, qP);
      break;
    case 4:
      matrix_J5_0p(Jp, q, qP);
      break;
    case 5:
      matrix_J6_0p(Jp, q, qP);
      break;
    }
    return Jp;
  }

  ow::MatrixDof UR10Model::Jtp_j_0(const ow::Vector3 &tj_0, const ow::VectorDof &q, const ow::VectorDof &qP, int j) const
  {
    ow::MatrixDof Jp = ow::MatrixDof::Zero();
    switch (j)
    {
    case 0:
      matrix_J1_0p(Jp, q, qP);
      matrix_Jt1_0p(Jp, tj_0, q, qP);
      break;
    case 1:
      matrix_J2_0p(Jp, q, qP);
      matrix_Jt2_0p(Jp, tj_0, q, qP);
      break;
    case 2:
      matrix_J3_0p(Jp, q, qP);
      matrix_Jt3_0p(Jp, tj_0, q, qP);
      break;
    case 3:
      matrix_J4_0p(Jp, q, qP);
      matrix_Jt4_0p(Jp, tj_0, q, qP);
      break;
    case 4:
      matrix_J5_0p(Jp, q, qP);
      matrix_Jt5_0p(Jp, tj_0, q, qP);
      break;
    case 5:
      matrix_J6_0p(Jp, q, qP);
      matrix_Jt6_0p(Jp, tj_0, q, qP);
      break;
    }
    return Jp;
  }

  ow::Scalar UR10Model::lowerJointLimits_j(int j) const
  {
    switch (j)
    {
    case 0:
      return lo_jl1;
    case 1:
      return lo_jl2;
    case 2:
      return lo_jl3;
    case 3:
      return lo_jl4;
    case 4:
      return lo_jl5;
    case 5:
      return lo_jl6;
    }
  }

  ow::Scalar UR10Model::upperJointLimits_j(int j) const
  {
    switch (j)
    {
    case 0:
      return hi_jl1;
    case 1:
      return hi_jl2;
    case 2:
      return hi_jl3;
    case 3:
      return hi_jl4;
    case 4:
      return hi_jl5;
    case 5:
      return hi_jl6;
    }
  }

  void UR10Model::broadcastFrames(const ow::VectorDof &q, const ros::Time &time)
  {
    ow::HomogeneousTransformation Transf_j_0, Transf_j_b;

    for (size_t i = 0; i < OW_ROBOT_DOF; ++i)
    {
      Transf_j_0 = T_j_0(q, i);
      Transf_j_b = T_0_b_ * Transf_j_0;

      tf_stamped_transform_[i].setData(Transf_j_b);
      tf_stamped_transform_[i].stamp_ = time;
    }
    Transf_j_b = Transf_j_b * T_tool_ef_;
    tf_stamped_transform_[OW_ROBOT_DOF].setData(Transf_j_b);
    tf_stamped_transform_[OW_ROBOT_DOF].stamp_ = time;

    tf_stamped_transform_[OW_ROBOT_DOF+1].setData(T_0_b_);
    tf_stamped_transform_[OW_ROBOT_DOF+1].stamp_ = time;
    br_.sendTransform(tf_stamped_transform_);
  }

  bool UR10Model::init(ros::NodeHandle &nh)
  {
    std::string ns = Base::name() + '/';

    ow::load(ns+"L1", L1);
    ow::load(ns+"L2", L2);
    ow::load(ns+"L3", L3);
    ow::load(ns+"L4", L4);
    ow::load(ns+"L5", L5);
    ow::load(ns+"L6", L6);
    L7 = 0.0;
    L8 = 0.0;
    L9 = 0.0;
    L10 = 0.0;
    L11 = 0.0;
    L12 = 0.0;

    m1 = 0.0;
    m2 = 0.0;
    m3 = 0.0;
    m4 = 0.0;
    m5 = 0.0;
    m6 = 0.0;

    I111 = 0.0;
    I112 = 0.0;
    I113 = 0.0;
    I122 = 0.0;
    I123 = 0.0;
    I133 = 0.0;

    I211 = 0.0;
    I212 = 0.0;
    I213 = 0.0;
    I222 = 0.0;
    I223 = 0.0;
    I233 = 0.0;

    I311 = 0.0;
    I312 = 0.0;
    I313 = 0.0;
    I322 = 0.0;
    I323 = 0.0;
    I333 = 0.0;

    I411 = 0.0;
    I412 = 0.0;
    I413 = 0.0;
    I422 = 0.0;
    I423 = 0.0;
    I433 = 0.0;

    I511 = 0.0;
    I512 = 0.0;
    I513 = 0.0;
    I522 = 0.0;
    I523 = 0.0;
    I533 = 0.0;

    I611 = 0.0;
    I612 = 0.0;
    I613 = 0.0;
    I622 = 0.0;
    I623 = 0.0;
    I633 = 0.0;

    ow::load(ns+"gx", gx);
    ow::load(ns+"gy", gy);
    ow::load(ns+"gz", gz);
    g_b_ << gx, gy, gz;

    ow::load(ns+"robot_0_frame", robot_0_frame_);
    ow::load(ns+"base_frame", base_frame_);
    ow::load(ns+"tool_frame", tool_frame_);

    ow::CartesianPosition X_0_b;
    ow::load(ns+"X_0_B", X_0_b);
    T_0_b_ = X_0_b;
    T_b_0_ = T_0_b_.inverse();

    ow::CartesianPosition X_tool_ef;
    ow::load(ns+"X_Tool_Ef", X_tool_ef);
    T_tool_ef_ = X_tool_ef;

    // gravity wrt base
    g_0_ = T_0_b_.orientation().inverse() * g_b_;

    // set inital guess theta
    matrix_th(theta_);

    // setup frames
    tf_stamped_transform_.resize(OW_ROBOT_DOF + 2);
    for (size_t i = 0; i < OW_ROBOT_DOF; ++i)
    {
      tf_stamped_transform_[i].frame_id_ = base_frame_;
      tf_stamped_transform_[i].child_frame_id_ = Base::name() + "_dh_" + std::to_string(i);
    }
    tf_stamped_transform_[OW_ROBOT_DOF].frame_id_ = base_frame_;
    tf_stamped_transform_[OW_ROBOT_DOF].child_frame_id_ = Base::name() + "_dh_tool";
    tf_stamped_transform_[OW_ROBOT_DOF+1].frame_id_ = base_frame_;
    tf_stamped_transform_[OW_ROBOT_DOF+1].child_frame_id_ = Base::name() + "_0";

    return true;
  }

} // namespace ur

#endif