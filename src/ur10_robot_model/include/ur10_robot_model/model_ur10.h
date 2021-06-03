/**
 * @file model_ur5.h
 *
 * @brief model of ur5 robot
 *
 * @ingroup models
 *
 */

#ifndef MODEL_UR10_H_
#define MODEL_UR10_H_

#include <ur10_robot_model/model_base.h>
#include <ow_core/types.h>
#include <tf/transform_broadcaster.h>

namespace ur
{

  /**
* Defines the kinematic and dynamic model of UR10 robot
*
* used by trajectory generator, controllers, simulator
*/
  class UR10Model :
    public model_interface::ModelBase
  {
  public:
    typedef model_interface::ModelBase Base; 
    typedef ow::MatrixX Regressor;
    typedef ow::VectorX Parameters;
    typedef ow::MatrixX IKSolutions;

  private:
    /* members */
    
    // dynamic model
    ow::MatrixDof M_;
    ow::MatrixDof C_;
    ow::VectorDof g_;

    // robot regressor
    Parameters theta_;
    Regressor Yr_;

    /* parameters */

    // link length
    ow::Scalar L1, L2, L3, L4, L5, L6, L7, L8, L9, L10, L11, L12;

    // link mass
    ow::Scalar m1, m2, m3, m4, m5, m6;

    // inertia tensors
    ow::Scalar I111, I112, I113, I122, I123, I133;
    ow::Scalar I211, I212, I213, I222, I223, I233;
    ow::Scalar I311, I312, I313, I322, I323, I333;
    ow::Scalar I411, I412, I413, I422, I423, I433;
    ow::Scalar I511, I512, I513, I522, I523, I533;
    ow::Scalar I611, I612, I613, I622, I623, I633;

    // gravity components
    ow::Scalar gx, gy, gz;

    // joint limits
    ow::Scalar lo_jl1, lo_jl2, lo_jl3, lo_jl4, lo_jl5, lo_jl6;
    ow::Scalar hi_jl1, hi_jl2, hi_jl3, hi_jl4, hi_jl5, hi_jl6;

    // gravity vectors
    ow::Vector3 g_b_, g_0_;              // gravity vectors

    // base to world transformation
    ow::HomogeneousTransformation T_0_b_;   
    ow::HomogeneousTransformation T_b_0_; 

    // tool to ef transformation
    ow::HomogeneousTransformation T_tool_ef_;    

    std::string robot_0_frame_;  // 0 frame tf name
    std::string base_frame_;     // base frame tf name
    std::string tool_frame_;     // tool frame tf name

    // broadcast frames
    tf::TransformBroadcaster br_;
    std::vector<tf::StampedTransform> tf_stamped_transform_;     // stack of tf transformations

  public:
    /** 
    * @brief constructor
    *
    * @param name of robot
    */
    UR10Model(const std::string &name = "ur10_model");

    /**
     * @brief Destroy the UR10Model object
     */
    virtual ~UR10Model();

    /** 
    * @brief inertia matrix computation
    *
    * @param inertia Matrix
    * @param joint position
    */
    const ow::MatrixDof& inertiaMatrix(const ow::VectorDof &q);

    /** 
    * @brief coriolis centripetal matrix computation
    *
    * @param coriolis centripetal matrix
    * @param joint position
    * @param joint velocity
    */
    const ow::MatrixDof& centripetalMatrix(const ow::VectorDof &q, const ow::VectorDof &qP);

    /** 
    * @brief gravitational vector computation
    *
    * @param gravitational vector
    * @param joint position
    */
    const ow::VectorDof& gravityVector(const ow::VectorDof &q);

    /** 
    * @brief regressor matrix computation
    *
    * @param regressor matrix
    * @param joint position
    * @param joint velocity
    * @param reference joint velocity
    * @param reference joint acceleration
    */
    const Regressor& regressor(const ow::VectorDof &q, const ow::VectorDof &qP, const ow::VectorDof &qrP, const ow::VectorDof &qrPP);

    /**
     * @brief inital guess for theta
     * 
     * @return const Parameters& 
     */
    const Parameters& parameterInitalGuess(); 

    /**
     * @brief broad cast frames in tf tree (for visualization)
     * 
     */
    void broadcastFrames(const ow::VectorDof &q, const ros::Time& time);

    /** 
    * @brief base frame wrt world framem_robotModel.T_ef_B(current.q).matrix()
    */
    ow::HomogeneousTransformation T_0_B() const;

    /** 
    * @brief world frame wrt base frame
    */
    ow::HomogeneousTransformation T_B_0() const;

    /**
     * @brief Tool frame wrt ef frame
     */
    ow::HomogeneousTransformation T_Tool_Ef() const;

    /** 
    * @brief gravity vector wrt word
    */
    ow::Vector3 g_B() const;

    /** 
    * @brief gravity vector wrt base
    */
    ow::Vector3 g_0() const;

    /** 
    * @brief endeffector transformation matrix wrt 0 frame
    */
    ow::HomogeneousTransformation T_ef_0(const ow::VectorDof &q) const;

    /** 
    * @brief endeffector transformation matrix wrt to base frame
    */
    ow::HomogeneousTransformation T_ef_B(const ow::VectorDof &q) const;

    /** 
    * @brief tool transformation matrix wrt 0 frame
    */
    ow::HomogeneousTransformation T_tool_0(const ow::VectorDof &q) const;

    /** 
    * @brief endeffector transformation matrix wrt to base frame
    */
    ow::HomogeneousTransformation T_tool_B(const ow::VectorDof &q) const;

    /** 
    * @brief joint transformation j (0-5) wrt 0 frame
    */
    ow::HomogeneousTransformation T_j_0(const ow::VectorDof &q, int j) const;

    /** 
    * @brief endeffector jacobian matrix wrt 0 frame
    */
    ow::MatrixDof J_ef_0(const ow::VectorDof &q) const;

    /** 
    * @brief tool jacobian matrix  wrt 0 frame
    */
    ow::MatrixDof J_tool_0(const ow::VectorDof &q) const;

    /** 
    * @brief jacobian matrix at j (0-5) wrt 0 frame
    */
    ow::MatrixDof J_j_0(const ow::VectorDof &q, int j) const;

    /** 
    * @brief jacobian matrix at j offseted by length tj wrt 0 frame 
    */
    ow::MatrixDof Jt_j_0(const ow::Vector3 &tj, const ow::VectorDof &q, int j) const;

    /** 
    * @brief jacobian derivative of end effector wrt 0 frame
    */
    ow::MatrixDof Jp_ef_0(const ow::VectorDof &q, const ow::VectorDof &qP) const;

    /** 
    * @brief jacobian derivative of tool wrt 0 frame
    */
    ow::MatrixDof Jp_tool_0(const ow::VectorDof &q, const ow::VectorDof &qP) const;

    /** 
    * @brief jacobian derivatives of at j (0-5) wrt 0 frame
    */
    ow::MatrixDof Jp_j_0(const ow::VectorDof &q, const ow::VectorDof &qP, int j) const;

    /** 
    * @brief jacobian derivatives at j offseted by length tj wrt 0 frame 
    */
    ow::MatrixDof Jtp_j_0(const ow::Vector3 &tj, const ow::VectorDof &q, const ow::VectorDof &qP, int j) const;

    /**
     * @brief return lower joint limits
     * 
     */
    ow::Scalar lowerJointLimits_j(int j) const;

    /**
     * @brief return upper joint limits
     * 
     */
    ow::Scalar upperJointLimits_j(int j) const;

    /**
     * @brief Get base Frame name
     */
    std::string getBaseFrame() const;

    /**
     * @brief Get base Frame name
     */
    std::string get0Frame() const;

    /**
     * @brief Get the Tool Frame name
     */
    std::string getToolFrame() const;

  private:
    virtual bool init(ros::NodeHandle& nh);

  private:
    /* matlab generated functions */

    /* inertia matrix */
    void matrix_M(ow::MatrixDof &M,
                  const ow::VectorDof &q) const;

    /* coriolis centripetal matrix */
    void matrix_C(ow::MatrixDof &C,
                  const ow::VectorDof &q,
                  const ow::VectorDof &qP) const;

    /* gravitational vector */
    void matrix_G(ow::VectorDof &G,
                  const ow::VectorDof &q) const;

    /* regressor matrix */
    void matrix_Y(Regressor &Y,
                  const ow::VectorDof &q,
                  const ow::VectorDof &qP,
                  const ow::VectorDof &qrP,
                  const ow::VectorDof &qrPP) const;

    /* parameter vector theta */
    void matrix_th(Parameters &th) const;

    /* transformations */
    void matrix_T1_0(ow::HomogeneousTransformation &T1_0,
                     const ow::VectorDof &q) const;
    void matrix_T2_0(ow::HomogeneousTransformation &T2_0,
                     const ow::VectorDof &q) const;
    void matrix_T3_0(ow::HomogeneousTransformation &T3_0,
                     const ow::VectorDof &q) const;
    void matrix_T4_0(ow::HomogeneousTransformation &T4_0,
                     const ow::VectorDof &q) const;
    void matrix_T5_0(ow::HomogeneousTransformation &T5_0,
                     const ow::VectorDof &q) const;
    void matrix_T6_0(ow::HomogeneousTransformation &T6_0,
                     const ow::VectorDof &q) const;

    /* jacobians */
    void matrix_J1_0(ow::MatrixDof &J1_0,
                     const ow::VectorDof &q) const;
    void matrix_J2_0(ow::MatrixDof &J2_0,
                     const ow::VectorDof &q) const;
    void matrix_J3_0(ow::MatrixDof &J3_0,
                     const ow::VectorDof &q) const;
    void matrix_J4_0(ow::MatrixDof &J4_0,
                     const ow::VectorDof &q) const;
    void matrix_J5_0(ow::MatrixDof &J5_0,
                     const ow::VectorDof &q) const;
    void matrix_J6_0(ow::MatrixDof &J6_0,
                     const ow::VectorDof &q) const;

    void matrix_Jt1_0(ow::MatrixDof &Jt1_0,
                      const ow::Vector3 &t,
                      const ow::VectorDof &q) const;
    void matrix_Jt2_0(ow::MatrixDof &Jt2_0,
                      const ow::Vector3 &t,
                      const ow::VectorDof &q) const;
    void matrix_Jt3_0(ow::MatrixDof &Jt3_0,
                      const ow::Vector3 &t,
                      const ow::VectorDof &q) const;
    void matrix_Jt4_0(ow::MatrixDof &Jt4_0,
                      const ow::Vector3 &t,
                      const ow::VectorDof &q) const;
    void matrix_Jt5_0(ow::MatrixDof &Jt5_0,
                      const ow::Vector3 &t,
                      const ow::VectorDof &q) const;
    void matrix_Jt6_0(ow::MatrixDof &Jt6_0,
                      const ow::Vector3 &t,
                      const ow::VectorDof &q) const;

    /* jacobian derivatives */
    void matrix_J1_0p(ow::MatrixDof &J1_0p,
                      const ow::VectorDof &q,
                      const ow::VectorDof &qP) const;
    void matrix_J2_0p(ow::MatrixDof &J2_0p,
                      const ow::VectorDof &q,
                      const ow::VectorDof &qP) const;
    void matrix_J3_0p(ow::MatrixDof &J3_0p,
                      const ow::VectorDof &q,
                      const ow::VectorDof &qP) const;
    void matrix_J4_0p(ow::MatrixDof &J4_0p,
                      const ow::VectorDof &q,
                      const ow::VectorDof &qP) const;
    void matrix_J5_0p(ow::MatrixDof &J5_0p,
                      const ow::VectorDof &q,
                      const ow::VectorDof &qP) const;
    void matrix_J6_0p(ow::MatrixDof &J6_0p,
                      const ow::VectorDof &q,
                      const ow::VectorDof &qP) const;

    void matrix_Jt1_0p(ow::MatrixDof &Jt1_0p,
                       const ow::Vector3 &t,
                       const ow::VectorDof &q,
                       const ow::VectorDof &qP) const;
    void matrix_Jt2_0p(ow::MatrixDof &Jt2_0p,
                       const ow::Vector3 &t,
                       const ow::VectorDof &q,
                       const ow::VectorDof &qP) const;
    void matrix_Jt3_0p(ow::MatrixDof &Jt3_0p,
                       const ow::Vector3 &t,
                       const ow::VectorDof &q,
                       const ow::VectorDof &qP) const;
    void matrix_Jt4_0p(ow::MatrixDof &Jt4_0p,
                       const ow::Vector3 &t,
                       const ow::VectorDof &q,
                       const ow::VectorDof &qP) const;
    void matrix_Jt5_0p(ow::MatrixDof &Jt5_0p,
                       const ow::Vector3 &t,
                       const ow::VectorDof &q,
                       const ow::VectorDof &qP) const;
    void matrix_Jt6_0p(ow::MatrixDof &Jt6_0p,
                       const ow::Vector3 &t,
                       const ow::VectorDof &q,
                       const ow::VectorDof &qP) const;
  };

} // namespace ur

#endif
