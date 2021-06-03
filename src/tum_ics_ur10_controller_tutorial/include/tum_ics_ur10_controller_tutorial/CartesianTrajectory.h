#ifndef UR_ROBOT_LLI_CARTESIANTRAJECTORY_H
#define UR_ROBOT_LLI_CARTESIANTRAJECTORY_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>

namespace tum_ics_ur_robot_lli
{   

    class CartesianTrajectory
    {
    private:
        double m_t0;
        double m_tf;
        Vector3d m_x0;
        Vector3d m_xf;
        Vector3d m_v0;
        Vector3d m_vf;
        Vector3d m_a0;
        Vector3d m_af;

    public:
        CartesianTrajectory();
        ~CartesianTrajectory();

        void init(
            const double t0,
            const double tf,
            const Vector3d initialPosition,
            const Vector3d finalPosition,
            const Vector3d initialVelocity = Vector3d::Zero(),
            const Vector3d finalVelocity = Vector3d::Zero(),
            const Vector3d initialAcceleration = Vector3d::Zero(),
            const Vector3d finalAcceleration = Vector3d::Zero());

        Matrix3d evaluate(double t);
        Vector3d evaluateScalar(const Vector6d constraints, const double t);

        static Matrix3d circle(const Vector3d startPosition, const double radius, const double period, const double t);

    private:
    };
    
} // namespace tum_ics_ur_robot_lli

#endif // UR_ROBOT_LLI_CARTESIANTRAJECTORY_H
