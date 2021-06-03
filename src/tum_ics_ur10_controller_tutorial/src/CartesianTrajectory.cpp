#include <tum_ics_ur10_controller_tutorial/CartesianTrajectory.h>

namespace tum_ics_ur_robot_lli
{

    CartesianTrajectory::CartesianTrajectory()
    {
    }

    CartesianTrajectory::~CartesianTrajectory()
    {
    }
    
    void CartesianTrajectory::init(
            const double t0,
            const double tf,
            const Vector3d initialPosition,
            const Vector3d finalPosition,
            const Vector3d initialVelocity,
            const Vector3d finalVelocity,
            const Vector3d initialAcceleration,
            const Vector3d finalAcceleration)
    {
        m_t0 = t0;
        m_tf = tf;
        m_x0 = initialPosition; 
        m_xf = finalPosition; 
        m_v0 = initialVelocity; 
        m_vf = finalVelocity; 
        m_a0 = initialAcceleration; 
        m_af = finalAcceleration; 
    }

    Matrix3d CartesianTrajectory::evaluate(double t) {
        Matrix3d T;
        T.setZero();

        for (int i = 0; i < 3; i++) {
            Vector6d constraints;
            constraints << m_x0[i], m_xf[i], m_v0[i], m_vf[i], m_a0[i], m_af[i];

            T.row(i) = evaluateScalar(constraints, t);
        }

        return T;
    }

    Vector3d CartesianTrajectory::evaluateScalar(const Vector6d constraints, double t) {
        t = std::max(0.0, std::min(t, m_tf));

        Matrix6d T;

        T << 1, m_t0, pow(m_t0, 2), pow(m_t0, 3), pow(m_t0, 4), pow(m_t0, 5),
             1, m_tf, pow(m_tf, 2), pow(m_tf, 3), pow(m_tf, 4), pow(m_tf, 5),
             0, 1,  2*m_t0, 3*pow(m_t0, 2), 4*pow(m_t0, 3), 5*pow(m_t0, 4),
             0, 1,  2*m_tf, 3*pow(m_tf, 2), 4*pow(m_tf, 3), 5*pow(m_tf, 4),
             0, 0,  2, 6*m_t0, 12*pow(m_t0, 2), 20*pow(m_t0, 3),
             0, 0,  2, 6*m_tf, 12*pow(m_tf, 2), 20*pow(m_tf, 3);

        Vector6d a = T.householderQr().solve(constraints);

        Vector3d trajectory;
        trajectory << a[0] + a[1]*t + a[2]*pow(t, 2) + a[3]*pow(t, 3) + a[4]*pow(t, 4) + a[5]*pow(t, 5),
                      0 + a[1] + 2*a[2]*t + 3*a[3]*pow(t, 2) + 4*a[4]*pow(t, 3) + 5*a[5]*pow(t, 4),
                      2*a[2] + 6*a[3]*t + 12*a[4]*pow(t, 2) + 20*a[5]*pow(t, 3);
        
        return trajectory;
    }

    Matrix3d CartesianTrajectory::circle(
        const Vector3d startPosition, 
        const double radius, 
        const double period, 
        const double t)
    {
      double angle = M_PI / period * t;

      double x = startPosition[0] + radius * sin(2*angle);
      double y = startPosition[1] + radius * cos(2*angle) - radius;
      double z = startPosition[2]; // constant

      double xp = radius * cos(2*angle);
      double yp = - radius * sin(2*angle);
      double zp = 0;

      double xpp = - radius * sin(2*angle);
      double ypp = - radius * cos(2*angle);
      double zpp = 0;

      Vector3d currentPosition(x, y, z);
      Vector3d currentVelocity(xp, yp, zp);
      Vector3d currentAcceleration(xpp, ypp, zpp);

      Matrix3d M;
      M.col(0) = currentPosition;
      M.col(1) = currentVelocity;
      M.col(2) = currentAcceleration;

      return M;
    }

} // namespace tum_ics_ur_robot_lli
