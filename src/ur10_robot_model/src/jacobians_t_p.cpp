#ifndef JACOBIANS_T_P_H_
#define JACOBIANS_T_P_H_

#include <ur10_robot_model/model_ur10.h>

using namespace ur;

void UR10Model::matrix_Jt1_0p(ow::MatrixDof &Jt1_0p,
                             const ow::Vector3 &t,
                             const ow::VectorDof &q,
                             const ow::VectorDof &qP) const
{
  ow::Scalar a37 = qP(0) * t(0);
  ow::Scalar a38 = qP(0) * t(2);

  Jt1_0p(0, 0) += -a37 * cos(q(0)) - a38 * sin(q(0));
  Jt1_0p(1, 0) += a38 * cos(q(0)) - a37 * sin(q(0));
}

void UR10Model::matrix_Jt2_0p(ow::MatrixDof &Jt2_0p,
                             const ow::Vector3 &t,
                             const ow::VectorDof &q,
                             const ow::VectorDof &qP) const
{
  ow::Scalar s2s1 = (sin(q(1))) * (sin(q(0)));
  ow::Scalar c2c1 = (cos(q(1))) * (cos(q(0)));
  ow::Scalar c1s2 = (cos(q(0))) * (sin(q(1)));
  ow::Scalar c2s1 = (cos(q(1))) * (sin(q(0)));
  ow::Scalar a37 = qP(1) * t(0);
  ow::Scalar a38 = qP(0) * t(2);
  ow::Scalar a39 = qP(0) * t(0);
  ow::Scalar a40 = qP(0) * t(1);
  ow::Scalar a41 = qP(1) * t(1);

  Jt2_0p(0, 0) += a40 * c1s2 - a39 * c2c1 + a41 * c2s1 + a37 * s2s1 - a38 * sin(q(0));
  Jt2_0p(0, 1) += a41 * c1s2 - a37 * c2c1 + a40 * c2s1 + a39 * s2s1;
  Jt2_0p(1, 0) += a40 * s2s1 - a37 * c1s2 - a39 * c2s1 - a41 * c2c1 + a38 * cos(q(0));
  Jt2_0p(1, 1) += a41 * s2s1 - a39 * c1s2 - a37 * c2s1 - a40 * c2c1;
  Jt2_0p(2, 1) += -a41 * cos(q(1)) - a37 * sin(q(1));
}

void UR10Model::matrix_Jt3_0p(ow::MatrixDof &Jt3_0p,
                             const ow::Vector3 &t,
                             const ow::VectorDof &q,
                             const ow::VectorDof &qP) const
{
  ow::Scalar s2 = sin(q(1));
  ow::Scalar s3 = sin(q(2));
  ow::Scalar a37 = qP(0) * t(1);
  ow::Scalar a38 = qP(0) * t(0);
  ow::Scalar a39 = qP(0) * t(2);
  ow::Scalar b1 = cos(q(1)) * cos(q(2)) * sin(q(0)) - s2 * s3 * sin(q(0));
  ow::Scalar b2 = cos(q(0)) * cos(q(1)) * cos(q(2)) - s2 * s3 * cos(q(0));
  ow::Scalar b3 = s2 * cos(q(0)) * cos(q(2)) + s3 * cos(q(0)) * cos(q(1));
  ow::Scalar b4 = s2 * cos(q(2)) * sin(q(0)) + s3 * cos(q(1)) * sin(q(0));
  ow::Scalar b5 = cos(q(1)) * cos(q(2)) - s2 * s3;
  ow::Scalar b6 = s2 * cos(q(2)) + s3 * cos(q(1));
  ow::Scalar d1 = (qP(1) + qP(2)) * t(1);
  ow::Scalar d2 = (qP(1) + qP(2)) * t(0);

  Jt3_0p(0, 0) += a37 * b3 - a38 * b2 + b1 * d1 + b4 * d2 - a39 * sin(q(0));
  Jt3_0p(0, 1) += a37 * b1 + a38 * b4 - b2 * d2 + b3 * d1;
  Jt3_0p(0, 2) += a37 * b1 + a38 * b4 - b2 * d2 + b3 * d1;
  Jt3_0p(1, 0) += a37 * b4 - a38 * b1 - b2 * d1 - b3 * d2 + a39 * cos(q(0));
  Jt3_0p(1, 1) += b4 * d1 - a38 * b3 - b1 * d2 - a37 * b2;
  Jt3_0p(1, 2) += b4 * d1 - a38 * b3 - b1 * d2 - a37 * b2;
  Jt3_0p(2, 1) += -b5 * d1 - b6 * d2;
  Jt3_0p(2, 2) += -b5 * d1 - b6 * d2;
}

void UR10Model::matrix_Jt4_0p(ow::MatrixDof &Jt4_0p,
                             const ow::Vector3 &t,
                             const ow::VectorDof &q,
                             const ow::VectorDof &qP) const
{
  ow::Scalar s2 = sin(q(1));
  ow::Scalar s4 = sin(q(3));
  ow::Scalar a37 = qP(0) * t(0);
  ow::Scalar a38 = qP(0) * t(2);
  ow::Scalar a42 = qP(0) * t(1);
  ow::Scalar b1 = (s4 * cos(q(0)) * cos(q(2)) + cos(q(0)) * cos(q(3)) * sin(q(2))) * s2 + ((cos(q(0)) * cos(q(1)) * sin(q(2))) * s4 - cos(q(0)) * cos(q(1)) * cos(q(2)) * cos(q(3)));
  ow::Scalar b2 = (cos(q(0)) * cos(q(2)) * cos(q(3)) - s4 * cos(q(0)) * sin(q(2))) * s2 + ((cos(q(0)) * cos(q(1)) * cos(q(2))) * s4 + cos(q(0)) * cos(q(1)) * cos(q(3)) * sin(q(2)));
  ow::Scalar b3 = (cos(q(2)) * cos(q(3)) * sin(q(0)) - s4 * sin(q(0)) * sin(q(2))) * s2 + ((cos(q(1)) * cos(q(2)) * sin(q(0))) * s4 + cos(q(1)) * cos(q(3)) * sin(q(0)) * sin(q(2)));
  ow::Scalar b4 = (s4 * cos(q(2)) * sin(q(0)) + cos(q(3)) * sin(q(0)) * sin(q(2))) * s2 + ((cos(q(1)) * sin(q(0)) * sin(q(2))) * s4 - cos(q(1)) * cos(q(2)) * cos(q(3)) * sin(q(0)));
  ow::Scalar b5 = (cos(q(2)) * cos(q(3)) - s4 * sin(q(2))) * s2 + ((cos(q(1)) * cos(q(2))) * s4 + cos(q(1)) * cos(q(3)) * sin(q(2)));
  ow::Scalar b6 = (cos(q(3)) * sin(q(2)) + s4 * cos(q(2))) * s2 + ((cos(q(1)) * sin(q(2))) * s4 - cos(q(1)) * cos(q(2)) * cos(q(3)));
  ow::Scalar d1 = (qP(1) + qP(2) + qP(3)) * t(2);
  ow::Scalar d2 = (qP(1) + qP(2) + qP(3)) * t(0);

  Jt4_0p(0, 0) += a37 * b1 - a38 * b2 + b3 * d2 + b4 * d1 - a42 * sin(q(0));
  Jt4_0p(0, 1) += a37 * b3 + a38 * b4 + b1 * d2 - b2 * d1;
  Jt4_0p(0, 2) += a37 * b3 + a38 * b4 + b1 * d2 - b2 * d1;
  Jt4_0p(0, 3) += a37 * b3 + a38 * b4 + b1 * d2 - b2 * d1;
  Jt4_0p(1, 0) += a37 * b4 - a38 * b3 - b1 * d1 - b2 * d2 + a42 * cos(q(0));
  Jt4_0p(1, 1) += b4 * d2 - a38 * b1 - b3 * d1 - a37 * b2;
  Jt4_0p(1, 2) += b4 * d2 - a38 * b1 - b3 * d1 - a37 * b2;
  Jt4_0p(1, 3) += b4 * d2 - a38 * b1 - b3 * d1 - a37 * b2;
  Jt4_0p(2, 1) += -b5 * d2 - b6 * d1;
  Jt4_0p(2, 2) += -b5 * d2 - b6 * d1;
  Jt4_0p(2, 3) += -b5 * d2 - b6 * d1;
}

void UR10Model::matrix_Jt5_0p(ow::MatrixDof &Jt5_0p,
                             const ow::Vector3 &t,
                             const ow::VectorDof &q,
                             const ow::VectorDof &qP) const
{
  ow::Scalar s1 = sin(q(0));
  ow::Scalar s3 = sin(q(2));
  ow::Scalar s4 = sin(q(3));
  ow::Scalar s5 = sin(q(4));
  ow::Scalar c2 = cos(q(1));
  ow::Scalar c5 = cos(q(4));
  ow::Scalar s5s3 = s5 * s3;
  ow::Scalar a37 = qP(4) * t(0);
  ow::Scalar a38 = qP(0) * t(0);
  ow::Scalar a39 = qP(0) * t(2);
  ow::Scalar a40 = qP(4) * t(2);
  ow::Scalar a41 = qP(0) * t(1);
  ow::Scalar b1 = (s3 * cos(q(0)) * cos(q(3)) + s4 * cos(q(0)) * cos(q(2))) * c2 + ((-cos(q(0)) * sin(q(1))) * s3 * s4 + cos(q(0)) * cos(q(2)) * cos(q(3)) * sin(q(1)));
  ow::Scalar b2 = (s3 * cos(q(3)) * sin(q(1)) - c2 * cos(q(2)) * cos(q(3)) + s4 * cos(q(2)) * sin(q(1)) + c2 * s3 * s4) * s1;
  ow::Scalar b3 = (c2 * s5s3 * cos(q(3)) - s4 * s5s3 * sin(q(1)) + s5 * cos(q(2)) * cos(q(3)) * sin(q(1)) + c2 * s4 * s5 * cos(q(2))) * s1;
  ow::Scalar b4 = (s1 * cos(q(2)) * cos(q(3)) * sin(q(1)) + c2 * s1 * s3 * cos(q(3)) + c2 * s1 * s4 * cos(q(2)) - s1 * s3 * s4 * sin(q(1))) * c5;
  ow::Scalar b5 = (s3 * cos(q(0)) * cos(q(3)) * sin(q(1)) - c2 * cos(q(0)) * cos(q(2)) * cos(q(3)) + s4 * cos(q(0)) * cos(q(2)) * sin(q(1))) * s5 + (c5 * s1 + (s5s3 * cos(q(0))) * c2 * s4);
  ow::Scalar b6 = (c2 * s1 * s3 * s4 - c2 * s1 * cos(q(2)) * cos(q(3)) + s1 * s3 * cos(q(3)) * sin(q(1)) + s1 * s4 * cos(q(2)) * sin(q(1))) * c5 + s5 * cos(q(0));
  ow::Scalar b7 = (c2 * s3 * s4 * s5 - c2 * s5 * cos(q(2)) * cos(q(3)) + s3 * s5 * cos(q(3)) * sin(q(1)) + s4 * s5 * cos(q(2)) * sin(q(1))) * s1 - c5 * cos(q(0));
  ow::Scalar b8 = (s3 * cos(q(0)) * cos(q(3)) * sin(q(1)) - c2 * cos(q(0)) * cos(q(2)) * cos(q(3)) + s4 * cos(q(0)) * cos(q(2)) * sin(q(1)) + c2 * s3 * s4 * cos(q(0))) * c5 - s1 * s5;
  ow::Scalar b9 = (cos(q(0)) * cos(q(2)) * cos(q(3)) * sin(q(1)) + c2 * s3 * cos(q(0)) * cos(q(3)) + c2 * s4 * cos(q(0)) * cos(q(2)) - s3 * s4 * cos(q(0)) * sin(q(1))) * c5;
  ow::Scalar b10 = (cos(q(0)) * cos(q(2)) * cos(q(3)) * sin(q(1)) + c2 * s3 * cos(q(0)) * cos(q(3)) + c2 * s4 * cos(q(0)) * cos(q(2)) - s3 * s4 * cos(q(0)) * sin(q(1))) * s5;
  ow::Scalar b11 = (s3 * cos(q(0)) * cos(q(3)) * sin(q(1)) - c2 * cos(q(0)) * cos(q(2)) * cos(q(3)) + s4 * cos(q(0)) * cos(q(2)) * sin(q(1)) + c2 * s3 * s4 * cos(q(0))) * c5;
  ow::Scalar b12 = (s3 * cos(q(0)) * cos(q(3)) * sin(q(1)) - c2 * cos(q(0)) * cos(q(2)) * cos(q(3)) + s4 * cos(q(0)) * cos(q(2)) * sin(q(1))) * s5 + (s5s3 * cos(q(0))) * c2 * s4;
  ow::Scalar b13 = (c2 * s4 * cos(q(0)) + cos(q(0)) * cos(q(3)) * sin(q(1))) * s3 + ((cos(q(0)) * cos(q(2)) * sin(q(1))) * s4 + (-cos(q(0)) * cos(q(2)) * cos(q(3))) * c2);
  ow::Scalar b14 = (c2 * s3 * cos(q(3)) + c2 * s4 * cos(q(2)) - s3 * s4 * sin(q(1)) + cos(q(2)) * cos(q(3)) * sin(q(1))) * s1;
  ow::Scalar b15 = (c2 * s1 * s3 * s4 - c2 * s1 * cos(q(2)) * cos(q(3)) + s1 * s3 * cos(q(3)) * sin(q(1)) + s1 * s4 * cos(q(2)) * sin(q(1))) * c5;
  ow::Scalar b16 = (c2 * s3 * s4 * s5 - c2 * s5 * cos(q(2)) * cos(q(3)) + s3 * s5 * cos(q(3)) * sin(q(1)) + s4 * s5 * cos(q(2)) * sin(q(1))) * s1;
  ow::Scalar b17 = (c2 * s4 * cos(q(2)) - s3 * s4 * sin(q(1)) + cos(q(2)) * cos(q(3)) * sin(q(1))) * s5 + (s5s3 * cos(q(3))) * c2;
  ow::Scalar b18 = (c2 * s3 * cos(q(3)) + c2 * s4 * cos(q(2)) - s3 * s4 * sin(q(1)) + cos(q(2)) * cos(q(3)) * sin(q(1))) * c5;
  ow::Scalar b19 = (cos(q(3)) * sin(q(1)) + c2 * s4) * s3 + ((cos(q(2)) * sin(q(1))) * s4 + (-cos(q(2)) * cos(q(3))) * c2);
  ow::Scalar b20 = (s3 * cos(q(3)) * sin(q(1)) - c2 * cos(q(2)) * cos(q(3)) + s4 * cos(q(2)) * sin(q(1)) + c2 * s3 * s4) * c5;
  ow::Scalar b21 = (s3 * cos(q(3)) * sin(q(1)) - c2 * cos(q(2)) * cos(q(3)) + s4 * cos(q(2)) * sin(q(1)) + c2 * s3 * s4) * s5;
  ow::Scalar d1 = (qP(1) + qP(2) + qP(3)) * t(1);
  ow::Scalar d2 = (qP(1) + qP(2) + qP(3)) * t(0);
  ow::Scalar d3 = (qP(1) + qP(2) + qP(3)) * t(2);

  Jt5_0p(0, 0) += a41 * b1 - a37 * b7 - a39 * b5 + a38 * b8 - a40 * b6 - b2 * d1 - b3 * d3 + b4 * d2;
  Jt5_0p(0, 1) += a38 * b4 - a39 * b3 - a41 * b2 + a37 * b10 + a40 * b9 + b1 * d1 + b11 * d2 - b12 * d3;
  Jt5_0p(0, 2) += a38 * b4 - a39 * b3 - a41 * b2 + a37 * b10 + a40 * b9 + b1 * d1 + b11 * d2 - b12 * d3;
  Jt5_0p(0, 3) += a38 * b4 - a39 * b3 - a41 * b2 + a37 * b10 + a40 * b9 + b1 * d1 + b11 * d2 - b12 * d3;
  Jt5_0p(0, 4) += a37 * b8 - a38 * b7 - a39 * b6 - a40 * b5 + b9 * d3 + b10 * d2;
  Jt5_0p(1, 0) += a37 * b5 + a38 * b6 - a39 * b7 + a40 * b8 + a41 * b14 - b9 * d2 + b10 * d3 + b13 * d1;
  Jt5_0p(1, 1) += a37 * b3 + a40 * b4 - a38 * b9 + a39 * b10 + a41 * b13 + b14 * d1 + b15 * d2 - b16 * d3;
  Jt5_0p(1, 2) += a37 * b3 + a40 * b4 - a38 * b9 + a39 * b10 + a41 * b13 + b14 * d1 + b15 * d2 - b16 * d3;
  Jt5_0p(1, 3) += a37 * b3 + a40 * b4 - a38 * b9 + a39 * b10 + a41 * b13 + b14 * d1 + b15 * d2 - b16 * d3;
  Jt5_0p(1, 4) += a37 * b6 + a38 * b5 + a39 * b8 - a40 * b7 + b3 * d2 + b4 * d3;
  Jt5_0p(2, 1) += a37 * b21 + a40 * b20 + b17 * d3 - b18 * d2 + b19 * d1;
  Jt5_0p(2, 2) += a37 * b21 + a40 * b20 + b17 * d3 - b18 * d2 + b19 * d1;
  Jt5_0p(2, 3) += a37 * b21 + a40 * b20 + b17 * d3 - b18 * d2 + b19 * d1;
  Jt5_0p(2, 4) += a40 * b17 - a37 * b18 + b20 * d3 + b21 * d2;
}

void UR10Model::matrix_Jt6_0p(ow::MatrixDof &Jt6_0p,
                             const ow::Vector3 &t,
                             const ow::VectorDof &q,
                             const ow::VectorDof &qP) const
{
  ow::Scalar s1 = sin(q(0));
  ow::Scalar s3 = sin(q(2));
  ow::Scalar s4 = sin(q(3));
  ow::Scalar s5 = sin(q(4));
  ow::Scalar s6 = sin(q(5));
  ow::Scalar c5 = cos(q(4));
  ow::Scalar c6 = cos(q(5));
  ow::Scalar c5c1 = c5 * (cos(q(0)));
  ow::Scalar c5s1 = c5 * s1;
  ow::Scalar s5s1 = s5 * s1;
  ow::Scalar s6s1 = s6 * s1;
  ow::Scalar c3s2 = (cos(q(2))) * (sin(q(1)));
  ow::Scalar a37 = qP(4) * t(0);
  ow::Scalar a38 = qP(4) * t(2);
  ow::Scalar a39 = qP(0) * t(2);
  ow::Scalar a40 = qP(4) * t(1);
  ow::Scalar a41 = qP(5) * t(1);
  ow::Scalar a42 = qP(0) * t(0);
  ow::Scalar a43 = qP(5) * t(0);
  ow::Scalar a44 = qP(0) * t(1);
  ow::Scalar b1 = (cos(q(0)) * cos(q(1)) * cos(q(2)) - s3 * cos(q(0)) * sin(q(1))) * c6 * s4 + (s3 * cos(q(0)) * cos(q(1)) * cos(q(3)) + cos(q(0)) * cos(q(2)) * cos(q(3)) * sin(q(1))) * c6 + (-c3s2 - s3 * cos(q(1))) * c5c1 * s4 * s6 + (cos(q(1)) * cos(q(2)) * cos(q(3)) - s3 * cos(q(3)) * sin(q(1))) * c5c1 * s6 + s5s1 * s6;
  ow::Scalar b2 = (-s3 * s6 * sin(q(1))) * c5 * s1 * s4 + (s6s1 * cos(q(1)) * cos(q(2))) * c5 * s4 + (c3s2 * s6s1 * cos(q(3)) + s3 * s6s1 * cos(q(1)) * cos(q(3))) * c5 + (c3s2 + s3 * cos(q(1))) * c6 * s1 * s4 + (s3 * cos(q(3)) * sin(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * c6 * s1;
  ow::Scalar b3 = (c3s2 * cos(q(3)) + s3 * cos(q(1)) * cos(q(3))) * s5s1 + ((s5 * cos(q(1)) * cos(q(2))) * s1 * s4 + (-s5 * sin(q(1))) * s1 * s3 * s4);
  ow::Scalar b4 = (s1 * s3 * sin(q(1)) - s1 * cos(q(1)) * cos(q(2))) * c6 * s4 + (-c3s2 * s1 * cos(q(3)) - s1 * s3 * cos(q(1)) * cos(q(3))) * c6 + (c3s2 + s3 * cos(q(1))) * c5s1 * s4 * s6 + (s3 * cos(q(3)) * sin(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * c5s1 * s6 + (s5 * cos(q(0))) * s6;
  ow::Scalar b5 = (s5 * cos(q(0)) + c5 * c3s2 * s1 * s4 + c5 * s1 * s3 * cos(q(3)) * sin(q(1)) - c5 * s1 * cos(q(1)) * cos(q(2)) * cos(q(3)) + c5 * s1 * s3 * s4 * cos(q(1))) * c6 + ((s6s1 * cos(q(1)) * cos(q(2))) * s4 + (s6s1 * cos(q(1)) * cos(q(3))) * s3 + c3s2 * s6s1 * cos(q(3)) + (-s6 * sin(q(1))) * s1 * s3 * s4);
  ow::Scalar b6 = (s3 * cos(q(0)) * cos(q(3)) * sin(q(1)) + s4 * cos(q(0)) * cos(q(2)) * sin(q(1)) - cos(q(0)) * cos(q(1)) * cos(q(2)) * cos(q(3)) + s3 * s4 * cos(q(0)) * cos(q(1))) * s5 + c5s1;
  ow::Scalar b7 = (c3s2 * s4 + s3 * s4 * cos(q(1)) + s3 * cos(q(3)) * sin(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * c5s1 + s5 * cos(q(0));
  ow::Scalar b8 = (c3s2 + s3 * cos(q(1))) * c6 * c5c1 * s4 + (s3 * cos(q(3)) * sin(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * c6 * c5c1 + (-s5s1) * c6 + (cos(q(0)) * cos(q(1)) * cos(q(2)) - s3 * cos(q(0)) * sin(q(1))) * s4 * s6 + (s3 * cos(q(0)) * cos(q(1)) * cos(q(3)) + cos(q(0)) * cos(q(2)) * cos(q(3)) * sin(q(1))) * s6;
  ow::Scalar b9 = (c3s2 + s3 * cos(q(1))) * c6 * s1 * s4 * s5 + (s3 * cos(q(3)) * sin(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * c6 * s1 * s5 + (-c5c1) * c6;
  ow::Scalar b10 = (cos(q(1)) * cos(q(2)) * cos(q(3)) - s3 * s4 * cos(q(1)) - s3 * cos(q(3)) * sin(q(1)) - c3s2 * s4) * s6s1 + ((c6 * c3s2 * s1 * cos(q(3))) * c5 + (c6 * s1 * cos(q(1)) * cos(q(2))) * c5 * s4 + (c6 * s1 * cos(q(1)) * cos(q(3))) * c5 * s3 + (-c6 * s1 * sin(q(1))) * c5 * s3 * s4);
  ow::Scalar b11 = (c3s2 * s4 * s6s1 - s6s1 * cos(q(1)) * cos(q(2)) * cos(q(3)) + s3 * s4 * s6s1 * cos(q(1)) + s3 * s6s1 * cos(q(3)) * sin(q(1))) * s5 - c5c1 * s6;
  ow::Scalar b12 = (s3 * cos(q(0)) * cos(q(1)) * cos(q(3)) + s4 * cos(q(0)) * cos(q(1)) * cos(q(2)) + cos(q(0)) * cos(q(2)) * cos(q(3)) * sin(q(1))) * c5 + (-c5c1 * sin(q(1))) * s3 * s4;
  ow::Scalar b13 = (s3 * cos(q(0)) * cos(q(3)) * sin(q(1)) + s4 * cos(q(0)) * cos(q(2)) * sin(q(1)) - cos(q(0)) * cos(q(1)) * cos(q(2)) * cos(q(3)) + s3 * s4 * cos(q(0)) * cos(q(1))) * s5;
  ow::Scalar b14 = (s6 * cos(q(0)) * cos(q(2)) * cos(q(3)) * sin(q(1)) - s3 * s4 * s6 * cos(q(0)) * sin(q(1)) + s3 * s6 * cos(q(0)) * cos(q(1)) * cos(q(3)) + s4 * s6 * cos(q(0)) * cos(q(1)) * cos(q(2))) * s5;
  ow::Scalar b15 = (s5 * cos(q(0)) * cos(q(2)) * cos(q(3)) * sin(q(1)) - s3 * s4 * s5 * cos(q(0)) * sin(q(1)) + s3 * s5 * cos(q(0)) * cos(q(1)) * cos(q(3)) + s4 * s5 * cos(q(0)) * cos(q(1)) * cos(q(2))) * c6;
  ow::Scalar b16 = (s3 * cos(q(0)) * sin(q(1)) - cos(q(0)) * cos(q(1)) * cos(q(2))) * c6 * s4 + (-s3 * cos(q(0)) * cos(q(1)) * cos(q(3)) - cos(q(0)) * cos(q(2)) * cos(q(3)) * sin(q(1))) * c6 + (c3s2 + s3 * cos(q(1))) * c5c1 * s4 * s6 + (s3 * cos(q(3)) * sin(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * c5c1 * s6;
  ow::Scalar b17 = (s6 * cos(q(0)) * cos(q(3)) * sin(q(1)) + s4 * s6 * cos(q(0)) * cos(q(1)) + c5 * c6 * s4 * cos(q(0)) * sin(q(1)) - c5 * c6 * cos(q(0)) * cos(q(1)) * cos(q(3))) * s3 + ((cos(q(0)) * cos(q(2)) * sin(q(1))) * s4 * s6 + (-cos(q(0)) * cos(q(1)) * cos(q(2)) * cos(q(3))) * s6 + (-c6 * cos(q(0)) * cos(q(2)) * cos(q(3)) * sin(q(1))) * c5 + (-c6 * cos(q(0)) * cos(q(1)) * cos(q(2))) * c5 * s4);
  ow::Scalar b18 = (s3 * cos(q(0)) * cos(q(3)) * sin(q(1)) + s4 * cos(q(0)) * cos(q(2)) * sin(q(1)) - cos(q(0)) * cos(q(1)) * cos(q(2)) * cos(q(3)) + s3 * s4 * cos(q(0)) * cos(q(1))) * c6 + ((c5 * cos(q(0)) * cos(q(2)) * cos(q(3)) * sin(q(1))) * s6 + (c5 * cos(q(0)) * cos(q(1)) * cos(q(2))) * s4 * s6 + (c5 * cos(q(0)) * cos(q(1)) * cos(q(3))) * s3 * s6 + (-c5c1 * sin(q(1))) * s3 * s4 * s6);
  ow::Scalar b19 = (c3s2 + s3 * cos(q(1))) * c6 * c5c1 * s4 + (s3 * cos(q(3)) * sin(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * c6 * c5c1 + (cos(q(0)) * cos(q(1)) * cos(q(2)) - s3 * cos(q(0)) * sin(q(1))) * s4 * s6 + (s3 * cos(q(0)) * cos(q(1)) * cos(q(3)) + cos(q(0)) * cos(q(2)) * cos(q(3)) * sin(q(1))) * s6;
  ow::Scalar b20 = (c5 * s1 - s5 * cos(q(0)) * cos(q(1)) * cos(q(2)) * cos(q(3)) + s3 * s4 * s5 * cos(q(0)) * cos(q(1)) + s3 * s5 * cos(q(0)) * cos(q(3)) * sin(q(1)) + s4 * s5 * cos(q(0)) * cos(q(2)) * sin(q(1))) * c6;
  ow::Scalar b21 = (c5s1 - s5 * cos(q(0)) * cos(q(1)) * cos(q(2)) * cos(q(3)) + s3 * s4 * s5 * cos(q(0)) * cos(q(1)) + s3 * s5 * cos(q(0)) * cos(q(3)) * sin(q(1)) + s4 * s5 * cos(q(0)) * cos(q(2)) * sin(q(1))) * s6;
  ow::Scalar b24 = (s3 * cos(q(0)) * cos(q(1)) * cos(q(3)) + s4 * cos(q(0)) * cos(q(1)) * cos(q(2)) + cos(q(0)) * cos(q(2)) * cos(q(3)) * sin(q(1)) - s3 * s4 * cos(q(0)) * sin(q(1))) * s5;
  ow::Scalar b25 = (c3s2 * s4 + s3 * s4 * cos(q(1)) + s3 * cos(q(3)) * sin(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * s5s1 - c5c1;
  ow::Scalar b26 = (s3 * cos(q(3)) * sin(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * c5c1 + ((cos(q(0)) * cos(q(2)) * sin(q(1))) * c5 * s4 - s5s1 + (cos(q(0)) * cos(q(1))) * c5 * s3 * s4);
  ow::Scalar b27 = (c5 * c6 * c3s2 * s4 - s3 * s4 * s6 * sin(q(1)) + c5 * c6 * s3 * cos(q(3)) * sin(q(1)) - c5 * c6 * cos(q(1)) * cos(q(2)) * cos(q(3)) + c5 * c6 * s3 * s4 * cos(q(1))) * s1 + ((s6s1 * cos(q(1)) * cos(q(2))) * s4 + (s6s1 * cos(q(1)) * cos(q(3))) * s3 + c3s2 * s6s1 * cos(q(3)));
  ow::Scalar b28 = (cos(q(1)) * cos(q(3)) - s4 * sin(q(1))) * c6 * s1 * s3 * s5 + (c3s2 * cos(q(3)) + s4 * cos(q(1)) * cos(q(2))) * c6 * s1 * s5;
  ow::Scalar b29 = (c3s2 * s4 + s3 * s4 * cos(q(1)) + s3 * cos(q(3)) * sin(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * s5s1;
  ow::Scalar b30 = (c3s2 * cos(q(3)) - s3 * s4 * sin(q(1)) + s3 * cos(q(1)) * cos(q(3)) + s4 * cos(q(1)) * cos(q(2))) * c5s1;
  ow::Scalar b31 = (c3s2 * s6s1 * cos(q(3)) + s3 * s6s1 * cos(q(1)) * cos(q(3)) + s4 * s6s1 * cos(q(1)) * cos(q(2)) - s1 * s3 * s4 * s6 * sin(q(1))) * s5;
  ow::Scalar b32 = (cos(q(1)) * cos(q(2)) - s3 * sin(q(1))) * c6 * s1 * s4 + (c3s2 * cos(q(3)) + s3 * cos(q(1)) * cos(q(3))) * c6 * s1 + (-c3s2 * s6 - s3 * s6 * cos(q(1))) * c5s1 * s4 + (s6 * cos(q(1)) * cos(q(2)) * cos(q(3)) - s3 * s6 * cos(q(3)) * sin(q(1))) * c5s1;
  ow::Scalar b35 = (cos(q(1)) * cos(q(2)) - s3 * sin(q(1))) * c5 * s4 * s6 + (c3s2 * cos(q(3)) + s3 * cos(q(1)) * cos(q(3))) * c5 * s6 + (c3s2 + s3 * cos(q(1))) * c6 * s4 + (s3 * cos(q(3)) * sin(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * c6;
  ow::Scalar b36 = (c3s2 * cos(q(3)) - s3 * s4 * sin(q(1)) + s3 * cos(q(1)) * cos(q(3)) + s4 * cos(q(1)) * cos(q(2))) * s6 + ((-c6 * cos(q(1)) * cos(q(2)) * cos(q(3))) * c5 + (c6 * c3s2) * c5 * s4 + (c6 * cos(q(3)) * sin(q(1))) * c5 * s3 + (c6 * cos(q(1))) * c5 * s3 * s4);
  ow::Scalar b37 = (c3s2 * s4 * s5 - s5 * cos(q(1)) * cos(q(2)) * cos(q(3)) + s3 * s4 * s5 * cos(q(1)) + s3 * s5 * cos(q(3)) * sin(q(1))) * c6;
  ow::Scalar b38 = (c3s2 * cos(q(3)) - s3 * s4 * sin(q(1)) + s3 * cos(q(1)) * cos(q(3)) + s4 * cos(q(1)) * cos(q(2))) * s5;
  ow::Scalar b39 = (c3s2 * s4 + s3 * s4 * cos(q(1)) + s3 * cos(q(3)) * sin(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * c5;
  ow::Scalar b40 = (c3s2 * s4 * s6 - s6 * cos(q(1)) * cos(q(2)) * cos(q(3)) + s3 * s4 * s6 * cos(q(1)) + s3 * s6 * cos(q(3)) * sin(q(1))) * s5;
  ow::Scalar b41 = (-c3s2 - s3 * cos(q(1))) * c5 * s4 * s6 + (cos(q(1)) * cos(q(2)) * cos(q(3)) - s3 * cos(q(3)) * sin(q(1))) * c5 * s6 + (cos(q(1)) * cos(q(2)) - s3 * sin(q(1))) * c6 * s4 + (c3s2 * cos(q(3)) + s3 * cos(q(1)) * cos(q(3))) * c6;
  ow::Scalar b42 = (cos(q(1)) * cos(q(2)) - s3 * sin(q(1))) * c5 * c6 * s4 + (c3s2 * cos(q(3)) + s3 * cos(q(1)) * cos(q(3))) * c5 * c6 + (-c3s2 - s3 * cos(q(1))) * s4 * s6 + (cos(q(1)) * cos(q(2)) * cos(q(3)) - s3 * cos(q(3)) * sin(q(1))) * s6;
  ow::Scalar b44 = (c3s2 * s5 * cos(q(3)) - s3 * s4 * s5 * sin(q(1)) + s3 * s5 * cos(q(1)) * cos(q(3)) + s4 * s5 * cos(q(1)) * cos(q(2))) * c6;
  ow::Scalar b46 = (c3s2 * s6 * cos(q(3)) - s3 * s4 * s6 * sin(q(1)) + s3 * s6 * cos(q(1)) * cos(q(3)) + s4 * s6 * cos(q(1)) * cos(q(2))) * s5;
  ow::Scalar d1 = (qP(1) + qP(2) + qP(3)) * t(1);
  ow::Scalar d2 = (qP(1) + qP(2) + qP(3)) * t(0);
  ow::Scalar d3 = (qP(1) + qP(2) + qP(3)) * t(2);

  Jt6_0p(0, 0) += a44 * b1 - a39 * b6 - a38 * b7 - a37 * b9 - a41 * b5 - a43 * b4 + a42 * b8 + a40 * b11 - b2 * d1 - b3 * d3 + b10 * d2;
  Jt6_0p(0, 1) += a38 * b12 - a44 * b2 - a39 * b3 + a37 * b15 + a42 * b10 - a40 * b14 - a41 * b17 + a43 * b18 - b13 * d3 - b16 * d1 + b19 * d2;
  Jt6_0p(0, 2) += a38 * b12 - a44 * b2 - a39 * b3 + a37 * b15 + a42 * b10 - a40 * b14 - a41 * b17 + a43 * b18 - b13 * d3 - b16 * d1 + b19 * d2;
  Jt6_0p(0, 3) += a38 * b12 - a44 * b2 - a39 * b3 + a37 * b15 + a42 * b10 - a40 * b14 - a41 * b17 + a43 * b18 - b13 * d3 - b16 * d1 + b19 * d2;
  Jt6_0p(0, 4) += (c3s2 * s4 + s3 * s4 * cos(q(1)) + s3 * cos(q(3)) * sin(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * a37 * c6 * c5c1 + (-s5s1) * a37 * c6 + (s6 * cos(q(1)) * cos(q(2)) * cos(q(3)) - c3s2 * s4 * s6 - s3 * s4 * s6 * cos(q(1)) - s3 * s6 * cos(q(3)) * sin(q(1))) * a40 * c5c1 + (s6 * s5s1) * a40 + (a44 * b11 - a39 * b7 - a42 * b9 - a38 * b6 - a41 * b20 - a43 * b21 + b12 * d3 - b14 * d1 + b15 * d2);
  Jt6_0p(0, 5) += a41 * b1 - a42 * b4 - a44 * b5 + a43 * b8 - a37 * b21 - a40 * b20 - b17 * d1 + b18 * d2;
  Jt6_0p(1, 0) += a42 * b5 - a43 * b1 - a44 * b4 + a41 * b8 + a37 * b20 - a40 * b21 + a38 * b26 - a39 * b25 + b17 * d2 + b18 * d1 + b24 * d3;
  Jt6_0p(1, 1) += a43 * b2 + a41 * b10 + a42 * b17 + a44 * b18 + a39 * b24 + a37 * b28 + a38 * b30 - a40 * b31 + b27 * d2 - b29 * d3 + b32 * d1;
  Jt6_0p(1, 2) += a43 * b2 + a41 * b10 + a42 * b17 + a44 * b18 + a39 * b24 + a37 * b28 + a38 * b30 - a40 * b31 + b27 * d2 - b29 * d3 + b32 * d1;
  Jt6_0p(1, 3) += a43 * b2 + a41 * b10 + a42 * b17 + a44 * b18 + a39 * b24 + a37 * b28 + a38 * b30 - a40 * b31 + b27 * d2 - b29 * d3 + b32 * d1;
  Jt6_0p(1, 4) += (s5 * cos(q(0)) + c5 * c3s2 * s1 * s4 + c5 * s1 * s3 * cos(q(3)) * sin(q(1)) - c5 * s1 * cos(q(1)) * cos(q(2)) * cos(q(3)) + c5 * s1 * s3 * s4 * cos(q(1))) * a37 * c6 + (c5s1 * cos(q(1)) * cos(q(2)) * cos(q(3)) - c3s2 * c5s1 * s4 - s5 * cos(q(0)) - c5s1 * s3 * s4 * cos(q(1)) - c5s1 * s3 * cos(q(3)) * sin(q(1))) * a40 * s6 + (a42 * b20 - a43 * b11 - a41 * b9 - a38 * b25 + a39 * b26 - a44 * b21 + b28 * d2 - b31 * d1 + b30 * d3);
  Jt6_0p(1, 5) += a43 * b5 - a41 * b4 - a37 * b11 - a42 * b1 - a40 * b9 + a44 * b8 + b2 * d2 + b10 * d1;
  Jt6_0p(2, 1) += a37 * b37 + a38 * b39 + a41 * b36 - a40 * b40 - a43 * b41 + b35 * d1 + b38 * d3 - b42 * d2;
  Jt6_0p(2, 2) += a37 * b37 + a38 * b39 + a41 * b36 - a40 * b40 - a43 * b41 + b35 * d1 + b38 * d3 - b42 * d2;
  Jt6_0p(2, 3) += a37 * b37 + a38 * b39 + a41 * b36 - a40 * b40 - a43 * b41 + b35 * d1 + b38 * d3 - b42 * d2;
  Jt6_0p(2, 4) += (s3 * s4 * sin(q(1)) - c3s2 * cos(q(3)) - s3 * cos(q(1)) * cos(q(3)) - s4 * cos(q(1)) * cos(q(2))) * a37 * c5 * c6 + (c3s2 * s6 * cos(q(3)) - s3 * s4 * s6 * sin(q(1)) + s3 * s6 * cos(q(1)) * cos(q(3)) + s4 * s6 * cos(q(1)) * cos(q(2))) * a40 * c5 + (a38 * b38 + a41 * b44 + a43 * b46 + b37 * d2 - b40 * d1 + b39 * d3);
  Jt6_0p(2, 5) += a41 * b35 + a37 * b46 + a40 * b44 - a43 * b42 + b36 * d1 - b41 * d2;
}

#endif