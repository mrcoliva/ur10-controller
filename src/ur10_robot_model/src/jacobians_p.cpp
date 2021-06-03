#ifndef JACOBIANS_P_H_
#define JACOBIANS_P_H_

#include <ur10_robot_model/model_ur10.h>

using namespace ur;

void UR10Model::matrix_J1_0p(ow::MatrixDof &J1_0p,
                            const ow::VectorDof &q,
                            const ow::VectorDof &qP) const
{
}

void UR10Model::matrix_J2_0p(ow::MatrixDof &J2_0p,
                            const ow::VectorDof &q,
                            const ow::VectorDof &qP) const
{
  ow::Scalar a37 = L2 * qP(1);
  ow::Scalar a38 = L2 * qP(0);

  J2_0p(0, 0) = a37 * sin(q(0)) * sin(q(1)) - a38 * cos(q(0)) * cos(q(1));
  J2_0p(0, 1) = a38 * sin(q(0)) * sin(q(1)) - a37 * cos(q(0)) * cos(q(1));
  J2_0p(1, 0) = -a37 * cos(q(0)) * sin(q(1)) - a38 * cos(q(1)) * sin(q(0));
  J2_0p(1, 1) = -a37 * cos(q(1)) * sin(q(0)) - a38 * cos(q(0)) * sin(q(1));
  J2_0p(2, 1) = -a37 * (sin(q(1)));
  J2_0p(3, 1) = (cos(q(0))) * qP(0);
  J2_0p(4, 1) = qP(0) * (sin(q(0)));
}

void UR10Model::matrix_J3_0p(ow::MatrixDof &J3_0p,
                            const ow::VectorDof &q,
                            const ow::VectorDof &qP) const
{
  ow::Scalar s1 = sin(q(0));
  ow::Scalar s2 = sin(q(1));
  ow::Scalar a37 = L2 * qP(1);
  ow::Scalar a38 = L2 * qP(0);
  ow::Scalar a39 = L3 * qP(0);
  ow::Scalar b1 = cos(q(0)) * cos(q(1)) * cos(q(2)) - s2 * cos(q(0)) * sin(q(2));
  ow::Scalar b2 = (cos(q(1)) * sin(q(2)) + s2 * cos(q(2))) * s1;
  ow::Scalar b3 = (cos(q(1)) * cos(q(2)) - s2 * sin(q(2))) * s1;
  ow::Scalar b4 = s2 * cos(q(0)) * cos(q(2)) + cos(q(0)) * cos(q(1)) * sin(q(2));
  ow::Scalar b5 = cos(q(1)) * sin(q(2)) + s2 * cos(q(2));
  ow::Scalar d1 = (qP(1) + qP(2)) * L3;

  J3_0p(0, 0) = b2 * d1 - a39 * b1 - a38 * cos(q(0)) * cos(q(1)) + a37 * s1 * s2;
  J3_0p(0, 1) = a39 * b2 - b1 * d1 - a37 * cos(q(0)) * cos(q(1)) + a38 * s1 * s2;
  J3_0p(0, 2) = a39 * b2 - b1 * d1;
  J3_0p(1, 0) = -a39 * b3 - b4 * d1 - a37 * s2 * cos(q(0)) - a38 * s1 * cos(q(1));
  J3_0p(1, 1) = -a39 * b4 - b3 * d1 - a37 * s1 * cos(q(1)) - a38 * s2 * cos(q(0));
  J3_0p(1, 2) = -a39 * b4 - b3 * d1;
  J3_0p(2, 1) = -b5 * d1 - a37 * s2;
  J3_0p(2, 2) = -b5 * d1;
  J3_0p(3, 1) = (cos(q(0))) * qP(0);
  J3_0p(3, 2) = (cos(q(0))) * qP(0);
  J3_0p(4, 1) = qP(0) * s1;
  J3_0p(4, 2) = qP(0) * s1;
}

void UR10Model::matrix_J4_0p(ow::MatrixDof &J4_0p,
                            const ow::VectorDof &q,
                            const ow::VectorDof &qP) const
{
  ow::Scalar s1 = sin(q(0));
  ow::Scalar s2 = sin(q(1));
  ow::Scalar c1 = cos(q(0));
  ow::Scalar c3 = cos(q(2));
  ow::Scalar a37 = L2 * qP(1);
  ow::Scalar a38 = L2 * qP(0);
  ow::Scalar a39 = L4 * qP(0);
  ow::Scalar a40 = L3 * qP(0);
  ow::Scalar b1 = (c3 * cos(q(1)) - s2 * sin(q(2))) * c1;
  ow::Scalar b2 = (cos(q(1)) * sin(q(2)) + c3 * s2) * s1;
  ow::Scalar b3 = (c3 * cos(q(1)) - s2 * sin(q(2))) * s1;
  ow::Scalar b4 = (cos(q(1)) * sin(q(2)) + c3 * s2) * c1;
  ow::Scalar b5 = cos(q(1)) * sin(q(2)) + c3 * s2;
  ow::Scalar d1 = (qP(1) + qP(2)) * L3;

  J4_0p(0, 0) = (a37 * s2 - a39) * s1 + (b2 * d1 - a40 * b1 - a38 * c1 * cos(q(1)));
  J4_0p(0, 1) = a40 * b2 - b1 * d1 + a38 * s1 * s2 - a37 * c1 * cos(q(1));
  J4_0p(0, 2) = a40 * b2 - b1 * d1;
  J4_0p(1, 0) = (a39 - a37 * s2) * c1 - (a40 * b3 + b4 * d1 + a38 * s1 * cos(q(1)));
  J4_0p(1, 1) = -a40 * b4 - b3 * d1 - a37 * s1 * cos(q(1)) - a38 * c1 * s2;
  J4_0p(1, 2) = -a40 * b4 - b3 * d1;
  J4_0p(2, 1) = -b5 * d1 - a37 * s2;
  J4_0p(2, 2) = -b5 * d1;
  J4_0p(3, 1) = c1 * qP(0);
  J4_0p(3, 2) = c1 * qP(0);
  J4_0p(3, 3) = c1 * qP(0);
  J4_0p(4, 1) = qP(0) * s1;
  J4_0p(4, 2) = qP(0) * s1;
  J4_0p(4, 3) = qP(0) * s1;
}

void UR10Model::matrix_J5_0p(ow::MatrixDof &J5_0p,
                            const ow::VectorDof &q,
                            const ow::VectorDof &qP) const
{
  ow::Scalar s1 = sin(q(0));
  ow::Scalar s2 = sin(q(1));
  ow::Scalar s4 = sin(q(3));
  ow::Scalar c1 = cos(q(0));
  ow::Scalar c4 = cos(q(3));
  ow::Scalar a37 = L2 * qP(1);
  ow::Scalar a38 = L2 * qP(0);
  ow::Scalar a39 = L4 * qP(0);
  ow::Scalar a40 = L3 * qP(0);
  ow::Scalar a43 = L5 * qP(0);
  ow::Scalar b1 = (c4 * s2 * sin(q(2)) + s2 * s4 * cos(q(2)) - c4 * cos(q(1)) * cos(q(2)) + s4 * cos(q(1)) * sin(q(2))) * s1;
  ow::Scalar b2 = (c4 * s2 * cos(q(2)) - s2 * s4 * sin(q(2)) + c4 * cos(q(1)) * sin(q(2)) + s4 * cos(q(1)) * cos(q(2))) * c1;
  ow::Scalar b3 = (cos(q(1)) * cos(q(2)) - s2 * sin(q(2))) * c1;
  ow::Scalar b4 = (cos(q(1)) * sin(q(2)) + s2 * cos(q(2))) * s1;
  ow::Scalar b5 = (c4 * s2 * sin(q(2)) + s2 * s4 * cos(q(2)) - c4 * cos(q(1)) * cos(q(2)) + s4 * cos(q(1)) * sin(q(2))) * c1;
  ow::Scalar b6 = (c4 * s2 * cos(q(2)) - s2 * s4 * sin(q(2)) + c4 * cos(q(1)) * sin(q(2)) + s4 * cos(q(1)) * cos(q(2))) * s1;
  ow::Scalar b7 = (cos(q(1)) * cos(q(2)) - s2 * sin(q(2))) * s1;
  ow::Scalar b8 = (cos(q(1)) * sin(q(2)) + s2 * cos(q(2))) * c1;
  ow::Scalar b9 = (s2 * sin(q(2)) - cos(q(1)) * cos(q(2))) * c4 + ((cos(q(1)) * sin(q(2))) * s4 + cos(q(2)) * s2 * s4);
  ow::Scalar b10 = cos(q(1)) * sin(q(2)) + s2 * cos(q(2));
  ow::Scalar d1 = (qP(1) + qP(2) + qP(3)) * L5;
  ow::Scalar d2 = (qP(1) + qP(2)) * L3;
  ow::Scalar d3 = qP(1) + qP(2) + qP(3);

  J5_0p(0, 0) = (a37 * s2 - a39) * s1 + (b1 * d1 - a43 * b2 - a40 * b3 + b4 * d2 - a38 * c1 * cos(q(1)));
  J5_0p(0, 1) = a40 * b4 + a43 * b1 - b2 * d1 - b3 * d2 + a38 * s1 * s2 - a37 * c1 * cos(q(1));
  J5_0p(0, 2) = a40 * b4 + a43 * b1 - b2 * d1 - b3 * d2;
  J5_0p(0, 3) = a43 * b1 - b2 * d1;
  J5_0p(1, 0) = (a39 - a37 * s2) * c1 - (a40 * b7 + a43 * b6 + b5 * d1 + b8 * d2 + a38 * s1 * cos(q(1)));
  J5_0p(1, 1) = -a40 * b8 - a43 * b5 - b6 * d1 - b7 * d2 - a37 * s1 * cos(q(1)) - a38 * c1 * s2;
  J5_0p(1, 2) = -a40 * b8 - a43 * b5 - b6 * d1 - b7 * d2;
  J5_0p(1, 3) = -a43 * b5 - b6 * d1;
  J5_0p(2, 1) = -b9 * d1 - b10 * d2 - a37 * s2;
  J5_0p(2, 2) = -b9 * d1 - b10 * d2;
  J5_0p(2, 3) = -b9 * d1;
  J5_0p(3, 1) = c1 * qP(0);
  J5_0p(3, 2) = c1 * qP(0);
  J5_0p(3, 3) = c1 * qP(0);
  J5_0p(3, 4) = -b5 * d3 - b6 * qP(0);
  J5_0p(4, 1) = qP(0) * s1;
  J5_0p(4, 2) = qP(0) * s1;
  J5_0p(4, 3) = qP(0) * s1;
  J5_0p(4, 4) = b2 * qP(0) - b1 * d3;
  J5_0p(5, 4) = (c4 * s2 * cos(q(2)) - s2 * s4 * sin(q(2)) + c4 * cos(q(1)) * sin(q(2)) + s4 * cos(q(1)) * cos(q(2))) * d3;
}

void UR10Model::matrix_J6_0p(ow::MatrixDof &J6_0p,
                            const ow::VectorDof &q,
                            const ow::VectorDof &qP) const
{
  ow::Scalar s1 = sin(q(0));
  ow::Scalar s2 = sin(q(1));
  ow::Scalar s3 = sin(q(2));
  ow::Scalar s4 = sin(q(3));
  ow::Scalar s5 = sin(q(4));
  ow::Scalar c1 = cos(q(0));
  ow::Scalar c4 = cos(q(3));
  ow::Scalar c5 = cos(q(4));
  ow::Scalar s2s1 = s2 * s1;
  ow::Scalar c4s5 = c4 * s5;
  ow::Scalar a37 = L2 * qP(1);
  ow::Scalar a38 = L2 * qP(0);
  ow::Scalar a39 = L6 * qP(0);
  ow::Scalar a40 = L6 * qP(4);
  ow::Scalar a41 = L4 * qP(0);
  ow::Scalar a42 = L3 * qP(0);
  ow::Scalar a45 = L5 * qP(0);
  ow::Scalar b1 = (s2s1 * cos(q(2)) + s1 * s3 * cos(q(1))) * s4 + ((-s1 * cos(q(1)) * cos(q(2))) * c4 + c4 * s3 * s2s1);
  ow::Scalar b2 = (c4 * s2 * cos(q(2)) + c4 * s3 * cos(q(1)) + s4 * cos(q(1)) * cos(q(2)) - s2 * s3 * s4) * c1;
  ow::Scalar b3 = (s2s1 * cos(q(2)) + s1 * s3 * cos(q(1))) * c4s5 + ((s1 * s5 * cos(q(1)) * cos(q(2))) * s4 + (-s5) * s3 * s4 * s2s1);
  ow::Scalar b4 = (cos(q(1)) * cos(q(2)) - s2 * s3) * c1;
  ow::Scalar b5 = (c4s5 * s2 * s3 - c4s5 * cos(q(1)) * cos(q(2)) + s2 * s4 * s5 * cos(q(2)) + s3 * s4 * s5 * cos(q(1))) * c1 + c5 * s1;
  ow::Scalar b6 = (s4 * s2s1 * cos(q(2)) + c4 * s3 * s2s1 + s1 * s3 * s4 * cos(q(1)) - c4 * s1 * cos(q(1)) * cos(q(2))) * c5 + c1 * s5;
  ow::Scalar b7 = s2s1 * cos(q(2)) + s1 * s3 * cos(q(1));
  ow::Scalar b8 = (c4 * c5 * s2 * cos(q(2)) - c5 * s2 * s3 * s4 + c4 * c5 * s3 * cos(q(1)) + c5 * s4 * cos(q(1)) * cos(q(2))) * c1;
  ow::Scalar b9 = (c4s5 * s2 * s3 - c4s5 * cos(q(1)) * cos(q(2)) + s2 * s4 * s5 * cos(q(2)) + s3 * s4 * s5 * cos(q(1))) * c1;
  ow::Scalar b10 = (c4 * s2 * s5 * cos(q(2)) - s2 * s3 * s4 * s5 + c4 * s3 * s5 * cos(q(1)) + s4 * s5 * cos(q(1)) * cos(q(2))) * c1;
  ow::Scalar b11 = (s2 * s4 * cos(q(2)) + s3 * s4 * cos(q(1)) - c4 * cos(q(1)) * cos(q(2)) + c4 * s2 * s3) * c1;
  ow::Scalar b12 = (c4 * s3 * cos(q(1)) + s4 * cos(q(1)) * cos(q(2)) - s2 * s3 * s4) * s1 + (s2s1 * cos(q(2))) * c4;
  ow::Scalar b13 = (cos(q(1)) * cos(q(2)) - s2 * s3) * s1;
  ow::Scalar b14 = (c4 * c5 * s2 * s3 + c5 * s2 * s4 * cos(q(2)) + c5 * s3 * s4 * cos(q(1)) - c4 * c5 * cos(q(1)) * cos(q(2))) * c1 - s1 * s5;
  ow::Scalar b15 = (s4 * s2s1 * cos(q(2)) + c4 * s3 * s2s1 + s1 * s3 * s4 * cos(q(1)) - c4 * s1 * cos(q(1)) * cos(q(2))) * s5 - c1 * c5;
  ow::Scalar b16 = (s2 * cos(q(2)) + s3 * cos(q(1))) * c1;
  ow::Scalar b17 = (s4 * s2s1 * cos(q(2)) + c4 * s3 * s2s1 + s1 * s3 * s4 * cos(q(1)) - c4 * s1 * cos(q(1)) * cos(q(2))) * s5;
  ow::Scalar b18 = (s3 * cos(q(1))) * c4 * c5 * s1 + (s2s1 * cos(q(2))) * c4 * c5 + (cos(q(1)) * cos(q(2)) - s2 * s3) * c5 * s1 * s4;
  ow::Scalar b19 = (s2 * s4 * cos(q(2)) + s3 * s4 * cos(q(1)) - c4 * cos(q(1)) * cos(q(2)) + c4 * s2 * s3) * c5;
  ow::Scalar b20 = (c4 * s3 + s4 * cos(q(2))) * s2 + (cos(q(1)) * s3 * s4 + (-cos(q(1)) * cos(q(2))) * c4);
  ow::Scalar b21 = (s2 * cos(q(2)) + s3 * cos(q(1))) * c4s5 + ((s5 * cos(q(1)) * cos(q(2))) * s4 + (-s5) * s2 * s3 * s4);
  ow::Scalar b22 = s2 * cos(q(2)) + s3 * cos(q(1));
  ow::Scalar d1 = (qP(1) + qP(2) + qP(3)) * L5;
  ow::Scalar d2 = (qP(1) + qP(2) + qP(3)) * L6;
  ow::Scalar d3 = (qP(1) + qP(2)) * L3;
  ow::Scalar d4 = qP(1) + qP(2) + qP(3);

  J6_0p(0, 0) = b1 * d1 - a40 * b6 - a42 * b4 - a45 * b2 - a39 * b5 - b3 * d2 + b7 * d3 - a41 * s1 + a37 * s2s1 - a38 * c1 * cos(q(1));
  J6_0p(0, 1) = a45 * b1 - a39 * b3 + a40 * b8 + a42 * b7 - b2 * d1 - b4 * d3 - b9 * d2 + a38 * s2s1 - a37 * c1 * cos(q(1));
  J6_0p(0, 2) = a45 * b1 - a39 * b3 + a40 * b8 + a42 * b7 - b2 * d1 - b4 * d3 - b9 * d2;
  J6_0p(0, 3) = a45 * b1 - a39 * b3 + a40 * b8 - b2 * d1 - b9 * d2;
  J6_0p(0, 4) = b8 * d2 - a40 * b5 - a39 * b6;
  J6_0p(1, 0) = (a41 - a37 * s2) * c1 + (a40 * b14 - a39 * b15 - a42 * b13 - a45 * b12 + b10 * d2 - b11 * d1 - b16 * d3 - a38 * s1 * cos(q(1)));
  J6_0p(1, 1) = a39 * b10 - a45 * b11 + a40 * b18 - a42 * b16 - b12 * d1 - b13 * d3 - b17 * d2 - a37 * s1 * cos(q(1)) - a38 * c1 * s2;
  J6_0p(1, 2) = a39 * b10 - a45 * b11 + a40 * b18 - a42 * b16 - b12 * d1 - b13 * d3 - b17 * d2;
  J6_0p(1, 3) = a39 * b10 - a45 * b11 + a40 * b18 - b12 * d1 - b17 * d2;
  J6_0p(1, 4) = a39 * b14 - a40 * b15 + b18 * d2;
  J6_0p(2, 1) = a40 * b19 - b20 * d1 + b21 * d2 - b22 * d3 - a37 * s2;
  J6_0p(2, 2) = a40 * b19 - b20 * d1 + b21 * d2 - b22 * d3;
  J6_0p(2, 3) = a40 * b19 - b20 * d1 + b21 * d2;
  J6_0p(2, 4) = a40 * b21 + b19 * d2;
  J6_0p(3, 1) = c1 * qP(0);
  J6_0p(3, 2) = c1 * qP(0);
  J6_0p(3, 3) = c1 * qP(0);
  J6_0p(3, 4) = -b11 * d4 - b12 * qP(0);
  J6_0p(3, 5) = b10 * d4 - b15 * qP(0) + b14 * qP(4);
  J6_0p(4, 1) = qP(0) * s1;
  J6_0p(4, 2) = qP(0) * s1;
  J6_0p(4, 3) = qP(0) * s1;
  J6_0p(4, 4) = b2 * qP(0) - b1 * d4;
  J6_0p(4, 5) = b3 * d4 + b5 * qP(0) + b6 * qP(4);
  J6_0p(5, 4) = (c4 * s2 * cos(q(2)) + c4 * s3 * cos(q(1)) + s4 * cos(q(1)) * cos(q(2)) - s2 * s3 * s4) * d4;
  J6_0p(5, 5) = (d4 * s2 * s3 * s5 - c5 * qP(4) * s2 * cos(q(2)) - c5 * qP(4) * s3 * cos(q(1)) - d4 * s5 * cos(q(1)) * cos(q(2))) * c4 + ((s4 * s5 * cos(q(1))) * d4 * s3 + (s4 * s5 * cos(q(2))) * d4 * s2 + (c5 * qP(4) * s4) * s2 * s3 - c5 * qP(4) * s4 * cos(q(1)) * cos(q(2)));
}

#endif