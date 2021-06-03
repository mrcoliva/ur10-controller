#ifndef JACOBIANS_H_
#define JACOBIANS_H_

#include <ur10_robot_model/model_ur10.h>

using namespace ur;

void UR10Model::matrix_J1_0(ow::MatrixDof &J1_0,
                           const ow::VectorDof &q) const
{

  J1_0(5, 0) = 1;
}

void UR10Model::matrix_J2_0(ow::MatrixDof &J2_0,
                           const ow::VectorDof &q) const
{

  J2_0(0, 0) = -L2 * ((cos(q(1))) * (sin(q(0))));
  J2_0(0, 1) = -L2 * ((cos(q(0))) * (sin(q(1))));
  J2_0(1, 0) = L2 * ((cos(q(1))) * (cos(q(0))));
  J2_0(1, 1) = -L2 * ((sin(q(1))) * (sin(q(0))));
  J2_0(2, 1) = L2 * (cos(q(1)));
  J2_0(3, 1) = sin(q(0));
  J2_0(4, 1) = -(cos(q(0)));
  J2_0(5, 0) = 1;
}

void UR10Model::matrix_J3_0(ow::MatrixDof &J3_0,
                           const ow::VectorDof &q) const
{
  ow::Scalar s1 = sin(q(0));
  ow::Scalar b2 = cos(q(0)) * cos(q(1)) * sin(q(2)) + cos(q(0)) * cos(q(2)) * sin(q(1));
  ow::Scalar b4 = (cos(q(1)) * sin(q(2)) + cos(q(2)) * sin(q(1))) * s1;
  ow::Scalar b5 = cos(q(1)) * cos(q(2)) - sin(q(1)) * sin(q(2));

  J3_0(0, 0) = (sin(q(1)) * sin(q(2)) - cos(q(1)) * cos(q(2))) * L3 * s1 + (-L2 * cos(q(1))) * s1;
  J3_0(0, 1) = -L3 * b2 - L2 * cos(q(0)) * sin(q(1));
  J3_0(0, 2) = -L3 * b2;
  J3_0(1, 0) = (cos(q(0)) * cos(q(1)) * cos(q(2)) - cos(q(0)) * sin(q(1)) * sin(q(2))) * L3 + L2 * cos(q(0)) * cos(q(1));
  J3_0(1, 1) = -L3 * b4 - L2 * s1 * sin(q(1));
  J3_0(1, 2) = -L3 * b4;
  J3_0(2, 1) = L3 * b5 + L2 * cos(q(1));
  J3_0(2, 2) = L3 * b5;
  J3_0(3, 1) = s1;
  J3_0(3, 2) = s1;
  J3_0(4, 1) = -(cos(q(0)));
  J3_0(4, 2) = -(cos(q(0)));
  J3_0(5, 0) = 1;
}

void UR10Model::matrix_J4_0(ow::MatrixDof &J4_0,
                           const ow::VectorDof &q) const
{
  ow::Scalar s1 = sin(q(0));
  ow::Scalar c1 = cos(q(0));
  ow::Scalar c3 = cos(q(2));
  ow::Scalar b2 = (cos(q(1)) * sin(q(2)) + c3 * sin(q(1))) * c1;
  ow::Scalar b4 = (cos(q(1)) * sin(q(2)) + c3 * sin(q(1))) * s1;
  ow::Scalar b5 = c3 * cos(q(1)) - sin(q(1)) * sin(q(2));

  J4_0(0, 0) = (sin(q(1)) * sin(q(2)) - c3 * cos(q(1))) * L3 * s1 + (-L2 * cos(q(1))) * s1 + L4 * c1;
  J4_0(0, 1) = -L3 * b2 - L2 * c1 * sin(q(1));
  J4_0(0, 2) = -L3 * b2;
  J4_0(1, 0) = (c3 * cos(q(1)) - sin(q(1)) * sin(q(2))) * L3 * c1 + (L2 * cos(q(1))) * c1 + L4 * s1;
  J4_0(1, 1) = -L3 * b4 - L2 * s1 * sin(q(1));
  J4_0(1, 2) = -L3 * b4;
  J4_0(2, 1) = L3 * b5 + L2 * cos(q(1));
  J4_0(2, 2) = L3 * b5;
  J4_0(3, 1) = s1;
  J4_0(3, 2) = s1;
  J4_0(3, 3) = s1;
  J4_0(4, 1) = -c1;
  J4_0(4, 2) = -c1;
  J4_0(4, 3) = -c1;
  J4_0(5, 0) = 1;
}

void UR10Model::matrix_J5_0(ow::MatrixDof &J5_0,
                           const ow::VectorDof &q) const
{
  ow::Scalar s1 = sin(q(0));
  ow::Scalar s2 = sin(q(1));
  ow::Scalar s3 = sin(q(2));
  ow::Scalar s4 = sin(q(3));
  ow::Scalar c1 = cos(q(0));
  ow::Scalar c2 = cos(q(1));
  ow::Scalar c3 = cos(q(2));
  ow::Scalar c4 = cos(q(3));
  ow::Scalar b3 = (c2 * s3 + c3 * s2) * c1;
  ow::Scalar b4 = (c2 * s3 * s4 - c2 * c3 * c4 + c3 * s2 * s4 + c4 * s2 * s3) * c1;
  ow::Scalar b7 = (c2 * s3 * s4 - c2 * c3 * c4 + c3 * s2 * s4 + c4 * s2 * s3) * s1;
  ow::Scalar b8 = (c2 * s3 + c3 * s2) * s1;
  ow::Scalar b9 = (c3 * s4 + c4 * s3) * c2 + ((-s2 * s3) * s4 + s2 * c3 * c4);
  ow::Scalar b10 = c2 * c3 - s2 * s3;

  J5_0(0, 0) = (-c3 * s4 - c4 * s3) * L5 * c2 * s1 + (s3 * s4 - c3 * c4) * L5 * s1 * s2 + (-L2 - L3 * c3) * c2 * s1 + (L3 * s3) * s1 * s2 + L4 * c1;
  J5_0(0, 1) = -L3 * b3 - L5 * b4 - L2 * c1 * s2;
  J5_0(0, 2) = -L3 * b3 - L5 * b4;
  J5_0(0, 3) = -L5 * b4;
  J5_0(1, 0) = (L2 * c2 + L3 * c2 * c3 - L3 * s2 * s3 - L5 * s2 * s3 * s4 + L5 * c2 * c3 * s4 + L5 * c2 * c4 * s3 + L5 * c3 * c4 * s2) * c1 + L4 * s1;
  J5_0(1, 1) = -L3 * b8 - L5 * b7 - L2 * s1 * s2;
  J5_0(1, 2) = -L3 * b8 - L5 * b7;
  J5_0(1, 3) = -L5 * b7;
  J5_0(2, 1) = L3 * b10 + L5 * b9 + L2 * c2;
  J5_0(2, 2) = L3 * b10 + L5 * b9;
  J5_0(2, 3) = L5 * b9;
  J5_0(3, 1) = s1;
  J5_0(3, 2) = s1;
  J5_0(3, 3) = s1;
  J5_0(3, 4) = (c2 * c3 * s4 + c2 * c4 * s3 + c3 * c4 * s2 - s2 * s3 * s4) * c1;
  J5_0(4, 1) = -c1;
  J5_0(4, 2) = -c1;
  J5_0(4, 3) = -c1;
  J5_0(4, 4) = (c2 * c3 * s4 + c2 * c4 * s3 + c3 * c4 * s2 - s2 * s3 * s4) * s1;
  J5_0(5, 0) = 1;
  J5_0(5, 4) = (s3 * s4 - c3 * c4) * c2 + ((c4 * s2) * s3 + s2 * c3 * s4);
}

void UR10Model::matrix_J6_0(ow::MatrixDof &J6_0,
                           const ow::VectorDof &q) const
{
  ow::Scalar s1 = sin(q(0));
  ow::Scalar s2 = sin(q(1));
  ow::Scalar s3 = sin(q(2));
  ow::Scalar s4 = sin(q(3));
  ow::Scalar s5 = sin(q(4));
  ow::Scalar c1 = cos(q(0));
  ow::Scalar c2 = cos(q(1));
  ow::Scalar c3 = cos(q(2));
  ow::Scalar c4 = cos(q(3));
  ow::Scalar c5 = cos(q(4));
  ow::Scalar c4s5 = c4 * s5;
  ow::Scalar c1s2 = c1 * s2;
  ow::Scalar b4 = (c1s2 * c4s5 + c1 * c2 * s4 * s5) * c3 + ((c1 * c2) * c4s5 * s3 + (-s4 * s5) * c1s2 * s3);
  ow::Scalar b5 = c3 * c1s2 + c1 * c2 * s3;
  ow::Scalar b6 = (c1s2 * s4 - c1 * c2 * c4) * c3 + ((c1 * c2 * s3) * s4 + s3 * c4 * c1s2);
  ow::Scalar b11 = (c2 * c3 * s4 * s5 + c2 * c4 * s3 * s5 + c3 * c4 * s2 * s5 - s2 * s3 * s4 * s5) * s1;
  ow::Scalar b12 = (c2 * s3 * s4 - c2 * c3 * c4 + c3 * s2 * s4 + c4 * s2 * s3) * s1;
  ow::Scalar b13 = (c2 * s3 + c3 * s2) * s1;
  ow::Scalar b15 = (c3 * s4 + c4 * s3) * c2 + ((-s2 * s3) * s4 + s2 * c3 * c4);
  ow::Scalar b16 = (c2 * s3 * s4 - c2 * c3 * c4 + c3 * s2 * s4) * s5 + c4s5 * s2 * s3;
  ow::Scalar b17 = c2 * c3 - s2 * s3;

  J6_0(0, 0) = (c3 * c4s5 - s3 * s4 * s5) * L6 * c2 * s1 + (-c4s5 * s3 - c3 * s4 * s5) * L6 * s1 * s2 + (c1 * c5) * L6 + (-L2 - L3 * c3 - L5 * c3 * s4 - L5 * c4 * s3) * c2 * s1 + (L3 * s3 - L5 * c3 * c4 + L5 * s3 * s4) * s1 * s2 + L4 * c1;
  J6_0(0, 1) = L6 * b4 - L3 * b5 - L5 * b6 - L2 * c1s2;
  J6_0(0, 2) = L6 * b4 - L3 * b5 - L5 * b6;
  J6_0(0, 3) = L6 * b4 - L5 * b6;
  J6_0(0, 4) = (c3 * c5 * c1s2 * s4 - s1 * s5 + c4 * c5 * c1s2 * s3 - c1 * c2 * c3 * c4 * c5 + c1 * c2 * c5 * s3 * s4) * L6;
  J6_0(1, 0) = (s3 * s4 * s5 - c3 * c4 * s5) * L6 * c1 * c2 + (c3 * s4 * s5 + c4 * s3 * s5) * L6 * c1s2 + (c5 * s1) * L6 + (L2 + L3 * c3 + L5 * c3 * s4 + L5 * c4 * s3) * c1 * c2 + (L5 * c3 * c4 - L3 * s3 - L5 * s3 * s4) * c1s2 + L4 * s1;
  J6_0(1, 1) = L6 * b11 - L5 * b12 - L3 * b13 - L2 * s1 * s2;
  J6_0(1, 2) = L6 * b11 - L5 * b12 - L3 * b13;
  J6_0(1, 3) = L6 * b11 - L5 * b12;
  J6_0(1, 4) = (s3 * s4 - c3 * c4) * L6 * c2 * c5 * s1 + (c3 * s2 * s4 + c4 * s2 * s3) * L6 * c5 * s1 + (c1 * s5) * L6;
  J6_0(2, 1) = L3 * b17 + L5 * b15 + L6 * b16 + L2 * c2;
  J6_0(2, 2) = L3 * b17 + L5 * b15 + L6 * b16;
  J6_0(2, 3) = L5 * b15 + L6 * b16;
  J6_0(2, 4) = (c5 * s2 * s3 * s4 - c2 * c3 * c5 * s4 - c2 * c4 * c5 * s3 - c3 * c4 * c5 * s2) * L6;
  J6_0(3, 1) = s1;
  J6_0(3, 2) = s1;
  J6_0(3, 3) = s1;
  J6_0(3, 4) = (c4 * c1s2 + c1 * c2 * s4) * c3 + ((c1 * c2) * c4 * s3 + (-s4) * c1s2 * s3);
  J6_0(3, 5) = (c3 * c1s2 * s4 + c4 * c1s2 * s3 + c1 * c2 * s3 * s4 - c1 * c2 * c3 * c4) * s5 + c5 * s1;
  J6_0(4, 1) = -c1;
  J6_0(4, 2) = -c1;
  J6_0(4, 3) = -c1;
  J6_0(4, 4) = (c2 * c3 * s4 + c2 * c4 * s3 + c3 * c4 * s2 - s2 * s3 * s4) * s1;
  J6_0(4, 5) = (c4s5 * s2 * s3 - c2 * c3 * c4s5 + c2 * s3 * s4 * s5 + c3 * s2 * s4 * s5) * s1 - c1 * c5;
  J6_0(5, 0) = 1;
  J6_0(5, 4) = (s3 * s4 - c3 * c4) * c2 + ((c4 * s2) * s3 + s2 * c3 * s4);
  J6_0(5, 5) = (s2 * s3 * s4 - c2 * c4 * s3 - c3 * c4 * s2 - c2 * c3 * s4) * s5;
}

#endif