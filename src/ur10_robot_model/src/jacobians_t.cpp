#ifndef JACOBIANS_T_H_
#define JACOBIANS_T_H_

#include <ur10_robot_model/model_ur10.h>

using namespace ur;

void UR10Model::matrix_Jt1_0(ow::MatrixDof &Jt1_0,
                            const ow::Vector3 &t,
                            const ow::VectorDof &q) const
{

  Jt1_0(0, 0) += t(2) * cos(q(0)) - t(0) * sin(q(0));
  Jt1_0(1, 0) += t(0) * cos(q(0)) + t(2) * sin(q(0));
}

void UR10Model::matrix_Jt2_0(ow::MatrixDof &Jt2_0,
                            const ow::Vector3 &t,
                            const ow::VectorDof &q) const
{

  Jt2_0(0, 0) += t(2) * cos(q(0)) - t(0) * cos(q(1)) * sin(q(0)) + t(1) * sin(q(0)) * sin(q(1));
  Jt2_0(0, 1) += -t(1) * cos(q(0)) * cos(q(1)) - t(0) * cos(q(0)) * sin(q(1));
  Jt2_0(1, 0) += t(2) * sin(q(0)) + t(0) * cos(q(0)) * cos(q(1)) - t(1) * cos(q(0)) * sin(q(1));
  Jt2_0(1, 1) += -t(1) * cos(q(1)) * sin(q(0)) - t(0) * sin(q(0)) * sin(q(1));
  Jt2_0(2, 1) += t(0) * cos(q(1)) - t(1) * sin(q(1));
}

void UR10Model::matrix_Jt3_0(ow::MatrixDof &Jt3_0,
                            const ow::Vector3 &t,
                            const ow::VectorDof &q) const
{
  ow::Scalar s3 = sin(q(2));
  ow::Scalar b1 = cos(q(1)) * cos(q(2)) * sin(q(0)) - s3 * sin(q(0)) * sin(q(1));
  ow::Scalar b2 = s3 * cos(q(1)) * sin(q(0)) + cos(q(2)) * sin(q(0)) * sin(q(1));
  ow::Scalar b3 = cos(q(0)) * cos(q(1)) * cos(q(2)) - s3 * cos(q(0)) * sin(q(1));
  ow::Scalar b4 = s3 * cos(q(0)) * cos(q(1)) + cos(q(0)) * cos(q(2)) * sin(q(1));
  ow::Scalar b5 = cos(q(1)) * cos(q(2)) - s3 * sin(q(1));
  ow::Scalar b6 = cos(q(2)) * sin(q(1)) + s3 * cos(q(1));

  Jt3_0(0, 0) += b2 * t(1) - b1 * t(0) + t(2) * cos(q(0));
  Jt3_0(0, 1) += -b3 * t(1) - b4 * t(0);
  Jt3_0(0, 2) += -b3 * t(1) - b4 * t(0);
  Jt3_0(1, 0) += b3 * t(0) - b4 * t(1) + t(2) * sin(q(0));
  Jt3_0(1, 1) += -b1 * t(1) - b2 * t(0);
  Jt3_0(1, 2) += -b1 * t(1) - b2 * t(0);
  Jt3_0(2, 1) += b5 * t(0) - b6 * t(1);
  Jt3_0(2, 2) += b5 * t(0) - b6 * t(1);
}

void UR10Model::matrix_Jt4_0(ow::MatrixDof &Jt4_0,
                            const ow::Vector3 &t,
                            const ow::VectorDof &q) const
{
  ow::Scalar s2 = sin(q(1));
  ow::Scalar s4 = sin(q(3));
  ow::Scalar b1 = (s4 * cos(q(2)) * sin(q(0)) + cos(q(3)) * sin(q(0)) * sin(q(2))) * s2 + ((cos(q(1)) * sin(q(0)) * sin(q(2))) * s4 - cos(q(1)) * cos(q(2)) * cos(q(3)) * sin(q(0)));
  ow::Scalar b2 = (cos(q(2)) * cos(q(3)) * sin(q(0)) - s4 * sin(q(0)) * sin(q(2))) * s2 + ((cos(q(1)) * cos(q(2)) * sin(q(0))) * s4 + cos(q(1)) * cos(q(3)) * sin(q(0)) * sin(q(2)));
  ow::Scalar b3 = (s4 * cos(q(0)) * cos(q(2)) + cos(q(0)) * cos(q(3)) * sin(q(2))) * s2 + ((cos(q(0)) * cos(q(1)) * sin(q(2))) * s4 - cos(q(0)) * cos(q(1)) * cos(q(2)) * cos(q(3)));
  ow::Scalar b4 = (cos(q(0)) * cos(q(2)) * cos(q(3)) - s4 * cos(q(0)) * sin(q(2))) * s2 + ((cos(q(0)) * cos(q(1)) * cos(q(2))) * s4 + cos(q(0)) * cos(q(1)) * cos(q(3)) * sin(q(2)));
  ow::Scalar b5 = (cos(q(2)) * cos(q(3)) - s4 * sin(q(2))) * s2 + ((cos(q(1)) * cos(q(2))) * s4 + cos(q(1)) * cos(q(3)) * sin(q(2)));
  ow::Scalar b6 = (cos(q(3)) * sin(q(2)) + s4 * cos(q(2))) * s2 + ((cos(q(1)) * sin(q(2))) * s4 - cos(q(1)) * cos(q(2)) * cos(q(3)));

  Jt4_0(0, 0) += b1 * t(0) - b2 * t(2) + t(1) * cos(q(0));
  Jt4_0(0, 1) += -b4 * t(0) - b3 * t(2);
  Jt4_0(0, 2) += -b4 * t(0) - b3 * t(2);
  Jt4_0(0, 3) += -b4 * t(0) - b3 * t(2);
  Jt4_0(1, 0) += b4 * t(2) - b3 * t(0) + t(1) * sin(q(0));
  Jt4_0(1, 1) += -b2 * t(0) - b1 * t(2);
  Jt4_0(1, 2) += -b2 * t(0) - b1 * t(2);
  Jt4_0(1, 3) += -b2 * t(0) - b1 * t(2);
  Jt4_0(2, 1) += b5 * t(2) - b6 * t(0);
  Jt4_0(2, 2) += b5 * t(2) - b6 * t(0);
  Jt4_0(2, 3) += b5 * t(2) - b6 * t(0);
}

void UR10Model::matrix_Jt5_0(ow::MatrixDof &Jt5_0,
                            const ow::Vector3 &t,
                            const ow::VectorDof &q) const
{
  ow::Scalar s1 = sin(q(0));
  ow::Scalar s2 = sin(q(1));
  ow::Scalar s3 = sin(q(2));
  ow::Scalar s4 = sin(q(3));
  ow::Scalar s5 = sin(q(4));
  ow::Scalar c1 = cos(q(0));
  ow::Scalar c5 = cos(q(4));
  ow::Scalar b2 = (s1 * s2 * s3 * cos(q(3)) - s1 * cos(q(1)) * cos(q(2)) * cos(q(3)) + s1 * s2 * s4 * cos(q(2)) + s1 * s3 * s4 * cos(q(1))) * c5 + c1 * s5;
  ow::Scalar b3 = (s2 * s3 * s5 * cos(q(3)) - s5 * cos(q(1)) * cos(q(2)) * cos(q(3)) + s2 * s4 * s5 * cos(q(2)) + s3 * s4 * s5 * cos(q(1))) * s1 - c1 * c5;
  ow::Scalar b4 = (s2 * s5 * cos(q(2)) * cos(q(3)) - s2 * s3 * s4 * s5 + s3 * s5 * cos(q(1)) * cos(q(3)) + s4 * s5 * cos(q(1)) * cos(q(2))) * c1;
  ow::Scalar b5 = (c5 * s2 * cos(q(2)) * cos(q(3)) - c5 * s2 * s3 * s4 + c5 * s3 * cos(q(1)) * cos(q(3)) + c5 * s4 * cos(q(1)) * cos(q(2))) * c1;
  ow::Scalar b6 = (s2 * s3 * cos(q(3)) + s2 * s4 * cos(q(2)) + s3 * s4 * cos(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * c1;
  ow::Scalar b7 = (c5 * s2 * s3 * cos(q(3)) - c5 * cos(q(1)) * cos(q(2)) * cos(q(3)) + c5 * s2 * s4 * cos(q(2)) + c5 * s3 * s4 * cos(q(1))) * c1 - s1 * s5;
  ow::Scalar b8 = (s2 * s3 * s5 * cos(q(3)) - s5 * cos(q(1)) * cos(q(2)) * cos(q(3)) + s2 * s4 * s5 * cos(q(2)) + s3 * s4 * s5 * cos(q(1))) * c1 + c5 * s1;
  ow::Scalar b10 = (s2 * s3 * cos(q(3)) + s2 * s4 * cos(q(2)) + s3 * s4 * cos(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * s1;
  ow::Scalar b11 = (s1 * s2 * cos(q(2)) * cos(q(3)) - s1 * s2 * s3 * s4 + s1 * s3 * cos(q(1)) * cos(q(3)) + s1 * s4 * cos(q(1)) * cos(q(2))) * c5;
  ow::Scalar b12 = (s2 * s5 * cos(q(2)) * cos(q(3)) - s2 * s3 * s4 * s5 + s3 * s5 * cos(q(1)) * cos(q(3)) + s4 * s5 * cos(q(1)) * cos(q(2))) * s1;
  ow::Scalar b13 = (s2 * s3 * cos(q(3)) + s2 * s4 * cos(q(2)) + s3 * s4 * cos(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * s5;
  ow::Scalar b14 = (cos(q(2)) * cos(q(3)) - s3 * s4) * s2 + ((cos(q(1)) * cos(q(2))) * s4 + (cos(q(1)) * cos(q(3))) * s3);
  ow::Scalar b15 = (s2 * s3 * cos(q(3)) + s2 * s4 * cos(q(2)) + s3 * s4 * cos(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * c5;

  Jt5_0(0, 0) += (s2 * t(1) * cos(q(2)) * cos(q(3)) - s2 * s3 * s4 * t(1) + s3 * t(1) * cos(q(1)) * cos(q(3)) + s4 * t(1) * cos(q(1)) * cos(q(2))) * s1 + (b2 * t(0) - b3 * t(2));
  Jt5_0(0, 1) += b4 * t(2) - b5 * t(0) + b6 * t(1);
  Jt5_0(0, 2) += b4 * t(2) - b5 * t(0) + b6 * t(1);
  Jt5_0(0, 3) += b4 * t(2) - b5 * t(0) + b6 * t(1);
  Jt5_0(0, 4) += b8 * t(0) + b7 * t(2);
  Jt5_0(1, 0) += (s2 * s3 * s4 * t(1) - s2 * t(1) * cos(q(2)) * cos(q(3)) - s3 * t(1) * cos(q(1)) * cos(q(3)) - s4 * t(1) * cos(q(1)) * cos(q(2))) * c1 + (b8 * t(2) - b7 * t(0));
  Jt5_0(1, 1) += b10 * t(1) - b11 * t(0) + b12 * t(2);
  Jt5_0(1, 2) += b10 * t(1) - b11 * t(0) + b12 * t(2);
  Jt5_0(1, 3) += b10 * t(1) - b11 * t(0) + b12 * t(2);
  Jt5_0(1, 4) += b3 * t(0) + b2 * t(2);
  Jt5_0(2, 1) += b13 * t(2) - b14 * t(1) - b15 * t(0);
  Jt5_0(2, 2) += b13 * t(2) - b14 * t(1) - b15 * t(0);
  Jt5_0(2, 3) += b13 * t(2) - b14 * t(1) - b15 * t(0);
  Jt5_0(2, 4) += (s2 * s3 * s4 * t(2) - s2 * t(2) * cos(q(2)) * cos(q(3)) - s3 * t(2) * cos(q(1)) * cos(q(3)) - s4 * t(2) * cos(q(1)) * cos(q(2))) * c5 + ((-s5 * t(0) * cos(q(1)) * cos(q(2))) * s4 + (-s5 * t(0) * cos(q(1)) * cos(q(3))) * s3 + (-s5 * t(0) * cos(q(2)) * cos(q(3))) * s2 + (s5 * t(0)) * s2 * s3 * s4);
}

void UR10Model::matrix_Jt6_0(ow::MatrixDof &Jt6_0,
                            const ow::Vector3 &t,
                            const ow::VectorDof &q) const
{
  ow::Scalar s1 = sin(q(0));
  ow::Scalar s2 = sin(q(1));
  ow::Scalar s3 = sin(q(2));
  ow::Scalar s4 = sin(q(3));
  ow::Scalar s5 = sin(q(4));
  ow::Scalar s6 = sin(q(5));
  ow::Scalar c1 = cos(q(0));
  ow::Scalar c2 = cos(q(1));
  ow::Scalar c4 = cos(q(3));
  ow::Scalar c5 = cos(q(4));
  ow::Scalar c6 = cos(q(5));
  ow::Scalar s4s3 = s4 * s3;
  ow::Scalar b1 = (c5 * s4 * cos(q(2)) + c4 * c5 * s3) * c6 * s1 * s2 + (c2 * c5 * s3 * s4 - c2 * c4 * c5 * cos(q(2))) * c6 * s1 + (c1 * s5) * c6 + (c4 * cos(q(2)) - s4s3) * s1 * s2 * s6 + (c2 * s4 * cos(q(2)) + c2 * c4 * s3) * s1 * s6;
  ow::Scalar b2 = (c6 * s2 * s3 * s4 - c2 * c6 * s4 * cos(q(2)) - c4 * c6 * s2 * cos(q(2)) - c2 * c4 * c6 * s3 + c2 * c5 * s3 * s4 * s6 + c4 * c5 * s2 * s3 * s6 - c2 * c4 * c5 * s6 * cos(q(2)) + c5 * s2 * s4 * s6 * cos(q(2))) * s1 + (c1 * s5) * s6;
  ow::Scalar b4 = (c2 * c6 * s4s3 + c4 * c6 * s2 * s3 - c2 * c4 * c6 * cos(q(2)) + c6 * s2 * s4 * cos(q(2)) + c2 * c4 * c5 * s3 * s6 - c5 * s2 * s3 * s4 * s6 + c2 * c5 * s4 * s6 * cos(q(2)) + c4 * c5 * s2 * s6 * cos(q(2))) * c1;
  ow::Scalar b5 = (c2 * c4 * s3 * s5 - s2 * s3 * s4 * s5 + c2 * s4 * s5 * cos(q(2)) + c4 * s2 * s5 * cos(q(2))) * c1;
  ow::Scalar b6 = (c2 * c4 * s6 * cos(q(2)) - c2 * s3 * s4 * s6 - c4 * s2 * s3 * s6 - c5 * c6 * s2 * s4s3 - s2 * s4 * s6 * cos(q(2)) + c2 * c4 * c5 * c6 * s3 + c2 * c5 * c6 * s4 * cos(q(2)) + c4 * c5 * c6 * s2 * cos(q(2))) * c1;
  ow::Scalar b10 = (s4s3 - c4 * cos(q(2))) * c1 * c2 * c5 * c6 + (s4 * s6 * cos(q(2)) + c4 * s3 * s6) * c1 * c2 + (s2 * s4 * cos(q(2)) + c4 * s2 * s3) * c1 * c5 * c6 + (c4 * s2 * s6 * cos(q(2)) - s2 * s3 * s4 * s6) * c1 + (-s1 * s5) * c6;
  ow::Scalar b11 = (c4 * cos(q(2)) - s4s3) * c1 * c6 * s2 + (c2 * s4 * cos(q(2)) + c2 * c4 * s3) * c1 * c6 + (-c5 * s4 * cos(q(2)) - c4 * c5 * s3) * c1 * s2 * s6 + (c2 * c4 * c5 * cos(q(2)) - c2 * c5 * s4s3) * c1 * s6 + (s1 * s5) * s6;
  ow::Scalar b13 = (c2 * c6 * s4s3 + c4 * c6 * s2 * s3 - c5 * s2 * s6 * s4s3 - c2 * c4 * c6 * cos(q(2)) + c6 * s2 * s4 * cos(q(2)) + c2 * c4 * c5 * s3 * s6 + c2 * c5 * s4 * s6 * cos(q(2)) + c4 * c5 * s2 * s6 * cos(q(2))) * s1;
  ow::Scalar b14 = (c2 * c4 * s3 * s5 - s2 * s3 * s4 * s5 + c2 * s4 * s5 * cos(q(2)) + c4 * s2 * s5 * cos(q(2))) * s1;
  ow::Scalar b15 = (c2 * s6 * s4s3 + c4 * s2 * s3 * s6 - c2 * c4 * s6 * cos(q(2)) + s2 * s4 * s6 * cos(q(2)) - c2 * c4 * c5 * c6 * s3 + c5 * c6 * s2 * s3 * s4 - c2 * c5 * c6 * s4 * cos(q(2)) - c4 * c5 * c6 * s2 * cos(q(2))) * s1;
  ow::Scalar b19 = (s2 * s4 * cos(q(2)) + c2 * s3 * s4 + c4 * s2 * s3 - c2 * c4 * cos(q(2))) * s5;
  ow::Scalar b20 = (s4 * s6 * cos(q(2)) + c4 * s3 * s6 + c5 * c6 * s3 * s4 - c4 * c5 * c6 * cos(q(2))) * c2 + ((c5 * c6 * s2) * c4 * s3 + (s2 * cos(q(2))) * c4 * s6 + (-s2 * s4) * s3 * s6 + c5 * c6 * s2 * s4 * cos(q(2)));
  ow::Scalar b21 = (c6 * s4 * cos(q(2)) + c4 * c6 * s3 - c5 * s3 * s4 * s6 + c4 * c5 * s6 * cos(q(2))) * c2 + ((c4 * cos(q(2))) * c6 * s2 + (-c4 * c5 * s3 * s6) * s2 + (-c5 * s6 * cos(q(2))) * s2 * s4 + (-s3) * c6 * s2 * s4);

  Jt6_0(0, 0) += (c4 * cos(q(2)) - s4s3) * c2 * s1 * s5 * t(2) + (-s2 * s4 * cos(q(2)) - c4 * s2 * s3) * s1 * s5 * t(2) + (c1 * c5) * t(2) + (b1 * t(0) - b2 * t(1));
  Jt6_0(0, 1) += b4 * t(1) - b6 * t(0) + b5 * t(2);
  Jt6_0(0, 2) += b4 * t(1) - b6 * t(0) + b5 * t(2);
  Jt6_0(0, 3) += b4 * t(1) - b6 * t(0) + b5 * t(2);
  Jt6_0(0, 4) += (s4s3 * t(2) - c4 * t(2) * cos(q(2))) * c1 * c2 * c5 + (c6 * s4s3 * t(0) - s3 * s4 * s6 * t(1) - c4 * c6 * t(0) * cos(q(2)) + c4 * s6 * t(1) * cos(q(2))) * c1 * c2 * s5 + (c4 * s2 * s3 * t(2) + s2 * s4 * t(2) * cos(q(2))) * c1 * c5 + (c4 * c6 * s2 * s3 * t(0) - s2 * s4 * s6 * t(1) * cos(q(2)) - c4 * s2 * s3 * s6 * t(1) + c6 * s2 * s4 * t(0) * cos(q(2))) * c1 * s5 + (c6 * s1 * t(0) - s1 * s6 * t(1)) * c5 + (-s1 * t(2)) * s5;
  Jt6_0(0, 5) += b10 * t(1) - b11 * t(0);
  Jt6_0(1, 0) += (s4s3 - c4 * cos(q(2))) * c1 * c2 * s5 * t(2) + (s2 * s4 * cos(q(2)) + c4 * s2 * s3) * c1 * s5 * t(2) + (c5 * s1) * t(2) - (b10 * t(0) + b11 * t(1));
  Jt6_0(1, 1) += b13 * t(1) + b15 * t(0) + b14 * t(2);
  Jt6_0(1, 2) += b13 * t(1) + b15 * t(0) + b14 * t(2);
  Jt6_0(1, 3) += b13 * t(1) + b15 * t(0) + b14 * t(2);
  Jt6_0(1, 4) += (s4s3 * t(2) - c4 * t(2) * cos(q(2))) * c2 * c5 * s1 + (c6 * s4s3 * t(0) - s6 * s4s3 * t(1) - c4 * c6 * t(0) * cos(q(2)) + c4 * s6 * t(1) * cos(q(2))) * c2 * s1 * s5 + (c4 * s2 * s3 * t(2) + s2 * s4 * t(2) * cos(q(2))) * c5 * s1 + (c1 * s6 * t(1) - c1 * c6 * t(0)) * c5 + (c4 * c6 * s2 * s3 * t(0) - s2 * s4 * s6 * t(1) * cos(q(2)) - c4 * s2 * s3 * s6 * t(1) + c6 * s2 * s4 * t(0) * cos(q(2))) * s1 * s5 + (c1 * t(2)) * s5;
  Jt6_0(1, 5) += b1 * t(1) + b2 * t(0);
  Jt6_0(2, 1) += b19 * t(2) - b20 * t(0) - b21 * t(1);
  Jt6_0(2, 2) += b19 * t(2) - b20 * t(0) - b21 * t(1);
  Jt6_0(2, 3) += b19 * t(2) - b20 * t(0) - b21 * t(1);
  Jt6_0(2, 4) += (c6 * t(0) - s6 * t(1)) * s2 * s3 * s4 * s5 + (c5 * t(2)) * s2 * s3 * s4 + (c4 * s6 * t(1) * cos(q(2)) - c4 * c6 * t(0) * cos(q(2))) * s2 * s5 + (-c4 * c5 * t(2) * cos(q(2))) * s2 + (c2 * c4 * s6 * t(1) - c2 * c4 * c6 * t(0)) * s3 * s5 + (-c2 * c4 * c5 * t(2)) * s3 + (c2 * s6 * t(1) * cos(q(2)) - c2 * c6 * t(0) * cos(q(2))) * s4 * s5 + (-c2 * c5 * t(2) * cos(q(2))) * s4;
  Jt6_0(2, 5) += (-c5 * s4 * cos(q(2)) - c4 * c5 * s3) * c2 * c6 * t(1) + (c4 * t(0) * cos(q(2)) - s4s3 * t(0)) * c2 * c6 + (s4s3 - c4 * cos(q(2))) * c2 * s6 * t(1) + (-c4 * c5 * s3 * t(0) - c5 * s4 * t(0) * cos(q(2))) * c2 * s6 + (c5 * s2 * s3 * s4 - c4 * c5 * s2 * cos(q(2))) * c6 * t(1) + (-c4 * s2 * s3 * t(0) - s2 * s4 * t(0) * cos(q(2))) * c6 + (s2 * s4 * cos(q(2)) + c4 * s2 * s3) * s6 * t(1) + (c5 * s2 * s3 * s4 * t(0) - c4 * c5 * s2 * t(0) * cos(q(2))) * s6;
}

#endif