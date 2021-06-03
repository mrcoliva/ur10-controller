#ifndef TRANSFORMATIONS_H_
#define TRANSFORMATIONS_H_

#include <ur10_robot_model/model_ur10.h>

using namespace ur;

void UR10Model::matrix_T1_0(ow::HomogeneousTransformation &T1_0,
                            const ow::VectorDof &q) const
{

  T1_0(0, 0) = cos(q(0));
  T1_0(0, 1) = 0;
  T1_0(0, 2) = sin(q(0));
  T1_0(0, 3) = 0;
  T1_0(1, 0) = sin(q(0));
  T1_0(1, 1) = 0;
  T1_0(1, 2) = -(cos(q(0)));
  T1_0(1, 3) = 0;
  T1_0(2, 0) = 0;
  T1_0(2, 1) = 1;
  T1_0(2, 2) = 0;
  T1_0(2, 3) = L1;
  T1_0(3, 0) = 0;
  T1_0(3, 1) = 0;
  T1_0(3, 2) = 0;
  T1_0(3, 3) = 1;
}

void UR10Model::matrix_T2_0(ow::HomogeneousTransformation &T2_0,
                            const ow::VectorDof &q) const
{

  T2_0(0, 0) = (cos(q(1))) * (cos(q(0)));
  T2_0(0, 1) = -((cos(q(0))) * (sin(q(1))));
  T2_0(0, 2) = sin(q(0));
  T2_0(0, 3) = L2 * ((cos(q(1))) * (cos(q(0))));
  T2_0(1, 0) = (cos(q(1))) * (sin(q(0)));
  T2_0(1, 1) = -((sin(q(1))) * (sin(q(0))));
  T2_0(1, 2) = -(cos(q(0)));
  T2_0(1, 3) = L2 * ((cos(q(1))) * (sin(q(0))));
  T2_0(2, 0) = sin(q(1));
  T2_0(2, 1) = cos(q(1));
  T2_0(2, 2) = 0;
  T2_0(2, 3) = L1 + L2 * sin(q(1));
  T2_0(3, 0) = 0;
  T2_0(3, 1) = 0;
  T2_0(3, 2) = 0;
  T2_0(3, 3) = 1;
}

void UR10Model::matrix_T3_0(ow::HomogeneousTransformation &T3_0,
                            const ow::VectorDof &q) const
{
  ow::Scalar s1 = sin(q(0));
  ow::Scalar s2 = sin(q(1));

  T3_0(0, 0) = cos(q(0)) * cos(q(1)) * cos(q(2)) - s2 * cos(q(0)) * sin(q(2));
  T3_0(0, 1) = -s2 * cos(q(0)) * cos(q(2)) - cos(q(0)) * cos(q(1)) * sin(q(2));
  T3_0(0, 2) = s1;
  T3_0(0, 3) = (cos(q(0)) * cos(q(1)) * cos(q(2)) - s2 * cos(q(0)) * sin(q(2))) * L3 + L2 * cos(q(0)) * cos(q(1));
  T3_0(1, 0) = (cos(q(1)) * cos(q(2)) - s2 * sin(q(2))) * s1;
  T3_0(1, 1) = (-cos(q(1)) * sin(q(2)) - s2 * cos(q(2))) * s1;
  T3_0(1, 2) = -(cos(q(0)));
  T3_0(1, 3) = (cos(q(1)) * cos(q(2)) - s2 * sin(q(2))) * L3 * s1 + (L2 * cos(q(1))) * s1;
  T3_0(2, 0) = cos(q(1)) * sin(q(2)) + s2 * cos(q(2));
  T3_0(2, 1) = cos(q(1)) * cos(q(2)) - s2 * sin(q(2));
  T3_0(2, 2) = 0;
  T3_0(2, 3) = (L2 + L3 * cos(q(2))) * s2 + (L1 + (cos(q(1)) * sin(q(2))) * L3);
  T3_0(3, 0) = 0;
  T3_0(3, 1) = 0;
  T3_0(3, 2) = 0;
  T3_0(3, 3) = 1;
}

void UR10Model::matrix_T4_0(ow::HomogeneousTransformation &T4_0,
                            const ow::VectorDof &q) const
{
  ow::Scalar s1 = sin(q(0));
  ow::Scalar s2 = sin(q(1));
  ow::Scalar s3 = sin(q(2));
  ow::Scalar c2s1 = (cos(q(1))) * s1;

  T4_0(0, 0) = (-s2 * cos(q(0)) * cos(q(3)) - cos(q(0)) * cos(q(1)) * sin(q(3))) * s3 + ((-cos(q(0)) * cos(q(2)) * sin(q(3))) * s2 + cos(q(0)) * cos(q(1)) * cos(q(2)) * cos(q(3)));
  T4_0(0, 1) = s1;
  T4_0(0, 2) = (cos(q(0)) * cos(q(2)) * cos(q(3)) - s3 * cos(q(0)) * sin(q(3))) * s2 + ((cos(q(0)) * cos(q(1)) * cos(q(3))) * s3 + cos(q(0)) * cos(q(1)) * cos(q(2)) * sin(q(3)));
  T4_0(0, 3) = (cos(q(0)) * cos(q(1)) * cos(q(2)) - s2 * s3 * cos(q(0))) * L3 + (L4 * s1 + L2 * cos(q(0)) * cos(q(1)));
  T4_0(1, 0) = (cos(q(2)) * cos(q(3)) - s3 * sin(q(3))) * c2s1 + ((-cos(q(2)) * sin(q(3))) * s1 * s2 + (-cos(q(3))) * s1 * s2 * s3);
  T4_0(1, 1) = -(cos(q(0)));
  T4_0(1, 2) = (cos(q(2)) * sin(q(3)) + s3 * cos(q(3))) * c2s1 + ((cos(q(2)) * cos(q(3))) * s1 * s2 + (-sin(q(3))) * s1 * s2 * s3);
  T4_0(1, 3) = (L2 + L3 * cos(q(2))) * c2s1 + ((-s1 * s2 * s3) * L3 - L4 * cos(q(0)));
  T4_0(2, 0) = (cos(q(2)) * cos(q(3)) - s3 * sin(q(3))) * s2 + ((cos(q(1)) * cos(q(3))) * s3 + cos(q(1)) * cos(q(2)) * sin(q(3)));
  T4_0(2, 1) = 0;
  T4_0(2, 2) = (cos(q(2)) * sin(q(3)) + s3 * cos(q(3))) * s2 + ((cos(q(1)) * sin(q(3))) * s3 - cos(q(1)) * cos(q(2)) * cos(q(3)));
  T4_0(2, 3) = (L2 + L3 * cos(q(2))) * s2 + (L1 + (s3 * cos(q(1))) * L3);
  T4_0(3, 0) = 0;
  T4_0(3, 1) = 0;
  T4_0(3, 2) = 0;
  T4_0(3, 3) = 1;
}

void UR10Model::matrix_T5_0(ow::HomogeneousTransformation &T5_0,
                            const ow::VectorDof &q) const
{
  ow::Scalar s1 = sin(q(0));
  ow::Scalar s2 = sin(q(1));
  ow::Scalar s3 = sin(q(2));
  ow::Scalar s5 = sin(q(4));
  ow::Scalar c3 = cos(q(2));
  ow::Scalar c2s1 = (cos(q(1))) * s1;

  T5_0(0, 0) = (cos(q(0)) * cos(q(1)) * cos(q(3)) * cos(q(4)) - s2 * cos(q(0)) * cos(q(4)) * sin(q(3))) * c3 + (s1 * s5 + (-cos(q(0)) * cos(q(1)) * cos(q(4)) * sin(q(3))) * s3 + (-cos(q(0)) * cos(q(3)) * cos(q(4))) * s2 * s3);
  T5_0(0, 1) = (s3 * cos(q(0)) * sin(q(3)) - c3 * cos(q(0)) * cos(q(3))) * s2 + ((-cos(q(0)) * cos(q(1)) * sin(q(3))) * c3 + (-cos(q(0)) * cos(q(1)) * cos(q(3))) * s3);
  T5_0(0, 2) = (s3 * cos(q(0)) * cos(q(1)) * sin(q(3)) - c3 * cos(q(0)) * cos(q(1)) * cos(q(3)) + c3 * s2 * cos(q(0)) * sin(q(3)) + s2 * s3 * cos(q(0)) * cos(q(3))) * s5 + s1 * cos(q(4));
  T5_0(0, 3) = (c3 * cos(q(0)) * cos(q(1)) * sin(q(3)) + s3 * cos(q(0)) * cos(q(1)) * cos(q(3)) + c3 * s2 * cos(q(0)) * cos(q(3)) - s2 * s3 * cos(q(0)) * sin(q(3))) * L5 + ((L3 * cos(q(0)) * cos(q(1))) * c3 + L4 * s1 + L2 * cos(q(0)) * cos(q(1)) + (-L3 * cos(q(0))) * s2 * s3);
  T5_0(1, 0) = (c3 * cos(q(1)) * cos(q(3)) * cos(q(4)) - c3 * s2 * cos(q(4)) * sin(q(3)) - s2 * s3 * cos(q(3)) * cos(q(4))) * s1 + ((-c2s1 * cos(q(4)) * sin(q(3))) * s3 - s5 * cos(q(0)));
  T5_0(1, 1) = (s2 * s3 * sin(q(3)) - c3 * s2 * cos(q(3)) - c3 * cos(q(1)) * sin(q(3))) * s1 + (-c2s1 * cos(q(3))) * s3;
  T5_0(1, 2) = (s2 * sin(q(3)) - cos(q(1)) * cos(q(3))) * c3 * s1 * s5 + (s2 * cos(q(3))) * s1 * s3 * s5 + (c2s1 * sin(q(3))) * s3 * s5 - cos(q(0)) * cos(q(4));
  T5_0(1, 3) = (L3 * c3 * cos(q(1)) - L3 * s2 * s3 + L5 * c3 * s2 * cos(q(3)) - L5 * s2 * s3 * sin(q(3)) + L5 * c3 * cos(q(1)) * sin(q(3))) * s1 + (L2 * c2s1 - L4 * cos(q(0)) + (c2s1 * cos(q(3))) * L5 * s3);
  T5_0(2, 0) = (s2 * cos(q(3)) * cos(q(4)) + cos(q(1)) * cos(q(4)) * sin(q(3))) * c3 + ((cos(q(1)) * cos(q(3)) * cos(q(4))) * s3 + (-cos(q(4)) * sin(q(3))) * s2 * s3);
  T5_0(2, 1) = (cos(q(1)) * cos(q(3)) - s2 * sin(q(3))) * c3 + ((-cos(q(1)) * sin(q(3))) * s3 + (-cos(q(3))) * s2 * s3);
  T5_0(2, 2) = (s2 * s3 * sin(q(3)) - c3 * s2 * cos(q(3)) - c3 * cos(q(1)) * sin(q(3)) - s3 * cos(q(1)) * cos(q(3))) * s5;
  T5_0(2, 3) = (L2 + L3 * c3 + L5 * c3 * sin(q(3)) + L5 * s3 * cos(q(3))) * s2 + (L1 + (L3 * cos(q(1))) * s3 + (cos(q(1)) * sin(q(3))) * L5 * s3 + (-cos(q(1)) * cos(q(3))) * L5 * c3);
  T5_0(3, 0) = 0;
  T5_0(3, 1) = 0;
  T5_0(3, 2) = 0;
  T5_0(3, 3) = 1;
}

void UR10Model::matrix_T6_0(ow::HomogeneousTransformation &T6_0,
                            const ow::VectorDof &q) const
{
  ow::Scalar s1 = sin(q(0));
  ow::Scalar s2 = sin(q(1));
  ow::Scalar s3 = sin(q(2));
  ow::Scalar s4 = sin(q(3));
  ow::Scalar s5 = sin(q(4));
  ow::Scalar s6 = sin(q(5));
  ow::Scalar c1 = cos(q(0));
  ow::Scalar c4 = cos(q(3));
  ow::Scalar c3s2 = (cos(q(2))) * s2;
  ow::Scalar c2s3 = (cos(q(1))) * s3;
  ow::Scalar c2s1 = (cos(q(1))) * s1;
  ow::Scalar c4s1 = c4 * s1;

  T6_0(0, 0) = (s2 * s3 * s4 * s6 - c4 * s2 * s6 * cos(q(2)) - c4 * s3 * s6 * cos(q(1)) - s4 * s6 * cos(q(1)) * cos(q(2)) + c4 * cos(q(1)) * cos(q(2)) * cos(q(4)) * cos(q(5)) - c4 * s2 * s3 * cos(q(4)) * cos(q(5)) - s2 * s4 * cos(q(2)) * cos(q(4)) * cos(q(5)) - s3 * s4 * cos(q(1)) * cos(q(4)) * cos(q(5))) * c1 + s1 * s5 * cos(q(5));
  T6_0(0, 1) = (s2 * s3 * s4 * cos(q(5)) - s4 * cos(q(1)) * cos(q(2)) * cos(q(5)) - c4 * c3s2 * cos(q(5)) - c4 * s3 * cos(q(1)) * cos(q(5)) + s2 * s4 * s6 * cos(q(2)) * cos(q(4)) + s3 * s4 * s6 * cos(q(1)) * cos(q(4)) - c4 * s6 * cos(q(1)) * cos(q(2)) * cos(q(4)) + c4 * s2 * s3 * s6 * cos(q(4))) * c1 + (-s1 * s5) * s6;
  T6_0(0, 2) = (c4 * s2 * s3 * s5 + s2 * s4 * s5 * cos(q(2)) + s3 * s4 * s5 * cos(q(1)) - c4 * s5 * cos(q(1)) * cos(q(2))) * c1 + s1 * cos(q(4));
  T6_0(0, 3) = (c2s3 + c3s2) * L5 * c1 * c4 + (s4 * cos(q(1)) * cos(q(2)) - s2 * s3 * s4) * L5 * c1 + (s2 * s3 * s5 - s5 * cos(q(1)) * cos(q(2))) * L6 * c1 * c4 + (s2 * s4 * s5 * cos(q(2)) + s3 * s4 * s5 * cos(q(1))) * L6 * c1 + (s1 * cos(q(4))) * L6 + (L2 * cos(q(1)) + L3 * cos(q(1)) * cos(q(2)) - L3 * s2 * s3) * c1 + L4 * s1;
  T6_0(1, 0) = (-c2s3 - c3s2) * c4 * s1 * s6 + (-s2 * s3 * cos(q(4)) * cos(q(5))) * c4 * s1 + (c2s1 * cos(q(2)) * cos(q(4)) * cos(q(5))) * c4 + (s2 * s3) * s1 * s4 * s6 + (-c3s2 * cos(q(4)) * cos(q(5))) * s1 * s4 + (-c2s1 * cos(q(2))) * s4 * s6 + (-c2s1 * s3 * cos(q(4)) * cos(q(5))) * s4 - c1 * s5 * cos(q(5));
  T6_0(1, 1) = (c1 * s5 + c2s1 * s3 * s4 * cos(q(4)) + c3s2 * s1 * s4 * cos(q(4)) - c4 * c2s1 * cos(q(2)) * cos(q(4)) + c4 * s1 * s2 * s3 * cos(q(4))) * s6 + ((-c4s1 * cos(q(1)) * cos(q(5))) * s3 + (s1 * s2 * cos(q(5))) * s3 * s4 - c3s2 * c4s1 * cos(q(5)) + (-cos(q(2)) * cos(q(5))) * c2s1 * s4);
  T6_0(1, 2) = (c2s1 * s3 * s4 + c3s2 * s1 * s4 + c4s1 * s2 * s3 - c4 * c2s1 * cos(q(2))) * s5 - c1 * cos(q(4));
  T6_0(1, 3) = (s4 * cos(q(2))) * L5 * c2s1 + (-s1 * s2 * s4) * L5 * s3 + (c2s3 * c4s1 + c3s2 * c4s1) * L5 + (s4 * s5) * L6 * c2s1 * s3 + (-c4 * s5 * cos(q(2))) * L6 * c2s1 + (c4s1 * s2 * s5) * L6 * s3 + (c3s2 * s1 * s4 * s5 - c1 * cos(q(4))) * L6 + (L2 + L3 * cos(q(2))) * c2s1 + (-L3 * s1 * s2) * s3 - L4 * c1;
  T6_0(2, 0) = (-s2) * c4 * s3 * s6 + (cos(q(1)) * cos(q(2))) * c4 * s6 + (c2s3 * cos(q(4)) * cos(q(5)) + c3s2 * cos(q(4)) * cos(q(5))) * c4 + (-cos(q(1))) * s3 * s4 * s6 + (-s2 * cos(q(4)) * cos(q(5))) * s3 * s4 + (-c3s2) * s4 * s6 + (cos(q(1)) * cos(q(2)) * cos(q(4)) * cos(q(5))) * s4;
  T6_0(2, 1) = (-s2 * cos(q(5))) * c4 * s3 + (-c2s3 * cos(q(4)) - c3s2 * cos(q(4))) * c4 * s6 + (cos(q(1)) * cos(q(2)) * cos(q(5))) * c4 + (s2 * cos(q(4))) * s3 * s4 * s6 + (-cos(q(1)) * cos(q(5))) * s3 * s4 + (-cos(q(1)) * cos(q(2)) * cos(q(4))) * s4 * s6 + (-c3s2 * cos(q(5))) * s4;
  T6_0(2, 2) = (-c2s3 - c3s2) * c4 * s5 + (s2 * s3 - cos(q(1)) * cos(q(2))) * s4 * s5;
  T6_0(2, 3) = (s2 * s3 - cos(q(1)) * cos(q(2))) * L5 * c4 + (c2s3 + c3s2) * L5 * s4 + (-c2s3 * s5 - c3s2 * s5) * L6 * c4 + (s2 * s3 * s5 - s5 * cos(q(1)) * cos(q(2))) * L6 * s4 + (L1 + L3 * c2s3 + L3 * c3s2 + L2 * s2);
  T6_0(3, 0) = 0;
  T6_0(3, 1) = 0;
  T6_0(3, 2) = 0;
  T6_0(3, 3) = 1;
}

#endif