#ifndef MATRIX_M_H_
#define MATRIX_M_H_

#include <ur10_robot_model/model_ur10.h>

using namespace ur;

void UR10Model::matrix_M(ow::MatrixDof &M,
                        const ow::VectorDof &q) const
{
  ow::Scalar s2 = sin(q(1));
  ow::Scalar s4 = sin(q(3));
  ow::Scalar s5 = sin(q(4));
  ow::Scalar s6 = sin(q(5));
  ow::Scalar c3 = cos(q(2));
  ow::Scalar c4 = cos(q(3));
  ow::Scalar c5 = cos(q(4));
  ow::Scalar c6 = cos(q(5));
  ow::Scalar c2c2 = (cos(q(1))) * (cos(q(1)));
  ow::Scalar c3c3 = c3 * c3;
  ow::Scalar c4c4 = c4 * c4;
  ow::Scalar c5c5 = c5 * c5;
  ow::Scalar c3c3c2c2 = c3c3 * c2c2;
  ow::Scalar c6c6 = c6 * c6;
  ow::Scalar c2s2 = (cos(q(1))) * s2;
  ow::Scalar c3s3 = c3 * (sin(q(2)));
  ow::Scalar c5s5 = c5 * s5;
  ow::Scalar c5s5c6 = c5s5 * c6;
  ow::Scalar c5s5s6 = c5s5 * s6;
  ow::Scalar c6s6 = c6 * s6;
  ow::Scalar c3s3c2s2 = c3s3 * c2s2;
  ow::Scalar c4s5 = c4 * s5;
  ow::Scalar c6c5 = c6 * c5;
  ow::Scalar c5s6 = c5 * s6;
  ow::Scalar c6s6s5 = c6s6 * s5;
  ow::Scalar a37 = L3 * L3 * m4;
  ow::Scalar a38 = L3 * L3 * m5;
  ow::Scalar a39 = L3 * L3 * m6;
  ow::Scalar a42 = L5 * L5 * m6;
  ow::Scalar a43 = L9 * L9 * m3;
  ow::Scalar a45 = L11 * L11 * m5;
  ow::Scalar a46 = L2 * L2 * m3;
  ow::Scalar a47 = L2 * L2 * m4;
  ow::Scalar a48 = L2 * L2 * m5;
  ow::Scalar a49 = L2 * L2 * m6;
  ow::Scalar a50 = L8 * L8 * m2;
  ow::Scalar a51 = L12 * L12 * m6;
  ow::Scalar a52 = L4 * L12 * m6;
  ow::Scalar a53 = L3 * L5 * m6;
  ow::Scalar a54 = L3 * L11 * m5;
  ow::Scalar a55 = L3 * L12 * m6;
  ow::Scalar a56 = L2 * L3 * m4;
  ow::Scalar a57 = L2 * L3 * m5;
  ow::Scalar a58 = L2 * L3 * m6;
  ow::Scalar a59 = L2 * L9 * m3;
  ow::Scalar a60 = L5 * L12 * m6;
  ow::Scalar a63 = L2 * L12 * m6;
  ow::Scalar a64 = L2 * L4 * m5;
  ow::Scalar a65 = L2 * L4 * m6;
  ow::Scalar a66 = L2 * L10 * m4;
  ow::Scalar b24 = (c4 * s2 + s4 * cos(q(1))) * c3 + ((cos(q(1)) * sin(q(2))) * c4 + (-sin(q(2))) * s2 * s4);
  ow::Scalar b25 = (c3 * s4 * cos(q(1)) - s2 * s4 * sin(q(2)) + c4 * cos(q(1)) * sin(q(2)) + c3 * c4 * s2) * c5s5;
  ow::Scalar b26 = (c6 * s4 * cos(q(1)) + c4 * c6 * s2 + c5 * s2 * s4 * s6 - c4 * c5 * s6 * cos(q(1))) * c3 + ((c4 * cos(q(1)) * sin(q(2))) * c6 + (c4 * c5 * s6 * sin(q(2))) * s2 + (c5 * s6 * cos(q(1)) * sin(q(2))) * s4 + (-sin(q(2))) * c6 * s2 * s4) + 2 * ((c5c5 * s2 * s4 * sin(q(2)) - c3 * c5c5 * s4 * cos(q(1)) - c4 * c5c5 * cos(q(1)) * sin(q(2)) - c3 * c4 * c5c5 * s2) * c6);
  ow::Scalar b27 = (s2 * sin(q(2)) - c3 * cos(q(1))) * c4 + ((cos(q(1)) * sin(q(2))) * s4 + c3 * s2 * s4);
  ow::Scalar b28 = (c4 * s2 * sin(q(2)) + s4 * cos(q(1)) * sin(q(2)) + c3 * s2 * s4 - c3 * c4 * cos(q(1))) * c5;
  ow::Scalar b29 = (c3 * s4 * cos(q(1)) - s2 * s4 * sin(q(2)) + c4 * cos(q(1)) * sin(q(2)) + c3 * c4 * s2) * s5;
  ow::Scalar b30 = (c4 * s2 + s4 * cos(q(1))) * c3 + ((cos(q(1)) * sin(q(2))) * c4 + (-sin(q(2))) * s2 * s4) + 2 * ((s2 * s4 * sin(q(2)) - c3 * s4 * cos(q(1)) - c4 * cos(q(1)) * sin(q(2)) - c3 * c4 * s2) * c5c5);
  ow::Scalar b31 = c3 * cos(q(1)) - s2 * sin(q(2));
  ow::Scalar b32 = (c4 - c4 * c6c6) * c3 * c5s5 * s2 + (cos(q(1)) - c6c6 * cos(q(1))) * c3 * c5s5 * s4 + (c6s6 * s5) * c3 * s2 * s4 + (-c4 * c6s6 * s5 * cos(q(1))) * c3 + (c6c6 * sin(q(2)) - sin(q(2))) * c5s5 * s2 * s4 + (c4 * cos(q(1)) * sin(q(2)) - c4 * c6c6 * cos(q(1)) * sin(q(2))) * c5s5 + (c4 * c6s6 * s5 * sin(q(2))) * s2 + (c6s6 * s5 * cos(q(1)) * sin(q(2))) * s4;
  ow::Scalar b33 = (c3 * s2 * s4 * s5 - c3 * c4 * s5 * cos(q(1)) + c4 * s2 * s5 * sin(q(2)) + s4 * s5 * cos(q(1)) * sin(q(2))) * c6s6 + ((c6c6 * c5s5 * s2 * sin(q(2))) * s4 + (-c4 * c6c6 * c5s5 * s2) * c3 + (-c6c6 * c5s5 * cos(q(1))) * c3 * s4 - c4 * c6c6 * c5s5 * cos(q(1)) * sin(q(2)));
  ow::Scalar b34 = (s4 * s6 * sin(q(2)) - c3 * c4 * s6 + c4 * c5 * c6 * sin(q(2)) + c3 * c5 * c6 * s4) * s2 + ((c5 * c6 * cos(q(1)) * sin(q(2))) * s4 + (-c4 * c5 * c6 * cos(q(1))) * c3 + (-c4 * cos(q(1)) * sin(q(2))) * s6 + (-cos(q(1))) * c3 * s4 * s6) + 2 * ((c3 * c4 * s2 * s6 + c3 * s4 * s6 * cos(q(1)) - s2 * s4 * s6 * sin(q(2)) + c4 * s6 * cos(q(1)) * sin(q(2))) * c5c5);
  ow::Scalar b35 = cos(q(1)) * sin(q(2)) + c3 * s2;
  ow::Scalar b36 = (c3 * c4 * cos(q(1)) - s4 * cos(q(1)) * sin(q(2)) - c3 * s2 * s4 - c4 * s2 * sin(q(2))) * s5 + 2 * ((c6c6 * s2 * s5 * sin(q(2)) - c3 * c6c6 * s5 * cos(q(1)) + c5s5 * c6s6 * cos(q(1)) * sin(q(2)) + c3 * c5s5 * c6s6 * s2) * c4 + ((c6c6 * s2 * s4 * s5) * c3 + (s4 * cos(q(1))) * c3 * c5s5 * c6s6 + (-s2 * s4 * sin(q(2))) * c5s5 * c6s6 + c6c6 * s4 * s5 * cos(q(1)) * sin(q(2))));
  ow::Scalar b37 = (cos(q(1)) * sin(q(2)) + c3 * s2) * c5;
  ow::Scalar b38 = (c4 * s2 * sin(q(2)) + s4 * cos(q(1)) * sin(q(2)) + c3 * s2 * s4 - c3 * c4 * cos(q(1))) * s5;
  ow::Scalar b39 = (c3 * s4 * cos(q(1)) - s2 * s4 * sin(q(2)) + c4 * cos(q(1)) * sin(q(2)) + c3 * c4 * s2) * c5;
  ow::Scalar b40 = (c3 * c4 * s2 * s5 + c3 * s4 * s5 * cos(q(1)) - s2 * s4 * s5 * sin(q(2)) + c4 * s5 * cos(q(1)) * sin(q(2))) * c6;
  ow::Scalar b41 = (c3 * c4 * s2 * s6 + c3 * s4 * s6 * cos(q(1)) - s2 * s4 * s6 * sin(q(2)) + c4 * s6 * cos(q(1)) * sin(q(2))) * s5;
  ow::Scalar b42 = (c3 * s4 * cos(q(1)) - s2 * s4 * sin(q(2)) + c4 * cos(q(1)) * sin(q(2)) + c3 * c4 * s2) * c5 + 2 * ((c6s6 * s2 * s4 - c4 * c6s6 * cos(q(1)) - c6 * c6c5 * s4 * cos(q(1)) - c4 * c6 * c6c5 * s2) * c3 + ((cos(q(1)) * sin(q(2))) * c6s6 * s4 + (c6 * c6c5 * sin(q(2))) * s2 * s4 + (c4 * sin(q(2))) * c6s6 * s2 - c4 * c6 * c6c5 * cos(q(1)) * sin(q(2))));
  ow::Scalar b43 = (c3 * cos(q(1)) - s2 * sin(q(2))) * s5;
  ow::Scalar b44 = (c6 * c5s6) * c3 * c4 * s2 + (cos(q(1)) - c6c6 * cos(q(1))) * c3 * c4 + (c6c6 - 1) * c3 * s2 * s4 + (c6 * c5s6 * cos(q(1))) * c3 * s4 + (c6c6 * sin(q(2)) - sin(q(2))) * c4 * s2 + (c6 * c5s6 * cos(q(1)) * sin(q(2))) * c4 + (-c6 * c5s6 * sin(q(2))) * s2 * s4 + (c6c6 * cos(q(1)) * sin(q(2)) - cos(q(1)) * sin(q(2))) * s4;
  ow::Scalar b45 = (c6c6 * s2 * s4 - c4 * c6c6 * cos(q(1)) + c6 * c5s6 * s4 * cos(q(1)) + c4 * c6 * c5s6 * s2) * c3 + ((cos(q(1)) * sin(q(2))) * c6c6 * s4 + (c4 * sin(q(2))) * c6c6 * s2 + (-c6 * c5s6 * sin(q(2))) * s2 * s4 + c4 * c6 * c5s6 * cos(q(1)) * sin(q(2)));
  ow::Scalar b46 = (c4 * s6 * cos(q(1)) - s2 * s4 * s6 + c5 * c6 * s4 * cos(q(1)) + c4 * c5 * c6 * s2) * c3 + ((c5 * c6 * cos(q(1)) * sin(q(2))) * c4 + (-c5 * c6 * s4 * sin(q(2))) * s2 + (-s4 * cos(q(1)) * sin(q(2))) * s6 + (-sin(q(2))) * c4 * s2 * s6);
  ow::Scalar b47 = (c6 * s2 * s4 - c4 * c6 * cos(q(1)) + c4 * c5 * s2 * s6 + c5 * s4 * s6 * cos(q(1))) * c3 + ((cos(q(1)) * sin(q(2))) * c6 * s4 + (c4 * sin(q(2))) * c6 * s2 + (-c5 * s6 * sin(q(2))) * s2 * s4 + c4 * c5 * s6 * cos(q(1)) * sin(q(2)));
  ow::Scalar b48 = (1 - c5c5) * c6c6;
  ow::Scalar b49 = c5c5 - 1;
  ow::Scalar b50 = 2 * ((1 - c5c5) * c6s6);
  ow::Scalar b51 = (1 - c6c6) * c5c5 + (c6c6 - 1);
  ow::Scalar b54 = c3 * c4s5 - s4 * s5 * sin(q(2));
  ow::Scalar b55 = c3 * s4 + c4 * sin(q(2));
  ow::Scalar b56 = -s5 + 2 * c6c6 * s5;
  ow::Scalar b57 = (c3 * s4 + c4 * sin(q(2))) * c5;
  ow::Scalar d1 = I523 - a60;
  ow::Scalar d7 = (L5 * m6 + L11 * m5) * L2;
  ow::Scalar d9 = I511 - I522 - I633 + a51;
  ow::Scalar d10 = (L5 * m6 + L11 * m5) * L4 + I423;
  ow::Scalar d11 = (m5 + m6) * L3 * L4 + (L10 * m4) * L3 - I313;
  ow::Scalar d12 = I533 + a51;
  ow::Scalar d13 = I511 + a51;

  M(0, 0) = (I522 - I511 - I622 + I633 - a51 - I611 * c6c6 + I622 * c6c6) * c2c2 * c5c5 + (I222 - I211 + I311 - I322 - I411 + I422 - I522 + I533 + I611 - I633 - a37 - a38 - a39 - a42 - a43 - a45 + a46 + a47 + a48 + a49 + a50 + a51 - I611 * c6c6 + I622 * c6c6) * c2c2 + (I522 - I511 - I622 + I633 - a51 - I611 * c6c6 + I622 * c6c6) * c3c3 * c5c5 + (I311 - I322 - I411 + I422 - I522 + I533 + I611 - I633 - a37 - a38 - a39 - a42 - a43 - a45 + a51 - I611 * c6c6 + I622 * c6c6) * c3c3 + (I522 - I511 - I622 + I633 - a51 - I611 * c6c6 + I622 * c6c6) * c4c4 * c5c5 + (I422 - I411 - I522 + I533 + I611 - I633 - a42 - a45 + a51 - I611 * c6c6 + I622 * c6c6) * c4c4 + (I511 - I522 + I622 - I633 + a51 + I611 * c6c6 - I622 * c6c6) * c5c5 + (I133 + I211 + I322 + I411 + I522 + I633 + a37 + a38 + a39 + a42 + a43 + a45 + L4 * L4 * m5 + L4 * L4 * m6 + L10 * L10 * m4) + 2 * ((d7 * sin(q(2))) * c4 * c2c2 + (I522 * c2s2 - I511 * c3s3 - I511 * c2s2 + I522 * c3s3 - I622 * c2s2 - I622 * c3s3 + I633 * c2s2 + I633 * c3s3 - a51 * c2s2 - a51 * c3s3 - I611 * c6c6 * c2s2 - I611 * c6c6 * c3s3 + I622 * c6c6 * c2s2 + I622 * c6c6 * c3s3) * c4 * c5c5 * s4 + (I412 - I513 * c5 - I612 * c5 - I411 * c2s2 - I411 * c3s3 + I422 * c2s2 + I422 * c3s3 - I522 * c2s2 - I522 * c3s3 + I533 * c2s2 + I533 * c3s3 + I611 * c2s2 + I611 * c3s3 - I633 * c2s2 - I633 * c3s3 - a42 * c2s2 - a45 * c2s2 + a51 * c2s2 - a42 * c3s3 - a45 * c3s3 + a51 * c3s3 + d1 * s5 + I611 * c5 * c6s6 - I622 * c5 * c6s6 - I611 * c6c6 * c2s2 - I611 * c6c6 * c3s3 + I622 * c6c6 * c2s2 + I622 * c6c6 * c3s3 - I623 * c6 * s5 - I613 * s5 * s6) * c4 * s4 + (c3 * c2s2 * d7 - a54 * c2s2 - a53 * c3 * sin(q(2)) - a54 * c3 * sin(q(2)) - a53 * c2s2 + a63 * c2s2 * s5 * sin(q(2))) * c4 + (I511 * c3c3 + I511 * c4c4 - I522 * c3c3 - I522 * c4c4 + I622 * c3c3 + I622 * c4c4 - I633 * c3c3 - I633 * c4c4 + I612 * c6s6 + a51 * c3c3 + a51 * c4c4 + I611 * c3c3 * c6c6 + I611 * c4c4 * c6c6 - I622 * c3c3 * c6c6 - I622 * c4c4 * c6c6) * c2c2 * c5c5 + (c3 * d7 - a54 - a53 + a63 * s5 * sin(q(2))) * c2c2 * s4 + (I411 * c4c4 - I422 * c4c4 + I522 * c4c4 - I533 * c4c4 - I611 * c4c4 + I633 * c4c4 + I512 * c5s5 + I612 * c6s6 + a56 * c3 + a57 * c3 + a58 * c3 + a59 * c3 + a42 * c4c4 + a45 * c4c4 - a51 * c4c4 + a55 * c4s5 + I613 * c6 * c5s5 + I611 * c3c3 * c6c6 + I611 * c4c4 * c6c6 - I622 * c3c3 * c6c6 - I622 * c4c4 * c6c6 - I623 * c5s5 * s6 - a63 * c3 * c4s5) * c2c2 + (I522 * c3s3c2s2 - I511 * c3s3c2s2 - I612 * c6s6 - I622 * c3s3c2s2 + I633 * c3s3c2s2 - a51 * c3s3c2s2 + I511 * c3c3 * c4c4 - I522 * c3c3 * c4c4 + I622 * c3c3 * c4c4 - I633 * c3c3 * c4c4 + I612 * c3c3 * c6s6 + I612 * c4c4 * c6s6 - I611 * c6c6 * c3s3c2s2 + I622 * c6c6 * c3s3c2s2 + a51 * c3c3 * c4c4 + I611 * c3c3 * c4c4 * c6c6 - I622 * c3c3 * c4c4 * c6c6) * c5c5 + (a53 + a54 - a53 * c3c3 - a54 * c3c3 - c2s2 * d7 * sin(q(2)) - a55 * c2s2 * s5 - a55 * c3s3 * s5 + a63 * c3 * c2s2 * s5) * s4 + (I322 * c3c3c2c2 - I311 * c3c3c2c2 + I411 * c3c3c2c2 - I422 * c3c3c2c2 + I522 * c3c3c2c2 - I533 * c3c3c2c2 - I611 * c3c3c2c2 + I633 * c3c3c2c2 + I212 * c2s2 - I312 * c2s2 - I312 * c3s3 + I412 * c2s2 + I412 * c3s3 - I512 * c5s5 - I613 * c5s5c6 + I623 * c5s5s6 + I311 * c3s3c2s2 - I322 * c3s3c2s2 - I411 * c3s3c2s2 + I422 * c3s3c2s2 - I522 * c3s3c2s2 + I533 * c3s3c2s2 + I611 * c3s3c2s2 - I633 * c3s3c2s2 + a52 * c5 + a37 * c3c3c2c2 + a38 * c3c3c2c2 + a39 * c3c3c2c2 + a42 * c3c3c2c2 + a43 * c3c3c2c2 + a45 * c3c3c2c2 - a51 * c3c3c2c2 - a55 * c4s5 - a37 * c3s3c2s2 - a38 * c3s3c2s2 - a39 * c3s3c2s2 - a42 * c3s3c2s2 - a43 * c3s3c2s2 - a45 * c3s3c2s2 + a51 * c3s3c2s2 - a56 * c2s2 * sin(q(2)) - a57 * c2s2 * sin(q(2)) - a58 * c2s2 * sin(q(2)) - a59 * c2s2 * sin(q(2)) - I513 * c5 * c2s2 - I513 * c5 * c3s3 - I612 * c5 * c2s2 - I612 * c5 * c3s3 + I411 * c3c3 * c4c4 - I422 * c3c3 * c4c4 + I522 * c3c3 * c4c4 - I533 * c3c3 * c4c4 - I611 * c3c3 * c4c4 + I633 * c3c3 * c4c4 + I512 * c3c3 * c5s5 + I512 * c4c4 * c5s5 + I612 * c3c3 * c6s6 + I612 * c4c4 * c6s6 - I611 * c6c6 * c3s3c2s2 + I622 * c6c6 * c3s3c2s2 + a42 * c3c3 * c4c4 + a45 * c3c3 * c4c4 - a51 * c3c3 * c4c4 + a55 * c3c3 * c4s5 + c2s2 * d1 * s5 + c3s3 * d1 * s5 + I613 * c6 * c3c3 * c5s5 + I613 * c6 * c4c4 * c5s5 + I611 * c5 * c2s2 * c6s6 + I611 * c5 * c3s3 * c6s6 - I622 * c5 * c2s2 * c6s6 - I622 * c5 * c3s3 * c6s6 + I611 * c3c3 * c4c4 * c6c6 - I622 * c3c3 * c4c4 * c6c6 - I623 * c6 * c2s2 * s5 - I623 * c6 * c3s3 * s5 - I623 * c3c3 * c5s5 * s6 - I623 * c4c4 * c5s5 * s6 - I613 * c2s2 * s5 * s6 - I613 * c3s3 * s5 * s6) + 2 * ((I513 * c5 - I412 + I612 * c5 + I411 * c3s3 - I422 * c3s3 + I522 * c3s3 - I533 * c3s3 - I611 * c3s3 + I633 * c3s3 + a42 * c3s3 + a45 * c3s3 - a51 * c3s3 - d1 * s5 - I611 * c5 * c6s6 + I622 * c5 * c6s6 + I511 * c5c5 * c3s3 - I522 * c5c5 * c3s3 + I611 * c6c6 * c3s3 + I622 * c5c5 * c3s3 - I622 * c6c6 * c3s3 - I633 * c5c5 * c3s3 + I623 * c6 * s5 + a51 * c5c5 * c3s3 + I613 * s5 * s6 + I611 * c5c5 * c6c6 * c3s3 - I622 * c5c5 * c6c6 * c3s3) * c4 * c2c2 * s4 + (a53 * c3 * sin(q(2)) + a54 * c3 * sin(q(2))) * c4 * c2c2 + (I513 * c5 - I412 + I612 * c5 + I411 * c2s2 - I422 * c2s2 + I522 * c2s2 - I533 * c2s2 - I611 * c2s2 + I633 * c2s2 + a42 * c2s2 + a45 * c2s2 - a51 * c2s2 - d1 * s5 - I611 * c5 * c6s6 + I622 * c5 * c6s6 + I511 * c5c5 * c2s2 - I522 * c5c5 * c2s2 + I611 * c6c6 * c2s2 + I622 * c5c5 * c2s2 - I622 * c6c6 * c2s2 - I633 * c5c5 * c2s2 + I623 * c6 * s5 + a51 * c5c5 * c2s2 + I613 * s5 * s6 + I611 * c5c5 * c6c6 * c2s2 - I622 * c5c5 * c6c6 * c2s2) * c4 * c3c3 * s4 + (a53 * c2s2 + a54 * c2s2) * c4 * c3c3 + (I612 * c5 * c6c6 + I512 * c2s2 * c5s5 + I512 * c3s3 * c5s5 + I612 * c2s2 * c6s6 + I612 * c3s3 * c6s6 + I613 * c6 * c2s2 * c5s5 + I613 * c6 * c3s3 * c5s5 + I612 * c5c5 * c2s2 * c6s6 + I612 * c5c5 * c3s3 * c6s6 - I623 * c2s2 * c5s5 * s6 - I623 * c3s3 * c5s5 * s6) * c4 * s4 + (a55 * c3 * c2s2 * s5 * sin(q(2))) * c4 + (I422 * c4c4 - I411 * c4c4 - I522 * c4c4 + I533 * c4c4 + I611 * c4c4 - I633 * c4c4 - a42 * c4c4 - a45 * c4c4 + a51 * c4c4 - I511 * c4c4 * c5c5 + I522 * c4c4 * c5c5 - I611 * c4c4 * c6c6 - I622 * c4c4 * c5c5 + I622 * c4c4 * c6c6 + I633 * c4c4 * c5c5 - I612 * c5c5 * c6s6 - a51 * c4c4 * c5c5 - I611 * c4c4 * c5c5 * c6c6 + I622 * c4c4 * c5c5 * c6c6) * c2c2 * c3c3 + (a55 * c3s3 * s5) * c2c2 * s4 + (I312 * c3s3 - a55 * c4s5 * c3 * c3 - I412 * c3s3 + I513 * c5 * c3s3 + I612 * c5 * c3s3 - I512 * c4c4 * c5s5 - I612 * c4c4 * c6s6 - c3s3 * d1 * s5 - I613 * c6 * c4c4 * c5s5 - I611 * c5 * c3s3 * c6s6 + I622 * c5 * c3s3 * c6s6 - I612 * c4c4 * c5c5 * c6s6 + I623 * c6 * c3s3 * s5 + I623 * c4c4 * c5s5 * s6 + I613 * c3s3 * s5 * s6) * c2c2 + (a55 * c2s2 * s5) * c3c3 * s4 + (I312 * c2s2 - I412 * c2s2 + I513 * c5 * c2s2 + I612 * c5 * c2s2 - I512 * c4c4 * c5s5 - I612 * c4c4 * c6s6 - c2s2 * d1 * s5 - I613 * c6 * c4c4 * c5s5 - I611 * c5 * c2s2 * c6s6 + I622 * c5 * c2s2 * c6s6 - I612 * c4c4 * c5c5 * c6s6 + I623 * c6 * c2s2 * s5 + I623 * c4c4 * c5s5 * s6 + I613 * c2s2 * s5 * s6) * c3c3 + (a53 * c3c3c2c2 + a54 * c3c3c2c2 - a53 * c3s3c2s2 - a54 * c3s3c2s2) * s4 + (I512 * c5s5 * c3s3c2s2 - I412 * c4c4 * c3s3 - I512 * c3c3c2c2 * c5s5 - I612 * c3c3c2c2 * c6s6 - I412 * c4c4 * c2s2 + I612 * c6s6 * c3s3c2s2 + I513 * c5 * c4c4 * c2s2 + I513 * c5 * c4c4 * c3s3 + I612 * c5 * c4c4 * c2s2 + I612 * c5 * c4c4 * c3s3 + I612 * c5 * c6c6 * c2s2 + I612 * c5 * c6c6 * c3s3 - I613 * c6 * c3c3c2c2 * c5s5 + I613 * c6 * c5s5 * c3s3c2s2 + I511 * c4c4 * c5c5 * c3s3c2s2 - I522 * c4c4 * c5c5 * c3s3c2s2 + I611 * c4c4 * c6c6 * c3s3c2s2 + I622 * c4c4 * c5c5 * c3s3c2s2 - I622 * c4c4 * c6c6 * c3s3c2s2 - I633 * c4c4 * c5c5 * c3s3c2s2 + I411 * c4c4 * c2s2 * c3s3 - I422 * c4c4 * c2s2 * c3s3 + I522 * c4c4 * c2s2 * c3s3 - I533 * c4c4 * c2s2 * c3s3 - I611 * c4c4 * c2s2 * c3s3 + I633 * c4c4 * c2s2 * c3s3 + I612 * c5c5 * c6s6 * c3s3c2s2 + I623 * c3c3c2c2 * c5s5 * s6 - I623 * c5s5 * c3s3c2s2 * s6 + a51 * c4c4 * c5c5 * c3s3c2s2 + a42 * c4c4 * c2s2 * c3s3 + a45 * c4c4 * c2s2 * c3s3 - a51 * c4c4 * c2s2 * c3s3 - c4c4 * c2s2 * d1 * s5 - c4c4 * c3s3 * d1 * s5 - I611 * c5 * c4c4 * c2s2 * c6s6 - I611 * c5 * c4c4 * c3s3 * c6s6 + I622 * c5 * c4c4 * c2s2 * c6s6 + I622 * c5 * c4c4 * c3s3 * c6s6 + I611 * c4c4 * c5c5 * c6c6 * c3s3c2s2 - I622 * c4c4 * c5c5 * c6c6 * c3s3c2s2 + I623 * c6 * c4c4 * c2s2 * s5 + I623 * c6 * c4c4 * c3s3 * s5 + I613 * c4c4 * c2s2 * s5 * s6 + I613 * c4c4 * c3s3 * s5 * s6) + 2 * ((I513 * c5 * c2s2 - I412 * c2s2 + I612 * c5 * c2s2 - I512 * c2c2 * c5s5 - I612 * c2c2 * c6s6 - c2s2 * d1 * s5 - I613 * c6 * c2c2 * c5s5 - I611 * c5 * c2s2 * c6s6 + I622 * c5 * c2s2 * c6s6 - I612 * c2c2 * c5c5 * c6s6 + I623 * c6 * c2s2 * s5 + I623 * c2c2 * c5s5 * s6 + I613 * c2s2 * s5 * s6) * c4 * c3s3 * s4 + (I412 * c3c3c2c2 - I513 * c5 * c3c3c2c2 - I612 * c5 * c3c3c2c2 + c3c3c2c2 * d1 * s5 - I612 * c5 * c2c2 * c6c6 - I612 * c5 * c3c3 * c6c6 + I611 * c5 * c3c3c2c2 * c6s6 - I622 * c5 * c3c3c2c2 * c6s6 - I512 * c3c3 * c2s2 * c5s5 - I612 * c3c3 * c2s2 * c6s6 - I623 * c6 * c3c3c2c2 * s5 - I613 * c3c3c2c2 * s5 * s6 - I613 * c6 * c3c3 * c2s2 * c5s5 - I612 * c3c3 * c5c5 * c2s2 * c6s6 + I623 * c3c3 * c2s2 * c5s5 * s6) * c4 * s4 + (I412 * c2c2 - I513 * c5 * c2c2 - I612 * c5 * c2c2 - I612 * c5 * c6c6 - I512 * c2s2 * c5s5 - I612 * c2s2 * c6s6 + c2c2 * d1 * s5 + I611 * c5 * c2c2 * c6s6 - I622 * c5 * c2c2 * c6s6 - I613 * c6 * c2s2 * c5s5 - I623 * c6 * c2c2 * s5 + I623 * c2s2 * c5s5 * s6 - I613 * c2c2 * s5 * s6) * c4c4 * c3s3 + (I412 * c3c3 * c2s2 - I513 * c5 * c3c3 * c2s2 - I612 * c5 * c3c3 * c2s2 - I612 * c5 * c6c6 * c2s2 + I512 * c2c2 * c3c3 * c5s5 + I612 * c2c2 * c3c3 * c6s6 - I612 * c5c5 * c6s6 * c3s3c2s2 + c3c3 * c2s2 * d1 * s5 + I613 * c6 * c2c2 * c3c3 * c5s5 + I611 * c5 * c3c3 * c2s2 * c6s6 - I622 * c5 * c3c3 * c2s2 * c6s6 + I612 * c2c2 * c3c3 * c5c5 * c6s6 - I623 * c6 * c3c3 * c2s2 * s5 - I623 * c2c2 * c3c3 * c5s5 * s6 - I613 * c3c3 * c2s2 * s5 * s6) * c4c4 + (-I612 * c5 * c2c2 * c6c6) * c3s3 - I612 * c5 * c3c3 * c6c6 * c2s2 + 2 * ((c4c4 * c3s3 + c4 * c3c3 * s4) * I612 * c5 * c2c2 * c6c6 + (c3c3 * c4c4 * c2s2 - c4 * c2s2 * c3s3 * s4) * I612 * c5 * c6c6))));
  M(0, 1) = (I213 - a64 - a65 - a66 - a63 * c5) * s2 + (I323 * b31 + I413 * b24 - I512 * b30 + I513 * b38 - I613 * b26 - I611 * b33 - I612 * b36 + I622 * b32 - I623 * b34 + a52 * b29 - a55 * b37 + b28 * d1 + b25 * d9 - b27 * d10 - b35 * d11 + I223 * cos(q(1)));
  M(0, 2) = I323 * b31 + I413 * b24 - I512 * b30 + I513 * b38 - I613 * b26 - I611 * b33 - I612 * b36 + I622 * b32 - I623 * b34 + a52 * b29 - a55 * b37 + b28 * d1 + b25 * d9 - b27 * d10 - b35 * d11;
  M(0, 3) = I413 * b24 - I512 * b30 + I513 * b38 - I613 * b26 - I611 * b33 - I612 * b36 + I622 * b32 - I623 * b34 + a52 * b29 + b28 * d1 + b25 * d9 - b27 * d10;
  M(0, 4) = I513 * b39 + I612 * b42 + I613 * b41 - I611 * b44 + I623 * b40 + I622 * b45 + a52 * b28 + a55 * b43 - b29 * d1 + b27 * d12 + a63 * s5 * cos(q(1));
  M(0, 5) = I613 * b46 - I633 * b29 - I623 * b47;
  M(1, 0) = (I213 - a64 - a65 - a66 - a63 * c5) * s2 + (I323 * b31 + I413 * b24 - I512 * b30 + I513 * b38 - I613 * b26 - I611 * b33 - I612 * b36 + I622 * b32 - I623 * b34 + a52 * b29 - a55 * b37 + b28 * d1 + b25 * d9 - b27 * d10 - b35 * d11 + I223 * cos(q(1)));
  M(1, 1) = (I522 + I633) * c5c5 + (I233 + I333 + I433 + a37 + a38 + a39 + a42 + a43 + a45 + a46 + a47 + a48 + a49 + a50 + I611 * b48 - I612 * b50 - I622 * b51 - b49 * d13) + 2 * ((-a63) * c3 * c4s5 + c3 * d7 * s4 + (a56 + a57 + a58 + a59) * c3 + (-a55) * c4s5 + (c4 * sin(q(2))) * d7 + (a53 + a54 + a63 * s5 * sin(q(2))) * s4 + (I512 * c5s5 + I613 * c5s5c6 - I623 * c5s5s6));
  M(1, 2) = (a56 + a57 + a58 + a59) * c3 + (I522 + I633) * c5c5 + (I333 + I433 + a37 + a38 + a39 + a42 + a43 + a45 + I611 * b48 - I612 * b50 - I622 * b51 - a63 * b54 - b49 * d13 + b55 * d7) + 2 * ((a53 + a54) * s4 + (I512 * c5s5 + I613 * c5s5c6 - I623 * c5s5s6 - a55 * c4s5));
  M(1, 3) = (I522 + I633) * c5c5 + (a53 + a54) * s4 + (I433 + a42 + a45 + I611 * b48 - I612 * b50 - I622 * b51 - a63 * b54 - a55 * c4s5 - b49 * d13 + b55 * d7) + 2 * (I512 * c5s5 + I613 * c5s5c6 - I623 * c5s5s6);
  M(1, 4) = (I523 - a60 - a55 * s4) * c5 + (I622 - I611) * c6s6s5 + (I513 * s5 - I623 * c6c5 - I613 * c5s6 - I612 * b56 - a63 * b57);
  M(1, 5) = (I613 * c6 - I623 * s6) * s5 + I633 * c5;
  M(2, 0) = I323 * b31 + I413 * b24 - I512 * b30 + I513 * b38 - I613 * b26 - I611 * b33 - I612 * b36 + I622 * b32 - I623 * b34 + a52 * b29 - a55 * b37 + b28 * d1 + b25 * d9 - b27 * d10 - b35 * d11;
  M(2, 1) = (a56 + a57 + a58 + a59) * c3 + (I522 + I633) * c5c5 + (I333 + I433 + a37 + a38 + a39 + a42 + a43 + a45 + I611 * b48 - I612 * b50 - I622 * b51 - a63 * b54 - b49 * d13 + b55 * d7) + 2 * ((a53 + a54) * s4 + (I512 * c5s5 + I613 * c5s5c6 - I623 * c5s5s6 - a55 * c4s5));
  M(2, 2) = (I522 + I633) * c5c5 + (I333 + I433 + a37 + a38 + a39 + a42 + a43 + a45 + I611 * b48 - I612 * b50 - I622 * b51 - b49 * d13) + 2 * ((a53 + a54) * s4 + (I512 * c5s5 + I613 * c5s5c6 - I623 * c5s5s6 - a55 * c4s5));
  M(2, 3) = (I522 + I633) * c5c5 + (a53 + a54) * s4 + (I433 + a42 + a45 + I611 * b48 - I612 * b50 - I622 * b51 - a55 * c4s5 - b49 * d13) + 2 * (I512 * c5s5 + I613 * c5s5c6 - I623 * c5s5s6);
  M(2, 4) = (I523 - a60 - a55 * s4) * c5 + (I622 - I611) * c6s6s5 + (I513 * s5 - I623 * c6c5 - I613 * c5s6 - I612 * b56);
  M(2, 5) = (I613 * c6 - I623 * s6) * s5 + I633 * c5;
  M(3, 0) = I413 * b24 - I512 * b30 + I513 * b38 - I613 * b26 - I611 * b33 - I612 * b36 + I622 * b32 - I623 * b34 + a52 * b29 + b28 * d1 + b25 * d9 - b27 * d10;
  M(3, 1) = (I522 + I633) * c5c5 + (a53 + a54) * s4 + (I433 + a42 + a45 + I611 * b48 - I612 * b50 - I622 * b51 - a63 * b54 - a55 * c4s5 - b49 * d13 + b55 * d7) + 2 * (I512 * c5s5 + I613 * c5s5c6 - I623 * c5s5s6);
  M(3, 2) = (I522 + I633) * c5c5 + (a53 + a54) * s4 + (I433 + a42 + a45 + I611 * b48 - I612 * b50 - I622 * b51 - a55 * c4s5 - b49 * d13) + 2 * (I512 * c5s5 + I613 * c5s5c6 - I623 * c5s5s6);
  M(3, 3) = (I522 + I633) * c5c5 + (I433 + a42 + a45 + I611 * b48 - I612 * b50 - I622 * b51 - b49 * d13) + 2 * (I512 * c5s5 + I613 * c5s5c6 - I623 * c5s5s6);
  M(3, 4) = (I523 - a60) * c5 + (I622 - I611) * c6s6s5 + (I513 * s5 - I623 * c6c5 - I613 * c5s6 - I612 * b56);
  M(3, 5) = (I613 * c6 - I623 * s6) * s5 + I633 * c5;
  M(4, 0) = I513 * b39 + I612 * b42 + I613 * b41 - I611 * b44 + I623 * b40 + I622 * b45 + a52 * b28 + a55 * b43 - b29 * d1 + b27 * d12 + a63 * s5 * cos(q(1));
  M(4, 1) = (I523 - a60 - a55 * s4) * c5 + (I622 - I611) * c6s6s5 + (I513 * s5 - I623 * c6c5 - I613 * c5s6 - I612 * b56 - a63 * b57);
  M(4, 2) = (I523 - a60 - a55 * s4) * c5 + (I622 - I611) * c6s6s5 + (I513 * s5 - I623 * c6c5 - I613 * c5s6 - I612 * b56);
  M(4, 3) = (I523 - a60) * c5 + (I622 - I611) * c6s6s5 + (I513 * s5 - I623 * c6c5 - I613 * c5s6 - I612 * b56);
  M(4, 4) = (1 - c6c6) * I611 + (I533 + a51 + I622 * c6c6) + 2 * I612 * c6s6;
  M(4, 5) = -I623 * c6 - I613 * s6;
  M(5, 0) = I613 * b46 - I633 * b29 - I623 * b47;
  M(5, 1) = (I613 * c6 - I623 * s6) * s5 + I633 * c5;
  M(5, 2) = (I613 * c6 - I623 * s6) * s5 + I633 * c5;
  M(5, 3) = (I613 * c6 - I623 * s6) * s5 + I633 * c5;
  M(5, 4) = -I623 * c6 - I613 * s6;
  M(5, 5) = I633;
}

#endif