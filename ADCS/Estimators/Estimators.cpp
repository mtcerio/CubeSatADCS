/***************************************************************************//**
 * @file quest.cpp
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2019 </b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/
 
#include "Estimators.h"
#include "mbed.h"
Serial pc2(USBTX, USBRX, 115200);
void printMat2(SimpleMatrix::Matrix a);

using namespace SimpleMatrix;

void Estimators::QUEST(float quat[4], int N, float **s_eci, float **s_body, float *omega, float tolerance = 1e-5){
    /* Variable to store the solution */
    float lambda;       // The estimated eigen value of the problem
    Vector x;           // Quaternion's vector (of the eigen vector)
    float gamma;        // Quaternion's rotation (of the eigen vector)
    
    /* Internal computation variables  */
    float lambda0 = 0;  // A reasonable initial value for lambda
    Vector s_a[N];      // ECI frame vector array
    Vector s_b[N];      // Body frame vector array
    Vector k12;
    Matrix B, S, Id;
    float k22;
    float a, b, c, d;
    float alpha, beta;
    float detS;
    float normQ;

    /* Preparation of the Newton optimisation problem to find lambda */
    for(int i = 0; i < N; i++){
        lambda0 += omega[i];
    }

    for(int i = 0; i < N; i++){
        s_a[i] = Vector(s_eci[i]);
        s_b[i] = Vector(s_body[i]);
    }

    for(int i = 0; i < N; i++){
        B += omega[i] * ( Matrix::vec_mul(s_a[i], s_b[i]) );
    }

    B = B.transpose();

    pc2.printf("\n\nB matrix\n\r");
    printMat2(B);

    S = B + B.transpose();
    detS = S.det();

    k22 = B.tr();
    k12.setCoef(0, B.getCoef(1, 2)-B.getCoef(2, 1));
    k12.setCoef(1, B.getCoef(2, 0)-B.getCoef(0, 2));
    k12.setCoef(2, B.getCoef(0, 1)-B.getCoef(1, 0));

    a = k22 * k22 - (S.adj()).tr();
    b = k22 * k22 + k12.dot(k12);
    c = detS + k12.dot(S * k12);
    d = k12.dot(S * S * k12);

    /* Newton's optimization method to find lambda */
    int iteration = 0;
    lambda = lambda0;
    float lambda_prev = 0;
    while(fabs(lambda-lambda_prev) > tolerance && iteration < 10000){
        // pc2.printf("In loop\n\r");
        lambda_prev = lambda;
        lambda -= (lambda*lambda*lambda*lambda - (a + b) * lambda * lambda - c *lambda + (a * b + c * k22 - d)) / (4 * lambda * lambda * lambda - 2 * (a + b) * lambda - c);
        iteration++;
    }

    /* Then find the eigen vector associated with the eigen value lambda */
    Id.setCoef(0, 0, 1.0f);     // Creating the identity matrix
    Id.setCoef(1, 1, 1.0f);
    Id.setCoef(2, 2, 1.0f);

    alpha = lambda * lambda - a;
    beta = lambda - k22;
    gamma = (lambda + k22) * alpha - detS; 
    x = (alpha * Id + beta * S + S * S) * k12;

    normQ = sqrt(gamma * gamma + x.dot(x));
    x /= normQ;
    gamma /= normQ;

    /* Returning the quaternion */
    quat[0] = x.getCoef(0);
    quat[1] = x.getCoef(1);
    quat[2] = x.getCoef(2);
    quat[3] = gamma;
}

void printMat2(SimpleMatrix::Matrix a){
    float pcoef[9];
    a.getCoef(pcoef);
    pc2.printf("[[ %f, %f, %f ]\n\r" , pcoef[0], pcoef[1], pcoef[2]);
    pc2.printf(" [ %f, %f, %f ]\n\r" , pcoef[3], pcoef[4], pcoef[5]);
    pc2.printf(" [ %f, %f, %f ]]\n\r", pcoef[6], pcoef[7], pcoef[8]);
}