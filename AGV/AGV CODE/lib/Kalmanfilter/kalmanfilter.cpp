#include <Arduino.h> //gör så man kan definiera pins mm
#include <types.h>
#include "kalmanfilter.h"
#include <math.h>

KalmanFilter::KalmanFilter(float dt) // konstruktor för klassen KalmanFilter. dt är tidssteg mellan mätningar.
{
  _dt = dt;
  _init_matrices(); // sätter upp alla matriser, A, H, P, Q, R
}

// 3. Kalman gain, 4. Correction, 5. Error matrix
void KalmanFilter::update(float z[meas_dim], float accX, float accY, float gyroZ)
{
  _predict(accX, accY, gyroZ);

  float y[meas_dim]; // y ska innehålla skillnaden mellan sensor och modell

  // innovation, y = z - Hx. detta används i 4. correction
  for (int i = 0; i < meas_dim; i++)
  {

    float hx = 0;

    for (int j = 0; j < state_dim; j++)
      hx += H[i][j] * x[j];

    y[i] = z[i] - hx;
  }

  // 3. Kalman gain (förberedelse), S = HPH^T + R
  float HP[meas_dim][state_dim];

  for (int i = 0; i < meas_dim; i++)
    for (int j = 0; j < state_dim; j++)
    {
      HP[i][j] = 0;

      for (int k = 0; k < state_dim; k++)
        HP[i][j] += H[i][k] * P[k][j];
    }

  float S[meas_dim][meas_dim];

  for (int i = 0; i < meas_dim; i++)
    for (int j = 0; j < meas_dim; j++)
    {
      S[i][j] = 0;

      for (int k = 0; k < state_dim; k++)
        S[i][j] += HP[i][k] * H[j][k];

      S[i][j] += R[i][j];
    }

  float Sinv[meas_dim][meas_dim];

  _invert2x2(S, Sinv);

  // 3. Kalman gain, K = PH^T S^-1
  float PHt[state_dim][meas_dim];

  // beräknar PH^T
  for (int i = 0; i < state_dim; i++)
    for (int j = 0; j < meas_dim; j++)
    {
      PHt[i][j] = 0;

      for (int k = 0; k < state_dim; k++)
        PHt[i][j] += P[i][k] * H[j][k];
    }

  // beräknar K (Kalman gain), hur mycket sensorer ska påverka state
  for (int i = 0; i < state_dim; i++)
    for (int j = 0; j < meas_dim; j++)
    {
      K[i][j] = 0;

      for (int k = 0; k < meas_dim; k++)
        K[i][j] += PHt[i][k] * Sinv[k][j];
    }

  // 4. Correction
  for (int i = 0; i < state_dim; i++)
    for (int j = 0; j < meas_dim; j++)
      x[i] += K[i][j] * y[j];

  // 5. Error matrix
  float KH[state_dim][state_dim];

  for (int i = 0; i < state_dim; i++)
    for (int j = 0; j < state_dim; j++)
    {
      KH[i][j] = 0;

      for (int k = 0; k < meas_dim; k++)
        KH[i][j] += K[i][k] * H[k][j];
    }

  float I_KH[state_dim][state_dim];

  for (int i = 0; i < state_dim; i++)
    for (int j = 0; j < state_dim; j++)
    {

      if (i == j)
        I_KH[i][j] = 1.0f - KH[i][j];
      else
        I_KH[i][j] = -KH[i][j];
    }

  float temp[state_dim][state_dim];

  for (int i = 0; i < state_dim; i++)
    for (int j = 0; j < state_dim; j++)
    {
      temp[i][j] = 0;

      for (int k = 0; k < state_dim; k++)
        temp[i][j] += I_KH[i][k] * P[k][j];
    }

  float P_new[state_dim][state_dim];

  for (int i = 0; i < state_dim; i++)
    for (int j = 0; j < state_dim; j++)
    {
      P_new[i][j] = 0;

      for (int k = 0; k < state_dim; k++)
        P_new[i][j] += temp[i][k] * I_KH[j][k];
    }

  for (int i = 0; i < state_dim; i++)
    for (int j = 0; j < state_dim; j++)
      P[i][j] = P_new[i][j];
}

// sätt upp alla matriser
void KalmanFilter::_init_matrices()
{
  //reset state
  for (int i = 0; i < state_dim; i++)
    x[i] = 0;

  //p matrix
  for (int i = 0; i < state_dim; i++)
  {
    for (int j = 0; j < state_dim; j++)
      P[i][j] = (i == j) ? 1 : 0;
  }

  //q matrix
  for (int i = 0; i < state_dim; i++)
  {
    for (int j = 0; j < state_dim; j++)
      Q[i][j] = 0;
  }

  Q[0][0] = 0.01;
  Q[1][1] = 0.01;

  Q[2][2] = 0.01;
  Q[3][3] = 0.01;

  Q[4][4] = 0.05;

  //r matrix
  for (int i = 0; i < meas_dim; i++)
  {
    for (int j = 0; j < meas_dim; j++)
      R[i][j] = 0;
  }

  R[0][0] = 0.05;
  R[1][1] = 0.05;

  //h matrix
  for (int i = 0; i < meas_dim; i++)
  { 
    for (int j = 0; j < state_dim; j++)
    {
      H[i][j] = 0;
    }
  }

  //uwb x
  H[0][0] = 1;

  //uwb y
  H[1][1] = 1;

}

  /*
  float dt2 = dt * dt;

  // reset state vector -> x = [0 0 0 0 0 0]
  for (int i = 0; i < state_dim; i++)
    x[i] = 0;

  // P matrix (covariance), får en identitetsmatris -> varje state har en initial osäkerhet = 1
  for (int i = 0; i < state_dim; i++)
    for (int j = 0; j < state_dim; j++)
      P[i][j] = (i == j) ? 1 : 0;



  // Q matrix (process noise), identitetsmatris fast med 0.01
  for (int i = 0; i < state_dim; i++)
    for (int j = 0; j < state_dim; j++)
      Q[i][j] = (i == j) ? 0.01 : 0;

  // R matrix (measurement noise), eftersom brus för DWM är lägre betyder det att filtret litar mer på DWM än IMU
  R[0][0] = 0.5; // IMU acceleration x, brus 0.5
  R[0][1] = 0;
  R[0][2] = 0;
  R[0][3] = 0;
  R[1][0] = 0;
  R[1][1] = 0.5; // IMU acceleration y, brus 0.5
  R[1][2] = 0;
  R[1][3] = 0;
  R[2][0] = 0;
  R[2][1] = 0;
  R[2][2] = 0.05; // DWM x, brus 0.05
  R[2][3] = 0;
  R[3][0] = 0;
  R[3][1] = 0;
  R[3][2] = 0;
  R[3][3] = 0.05; // DWM y, brus 0.05

  // A matrix (transition matrix)

  // matrisen nollställs
  for (int i = 0; i < state_dim; i++)
    for (int j = 0; j < state_dim; j++)
      A[i][j] = 0;

  // elementen sätts -> A = [1 0 0 0 0 0; 1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0], detta gör att state skiftas mellan tidssteg
  A[0][0] = 1;

  A[1][0] = 1;
  A[2][1] = 1;

  A[3][3] = 1;

  A[4][3] = 1;
  A[5][4] = 1;

  // H matrix (observation matrix)

  float c = 1.0 / (dt2);

  // H = [1/dt^2 -2/dt^2 1/dt^2 0 0 0; 0 0 0 1/dt^2 -2/dt^2 1/dt^2; 1 0 0 0 0 0; 0 0 0 1 0 0], rad 1 -> acc x, rad 2 -> acc y, rad 3 -> DWM x, rad 4 -> DWM y

  H[0][0] = c;
  H[0][1] = -2 * c;
  H[0][2] = c;
  H[0][3] = 0;
  H[0][4] = 0;
  H[0][5] = 0;

  H[1][0] = 0;
  H[1][1] = 0;
  H[1][2] = 0;
  H[1][3] = c;
  H[1][4] = -2 * c;
  H[1][5] = c;

  H[2][0] = 1;
  H[2][1] = 0;
  H[2][2] = 0;
  H[2][3] = 0;
  H[2][4] = 0;
  H[2][5] = 0;

  H[3][0] = 0;
  H[3][1] = 0;
  H[3][2] = 0;
  H[3][3] = 1;
  H[3][4] = 0;
  H[3][5] = 0;
}

*/

  // 1. Prediction och 2. Prediction error
  void KalmanFilter::_predict(float accX, float accY, float gyroZ)
  {

    float dt2 = _dt * _dt;

    //rotate imu -> world frame

    float yaw = x[4];

    float cosYaw = cos(yaw);
    float sinYaw = sin(yaw);

    float accWorldX = cosYaw * accX - sinYaw * accY;

    float accWorldY = sinYaw * accX + cosYaw * accY;

    // motion model
    x[0] += x[2] * _dt + 0.5f * accWorldX * dt2;
    x[1] += x[3] * _dt + 0.5f * accWorldY * dt2;

    x[2] += accWorldX * _dt;
    x[3] += accWorldY * _dt;

    x[4] += gyroZ * _dt;

    while (x[4] > PI)
      x[4] -= 2.0f * PI;

    while (x[4] < -PI)
      x[4] += 2.0f * PI;

    // a matrix
    for (int i = 0; i < state_dim; i++)
    {
      for (int j = 0; j < state_dim; j++)
        A[i][j] = 0;
    }

    A[0][0] = 1;
    A[0][2] = _dt;

    A[1][1] = 1;
    A[1][3] = _dt;

    A[2][2] = 1;

    A[3][3] = 1;

    A[4][4] = 1;

    //prediction covariance, P = A P A^T + Q
    float AP[state_dim][state_dim];

    for (int i = 0; i < state_dim; i++)
    {
      for (int j = 0; j < state_dim; j++)
      {
        AP[i][j] = 0;

        for (int k = 0; k < state_dim; k++)
          AP[i][j] += A[i][k] * P[k][j];
      }
    }

    float APA[state_dim][state_dim];

    for (int i = 0; i < state_dim; i++)
    {
      for (int j = 0; j < state_dim; j++)
      {
        APA[i][j] = 0;

        for (int k = 0; k < state_dim; k++)
          APA[i][j] += AP[i][k] * A[j][k];
      }
    }

    for (int i = 0; i < state_dim; i++)
    {
      for (int j = 0; j < state_dim; j++)
        P[i][j] = APA[i][j] + Q[i][j];
    }

  }

    // 3x3 matrix inverse
  void KalmanFilter::_invert2x2(float M[2][2], float inv[2][2])
  {
    float det =
        M[0][0] * M[1][1] -
        M[0][1] * M[1][0];

    if (fabs(det) < 1e-6f)
        return;

    float invDet = 1.0f / det;

    inv[0][0] =  M[1][1] * invDet;
    inv[0][1] = -M[0][1] * invDet;

    inv[1][0] = -M[1][0] * invDet;
    inv[1][1] =  M[0][0] * invDet;
}

      
  



        /*
        float x_new[state_dim]; // skapa temporär vektor för nytt state

        // 1. Prediction, x_new = A*x,
        for (int i = 0; i < state_dim; i++)
        {
          x_new[i] = 0;

          for (int j = 0; j < state_dim; j++)
            x_new[i] += A[i][j] * x[j];
        }

        for (int i = 0; i < state_dim; i++)
          x[i] = x_new[i];

        // 2. Prediction error, P = A P A^T + Q.

        float AP[state_dim][state_dim];

        for (int i = 0; i < state_dim; i++)
          for (int j = 0; j < state_dim; j++)
          {
            AP[i][j] = 0;

            // AP = A*P
            for (int k = 0; k < state_dim; k++)
              AP[i][j] += A[i][k] * P[k][j];
          }

        float APA[state_dim][state_dim];

        for (int i = 0; i < state_dim; i++)
          for (int j = 0; j < state_dim; j++)
          {
            APA[i][j] = 0;

            // APA = AP * A^T
            for (int k = 0; k < state_dim; k++)
              APA[i][j] += AP[i][k] * A[j][k];
          }

        // lägg till process noise Q, P = APA + Q->= A P A ^ T + Q
        for (int i = 0; i < state_dim; i++)
          for (int j = 0; j < state_dim; j++)
            P[i][j] = APA[i][j] + Q[i][j];
      }

      

    // för att beräkna invers av 4x4 matris (gauss jordan)
    void KalmanFilter::_invert4x4(float M[4][4], float inv[4][4])
    {
      float aug[4][8];
      for (int i = 0; i < 4; i++)
      {
        for (int j = 0; j < 4; j++)
          aug[i][j] = M[i][j];
        for (int j = 0; j < 4; j++)
          aug[i][j + 4] = (i == j) ? 1.0f : 0.0f;
      }

      for (int col = 0; col < 4; col++)
      {
        float pivot = aug[col][col];
        for (int j = 0; j < 8; j++)
          aug[col][j] /= pivot;
        for (int row = 0; row < 4; row++)
        {
          if (row == col)
            continue;
          float factor = aug[row][col];
          for (int j = 0; j < 8; j++)
            aug[row][j] -= factor * aug[col][j];
        }
      }

      for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
          inv[i][j] = aug[i][j + 4];
    };

    */
