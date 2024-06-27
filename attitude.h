#ifndef ATTITUDE_H
#define ATTITUDE_H

#define COS45DEG 0.525322f

struct DCM {
  float a11;
  float a12;
  float a13;
  float a21;
  float a22;
  float a23;
  float a31;
  float a32;
  float a33;
};

class Attitude {
  private:
    float quat[4]; // attitude quaternion w.r.t initial attitude
    DCM dcm; // dcm corresponding to attitude quaternion
    void updateDCM(); // update DCM

  public:
    void initAttitude();
    void updateAttitude(float* new_quat);
    bool isTilted45deg();
};

void Attitude::initAttitude() {
  quat[0] = 1.0f;
  quat[1] = 0.0f;
  quat[2] = 0.0f;
  quat[3] = 0.0f;

  return;
}

void Attitude::updateAttitude(float* new_quat) {
  quat[0] = new_quat[0];
  quat[1] = new_quat[1];
  quat[2] = new_quat[2];
  quat[3] = new_quat[3];
  updateDCM(); // update DCM as well, from new quaternion estimate.

  return;
}

void Attitude::updateDCM() {
  float q0, q1, q2, q3;
  float q0q0, q1q1, q2q2, q3q3, q0q1, q0q2, q0q3, q1q2, q1q3, q2q3;

  q0 = quat[0]; q1 = quat[1]; q2 = quat[2]; q3 = quat[3];

  // for calculation
  q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3;
  q0q1 = q0*q1; q0q2 = q0*q2; q0q3 = q0*q3; q1q2 = q1*q2; q1q3 = q1*q3; q2q3 = q2*q3;

  // construct elements of DCM
  dcm.a11 = q0q0 + q1q1 - q2q2 - q3q3;
  dcm.a12 = 2*(q1q2 + q0q3);
  dcm.a13 = 2*(q1q3 - q0q2);

  dcm.a21 = 2*(q1q2 - q0q3);
  dcm.a22 = q0q0 - q1q1 + q2q2 - q3q3;
  dcm.a23 = 2*(q2q3 + q0q1);

  dcm.a31 = 2*(q1q3 + q0q2);
  dcm.a32 = 2*(q2q3 - q0q1);
  dcm.a33 = q0q0 - q1q1 - q2q2 + q3q3;

  return;
}

bool Attitude::isTilted45deg() {
  float xb_proj_on_xi;
  xb_proj_on_xi = dcm.a11;

  return (xb_proj_on_xi > COS45DEG);
}

#endif // MPU9250_H