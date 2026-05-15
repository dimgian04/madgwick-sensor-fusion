#include <Arduino_LSM6DS3.h>
#include <math.h>

#define deltat 0.0096f  // ~1/104Hz, LSM6DS3 sample rate
#define gyroMeasError (M_PI * (5.0f / 180.0f))
#define beta (sqrt(3.0f / 4.0f) * gyroMeasError)

float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f;

void madgwickUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z)
{
  float norm;
  float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4;
  float f_1, f_2, f_3;
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;
  float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;

  float halfSEq_1 = 0.5f * SEq_1;
  float halfSEq_2 = 0.5f * SEq_2;
  float halfSEq_3 = 0.5f * SEq_3;
  float halfSEq_4 = 0.5f * SEq_4;
  float twoSEq_1 = 2.0f * SEq_1;
  float twoSEq_2 = 2.0f * SEq_2;
  float twoSEq_3 = 2.0f * SEq_3;

  // Normalise accelerometer
  norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
  if (norm == 0.0f) return;
  a_x /= norm; a_y /= norm; a_z /= norm;

  // Objective function (Eq. 25)
  f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
  f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
  f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;

  // Jacobian (Eq. 26)
  J_11or24 = twoSEq_3;
  J_12or23 = 2.0f * SEq_4;
  J_13or22 = twoSEq_1;
  J_14or21 = twoSEq_2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;

  // Gradient = J^T * f (Eq. 20)
  SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
  SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
  SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
  SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

  // Normalise gradient
  norm = sqrt(SEqHatDot_1*SEqHatDot_1 + SEqHatDot_2*SEqHatDot_2 + SEqHatDot_3*SEqHatDot_3 + SEqHatDot_4*SEqHatDot_4);
  if (norm == 0.0f) return;
  SEqHatDot_1 /= norm; SEqHatDot_2 /= norm; SEqHatDot_3 /= norm; SEqHatDot_4 /= norm;

  // Quaternion rate from gyroscope (Eq. 12)
  SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
  SEqDot_omega_2 =  halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
  SEqDot_omega_3 =  halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
  SEqDot_omega_4 =  halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

  // Fuse: q_dot = q_dot_gyro - beta * gradient (Eq. 43), then integrate (Eq. 42)
  SEq_1 += (SEqDot_omega_1 - beta * SEqHatDot_1) * deltat;
  SEq_2 += (SEqDot_omega_2 - beta * SEqHatDot_2) * deltat;
  SEq_3 += (SEqDot_omega_3 - beta * SEqHatDot_3) * deltat;
  SEq_4 += (SEqDot_omega_4 - beta * SEqHatDot_4) * deltat;

  // Normalise quaternion
  norm = sqrt(SEq_1*SEq_1 + SEq_2*SEq_2 + SEq_3*SEq_3 + SEq_4*SEq_4);
  SEq_1 /= norm; SEq_2 /= norm; SEq_3 /= norm; SEq_4 /= norm;
}


void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("IMU init failed!");
    while (1);
  }
}

void loop() {
  float ax, ay, az, gx, gy, gz;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);   // in g
    IMU.readGyroscope(gx, gy, gz);      // in deg/s

    // Convert to filter units: acc needs g (already), gyro needs rad/s
    float wx = gx * M_PI / 180.0f;
    float wy = gy * M_PI / 180.0f;
    float wz = gz * M_PI / 180.0f;

    madgwickUpdate(wx, wy, wz, ax, ay, az);

    Serial.print(SEq_1, 6); Serial.print(",");
    Serial.print(SEq_2, 6); Serial.print(",");
    Serial.print(SEq_3, 6); Serial.print(",");
    Serial.println(SEq_4, 6);
  }
}
