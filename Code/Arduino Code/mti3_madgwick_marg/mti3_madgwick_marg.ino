#include <math.h>

#define MT Serial1


#define deltat 0.01f  // 1/sampleRate. At 100Hz this is 0.01
//#define gyroMeasError (M_PI * (5.0f / 180.0f))   // gyro error in rad/s (5 deg/s)
//#define gyroMeasError (M_PI * (0.029f / 180.0f))   // gyro error in rad/s (0.029 deg/s RMS across all axes)
//#define gyroMeasDrift (M_PI * (0.2f / 180.0f))    // gyro drift in rad/s/s (0.2 deg/s/s)
//#define beta  (sqrt(3.0f / 4.0f) * gyroMeasError) // ~0.0766
//#define zeta  (sqrt(3.0f / 4.0f) * gyroMeasDrift)  // ~0.00307

#define beta 0.06f
#define zeta 0.005f

// Filter state (global)
float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f;
float b_x = 1.0f, b_z = 0.0f;
float w_bx = 0.0f, w_by = 0.0f, w_bz = 0.0f;

void madgwickUpdate(float w_x, float w_y, float w_z,
                    float a_x, float a_y, float a_z,
                    float m_x, float m_y, float m_z)
{
  float norm;
  float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4;
  float f_1, f_2, f_3, f_4, f_5, f_6;
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;
  float J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64;
  float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;
  float w_err_x, w_err_y, w_err_z;
  float h_x, h_y, h_z;

  float halfSEq_1 = 0.5f * SEq_1;
  float halfSEq_2 = 0.5f * SEq_2;
  float halfSEq_3 = 0.5f * SEq_3;
  float halfSEq_4 = 0.5f * SEq_4;
  float twoSEq_1 = 2.0f * SEq_1;
  float twoSEq_2 = 2.0f * SEq_2;
  float twoSEq_3 = 2.0f * SEq_3;
  float twoSEq_4 = 2.0f * SEq_4;
  float twob_x = 2.0f * b_x;
  float twob_z = 2.0f * b_z;
  float twob_xSEq_1 = 2.0f * b_x * SEq_1;
  float twob_xSEq_2 = 2.0f * b_x * SEq_2;
  float twob_xSEq_3 = 2.0f * b_x * SEq_3;
  float twob_xSEq_4 = 2.0f * b_x * SEq_4;
  float twob_zSEq_1 = 2.0f * b_z * SEq_1;
  float twob_zSEq_2 = 2.0f * b_z * SEq_2;
  float twob_zSEq_3 = 2.0f * b_z * SEq_3;
  float twob_zSEq_4 = 2.0f * b_z * SEq_4;
  float SEq_1SEq_2;
  float SEq_1SEq_3 = SEq_1 * SEq_3;
  float SEq_1SEq_4;
  float SEq_2SEq_3;
  float SEq_2SEq_4 = SEq_2 * SEq_4;
  float SEq_3SEq_4;
  float twom_x = 2.0f * m_x;
  float twom_y = 2.0f * m_y;
  float twom_z = 2.0f * m_z;

  // Normalise accelerometer
  norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
  if (norm == 0.0f) return;
  a_x /= norm; a_y /= norm; a_z /= norm;

  // Normalise magnetometer
  norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
  if (norm == 0.0f) return;
  m_x /= norm; m_y /= norm; m_z /= norm;

  // Objective function (Eq. 25 + 29)
  f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
  f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
  f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
  f_4 = twob_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
  f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y;
  f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z;

  // Jacobian (Eq. 26 + 30)
  J_11or24 = twoSEq_3;
  J_12or23 = 2.0f * SEq_4;
  J_13or22 = twoSEq_1;
  J_14or21 = twoSEq_2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;
  J_41 = twob_zSEq_3;
  J_42 = twob_zSEq_4;
  J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1;
  J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2;
  J_51 = twob_xSEq_4 - twob_zSEq_2;
  J_52 = twob_xSEq_3 + twob_zSEq_1;
  J_53 = twob_xSEq_2 + twob_zSEq_4;
  J_54 = twob_xSEq_1 - twob_zSEq_3;
  J_61 = twob_xSEq_3;
  J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
  J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
  J_64 = twob_xSEq_2;

  // Gradient = J^T * f (Eq. 20)
  SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
  SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
  SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
  SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;

  // Normalise gradient
  norm = sqrt(SEqHatDot_1*SEqHatDot_1 + SEqHatDot_2*SEqHatDot_2 + SEqHatDot_3*SEqHatDot_3 + SEqHatDot_4*SEqHatDot_4);
  if (norm == 0.0f) return;
  SEqHatDot_1 /= norm; SEqHatDot_2 /= norm; SEqHatDot_3 /= norm; SEqHatDot_4 /= norm;

  // Gyroscope bias drift compensation (Section 3.5, Eq. 47-49)
  w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
  w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
  w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
  w_bx += w_err_x * deltat * zeta;
  w_by += w_err_y * deltat * zeta;
  w_bz += w_err_z * deltat * zeta;
  w_x -= w_bx;
  w_y -= w_by;
  w_z -= w_bz;

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

  // Magnetic distortion compensation (Section 3.4, Eq. 45-46)
  SEq_1SEq_2 = SEq_1 * SEq_2;
  SEq_1SEq_3 = SEq_1 * SEq_3;
  SEq_1SEq_4 = SEq_1 * SEq_4;
  SEq_3SEq_4 = SEq_3 * SEq_4;
  SEq_2SEq_3 = SEq_2 * SEq_3;
  SEq_2SEq_4 = SEq_2 * SEq_4;
  h_x = twom_x * (0.5f - SEq_3*SEq_3 - SEq_4*SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
  h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEq_2*SEq_2 - SEq_4*SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
  h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEq_2*SEq_2 - SEq_3*SEq_3);
  b_x = sqrt(h_x * h_x + h_y * h_y);
  b_z = h_z;
}

const uint8_t PREAMBLE = 0xFA;
const uint8_t BID      = 0xFF;

uint8_t xbusChecksum(const uint8_t *msg, uint8_t len) {
  uint8_t sum = 0;
  for (uint8_t i = 1; i < len; i++) sum += msg[i];
  return (uint8_t)(-sum);
}

void sendXbus(uint8_t mid, const uint8_t *data, uint8_t dataLen) {
  uint8_t msg[64];
  msg[0] = PREAMBLE;
  msg[1] = BID;
  msg[2] = mid;
  msg[3] = dataLen;
  for (uint8_t i = 0; i < dataLen; i++) msg[4 + i] = data[i];
  msg[4 + dataLen] = xbusChecksum(msg, 4 + dataLen);
  MT.write(msg, 5 + dataLen);
  MT.flush();
}

bool waitForMID(uint8_t targetMID, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (MT.available() >= 5) {
      if (MT.read() == PREAMBLE && MT.read() == BID) {
        uint8_t mid = MT.read();
        uint8_t len = MT.read();
        for (uint8_t i = 0; i < len + 1; i++) { while (!MT.available()); MT.read(); }
        if (mid == targetMID) return true;
      }
    }
  }
  return false;
}

float parseBEFloat(const uint8_t *p) {
  uint8_t b[4] = { p[3], p[2], p[1], p[0] };
  float f;
  memcpy(&f, b, 4);
  return f;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  MT.begin(115200);
  delay(500);

  // GoToConfig
  sendXbus(0x30, nullptr, 0);
  waitForMID(0x31, 2000);

  // SetOutputConfiguration: Acc(100Hz) + Gyro(100Hz) + Mag(100Hz)
  uint8_t outCfg[] = {
    0x40, 0x20, 0x00, 0x64,  // Acceleration 100Hz
    0x80, 0x20, 0x00, 0x64,  // RateOfTurn 100Hz
    0xC0, 0x20, 0x00, 0x64   // MagneticField 100Hz
  };
  sendXbus(0xC0, outCfg, sizeof(outCfg));
  waitForMID(0xC1, 2000);

  // GoToMeasurement
  sendXbus(0x10, nullptr, 0);
  waitForMID(0x11, 2000);
}

void loop() {
  if (MT.available() < 4) return;
  if (MT.read() != PREAMBLE) return;
  if (MT.read() != BID) return;
  uint8_t mid = MT.read();
  uint8_t len = MT.read();

  // Read payload
  uint8_t payload[128];
  for (uint8_t i = 0; i < len; i++) { while (!MT.available()); payload[i] = MT.read(); }
  MT.read(); // checksum

  if (mid != 0x36) return; // only process MTData2

  // Parse acc, gyro, mag from payload
  float ax=0,ay=0,az=0, gx=0,gy=0,gz=0, mx=0,my=0,mz=0;
  uint8_t i = 0;
  while (i < len) {
    uint16_t did = ((uint16_t)payload[i] << 8) | payload[i+1];
    uint8_t dlen = payload[i+2];
    uint8_t *dp = &payload[i+3];
    if      (did == 0x4020 && dlen == 12) { ax = parseBEFloat(dp); ay = parseBEFloat(dp+4); az = parseBEFloat(dp+8); }
    else if (did == 0x8020 && dlen == 12) { gx = parseBEFloat(dp); gy = parseBEFloat(dp+4); gz = parseBEFloat(dp+8); }
    else if (did == 0xC020 && dlen == 12) { mx = parseBEFloat(dp); my = parseBEFloat(dp+4); mz = parseBEFloat(dp+8); }
    i += 3 + dlen;
  }

  // Run Madgwick filter (acc in m/s², gyro in rad/s, mag in a.u.)
  madgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz);

  // Send quaternion or quaternion + estimated gyro bias to MATLAB
  Serial.print(SEq_1, 6); Serial.print(",");
  Serial.print(SEq_2, 6); Serial.print(",");
  Serial.print(SEq_3, 6); Serial.print(",");
  Serial.println(SEq_4, 6);
  
  // or quaternion + estimated gyro bias to MATLAB
  //Serial.print(SEq_1, 6); Serial.print(",");
  //Serial.print(SEq_2, 6); Serial.print(",");
  //Serial.print(SEq_3, 6); Serial.print(",");
  //Serial.print(SEq_4, 6); Serial.print(",");
  //Serial.print(w_bx, 6); Serial.print(",");
  //Serial.print(w_by, 6); Serial.print(",");
  //Serial.println(w_bz, 6);
}
