#include <math.h>

#define MT Serial1

#define deltat 0.01f  // 1/100Hz = 0.01s (Xsens configured at 100Hz below)
#define gyroMeasError (M_PI * (5.0f / 180.0f))    // 5 deg/s expressed in rad/s
#define beta (sqrt(3.0f / 4.0f) * gyroMeasError)  // ~0.0766

// Filter state (global)
float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f;

void madgwickUpdate(float w_x, float w_y, float w_z,
                    float a_x, float a_y, float a_z)
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

  // Normalise accelerometer (units cancel out - safe for m/s^2 or g)
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
  norm = sqrt(SEqHatDot_1*SEqHatDot_1 + SEqHatDot_2*SEqHatDot_2 +
              SEqHatDot_3*SEqHatDot_3 + SEqHatDot_4*SEqHatDot_4);
  if (norm == 0.0f) return;
  SEqHatDot_1 /= norm; SEqHatDot_2 /= norm;
  SEqHatDot_3 /= norm; SEqHatDot_4 /= norm;

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

  // SetOutputConfiguration: Acc(100Hz) + Gyro(100Hz) only (NO magnetometer)
  uint8_t outCfg[] = {
    0x40, 0x20, 0x00, 0x64,  // Acceleration 100Hz
    0x80, 0x20, 0x00, 0x64   // RateOfTurn 100Hz
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

  // Parse acc and gyro from payload (ignore anything else)
  float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
  uint8_t i = 0;
  while (i < len) {
    uint16_t did = ((uint16_t)payload[i] << 8) | payload[i+1];
    uint8_t dlen = payload[i+2];
    uint8_t *dp = &payload[i+3];
    if      (did == 0x4020 && dlen == 12) { ax = parseBEFloat(dp); ay = parseBEFloat(dp+4); az = parseBEFloat(dp+8); }
    else if (did == 0x8020 && dlen == 12) { gx = parseBEFloat(dp); gy = parseBEFloat(dp+4); gz = parseBEFloat(dp+8); }
    i += 3 + dlen;
  }

  // Run Madgwick IMU filter (acc in m/s^2 - normalised inside; gyro in rad/s - native)
  madgwickUpdate(gx, gy, gz, ax, ay, az);

  // Send quaternion to MATLAB
  Serial.print(SEq_1, 6); Serial.print(",");
  Serial.print(SEq_2, 6); Serial.print(",");
  Serial.print(SEq_3, 6); Serial.print(",");
  Serial.println(SEq_4, 6);
}
