#define MT Serial1

// Xbus Message IDs
const uint8_t PREAMBLE = 0xFA;
const uint8_t BID      = 0xFF;
const uint8_t MID_GOTOCONFIG  = 0x30;
const uint8_t MID_GOTOMEAS    = 0x10;
const uint8_t MID_SETOUTPUTCONFIG = 0xC0;
const uint8_t MID_MTDATA2     = 0x36;

// compute Xbus checksum
uint8_t xbusChecksum(const uint8_t *msg, uint8_t len) {
  uint8_t sum = 0;
  for (uint8_t i = 1; i < len; i++) sum += msg[i];  // skip preamble
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

// wait for an Xbus message with specific MID
bool waitForMID(uint8_t targetMID, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (MT.available() >= 5) {
      if (MT.read() == PREAMBLE) {
        if (MT.read() == BID) {
          uint8_t mid = MT.read();
          uint8_t len = MT.read();
          // consume payload + checksum
          for (uint8_t i = 0; i < len + 1; i++) {
            while (!MT.available());
            MT.read();
          }
          if (mid == targetMID) return true;
        }
      }
    }
  }
  return false;
}

// parse big-endian float32 from byte array
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
  
  Serial.println("Starting MTi-3 config...");
  
  //Step 1: GoToConfig
  sendXbus(MID_GOTOCONFIG, nullptr, 0);
  if (waitForMID(0x31, 2000)) Serial.println("Config ACK received.");
  else Serial.println("No config ACK (continuing anyway).");
  
  // Step 2: SetOutputConfiguration
  // Request: Acceleration(0x4020), RateOfTurn(0x8020), MagneticField(0xC020) @ 100Hz
  uint8_t outCfg[] = {
    0x40, 0x20, 0x00, 0x64,  // Acceleration, 100Hz
    0x80, 0x20, 0x00, 0x64,  // RateOfTurn,   100Hz
    0xC0, 0x20, 0x00, 0x64   // MagneticField,100Hz
  };
  sendXbus(MID_SETOUTPUTCONFIG, outCfg, sizeof(outCfg));
  if (waitForMID(0xC1, 2000)) Serial.println("OutputConfig ACK.");
  else Serial.println("No outputConfig ACK.");
  
  // Step 3: GoToMeasurement
  sendXbus(MID_GOTOMEAS, nullptr, 0);
  if (waitForMID(0x11, 2000)) Serial.println("Measurement ACK. Streaming...");
  else Serial.println("No measurement ACK (continuing).");
  
  Serial.println("ax,ay,az,gx,gy,gz,mx,my,mz");
}

void loop() {
  // Look for MTData2 packets
  if (MT.available() >= 4) {
    if (MT.read() != PREAMBLE) return;
    if (MT.read() != BID) return;
    uint8_t mid = MT.read();
    uint8_t len = MT.read();
    
    if (mid != MID_MTDATA2) {
      // skip
      for (uint8_t i = 0; i < len + 1; i++) {
        while (!MT.available());
        MT.read();
      }
      return;
    }
    
    // Read payload
    uint8_t payload[128];
    for (uint8_t i = 0; i < len; i++) {
      while (!MT.available());
      payload[i] = MT.read();
    }
    MT.read();  // checksum (ignore)
    
    // Parse payload: each data packet = 2B ID + 1B len + data
    float ax=0,ay=0,az=0, gx=0,gy=0,gz=0, mx=0,my=0,mz=0;
    uint8_t i = 0;
    while (i < len) {
      uint16_t did = ((uint16_t)payload[i] << 8) | payload[i+1];
      uint8_t dlen = payload[i+2];
      uint8_t *dp = &payload[i+3];
      
      if (did == 0x4020 && dlen == 12) {
        ax = parseBEFloat(dp);
        ay = parseBEFloat(dp+4);
        az = parseBEFloat(dp+8);
      } else if (did == 0x8020 && dlen == 12) {
        gx = parseBEFloat(dp);
        gy = parseBEFloat(dp+4);
        gz = parseBEFloat(dp+8);
      } else if (did == 0xC020 && dlen == 12) {
        mx = parseBEFloat(dp);
        my = parseBEFloat(dp+4);
        mz = parseBEFloat(dp+8);
      }
      i += 3 + dlen;
    }
    
    // Print CSV
    Serial.print(ax,4); Serial.print(",");
    Serial.print(ay,4); Serial.print(",");
    Serial.print(az,4); Serial.print(",");
    Serial.print(gx,4); Serial.print(",");
    Serial.print(gy,4); Serial.print(",");
    Serial.print(gz,4); Serial.print(",");
    Serial.print(mx,4); Serial.print(",");
    Serial.print(my,4); Serial.print(",");
    Serial.println(mz,4);
  }
}