/******************************************************************************
 * File    : 7Semi_BNO08x.cpp
 * Module  : 7Semi_BNO08x
 * Version : 0.1.0
 * License : MIT
 *
 * Notes
 * -----
 * - The transport layer is provided by a BnoBus implementation (I2C/SPI/UART).
 * - SPI typically requires INTN-driven reads for stability.
 ******************************************************************************/

#include "7Semi_BNO08x.h"

#ifndef BNO_RX_CAP
#define BNO_RX_CAP 64
#endif

// ============================ Internal helpers ============================

/**
 * - Read a signed 16-bit integer from little-endian bytes
 * - p[0] = LSB, p[1] = MSB
 */
static inline int16_t read2Bytes(const uint8_t *p)
{
  return (int16_t)((p[1] << 8) | p[0]);
}

/**
 * - Parse 3x int16 vector from a report with given id and scale
 * - pkt   : SHTP frame buffer
 * - n     : frame length
 * - id    : expected SH-2 report id
 * - scale : divisor for fixed-point scaling
 * - out   : output Vec3
 * - return: true if parsed successfully
 */
static inline bool parseVec3L(
    const uint8_t *pkt,
    size_t n,
    uint8_t id,
    float scale,
    Vec3 &out)
{
  if (!pkt || n < 19)
    return false; // 9-byte header + 3*2 bytes data + id
  if (pkt[9] != id)
    return false; // check report id matches

  out.x = read2Bytes(&pkt[13]) / scale; // x at pkt[13..14]
  out.y = read2Bytes(&pkt[15]) / scale; // y at pkt[15..16]
  out.z = read2Bytes(&pkt[17]) / scale; // z at pkt[17..18]
  return true;
}

/**
 * - Parse quaternion (4x int16) from a report with given id and scale
 * - pkt   : SHTP frame buffer
 * - n     : frame length
 * - id    : expected SH-2 report id
 * - scale : divisor for fixed-point scaling
 * - out   : output Quat
 * - return: true if parsed successfully
 * - This function:
 *   - report id is checked at pkt[9]
 *   - i/j/k/r are read from pkt[13..20]
 */
static inline bool parseQuatL(
    const uint8_t *pkt,
    size_t n,
    uint8_t id,
    float scale,
    Quat &out)
{
  if (!pkt || n < 21)
    return false; // 9-byte header + 4*2 bytes data + id
  if (pkt[9] != id)
    return false; // check report id matches

  out.i = read2Bytes(&pkt[13]) / scale; // i at pkt[13..14]
  out.j = read2Bytes(&pkt[15]) / scale; // j at pkt[15..16]
  out.k = read2Bytes(&pkt[17]) / scale; // k at pkt[17..18]
  out.r = read2Bytes(&pkt[19]) / scale; // r at pkt[19..20]
  return true;
}

// =============================== Begin ===============================

/**
 * - Initialize underlying bus (I2C / SPI / UART)
 * - return : true if bus initialized successfully
 * - Bus object must be injected via constructor in .h file
 */
bool BNO08x_7Semi::begin()
{
  return bus && bus->begin(); // initialize bus
}

// =============================== IO wrappers =============================

/**
 * - Read one SHTP frame into user buffer
 * - buffer    : destination buffer
 * - len       : capacity (>=4)
 * - return    : number of bytes copied, 0 if none
 */
int BNO08x_7Semi::readPacket(uint8_t *buffer, size_t len)
{
  if (!bus || !buffer || len < 4)
    return 0;

  int n = bus->rx(buffer, len); // read one SHTP frame
  return (n > 0) ? n : 0;
}

/**
 * - Helper that reads and parses one packet
 * - Call this very frequently in loop() for continuous update
 */
void BNO08x_7Semi::processData()
{
  uint8_t pkt[BNO_RX_CAP];
  int n = readPacket(pkt, sizeof(pkt));
  if (n > 0)
    processPacket(pkt, (size_t)n);
}

// ============================ Packet processor ===========================

/**
 * - Decode one SHTP frame and update cached data
 * - Ignores:
 *   - control channel packets (command responses)
 *   - non-input channels
 * - Updates:
 *   - channel is pkt[2] low nibble
 *   - report id is pkt[9]
 */
void BNO08x_7Semi::processPacket(const uint8_t *pkt, size_t n)
{
  if (!pkt || n < 10)
    return; // must have at least header + id

  const uint8_t ch = pkt[2] & 0x0F; // channel = low nibble of pkt[2]
  const uint8_t report_id = pkt[9]; // report id at pkt[9]
                                    /** Ignore command/control packets */
  if (ch == SHTP_CH_CTRL)
    return;

  /** Accept only input channels */
  if (ch != SHTP_CH_INPUT && ch != SHTP_CH_WAKE && ch != SHTP_CH_GYRO_RV)
    return;

  const uint32_t now = us_ticker_read();

  Vec3 v{};
  Quat q{};
  bool status = false;
  switch (report_id)
  {                                         // report id
  case ACCELEROMETER:                       // 0x01
    status = parseAccelerometer(pkt, n, v); // parse accel
    if (status)
    {
      data.accel_mps2 = v;
      data.hasAccel = true; // new data
    }
    break;

  case GYROSCOPE_CALIBRATED:            // 0x02
    status = parseGyroscope(pkt, n, v); // parse gyro
    if (status)
    {
      data.gyro_rps = v;
      data.hasGyro = true;
    }
    break;

  case MAGNETIC_FIELD_CALIBRATED:          // 0x03
    status = parseMagnetometer(pkt, n, v); // parse mag
    if (status)
    {
      data.mag_uT = v;
      data.hasMag = true;
    }
    break;

  case LINEAR_ACCELERATION:               // 0x04 
    status = parseLinearAccel(pkt, n, v); // parse linear accel
    if (status)
    {
      data.linear_mps2 = v;
      data.hasLinear = true;
    }
    break;

  case GRAVITY:                          // 0x06
    status = parseGravity(pkt, n, v);    // parse gravity
    if (status)
    {
      data.gravity_mps2 = v;
      data.hasGravity = true;
    }
    break;

  case ROTATION_VECTOR:                      // 0x05
    status = parseRotationVector(pkt, n, q); // parse rv
    if (status)
    {
      data.rv_q = q;
      data.hasQuat = true;
    }
    break;

  case GAME_ROTATION_VECTOR:                     // 0x08
    status = parseGameRotationVector(pkt, n, q); // parse game rv
    if (status)
    {
      data.grv_q = q;
      data.hasGameQuat = true;
    }
    break;

  case GEOMAGNETIC_ROTATION_VECTOR:                // 0x09
    status = parseGeoRotationVector(pkt, n, q);    // parse geomagnetic rv
    if (status)
    {
      data.gerv_q = q;
      data.hasGeoQuat = true;
    }
    break;

  default:
    return;
  }

  if (status)
  {
    data.t_us = now;
    data.dataReady = true;
  }
}

// ========================= Raw parsers (pkt -> values) =====================

/**
 * - Parse accelerometer (m/s^2, Q-notation scale)
 * - pkt : SHTP frame buffer
 * - n   : frame length
 * - out : Vec3 output
 * - return : true if parsed successfully
 * - Accelerometer vector
 * - Scaling uses Q8 => divide by 256.0f
 */
bool BNO08x_7Semi::parseAccelerometer(const uint8_t *pkt, size_t n, Vec3 &out) const
{
  return parseVec3L(pkt, n, ACCELEROMETER, 256.0f, out); 
}

/**
 * - Parse gyroscope (rad/s, Q-notation scale)
 * - pkt : SHTP frame buffer
 * - n   : frame length
 * - out : Vec3 output
 * - return : true if parsed successfully
 * - Gyroscope calibrated vector
 * -  Scaling uses Q8 => divide by 256.0f
 */
bool BNO08x_7Semi::parseGyroscope(const uint8_t *pkt, size_t n, Vec3 &out) const
{
  return parseVec3L(pkt, n, GYROSCOPE_CALIBRATED, 256.0f, out);
}

/**
 * Parse magnetometer (µT, Q-notation scale)
 * - pkt : SHTP frame buffer
 * - n   : frame length
 * - out : Vec3 output
 * - return : true if parsed successfully
 * - Magnetometer calibrated vector
 * -  Scaling uses Q8 => divide by 256.0f
 */
bool BNO08x_7Semi::parseMagnetometer(const uint8_t *pkt, size_t n, Vec3 &out) const
{
  return parseVec3L(pkt, n, MAGNETIC_FIELD_CALIBRATED, 256.0f, out);
}

/**
 * - Parse linear acceleration (m/s^2, Q8)
 * - pkt : SHTP frame buffer
 * - n   : frame length
 * - out : Vec3 output
 * - return : true if parsed successfully
 * - Linear acceleration vector
 * -  Scaling uses Q8 => divide by 256.0f
 */
bool BNO08x_7Semi::parseLinearAccel(const uint8_t *pkt, size_t n, Vec3 &out) const
{
  return parseVec3L(pkt, n, LINEAR_ACCELERATION, 256.0f, out);
}

/**
 * - Parse gravity vector (m/s^2, Q8)
 * - pkt : SHTP frame buffer
 * - n   : frame length
 * - out : Vec3 output
 * - return : true if parsed successfully
 * - Gravity vector
 * - Scaling uses Q8 => divide by 256.0f
 */
bool BNO08x_7Semi::parseGravity(const uint8_t *pkt, size_t n, Vec3 &out) const
{
  return parseVec3L(pkt, n, GRAVITY, 256.0f, out);
}

/**
 * - Parse rotation vector quaternion (unitless, Q14)
 * - pkt : SHTP frame buffer
 * - n   : frame length
 * - out : Quat output
 * - return : true if parsed successfully
 * - Rotation vector quaternion
 * - Scaling uses Q14 => divide by 16384.0f
 */
bool BNO08x_7Semi::parseRotationVector(const uint8_t *pkt, size_t n, Quat &out) const
{
  return parseQuatL(pkt, n, ROTATION_VECTOR, 16384.0f, out);
}

/**
 * - Parse game rotation vector (unitless, Q14)
 * - pkt : SHTP frame buffer
 * - n   : frame length
 * - out : Quat output
 * - return : true if parsed successfully
 * - Game rotation vector quaternion
 * - Scaling uses Q14 => divide by 16384.0f
 */
bool BNO08x_7Semi::parseGameRotationVector(const uint8_t *pkt, size_t n, Quat &out) const
{
  return parseQuatL(pkt, n, GAME_ROTATION_VECTOR, 16384.0f, out);
}

/**
 * - Parse geomagnetic rotation vector (unitless, Q14)
 * - pkt : SHTP frame buffer
 * - n   : frame length
 * - out : Quat output
 * - return : true if parsed successfully
 * - Geomagnetic rotation vector quaternion
 * - Scaling uses Q14 => divide by 16384.0f
 */
bool BNO08x_7Semi::parseGeoRotationVector(const uint8_t *pkt, size_t n, Quat &out) const
{
  return parseQuatL(pkt, n, GEOMAGNETIC_ROTATION_VECTOR, 16384.0f, out);
}

// ====================== Cached-data convenience getters ===================

/**
 * - Get latest accelerometer 
 * - x,y,z : outputs (m/s^2)
 * - return: true if available
 * - After reading, the hasAccel flag is cleared
 */
bool BNO08x_7Semi::getAccelerometer(float &x, float &y, float &z)
{
  if (!data.hasAccel)
    return false; // no new data
  data.hasAccel = false;
  x = data.accel_mps2.x;
  y = data.accel_mps2.y;
  z = data.accel_mps2.z;
  return true;
}

/**
 * - Get latest gyroscope from cache
 * - x,y,z : outputs (rad/s)
 * - return: true if available
 * - After reading, the hasGyro flag is cleared
 */
bool BNO08x_7Semi::getGyroscope(float &x, float &y, float &z)
{
  if (!data.hasGyro)
    return false;
  data.hasGyro = false;
  x = data.gyro_rps.x;
  y = data.gyro_rps.y;
  z = data.gyro_rps.z;
  return true;
}

/**
 * 
 * - Get latest magnetometer from cache
 * - x,y,z : outputs (µT)
 * - return: true if available
 * - After reading, the hasMag flag is cleared
 */
bool BNO08x_7Semi::getMagnetometer(float &x, float &y, float &z)
{
  if (!data.hasMag)
    return false;
  data.hasMag = false;
  x = data.mag_uT.x;
  y = data.mag_uT.y;
  z = data.mag_uT.z;
  return true;
}

/**
 * - Get latest linear acceleration from cache
 * - x,y,z : outputs (m/s^2)
 * - return: true if available
 * - After reading, the hasLinear flag is cleared
 */
bool BNO08x_7Semi::getLinearAccel(float &x, float &y, float &z)
{
  if (!data.hasLinear)
    return false;
  data.hasLinear = false;
  x = data.linear_mps2.x;
  y = data.linear_mps2.y;
  z = data.linear_mps2.z;
  return true;
}

/**
 * - Get latest gravity vector from cache
 * - x,y,z : outputs (m/s^2)
 * - return: true if available
 * - After reading, the hasGravity flag is cleared
 */
bool BNO08x_7Semi::getGravity(float &x, float &y, float &z)
{
  if (!data.hasGravity)
    return false;
  data.hasGravity = false;
  x = data.gravity_mps2.x;
  y = data.gravity_mps2.y;
  z = data.gravity_mps2.z;
  return true;
}

/**
 * - Get latest quaternion from cache
 * - i,j,k,r : outputs (unitless)
 * - return  : true if available
 * - Get latest rotation vector quaternion (RV)
 * - After reading, the hasQuat flag is cleared
 */
bool BNO08x_7Semi::getQuaternion(float &i, float &j, float &k, float &r)
{
  if (!data.hasQuat)
    return false;
  data.hasQuat = false;
  i = data.rv_q.i;
  j = data.rv_q.j;
  k = data.rv_q.k;
  r = data.rv_q.r;
  return true;
}

/**
 * - Get latest game rotation vector quaternion (GRV)
 * - i,j,k,r : outputs (unitless)
 * - return  : true if available
 * - After reading, the hasGameQuat flag is cleared
 */
bool BNO08x_7Semi::getGameRotationVector(float &i, float &j, float &k, float &r)
{
  if (!data.hasGameQuat)
    return false;
  data.hasGameQuat = false;
  i = data.grv_q.i;
  j = data.grv_q.j;
  k = data.grv_q.k;
  r = data.grv_q.r;
  return true;
}

/**
 * - Get latest geomagnetic rotation vector quaternion (GeoRV)
 * - i,j,k,r : outputs (unitless)
 * - return  : true if available
 * - After reading, the hasGeoQuat flag is cleared  
 */
bool BNO08x_7Semi::getGeoRotationVector(float &i, float &j, float &k, float &r)
{
  if (!data.hasGeoQuat)
    return false;
  data.hasGeoQuat = false;
  i = data.gerv_q.i;
  j = data.gerv_q.j;
  k = data.gerv_q.k;
  r = data.gerv_q.r;
  return true;
}

// ================================ Internals ================================

/**
 * - Enable one SH-2 report at a given interval
 * - reportId : * report id (e.g., ACCELEROMETER)
 * - intervalMs : requested report interval in milliseconds
 * - channel  : SHTP channel to use
 * - command  : command code (usually 0xFD Set Feature)
 * - return   : true if SetFeature frame transmitted successfully
 * - Builds and sends Set Feature command frame
 */
bool BNO08x_7Semi::writeSetFeature_(
    uint8_t reportId,
    uint32_t intervalMs,
    uint8_t channel,
    uint8_t command)
{
  if (!bus)
    return false;

  const uint32_t us = intervalMs * 1000UL;
  uint8_t tx[21] = {0};
  const uint16_t L = sizeof(tx);
  tx[0] = L & 0xFF; // Length LSB
  tx[1] = L >> 8;   // Length MSB
  tx[2] = channel;  
  tx[3] = seq[channel]++; // Sequence number

  tx[4] = command;  // 0xFD Set Feature
  tx[5] = reportId; // Feature / report id
  tx[6] = 0;        // feature flags
  tx[7] = 0;        // change sensitivity L
  tx[8] = 0;        // change sensitivity H
  tx[9] = us & 0xFF;// report interval LSB
  tx[10] = (us >> 8) & 0xFF;
  tx[11] = (us >> 16) & 0xFF;
  tx[12] = (us >> 24) & 0xFF;// report interval MSB

  /** Batch interval not used */
  for (int i = 13; i < 21; i++)
    tx[i] = 0;   //  not used

  return bus->tx(tx, sizeof(tx));
}

/**
 * - Wait for "Set Feature Response" frame
 * - expects:
 *   - channel == 2 (control)
 *   - reportId == 0xFC (SetFeatureResponse)
 *   - buf[5] == expectedFeatureId
 *   - buf[6] == status (0 = success)
 *
 * - IMPORTANT:
 *   - Some firmwares use Command Response reportId 0xF1 instead.
 *   - If enableReport() prints FAIL but data works, response type may differ.
 */
bool BNO08x_7Semi::waitForSetFeatureResponse(uint8_t expectedFeatureId, uint32_t timeout)
{
  rtos::Kernel::Clock::time_point start = rtos::Kernel::Clock::now();
  uint8_t buf[BNO_RX_CAP];

  while (rtos::Kernel::Clock::now() - start < std::chrono::milliseconds(timeout))
  {
    uint8_t n = bus->rx(buf, BNO_RX_CAP);

    if (n > 4)
    {
      ThisThread::sleep_for(5ms);
      uint8_t channel = buf[2] & 0x0F;
      uint8_t reportId = buf[4]; // Command Response report must be on Control channel and reportId == 0xFC
      if (channel == 2 && reportId == 0xFC)
      {
        uint8_t featureId = buf[5];
        if (featureId == expectedFeatureId)
        {
          uint8_t status = buf[6];
          return (status == 0x00);
        }
      }
    }
    rtos::ThisThread::yield();
  }
  return false;
}

// ====================== Convenience enable functions ======================

/**
 * - Enable accelerometer report
 */
bool BNO08x_7Semi::enableAcc(uint32_t intervalMs)
{
  return enableReport(ACCELEROMETER, intervalMs);
}

/**
 * - Enable calibrated gyro report
 */
bool BNO08x_7Semi::enableGyro(uint32_t intervalMs)
{
  return enableReport(GYROSCOPE_CALIBRATED, intervalMs);
}

/**
 * - Enable calibrated magnetometer report
 */
bool BNO08x_7Semi::enableMag(uint32_t intervalMs)
{
  return enableReport(MAGNETIC_FIELD_CALIBRATED, intervalMs);
}

/**
 * - Enable rotation vector quaternion report
 */
bool BNO08x_7Semi::enableRotationVector(uint32_t intervalMs)
{
  return enableReport(ROTATION_VECTOR, intervalMs);
}

/**
 * - Enable game rotation vector quaternion report
 */
bool BNO08x_7Semi::enableGameRotationVector(uint32_t intervalMs)
{
  return enableReport(GAME_ROTATION_VECTOR, intervalMs);
}

/**
 * - Enable geomagnetic rotation vector quaternion report
 */
bool BNO08x_7Semi::enableGeoRotationVector(uint32_t intervalMs)
{
  return enableReport(GEOMAGNETIC_ROTATION_VECTOR, intervalMs);
}

/**
 * - Enable linear acceleration report
 */
bool BNO08x_7Semi::enableLinearAccel(uint32_t intervalMs)
{
  return enableReport(LINEAR_ACCELERATION, intervalMs);
}

/**
 * - Enable gravity vector report
 */
bool BNO08x_7Semi::enableGravity(uint32_t intervalMs)
{
  return enableReport(GRAVITY, intervalMs);
}

/**
 * - Enable one SH-2 report at a given interval
 * - reportId : * report id (e.g., ACCELEROMETER)
 * - intervalMs : requested report interval in milliseconds
 * - pumpMs   : optional time to pump RX after enabling (default 5 ms)
 * - return   : true if SetFeature frame transmitted
 * - Enable any report via Set Feature
 * - Retries once if response not received
 */
bool BNO08x_7Semi::enableReport(uint8_t reportId, uint32_t intervalMs)
{
  if (!writeSetFeature_(reportId, intervalMs, SHTP_CH_CTRL, 0xFD))
    return false;

  if (!waitForSetFeatureResponse(reportId, 100))
  {
    if (!writeSetFeature_(reportId, intervalMs, SHTP_CH_CTRL, 0xFD))
      return false;
    if (!waitForSetFeatureResponse(reportId, 100))
      return false;
  }

  return true;
}
