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

using namespace std::chrono_literals;

#ifndef BNO_RX_CAP
#define BNO_RX_CAP 300
#endif

enum : uint8_t {
  SHTP_REPORT_PRODUCT_ID_REQUEST  = 0xF9,
  SHTP_REPORT_PRODUCT_ID_RESPONSE = 0xF8,
  SHTP_REPORT_BASE_TIMESTAMP      = 0xFB,
  SHTP_REPORT_TIMESTAMP_REBASE    = 0xFA,
};

enum : size_t {
  SIZEOF_BASE_TIMESTAMP = 5,
  SIZEOF_TIMESTAMP_REBASE = 5,
  SIZEOF_VEC3_REPORT = 10,
  SIZEOF_GAME_ROTATION_VECTOR_REPORT = 12,
  SIZEOF_ROTATION_VECTOR_REPORT = 14,
};

// ============================ Internal helpers ============================

/**
 * - Read a signed 16-bit integer from little-endian bytes
 * - p[0] = LSB, p[1] = MSB
 */
static inline int16_t read2Bytes(const uint8_t *p)
{
  return (int16_t)((p[1] << 8) | p[0]);
}

static inline int sensorReportOffset(const uint8_t *pkt, size_t n)
{
  if (!pkt || n <= 4)
    return -1;

  size_t off = 4; // SHTP header length
  if (pkt[off] == SHTP_REPORT_BASE_TIMESTAMP) {
    off += SIZEOF_BASE_TIMESTAMP;
    if (n <= off)
      return -1;
  }

  return (int)off;
}

static inline size_t sensorReportSize(uint8_t reportId)
{
  switch (reportId) {
  case SHTP_REPORT_TIMESTAMP_REBASE:
    return SIZEOF_TIMESTAMP_REBASE;
  case ACCELEROMETER:
  case GYROSCOPE_CALIBRATED:
  case MAGNETIC_FIELD_CALIBRATED:
  case LINEAR_ACCELERATION:
  case GRAVITY:
    return SIZEOF_VEC3_REPORT;
  case GAME_ROTATION_VECTOR:
    return SIZEOF_GAME_ROTATION_VECTOR_REPORT;
  case ROTATION_VECTOR:
  case GEOMAGNETIC_ROTATION_VECTOR:
    return SIZEOF_ROTATION_VECTOR_REPORT;
  default:
    return 0;
  }
}

static inline bool parseVec3At(
    const uint8_t *pkt,
    size_t n,
    size_t off,
    uint8_t id,
    float scale,
    Vec3 &out)
{
  if (!pkt || n < off + SIZEOF_VEC3_REPORT)
    return false;
  if (pkt[off] != id)
    return false;

  out.x = read2Bytes(&pkt[off + 4]) / scale;
  out.y = read2Bytes(&pkt[off + 6]) / scale;
  out.z = read2Bytes(&pkt[off + 8]) / scale;
  return true;
}

static inline bool parseQuatAt(
    const uint8_t *pkt,
    size_t n,
    size_t off,
    uint8_t id,
    float scale,
    Quat &out)
{
  const size_t reportSize = sensorReportSize(id);
  if (!pkt || reportSize == 0 || n < off + reportSize)
    return false;
  if (pkt[off] != id)
    return false;

  out.i = read2Bytes(&pkt[off + 4]) / scale;
  out.j = read2Bytes(&pkt[off + 6]) / scale;
  out.k = read2Bytes(&pkt[off + 8]) / scale;
  out.r = read2Bytes(&pkt[off + 10]) / scale;
  return true;
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
  const int off = sensorReportOffset(pkt, n);
  return off >= 0 && parseVec3At(pkt, n, (size_t)off, id, scale, out);
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
 *   - report id is checked after the SHTP header and optional timestamp
 *   - i/j/k/r are read from the report payload
 */
static inline bool parseQuatL(
    const uint8_t *pkt,
    size_t n,
    uint8_t id,
    float scale,
    Quat &out)
{
  const int off = sensorReportOffset(pkt, n);
  return off >= 0 && parseQuatAt(pkt, n, (size_t)off, id, scale, out);
}

// =============================== Begin ===============================

/**
 * - Initialize underlying bus (I2C / SPI / UART)
 * - return : true if bus initialized successfully
 * - Bus object must be injected via constructor in .h file
 */
bool BNO08x_7Semi::begin()
{
  if (!bus || !bus->begin())
    return false;

  drainStartupPackets_(500);
  if (!writeProductIdRequest_())
    return false;

  return waitForProductIdResponse_(500);
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
 * - Helper that reads and parses a small batch of pending packets
 * - Call this very frequently in loop() for continuous update
 */
void BNO08x_7Semi::processData()
{
  uint8_t pkt[BNO_RX_CAP];
  for (int i = 0; i < 4; i++) {
    int n = readPacket(pkt, sizeof(pkt));
    if (n <= 0)
      break;
    processPacket(pkt, (size_t)n);
  }
}

// ============================ Packet processor ===========================

/**
 * - Decode one SHTP frame and update cached data
 * - Ignores:
 *   - control channel packets (command responses)
 *   - non-input channels
 * - Updates:
 *   - channel is pkt[2] low nibble
 *   - report id follows the SHTP header and optional timestamp
 */
void BNO08x_7Semi::processPacket(const uint8_t *pkt, size_t n)
{
  if (!pkt || n < 5)
    return; // must have at least header + id

  const uint8_t ch = pkt[2] & 0x0F; // channel = low nibble of pkt[2]
  /** Ignore command/control packets */
  if (ch == SHTP_CH_CTRL)
    return;

  /** Accept only input channels */
  if (ch != SHTP_CH_INPUT && ch != SHTP_CH_WAKE)
    return;

  const uint32_t now = us_ticker_read();
  size_t reportOffset = 4;

  if (pkt[reportOffset] == SHTP_REPORT_BASE_TIMESTAMP) {
    reportOffset += SIZEOF_BASE_TIMESTAMP;
  }

  bool updated = false;
  while (reportOffset < n)
  {
    const uint8_t report_id = pkt[reportOffset];
    const size_t reportSize = sensorReportSize(report_id);
    if (reportSize == 0 || reportOffset + reportSize > n)
      break;

    Vec3 v{};
    Quat q{};
    bool status = false;

    switch (report_id)
    {
    case SHTP_REPORT_TIMESTAMP_REBASE:
      break;

    case ACCELEROMETER:
      status = parseVec3At(pkt, n, reportOffset, ACCELEROMETER, 256.0f, v);
      if (status) {
        data.accel_mps2 = v;
        data.hasAccel = true;
      }
      break;

    case GYROSCOPE_CALIBRATED:
      status = parseVec3At(pkt, n, reportOffset, GYROSCOPE_CALIBRATED, 512.0f, v);
      if (status) {
        data.gyro_rps = v;
        data.hasGyro = true;
      }
      break;

    case MAGNETIC_FIELD_CALIBRATED:
      status = parseVec3At(pkt, n, reportOffset, MAGNETIC_FIELD_CALIBRATED, 16.0f, v);
      if (status) {
        data.mag_uT = v;
        data.hasMag = true;
      }
      break;

    case LINEAR_ACCELERATION:
      status = parseVec3At(pkt, n, reportOffset, LINEAR_ACCELERATION, 256.0f, v);
      if (status) {
        data.linear_mps2 = v;
        data.hasLinear = true;
      }
      break;

    case GRAVITY:
      status = parseVec3At(pkt, n, reportOffset, GRAVITY, 256.0f, v);
      if (status) {
        data.gravity_mps2 = v;
        data.hasGravity = true;
      }
      break;

    case ROTATION_VECTOR:
      status = parseQuatAt(pkt, n, reportOffset, ROTATION_VECTOR, 16384.0f, q);
      if (status) {
        data.rv_q = q;
        data.hasQuat = true;
      }
      break;

    case GAME_ROTATION_VECTOR:
      status = parseQuatAt(pkt, n, reportOffset, GAME_ROTATION_VECTOR, 16384.0f, q);
      if (status) {
        data.grv_q = q;
        data.hasGameQuat = true;
      }
      break;

    case GEOMAGNETIC_ROTATION_VECTOR:
      status = parseQuatAt(pkt, n, reportOffset, GEOMAGNETIC_ROTATION_VECTOR, 16384.0f, q);
      if (status) {
        data.gerv_q = q;
        data.hasGeoQuat = true;
      }
      break;

    default:
      break;
    }

    updated = status || updated;
    reportOffset += reportSize;
  }

  if (updated) {
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
 * -  Scaling uses Q9 => divide by 512.0f
 */
bool BNO08x_7Semi::parseGyroscope(const uint8_t *pkt, size_t n, Vec3 &out) const
{
  return parseVec3L(pkt, n, GYROSCOPE_CALIBRATED, 512.0f, out);
}

/**
 * Parse magnetometer (µT, Q-notation scale)
 * - pkt : SHTP frame buffer
 * - n   : frame length
 * - out : Vec3 output
 * - return : true if parsed successfully
 * - Magnetometer calibrated vector
 * -  Scaling uses Q4 => divide by 16.0f
 */
bool BNO08x_7Semi::parseMagnetometer(const uint8_t *pkt, size_t n, Vec3 &out) const
{
  return parseVec3L(pkt, n, MAGNETIC_FIELD_CALIBRATED, 16.0f, out);
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

bool BNO08x_7Semi::writeProductIdRequest_()
{
  if (!bus)
    return false;

  uint8_t tx[6] = {0};
  const uint16_t L = sizeof(tx);
  tx[0] = L & 0xFF;
  tx[1] = L >> 8;
  tx[2] = SHTP_CH_CTRL;
  tx[3] = seq[SHTP_CH_CTRL]++;
  tx[4] = SHTP_REPORT_PRODUCT_ID_REQUEST;
  tx[5] = 0;

  return bus->tx(tx, sizeof(tx));
}

void BNO08x_7Semi::drainStartupPackets_(uint32_t timeoutMs)
{
  if (!bus)
    return;

  uint8_t buf[BNO_RX_CAP];
  const auto quietTime = std::chrono::milliseconds(50);
  auto deadline = rtos::Kernel::Clock::now() + std::chrono::milliseconds(timeoutMs);
  auto quietDeadline = rtos::Kernel::Clock::now() + quietTime;
  bool sawPacket = false;

  while (rtos::Kernel::Clock::now() < deadline)
  {
    int n = bus->rx(buf, sizeof(buf));
    if (n > 0) {
      sawPacket = true;
      quietDeadline = rtos::Kernel::Clock::now() + quietTime;
    } else {
      if (sawPacket && rtos::Kernel::Clock::now() >= quietDeadline)
        break;
      ThisThread::sleep_for(2ms);
    }
  }
}

bool BNO08x_7Semi::waitForProductIdResponse_(uint32_t timeoutMs)
{
  if (!bus)
    return false;

  uint8_t buf[BNO_RX_CAP];
  auto deadline = rtos::Kernel::Clock::now() + std::chrono::milliseconds(timeoutMs);

  while (rtos::Kernel::Clock::now() < deadline)
  {
    int n = bus->rx(buf, sizeof(buf));
    if (n >= 5)
    {
      const uint8_t channel = buf[2] & 0x0F;
      if (channel == SHTP_CH_CTRL && buf[4] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
        return true;

      processPacket(buf, (size_t)n);
    }
    else
    {
      ThisThread::sleep_for(2ms);
    }
  }

  return false;
}

/**
 * - Wait for a Get Feature Response frame
 * - expects:
 *   - channel == 2 (control)
 *   - reportId == 0xFC (Get Feature Response)
 *   - buf[5] == expectedFeatureId
 */
bool BNO08x_7Semi::waitForSetFeatureResponse(uint8_t expectedFeatureId, uint32_t timeout)
{
  rtos::Kernel::Clock::time_point start = rtos::Kernel::Clock::now();
  uint8_t buf[BNO_RX_CAP];

  while (rtos::Kernel::Clock::now() - start < std::chrono::milliseconds(timeout))
  {
    int n = bus->rx(buf, BNO_RX_CAP);

    if (n >= 6)
    {
      uint8_t channel = buf[2] & 0x0F;
      uint8_t reportId = buf[4];
      if (channel == 2 && reportId == 0xFC)
      {
        uint8_t featureId = buf[5];
        if (featureId == expectedFeatureId)
        {
          return true;
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
 * - return   : true if SetFeature frame transmitted
 * - Enable any report via Set Feature
 */
bool BNO08x_7Semi::enableReport(uint8_t reportId, uint32_t intervalMs)
{
  if (!writeSetFeature_(reportId, intervalMs, SHTP_CH_CTRL, 0xFD))
    return false;

  ThisThread::sleep_for(10ms);
  return true;
}
