/******************************************************************************
 * File    : 7Semi_BNO08x.h
 * Module  : 7Semi_BNO08x — Minimal SH-2/SHTP parser for BNO08x (BNO080/085/086)
 * Version : 0.1.0
 * License : MIT
 *
 * Summary
 * -------
 * - Lightweight driver core focused on decoding SH-2 sensor reports.
 * - Transport is injected (I2C / SPI / UART) via small bus adapters.
 * - Keeps only BNO08x-native IMU + fusion reports (no external env sensors).
 *
 * Notes
 * -----
 * - BNO08x can optionally act as a sensor hub for external sensors (pressure,
 *   humidity, etc.) on AUX I2C depending on firmware.
 * - This minimal version removes those features by default to avoid confusion.
 ******************************************************************************/

#pragma once
#include "BnoSPIBus.h"
#include "BnoSelect.h"
#include "BnoBus.h"

/* ============================ Small value types ============================ */

/**
 * Vec3
 * ----
 * - Generic 3-axis vector container
 * - Units depend on report type (m/s², rad/s, µT, etc.)
 */
struct Vec3 {
  float x = 0, y = 0, z = 0;
};

/**
 * Quat
 * ----
 * - Quaternion container in SH-2 ordering
 * - r = w component (scalar) comes last
 */
struct Quat {
  float i = 0, j = 0, k = 0, r = 1;
};

/* =============================== SHTP channels ============================= */

/**
 * SHTP Channels
 * -------------
 * - BNO08x uses SHTP protocol with multiple logical channels
 * - We mainly use:
 *   - CTRL for Set Feature commands
 *   - INPUT / WAKE for sensor output
 */
enum : uint8_t {
  SHTP_CH_CMD  = 0,
  SHTP_CH_EXEC = 1,
  SHTP_CH_CTRL = 2,
  SHTP_CH_INPUT = 3,
  SHTP_CH_WAKE  = 4,
  SHTP_CH_GYRO_RV = 5
};

/* ================================ Report IDs =============================== */

/**
 * Report IDs supported by this minimal driver
 * -------------------------------------------
 * - Only core IMU + fusion outputs are included
 * - These are always supported on BNO08x standard firmware
 */
enum : uint8_t {
  ACCELEROMETER               = 0x01,
  GYROSCOPE_CALIBRATED        = 0x02,
  MAGNETIC_FIELD_CALIBRATED   = 0x03,
  LINEAR_ACCELERATION         = 0x04,
  ROTATION_VECTOR             = 0x05,
  GRAVITY                     = 0x06,
  GAME_ROTATION_VECTOR        = 0x08,
  GEOMAGNETIC_ROTATION_VECTOR = 0x09,
};

/* ================================ Cached state ============================= */

/**
 * State
 * -----
 * - Holds the most recent decoded sensor values
 * - Each value has a corresponding has* flag
 * - After user reads a value via getter, the flag is cleared
 */
struct State {
  uint32_t t_us = 0;        /* Timestamp in microseconds when last update happened */
  bool dataReady = false;   /* True when at least one report updated the cache */

  Vec3 accel_mps2{};        /* Accelerometer output (m/s²) */
  Vec3 linear_mps2{};       /* Linear acceleration (m/s²) */
  Vec3 gravity_mps2{};      /* Gravity vector (m/s²) */
  Vec3 gyro_rps{};          /* Gyroscope output (rad/s) */
  Vec3 mag_uT{};            /* Magnetometer output (µT) */

  Quat rv_q{};              /* Rotation vector quaternion */
  Quat grv_q{};             /* Game rotation vector quaternion */
  Quat gerv_q{};            /* Geomagnetic rotation vector quaternion */

  bool hasAccel = false;
  bool hasLinear = false;
  bool hasGravity = false;
  bool hasGyro = false;
  bool hasMag = false;
  bool hasQuat = false;
  bool hasGameQuat = false;
  bool hasGeoQuat = false;
};

/* ================================ Driver API ================================ */

/**
 * BNO08x_7Semi
 * ------------
 * - Minimal SH-2/SHTP report parser
 * - Transport (I2C/SPI/UART) is injected using BnoBus abstraction
 *
 * Usage
 * -----
 * - Construct with bus reference
 * - begin() once
 * - enableReport() desired sensors
 * - call processData() frequently in loop()
 * - use get*() functions to fetch decoded values
 */
class BNO08x_7Semi {
public:
  /**
   * Constructor
   * -----------
   * - Bind this driver to a bus object
   * - Bus object must live for the lifetime of this class
   */
  explicit BNO08x_7Semi(BnoBus& bus) : bus(&bus) {}

  /**
   * begin()
   * -------
   * - Initializes the selected transport bus
   * - Does not enable any reports automatically
   */
  bool begin();

  /**
   * enableReport()
   * --------------
   * - Sends SH-2 Set Feature command (0xFD)
   * - Requests periodic output report at intervalMs
   */
  bool enableReport(uint8_t reportId, uint32_t intervalMs);

  /**
   * readPacket()
   * ------------
   * - Reads one full SHTP frame into user buffer (non-blocking)
   * - Returns number of bytes copied
   */
  int readPacket(uint8_t* buffer, size_t bufferLen);

  /**
   * processPacket()
   * ---------------
   * - Parses one SHTP packet
   * - Updates the cached state if packet is a known report
   */
  void processPacket(const uint8_t* pkt, size_t n);

  /**
   * processData()
   * -------------
   * - Convenience wrapper:
   *   - Reads a bounded batch of pending packets
   *   - Decodes each one
   * - Call this as frequently as possible in loop()
   */
  void processData();

  /* -------------------- Cached-state getters -------------------- */

  /** Latest accelerometer vector (m/s²) */
  bool getAccelerometer(float& x, float& y, float& z);

  /** Latest gyroscope vector (rad/s) */
  bool getGyroscope(float& x, float& y, float& z);

  /** Latest magnetometer vector (µT) */
  bool getMagnetometer(float& x, float& y, float& z);

  /** Latest linear acceleration vector (m/s²) */
  bool getLinearAccel(float& x, float& y, float& z);

  /** Latest gravity vector (m/s²) */
  bool getGravity(float& x, float& y, float& z);

  /** Latest rotation quaternion (unitless) */
  bool getQuaternion(float& i, float& j, float& k, float& r);

  /** Latest game rotation quaternion */
  bool getGameRotationVector(float& i, float& j, float& k, float& r);

  /** Latest geomagnetic rotation quaternion */
  bool getGeoRotationVector(float& i, float& j, float& k, float& r);

  /* -------------------- Convenience enable functions -------------------- */

  /** Enable accelerometer report */
  bool enableAcc(uint32_t intervalMs = 10);

  /** Enable gyroscope report */
  bool enableGyro(uint32_t intervalMs = 10);

  /** Enable magnetometer report */
  bool enableMag(uint32_t intervalMs = 10);

  /** Enable rotation vector fusion report */
  bool enableRotationVector(uint32_t intervalMs = 10);

  /** Enable game rotation vector fusion report */
  bool enableGameRotationVector(uint32_t intervalMs = 10);

  /** Enable geomagnetic rotation vector fusion report */
  bool enableGeoRotationVector(uint32_t intervalMs = 10);

  /** Enable linear acceleration report */
  bool enableLinearAccel(uint32_t intervalMs = 10);

  /** Enable gravity vector report */
  bool enableGravity(uint32_t intervalMs = 10);

  /**
   * available()
   * -----------
   * - True once any report has updated the state
   * - Does not clear automatically
   */
  bool available() const { return data.dataReady; }

  /**
   * state()
   * -------
   * - Access raw cached state structure
   * - Read-only, safe for debugging
   */
  const State& state() const { return data; }

private:
  /**
   * writeSetFeature_()
   * -----------------
   * - Builds and transmits SH-2 Set Feature command frame
   * - This requests a specific report at a given interval
   */
  bool writeSetFeature_(uint8_t reportId, uint32_t intervalMs, uint8_t channel, uint8_t command);

  /** Send Product ID Request (0xF9) on the control channel. */
  bool writeProductIdRequest_();

  /** Drain startup advertisement / initialize packets after reset. */
  void drainStartupPackets_(uint32_t timeoutMs);

  /** Wait for Product ID Response (0xF8), proving SHTP communication works. */
  bool waitForProductIdResponse_(uint32_t timeoutMs);

  /**
   * waitForSetFeatureResponse()
   * --------------------------
   * - Waits for control channel Get Feature Response after Set Feature
   * - Returns true when the expected feature id is acknowledged
   */
  bool waitForSetFeatureResponse(uint8_t expectedFeatureId, uint32_t timeout);

  /* ---------------- Raw report parsers ---------------- */

  /**
   * Each parser checks:
   * - Report ID matches expected report
   * - Payload length is valid
   * - Extracts fixed-point values and scales into floats
   */
  bool parseAccelerometer(const uint8_t* pkt, size_t n, Vec3& out_ms2) const;
  bool parseGyroscope(const uint8_t* pkt, size_t n, Vec3& out_rps) const;
  bool parseMagnetometer(const uint8_t* pkt, size_t n, Vec3& out_uT) const;
  bool parseLinearAccel(const uint8_t* pkt, size_t n, Vec3& out_ms2) const;
  bool parseGravity(const uint8_t* pkt, size_t n, Vec3& out_ms2) const;
  bool parseRotationVector(const uint8_t* pkt, size_t n, Quat& out_q) const;
  bool parseGameRotationVector(const uint8_t* pkt, size_t n, Quat& out_q) const;
  bool parseGeoRotationVector(const uint8_t* pkt, size_t n, Quat& out_q) const;

private:
  BnoBus* bus = nullptr;      /* Injected transport implementation */
  State data{};               /* Cached decoded values */
  uint8_t seq[8] = {0};       /* SHTP sequence counters (one per channel) */
};
