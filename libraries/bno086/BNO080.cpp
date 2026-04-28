//
// USC RPL BNO080 driver.
//

/*
 * Overview of BNO080 Communications
 * ===============================================
 *
 * Hilcrest has developed a protocol called SHTP (Sensor Hub Transport Protocol) for binary communications with
 * the BNO080 and the other IMUs it sells.  Over this protocol, SH-2 (Sensor Hub 2) messages are sent to configure
 * the chip and read data back.
 *
 * SHTP messages are divided at two hierarchical levels: first the channel, then the report ID.  Each category
 * of messages (system commands, sensor data reports, etc.) has its own channel, and the individual messages
 * in each channel are identified by their report id, which is the first byte of the message payload (note that the
 * datasheets don't *always* call the first byte the report ID, but that byte does identify the report, so I'm going
 * with it).
 *
 * ===============================================
 *
 * Information about the BNO080 is split into three datasheets.  Here's the download links and what they cover:
 *
 * - the BNO080 datasheet: http://www.hillcrestlabs.com/download/5a05f340566d07c196001ec1
 * -- Chip pinouts
 * -- Example circuits
 * -- Physical specifications
 * -- Supported reports and configuration settings (at a high level)
 * -- List of packets on the SHTP executable channel
 *
 * - the SHTP protocol: http://www.hillcrestlabs.com/download/59de8f99cd829e94dc0029d7
 * -- SHTP transmit and receive protcols (for SPI, I2C, and UART)
 * -- SHTP binary format
 * -- packet types on the SHTP command channel
 *
 * - the SH-2 reference: http://www.hillcrestlabs.com/download/59de8f398934bf6faa00293f
 * -- list of packets and their formats for all channels other than command and executable
 * -- list of FRS (Flash Record System) entries and their formats
 *
 * ===============================================
 *
 * Overview of SHTP channels:
 *
 * 0 -> Command
 * -- Used for protocol-global packets, currently only the advertisement packet (which lists all the channels) and error reports
 *
 * 1 -> Executable
 * -- Used for things that control the software on the chip: commands to reset and sleep
 * -- Also used by the chip to report when it's done booting up
 *
 * 2 -> Control
 * -- Used to send configuration commands to the IMU and for it to send back responses.
 * -- Common report IDs: Command Request (0xF2), Set Feature (0xFD)
 *
 * 3 -> Sensor Reports
 * -- Used for sensors to send back data reports.
 * -- AFAIK the only report ID on this channel will be 0xFB (Report Base Timestamp); sensor data is sent in a series of structures
 *    following an 0xFB
 *
 * 4 -> Wake Sensor Reports
 * -- same as above, but for sensors configured to wake the device
 *
 * 5 -> Gyro Rotation Vector
 * -- a dedicated channel for the Gyro Rotation Vector sensor report
 * -- Why does this get its own channel?  I don't know!!!
 */

#include "BNO080.h"
#include "BNO080Constants.h"
#include <cinttypes>
#include <algorithm>

/// Set to 1 to enable debug printouts.  Should be very useful if the chip is giving you trouble.
/// When debugging, it is recommended to use the highest possible serial baudrate so as not to interrupt the timing of operations.
#define BNO_DEBUG 0

BNO080Base::BNO080Base(Stream *debugPort, PinName user_INTPin, PinName user_RSTPin) :
		_debugPort(debugPort),
		_int(user_INTPin),
		_rst(user_RSTPin, 1),
		commandSequenceNumber(0),
		stability(UNKNOWN),
		stepDetected(false),
		stepCount(0),
		significantMotionDetected(false),
		shakeDetected(false),
		xAxisShake(false),
		yAxisShake(false),
		zAxisShake(false)
{
	// zero sequence numbers
	memset(sequenceNumber, 0, sizeof(sequenceNumber));
}

bool BNO080Base::begin()
{
	//Configure the BNO080

	_rst = 0; // Reset BNO080
	ThisThread::sleep_for(1ms); // Min length not specified in datasheet?
	_rst = 1; // Bring out of reset

	// wait for a falling edge (NOT just a low) on the INT pin to denote startup
	Timer timeoutTimer;
    timeoutTimer.start();                  

	bool highDetected = false;
	bool lowDetected = false;

	while(true)
	{
		if(timeoutTimer.elapsed_time() > BNO080_RESET_TIMEOUT)
		{
			_debugPort->printf("Error: BNO080 reset timed out, chip not detected.\n");
			return false;
		}

		// simple edge detector
		if(!highDetected)
		{
			if(_int == 1)
			{
				highDetected = true;
			}
		}
		else if(!lowDetected)
		{
			if(_int == 0)
			{
				lowDetected = true;
			}
		}
		else
		{
			// high and low detected
			break;
		}
	}

#if BNO_DEBUG
	_debugPort->printf("BNO080 detected!\r\n");
#endif

	// At system startup, the hub must send its full advertisement message (see SHTP 5.2 and 5.3) to the
	// host. It must not send any other data until this step is complete.
	// We don't actually care what's in it, we're just using it as a signal to indicate that the reset is complete.
	receivePacket();

	// now, after startup, the BNO will send an Unsolicited Initialize response (SH-2 section 6.4.5.2), and an Executable Reset command
	if(!waitForPacket(CHANNEL_EXECUTABLE, EXECUTABLE_REPORTID_RESET))
	{
		_debugPort->printf("No initialization report from BNO080.\n");
		return false;
	}
	else
	{
#if BNO_DEBUG
		_debugPort->printf("BNO080 reports initialization successful!\n");
#endif
	}

	// Finally, we want to interrogate the device about its model and version.
	clearSendBuffer();
	txShtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	txShtpData[1] = 0; //Reserved
	sendPacket(CHANNEL_CONTROL, 2);

	waitForPacket(CHANNEL_CONTROL, SHTP_REPORT_PRODUCT_ID_RESPONSE, 5ms);

	if (rxShtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
	{
		majorSoftwareVersion = rxShtpData[2];
		minorSoftwareVersion = rxShtpData[3];
		patchSoftwareVersion = (rxShtpData[13] << 8) | rxShtpData[12];
		partNumber = (rxShtpData[7] << 24) | (rxShtpData[6] << 16) | (rxShtpData[5] << 8) | rxShtpData[4];
		buildNumber = (rxShtpData[11] << 24) | (rxShtpData[10] << 16) | (rxShtpData[9] << 8) | rxShtpData[8];

#if BNO_DEBUG
		_debugPort->printf("BNO080 reports as SW version %hhu.%hhu.%hu, build %lu, part no. %lu\n",
						   majorSoftwareVersion, minorSoftwareVersion, patchSoftwareVersion,
						   buildNumber, partNumber);
#endif

	}
	else
	{
		_debugPort->printf("Bad response from product ID command.\n");
		return false;
	}

	// successful init
	return true;

}

void BNO080Base::tare(bool zOnly)
{
	clearSendBuffer();

 	// from SH-2 section 6.4.4.1
	txShtpData[3] = 0; // perform tare now

	if(zOnly)
	{
		txShtpData[4] = 0b100; // tare Z axis
	}
	else
	{
		txShtpData[4] = 0b111; // tare X, Y, and Z axes
	}

	txShtpData[5] = 0; // reorient all motion outputs

	sendCommand(COMMAND_TARE);
}

bool BNO080Base::enableCalibration(bool calibrateAccel, bool calibrateGyro, bool calibrateMag)
{
	// send the Configure ME Calibration command
	clearSendBuffer();

	txShtpData[3] = static_cast<uint8_t>(calibrateAccel ? 1 : 0);
	txShtpData[4] = static_cast<uint8_t>(calibrateGyro ? 1 : 0);
	txShtpData[5] = static_cast<uint8_t>(calibrateMag ? 1 : 0);

	txShtpData[6] = 0; // Configure ME Calibration command

	txShtpData[7] = 0; // planar accelerometer calibration always disabled

	sendCommand(COMMAND_ME_CALIBRATE);

	// now, wait for the response
	if(!waitForPacket(CHANNEL_CONTROL, SHTP_REPORT_COMMAND_RESPONSE))
	{
#if BNO_DEBUG
		_debugPort->printf("Timeout waiting for calibration response!\n");
#endif
		return false;
	}

	if(rxShtpData[2] != COMMAND_ME_CALIBRATE)
	{
#if BNO_DEBUG
		_debugPort->printf("Received wrong response to calibration command!\n");
#endif
		return false;
	}

	if(rxShtpData[5] != 0)
	{
#if BNO_DEBUG
		_debugPort->printf("IMU reports calibrate command failed!\n");
#endif
		return false;
	}

	// acknowledge checks out!
	return true;
}

bool BNO080Base::saveCalibration()
{
	clearSendBuffer();

	// no arguments
	sendCommand(COMMAND_SAVE_DCD);

	// now, wait for the response
	if(!waitForPacket(CHANNEL_CONTROL, SHTP_REPORT_COMMAND_RESPONSE))
	{
#if BNO_DEBUG
		_debugPort->printf("Timeout waiting for calibration response!\n");
#endif
		return false;
	}

	if(rxShtpData[2] != COMMAND_SAVE_DCD)
	{
#if BNO_DEBUG
		_debugPort->printf("Received wrong response to calibration command!\n");
#endif
		return false;
	}

	if(rxShtpData[5] != 0)
	{
#if BNO_DEBUG
		_debugPort->printf("IMU reports calibrate command failed!\n");
#endif
		return false;
	}

	// acknowledge checks out!
	return true;
}

void BNO080Base::setSensorOrientation(Quaternion orientation)
{
	clearSendBuffer();

	// convert floats to Q
	int16_t Q_x = floatToQ(orientation.x(), ORIENTATION_QUAT_Q_POINT);
	int16_t Q_y = floatToQ(orientation.y(), ORIENTATION_QUAT_Q_POINT);
	int16_t Q_z = floatToQ(orientation.z(), ORIENTATION_QUAT_Q_POINT);
	int16_t Q_w = floatToQ(orientation.w(), ORIENTATION_QUAT_Q_POINT);

	txShtpData[3] = 2; // set reorientation

	txShtpData[4] = static_cast<uint8_t>(Q_x & 0xFF); //P1 - X component LSB
	txShtpData[5] = static_cast<uint8_t>(Q_x >> 8); //P2 - X component MSB

	txShtpData[6] = static_cast<uint8_t>(Q_y & 0xFF); //P3 - Y component LSB
	txShtpData[7] = static_cast<uint8_t>(Q_y >> 8); //P4 - Y component MSB

	txShtpData[8] = static_cast<uint8_t>(Q_z & 0xFF); //P5 - Z component LSB
	txShtpData[9] = static_cast<uint8_t>(Q_z >> 8); //P6 - Z component MSB

	txShtpData[10] = static_cast<uint8_t>(Q_w & 0xFF); //P7 - W component LSB
	txShtpData[11] = static_cast<uint8_t>(Q_w >> 8); //P8 - W component MSB

	//Using this shtpData packet, send a command
	sendCommand(COMMAND_TARE); // Send tare command

	// NOTE: unlike literally every other command, a sensor orientation command is never acknowledged in any way.
}

#define ORIENTATION_RECORD_LEN 4

bool BNO080Base::setPermanentOrientation(Quaternion orientation)
{
	uint32_t orientationRecord[ORIENTATION_RECORD_LEN];

	// each word is one element of the quaternion
	orientationRecord[0] = static_cast<uint32_t>(floatToQ_dword(orientation.x(), FRS_ORIENTATION_Q_POINT));
	orientationRecord[1] = static_cast<uint32_t>(floatToQ_dword(orientation.y(), FRS_ORIENTATION_Q_POINT));
	orientationRecord[2] = static_cast<uint32_t>(floatToQ_dword(orientation.z(), FRS_ORIENTATION_Q_POINT));
	orientationRecord[3] = static_cast<uint32_t>(floatToQ_dword(orientation.w(), FRS_ORIENTATION_Q_POINT));

	return writeFRSRecord(FRS_RECORDID_SYSTEM_ORIENTATION, orientationRecord, ORIENTATION_RECORD_LEN);
}                                                                                                       

bool BNO080Base::updateData()
{
	if(_int.read() != 0)
	{
		// no waiting packets
		return false;
	}

	while(_int.read() == 0)
	{
		if(!receivePacket())
		{
			// comms error
			return false;
		}

		processPacket();

		// Allow time for the IMU to ready another packet if it has one
		wait_us(150); // time between packets measured with a logic analyzer
	}

	// packets were received, so data may have changed
	return true;
}

uint8_t BNO080Base::getReportStatus(Report report)
{
	uint8_t reportNum = static_cast<uint8_t>(report);
	if(reportNum > STATUS_ARRAY_LEN)
	{
		return 0;
	}

	return reportStatus[reportNum];
}

const char* BNO080Base::getReportStatusString(Report report)
{
	switch(getReportStatus(report))
	{
		case 0:
			return "Unreliable";
		case 1:
			return "Accuracy Low";
		case 2:
			return "Accuracy Medium";
		case 3:
			return "Accuracy High";
		default:
			return "Error";
	}
}

bool BNO080Base::hasNewData(Report report)
{
	uint8_t reportNum = static_cast<uint8_t>(report);
	if(reportNum > STATUS_ARRAY_LEN)
	{
		return false;
	}

	bool newData = reportHasBeenUpdated[reportNum];
	reportHasBeenUpdated[reportNum] = false; // clear flag
	return newData;
}

//Sends the packet to enable the rotation vector
void BNO080Base::enableReport(Report report, uint16_t timeBetweenReports)
{
#if BNO_DEBUG
	// check time is valid
	float periodSeconds = static_cast<float>(timeBetweenReports / 1000.0);

	if(periodSeconds < getMinPeriod(report))
	{
		_debugPort->printf("Error: attempt made to set report 0x%02hhx to period of %.06f s, which is smaller than its min period of %.06f s.\r\n",
						   static_cast<uint8_t>(report), periodSeconds, getMinPeriod(report));
		return;
	}

#endif
	setFeatureCommand(static_cast<uint8_t>(report), timeBetweenReports);

	// note: we don't wait for ACKs on these packets because they can take quite a while, like half a second, to come in
}

void BNO080Base::disableReport(Report report)
{
	// set the report's polling period to zero to disable it
	setFeatureCommand(static_cast<uint8_t>(report), 0);
}

uint32_t BNO080Base::getSerialNumber()
{
	uint32_t serNoBuffer;

	if(!readFRSRecord(FRS_RECORDID_SERIAL_NUMBER, &serNoBuffer, 1))
	{
		return 0;
	}

	return serNoBuffer;
}

float BNO080Base::getRange(Report report)
{
	loadReportMetadata(report);

	return qToFloat_dword(metadataRecord[1], getQ1(report));
}


float BNO080Base::getResolution(Report report)
{
	loadReportMetadata(report);

	return qToFloat_dword(metadataRecord[2], getQ1(report));
}

float BNO080Base::getPower(Report report)
{
	loadReportMetadata(report);

	uint16_t powerQ = static_cast<uint16_t>(metadataRecord[3] & 0xFFFF);

	return qToFloat_dword(powerQ, POWER_Q_POINT);
}

float BNO080Base::getMinPeriod(Report report)
{
	loadReportMetadata(report);

	return metadataRecord[4] / 1e6f; // convert from microseconds to seconds
}

float BNO080Base::getMaxPeriod(Report report)
{
	loadReportMetadata(report);

	if(getMetaVersion() == 3)
	{
		// no max period entry in this record format
		return -1.0f;
	}

	return metadataRecord[9] / 1e6f; // convert from microseconds to seconds
}

void BNO080Base::printMetadataSummary(Report report)
{
#if BNO_DEBUG
	if(!loadReportMetadata(report))
	{
		_debugPort->printf("Failed to load report metadata!\n");
	}

	_debugPort->printf("======= Metadata for report 0x%02hhx =======\n", static_cast<uint8_t>(report));

	_debugPort->printf("Range: +- %.04f units\n", getRange(report));
	_debugPort->printf("Resolution: %.04f units\n", getResolution(report));
	_debugPort->printf("Power Used: %.03f mA\n", getPower(report));
	_debugPort->printf("Min Period: %.06f s\n", getMinPeriod(report));
	_debugPort->printf("Max Period: %.06f s\n\n", getMaxPeriod(report));

#endif
}

int16_t BNO080Base::getQ1(Report report)
{
	loadReportMetadata(report);

	return static_cast<int16_t>(metadataRecord[7] & 0xFFFF);
}

int16_t BNO080Base::getQ2(Report report)
{
	loadReportMetadata(report);

	return static_cast<int16_t>(metadataRecord[7] >> 16);
}

int16_t BNO080Base::getQ3(Report report)
{
	loadReportMetadata(report);

	return static_cast<int16_t>(metadataRecord[8] >> 16);
}

void BNO080Base::processPacket()
{
	if(rxShtpHeader[2] == CHANNEL_CONTROL)
	{
		// currently no command reports are read
	}
	else if(rxShtpHeader[2] == CHANNEL_EXECUTABLE)
	{
		// currently no executable reports are read
	}
	else if(rxShtpHeader[2] == CHANNEL_COMMAND)
	{

	}
	else if(rxShtpHeader[2] == CHANNEL_REPORTS || rxShtpHeader[2] == CHANNEL_WAKE_REPORTS)
	{
		if(rxShtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
		{
			// sensor data packet
			parseSensorDataPacket();
		}
	}
}

// sizes of various sensor data packet elements
#define SIZEOF_BASE_TIMESTAMP 5
#define SIZEOF_TIMESTAMP_REBASE 5
#define SIZEOF_ACCELEROMETER 10
#define SIZEOF_LINEAR_ACCELERATION 10
#define SIZEOF_GYROSCOPE_CALIBRATED 10
#define SIZEOF_MAGNETIC_FIELD_CALIBRATED 10
#define SIZEOF_MAGNETIC_FIELD_UNCALIBRATED 16
#define SIZEOF_ROTATION_VECTOR 14
#define SIZEOF_GAME_ROTATION_VECTOR 12
#define SIZEOF_GEOMAGNETIC_ROTATION_VECTOR 14
#define SIZEOF_TAP_DETECTOR 5
#define SIZEOF_STABILITY_REPORT 6
#define SIZEOF_STEP_DETECTOR 8
#define SIZEOF_STEP_COUNTER 12
#define SIZEOF_SIGNIFICANT_MOTION 6
#define SIZEOF_SHAKE_DETECTOR 6

void BNO080Base::parseSensorDataPacket()
{
	size_t currReportOffset = 0;

	// every sensor data report first contains a timestamp offset to show how long it has been between when
	// the host interrupt was sent and when the packet was transmitted.
	// We don't use interrupts and don't care about times, so we can throw this out.
	currReportOffset += SIZEOF_BASE_TIMESTAMP;

	while(currReportOffset < rxPacketLength)
	{
		// lots of sensor reports use 3 16-bit numbers stored in bytes 4 through 9
		// we can save some time by parsing those out here.
		uint16_t data1 = (uint16_t)rxShtpData[currReportOffset + 5] << 8 | rxShtpData[currReportOffset + 4];
		uint16_t data2 = (uint16_t)rxShtpData[currReportOffset + 7] << 8 | rxShtpData[currReportOffset + 6];
		uint16_t data3 = (uint16_t)rxShtpData[currReportOffset + 9] << 8 | rxShtpData[currReportOffset + 8];

		uint8_t reportNum = rxShtpData[currReportOffset];

		if(reportNum != SENSOR_REPORTID_TIMESTAMP_REBASE)
		{
			// set status from byte 2
			reportStatus[reportNum] = static_cast<uint8_t>(rxShtpData[currReportOffset + 2] & 0b11);

			// set updated flag
			reportHasBeenUpdated[reportNum] = true;
		}

		switch(rxShtpData[currReportOffset])
		{
			case SENSOR_REPORTID_TIMESTAMP_REBASE:
				currReportOffset += SIZEOF_TIMESTAMP_REBASE;
				break;

			case SENSOR_REPORTID_ACCELEROMETER:

				totalAcceleration = TVector3(
						qToFloat(data1, ACCELEROMETER_Q_POINT),
						qToFloat(data2, ACCELEROMETER_Q_POINT),
				 		qToFloat(data3, ACCELEROMETER_Q_POINT));

				currReportOffset += SIZEOF_ACCELEROMETER;
				break;

			case SENSOR_REPORTID_LINEAR_ACCELERATION:

				linearAcceleration = TVector3(
						qToFloat(data1, ACCELEROMETER_Q_POINT),
						qToFloat(data2, ACCELEROMETER_Q_POINT),
						qToFloat(data3, ACCELEROMETER_Q_POINT));

				currReportOffset += SIZEOF_LINEAR_ACCELERATION;
				break;

			case SENSOR_REPORTID_GRAVITY:

				gravityAcceleration = TVector3(
						qToFloat(data1, ACCELEROMETER_Q_POINT),
						qToFloat(data2, ACCELEROMETER_Q_POINT),
						qToFloat(data3, ACCELEROMETER_Q_POINT));

				currReportOffset += SIZEOF_LINEAR_ACCELERATION;
				break;

			case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:

				gyroRotation = TVector3(
						qToFloat(data1, GYRO_Q_POINT),
						qToFloat(data2, GYRO_Q_POINT),
						qToFloat(data3, GYRO_Q_POINT));

				currReportOffset += SIZEOF_GYROSCOPE_CALIBRATED;
				break;

			case SENSOR_REPORTID_MAGNETIC_FIELD_CALIBRATED:

				magField = TVector3(
						qToFloat(data1, MAGNETOMETER_Q_POINT),
						qToFloat(data2, MAGNETOMETER_Q_POINT),
						qToFloat(data3, MAGNETOMETER_Q_POINT));

				currReportOffset += SIZEOF_MAGNETIC_FIELD_CALIBRATED;
				break;

			case SENSOR_REPORTID_MAGNETIC_FIELD_UNCALIBRATED:
			{
				magFieldUncalibrated = TVector3(
						qToFloat(data1, MAGNETOMETER_Q_POINT),
						qToFloat(data2, MAGNETOMETER_Q_POINT),
						qToFloat(data3, MAGNETOMETER_Q_POINT));

				uint16_t ironOffsetXQ = (uint16_t) rxShtpData[currReportOffset + 11] << 8 | rxShtpData[currReportOffset + 10];
				uint16_t ironOffsetYQ = (uint16_t) rxShtpData[currReportOffset + 13] << 8 | rxShtpData[currReportOffset + 12];
				uint16_t ironOffsetZQ = (uint16_t) rxShtpData[currReportOffset + 15] << 8 | rxShtpData[currReportOffset + 14];

				hardIronOffset = TVector3(
						qToFloat(ironOffsetXQ, MAGNETOMETER_Q_POINT),
					  	qToFloat(ironOffsetYQ, MAGNETOMETER_Q_POINT),
	  					qToFloat(ironOffsetZQ, MAGNETOMETER_Q_POINT));

				currReportOffset += SIZEOF_MAGNETIC_FIELD_UNCALIBRATED;
			}
				break;

			case SENSOR_REPORTID_ROTATION_VECTOR:
			{
				uint16_t realPartQ = (uint16_t) rxShtpData[currReportOffset + 11] << 8 | rxShtpData[currReportOffset + 10];
				uint16_t accuracyQ = (uint16_t) rxShtpData[currReportOffset + 13] << 8 | rxShtpData[currReportOffset + 12];

				rotationVector = TVector4(
						qToFloat(data1, ROTATION_Q_POINT),
						qToFloat(data2, ROTATION_Q_POINT),
						qToFloat(data3, ROTATION_Q_POINT),
						qToFloat(realPartQ, ROTATION_Q_POINT));

				rotationAccuracy = qToFloat(accuracyQ, ROTATION_ACCURACY_Q_POINT);

				currReportOffset += SIZEOF_ROTATION_VECTOR;
			}
				break;

			case SENSOR_REPORTID_GAME_ROTATION_VECTOR:
			{
				uint16_t realPartQ = (uint16_t) rxShtpData[currReportOffset + 11] << 8 | rxShtpData[currReportOffset + 10];

				gameRotationVector = TVector4(
						qToFloat(data1, ROTATION_Q_POINT),
						qToFloat(data2, ROTATION_Q_POINT),
						qToFloat(data3, ROTATION_Q_POINT),
						qToFloat(realPartQ, ROTATION_Q_POINT));

				currReportOffset += SIZEOF_GAME_ROTATION_VECTOR;
			}
				break;

			case SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR:
			{
				uint16_t realPartQ = (uint16_t) rxShtpData[currReportOffset + 11] << 8 | rxShtpData[currReportOffset + 10];
				uint16_t accuracyQ = (uint16_t) rxShtpData[currReportOffset + 13] << 8 | rxShtpData[currReportOffset + 12];

				geomagneticRotationVector = TVector4(
						qToFloat(data1, ROTATION_Q_POINT),
						qToFloat(data2, ROTATION_Q_POINT),
						qToFloat(data3, ROTATION_Q_POINT),
						qToFloat(realPartQ, ROTATION_Q_POINT));

				geomagneticRotationAccuracy = qToFloat(accuracyQ, ROTATION_ACCURACY_Q_POINT);

				currReportOffset += SIZEOF_GEOMAGNETIC_ROTATION_VECTOR;
			}
				break;

			case SENSOR_REPORTID_TAP_DETECTOR:

				// since we got the report, a tap was detected
				tapDetected = true;

				doubleTap = (rxShtpData[currReportOffset + 4] & (1 << 6)) != 0;

				currReportOffset += SIZEOF_TAP_DETECTOR;
				break;

			case SENSOR_REPORTID_STABILITY_CLASSIFIER:
			{
				uint8_t classificationNumber = rxShtpData[currReportOffset + 4];

				if(classificationNumber > 4)
				{
					classificationNumber = 0;
				}

				stability = static_cast<Stability>(classificationNumber);

				currReportOffset += SIZEOF_STABILITY_REPORT;
			}
				break;

			case SENSOR_REPORTID_STEP_DETECTOR:

				// the fact that we got the report means that a step was detected
				stepDetected = true;

				currReportOffset += SIZEOF_STEP_DETECTOR;

				break;

			case SENSOR_REPORTID_STEP_COUNTER:

				stepCount = rxShtpData[currReportOffset + 9] << 8 | rxShtpData[currReportOffset + 8];

				currReportOffset += SIZEOF_STEP_COUNTER;

				break;

			case SENSOR_REPORTID_SIGNIFICANT_MOTION:

				// the fact that we got the report means that significant motion was detected
				significantMotionDetected = true;

				currReportOffset += SIZEOF_SIGNIFICANT_MOTION;
				
				break;

			case SENSOR_REPORTID_SHAKE_DETECTOR:

				shakeDetected = true;

				xAxisShake = (rxShtpData[currReportOffset + 4] & 1) != 0;
				yAxisShake = (rxShtpData[currReportOffset + 4] & (1 << 1)) != 0;
				zAxisShake = (rxShtpData[currReportOffset + 4] & (1 << 2)) != 0;

				currReportOffset += SIZEOF_SHAKE_DETECTOR;

				break;
				
			default:
				_debugPort->printf("Error: unrecognized report ID in sensor report: %hhx.  Byte %u, length %hu\n", rxShtpData[currReportOffset], currReportOffset, rxPacketLength);
				return;
		}

		if(currReportOffset >= SHTP_RX_PACKET_SIZE)
		{
			_debugPort->printf("Error: sensor report longer than packet buffer! Some data was not read! Increase buffer size or decrease number of reports!\r\n");
			return;
		}
	}

}

bool BNO080Base::waitForPacket(int channel, uint8_t reportID, std::chrono::milliseconds timeout)
{
	Timer timeoutTimer;
	timeoutTimer.start();

	while(timeoutTimer.elapsed_time() <= timeout)
	{
		if(_int.read() == 0)
		{
			if(!receivePacket(timeout))
			{
				return false;
			}

			if(channel == rxShtpHeader[2] && reportID == rxShtpData[0])
			{
				// found correct packet!
				return true;
			}
			else
			{
				// other data packet, send to proper channels
				processPacket();
			}
		}
	}

	_debugPort->printf("Packet wait timeout.\n");
	return false;
}

//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float BNO080Base::qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
	float qFloat = fixedPointValue;
	qFloat *= pow(2.0f, qPoint * -1);
	return (qFloat);
}

float BNO080Base::qToFloat_dword(uint32_t fixedPointValue, int16_t qPoint)
{
	float qFloat = fixedPointValue;
	qFloat *= pow(2.0f, qPoint * -1);
	return (qFloat);
}

//Given a floating point value and a Q point, convert to Q
//See https://en.wikipedia.org/wiki/Q_(number_format)
int16_t BNO080Base::floatToQ(float qFloat, uint8_t qPoint)
{
	int16_t qVal = static_cast<int16_t>(qFloat * pow(2.0f, qPoint));
	return qVal;
}

int32_t BNO080Base::floatToQ_dword(float qFloat, uint16_t qPoint)
{
	int32_t qVal = static_cast<int32_t>(qFloat * pow(2.0f, qPoint));
	return qVal;
}
//Tell the sensor to do a command
//See 6.3.8 page 41, Command request
//The caller is expected to set P0 through P8 prior to calling
void BNO080Base::sendCommand(uint8_t command)
{
	txShtpData[0] = SHTP_REPORT_COMMAND_REQUEST; //Command Request
	txShtpData[1] = commandSequenceNumber++; //Increments automatically each function call
	txShtpData[2] = command; //Command

	//Caller must set these
	/*shtpData[3] = 0; //P0
		shtpData[4] = 0; //P1
		shtpData[5] = 0; //P2
		shtpData[6] = 0;
		shtpData[7] = 0;
		shtpData[8] = 0;
		shtpData[9] = 0;
		shtpData[10] = 0;
		shtpData[11] = 0;*/

	//Transmit packet on channel 2, 12 bytes
	sendPacket(CHANNEL_CONTROL, 12);
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier
void BNO080Base::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig)
{
	uint32_t microsBetweenReports = static_cast<uint32_t>(timeBetweenReports * 1000);

	const uint32_t batchMicros = 0;

	txShtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND; //Set feature command. Reference page 55
	txShtpData[1] = reportID; //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
	txShtpData[2] = 0; //Feature flags
	txShtpData[3] = 0; //Change sensitivity (LSB)
	txShtpData[4] = 0; //Change sensitivity (MSB)
	txShtpData[5] = (microsBetweenReports >> 0) & 0xFF; //Report interval (LSB) in microseconds. 0x7A120 = 500ms
	txShtpData[6] = (microsBetweenReports >> 8) & 0xFF; //Report interval
	txShtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
	txShtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
	txShtpData[9] = (batchMicros >> 0) & 0xFF;  //Batch Interval (LSB)
	txShtpData[10] = (batchMicros >> 8) & 0xFF; //Batch Interval
	txShtpData[11] = (batchMicros >> 16) & 0xFF;//Batch Interval
	txShtpData[12] = (batchMicros >> 24) & 0xFF;//Batch Interval (MSB)
	txShtpData[13] = (specificConfig >> 0) & 0xFF; //Sensor-specific config (LSB)
	txShtpData[14] = (specificConfig >> 8) & 0xFF; //Sensor-specific config
	txShtpData[15] = (specificConfig >> 16) & 0xFF; //Sensor-specific config
	txShtpData[16] = (specificConfig >> 24) & 0xFF; //Sensor-specific config (MSB)

	//Transmit packet on channel 2, 17 bytes
	sendPacket(CHANNEL_CONTROL, 17);
}

bool BNO080Base::readFRSRecord(uint16_t recordID, uint32_t* readBuffer, uint16_t readLength)
{
	// send initial read request
	clearSendBuffer();

	txShtpData[0] = SHTP_REPORT_FRS_READ_REQUEST;
	// read offset of 0 -> start at the start of the record
	txShtpData[2] = 0;
	txShtpData[3] = 0;
	// record ID
	txShtpData[4] = static_cast<uint8_t>(recordID & 0xFF);
	txShtpData[5] = static_cast<uint8_t>(recordID >> 8);
	// block size
	txShtpData[6] = static_cast<uint8_t>(readLength & 0xFF);
	txShtpData[7] = static_cast<uint8_t>(readLength >> 8);

	sendPacket(CHANNEL_CONTROL, 8);

	// now, read back the responses
	size_t readOffset = 0;
	while(readOffset < readLength)
	{
		// it seems like it can take quite a long time for FRS data to be read, so we have to increase the timeout
		if(!waitForPacket(CHANNEL_CONTROL, SHTP_REPORT_FRS_READ_RESPONSE, 300ms))
		{
#if BNO_DEBUG
			_debugPort->printf("Error: did not receive FRS read response after sending read request!\n");
#endif
			return false;
		}

		uint8_t status = static_cast<uint8_t>(rxShtpData[1] & 0b1111);
		uint8_t dataLength = rxShtpData[1] >> 4;

		// check status
		if(status == 1)
		{
#if BNO_DEBUG
			_debugPort->printf("Error: FRS reports invalid record ID!\n");
#endif
			return false;
		}
		else if(status == 2)
		{
#if BNO_DEBUG
			_debugPort->printf("Error: FRS is busy!\n");
#endif
			return false;
		}
		else if(status == 4)
		{
#if BNO_DEBUG
			_debugPort->printf("Error: FRS reports offset is out of range!\n");
#endif
			return false;
		}
		else if(status == 5)
		{
#if BNO_DEBUG
			_debugPort->printf("Error: FRS reports record %hx is empty!\n", recordID);
#endif
			return false;
		}
		else if(status == 8)
		{
#if BNO_DEBUG
			_debugPort->printf("Error: FRS reports flash memory device unavailable!\n");
#endif
			return false;
		}

		// check data length
		if(dataLength == 0)
		{
#if BNO_DEBUG
			_debugPort->printf("Error: Received FRS packet with 0 data length!\n");
#endif
			return false;
		}
		else if(dataLength == 1)
		{
			if(readOffset + 1 != readLength)
			{
#if BNO_DEBUG
				_debugPort->printf("Error: Received 1 length packet but more than 1 byte remains to be be read!\n");
#endif
				return false;
			}
		}

		// now, _finally_, read the dang words
		readBuffer[readOffset] = (rxShtpData[7] << 24) | (rxShtpData[6] << 16) | (rxShtpData[5] << 8) | (rxShtpData[4]);

		// check if we only wanted the first word
		++readOffset;
		if(readOffset == readLength)
		{
			break;
		}

		readBuffer[readOffset] = (rxShtpData[11] << 24) | (rxShtpData[10] << 16) | (rxShtpData[9] << 8) | (rxShtpData[8]);
		readOffset++;
	}

	// read successful
	return true;

}

bool BNO080Base::writeFRSRecord(uint16_t recordID, uint32_t* buffer, uint16_t length)
{
	// send initial write request, which tells the chip where we're writing
	clearSendBuffer();

	txShtpData[0] = SHTP_REPORT_FRS_WRITE_REQUEST;
	// length to write (must be <= record length)
	txShtpData[2] = static_cast<uint8_t>(length & 0xFF);
	txShtpData[3] = static_cast<uint8_t>(length >> 8);
	// record ID
	txShtpData[4] = static_cast<uint8_t>(recordID & 0xFF);
	txShtpData[5] = static_cast<uint8_t>(recordID >> 8);

	sendPacket(CHANNEL_CONTROL, 6);

	// wait for FRS to become ready
	if(!waitForPacket(CHANNEL_CONTROL, SHTP_REPORT_FRS_WRITE_RESPONSE, 300ms))
	{
#if BNO_DEBUG
		_debugPort->printf("Error: did not receive FRS write ready response after sending write request!\r\n");
#endif
		return false;
	}

	if(rxShtpData[1] != 4)
	{
#if BNO_DEBUG
		_debugPort->printf("Error: FRS reports error initiating write operation: %hhu!\r\n", rxShtpData[1]);
#endif
		return false;
	}

	// now, send the actual data
	for(uint16_t wordIndex = 0; wordIndex < length; wordIndex += 2)
	{
		// send packet containing 2 words
		clearSendBuffer();
		txShtpData[0] = SHTP_REPORT_FRS_WRITE_DATA;

		// offset to write at
		txShtpData[2] = static_cast<uint8_t>(wordIndex & 0xFF);
		txShtpData[3] = static_cast<uint8_t>(wordIndex >> 8);

		// data 0
		*reinterpret_cast<uint32_t*>(txShtpData + 4) = buffer[wordIndex];

		// data 1, if it exists
		if(wordIndex != length - 1)
		{
			*reinterpret_cast<uint32_t*>(txShtpData + 8) = buffer[wordIndex + 1];
		}

		sendPacket(CHANNEL_CONTROL, 12);

		// wait for acknowledge
		if(!waitForPacket(CHANNEL_CONTROL, SHTP_REPORT_FRS_WRITE_RESPONSE, 300ms))
		{
#if BNO_DEBUG
			_debugPort->printf("Error: did not receive FRS write response after sending write data!\r\n");
#endif
			return false;
		}

		uint8_t status = rxShtpData[1];
		
		switch(status)
		{
			case 0:
				if(length - wordIndex >= 2)
				{
					// status OK, write still in progress
				}
				else
				{
#if BNO_DEBUG
					_debugPort->printf("Error: FRS reports write in progress when it should be complete!\r\n");
#endif
					return false;
				}
				break;
			case 3:
			case 8:
				if(length - wordIndex <= 2)
				{
					// status OK, write complete
				}
				else
				{
#if BNO_DEBUG
					_debugPort->printf("Error: FRS reports write complete when it should be still going!\n");
#endif
					return false;
				}
				break;
			case 1:
#if BNO_DEBUG
				_debugPort->printf("Error: FRS reports invalid record ID!\n");
#endif
				return false;
			case 2:
#if BNO_DEBUG
				_debugPort->printf("Error: FRS is busy!\n");
#endif
				return false;
			case 5:
#if BNO_DEBUG
				_debugPort->printf("Error: FRS reports write failed!\n");
#endif
				return false;
			case 6:
#if BNO_DEBUG
				_debugPort->printf("Error: FRS reports data received while not in write mode!\n");
#endif
				return false;
			case 7:
#if BNO_DEBUG
				_debugPort->printf("Error: FRS reports invalid length!\n");
#endif
				return false;
			case 9:
#if BNO_DEBUG
				_debugPort->printf("Error: FRS reports invalid data for this record!\n");
#endif
				return false;

			case 10:
#if BNO_DEBUG
				_debugPort->printf("Error: FRS reports flash device unavailable!\n");
#endif
				return false;

			case 11:
#if BNO_DEBUG
				_debugPort->printf("Error: FRS reports record is read-only!\n");
#endif
				return false;
			default:
#if BNO_DEBUG
				_debugPort->printf("Error: FRS reports unknown result code %hhu!\n", status);
#endif
				break;

		}
	}

	// write complete
	return true; 
}

//Pretty prints the contents of a packet in the given buffer
void BNO080Base::printPacket(uint8_t * buffer)
{
#if BNO_DEBUG
	//Print the four byte header
	_debugPort->printf("Header:");
	for (uint8_t x = 0 ; x < 4 ; x++)
	{
		_debugPort->printf(" ");
		_debugPort->printf("%02hhx", buffer[x]);
	}

	//Calculate the number of data bytes in this packet
	uint16_t totalLength = (static_cast<uint16_t>(buffer[1]) << 8 | buffer[0]);
	totalLength &= ~(1 << 15);
	uint16_t packetLength = totalLength - 4; //Remove the header bytes from the data count

	uint16_t printLength = std::min(packetLength, static_cast<uint16_t>(40)); //Artificial limit. We don't want the phone book.

	_debugPort->printf(" Body:");
	for (uint16_t x = 0 ; x < printLength ; x++)
	{
		_debugPort->printf(" ");
		_debugPort->printf("%02hhx", buffer[x + SHTP_HEADER_SIZE]);
	}

	_debugPort->printf(", Length:");
	_debugPort->printf("%d", packetLength + SHTP_HEADER_SIZE);
	_debugPort->printf(", SeqNum: %hhu", buffer[3]);

	_debugPort->printf(", Channel:");
	if (buffer[2] == 0) _debugPort->printf("Command");
	else if (buffer[2] == 1) _debugPort->printf("Executable");
	else if (buffer[2] == 2) _debugPort->printf("Control");
	else if (buffer[2] == 3) _debugPort->printf("Sensor-report");
	else if (buffer[2] == 4) _debugPort->printf("Wake-report");
	else if (buffer[2] == 5) _debugPort->printf("Gyro-vector");
	else _debugPort->printf("%hhu", buffer[2]);

	_debugPort->printf("\n");
#endif
}


void BNO080Base::clearSendBuffer()
{
	memset(txPacketBuffer, 0, SHTP_HEADER_SIZE + SHTP_MAX_TX_PACKET_SIZE);
}

bool BNO080Base::loadReportMetadata(BNO080Base::Report report)
{
	uint16_t reportMetaRecord = 0;

	// first, convert the report into the correct FRS record ID for that report's metadata
	// data from SH-2 section 5.1
	switch(report)
	{
		case TOTAL_ACCELERATION:
			reportMetaRecord = 0xE301;
			break;
		case LINEAR_ACCELERATION:
			reportMetaRecord = 0xE303;
			break;
		case GRAVITY_ACCELERATION:
			reportMetaRecord = 0xE304;
			break;
		case GYROSCOPE:
			reportMetaRecord = 0xE306;
			break;
		case MAG_FIELD:
			reportMetaRecord = 0xE309;
			break;
		case MAG_FIELD_UNCALIBRATED:
			reportMetaRecord = 0xE30A;
			break;
		case ROTATION:
			reportMetaRecord = 0xE30B;
			break;
		case GEOMAGNETIC_ROTATION:
			reportMetaRecord = 0xE30D;
			break;
		case GAME_ROTATION:
			reportMetaRecord = 0xE30C;
			break;
		case TAP_DETECTOR:
			reportMetaRecord = 0xE313;
			break;
		case STABILITY_CLASSIFIER:
			reportMetaRecord = 0xE317;
			break;
		case STEP_DETECTOR:
			reportMetaRecord = 0xE314;
			break;
		case STEP_COUNTER:
			reportMetaRecord = 0xE315;
			break;
		case SIGNIFICANT_MOTION:
			reportMetaRecord = 0xE316;
			break;
		case SHAKE_DETECTOR:
			reportMetaRecord = 0xE318;
			break;
	}

	// if we already have that data stored, everything's OK
	if(bufferMetadataRecord == reportMetaRecord)
	{
		return true;
	}

	// now, load the metadata into the buffer
	if(!readFRSRecord(reportMetaRecord, metadataRecord, METADATA_BUFFER_LEN))
	{
		// clear this so future calls won't try to use the cached version
		bufferMetadataRecord = 0;

		return false;
	}

	bufferMetadataRecord = reportMetaRecord;

	return true;
}

BNO080I2C::BNO080I2C(Stream *debugPort, PinName user_SDApin, PinName user_SCLpin, PinName user_INTPin, PinName user_RSTPin,
					   uint8_t i2cAddress, int i2cPortSpeed) :
	   BNO080Base(debugPort, user_INTPin, user_RSTPin),
		_i2cPort(user_SDApin, user_SCLpin),
		_i2cAddress(i2cAddress)
{

	//Get user settings
	_i2cPortSpeed = i2cPortSpeed;
	if(_i2cPortSpeed > 4000000)
	{
		_i2cPortSpeed = 4000000; //BNO080 max is 400Khz
	}
	_i2cPort.frequency(_i2cPortSpeed);
}


//Given the data packet, send the header then the data
//Returns false if sensor does not ACK
bool BNO080I2C::sendPacket(uint8_t channelNumber, uint8_t dataLength)
{

	uint16_t totalLength = dataLength + 4; //Add four bytes for the header

	txShtpHeader[0] = totalLength & 0xFF;
	txShtpHeader[1] = totalLength >> 8;
	txShtpHeader[2] = channelNumber;
	txShtpHeader[3] = sequenceNumber[channelNumber];

#if BNO_DEBUG
	_debugPort->printf("Transmitting packet: ----------------\n");
	printPacket(txPacketBuffer);
#endif

	// send packet to IMU
	int writeResult = _i2cPort.write(_i2cAddress << 1, reinterpret_cast<char*>(txPacketBuffer), totalLength);

	if(writeResult != 0)
	{
		_debugPort->printf("BNO I2C write failed!\n");
		return false;
	}
	return true;
}

//Check to see if there is any new data available
//Read the contents of the incoming packet into the shtpData array
bool BNO080I2C::receivePacket(std::chrono::milliseconds timeout)
{
	Timer waitStartTime;
	waitStartTime.start();

	while(_int.read() != 0)
	{
		if(waitStartTime.elapsed_time() > timeout)
		{
			_debugPort->printf("BNO I2C header wait timeout\n");
			return false;
		}

	}

	// start the transaction and contact the IMU
	_i2cPort.start();

	// to indicate an i2c read, shift the 7 bit address up 1 bit and set bit 0 to a 1
	int writeResult = _i2cPort.write((_i2cAddress << 1) | 0x1);

	if(writeResult != 1)
	{
		_debugPort->printf("BNO I2C read failed!\n");
		return false;
	}

	//Get the first four bytes, aka the packet header
	uint8_t packetLSB = static_cast<uint8_t>(_i2cPort.read(true));
	uint8_t packetMSB = static_cast<uint8_t>(_i2cPort.read(true));
	uint8_t channelNumber = static_cast<uint8_t>(_i2cPort.read(true));
	uint8_t sequenceNum = static_cast<uint8_t>(_i2cPort.read(true)); //Not sure if we need to store this or not

	//Store the header info
	rxShtpHeader[0] = packetLSB;
	rxShtpHeader[1] = packetMSB;
	rxShtpHeader[2] = channelNumber;
	rxShtpHeader[3] = sequenceNum;

	if(rxShtpHeader[0] == 0xFF && rxShtpHeader[1] == 0xFF)
	{
		// invalid according to BNO080 datasheet section 1.4.1

#if BNO_DEBUG
		_debugPort->printf("Recieved 0xFFFF packet length, protocol error!\n");
#endif
		return false;
	}

	//Calculate the number of data bytes in this packet
	rxPacketLength = (static_cast<uint16_t>(packetMSB) << 8 | packetLSB);

	// Clear the MSbit.
	// This bit indicates if this package is a continuation of the last. TBH, I don't really know what this means (it's not really explained in the datasheet)
	// but we don't actually care about any of the advertisement packets
	// that use this, so we can just cut off the rest of the packet by releasing chip select.
	rxPacketLength &= ~(1 << 15);

	if (rxPacketLength == 0)
	{
		// Packet is empty
		return (false); //All done
	}

	rxPacketLength -= 4; //Remove the header bytes from the data count

	//Read incoming data into the shtpData array
	for (uint16_t dataSpot = 0 ; dataSpot < rxPacketLength ; dataSpot++)
	{
		bool sendACK = dataSpot < rxPacketLength - 1;

		// per the datasheet, 0xFF is used as filler for the receiver to transmit back
		uint8_t incoming = static_cast<uint8_t>(_i2cPort.read(sendACK));
		if (dataSpot < (sizeof(rxPacketBuffer) - SHTP_HEADER_SIZE)) //BNO080 can respond with upto 270 bytes, avoid overflow
			rxShtpData[dataSpot] = incoming; //Store data into the shtpData array
	}

	_i2cPort.stop();

	/*
	// first read the header to get the length of the packet
	int i2cResult;
	i2cResult = _i2cPort.read(_i2cAddress << 1, reinterpret_cast<char*>(packetBuffer), SHTP_HEADER_SIZE, false);
	if(i2cResult != 0)
	{
		_debugPort->printf("BNO I2C length read failed!\n");
		return false;
	}

	if(shtpHeader[0] == 0xFF && shtpHeader[1] == 0xFF)
	{
		// invalid according to BNO080 datasheet section 1.4.1
#if BNO_DEBUG
		_debugPort->printf("Recieved 0xFFFF packet length, protocol error!\n");
#endif
		return false;
	}

	//Calculate the number of data bytes in this packet
	uint16_t totalLength = (static_cast<uint16_t>(shtpHeader[1]) << 8 | shtpHeader[0]);

	// Clear the MSbit.
	// This bit indicates if this package is a continuation of the last. TBH, I don't really know what this means (it's not really explained in the datasheet)
	// but we don't actually care about any of the advertisement packets
	// that use this, so we can just cut off the rest of the packet by releasing chip select.
	totalLength &= ~(1 << 15);

	if (totalLength == 0)
	{
		// Packet is empty
		return (false); //All done
	}

	// only receive as many bytes as we can fit
	if(totalLength > sizeof(packetBuffer))
	{
		totalLength = sizeof(packetBuffer);
	}

	packetLength = totalLength - 4; //Remove the header bytes from the data count

	waitStartTime.reset();

	while(_int.read() != 0)
	{
		if(waitStartTime.elapsed_time() > timeout)
		{
			_debugPort->printf("BNO I2C wait timeout\n");
			return false;
		}

	}

	//Read the actual packet bytes
	_debugPort->printf("Attempting to read %" PRIu16 " bytes\n", totalLength);
	i2cResult = _i2cPort.read(_i2cAddress << 1, reinterpret_cast<char*>(packetBuffer), totalLength);
	if(i2cResult != 0)
	{
		_debugPort->printf("BNO I2C read failed!\n");
		return false;
	}
	*/

#if BNO_DEBUG
	_debugPort->printf("Recieved packet: ----------------\n");
	printPacket(rxPacketBuffer); // note: add 4 for the header length
#endif

	return (true); //We're done!
}

BNO080SPI::BNO080SPI(Stream *debugPort, PinName rstPin, PinName intPin, PinName wakePin, PinName misoPin,
					 PinName mosiPin, PinName sclkPin, PinName csPin, int spiSpeed):
BNO080Base(debugPort, intPin, rstPin),
_spiPort(mosiPin, misoPin, sclkPin, csPin, use_gpio_ssel),
_wakePin(wakePin, 1), // wake pin needs to be 1 on reset so that the BNO boots up into SPI mode
_spiSpeed(spiSpeed)
{
	_spiPort.frequency(spiSpeed);
	_spiPort.format(8, 3);
	_spiPort.set_default_write_value(0x0);
}

bool BNO080SPI::sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
	if(_int.read() == 0)
	{
		// The BNO is already awake because it has a packet it wants to send to us
	}
	else
	{
		// assert WAKE to tell the BNO to prepare for a transfer
		_wakePin = 0;

		Timer waitStartTime;
		waitStartTime.start();
		const std::chrono::milliseconds timeout = 10ms;

		while(_int.read() != 0)
		{
			if(waitStartTime.elapsed_time() > timeout)
			{
				_debugPort->printf("BNO SPI wake wait timeout\n");
				_wakePin = 1;
				return false;
			}
		}
		_wakePin = 1;
	}

	uint16_t totalLength = dataLength + 4; //Add four bytes for the header

	txShtpHeader[0] = totalLength & 0xFF;
	txShtpHeader[1] = totalLength >> 8;
	txShtpHeader[2] = channelNumber;
	txShtpHeader[3] = sequenceNumber[channelNumber];

#if BNO_DEBUG
	_debugPort->printf("Transmitting packet: ----------------\n");
	printPacket(txPacketBuffer);
#endif

	// wipe out any existing RX packet header so we know if we received a packet
	memset(rxPacketBuffer, 0, SHTP_HEADER_SIZE);

	// send packet to IMU.
	// This also might receive the first part of another packet, which there is no way to avoid.
	spiTransferAndWait(txPacketBuffer, totalLength, rxPacketBuffer, totalLength);

	if(rxShtpHeader[0] == 0 && rxShtpHeader[0] == 0)
	{
		// no header data so no packet received
		return true;
	}
	else
	{
		// received first part of data packet while writing
		if(receiveCompletePacket(totalLength))
		{
			// received data packet, send to proper channels
			processPacket();
			return true;
		}

		// receive failed
		return false;
	}
}

//Check to see if there is any new data available
//Read the contents of the incoming packet into the shtpData array
bool BNO080SPI::receivePacket(std::chrono::milliseconds timeout)
{
	Timer waitStartTime;
	waitStartTime.start();

	while (_int.read() != 0)
	{
		if (waitStartTime.elapsed_time() > timeout)
		{
			_debugPort->printf("BNO SPI header wait timeout\n");
			return false;
		}

	}

	// read the header bytes first.
	spiTransferAndWait(nullptr, 0, rxPacketBuffer, SHTP_HEADER_SIZE);

	// now read the data
	return receiveCompletePacket(SHTP_HEADER_SIZE, timeout);
}

bool BNO080SPI::receiveCompletePacket(size_t bytesRead, std::chrono::milliseconds timeout)
{
	// Process header bytes
	// ------------------------------------------------------------------------
	if (rxShtpHeader[0] == 0xFF && rxShtpHeader[1] == 0xFF)
	{
		// invalid according to BNO080 datasheet section 1.4.1

#if BNO_DEBUG
		_debugPort->printf("Recieved 0xFFFF packet length, protocol error!\n");
#endif
		return false;
	}

	//Calculate the number of data bytes in this packet
	uint16_t totalLength = (static_cast<uint16_t>(rxShtpHeader[1]) << 8 | rxShtpHeader[0]);

	// Clear the MSbit.
	totalLength &= ~(1 << 15);

	if (totalLength == 0)
	{
		// Packet is empty
		return (false); //All done
	}

	rxPacketLength = totalLength - SHTP_HEADER_SIZE;

	if(totalLength <= bytesRead)
	{
		// the original transaction already read the completed packet!  We're done.
		return true;
	}

	// Receive data
	// ------------------------------------------------------------------------

	// Wait for it to be ready to talk to us again.
	// Note: in my testing this takes about 200ms
	Timer waitStartTime;
	waitStartTime.start();
	while (_int.read() != 0)
	{
		if (waitStartTime.elapsed_time() > timeout)
		{
			_debugPort->printf("BNO SPI continued packet header wait timeout\n");
			return false;
		}

	}

	if(rxPacketLength > SHTP_RX_PACKET_SIZE)
	{
		_debugPort->printf("Packet too long (%" PRIu16 " bytes), increase SHTP_RX_PACKET_SIZE\n", rxPacketLength);
		_debugPort->printf("Packet dropped, expect subsequent driver errors.\n");
		return false;
	}

	if(bytesRead == SHTP_HEADER_SIZE)
	{
		// just read the entire packet into the buffer
		spiTransferAndWait(nullptr, 0, rxPacketBuffer, totalLength);
	}
	else
	{
		// we want to receive a new header, plus the remaining data bytes that haven't been read.
		size_t receiveLength = SHTP_HEADER_SIZE + (totalLength - bytesRead);

		// read remaining bytes into the data buffer starting at the next byte
		spiTransferAndWait(nullptr, 0, rxPacketBuffer + bytesRead, receiveLength);

		// erase the new header we just read, leaving only the data as a contiguous block
		std::memmove(rxPacketBuffer + bytesRead, rxPacketBuffer + bytesRead + SHTP_HEADER_SIZE, receiveLength - SHTP_HEADER_SIZE);
	}

#if BNO_DEBUG
	_debugPort->printf("Recieved packet: ----------------\n");
	printPacket(rxPacketBuffer);
#endif

	//ThisThread::sleep_for(4ms);

	// done!
	return true;
}

#if USE_ASYNC_SPI

void BNO080SPI::spiTransferAndWait(const uint8_t *tx_buffer, int tx_length, uint8_t *rx_buffer, int rx_length)
{
	_spiPort.transfer(tx_buffer, tx_length, rx_buffer, rx_length,
					  callback(this, &BNO080SPI::onSPITransferComplete),
					  SPI_EVENT_COMPLETE | SPI_EVENT_ERROR);

	uint32_t waitResult = spiCompleteFlag.wait_any(SPI_EVENT_ALL);
	if(!(waitResult & SPI_EVENT_COMPLETE))
	{
		// at least let the user know the error happened...
		_debugPort->printf("BNO Async SPI Error %" PRIu32 "\n", waitResult);
	}

}

void BNO080SPI::onSPITransferComplete(int event)
{
	spiCompleteFlag.set(event);
}

#endif