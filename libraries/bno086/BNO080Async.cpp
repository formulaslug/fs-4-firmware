//
// Created by jamie on 11/18/2020.
//

#include "BNO080Async.h"

#include <cinttypes>

using namespace std::chrono_literals;

#define BNO_ASYNC_DEBUG 0

// event flags constants for signalling the thread
#define EF_INTERRUPT 0b1
#define EF_SHUTDOWN 0b10
#define EF_RESTART 0b100

void BNO080Async::threadMain()
{
	while(commLoop())
	{
		// loop forever
	}
}

bool BNO080Async::commLoop()
{
	_rst = 0; // Reset BNO080
	ThisThread::sleep_for(1ms); // Min length not specified in datasheet?
	_rst = 1; // Bring out of reset

	// wait for a falling edge (NOT just a low) on the INT pin to denote startup
	{
		EventFlags edgeWaitFlags;
		_int.fall(callback([&](){edgeWaitFlags.set(1);}));

		// have the RTOS wait until an edge is detected or the timeout is hit
		uint32_t edgeWaitEvent = edgeWaitFlags.wait_any(1, (BNO080_RESET_TIMEOUT).count());

		if(!edgeWaitEvent)
		{
			debugPrintf("Error: BNO080 reset timed out, chip not detected.\n");
		}
	}

#if BNO_ASYNC_DEBUG
	debugPrintf("BNO080 detected!\r\n");
#endif

	wakeupFlags.set(EF_INTERRUPT);

	// configure interrupt to send the event flag
	_int.fall(callback([&]()
	{
		if(!inTXRX)
		{
			wakeupFlags.set(EF_INTERRUPT);
		}
	}));

	while(true)
	{
		uint32_t wakeupEvent = wakeupFlags.wait_any(EF_INTERRUPT | EF_SHUTDOWN | EF_RESTART);

		if(wakeupEvent & EF_SHUTDOWN)
		{
			// shutdown thread permanently
			return false;
		}
		else if(wakeupEvent & EF_RESTART)
		{
			// restart thread
			return true;
		}

		// lock the mutex to handle remaining cases
		bnoDataMutex.lock();

		if(wakeupEvent & EF_INTERRUPT)
		{
			while(_int == 0)
			{
				inTXRX = true;

				// send data if there is data to send.  This may also receive a packet.
				if (dataToSend)
				{
					BNO080SPI::sendPacket(txPacketChannelNumber, txPacketDataLength);
					dataToSend = false;
				}
				else
				{
					BNO080SPI::receivePacket();
				}

				inTXRX = false;

				// clear the wake flag if it was set
				_wakePin = 1;

				// If the IMU wants to send us an interrupt immediately after a transaction,
				// it will be missed due to the inTXRX block.  So, we need to keep checking _int
				// in a loop.

				// update received flag
				dataReceived = true;

				// check if this packet is being waited on.
				if (waitingForPacket)
				{
					if (waitingForReportID == rxShtpData[0] && waitingForChannel == rxShtpHeader[2])
					{
						// unblock main thread so it can process this packet
						waitingPacketArrived = true;
						waitingPacketArrivedCV.notify_all();

						// unlock mutex and wait until the main thread says it's OK to receive another packet
						clearToRxNextPacket = false;
						waitingPacketProcessedCV.wait([&]() { return clearToRxNextPacket; });
					}
				}
				else
				{
					processPacket();
				}

				// clear the interrupt flag if it has been set because we're about to check _int anyway.
				// Prevents spurious wakeups.
				wakeupFlags.clear(EF_INTERRUPT);

			}

		}

		// unlock data mutex before waiting for flags
		bnoDataMutex.unlock();
	}
}

bool BNO080Async::sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
	// first make sure mutex is locked
	if(bnoDataMutex.get_owner() != ThisThread::get_id())
	{
		debugPrintf("IMU communication function called without bnoDataMutex locked!\n");
		return false;
	}

	// now set class variables
	txPacketChannelNumber = channelNumber;
	txPacketDataLength = dataLength;
	dataToSend = true;

	// signal thread to wake up by sending _wake signal (which will cause the IMU to interrupt us once ready)
	_wakePin = 0;

	return true;
}

void BNO080Async::clearSendBuffer()
{
	// first make sure mutex is locked
	if(bnoDataMutex.get_owner() != ThisThread::get_id())
	{
		debugPrintf("IMU communication function called without bnoDataMutex locked!\n");
		return;
	}

	// Check if we are trying to send a packet while another packet is still queued.
	// Since we don't have an actual queue, just wait until the other packet is sent.
	while(dataToSend)
	{
		dataSentCV.wait();
	}

	// now actually erase the buffer
	BNO080Base::clearSendBuffer();
}

bool BNO080Async::waitForPacket(int channel, uint8_t reportID, std::chrono::milliseconds timeout)
{
	// first make sure mutex is locked
	if(bnoDataMutex.get_owner() != ThisThread::get_id())
	{
		debugPrintf("IMU communication function called without bnoDataMutex locked!\n");
		return false;
	}

	// send information to thread
	waitingForPacket = true;
	waitingForChannel = channel;
	waitingForReportID = reportID;
	waitingPacketArrived = false;

	// now unlock mutex and allow thread to run and receive packets
	waitingPacketArrivedCV.wait_for(timeout, [&]() {return waitingPacketArrived;});

	if(!waitingPacketArrived)
	{
		debugPrintf("Packet wait timeout.\n");
		return false;
	}

	// packet we are waiting for is now in the buffer.
	waitingForPacket = false;

	// Now we can unblock the comms thread and allow it to run.
	// BUT, it can't actually start until bnoDataMutex is released.
	// This means that the packet data is guaranteed to stay in the buffer until that mutex is released
	// which will happen either when the main thread is done with the IMU, or on the next sendPacket() or
	// waitForPacket() call.
	clearToRxNextPacket = true;
	waitingPacketProcessedCV.notify_all();

	return true;
}

BNO080Async::BNO080Async(Stream *debugPort, PinName rstPin, PinName intPin, PinName wakePin, PinName misoPin,
						 PinName mosiPin, PinName sclkPin, PinName csPin, int spiSpeed, osPriority threadPriority):
 BNO080SPI(debugPort, rstPin, intPin, wakePin, misoPin, mosiPin, sclkPin, csPin, spiSpeed),
 commThread(threadPriority),
 dataSentCV(bnoDataMutex),
 waitingPacketArrivedCV(bnoDataMutex),
 waitingPacketProcessedCV(bnoDataMutex)
{

}


bool BNO080Async::begin()
{
	// shut down thread if it's running
	if(commThread.get_state() == Thread::Deleted)
	{
		// start thread for the first time
		commThread.start(callback(this, &BNO080Async::threadMain));
	}
	else
	{
		// restart thread
		wakeupFlags.set(EF_RESTART);
	}


	{
		ScopedLock<Mutex> lock(bnoDataMutex);

		// once the thread starts it, the BNO will send an Unsolicited Initialize response (SH-2 section 6.4.5.2), and an Executable Reset command
		if(!waitForPacket(CHANNEL_EXECUTABLE, EXECUTABLE_REPORTID_RESET, 1s))
		{
			debugPrintf("No initialization report from BNO080.\n");
			return false;
		}
		else
		{
#if BNO_DEBUG
			debugPrintf("BNO080 reports initialization successful!\n");
#endif
		}

		// Finally, we want to interrogate the device about its model and version.
		clearSendBuffer();
		txShtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
		txShtpData[1] = 0; //Reserved
		sendPacket(CHANNEL_CONTROL, 2);

		waitForPacket(CHANNEL_CONTROL, SHTP_REPORT_PRODUCT_ID_RESPONSE);

		if (rxShtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
		{
			majorSoftwareVersion = rxShtpData[2];
			minorSoftwareVersion = rxShtpData[3];
			patchSoftwareVersion = (rxShtpData[13] << 8) | rxShtpData[12];
			partNumber = (rxShtpData[7] << 24) | (rxShtpData[6] << 16) | (rxShtpData[5] << 8) | rxShtpData[4];
			buildNumber = (rxShtpData[11] << 24) | (rxShtpData[10] << 16) | (rxShtpData[9] << 8) | rxShtpData[8];

#if BNO_DEBUG
			debugPrintf("BNO080 reports as SW version %hhu.%hhu.%hu, build %lu, part no. %lu\n",
						   majorSoftwareVersion, minorSoftwareVersion, patchSoftwareVersion,
						   buildNumber, partNumber);
#endif

		}
		else
		{
			debugPrintf("Bad response from product ID command.\n");
			return false;
		}
	}


	// successful init
	return true;
}

bool BNO080Async::updateData()
{
	bool newData = dataReceived;
	dataReceived = false;
	return newData;
}






