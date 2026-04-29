#include "mbed.h"
#include "BNO080.h"

using namespace std::chrono_literals;

#define BNO_RST   PC_8
#define BNO_INT   PB_13
#define BNO_WAKE  PA_11
#define BNO_MISO  PB_14
#define BNO_MOSI  PB_15
#define BNO_SCK   PC_7
#define BNO_CS    PB_12

BNO086SPI imu(
	nullptr, 
	BNO_RST,
	BNO_INT,
	BNO_WAKE,
	BNO_MISO,
	BNO_MOSI,
	BNO_SCK,
	BNO_CS,
	1000000);

int main() {

    printf("BNO086 SPI test\n");

	if(!imu.begin())
	{
		printf("Could not find BNO086.\n");
		while(true)
		{
			ThisThread::sleep_for(1s);
		}
	}

	printf("BNO086 connected\n");
	printf("Firmware: %u.%u.%u\n",
		   imu.majorSoftwareVersion,
		   imu.minorSoftwareVersion,
		   imu.patchSoftwareVersion);

	imu.enableReport(BNO080Base::GAME_ROTATION, 20); // 20 ms = 50 Hz

	while(true)
	{
		imu.updateData();

		if(imu.hasNewData(BNO080Base::GAME_ROTATION))
		{
			Quaternion q = imu.gameRotationVector;

			printf("quat: x=% .4f y=% .4f z=% .4f w=% .4f\n",
				   q.x(),
				   q.y(),
				   q.z(),
				   q.w());
		}

		ThisThread::sleep_for(2ms);
	}
}
