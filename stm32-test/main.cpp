#include "mbed.h"
#include "tmp1075.hpp"

I2C i2c{D4, D5};
TMP1075 temp{i2c, 0b1001000};

int main() {
    printf("Hello World!!\n");

    i2c.frequency(100000);

    temp.begin();
    // temp.setConversionMode(false);

    printf("temp id: 0x%0X\n", temp.getDeviceId());

    while (true) {
        temp.startConversion();
        temp.setConversionTime(TMP1075::ConversionTime::ConversionTime220ms);
        ThisThread::sleep_for(200ms);
        float c = temp.getTemperatureCelsius();
        printf("%f C, 0x raw\n", c);
        ThisThread::sleep_for(10ms);
    }

    return 0;
}
