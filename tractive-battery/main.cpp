#include "FlashIAP.h"
#include "mbed.h"

const uint32_t USER_FLASH_BEGIN = 0x0800C000; // sector 3
const uint32_t SOH_VAL = sizeof(int);
char buffer[] = {0x12, 0x34, 0x56, 0x78};

int main(int argc, char const* argv[]) {

    FlashIAP flash;
    flash.init();
    flash.erase(USER_FLASH_BEGIN, SOH_VAL);
    flash.program(&buffer, USER_FLASH_BEGIN, 1);

    return 0;
}