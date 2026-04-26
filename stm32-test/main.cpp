#include "mbed.h"
#include "light_strip.h"

LightStrip<3> light_strip{
    {
        0, 0, 255,
        255, 255, 255,
        0, 255, 0
    }
};
// LightStrip<8> light_strip{
//     lights_out, 
//     {0, 255, 0, 
//     0, 255, 0,
//     0, 255, 0,
//     255, 0, 0,
//     255, 0, 0,
//     255, 0, 0,
//     255, 0, 0,
//     255, 0, 0} 
// };

int main() {
    // int i = 0;
    
    // light_strip.edit_LED_RGB(2, {0, 0, 0});

    while (true) {
        // if (i >= 5) {
        //     light_strip.edit_LED(4, {0, 255, 0}); 
        // } else {
        //     light_strip.edit_LED(4, {255, 0, 0});
        // }
        // i++;
        // if (i == 10) {
        //     i = 0;
        // }
        light_strip.update_LEDs();
        // wait_us(500 + light_strip.get_LED_update_time());
        ThisThread::sleep_for(1s);
    }

    return 0;
}
