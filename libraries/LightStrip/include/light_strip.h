//
// Created by Jackson Pinsonneault on 10/28/25.
//

#ifndef MBED_OS_LIGHT_STRIP_H
#define MBED_OS_LIGHT_STRIP_H

#include "mbed.h"
#include "light_strip_exec.h"
#include "array"
#include <cstdint>
#include <cstdio>
#include <functional>

using color = std::array<uint8_t, 3>;

template<uint8_t size>
class LightStrip {

public:
    /** Create an object which wraps the WS2813B light strip
    */
    explicit LightStrip() {
        exec_gpio_init();
    }

    /** Create an object which wraps the WS2813B light strip
     * @param GRB_list sets all the LEDs to a color following the GRB format. Every 3 numbers are another LED which starts from index 0 to how many you put (index 0 is first light in sequence, and it goes from there). Setting a color to 0,0,0 means it'll be off.
    */
    explicit LightStrip(const std::initializer_list<uint8_t>& GRB_list) {
        int i = 0;
        for (uint8_t val : GRB_list) {
            _LED_list[i / 3][i % 3] = val;
            i++;
        }

        printf("Light strip initiated!\n"); // currently requires this for some weird (likely timing) reason

        exec_gpio_init();
    }
    
    /** Edits a specified LED to the GRB color you desire
     * @param LED_index Specifies the index of the LED you're editing. Indexes starts from index 0 to how many you put (index 0 is first light in sequence, and it goes from there).
     * @param GRB_list List of 3 numbers from 0-255 which determine the respective GRB color for the LED you're editing. Setting a color to 0,0,0 means it'll be off.
    */
    void edit_LED(uint8_t LED_index, const std::initializer_list<uint8_t>& GRB_list) {
        if (LED_index >= size) return;

        for (uint8_t i = 0; i < 3; i++) {
            _LED_list[LED_index][i] = GRB_list.begin()[i];
        }
    }

    /** Edits a specified LED to the RGB color you desire
     * @param LED_index Specifies the index of the LED you're editing. Indexes starts from index 0 to how many you put (index 0 is first light in sequence, and it goes from there).
     * @param RGB_list List of 3 numbers from 0-255 which determine the respective RGB color for the LED you're editing. Setting a color to 0,0,0 means it'll be off.
    */
    void edit_LED_RGB(uint8_t LED_index, const std::initializer_list<uint8_t>& RGB_list) {
        if (LED_index >= size) return;

        _LED_list[LED_index][1] = RGB_list.begin()[1];
        _LED_list[LED_index][0] = RGB_list.begin()[0];
        _LED_list[LED_index][2] = RGB_list.begin()[2]; 
    }

    /** Sends data over pin PC_13 to the light strip to update its colors
    */
    void update_LEDs() {
        for (const color& list : _LED_list) {
            send_byte(list[0]);
            send_byte(list[1]);
            send_byte(list[2]);
        }
    }

private:
    std::array<color, size> _LED_list; // LED list which contains the colors the LED strip will be set to

    /** Sends data for each bit
     * @param b The bit you want to send.
    */
    void send_bit(const uint8_t b) const {
        exec_gpio_set();
        if (b) {
            exec_delay_620ns();
        } else {
            exec_delay_250ns();
        }
        exec_gpio_clear();
        exec_delay_620ns();
    }

    /** Sends data for each byte
     * @param b The byte you want to send.
    */
    void send_byte(const uint8_t b) const {
        send_bit(b & 0b10000000);
        send_bit(b & 0b01000000);
        send_bit(b & 0b00100000);
        send_bit(b & 0b00010000);
        send_bit(b & 0b00001000);
        send_bit(b & 0b00000100);
        send_bit(b & 0b00000010);
        send_bit(b & 0b00000001);
    }

};

#endif //MBED_OS_LIGHT_STRIP_H
