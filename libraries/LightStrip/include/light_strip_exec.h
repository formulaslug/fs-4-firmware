//
// Created by Jackson Pinsonneault on 1/15/26.
//

#ifndef LIGHT_STRIP_UTILS_LIGHT_STRIP_EXEC_H
#define LIGHT_STRIP_UTILS_LIGHT_STRIP_EXEC_H

/** Initializes the bit banging operation for setting/clearing pin PC_13
*/
void exec_gpio_init();

/** Bit banging operation for setting pin PC_13
*/
void exec_gpio_set();

/** Bit banging operation for clearing pin PC_13
*/
void exec_gpio_clear();

/** Delays operations for approximately 250ns (times from oscilliscope)
*/
void exec_delay_250ns();

/** Delays operations for approximately 620ns (times from oscilliscope)
*/
void exec_delay_620ns();

#endif //LIGHT_STRIP_UTILS_LIGHT_STRIP_EXEC_H
