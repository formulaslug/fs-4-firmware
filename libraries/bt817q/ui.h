//
// Created by Nova Mondal on 4/30/25.
//

#include "mbed.h"

#ifndef UI_H
#define UI_H

struct Color {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t alpha = 255;
};

static constexpr Color red{255, 0, 0};

static constexpr Color orange{200, 89, 41};

static constexpr Color yellow{255, 200, 0};

static constexpr Color green{0, 186, 81};

static constexpr Color blue{0, 160, 240};

static constexpr Color white{255, 255, 255, 255};

static constexpr Color mid_gray{101, 101, 101};

static constexpr Color black{0, 0, 0};

struct Point {
  uint16_t x;
  uint16_t y;
};

#endif // UI_H
