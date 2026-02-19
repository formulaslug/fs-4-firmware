#pragma once

#include "layouts.h"

class ChargerDefaultLayout : public Layouts {
public:
  using Layouts::Layouts;
  //just try to output charging status 1 or 0.
  void drawChargerDdefaultLayout(bool is_charging);
};

