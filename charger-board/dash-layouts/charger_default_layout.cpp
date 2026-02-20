

#include "charger_default_layout.h"


void ChargerDefaultLayout::drawChargerDefaultLayout(bool is_charging) {
  if(failure == startFrame()) {
    return;
  }
  loadFonts(); 

  clear(255, 255, 255);
  setMainColor(black);

  if(is_charging){
    drawText(120, 200, "CHARGING", 31);
    drawRect(Point{100, 260}, Point{700, 320}, green);

  }else{
    drawText(120, 200, "NOT CHARGING", 31);
    drawRect(Point{100, 260}, Point{700, 320}, green);
  }
  endFrame();
}
