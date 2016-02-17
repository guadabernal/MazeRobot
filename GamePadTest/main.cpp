#include <windows.h>
#include <stdint.h>

#include <iostream>
#include "gamepad.h"
#include "utils.h"
#include "SerialPort.h"



int main() {
  Gamepad gpad(0);
  SerialPort serial;
  Sleep(1000);
  serial.connect("\\\\.\\COM11");

  while (true) {
    gpad.update();
    float lx = scaleInput(gpad.getStickLeftX());
    float ly = scaleInput(gpad.getStickLeftY());
    float rx = scaleInput(gpad.getStickRightX());
    float ry = scaleInput(gpad.getStickRightY());
    printf("lx = %f ly =%f rx = %f ry = %f\n", lx, ly, rx, ry);
    int16_t v[2];
    v[0] = ly * 255;
    v[1] = ry * 255;
    int ret = serial.WriteData((char*)v, 4);
    Sleep(20);
  }

}