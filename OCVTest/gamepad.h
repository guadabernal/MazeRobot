#ifndef GAMEPAD_H
#define GAMEPAD_H
#include <XInput.h>
#include <Windows.h>
#pragma comment(lib,"xinput9_1_0.lib")

class Gamepad
{
public:
  Gamepad() {}
  Gamepad(int GamepadIndex) : GamepadIndex(GamepadIndex) {}
  void update() {
    ZeroMemory(&state, sizeof(XINPUT_STATE));
    XInputGetState(GamepadIndex, &state);
  }
  bool Connected() {
    ZeroMemory(&state, sizeof(XINPUT_STATE));
    DWORD Result = XInputGetState(GamepadIndex, &state);
    return (Result == ERROR_SUCCESS);
  }
  float getStickLeftY() {
    float y = (state.Gamepad.sThumbLY - 128) / 257.0f;
    return y > 0 ? y / 127.0f : y / 128.0f;
  }
  float getStickLeftX() {
    float x = (state.Gamepad.sThumbLX - 128) / 257.0f;
    return x > 0 ? x / 127.0f : x / 128.0f;
  }
  float getStickRightY() {
    float y = (state.Gamepad.sThumbRY - 128) / 257.0f;
    return y > 0 ? y / 127.0f : y / 128.0f;
  }
  float getStickRightX() {
    float x = (state.Gamepad.sThumbRX - 128) / 257.0f;
    return x > 0 ? x / 127.0f : x / 128.0f;
  }
  bool leftButtonPress(){
    return (state.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER);
  }
  bool rightButtonPress(){
    return (state.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER);
  }
  
private:
  XINPUT_STATE state;
  size_t GamepadIndex;
};


float scaleInput(float v) {
  //float scaleArray[] = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
  //  0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
  float scaleArray[] = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
    0.30, 0.36, 0.43, 0.50, 0.60, 0.65, 0.70, 0.75, 1.00 };

  int index = (int)(v * 16.0);
  if (index < 0)  index = -index;
  if (index > 16) index = 16;
  if (v < 0)
    return(-scaleArray[index]);
  else
    return(scaleArray[index]);
}

#endif