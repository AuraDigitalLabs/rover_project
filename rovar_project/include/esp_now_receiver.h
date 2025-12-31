/*
 * ESP-NOW receiver for joystick data
 */

#ifndef ESP_NOW_RECEIVER_H
#define ESP_NOW_RECEIVER_H

struct JoystickData {
  int x;
  int y;
  bool button;
};

void initEspNowReceiver();
bool espNowReadJoystick(JoystickData& out);

#endif // ESP_NOW_RECEIVER_H
