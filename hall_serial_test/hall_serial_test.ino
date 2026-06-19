// =============================================================================
//  hall_serial_test.ino  --  MINIMAL diagnostic: Hall sensors + Serial only.
//
//  NO Bluetooth, NO servos, NO FreeRTOS tasks. Lowest possible power draw, so if
//  the board still resets here it is NOT BLE/servo current — and the printed
//  "reset reason" tells us exactly what happened. Use this to prove the board,
//  USB Serial, and the Hall ADC all work before bringing the full firmware back.
//
//  BOARD SETTINGS (same as the main firmware):
//    Tools -> Board: ESP32S3 Dev Module
//    Tools -> USB CDC On Boot: Enabled   (so Serial comes out the native USB)
//    Serial Monitor @ 115200
// =============================================================================
#include <Arduino.h>
#include "esp_system.h"

// Right-hand Hall pins (matches glove_firmware_rtos/hall.cpp): {bend0, bend1, splay}
const int hall_pins[5][3] = {
  {  4,  5,  6 },   // thumb
  {  7, 15, 16 },   // index
  { 17, 18,  8 },   // middle
  { 11, 12, 13 },   // ring
  {  3,  9, 10 },   // pinky
};
const char* names[5] = { "THM", "IDX", "MID", "RNG", "PNK" };

uint32_t tick = 0;

void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);          // native USB: never block when no monitor attached
#endif
  delay(800);                        // let the USB host enumerate the port

  Serial.println();
  Serial.println("=== HALL + SERIAL MINIMAL TEST (no BLE, no servos) ===");
  // If this number changes each time the log restarts, the board is RESETTING.
  //  1=POWERON  3=SW  4=PANIC(crash)  5/6/7=WATCHDOG  8=DEEPSLEEP  9=BROWNOUT(power)
  Serial.printf("reset reason: %d   "
                "[1=poweron 3=sw 4=panic 5/6/7=watchdog 8=deepsleep 9=BROWNOUT]\n",
                (int)esp_reset_reason());

  for (int i = 0; i < 5; i++)
    for (int j = 0; j < 3; j++)
      pinMode(hall_pins[i][j], INPUT);

  Serial.println("reading hall sensors...");
}

void loop() {
  // tick + uptime: if these reset to 0 / drop, the board rebooted.
  Serial.printf("#%lu  t=%lus  | ", (unsigned long)tick++, (unsigned long)(millis() / 1000));
  for (int i = 0; i < 5; i++) {
    Serial.printf("%s", names[i]);
    for (int j = 0; j < 3; j++) Serial.printf(" %4d", analogRead(hall_pins[i][j]));
    Serial.print("  ");
  }
  Serial.println();
  delay(250);                        // ~4 prints/sec
}
