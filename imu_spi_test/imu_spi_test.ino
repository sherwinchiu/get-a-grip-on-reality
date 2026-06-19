// =============================================================================
//  imu_spi_test.ino  --  MINIMAL bring-up test: talk to the IIM-42652 over SPI.
//
//  Goal: prove the SPI WIRING works before writing a full driver. It just reads
//  the WHO_AM_I register (0x75), which must return 0x6F on an IIM-42652. If you
//  see 0x6F, the bus + wiring are good and we can build the real SPI driver.
//
//  WHY SPI MODE "just works": the IIM-42652 auto-selects its interface — the FIRST
//  falling edge on CS switches it into SPI mode (and disables I2C) until the next
//  power cycle. So no config register is needed; toggling CS is enough.
//
//  BOARD SETTINGS: ESP32S3 Dev Module, USB CDC On Boot: Enabled, Serial @ 115200.
// =============================================================================
#include <Arduino.h>
#include <SPI.h>

// >>>>>>>>>>>>>>>  YOUR WIRING (ESP32-S3 JTAG-function pins)  <<<<<<<<<<<<<<<<<<
//  Mapped from the JTAG names you gave: clock=MTCK(39), data-in=MTDI(41) -> MOSI,
//  data-out=MTDO(40) -> MISO, mode-select=MTMS(42) -> CS.
//  If WHO_AM_I doesn't read, the most likely fix is swapping MOSI(41) <-> MISO(40).
const int PIN_SCLK = 39;   // MTCK -> IMU SCLK  (serial clock)
const int PIN_MOSI = 41;   // MTDI -> IMU SDI   (data INTO the IMU)   [MOSI]
const int PIN_MISO = 40;   // MTDO -> IMU SDO   (data OUT of the IMU) [MISO]
const int PIN_CS   = 42;   // MTMS -> IMU CS    (chip select)
// ----------------------------------------------------------------------------

// IIM-42652 register map (same as the I2C library uses)
static const uint8_t REG_WHO_AM_I = 0x75;   // should read 0x6F
static const uint8_t CHIP_ID      = 0x6F;

SPIClass spi(FSPI);
// 1 MHz is slow + safe for a wiring test (the chip allows up to 24 MHz). InvenSense
// parts use SPI mode 0 or 3 — we use mode 3 here; if WHO_AM_I is wrong, try mode 0.
static SPISettings cfg(1000000, MSBFIRST, SPI_MODE3);

uint8_t readReg(uint8_t reg) {
  spi.beginTransaction(cfg);
  digitalWrite(PIN_CS, LOW);
  spi.transfer(reg | 0x80);            // MSB=1 -> READ
  uint8_t v = spi.transfer(0x00);      // clock out the value
  digitalWrite(PIN_CS, HIGH);
  spi.endTransaction();
  return v;
}

void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif
  delay(800);
  Serial.println();
  Serial.println("=== IIM-42652 SPI WHO_AM_I test ===");
  Serial.printf("pins: SCLK=%d  MOSI(SDI)=%d  MISO(SDO)=%d  CS=%d\n",
                PIN_SCLK, PIN_MOSI, PIN_MISO, PIN_CS);

  pinMode(21, OUTPUT);                  // status LED: proves the sketch is RUNNING
                                        // even if serial is mis-configured
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);          // CS idle high
  spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  // A couple of CS toggles to make sure the chip has latched into SPI mode.
  for (int i = 0; i < 3; i++) { digitalWrite(PIN_CS, LOW); delayMicroseconds(10); digitalWrite(PIN_CS, HIGH); delayMicroseconds(10); }
}

void loop() {
  digitalWrite(21, HIGH);              // LED ON while we read
  uint8_t who = readReg(REG_WHO_AM_I);
  Serial.printf("WHO_AM_I = 0x%02X  (expect 0x%02X)  ->  %s\n",
                who, CHIP_ID,
                (who == CHIP_ID) ? "WIRING OK"
                : (who == 0x00 || who == 0xFF) ? "no response (check CS/MISO/power/mode)"
                : "got data but wrong ID (check MISO/mode 0 vs 3)");
  delay(250);
  digitalWrite(21, LOW);              // LED OFF -> visible 2 Hz blink = sketch alive
  delay(250);
}
