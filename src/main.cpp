#include "Arduino.h"

//----------------------------------------------------------------------------
// This sketch scans valid i2c 7-bit addresses
// This does not scan for devices using 10-bit addresses
//----------------------------------------------------------------------------

#include <Wire.h>

#define SERIAL_BAUD_RATE   115200
#define REPEAT_DELAY       5000 // milliseconds
#define SDA_PIN D3
#define SCL_PIN D4
#define SCAN_ADDRESS_START       0x07
#define SCAN_ADDRESS_END         0x7F
#define I2C_RANGE_START          0x00 // 0x00 is the begining of 7-bit i2c address space
#define I2C_RANGE_END            0x7F // 0x7F is the end of 7-bit i2c address space

uint8_t deviceStatus[128]; // global

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial); // Wait for serial monitor
  Wire.begin(SDA_PIN, SCL_PIN);
}

void displayMatch(uint8_t addr)
{
  Serial.print("-[ 0x"); Serial.print(addr, HEX); Serial.println(" ]------------------------------------------");
  if ((addr >= 0x00) && (addr <= 0x07))  Serial.println("Reserved space");
  if ((addr >= 0x78) && (addr <= 0x7f))  Serial.println("Reserved space");
  if ((addr >= 0x76) && (addr <= 0x77))  Serial.println("Bosch BME280 - Temp/Pressure/Humidity Sensor");
  if ((addr >= 0x76) && (addr <= 0x77))  Serial.println("Bosch BME680 - Temp/Pressure/Humidity/VOC Sensor");
  if (addr == 0x77)                      Serial.println("Bosch BMP085 - Temp/Pressure Sensor");
  if (addr == 0x77)                      Serial.println("Bosch BMP180 - Temp/Pressure Sensor");
  if ((addr >= 0x76) && (addr <= 0x77))  Serial.println("Bosch BMP280 - Temp/Pressure Sensor");
  if ((addr >= 0x20) && (addr <= 0x27))  Serial.println("PCF8574 - 8-bit I/O Expander");
  if ((addr >= 0x38) && (addr <= 0x3F))  Serial.println("PCF8574A - 8-bit I/O Expander");
  if (addr == 0x27)                      Serial.println("PCF8574 - LCD with I2C Converter");
  if (addr == 0x3f)                      Serial.println("PCF8574A - LCD with I2C Converter");
  if ((addr >= 0x44) && (addr <= 0x45))  Serial.println("Sensirion SHT30/SHT31/SHT35 - Temp/Humidity Sensor");
  if (addr == 0x44)                      Serial.println("Nettigo HECA - Heating Element Control Assembly");
  if ((addr >= 0x5a) && (addr <= 0x5b))  Serial.println("ams CCS811 - VOC/eCO2 Sensor");
  if (addr == 0x68)                      Serial.println("Maxim DS1307 - Real-Time Clock");
  if (addr == 0x68)                      Serial.println("Maxim DS3231 - Real-Time Clock");
  if (addr == 0x50)                      Serial.println("Microchip 24C32 - Serial EEPROM");
  if ((addr >= 0x70) && (addr <= 0x77))  Serial.println("Holtek HT16K33 - LED Matrix Driver");
  if (addr == 0x40)                      Serial.println("TE Conectivity HTU21D-F - Temp/Humidity Sensor");
  if ((addr >= 0x19) && (addr <= 0x1e))  Serial.println("ST LSM303AGR - 3-axis Accelerometer/Magnetometer");
  if (addr == 0x2f)                      Serial.println("Analog Devices AD5243 - Dual Potentiometer");
  if ((addr >= 0x2c) && (addr <= 0x2f))  Serial.println("Analog Devices AD5248 - Dual Potentiometer");
  if ((addr >= 0x2c) && (addr <= 0x2f))  Serial.println("Analog Devices AD5251 - Dual Potentiometer");
  if ((addr >= 0x2c) && (addr <= 0x2f))  Serial.println("Analog Devices AD5252 - Dual Potentiometer");
  if ((addr >= 0x49) && (addr <= 0x4a))  Serial.println("Texas Instruments ADS1115 - 4-ch 16-bit ADC");
  if ((addr == 0x1d) || (addr == 0x53))  Serial.println("Analog Devices ADXL345 - 3-axis Accelerometer");
  if ((addr >= 0x0c) && (addr <= 0x0f))  Serial.println("AsahiKasei AK8975 - 3-axis Magnetometer");
  if (addr == 0x5c)                      Serial.println("Aosong AM2315 - Temp/Humidity Sensor");
  if ((addr >= 0x68) && (addr <= 0x69))  Serial.println("Panasonic AMG8833 - IR Thermal Camera");
  if (addr == 0x52)                      Serial.println("Broadcom APDS-9250 - IR/RGB/Ambient Light Sensor");
  if (addr == 0x39)                      Serial.println("Avago Technologies APDS-9960 - IR/Color/Proximity Sensor");
  if (addr == 0x49)                      Serial.println("ams AS7262 - 6-channel Visible Spectral ID");
  if (addr == 0x38)                      Serial.println("Bosch BMA150 - 3-axis Accelerometer");
  if (addr == 0x77)                      Serial.println("Bosch BMA180 - 3-axis Accelerometer");

  // Needs verification. Data copied from https://i2cdevices.org/devices

  // if ((addr >= 0x68) && (addr <= 0x69)) Serial.println("ITG3200 - Gyro");
  // if ((addr >= 0x20) && (addr <= 0x21)) Serial.println("MCP23008 - 8-Bit I/O Expander with Serial Interface I2C GPIO expander");
  // if ((addr >= 0x20) && (addr <= 0x21)) Serial.println("MCP23017 - I2C GPIO expander");
  // if (addr == 0x13) Serial.println("VCNL40x0 - proximity sensor");
  // if ((addr >= 0x38) && (addr <= 0x39)) Serial.println("VEML6070 - UVA Light Sensor with I2C Interface");
  // if (addr == 0x10) Serial.println("VEML7700 - High Accuracy Ambient Light Sensor ");
  // if (addr == 0x29) Serial.println("VL53L0x - Time Of Flight distance sensor");
  // if (addr == 0x29) Serial.println("VL6180X - Time Of Flight distance sensor");
  // if (addr == 0x10) Serial.println("VML6075 - UVA and UVB Light Sensor with I2C Interface");

  // if ((addr >= 0x28) && (addr <= 0x29)) Serial.println("BNO055 - Absolute Orientation Sensor");
  // if ((addr >= 0x28) && (addr <= 0x2d)) Serial.println("CAP1188 - 8-channel Capacitive Touch");
  // if ((addr >= 0x2c) && (addr <= 0x2d)) Serial.println("CAT5171 - 256‐position I2C Compatible Digital Potentiometer ");
  // if (addr == 0x20) Serial.println("Chirp! - Water sensor");
  // if (addr == 0x5a) Serial.println("DRV2605 - Haptic Motor Driver");
  // if (addr == 0x38) Serial.println("FT6x06 - Capacitive Touch Driver");
  // if ((addr >= 0x20) && (addr <= 0x21)) Serial.println("FXAS21002 - 3-axis gyroscope");
  // if ((addr >= 0x1f) && (addr <= 0x1c)) Serial.println("FXOS8700 - 6-axis sensor with integrated linear accelerometer and magnetometer");
  // if ((addr >= 0x42) && (addr <= 0x43)) Serial.println("HDC1008 - Low Power, High Accuracy Digital Humidity Sensor with Temperature Sensor");
  // if (addr == 0x1e) Serial.println("HMC5883 - 3-Axis Digital Compass/Magnetometer IC");
  // if (addr == 0x5f) Serial.println("HTS221 - Capacitive digital sensor for relative humidity and temperature");
  // if ((addr >= 0x42) && (addr <= 0x49)) Serial.println("INA219 - 26V Bi-Directional High-Side Current/Power/Voltage Monitor");
  // if ((addr >= 0x42) && (addr <= 0x49)) Serial.println("INA260 - Precision Digital Current and Power Monitor With Low-Drift, Precision Integrated Shunt");
  // if ((addr >= 0x66) && (addr <= 0x77)) Serial.println("IS31FL3731 - 144-LED Audio Modulated Matrix LED Driver (CharliePlex)");
  // if (addr == 0x44) Serial.println("ISL29125 - Digital Red, Green and Blue Color Light Sensor with IR Blocking Filter");
  // if (addr == 0x0e) Serial.println("IST-8310 - Three-axis Magnetometer");
  // if ((addr >= 0x6a) && (addr <= 0x6b)) Serial.println("L3GD20H - gyroscope");
  // if ((addr >= 0x19) && (addr <= 0x18)) Serial.println("LIS3DH - 3-axis accelerometer");
  // if (addr == 0x0e) Serial.println("MAG3110 - 3-Axis Magnetometer");
  // if (addr == 0x57) Serial.println("MAX3010x - Pulse &amp; Oximetry sensor");
  // if ((addr >= 0x4b) && (addr <= 0x4a)) Serial.println("MAX44009 - Ambient Light Sensor with ADC");
  // if ((addr >= 0x51) && (addr <= 0x53)) Serial.println("MB85RC - Ferroelectric RAM");
  // if ((addr >= 0x61) && (addr <= 0x60)) Serial.println("MCP4725A0 - 12-bit DAC");
  // if ((addr >= 0x66) && (addr <= 0x60)) Serial.println("MCP4725A1 - 12-Bit Digital-to-Analog Converter with EEPROM Memory");
  // if ((addr >= 0x65) && (addr <= 0x64)) Serial.println("MCP4725A2 - 12-Bit Digital-to-Analog Converter with EEPROM Memory");
  // if ((addr >= 0x66) && (addr <= 0x67)) Serial.println("MCP4725A3 - 12-Bit Digital-to-Analog Converter with EEPROM Memory");
  // if ((addr >= 0x1f) && (addr <= 0x1c)) Serial.println("MCP9808 - ±0.5°C Maximum Accuracy Digital Temperature Sensor");
  // if (addr == 0x5a) Serial.println("MLX90614 - IR temperature sensor");
  // if ((addr >= 0x1d) && (addr <= 0x1c)) Serial.println("MMA845x - 3-axis, 14-bit/8-bit digital accelerometer");
  // if (addr == 0x60) Serial.println("MPL115A2 - Miniature I2C digital barometer, 50 to 115 kPa");
  // if (addr == 0x60) Serial.println("MPL3115A2 - Barometric Pressure");
  // if ((addr >= 0x5c) && (addr <= 0x5d)) Serial.println("MPR121 - 12-point capacitive touch sensor");
  // if ((addr >= 0x68) && (addr <= 0x69)) Serial.println("MPU6050 - Six-Axis (Gyro + Accelerometer) MEMS MotionTracking™ Devices");
  // if ((addr >= 0x68) && (addr <= 0x69)) Serial.println("MPU-9250 - 9-DoF IMU Gyroscope, Accelerometer and Magnetometer");
  // if ((addr >= 0x76) && (addr <= 0x77)) Serial.println("MS5607 - Barometric Pressure");
  // if ((addr >= 0x76) && (addr <= 0x77)) Serial.println("MS5611 - Barometric Pressure");
  // if ((addr >= 0x41) && (addr <= 0x40)) Serial.println("NE5751 - Audio processor for IV communication");
  // if (addr == 0x52) Serial.println("Nunchuck controller - Nintendo");
  // if (addr == 0x22) Serial.println("PCA1070 - Multistandard programmable analog CMOS speech transmission IC");
  // if ((addr >= 0x42) && (addr <= 0x5f)) Serial.println("PCA9685 - 16-channel PWM driver default address");
  // if ((addr >= 0x25) && (addr <= 0x24)) Serial.println("PCD3311C - DTMF/modem/musical tone generator");
  // if ((addr >= 0x25) && (addr <= 0x24)) Serial.println("PCD3312C - DTMF/modem/musical-tone generator");
  // if (addr == 0x68) Serial.println("PCF8523 - RTC");
  // if ((addr >= 0x3b) && (addr <= 0x3c)) Serial.println("PCF8569 - LCD column driver for dot matrix displays ");
  // if ((addr >= 0x68) && (addr <= 0x6b)) Serial.println("PCF8573 - Clock/calendar with Power Fail Detector");
  // if (addr == 0x3a) Serial.println("PCF8577C - 32/64-segment LCD display driver");
  // if ((addr >= 0x3d) && (addr <= 0x3c)) Serial.println("PCF8578 - Row/column LCD dot matrix driver/display ");
  // if (addr == 0x48) Serial.println("PN532 - NFC/RFID reader");
  // if ((addr >= 0x3b) && (addr <= 0x39)) Serial.println("SAA1064 - 4-digit LED driver");
  // if ((addr >= 0x30) && (addr <= 0x31)) Serial.println("SAA2502 - MPEG audio source decoder");
  // if ((addr >= 0x23) && (addr <= 0x21)) Serial.println("SAA4700 - VPS Dataline Processor");
  // if (addr == 0x11) Serial.println("SAA5243P/E - Computer controlled teletext circuit ");
  // if (addr == 0x11) Serial.println("SAA5243P/H - Computer controlled teletext circuit ");
  // if (addr == 0x11) Serial.println("SAA5243P/K - Computer controlled teletext circuit ");
  // if (addr == 0x11) Serial.println("SAA5243P/L - Computer controlled teletext circuit ");
  // if (addr == 0x11) Serial.println("SAA5246 - Integrated VIP and teletext");
  // if ((addr >= 0x62) && (addr <= 0x60)) Serial.println("SAB3035 - Digital tuning circuit for computer-controlled TV ");
  // if ((addr >= 0x62) && (addr <= 0x60)) Serial.println("SAB3037 - Digital tuning circuit for computer-controlled TV");
  // if (addr == 0x58) Serial.println("SGP30 - Gas Sensor");
  // if ((addr >= 0x3d) && (addr <= 0x3c)) Serial.println("SH1106 - 132 X 64 Dot Matrix OLED/PLED  Preliminary Segment/Common Driver with Controller");
  // if (addr == 0x60) Serial.println("Si1145 - Proximity/UV/Ambient Light Sensor IC With I2C Interface");
  // if ((addr >= 0x11) && (addr <= 0x63)) Serial.println("Si4713 - FM Radio Transmitter with Receive Power Scan");
  // if ((addr >= 0x61) && (addr <= 0x60)) Serial.println("Si5351A - Clock Generator");
  // if (addr == 0x40) Serial.println("Si7021 - Humidity/Temp sensor");
  // if (addr == 0x69) Serial.println("SPS30 - Particulate Matter Sensor for Air Quality Monitoring and Control");
  // if ((addr >= 0x3d) && (addr <= 0x3c)) Serial.println("SSD1305 - 132 x 64 Dot Matrix OLED/PLED Segment/Common Driver with Controller");
  // if ((addr >= 0x3d) && (addr <= 0x3c)) Serial.println("SSD1306 - 128 x 64 Dot Matrix Monochrome OLED/PLED Segment/Common Driver with Controller ");
  // if ((addr >= 0x41) && (addr <= 0x44)) Serial.println("STMPE610 - Resistive Touch controller");
  // if ((addr >= 0x41) && (addr <= 0x44)) Serial.println("STMPE811 - Resistive touchscreen controller");
  // if ((addr >= 0x70) && (addr <= 0x73)) Serial.println("TCA9548 - 1-to-8 I2C Multiplexer");
  // if (addr == 0x29) Serial.println("TCS34725 - color sensor");
  // if (addr == 0x44) Serial.println("TDA4670 - Picture signal improvement circuit");
  // if (addr == 0x44) Serial.println("TDA4671 - Picture signal improvement circuit");
  // if (addr == 0x44) Serial.println("TDA4672 - Picture signal improvement (PSI) circuit");
  // if (addr == 0x44) Serial.println("TDA4680 - Video processor");
  // if (addr == 0x44) Serial.println("TDA4687 - Video processor");
  // if (addr == 0x44) Serial.println("TDA4688 - Video processor");
  // if (addr == 0x44) Serial.println("TDA4780 - Video control with gamma control");
  // if (addr == 0x46) Serial.println("TDA8370 - High/medium perf. sync. processor");
  // if (addr == 0x45) Serial.println("TDA8376 - One-chip multistandard video");
  // if (addr == 0x42) Serial.println("TDA8415 - TVNCR stereo/dual sound processor");
  // if (addr == 0x42) Serial.println("TDA8417 - TVNCR stereo/dual sound processor");
  // if ((addr >= 0x41) && (addr <= 0x40)) Serial.println("TDA8421 - Audio processor with loudspeaker and headphone channel ");
  // if (addr == 0x41) Serial.println("TDA8424 - Audio processor with loudspeaker channel");
  // if (addr == 0x41) Serial.println("TDA8425 - Audio processor with loudspeaker channel");
  // if (addr == 0x41) Serial.println("TDA8426 - Hi-fi stereo audio processor");
  // if (addr == 0x44) Serial.println("TDA8442 - Interface for colour decoder");
  // if (addr == 0x46) Serial.println("TDA9150 - Deflection processor");
  // if ((addr >= 0x41) && (addr <= 0x40)) Serial.println("TDA9860 - Hi-fi audio processor");
  // if (addr == 0x60) Serial.println("TEA5767 - Radio receiver");
  // if (addr == 0x61) Serial.println("TEA6100 - FM/IF for computer-controlled radio");
  // if (addr == 0x40) Serial.println("TEA6300 - Sound fader control and preamplifier/source selector");
  // if (addr == 0x40) Serial.println("TEA6320 - 4-input tone/volume controller with fader control");
  // if (addr == 0x40) Serial.println("TEA6330 - Sound fader control circuit for car radios");
  // if ((addr >= 0x42) && (addr <= 0x45)) Serial.println("TMP006 - Infrared Thermopile Sensor in Chip-Scale Package");
  // if ((addr >= 0x42) && (addr <= 0x45)) Serial.println("TMP007 - IR Temperature sensor");
  // if ((addr >= 0x4b) && (addr <= 0x49)) Serial.println("TMP102 - Temperature sensor");
  // if (addr == 0x58) Serial.println("TPA2016 - 2.8-W/Ch Stereo Class-D Audio Amplifier With Dynamic Range Compression and Automatic Gain Control");
  // if ((addr >= 0x62) && (addr <= 0x60)) Serial.println("TSA5511 - 1.3 GHz PLL frequency synthesizer for TV");
  // if ((addr >= 0x39) && (addr <= 0x49)) Serial.println("TSL2561 - light sensor");
  // if (addr == 0x29) Serial.println("TSL2591 - light sensor");
  // if ((addr >= 0x62) && (addr <= 0x63)) Serial.println("UMA1014T - Low-power frequency synthesizer for mobile radio communications");

  Serial.println();
}

void displayResults()
{
  // Terminal Colours fo VT100, eg. Putty
  // \033[30m - black
  // \033[31m - red
  // \033[32m - green
  // \033[33m - yellow
  // \033[34m - blue
  // \033[35m - magenta
  // \033[36m - cyan
  // \033[37m - white
  // \033[39m - default

  char textbuffer[128];
  char tmpstr[128];

  strcpy(textbuffer, "   ");

  for (int i = 0; i < 0x10; i++)
  {
    sprintf(tmpstr, " %2x", i);
    strcat(textbuffer, tmpstr);
  }
  //Serial.print("\033[43m\033[30m");
  Serial.println(textbuffer);
  //Serial.print("\033[49m\033[39m");

  textbuffer[0] = 0; // Clear the buffer

  // Aligning display output to 16 byte intervals via integer math
  for(uint8_t addr = (I2C_RANGE_START) / 0x10 * 0x10;
      addr < (I2C_RANGE_END+1) / 0x10 * 0x10;
      addr++)
  {
    if (!(addr % 0x10)) // Start of a line
    {
      sprintf(textbuffer, "%02x:", addr / 0x10);
    }

    if (addr < SCAN_ADDRESS_START || addr > SCAN_ADDRESS_END)
    {
      sprintf(tmpstr, " %02s", "  ");
    }
    else if (deviceStatus[addr] == 0)
    {
      sprintf(tmpstr, " %02x", addr);
    }
    else if (deviceStatus[addr] == 4)
    {
      sprintf(tmpstr, " %02s", "??");
    }
    else
    {
      sprintf(tmpstr, " %02s", "--");
    }

    strcat(textbuffer, tmpstr);

    if (!((addr+1) % 0x10) && textbuffer[0] != 0)
    {
      Serial.println(textbuffer);
      textbuffer[0] = 0;
    }
  } 
}

void loop()
{
  for(uint8_t addr = SCAN_ADDRESS_START; addr <= SCAN_ADDRESS_END; addr++ ) 
  {
    // Address the device
    Wire.beginTransmission(addr);
    
    // Check for ACK (detection of device), NACK or error
    deviceStatus[addr] = Wire.endTransmission();
  }
  //Serial.write(12); // clear screen
  Serial.println();
  displayResults();
  Serial.println();

  for(uint8_t addr = SCAN_ADDRESS_START; addr <= SCAN_ADDRESS_END; addr++ ) 
  {
    if (deviceStatus[addr] == 0) {
      displayMatch(addr);
    }
  }
  
  delay(REPEAT_DELAY); // Pause between scans
}
