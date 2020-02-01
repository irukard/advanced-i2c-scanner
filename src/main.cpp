#include "Arduino.h"

//----------------------------------------------------------------------------
// This sketch scans valid i2c 7-bit addresses
// This does not scan for devices using 10-bit addresses
//----------------------------------------------------------------------------

#include <Wire.h>

#define MODE_WRITE_QUICK  true // Takes precedence over read-byte mode
#define MODE_READ_BYTE    false
#define SERIAL_BAUD_RATE   9600
#define REPEAT_DELAY       5000 // milliseconds
#define SDA_PIN D3
#define SCL_PIN D4

// By default use write-quick mode, but for these use read-byte mode
#define SCAN_ADDRESS_START       0x03
#define SCAN_ADDRESS_END         0x77

#define I2C_RANGE_START          0x00
#define I2C_RANGE_END            0x7F
#define READ_HINT_RANGE_1_START  0x30
#define READ_HINT_RANGE_1_END    0x37
#define READ_HINT_RANGE_2_START  0x50
#define READ_HINT_RANGE_2_END    0x5f

uint8_t deviceStatus[128]; // global

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial); // Wait for serial monitor
  Serial.println("I2C Bus Scanner");
  
  if (MODE_READ_BYTE || !MODE_WRITE_QUICK) {
    Serial.println();
    Serial.println("Note: Read-byte mode on Arduino may block indefinitely");
    Serial.println("      if a device is not capable of that mode.");
  }

  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  Wire.begin(SDA_PIN, SCL_PIN);
}

void displayResults()
{
  char textbuffer[128];
  char tmpstr[128];

  Serial.println();

  strcpy(textbuffer, "   ");

  for (int i = 0; i < 0x10; i++)
  {
    sprintf(tmpstr, " %2x", i);
    strcat(textbuffer, tmpstr);
  }

  Serial.println(textbuffer);

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
  Serial.println();
  Serial.println("Scanning: ");

  for(uint8_t addr = SCAN_ADDRESS_START; addr <= SCAN_ADDRESS_END; addr++ ) 
  {
    // Address the device
    Wire.beginTransmission(addr);
    
    // Read-byte mode: Request one byte from the device
    if (!MODE_WRITE_QUICK) {
      if ((MODE_READ_BYTE) || 
          (addr >= READ_HINT_RANGE_1_START && addr <= READ_HINT_RANGE_1_END) ||
          (addr >= READ_HINT_RANGE_2_START && addr <= READ_HINT_RANGE_2_END))
      {
        uint8_t throwaway = Wire.requestFrom(addr, (uint8_t)1);
      }
    }

    // Check for ACK (detection of device), NACK or error
    deviceStatus[addr] = Wire.endTransmission();

    Serial.print(".");
  }
  Serial.println();

  displayResults();

  delay(REPEAT_DELAY); // Pause between scans
}
