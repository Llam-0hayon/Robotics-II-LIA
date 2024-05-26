/* I2C Master Control FOR ROBOTICS II LIA

This is the master code for a system designed for the Elegoo Smart Car V4.0
The purpose of this code is to use I2C communication to have this master Arduino read the ID of the car in front of it using an RFID scanner.
Then depending on the code it scans, it will either print "Access Granted" or "Access Denied" to an LCD attached to the master board and car.
If access is granted, the master will send the numerical value "1" to the slave board.

LCD Pin Configuration:
  - LCD RS pin to digital pin 7
  - LCD Enable pin to digital pin 6
  - LCD D4 pin to digital pin 5
  - LCD D5 pin to digital pin 4
  - LCD D6 pin to digital pin 3
  - LCD D7 pin to digital pin 2

RFID Pin Configuration
RFID-RC522:                    Arduino:
 - SDA                         - D10
 - SCK                         - D13
 - MOSI                        - D11
 - MISO                        - D12
 - RST                         - D9

I2C Pin Configuration
Master:                    Slave:
- SDA                      - SDA
- SCL                      - SCL
*/

/*_______I2C INITIALIZATION & VARIABLES______*/
#include <Wire.h>              // Include Arduino Wire library for I2C
#define SLAVE_ADDR 9           // Define Slave I2C Address

/*_______RFID INITIALIZATION & VARIABLES______*/
#include <SPI.h>               // Include SPI library
#include <MFRC522.h>           // Include MFRC522 library
#define RST_PIN 9              // Configurable reset pin
#define SS_PIN 10              // Configurable slave select pin
MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance

// UID codes of different people/cards
byte accessUIDme[4] = {0x8A, 0x14, 0x62, 0x16};           // Code of accepted RFID card
byte accessUIDdefaultcard[4] = {0x43, 0x69, 0x8B, 0x1A};  // Default card UID
byte accessUIDalben[4] = {0xAA, 0x74, 0xE3, 0x36};        // Alben's card UID
byte accessUIDartin[4] = {0x4A, 0x77, 0xD7, 0x36};        // Artin's card UID

/*_______LCD INITIALIZATION & VARIABLES______*/
#include <LiquidCrystal.h>      // Includes LiquidCrystal library which is used to program the LCD
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2; // Defines the LCD pin configuration
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                // Initializes LiquidCrystal object and defines the pins used on the LCD

void setup() {
  /*_______SERIAL MONITOR SETUP______*/
  Serial.begin(9600);                                // Initialize serial communications with the PC
  Serial.println("I2C Master");                      // Print "I2C Master" to the serial monitor

  /*_______RFID SETUP______*/
  while (!Serial);                                   // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
  SPI.begin();                                       // Initialize SPI bus
  mfrc522.PCD_Init();                                // Initialize MFRC522
  delay(4);                                          // Optional delay. Some boards need more time after init to be ready
  mfrc522.PCD_DumpVersionToSerial();                 // Show details of PCD - MFRC522 Card Reader details
  Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));

  /*_______I2C SETUP______*/
  Wire.begin();                                      // Initialize I2C communications as Master

  /*_______LCD SETUP______*/
  lcd.begin(16, 2);                                  // Define how many rows and columns there are on the LCD being used. The first row and column are row 0 and column 0.
}

void loop() {
  /*_______DETECTING RFID TAG______*/
  // Reset the loop if no new card is present on the sensor/reader. This saves the entire process when idle.
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return;                                          // Exit the loop if no new card is present
  }

  // Select one of the cards
  if (!mfrc522.PICC_ReadCardSerial()) {
    return;                                          // Exit the loop if the card cannot be read
  }

  /*_______GRANT OR DENY ACCESS______*/
  // Check if the scanned UID matches any of the predefined access UIDs
  if ((mfrc522.uid.uidByte[0] == accessUIDme[0] && mfrc522.uid.uidByte[1] == accessUIDme[1] && mfrc522.uid.uidByte[2] == accessUIDme[2] && mfrc522.uid.uidByte[3] == accessUIDme[3]) ||
      (mfrc522.uid.uidByte[0] == accessUIDdefaultcard[0] && mfrc522.uid.uidByte[1] == accessUIDdefaultcard[1] && mfrc522.uid.uidByte[2] == accessUIDdefaultcard[2] && mfrc522.uid.uidByte[3] == accessUIDdefaultcard[3]) ||
      (mfrc522.uid.uidByte[0] == accessUIDalben[0] && mfrc522.uid.uidByte[1] == accessUIDalben[1] && mfrc522.uid.uidByte[2] == accessUIDalben[2] && mfrc522.uid.uidByte[3] == accessUIDalben[3]) ||
      (mfrc522.uid.uidByte[0] == accessUIDartin[0] && mfrc522.uid.uidByte[1] == accessUIDartin[1] && mfrc522.uid.uidByte[2] == accessUIDartin[2] && mfrc522.uid.uidByte[3] == accessUIDartin[3])) {
    Serial.println("Access Granted");                // Print "Access Granted" to the serial monitor

    /*_______TRANSMIT Granted RFID CODE TO SLAVE______*/
    Wire.beginTransmission(SLAVE_ADDR);              // Begin I2C transmission to slave device
    Wire.write(1);                                   // Send the numerical value "1" to indicate access granted
    Wire.endTransmission();                          // End the I2C transmission
  } else {
    // Print "Access Denied" to the serial monitor
    Serial.println("Access Denied");

    // Print "Access Denied" to the LCD
    lcd.setCursor(0, 0);                            // Set cursor to the first column of the first row
    lcd.print("Access Denied");                     // Print "Access Denied" on the LCD
    delay(1000);                                    // Wait for 1 second
    lcd.clear();                                    // Clear the LCD
    lcd.setCursor(0, 0);                            // Set cursor to the first column of the first row
    lcd.print("Sorry :(");                          // Print "Sorry :(" on the LCD
    delay(1000);                                    // Wait for 1 second
    lcd.clear();                                    // Clear the LCD

    /*_______TRANSMIT Denied RFID CODE TO SLAVE______*/
    Wire.beginTransmission(SLAVE_ADDR);             // Begin I2C transmission to slave device
    Wire.write(2);                                  // Send the numerical value "2" to indicate access denied
    Wire.endTransmission();                         // End the I2C transmission
  }

  mfrc522.PICC_HaltA();                             // Halt the currently active PICC (Proximity Integrated Circuit Card)

  /*_______Print Who Is getting Access To LCD______*/ 
  // LCD Print for Liam (accessUIDme)
  if (mfrc522.uid.uidByte[0] == accessUIDme[0] && mfrc522.uid.uidByte[1] == accessUIDme[1] && mfrc522.uid.uidByte[2] == accessUIDme[2] && mfrc522.uid.uidByte[3] == accessUIDme[3]) {
    lcd.setCursor(0, 0);                            // Set cursor to the first column of the first row
    lcd.print("Access Granted");                    // Print "Access Granted" on the LCD
    delay(1000);                                    // Wait for 1 second
    lcd.clear();                                    // Clear the LCD
    lcd.setCursor(0, 0);                            // Set cursor to the first column of the first row
    lcd.print("Welcome Liam");                      // Print "Welcome Liam" on the LCD
    lcd.setCursor(0, 1);                            // Set cursor to the first column of the second row
    lcd.print("Student: 6225410");                  // Print "Student: 6225410" on the LCD
    delay(1000);                                    // Wait for 1 second
    lcd.clear();                                    // Clear the LCD
  }

  // LCD Print for Default Card (accessUIDdefaultcard)
  if (mfrc522.uid.uidByte[0] == accessUIDdefaultcard[0] && mfrc522.uid.uidByte[1] == accessUIDdefaultcard[1] && mfrc522.uid.uidByte[2] == accessUIDdefaultcard[2] && mfrc522.uid.uidByte[3] == accessUIDdefaultcard[3]) {
    lcd.setCursor(0, 0);                            // Set cursor to the first column of the first row
    lcd.print("Access Granted");                    // Print "Access Granted" on the LCD
    delay(1000);                                    // Wait for 1 second
    lcd.clear();                                    // Clear the LCD
    lcd.setCursor(0, 0);                            // Set cursor to the first column of the first row
    lcd.print("Welcome John Doe");                  // Print "Welcome John Doe" on the LCD
    lcd.setCursor(0, 1);                            // Set cursor to the first column of the second row
    lcd.print("Student: N/A");                      // Print "Student: N/A" on the LCD
    delay(1000);                                    // Wait for 1 second
    lcd.clear();                                    // Clear the LCD
  }

  // LCD Print for Alben (accessUIDalben)
  if (mfrc522.uid.uidByte[0] == accessUIDalben[0] && mfrc522.uid.uidByte[1] == accessUIDalben[1] && mfrc522.uid.uidByte[2] == accessUIDalben[2] && mfrc522.uid.uidByte[3] == accessUIDalben[3]) {
    lcd.setCursor(0, 0);                            // Set cursor to the first column of the first row
    lcd.print("Access Granted");                    // Print "Access Granted" on the LCD
    delay(1000);                                    // Wait for 1 second
    lcd.clear();                                    // Clear the LCD
    lcd.setCursor(0, 0);                            // Set cursor to the first column of the first row
    lcd.print("Welcome Alben");                     // Print "Welcome Alben" on the LCD
    lcd.setCursor(0, 1);                            // Set cursor to the first column of the second row
    lcd.print("Student: Unknown");                  // Print "Student: Unknown" on the LCD
    delay(1000);                                    // Wait for 1 second
    lcd.clear();                                    // Clear the LCD
  }

  // LCD Print for Artin (accessUIDartin)
  if (mfrc522.uid.uidByte[0] == accessUIDartin[0] && mfrc522.uid.uidByte[1] == accessUIDartin[1] && mfrc522.uid.uidByte[2] == accessUIDartin[2] && mfrc522.uid.uidByte[3] == accessUIDartin[3]) {
    lcd.setCursor(0, 0);                            // Set cursor to the first column of the first row
    lcd.print("Access Granted");                    // Print "Access Granted" on the LCD
    delay(1000);                                    // Wait for 1 second
    lcd.clear();                                    // Clear the LCD
    lcd.setCursor(0, 0);                            // Set cursor to the first column of the first row
    lcd.print("Welcome Artin");                     // Print "Welcome Artin" on the LCD
    lcd.setCursor(0, 1);                            // Set cursor to the first column of the second row
    lcd.print("Student: Unknown");                  // Print "Student: Unknown" on the LCD
    delay(1000);                                    // Wait for 1 second
    lcd.clear();                                    // Clear the LCD
  }
}
