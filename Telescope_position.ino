/* ///////////////////////////////////////////////////////////////////////////////////
  Telescope 2 Stellarium, by Nelson Ferraz
  Created: July  2016     V1.0

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
  /////////////////////////////////////////////////////////////////////////////////*/

#include "TelescopeConfig.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DueFlashStorage.h>  

// Non-volatile memory for the Due.  Standard ATMega library doesn't work.
//
DueFlashStorage dueFlashStorage;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Declarations for astronomy
//
unsigned long     seg_sideral = 1003;
const double      pi = 3.14159265358979324;
volatile int      lastEncoded1 = 0;
volatile long     encoderValue1 = 0;
volatile int      lastEncoded2 = 0;
volatile long     encoderValue2 = 0;
volatile double   refAltPulses = 0;
volatile double   refAzPulses = 0;

char  input[20];
char  txAR[10];
char  txDEC[11];
long  TSL = 0;
long  AltFactor;
long  AzFactor;
unsigned long t_ciclo_acumulado = 0, t_ciclo;
long    Az_tel_s, Alt_tel_s;
long    AR_tel_s, DEC_tel_s;
long    AR_stell_s, DEC_stell_s;
double  phi_rad,cos_phi_rad, sin_phi_rad;
int     hemCorrectLat;

// Declarations for menu displays
char  OLED_EQ_AR[16];  // Telescope position in equatorial coordinates
char  OLED_EQ_DEC[16];
char  OLED_HO_ALT[16]; // Telescope position in horizontal coordinates
char  OLED_HO_AZ[16];
char  OLED_Line_BEG[16];  // General usage
char  OLED_Line_MID[16];
char  OLED_Line_END[16];

// Declarations for menu system
//
volatile int encoderPos = 0; //tStores the current encoder position.
volatile int oldEncPos = 0; // Stores the last encoder position

// Button reading, including debounce without delay function declarations
//
byte oldButtonState = HIGH;  // assume switch open because of pull-up resistor
const unsigned long debounceTime = 10;  // milliseconds
unsigned long buttonPressTime;  // when the switch last changed state

// Menu and submenu/setting declarations
//
byte setting1 = 0;  // a variable which holds the value we set 
byte setting2 = 0;  // a variable which holds the value we set 
byte setting3 = 0;  // a variable which holds the value we set 

enum menuItems {
  TEL_HORIZONTAL,   // Current horizontal coordinates of our telescope
  TEL_EQUATORIAL,   // Current equatorial coordinates of our telescope
  TEL_LST,          // Local Sidereal Time at our observing position
  REF_AZIMUTH,      // Azimuth of our reference star
  REF_ALTITUDE,     // Altitude of our reference star
  TEL_LATITUDE,     // Latitude of our observing position
  SAVE_EEPROM,      // Save latitude, LST, star RA and star DEC to EEPROM memory
  SHOW_ENCODERS     // For debugging use, show encoder count values. THIS WILL MAKE THE COORDINATES CALCULATE INCORRECTLY
};
enum menuItems menuItem;

enum menuModes {
  SHOW,             // Menu is in display mode
  EDIT              // Menu is in edit mode
};
enum menuModes menuMode;

enum editingFields {
  HEMISPHERE,       // Editing hemisphere
  VALUE1,           // Editing Field 1
  VALUE2,           // Editing Field 2
  VALUE3,           // Editing Field 3
  CONFIRMATION      // Confirmation screen, yes or no
};
enum editingFields editingField;
int value1temp = 0;
int value2temp = 0;
int value3temp = 0;

enum yesOrNo {
  YES,            // Just started editing this item
  NO              // Have been editing this item on previous cycles
};
enum yesOrNo firstEdit;
enum yesOrNo confirmation;
enum yesOrNo newEncPos;
enum yesOrNo buttonPressed;

enum saveStates {
  WAITING,
  SAVED,
  CANCELLED
};
enum saveStates saveState;

enum hemispheres hemTemp;

// For encoder input processing
static uint8_t  prevNextCode = 0;
static uint16_t store=0;
static int8_t   val;
int             encoderMax = 1;
int             encoderMin = 0;

// For button debounce.  Replicate for each button that needs debouncing
char menuButtonState = 0;           // state of button
unsigned long menuButtonCount = 0;  // button debounce timer

// For updating OLED display
long previousMillis;
long oledUpdateTime = 100;  // OLED update time in milliseconds


//--------------------------------------------------------------------------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(9600);
  pinMode(enc_1A, INPUT_PULLUP);
  pinMode(enc_1B, INPUT_PULLUP);
  pinMode(enc_2A, INPUT_PULLUP);
  pinMode(enc_2B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc_1A), Encoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_1B), Encoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_2A), Encoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_2B), Encoder2, CHANGE);

  // Retrieve stored data from EEPROM
  //
  if (use_saved_data) getFromEEPROM();

  // Pre-calculate latitude data for telescope position
  //
  if (latHem == NORTH) hemCorrectLat = lat_HH;
  else hemCorrectLat = -lat_HH;
  phi_rad = (((hemCorrectLat * 3600) + (lat_MM * 60) + lat_SS) / 3600.0) * pi / 180.0;
  cos_phi_rad = cos(phi_rad);
  sin_phi_rad = sin(phi_rad);

  // Calculate local sidereal time from stored values
  //
  TSL = LST_HH * 3600 + LST_MM * 60 + LST_SS;
  while (TSL >= 86400) TSL = TSL - 86400;                

  // Calculate starting point for the encoders using the reference star data
  //
  reference_coords();
  encoderValue1 = (long) refAltPulses;  // Set the pulse count of encoder 1 to the reference star Altitude
  encoderValue2 = (long) refAzPulses;   // Set the pulse count of encoder 2 to the reference star Azimuth

  // Initialize OLED update timer
  //
  previousMillis = millis();

  // Menu system encoder setup
  //
  pinMode(enc_mA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(enc_mB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode (enc_mButton, INPUT_PULLUP); // setup the button pin

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  //
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    for(;;); // Don't proceed, loop forever
  }
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------
void loop()
{

  // Update local sidereal time with the time it took to execute the last loop()
  //
  t_ciclo = millis();
  if (t_ciclo_acumulado >= seg_sideral) {
    TSL++;
    t_ciclo_acumulado = t_ciclo_acumulado - seg_sideral;
    if (TSL >= 86400) {
      TSL = TSL - 86400;
    }
  }

  // Process the encoder data into equatorial coordinates
  //
  read_sensors();
  AZ_to_EQ();
  
  // Stellarium communication
  //
  if (Serial.available() > 0) {
    communication();
  }

  // Menu system
  //
  if( val=EncoderM() ) {
    encoderPos +=val;
  }
  rotaryMenu();
    
  // Update OLED display
  //
  if (abs(millis() - previousMillis) > oledUpdateTime) {
    update_OLED();
    previousMillis = millis();
  }

  // Record how long it took to run the main loop
  //
  t_ciclo = millis() - t_ciclo;
  t_ciclo_acumulado = t_ciclo_acumulado + t_ciclo;

}  // End loop


// Serial comms to Stellarium
//
void communication()
{
  int i = 0;
  input[i++] = Serial.read();
  delay(5);
  while ((input[i++] = Serial.read()) != '#') {
    delay(5);
  }
  input[i] = '\0';

  if (input[1] == ':' && input[2] == 'G' && input[3] == 'R' && input[4] == '#') {
    Serial.print(txAR);
  }

  if (input[1] == ':' && input[2] == 'G' && input[3] == 'D' && input[4] == '#') {
    Serial.print(txDEC);
  }
}

// Calculate altitude and azimuth from encoder pulse counts
//
void read_sensors() {
  long h_deg, h_min, h_seg, A_deg, A_min, A_seg;

  if (show_encoders == 0) {  // Show encoders is for debugging ONLY. If set, this will make coordinates calculate INCORRECTLY
    if (encoderValue2 >= pulses_enc2 || encoderValue2 <= -pulses_enc2) {
      encoderValue2 = 0;
    }
  }

  // There are 1296000 arc/sec per 360º
  //
  int enc1 = encoderValue1 / 1500;
  long encoder1_temp = encoderValue1 - (enc1 * 1500);
  long map1 = enc1 * map(1500, 0, pulses_enc1, 0, 1296000);
  int enc2 = encoderValue2 / 1500;
  long encoder2_temp = encoderValue2 - (enc2 * 1500);
  long map2 = enc2 * map(1500, 0, pulses_enc2, 0, 1296000);

  Alt_tel_s = map1 + map (encoder1_temp, 0, pulses_enc1, 0, 1296000);
  Az_tel_s  = map2 + map (encoder2_temp, 0, pulses_enc2, 0, 1296000);

  if (Az_tel_s < 0) Az_tel_s = 1296000 + Az_tel_s;
  if (Az_tel_s >= 1296000) Az_tel_s = Az_tel_s - 1296000 ;
}

// Process the Altitude rotary encoder input
//
void Encoder1() {
  int encoded1 = (digitalRead(enc_1A) << 1) | digitalRead(enc_1B);
  int sum  = (lastEncoded1 << 2) | encoded1;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue1 ++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue1 --;
  lastEncoded1 = encoded1;
}

// Process the Azimuth rotary encoder input
//
void Encoder2() {
  int encoded2 = (digitalRead(enc_2A) << 1) | digitalRead(enc_2B);
  int sum  = (lastEncoded2 << 2) | encoded2;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue2 ++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue2 --;
  lastEncoded2 = encoded2;
}

// Process the Menu rotary encoder input
// Debounce is implemented using a lookup table approach
// A vald CW or  CCW move returns 1, invalid returns 0.
//
int8_t EncoderM() {
  static int8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

  prevNextCode <<= 2;
  if (digitalRead(enc_mA)) prevNextCode |= 0x01;
  if (digitalRead(enc_mB)) prevNextCode |= 0x02;

  prevNextCode &= 0x0f;

   // If valid then store as 16 bit data.
   if  (rot_enc_table[prevNextCode] ) {
      store <<= 4;
      store |= prevNextCode;
      if ((store&0xff)==0x2b) return -1;
      if ((store&0xff)==0x17) return 1;
   }
   return 0;
}


// Calculate equitorial coordinates from horizontal coordinates
//
void AZ_to_EQ()
{
  double delta_tel, sin_h, cos_h, sin_A, cos_A, sin_DEC, cos_DEC;
  double H_telRAD, h_telRAD, A_telRAD;
  long H_tel;
  long arHH, arMM, arSS;
  long decDEG, decMM, decSS;
  char sDEC_tel;
  long altHH, altMM, altSS;
  long azDEG, azMM, azSS;

  A_telRAD = (Az_tel_s / 3600.0) * pi / 180.0;
  h_telRAD = (Alt_tel_s / 3600.0) * pi / 180.0;
  sin_h = sin(h_telRAD);
  cos_h = cos(h_telRAD);
  sin_A = sin(A_telRAD);
  cos_A = cos(A_telRAD);
  delta_tel = asin((sin_phi_rad * sin_h) + (cos_phi_rad * cos_h * cos_A));
  sin_DEC = sin(delta_tel);
  cos_DEC = cos(delta_tel);
  DEC_tel_s = long((delta_tel * 180.0 / pi) * 3600.0);

  while (DEC_tel_s >= 324000) {
    DEC_tel_s = DEC_tel_s - 324000;
  }
  while (DEC_tel_s <= -324000) {
    DEC_tel_s = DEC_tel_s + 324000;
  }

  H_telRAD = acos((sin_h - (sin_phi_rad * sin_DEC)) / (cos_phi_rad * cos_DEC));
  H_tel = long((H_telRAD * 180.0 / pi) * 240.0);

  if (sin_A >= 0) {
    H_tel = 86400 - H_tel;
  }
  AR_tel_s = TSL - H_tel;

  while (AR_tel_s >= 86400) {
    AR_tel_s = AR_tel_s - 86400;
  }
  while (AR_tel_s < 0) {
    AR_tel_s = AR_tel_s + 86400;
  }

  // Convert decimal equatorial coordinate data to hours, minutes, seconds
  arHH = AR_tel_s / 3600L;
  arMM = (AR_tel_s - arHH * 3600L) / 60L;
  arSS = (AR_tel_s - arHH * 3600L) - arMM * 60L;
  decDEG = abs(DEC_tel_s) / 3600L;
  decMM = (abs(DEC_tel_s) - decDEG * 3600L) / 60L;
  decSS = (abs(DEC_tel_s) - decDEG * 3600L) - decMM * 60L;
  (DEC_tel_s < 0) ? sDEC_tel = 45 : sDEC_tel = 43;

  // Equatorial coordinate data for Stellarium comms
  //
  sprintf(txAR, "%02d:%02d:%02d#", int(arHH), int(arMM), int(arSS));
  sprintf(txDEC, "%c%02d%c%02d:%02d#", sDEC_tel, int(decDEG), 223, int(decMM), int(decSS));

  // Equatorial coordinate data for OLED display
  //
  if (show_encoders) {  // For debugging ONLY. If set, this will make coordinates calculate INCORRECTLY
    sprintf(OLED_EQ_AR,   "RA  INVALID");
    sprintf(OLED_EQ_DEC,  "DEC INVALID");
  } else {
    sprintf(OLED_EQ_AR,  "RA   %02d:%02d:%02d", int(arHH), int(arMM), int(arSS));
    sprintf(OLED_EQ_DEC, "DEC %c%02d'%02d:%02d", sDEC_tel, int(decDEG), int(decMM), int(decSS));
  }
  
  // Convert decimal horizontal coordinate data to hours, minutes, seconds
  //
  altHH = Alt_tel_s / 3600;
  altMM = (Alt_tel_s - altHH * 3600) / 60;
  altSS = (Alt_tel_s - altHH * 3600) - altMM * 60;
  azDEG = abs(Az_tel_s) / 3600;
  azMM = (abs(Az_tel_s) - azDEG * 3600) / 60;
  azSS = (abs(Az_tel_s) - azDEG * 3600) - azMM * 60;

  // Horizontal coordinate data for OLED display
  //
  if (show_encoders) {  // For debugging ONLY. If set, this will make coordinates calculate INCORRECTLY
    sprintf(OLED_HO_ALT,  "Alt INVALID");
    sprintf(OLED_HO_AZ,   "Az  INVALID");
  } else {  
    sprintf(OLED_HO_ALT, "Alt %02d:%02d:%02d", int(altHH), int(altMM), int(altSS));
    sprintf(OLED_HO_AZ,  "Az  %02d:%02d:%02d", int(azDEG), int(azMM), int(azSS));
  }
}


// State machine logic for menu system
//
void rotaryMenu() {
  
  // Check for new encoder position
  //
  if(oldEncPos != encoderPos) {
    newEncPos = YES;
    oldEncPos = encoderPos;
  } else newEncPos = NO;

  // Read and debounce the encoder pushbutton
  //
  if (buttonDown(digitalRead(enc_mButton), &menuButtonCount, &menuButtonState, 10UL )) {  // debounce timer is 10 msec
    buttonPressed = YES;
  } else buttonPressed = NO;
  
  // If in display mode, use the encoder to cycle through the menu items
  //
  if (menuMode == SHOW) {

    if (show_encoders) encoderMax = SHOW_ENCODERS;
    else encoderMax = SAVE_EEPROM;
    encoderMin = 0;
    
    if (encoderPos < encoderMin) encoderPos = encoderMin;  
    if (encoderPos > encoderMax) encoderPos = encoderMax;  // Ensure encoder position does not exceed the maximum
    switch (encoderPos) {
      case 0:
        menuItem = TEL_HORIZONTAL;
        break;
      case 1:
        menuItem = TEL_EQUATORIAL;
        break;
      case 2:
        menuItem = TEL_LST;
        TSL_dec_to_HMS();
        break;
      case 3:
        menuItem = REF_AZIMUTH;
        break;
      case 4:
        menuItem = REF_ALTITUDE;
        break;
      case 5:
        menuItem = TEL_LATITUDE;
        break;
      case 6:
        menuItem = SAVE_EEPROM;
        break;
      case 7:
        menuItem = SHOW_ENCODERS;
        break;
    }
    if (buttonPressed == YES) {
      menuMode = EDIT;
      buttonPressed = NO;
    }
    
  } // End of display mode
  
  // Perform the necessary edit actions for each menu item
  //
  if (menuMode == EDIT) {
    switch (menuItem) {
      case TEL_HORIZONTAL:    // There is nothing to edit here
        encoderPos = menuItem;
        menuMode = SHOW;      // Change back to display mode
        break;

      case TEL_EQUATORIAL:    // There is nothing to edit here
        encoderPos = menuItem;
        menuMode = SHOW;      // Change back to display mode
        break;

      case TEL_LST:    // There are three fields to edit plus a confirmation
        switch (editingField) {
          case HEMISPHERE:    // Nothing to edit here
            editingField = VALUE1;  // Skip straight to the next field
            break;
 
          case VALUE1:    // LST hours
            if (firstEdit == YES) {
              TSL_dec_to_HMS();
              encoderPos = LST_HH;
              value1temp = LST_HH;
              value2temp = LST_MM;
              value3temp = LST_SS;
              firstEdit = NO;
            }
            encoderMin = 0;
            encoderMax = 23;
            if (encoderPos < encoderMin) encoderPos = encoderMin;  
            if (encoderPos > encoderMax) encoderPos = encoderMax;  // Ensure encoder position does not exceed the maximum
            value1temp = encoderPos;
            if (buttonPressed == YES) {
              editingField = VALUE2;
              firstEdit = YES;
            }
            break; // End of editing value 1
            
          case VALUE2:    // LST minutes
            if (firstEdit == YES) {
              encoderPos = LST_MM;
              firstEdit = NO;
            }
            encoderMin = 0;
            encoderMax = 59;
            if (encoderPos < encoderMin) encoderPos = encoderMin;  
            if (encoderPos > encoderMax) encoderPos = encoderMax;  // Ensure encoder position does not exceed the maximum
             value2temp = encoderPos;
            if (buttonPressed == YES) {  // Start editing value 3
              editingField = VALUE3;
              firstEdit = YES;
            }
            break; // End of editing value 2
            
          case VALUE3:    // LST seconds
            if (firstEdit == YES) {
              encoderPos = LST_SS;
              firstEdit = NO;
            }
            encoderMin = 0;
            encoderMax = 59;
            if (encoderPos < encoderMin) encoderPos = encoderMin;  
            if (encoderPos > encoderMax) encoderPos = encoderMax;  // Ensure encoder position does not exceed the maximum
            value3temp = encoderPos;
            if (buttonPressed == YES) {  // Move on to confirmation
              editingField = CONFIRMATION;
              firstEdit = YES;
            }
            break; // End of editing value 3
            
          case CONFIRMATION:
            if (firstEdit == YES) {
              encoderPos = 0;
              firstEdit = NO;
            }
            encoderMin = 0;
            encoderMax = 1;
            if (encoderPos < encoderMin) encoderPos = encoderMin;  
            if (encoderPos > encoderMax) encoderPos = encoderMax;  // Ensure encoder position does not exceed the maximum
            switch (encoderPos) {
              case 0:
                confirmation = NO;
                break;
              case 1:
                confirmation = YES;
                break;
            }
            if (buttonPressed == YES) {  // We are finished editing
              if (confirmation == YES) {
                // Copy the new values from temporary into live variables
                //
                LST_HH = value1temp;
                LST_MM = value2temp;
                LST_SS = value3temp;

                // Convert LST variables into TSL live value
                //
                TSL = LST_HH * 3600 + LST_MM * 60 + LST_SS;
                while (TSL >= 86400) TSL = TSL - 86400;
              }
              menuMode = SHOW;
              editingField = HEMISPHERE;
              firstEdit = YES;
              encoderPos = menuItem;
            }
            break; // End of confirmation
        }
        break; // End of editing LST

      case REF_AZIMUTH:    // There are three fields to edit plus a confirmation
        switch (editingField) {
          case HEMISPHERE:
            editingField = VALUE1;  // Skip straight to the next field
            break;
            
          case VALUE1:    // Reference star azimuth degrees
            if (firstEdit == YES) {
              encoderPos = refAZ_DD;
              value1temp = refAZ_DD;
              value2temp = refAZ_MM;
              value3temp = refAZ_SS;
              firstEdit = NO;
            }
            encoderMin = 0;
            encoderMax = 360;
            if (encoderPos < encoderMin) encoderPos = encoderMin;  
            if (encoderPos > encoderMax) encoderPos = encoderMax;  // Ensure encoder position does not exceed the maximum
            value1temp = encoderPos;
            if (buttonPressed == YES) {  // Start editing value 2
              editingField = VALUE2;
              firstEdit = YES;
            }
            break; // End of editing value 1
            
          case VALUE2:    // Reference star azimuth minutes
            if (firstEdit == YES) {
              encoderPos = refAZ_MM;
              firstEdit = NO;
            }
            encoderMin = 0;
            encoderMax = 59;
            if (encoderPos < encoderMin) encoderPos = encoderMin;  
            if (encoderPos > encoderMax) encoderPos = encoderMax;  // Ensure encoder position does not exceed the maximum
            value2temp = encoderPos;
            if (buttonPressed == YES) {  // Start editing value 3
              editingField = VALUE3;
              firstEdit = YES;
            }
            break; // End of editing value 2
            
          case VALUE3:    // Reference star azimuth seconds
            if (firstEdit == YES) {
              encoderPos = refAZ_SS;
              firstEdit = NO;
            }
            encoderMin = 0;
            encoderMax = 59;
            if (encoderPos < encoderMin) encoderPos = encoderMin;  
            if (encoderPos > encoderMax) encoderPos = encoderMax;  // Ensure encoder position does not exceed the maximum
            value3temp = encoderPos;
            if (buttonPressed == YES) {  // Move on to confirmation
              editingField = CONFIRMATION;
              firstEdit = YES;
            }
            break; // End of editing value 3
            
          case CONFIRMATION:
            if (firstEdit == YES) {
              encoderPos = 0;
              firstEdit = NO;
            }
            encoderMin = 0;
            encoderMax = 1;
            if (encoderPos < encoderMin) encoderPos = encoderMin;  
            if (encoderPos > encoderMax) encoderPos = encoderMax;  // Ensure encoder position does not exceed the maximum
            switch (encoderPos) {
              case 0:
                confirmation = NO;
                break;
              case 1:
                confirmation = YES;
                break;
            }
            if (buttonPressed == YES) {  // We are finished editing
              if (confirmation == YES) {
                // Copy the new values from temporary into live variables
                refAZ_DD = value1temp;
                refAZ_MM = value2temp;
                refAZ_SS = value3temp;
              }
              menuMode = SHOW;
              editingField = HEMISPHERE;
              firstEdit = YES;
              encoderPos = menuItem;
            }
            break; // End of confirmation
        }
        break; // End of editing reference star azimuth
        
      case REF_ALTITUDE:    // There are three fields to edit plus a confirmation
        switch (editingField) {
          case HEMISPHERE:      // Skip straight to the next field
            editingField = VALUE1;
            break;
            
          case VALUE1:    // Reference star altitude degrees
            if (firstEdit == YES) {
              encoderPos = refAlt_DD;
              value1temp = refAlt_DD;
              value2temp = refAlt_MM;
              value3temp = refAlt_SS;
              firstEdit = NO;
            }
            encoderMin = 0;
            encoderMax = 90;
            if (encoderPos < encoderMin) encoderPos = encoderMin;  
            if (encoderPos > encoderMax) encoderPos = encoderMax;  // Ensure encoder position does not exceed the maximum
            value1temp = encoderPos;
            if (buttonPressed == YES) {  // Start editing value 2
              editingField = VALUE2;
              firstEdit = YES;
            }
            break; // End of editing value 1
            
          case VALUE2:    // Reference star altitude minutes
            if (firstEdit == YES) {
              encoderPos = refAlt_MM;
              firstEdit = NO;
            }
            encoderMin = 0;
            encoderMax = 59;
            if (encoderPos < encoderMin) encoderPos = encoderMin;  
            if (encoderPos > encoderMax) encoderPos = encoderMax;  // Ensure encoder position does not exceed the maximum
            value2temp = encoderPos;
            if (buttonPressed == YES) {  // Start editing value 3
              editingField = VALUE3;
              firstEdit = YES;
            }
            break; // End of editing value 2
            
          case VALUE3:    // Reference star altitude seconds
            if (firstEdit == YES) {
              encoderPos = refAlt_SS;
              firstEdit = NO;
            }
            encoderMin = 0;
            encoderMax = 59;
            if (encoderPos < encoderMin) encoderPos = encoderMin;  
            if (encoderPos > encoderMax) encoderPos = encoderMax;  // Ensure encoder position does not exceed the maximum
            value3temp = encoderPos;
            if (buttonPressed == YES) {  // Move on to confirmation
              editingField = CONFIRMATION;
              firstEdit = YES;
            }
            break; // End of editing value 3
            
          case CONFIRMATION:
            if (firstEdit == YES) {
              encoderPos = 0;
              firstEdit = NO;
            }
            encoderMin = 0;
            encoderMax = 1;
            if (encoderPos < encoderMin) encoderPos = encoderMin;  
            if (encoderPos > encoderMax) encoderPos = encoderMax;  // Ensure encoder position does not exceed the maximum
            switch (encoderPos) {
              case 0:
                confirmation = NO;
                break;
              case 1:
                confirmation = YES;
                break;
            }
            if (buttonPressed == YES) {  // We are finished editing
              if (confirmation == YES) {
                // Copy the new values from temporary into live variables
                refAlt_DD = value1temp;
                refAlt_MM = value2temp;
                refAlt_SS = value3temp;
              }
              menuMode = SHOW;
              editingField = HEMISPHERE;
              firstEdit = YES;
              encoderPos = menuItem;
            }
            break; // End of confirmation
      }
      break; // End of editing reference star altitude 

      case TEL_LATITUDE:    // There are three fields to edit plus a confirmation
        switch (editingField) {
          case HEMISPHERE:    // Latitude hemisphere
            if (firstEdit == YES) {
              switch (latHem) {
                case NORTH:
                  encoderPos = 0;
                  break;
                case SOUTH:
                  encoderPos = 1;
                  break;
              }              
              value1temp = lat_HH;
              value2temp = lat_MM;
              value3temp = lat_SS;
              firstEdit = NO;
            }
            encoderMin = 0;
            encoderMax = 1;
            if (encoderPos < encoderMin) encoderPos = encoderMin;  
            if (encoderPos > encoderMax) encoderPos = encoderMax;  // Ensure encoder position does not exceed the maximum
            switch (encoderPos) {
              case 0:
                hemTemp = NORTH;
                break;
              case 1:
                hemTemp = SOUTH;
                break;
            }
            if (buttonPressed == YES) {  // We are finished editing
              editingField = VALUE1;
              firstEdit = YES;
            }
            break; // End of hemisphere
 
          case VALUE1:    // Latitude hours
            if (firstEdit == YES) {
              encoderPos = lat_HH;  // Valid range is 0 to 90
              firstEdit = NO;
            }
            encoderMin = 0;
            encoderMax = 89;
            if (encoderPos < encoderMin) encoderPos = encoderMin;  
            if (encoderPos > encoderMax) encoderPos = encoderMax;  // Ensure encoder position does not exceed the maximum
            value1temp = encoderPos;
            if (buttonPressed == YES) {
              editingField = VALUE2;
              firstEdit = YES;
            }
            break; // End of editing value 1
            
          case VALUE2:    // Latitude minutes
            if (firstEdit == YES) {
              encoderPos = lat_MM;
              firstEdit = NO;
            }
            encoderMin = 0;
            encoderMax = 59;
            if (encoderPos < encoderMin) encoderPos = encoderMin;  
            if (encoderPos > encoderMax) encoderPos = encoderMax;  // Ensure encoder position does not exceed the maximum
             value2temp = encoderPos;
            if (buttonPressed == YES) {  // Start editing value 3
              editingField = VALUE3;
              firstEdit = YES;
            }
            break; // End of editing value 2
            
          case VALUE3:    // Latitude seconds
            if (firstEdit == YES) {
              encoderPos = lat_SS;
              firstEdit = NO;
            }
            encoderMin = 0;
            encoderMax = 59;
            if (encoderPos < encoderMin) encoderPos = encoderMin;  
            if (encoderPos > encoderMax) encoderPos = encoderMax;  // Ensure encoder position does not exceed the maximum
            value3temp = encoderPos;
            if (buttonPressed == YES) {  // Move on to confirmation
              editingField = CONFIRMATION;
              firstEdit = YES;
            }
            break; // End of editing value 3
            
          case CONFIRMATION:
            if (firstEdit == YES) {
              encoderPos = 0;
              firstEdit = NO;
            }
            encoderMin = 0;
            encoderMax = 1;
            if (encoderPos < encoderMin) encoderPos = encoderMin;  
            if (encoderPos > encoderMax) encoderPos = encoderMax;  // Ensure encoder position does not exceed the maximum
            switch (encoderPos) {
              case 0:
                confirmation = NO;
                break;
              case 1:
                confirmation = YES;
                break;
            }
            if (buttonPressed == YES) {  // We are finished editing
              if (confirmation == YES) {
                // Copy the new values from temporary into live variables
                latHem = hemTemp;
                lat_HH = value1temp;
                lat_MM = value2temp;
                lat_SS = value3temp;
              }
              menuMode = SHOW;
              editingField = HEMISPHERE;
              firstEdit = YES;
              encoderPos = menuItem;
            }
            break; // End of confirmation
        }
        break; // End of editing latitude

    case SAVE_EEPROM:    // Only a yes or no confirmation here

      if (use_saved_data) {

        if (firstEdit == YES) {
          encoderPos = 0;
          firstEdit = NO;
        }
        encoderMin = 0;
        encoderMax = 1;
        if (encoderPos < encoderMin) encoderPos = encoderMin;  
        if (encoderPos > encoderMax) encoderPos = encoderMax;  // Ensure encoder position does not exceed the maximum
        switch (encoderPos) {
          case 0:
            confirmation = NO;
            break;
          case 1:
            confirmation = YES;
            break;
        }
        if (buttonPressed == YES) {  // We are finished editing
          if (confirmation == YES) {
            // Copy the variables into EEPROM
            saveToEEPROM();
            saveState = SAVED;
          } else saveState = CANCELLED;
          menuMode = SHOW;
          editingField = HEMISPHERE;
          firstEdit = YES;
          encoderPos = menuItem;
        }
      } else {  
        // If only hard-coded data is to be used
        //
        encoderPos = menuItem;
        menuMode = SHOW;      // Change back to display mode
      }
      break; // End of saving to EEPROM 

      case SHOW_ENCODERS:    // There is nothing to edit here
        encoderPos = menuItem;
        menuMode = SHOW;      // Change back to display mode
        break;

    }  // End of switch menuItem
    
  }  // End of menuMode = edit

  // Clear save state flag
  //
  if (menuItem != SAVE_EEPROM) saveState = WAITING;
  
  // Update OLED display on encoder change or button pressed
  //
  if (newEncPos == YES or buttonPressed == YES) update_OLED();
  buttonPressed = NO;

}  // End of menu logic

// Display data on OLED
//
void update_OLED()
{  
  char h;
  
  // Clear the display buffer
  //
  display.clearDisplay();
  display.setTextSize(1);           // Normal 1:1 pixel scale
  display.setTextColor(WHITE);      // Draw white text
  display.setCursor(0,0);          // Start at top-left corner
  display.cp437(true);              // Use full 256 char 'Code Page 437' font
      
  switch (menuItem) {
    case TEL_HORIZONTAL:
      display.println("Horizontal coord");    // Print title
      display.println("");                    // Print blank line
      display.println(F(OLED_HO_AZ));         // Print azimuth
      display.println(F(OLED_HO_ALT));        // Print altitude
      break;

    case TEL_EQUATORIAL:
      display.println("Equatorial coord");    // Print title
      display.println("");                    // Print blank line
      display.println(F(OLED_EQ_AR));         // Print right ascension
      display.println(F(OLED_EQ_DEC));        // Print declination
      break;

    case TEL_LST:
      display.println("Local Sidereal Time");            // Print title
      display.println("");                    // Print blank line

      switch (menuMode) {
        case SHOW:
          // Convert LST variables into TSL live value
          //
          TSL = LST_HH * 3600 + LST_MM * 60 + LST_SS;
          while (TSL >= 86400) TSL = TSL - 86400;                
          sprintf(OLED_Line_BEG, "LST %02d:%02d:%02d", int(LST_HH), int(LST_MM), int(LST_SS));
          display.println(F(OLED_Line_BEG));
          break;
        case EDIT:
          switch (editingField) {
            case HEMISPHERE:
              break;
            case VALUE1:
              sprintf(OLED_Line_BEG, "LST ");
              display.print(F(OLED_Line_BEG));
              // Change font colour to reverse
              sprintf(OLED_Line_MID, "%02d", int(value1temp));
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print(F(OLED_Line_MID));
              // Change font colour back to normal
              sprintf(OLED_Line_END, ":%02d:%02d", int(value2temp), int(value3temp));
              display.setTextColor(WHITE);      // Draw white text
              display.println(F(OLED_Line_END));
              break;
            case VALUE2:
              sprintf(OLED_Line_BEG, "LST %02d:", int(value1temp));
              display.print(F(OLED_Line_BEG));
              // Change font colour to reverse
              sprintf(OLED_Line_MID, "%02d", int(value2temp));
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print(F(OLED_Line_MID));
              // Change font colour back to normal
              sprintf(OLED_Line_END, ":%02d", int(value3temp));
              display.setTextColor(WHITE);      // Draw white text
              display.println(F(OLED_Line_END));
              break;
            case VALUE3:
              sprintf(OLED_Line_BEG, "LST %02d:%02d:", int(value1temp), int(value2temp));
              display.print(F(OLED_Line_BEG));
              // Change font colour to reverse
              sprintf(OLED_Line_MID, "%02d", int(value3temp));
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.println(F(OLED_Line_MID));
              // Change font colour back to normal
              display.setTextColor(WHITE);      // Draw white text
              break;
          case CONFIRMATION:
            sprintf(OLED_Line_BEG, "LST %02d:%02d:%02d", int(value1temp), int(value2temp), int(value3temp));
            display.println(F(OLED_Line_BEG));
            switch(confirmation) {
              case NO:
                display.print("Save? ");
                // Change font colour to reverse
                display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
                display.print("NO");
                // Change font colour back to normal
                display.setTextColor(WHITE);      // Draw white text
                display.println(" YES");
                break;
              case YES:
                display.print("Save? NO ");
                // Change font colour to reverse
                display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
                display.println("YES");
                // Change font colour back to normal
                display.setTextColor(WHITE);      // Draw white text
                break;
            }  // End confirmation switch

            break;
          } // End editingField switch

          break;
      } // End menuMode switch
      break;  // End of display LST

    case REF_AZIMUTH:
      display.println("Ref star azimuth");    // Print title
      display.println("");                    // Print blank line

      switch (menuMode) {
        case SHOW:
          sprintf(OLED_Line_BEG, "Az %02d:%02d:%02d", int(refAZ_DD), int(refAZ_MM), int(refAZ_SS));
          display.println(F(OLED_Line_BEG));
          break;
        case EDIT:
          switch (editingField) {
            case HEMISPHERE:
              break;
            case VALUE1:
              sprintf(OLED_Line_BEG, "Az ");
              display.print(F(OLED_Line_BEG));
              // Change font colour to reverse
              sprintf(OLED_Line_MID, "%02d", int(value1temp));
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print(F(OLED_Line_MID));
              // Change font colour back to normal
              sprintf(OLED_Line_END, ":%02d:%02d", int(value2temp), int(value3temp));
              display.setTextColor(WHITE);      // Draw white text
              display.println(F(OLED_Line_END));
              break;
            case VALUE2:
              sprintf(OLED_Line_BEG, "RA %02d:", int(value1temp));
              display.print(F(OLED_Line_BEG));
              // Change font colour to reverse
              sprintf(OLED_Line_MID, "%02d", int(value2temp));
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print(F(OLED_Line_MID));
              // Change font colour back to normal
              sprintf(OLED_Line_END, ":%02d", int(value3temp));
              display.setTextColor(WHITE);      // Draw white text
              display.println(F(OLED_Line_END));
              break;
            case VALUE3:
               sprintf(OLED_Line_BEG, "RA %02d:%02d:", int(value1temp), int(value2temp));
              display.print(F(OLED_Line_BEG));
              // Change font colour to reverse
              sprintf(OLED_Line_MID, "%02d", int(value3temp));
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.println(F(OLED_Line_MID));
              // Change font colour back to normal
              display.setTextColor(WHITE);      // Draw white text
              break;
           case CONFIRMATION:
              sprintf(OLED_Line_BEG, "Az %02d:%02d:%02d", int(value1temp), int(value2temp), int(value3temp));
              display.println(F(OLED_Line_BEG));
              switch(confirmation) {
                case NO:
                  display.print("Save? ");
                  // Change font colour to reverse
                  display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
                  display.print("NO");
                  // Change font colour back to normal
                  display.setTextColor(WHITE);      // Draw white text
                  display.println(" YES");
                  break;
                case YES:
                  display.print("Save? NO ");
                  // Change font colour to reverse
                  display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
                  display.println("YES");
                  // Change font colour back to normal
                  display.setTextColor(WHITE);      // Draw white text
                  break;
          }  // End confirmation switch

          break;
        } // End editingField switch

        break;
      } // End menuMode switch
      break;    // End of display reference star azimuth

    case REF_ALTITUDE:
      display.println("Ref star altitude");     // Print title
      display.println("");                    // Print blank line

      switch (menuMode) {
        case SHOW:
          sprintf(OLED_Line_BEG, "Alt %02d:%02d:%02d", int(refAlt_DD), int(refAlt_MM), int(refAlt_SS));
          display.println(F(OLED_Line_BEG));
          break;
        case EDIT:
          switch (editingField) {
            case HEMISPHERE:
              break;
            case VALUE1:
              sprintf(OLED_Line_BEG, "Alt ");
              display.print(F(OLED_Line_BEG));
              // Change font colour to reverse
              sprintf(OLED_Line_MID, "%02d", int(value1temp));
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print(F(OLED_Line_MID));
              // Change font colour back to normal
              sprintf(OLED_Line_END, ":%02d:%02d", int(value2temp), int(value3temp));
              display.setTextColor(WHITE);      // Draw white text
              display.println(F(OLED_Line_END));
              break;
            case VALUE2:
              sprintf(OLED_Line_BEG, "Alt %02d:", int(value1temp));
              display.print(F(OLED_Line_BEG));
              // Change font colour to reverse
              sprintf(OLED_Line_MID, "%02d", int(value2temp));
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print(F(OLED_Line_MID));
              // Change font colour back to normal
              sprintf(OLED_Line_END, ":%02d", int(value3temp));
              display.setTextColor(WHITE);      // Draw white text
              display.println(F(OLED_Line_END));
              break;
            case VALUE3:
              sprintf(OLED_Line_BEG, "Alt %02d:%02d:", int(value1temp), int(value2temp));
              display.print(F(OLED_Line_BEG));
              // Change font colour to reverse
              sprintf(OLED_Line_MID, "%02d", int(value3temp));
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.println(F(OLED_Line_MID));
              // Change font colour back to normal
              display.setTextColor(WHITE);      // Draw white text
              break;
           case CONFIRMATION:
              sprintf(OLED_Line_BEG, "Alt %02d:%02d:%02d", int(value1temp), int(value2temp), int(value3temp));
              display.println(F(OLED_Line_BEG));
              switch(confirmation) {
                case NO:
                  display.print("Save? ");
                  // Change font colour to reverse
                  display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
                  display.print("NO");
                  // Change font colour back to normal
                  display.setTextColor(WHITE);      // Draw white text
                  display.println(" YES");
                  break;
                case YES:
                  display.print("Save? NO ");
                  // Change font colour to reverse
                  display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
                  display.println("YES");
                  // Change font colour back to normal
                  display.setTextColor(WHITE);      // Draw white text
                  break;
              }  // End confirmation switch

              break;
          } // End editingField switch

          break;
      } // End menuMode switch

      break;  // End of display reference star altitude
      
    case TEL_LATITUDE:
      display.println("Latitude");            // Print title
      display.println("");                    // Print blank line

      switch (menuMode) {
        case SHOW:
          if (latHem == SOUTH) h = 'S';
          else h = 'N';
          sprintf(OLED_Line_BEG, "Lat %c%02d:%02d:%02d", h, int(lat_HH), int(lat_MM), int(lat_SS));
          display.println(F(OLED_Line_BEG));
          break;
        case EDIT:
          if (hemTemp == SOUTH) h = 'S';
          else h = 'N';
          switch (editingField) {
            case HEMISPHERE:
              sprintf(OLED_Line_BEG, "Lat ");
              display.print(F(OLED_Line_BEG));
              // Change font colour to reverse
              sprintf(OLED_Line_MID, "%c", h);
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print(F(OLED_Line_MID));
              // Change font colour back to normal
              sprintf(OLED_Line_END, "%02d:%02d:%02d", int(value1temp), int(value2temp), int(value3temp));
              display.setTextColor(WHITE);      // Draw white text
              display.println(F(OLED_Line_END));
              break;
            case VALUE1:
              sprintf(OLED_Line_BEG, "Lat %c", h);
              display.print(F(OLED_Line_BEG));
              // Change font colour to reverse
              sprintf(OLED_Line_MID, "%02d", int(value1temp));
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print(F(OLED_Line_MID));
              // Change font colour back to normal
              sprintf(OLED_Line_END, ":%02d:%02d", int(value2temp), int(value3temp));
              display.setTextColor(WHITE);      // Draw white text
              display.println(F(OLED_Line_END));
              break;
            case VALUE2:
              sprintf(OLED_Line_BEG, "Lat %c%02d:", h, int(value1temp));
              display.print(F(OLED_Line_BEG));
              // Change font colour to reverse
              sprintf(OLED_Line_MID, "%02d", int(value2temp));
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print(F(OLED_Line_MID));
              // Change font colour back to normal
              sprintf(OLED_Line_END, ":%02d", int(value3temp));
              display.setTextColor(WHITE);      // Draw white text
              display.println(F(OLED_Line_END));
              break;
            case VALUE3:
              sprintf(OLED_Line_BEG, "Lat %c%02d:%02d:", h, abs(int(value1temp)), int(value2temp));
              display.print(F(OLED_Line_BEG));
              // Change font colour to reverse
              sprintf(OLED_Line_MID, "%02d", int(value3temp));
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.println(F(OLED_Line_MID));
              // Change font colour back to normal
              display.setTextColor(WHITE);      // Draw white text
              break;

          case CONFIRMATION:
            sprintf(OLED_Line_BEG, "Lat %c%02d:%02d:%02d", h, int(value1temp), int(value2temp), int(value3temp));
            display.println(F(OLED_Line_BEG));
            switch(confirmation) {
              case NO:
                display.print("Save? ");
                // Change font colour to reverse
                display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
                display.print("NO");
                // Change font colour back to normal
                display.setTextColor(WHITE);      // Draw white text
                display.println(" YES");
                break;
              case YES:
                display.print("Save? NO ");
                // Change font colour to reverse
                display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
                display.println("YES");
                // Change font colour back to normal
                display.setTextColor(WHITE);      // Draw white text
                break;
            }  // End confirmation switch

            break;
          } // End editingField switch

          break;
      } // End menuMode switch
      break;  // End of display latitude

    case SAVE_EEPROM:
      display.println("Save input data");     // Print title
      display.println("");                    // Print blank line

      switch (menuMode) {
        case SHOW:
          if (use_saved_data) {
            switch (saveState) {
              case WAITING:
                display.println("Click to save");     // Print prompt
                break;
              case SAVED:
                display.println("Saved to EEPROM");     // Print prompt
                break;
              case CANCELLED:
                display.println("Cancelled");     // Print prompt
                break;
            }
          } else {
            display.println("Option is not");     // Print prompt
            display.println("available");     // Print prompt
          }
          break;
        
        case EDIT:
          switch(confirmation) {
            case NO:
              display.print("Save? ");
              // Change font colour to reverse
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print("NO");
              // Change font colour back to normal
              display.setTextColor(WHITE);      // Draw white text
              display.println(" YES");
              break;
            case YES:
              display.print("Save? NO ");
              // Change font colour to reverse
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.println("YES");
              // Change font colour back to normal
              display.setTextColor(WHITE);      // Draw white text
              break;
          }  // End confirmation switch
  
          break;
      } // End menuMode switch

      break;

    case SHOW_ENCODERS:
      display.println("Encoder count");       // Print title
      display.println("");                    // Print blank line
      display.println(encoderValue1);         // Print encoder1 count
      display.println(encoderValue2);         // Print encoder2 count
      break;

  }  // End menuItem switch

  display.display();  // Update the OLED display

} //End of update_OLED 

// Debounce subroutine
//
boolean buttonDown(char button, unsigned long *marker, char *butnstate, unsigned long interval) {

  // Deal with a button read; true if button pressed and debounced is a new event
  // Uses reading of button input, debounce store, state store and debounce interval.

  switch (*butnstate) {               // Odd states if was pressed, >= 2 if debounce in progress
    case 0: // Button up so far,
      if (button == HIGH) return false; // Nothing happening!
      else {
        *butnstate = 2;                 // record that is now pressed
        *marker = millis();             // note when was pressed
        return false;                   // and move on
      }

    case 1: // Button down so far,
      if (button == LOW) return false; // Nothing happening!
      else {
        *butnstate = 3;                 // record that is now released
        *marker = millis();             // note when was released
        return false;                   // and move on
      }

    case 2: // Button was up, now down.
      if (button == HIGH) {
        *butnstate = 0;                 // no, not debounced; revert the state
        return false;                   // False alarm!
      }
      else {
        if (millis() - *marker >= interval) {
          *butnstate = 1;               // jackpot!  update the state
          return true;                  // because we have the desired event!
        }
        else
          return false;                 // not done yet; just move on
      }

    case 3: // Button was down, now up.
      if (button == LOW) {
        *butnstate = 1;                 // no, not debounced; revert the state
        return false;                   // False alarm!
      }
      else {
        if (millis() - *marker >= interval) {
          *butnstate = 0;               // Debounced; update the state
          return false;                 // but it is not the event we want
        }
        else
          return false;                 // not done yet; just move on
      }
    default:                            // Error; recover anyway
      {
        *butnstate = 0;
        return false;                   // Definitely false!
      }
  }
}  // End of button debounce


// Save all user input data to EEPROM memory for permanent storage
// The Due DOESNT HAVE STANDARD EEPRPOM so the standard Arduino 
// library cannot be used
//
void saveToEEPROM() {

  // All of the data are of type int
  // Arduino integers comprise 4 8-bit bytes
  // Byte addresses increment by 4 for each integer variable stored
  //
  eeWriteInt(0,LST_HH);
  eeWriteInt(4,LST_MM);
  eeWriteInt(8,LST_SS);
  eeWriteInt(12,latHem);
  eeWriteInt(16,lat_HH);
  eeWriteInt(20,lat_MM);
  eeWriteInt(24,lat_SS);
  eeWriteInt(28,refAZ_DD);
  eeWriteInt(32,refAZ_MM);
  eeWriteInt(36,refAZ_SS);
  eeWriteInt(40,refAlt_DD);
  eeWriteInt(44,refAlt_MM);
  eeWriteInt(48,refAlt_SS);
  
}

// Retrieve all user data from EEPROM memory
//
void getFromEEPROM() {

  // All of the data are of type int
  // Arduino integers comprise 4 8-bit bytes
  // Byte addresses increment by 4 for each integer variable stored
  //
  LST_HH      = checkZero(eeGetInt(0));
  LST_MM      = checkZero(eeGetInt(4));
  LST_SS      = checkZero(eeGetInt(8));
  if (eeGetInt (12) == 0) latHem = NORTH;  // Special case for the enumerated hemisphere variable
  else latHem = SOUTH;
  lat_HH      = checkZero(eeGetInt(16));
  lat_MM      = checkZero(eeGetInt(20));
  lat_SS      = checkZero(eeGetInt(24));
  refAZ_DD   = checkZero(eeGetInt(28));
  refAZ_MM   = checkZero(eeGetInt(32));
  refAZ_SS   = checkZero(eeGetInt(36));
  refAlt_DD  = checkZero(eeGetInt(40));
  refAlt_MM  = checkZero(eeGetInt(44));
  refAlt_SS  = checkZero(eeGetInt(48));
}


// Function to write an integer to an address in non-volatile memory
//
void eeWriteInt(int pos, int val) {
    byte* p = (byte*) &val;
    dueFlashStorage.write(pos, *p);
    dueFlashStorage.write(pos + 1, *(p + 1));
    dueFlashStorage.write(pos + 2, *(p + 2));
    dueFlashStorage.write(pos + 3, *(p + 3));
}

// Function to read an integer from an address in non-volatile memory
//
int eeGetInt(int pos) {
  int val;
  byte* p = (byte*) &val;
  *p        = dueFlashStorage.read(pos);
  *(p + 1)  = dueFlashStorage.read(pos + 1);
  *(p + 2)  = dueFlashStorage.read(pos + 2);
  *(p + 3)  = dueFlashStorage.read(pos + 3);
  return val;
}

// Function to check the sign of an integer
// If the value is less than zero, then make equal to zero
//
int checkZero(int check) {
  int val;
  if (check < 0) val = 0;
  else val = check;
  return val;
}

// Convert reference star horizontal coordinates to telescope position
// This will be used as a starting reference for the telescope
// by converting the horizontal coordinates into encoder pulse starting positions
//
void reference_coords() {

  // Reference star altitude and azimuth
  //
  double a_rad = (((refAlt_DD * 3600) + (refAlt_MM * 60) + refAlt_SS) / 3600.0) * pi / 180.0; // Reference star altitude a
  double A_rad = (((refAZ_DD * 3600) + (refAZ_MM * 60) + refAZ_SS) / 3600.0) * pi / 180.0;  // Reference star azimuth A
  
  // Convert Altitude and Azimuth angles into the equivalent number of encoder pulses
  //
  refAltPulses = (a_rad * pulses_enc1) / (2.0 * pi);
  refAzPulses = (A_rad * pulses_enc2) / (2.0 * pi);

}

// Convert decimal LST data to hours, minutes, seconds
//
void TSL_dec_to_HMS () {
  LST_HH = TSL / 3600;
  LST_MM = (TSL - LST_HH * 3600) / 60;
  LST_SS = (TSL - LST_HH * 3600) - LST_MM * 60;
}
