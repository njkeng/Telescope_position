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
#include <Fonts/Picopixel.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Timer functions
#include "TimerControl.h"
extern void TimerInit(void);
extern void TimerControl(void);
extern void TimerStart(struct Timer* pTimer, int nCount);
extern void TimerReset(struct Timer* pTimer);
extern struct Timer pTimer[];

// Declarations for astronomy
//
unsigned long   seg_sideral = 1003;
const double    pi = 3.14159265358979324;
volatile int    lastEncoded1 = 0;
volatile long   encoderValue1 = 0;
volatile int    lastEncoded2 = 0;
volatile long   encoderValue2 = 0;

char  input[20];
char  txAR[10];
char  txDEC[11];
long  TSL;
unsigned long t_ciclo_acumulado = 0, t_ciclo;
long    Az_tel_s, Alt_tel_s;
long    AR_tel_s, DEC_tel_s;
long    AR_stell_s, DEC_stell_s;
double  cos_phi, sin_phi;
// double  alt, azi;

// Declarations for menu displays
char  OLED_EQ_AR[16];  // Telescope position in equatorial coordinates
char  OLED_EQ_DEC[16];
char  OLED_HO_ALT[16];  // Telescope position in horizontal coordinates
char  OLED_HO_AZ[16];

// Declarations for menu system
volatile byte encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile byte oldEncPos = 0; //DEBUGGING stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
// Button reading, including debounce without delay function declarations
byte oldButtonState = HIGH;  // assume switch open because of pull-up resistor
const unsigned long debounceTime = 10;  // milliseconds
unsigned long buttonPressTime;  // when the switch last changed state
boolean buttonPressed = 0; // a flag variable
// Menu and submenu/setting declarations
byte Mode = 0;   // This is which menu mode we are in at any given time (top level or one of the submenus)
const byte modeMax = 3; // This is the number of submenus/settings you want
byte setting1 = 0;  // a variable which holds the value we set 
byte setting2 = 0;  // a variable which holds the value we set 
byte setting3 = 0;  // a variable which holds the value we set 
/* Note: you may wish to change settingN etc to int, float or boolean to suit your application. 
 Remember to change "void setAdmin(byte name,*BYTE* setting)" to match and probably add some 
 "modeMax"-type overflow code in the "if(Mode == N && buttonPressed)" section*/

enum menuItems {
  TEL_EQUITORIAL,   // Current equitorial coordinates of our telescope
  TEL_HORIZONTAL,   // Current horizontal coordinates of our telescope
  TEL_LATITUDE,     // Latitude of our observing position
  STAR_RA,          // Right ascension of our reference star
  STAR_HA           // Hour angle of our reference star
};
enum menuItems menuItem;

enum menuModes {
  SHOW,          // Menu is in display mode
  EDIT              // Menu is in edit mode
};
enum menuModes menuMode;

enum editingFields {
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

// For encoder input processing
static uint8_t prevNextCode = 0;
static uint16_t store=0;
static int8_t val;

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

  cos_phi = cos((((latHH * 3600) + (latMM * 60) + latSS) / 3600.0) * pi / 180.0);
  sin_phi = sin((((latHH * 3600) + (latMM * 60) + latSS) / 3600.0) * pi / 180.0);

  TSL = poleAR_HH * 3600 + poleAR_MM * 60 + poleAR_SS + poleH_HH * 3600 + poleH_MM * 60 + poleH_SS;
  while (TSL >= 86400) TSL = TSL - 86400;

  // Initialize the timer control; also resets all timers
  TimerInit();
  
  // Start the timer
  TimerStart(&pTimerLCD, LCD_FREQ);

  // Menu system encoder setup
  pinMode(enc_mA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(enc_mB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode (enc_mButton, INPUT_PULLUP); // setup the button pin

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.setFont(&Picopixel);
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------
void loop()
{
 
  t_ciclo = millis();
  if (t_ciclo_acumulado >= seg_sideral) {
    TSL++;
    t_ciclo_acumulado = t_ciclo_acumulado - seg_sideral;
    if (TSL >= 86400) {
      TSL = TSL - 86400;
    }
  }

  read_sensors();
  AZ_to_EQ();
  
  if (Serial.available() > 0) {
    communication();
  }

  t_ciclo = millis() - t_ciclo;
  t_ciclo_acumulado = t_ciclo_acumulado + t_ciclo;


  // Menu system
  if( val=read_rotary() ) {
    encoderPos +=val;
  }
  rotaryMenu();
    
  // Update OLED display
  if(pTimerLCD.bExpired == true)
  {
    update_OLED();
    TimerStart(&pTimerLCD, LCD_FREQ);
  }// end if



}

//--------------------------------------------------------------------------------------------------------------------------------------------------------
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

//--------------------------------------------------------------------------------------------------------------------------------------------------------
void update_OLED()
{  
  // Clear the display buffer
  display.clearDisplay();
  display.setTextSize(2);           // Normal 1:1 pixel scale
  display.setTextColor(WHITE);      // Draw white text
  display.setCursor(0,10);          // Start at top-left corner
  display.cp437(true);              // Use full 256 char 'Code Page 437' font
  display.println(F(OLED_AR));      // Print right ascension
  display.println(F(OLED_DEC));     // Print declination
  display.display();
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------
void read_sensors() {
  long h_deg, h_min, h_seg, A_deg, A_min, A_seg;

  if (encoderValue2 >= pulses_enc2 || encoderValue2 <= -pulses_enc2) {
    encoderValue2 = 0;
  }
  int enc1 = encoderValue1 / 1500;
  long encoder1_temp = encoderValue1 - (enc1 * 1500);
  long map1 = enc1 * map(1500, 0, pulses_enc1, 0, 324000);
  int enc2 = encoderValue2 / 1500;
  long encoder2_temp = encoderValue2 - (enc2 * 1500);
  long map2 = enc2 * map(1500, 0, pulses_enc2, 0, 1296000);

  Alt_tel_s = map1 + map (encoder1_temp, 0, pulses_enc1, 0, 324000);
  Az_tel_s  = map2 + map (encoder2_temp, 0, pulses_enc2, 0, 1296000);

  if (Az_tel_s < 0) Az_tel_s = 1296000 + Az_tel_s;
  if (Az_tel_s >= 1296000) Az_tel_s = Az_tel_s - 1296000 ;

}

//--------------------------------------------------------------------------------------------------------------------------------------------------------
void Encoder1() {
  int encoded1 = (digitalRead(enc_1A) << 1) | digitalRead(enc_1B);
  int sum  = (lastEncoded1 << 2) | encoded1;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue1 ++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue1 --;
  lastEncoded1 = encoded1;
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------
void Encoder2() {
  int encoded2 = (digitalRead(enc_2A) << 1) | digitalRead(enc_2B);
  int sum  = (lastEncoded2 << 2) | encoded2;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue2 ++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue2 --;
  lastEncoded2 = encoded2;
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------
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
  delta_tel = asin((sin_phi * sin_h) + (cos_phi * cos_h * cos_A));
  sin_DEC = sin(delta_tel);
  cos_DEC = cos(delta_tel);
  DEC_tel_s = long((delta_tel * 180.0 / pi) * 3600.0);

  while (DEC_tel_s >= 324000) {
    DEC_tel_s = DEC_tel_s - 324000;
  }
  while (DEC_tel_s <= -324000) {
    DEC_tel_s = DEC_tel_s + 324000;
  }

  H_telRAD = acos((sin_h - (sin_phi * sin_DEC)) / (cos_phi * cos_DEC));
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
  arHH = AR_tel_s / 3600;
  arMM = (AR_tel_s - arHH * 3600) / 60;
  arSS = (AR_tel_s - arHH * 3600) - arMM * 60;
  decDEG = abs(DEC_tel_s) / 3600;
  decMM = (abs(DEC_tel_s) - decDEG * 3600) / 60;
  decSS = (abs(DEC_tel_s) - decDEG * 3600) - decMM * 60;
  (DEC_tel_s < 0) ? sDEC_tel = 45 : sDEC_tel = 43;

  // Equatorial coordinate data for Stellarium comms
  //
  sprintf(txAR, "%02d:%02d:%02d#", int(arHH), int(arMM), int(arSS));
  sprintf(txDEC, "%c%02d%c%02d:%02d#", sDEC_tel, int(decDEG), 223, int(decMM), int(decSS));

  // Equatorial coordinate data for OLED display
  //
  sprintf(OLED_EQ_AR,  "RA %02d:%02d:%02d", int(arHH), int(arMM), int(arSS));
  sprintf(OLED_EQ_DEC, "DEC %c%02d'%02d:%02d", sDEC_tel, int(decDEG), int(decMM), int(decSS));

  // Convert decimal horizontal coordinate data to hours, minutes, seconds
  altHH = Alt_tel_s / 3600;
  altMM = (Alt_tel_s - altHH * 3600) / 60;
  altSS = (Alt_tel_s - altHH * 3600) - altMM * 60;
  azDEG = abs(Az_tel_s) / 3600;
  azMM = (abs(Az_tel_s) - azDEG * 3600) / 60;
  azSS = (abs(Az_tel_s) - azDEG * 3600) - azMM * 60;

  // Horizontal coordinate data for OLED display
  //
  sprintf(OLED_HO_ALT,  "Alt %02d:%02d:%02d", int(altHH), int(altMM), int(altSS));
  sprintf(OLED_HO_AZ,  "Az   %02d:%02d:%02d", int(azDEG), int(azMM), int(azSS));

}

void rotaryMenu() {
  
  // Check for new encoder position
  if(oldEncPos != encoderPos) {
    newEncPos = YES;
    Serial.println(encoderPos);// DEBUGGING. Sometimes the serial monitor may show a value just outside modeMax due to this function. The menu shouldn't be affected.
    oldEncPos = encoderPos;
  } else newEncPos = NO;
  

  // Button debounce
  byte buttonState = digitalRead (enc_mButton); 
  if (buttonState != oldButtonState){
    if (millis () - buttonPressTime >= debounceTime){ // debounce
      buttonPressTime = millis ();  // when we closed the switch 
      oldButtonState =  buttonState;  // remember for next time 
      if (buttonState == LOW){
        Serial.println ("Button pressed"); // DEBUGGING: print that button has been pressed
        buttonPressed = YES;
      }
      else {
        Serial.println ("Button released"); // DEBUGGING: print that button has been released
        buttonPressed = NO;  
      }  
    }  // end if debounce time up
  } // end of state change


/*
  //Main menu section
  if (Mode == 0) {
    if (encoderPos > (modeMax+10)) encoderPos = modeMax; // check we haven't gone out of bounds below 0 and correct if we have
    else if (encoderPos > modeMax) encoderPos = 0; // check we haven't gone out of bounds above modeMax and correct if we have   
    if (buttonPressed){ 
      Mode = encoderPos; // set the Mode to the current value of input if button has been pressed
      Serial.print("Mode selected: "); //DEBUGGING: print which mode has been selected
      Serial.println(Mode); //DEBUGGING: print which mode has been selected
      buttonPressed = 0; // reset the button status so one press results in one action
      if (Mode == 1) {
        Serial.println("Mode 1"); //DEBUGGING: print which mode has been selected
        encoderPos = setting1; // start adjusting Vout from last set point
      }
      if (Mode == 2) {
        Serial.println("Mode 2"); //DEBUGGING: print which mode has been selected
        encoderPos = setting2; // start adjusting Imax from last set point
      }
      if (Mode == 3) {
        Serial.println("Mode 3"); //DEBUGGING: print which mode has been selected
        encoderPos = setting3; // start adjusting Vmin from last set point
      }
    }
  }
  if (Mode == 1 && buttonPressed) {
    setting1 = encoderPos; // record whatever value your encoder has been turned to, to setting 3
    setAdmin(1,setting1);
    //code to do other things with setting1 here, perhaps update display  
  }
  if (Mode == 2 && buttonPressed) {
    setting2 = encoderPos; // record whatever value your encoder has been turned to, to setting 2
    setAdmin(2,setting2);
    //code to do other things with setting2 here, perhaps update display   
  }
  if (Mode == 3 && buttonPressed){
    setting3 = encoderPos; // record whatever value your encoder has been turned to, to setting 3
    setAdmin(3,setting3);
    //code to do other things with setting3 here, perhaps update display 
  }
*/

// -------------------------------------------------------
// TRYING NEW STUFF HERE
// -------------------------------------------------------

  
  // If in display mode, use the encoder to cycle through the menu items
  //
  if (menuMode == SHOW) {
    
    encoderPos &= 4;  // Ensure encoder position is within the valid range 0 to 4 (5 items total)
    switch (encoderPos) {
      case 0:
        menuItem = TEL_EQUITORIAL;
        break;
      case 1:
        menuItem = TEL_HORIZONTAL;
        break;
      case 2:
        menuItem = TEL_LATITUDE;
        break;
      case 3:
        menuItem = STAR_RA;
        break;
      case 4:
        menuItem = STAR_HA;
        break;
    }
    if (buttonPressed == YES) menuMode = EDIT;
  } 
  
  // Perform the necessary edit actions for each menu item
  //
  if (menuMode == EDIT) {
    switch (menuItem) {
      case TEL_EQUITORIAL:    // There is nothing to edit here
        menuMode = SHOW;      // Change back to display mode
        break;
      case TEL_HORIZONTAL:    // There is nothing to edit here
        menuMode = SHOW;      // Change back to display mode
        break;
      case TEL_LATITUDE:    // There are three fields to edit plus a confirmation
        switch (editingField) {
          case VALUE1:    // Latitude hours
            if (firstEdit == YES) {
              encoderPos = latHH + 90;  // Valid range is -90 to +90, so offset by 90.
              firstEdit = NO;
            }
            // The valid range is -90 to +90.  This makes 181 valid values
            encoderPos &= 180;  // Ensure encoder position is within the valid range 0 to 180
            value1temp = encoderPos - 90;
            if (buttonPressed == YES) {  // Start editing value 2
              editingField = VALUE2;
              firstEdit = YES;
            }
            break; // End of editing value 1
            
          case VALUE2:    // Latitude minutes
            if (firstEdit == YES) {
              encoderPos = latMM;
              firstEdit = NO;
            }
            encoderPos &= 59;  // Ensure encoder position is within the valid range 0 to 59
            value2temp = encoderPos;
            if (buttonPressed == YES) {  // Start editing value 3
              editingField = VALUE3;
              firstEdit = YES;
            }
            break; // End of editing value 2
            
          case VALUE3:    // Latitude seconds
            if (firstEdit == YES) {
              encoderPos = latSS;
              firstEdit = NO;
            }
            encoderPos &= 59;  // Ensure encoder position is within the valid range 0 to 59
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
            encoderPos &= 1;  // Ensure encoder position is within the valid range 0 to 1
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
                latHH = value1temp;
                latMM = value2temp;
                latSS = value3temp;
              }
              menuMode = SHOW;
              editingField = VALUE1;
              firstEdit = YES;
            }
            break; // End of confirmation
        }
        break; // End of editing latitude
        
      case STAR_RA:    // There are three fields to edit plus a confirmation
        switch (editingField) {
          case VALUE1:    // Right ascension hours
            if (firstEdit == YES) {
              encoderPos = poleAR_HH;
              firstEdit = NO;
            }
            encoderPos &= 24;  // Ensure encoder position is within the valid range 0 to 24
            value1temp = encoderPos;
            if (buttonPressed == YES) {  // Start editing value 2
              editingField = VALUE2;
              firstEdit = YES;
            }
            break; // End of editing value 1
            
          case VALUE2:    // Right ascension minutes
            if (firstEdit == YES) {
              encoderPos = poleAR_MM;
              firstEdit = NO;
            }
            encoderPos &= 59;  // Ensure encoder position is within the valid range 0 to 59
            value2temp = encoderPos;
            if (buttonPressed == YES) {  // Start editing value 3
              editingField = VALUE3;
              firstEdit = YES;
            }
            break; // End of editing value 2
            
          case VALUE3:    // Right ascension seconds
            if (firstEdit == YES) {
              encoderPos = poleAR_SS;
              firstEdit = NO;
            }
            encoderPos &= 59;  // Ensure encoder position is within the valid range 0 to 59
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
            encoderPos &= 1;  // Ensure encoder position is within the valid range 0 to 1
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
                poleAR_HH = value1temp;
                poleAR_MM = value2temp;
                poleAR_SS = value3temp;
              }
              menuMode = SHOW;
              editingField = VALUE1;
              firstEdit = YES;
            }
            break; // End of confirmation
        }
        break; // End of editing star right ascension
        
      case STAR_HA:    // There are three fields to edit plus a confirmation
        switch (editingField) {
          case VALUE1:    // Right ascension hours
            if (firstEdit == YES) {
              encoderPos = poleH_HH;
              firstEdit = NO;
            }
            encoderPos &= 24;  // Ensure encoder position is within the valid range 0 to 24
            value1temp = encoderPos;
            if (buttonPressed == YES) {  // Start editing value 2
              editingField = VALUE2;
              firstEdit = YES;
            }
            break; // End of editing value 1
            
          case VALUE2:    // Right ascension minutes
            if (firstEdit == YES) {
              encoderPos = poleH_MM;
              firstEdit = NO;
            }
            encoderPos &= 59;  // Ensure encoder position is within the valid range 0 to 59
            value2temp = encoderPos;
            if (buttonPressed == YES) {  // Start editing value 3
              editingField = VALUE3;
              firstEdit = YES;
            }
            break; // End of editing value 2
            
          case VALUE3:    // Right ascension seconds
            if (firstEdit == YES) {
              encoderPos = poleH_SS;
              firstEdit = NO;
            }
            encoderPos &= 59;  // Ensure encoder position is within the valid range 0 to 59
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
            encoderPos &= 1;  // Ensure encoder position is within the valid range 0 to 1
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
                poleH_HH = value1temp;
                poleH_MM = value2temp;
                poleH_SS = value3temp;
              }
              menuMode = SHOW;
              editingField = VALUE1;
              firstEdit = YES;
            }
            break; // End of confirmation
      }
      break; // End of editing star right ascension 
    }
  }

  // Update OLED display on encoder change or button pressed
  if (newEncPos == YES or buttonPressed == YES) update_OLED();

}

// Carry out common activities each time a setting is changed
void setAdmin(byte name, byte setting){
  Serial.print("Setting "); //DEBUGGING
  Serial.print(name); //DEBUGGING
  Serial.print(" = "); //DEBUGGING
  Serial.println(setting);//DEBUGGING
  encoderPos = 0; // reorientate the menu index - optional as we have overflow check code elsewhere
  buttonPressed = 0; // reset the button status so one press results in one action
  Mode = 0; // go back to top level of menu, now that we've set values
  Serial.println("Main Menu"); //DEBUGGING
}


// Process rotary encoder input
// Debounce is implemented using a lookup table approach
// A vald CW or  CCW move returns 1, invalid returns 0.
int8_t read_rotary() {
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
