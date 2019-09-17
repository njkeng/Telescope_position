
//**********************************************
// Config your sensor pins and location here!
//**********************************************


// Define the number of pulses that your encoder (1 and 2) gives by turn, and multiply by 4
// Example: 600 x 15 x 4  (600 pulses by turn; 15 is the gear ratio)
// OR
// Calibrate by performing many revolutions (50?) and keep track of the count of encoder pulses
//
long pulses_enc1 = 46331;              
long pulses_enc2 = 46277;       


// Define DUE's pins
//
#define enc_1A 2            // define DUE pin to encoder 1-channel A                     
#define enc_1B 3            // define DUE pin to encoder 1-channel B  
#define enc_2A 4            // define DUE pin to encoder 2-channel A  
#define enc_2B 5            // define DUE pin to encoder 2-channel B  
#define enc_mA 6            // define DUE pin to menu encoder channel A
#define enc_mB 7            // define DUE pin to menu encoder channel B    
#define enc_mButton 8       // Encoder built-in pushbutton

// Set this to 0 if you want to use the values coded below
// Set this to 1 if you want to save the values you enter via the menu
//
#define use_saved_data 1

enum hemispheres {
 NORTH,
 SOUTH 
};

// enter your latitude (example: NORTH 40º33'20'')
//
enum hemispheres latHem = SOUTH;  // Valid values are NORTH or SOUTH
int lat_HH = 27;
int lat_MM = 28;
int lat_SS = 5;

// enter apparent (local) sidereal time (LST: HH:MM:SS)
//
int LST_HH = 11;
int LST_MM = 55;
int LST_SS = 38;

// enter reference star right ascension (AR: HH:MM:SS)
// for Northern hemisphere, the reference star would normally be the Pole Star (Polaris)
//
int starAR_HH = 8;
int starAR_MM = 7;
int starAR_SS = 59;

// enter reference star declination (Dec: DD:MM:SS)
// for Northern hemisphere, the reference star would normally be the Pole Star (Polaris)
//
int starDec_DD = 8;
int starDec_MM = 7;
int starDec_SS = 59;

// For DEBUGGING use only
// SETTING THIS FLAG WILL MAKE THE COORDINATES CALCULATE INCORRECTLY
//
#define show_encoders 0
