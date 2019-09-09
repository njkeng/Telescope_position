
//**********************************************
// Config your sensor pins and location here!
//**********************************************


// Define the number of pulses that your encoder (1 and 2) gives by turn, and multiply by 4
// In my case: 600 x 15 x 4  (600 pulses by turn; 15 is the gear ratio), so:
long pulses_enc1 = 36000;              
long pulses_enc2 = 36000;       


// Define DUE's pins
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

// enter your latitude (example: North 40º33'20'')
enum hemispheres latHem = NORTH;  // Valid values are NORTH or SOUTH
int latHH = 1;
int latMM = 2;
int latSS = 3;

// enter reference star right ascension (AR: HH:MM:SS)
// for Northern hemisphere, the reference star would normally be the Pole Star (Polaris)
int starAR_HH = 4;    // this means 2 hours, 52 minutes and 16 seconds
int starAR_MM = 5;
int starAR_SS = 6;

// enter reference star hour angle (H: HH:MM:SS)
// for Northern hemisphere, the reference star would normally be the Pole Star (Polaris)
int starH_HH = 7;
int starH_MM = 8;
int starH_SS = 9;

// For DEBUGGING use only
// SETTING THIS FLAG WILL MAKE THE COORDINATES CALCULATE INCORRECTLY
//
#define show_encoders 1
