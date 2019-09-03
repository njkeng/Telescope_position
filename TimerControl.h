#ifndef _TIMERCONTROL_
#define _DUECANLAYER_

struct Timer
{
  int  nCount;
  bool bStart;
  bool bExpired;
};

#define TIMERFREQUENCY                    1000    // Time = 1 sec / TIMERFREQUENCY
#define LCD_FREQ                    	    100	  // Frequency to update the LCD display


#define TIMERS                            1       // Number of timers
#define pTimerLCD                         pTimer[0]

#endif
