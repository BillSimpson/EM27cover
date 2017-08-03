#include <AccelStepper.h>

// Bill Simpson (wrsimpson@alaska.edu) 25 Jun 2017
//
// This code is placed into the public domain
// 
// This is code to run an arduino microcontroller to 
// open and close a cover for the EM27/Sun spectrometer.
// The code drives a stepper motor that does the motion
// and reads limit switches to determine open/closed state.
// There is a basic text-based interface between the 
// controller and the host computer with the intent that
// the host computer logs the state of the cover and such
// events.  The arduino also has a rain sensor that will
// force the cover to close in the event of rain.
//
// Text commands:
// o = Open cover once; will not re-open after rain
// f = Open cover and try to keep it open as much as possible
// c = Close cover
// ? = Report cover state (open/closed & rain)
// v = toggle verbose mode
// 
// Event feedback:
// If the cover closes or opens, that is sent as "closing by request" or "closing by rain"
// When the cover reaches a state, that is reported as "closed state achieved"
// the ? command returns: "cover closed/open/unknown, rain absent/present"
//
// The hardware uses a Pololu DRV8825 current-controlled (chopper) driver
// Note that the current is set in the hardware (potentiometer).  We use the "enable"
// feature to turn off the motor current when the motor is at an end state.
//
// The carrier board has a header installed from DIR to M1 on the digital side
// Those header pins are inserted into the UnoDIO pins D7 to D2
// The enable and fault pins are jumpered to other digital pins 
// 
// Note that the motor power requires a capacitor to smooth input, and the microcontroller
// ground is common with (connected to) the motor drive ground.
//
// Limit switches are open (high) when not at limit and short to ground (low) at limit
// Rain sensor is connected digitally.
//
// Motor variables below.  Calculates microsteps from USTEP_DIVISOR
// Due to hardware choice to wire M0 low, only full, 1/4, 1/16, and 1/32 can be selected
// Note that the driver may not run at high microsteps / second (>4000 microsteps/sec)
// for Arduino Uno (16MHz) according to AccelStepper manual; needs testing.

#define STEPS_FULLMOVE 6250 
// actual move is about 5625, but this should be more to assure full open
#define USTEP_DIVISOR 4
// the divisor for microstepping, 1 = full steppping, 4 = 1/4 stepping,
// 16 = 1/16 stepping, 32 = 1/32 stepping  
#define STEP_SPEED 250
// Speed in  full steps per second. For 200 steps/rot and 8mm/rot this is 10 mm/sec
#define STEP_ACCEL 1000
// Accel in full steps per second per second
#define TIMEOUT_MILLIS 60000
// Timeout (in milliseconds) for the stepper drive loop

#define DIRECTION_OPEN +1
#define DIRECTION_CLOSE -1
// state codes
#define STATE_UNKNOWN 0  
// used when not at either limit
#define STATE_CLOSED 1  
// used when at the closed limit
#define STATE_OPENED 2  
// used when at the open limit
#define STATE_ERROR 3   
// if both switches are triggered -- motor cannot move
#define STATE_FORCEOPEN 9
// used to keep the cover open as often as possible
// fault codes
#define FAULT_NONE 0
#define FAULT_MOTOR_SHORT 1
#define FAULT_DRIVER_FAULT 2
// condition codes
#define COND_CLEAR 0
#define COND_RAIN 1

// Driver for stepper motor

#define DPIN_DIR 7
#define DPIN_STEP 6
#define DPIN_NOT_SLEEP 5
#define DPIN_NOT_RESET 4
#define DPIN_M2 3
#define DPIN_M1 2

#define DPIN_ENABLE 8     
// output to enable / disable
#define DPIN_DRIVER_FAULT 9      
// input from driver board indicating fault

// Limit switches

#define DPIN_LIMIT_CLOSED 10   
// input from limit switch
#define DPIN_LIMIT_OPENED 11   
// input from limit switch

// Rain 
#define DPIN_RAIN 12           
// input from rain sensor (digital)
#define APIN_RAIN 0
// input for analog rain sensor (analog)
#define MILLIVOLTS_PER_COUNT 4.8828
// Conversion factor to get millivolts per DIO count
#define RAIN_THRESHOLD_MV 2900
// Threshold for rain (from 3300 mV for true full clear sky (open circuit))
#define NUMPNTS 50
// set number of points to average and calc standard deviation
#define VERBOSE_DEFAULT false

// Global variables

char commandchar;        // command character
int state = STATE_UNKNOWN;
int requeststate = STATE_CLOSED;
int last_state = STATE_UNKNOWN;
int condition = COND_CLEAR;
int fault = FAULT_NONE;   
int verbose_logging = VERBOSE_DEFAULT;
float duration, rain_mv;
int i;
double rain_mv_sum, rain_mv_sumsqr; 
float rain_mv_avg, rain_mv_std;

AccelStepper stepper(AccelStepper::DRIVER, DPIN_STEP, DPIN_DIR);  // sets up 2-wire stepper driver

// Setup for serial io and driving the stepper
void setup() {
  // Start serial for input / output
  Serial.begin(9600);
  Serial.println("EM27Sun Cover Driver v1.4");
  Serial.print("Rain threshold set at ");
  Serial.print(RAIN_THRESHOLD_MV);
  Serial.println("mV");
  // Set up for the Stepper motor
  stepper.setPinsInverted(true,false,true); // sets direction and enable pins inverted
  stepper.setMaxSpeed(USTEP_DIVISOR * STEP_SPEED);
  stepper.setAcceleration(USTEP_DIVISOR * STEP_ACCEL);
  stepper.setEnablePin(DPIN_ENABLE);

  // Set up the pins for power and motor microstep selection
  pinMode(DPIN_NOT_SLEEP, OUTPUT);
  pinMode(DPIN_NOT_RESET, OUTPUT);
  pinMode(DPIN_M2, OUTPUT);
  pinMode(DPIN_M1, OUTPUT);
  // Turn on power to driver board
  digitalWrite(DPIN_NOT_SLEEP, HIGH);
  digitalWrite(DPIN_NOT_RESET, HIGH);
  if (USTEP_DIVISOR == 1) {
    // Set for full stepping
    digitalWrite(DPIN_M2, LOW);
    digitalWrite(DPIN_M1, LOW);
  }
  if (USTEP_DIVISOR == 4) {
    // Set for 1/4 stepping
    digitalWrite(DPIN_M2, LOW);
    digitalWrite(DPIN_M1, HIGH);
  }
  if (USTEP_DIVISOR == 16) {
    // Set for 1/16 stepping
    digitalWrite(DPIN_M2, HIGH);
    digitalWrite(DPIN_M1, LOW);
  }
  if (USTEP_DIVISOR == 32) {
    // Set for 1/16 stepping
    digitalWrite(DPIN_M2, HIGH);
    digitalWrite(DPIN_M1, HIGH);
  }
  // Set up the pins for inputs
  pinMode(DPIN_DRIVER_FAULT, INPUT);
  pinMode(DPIN_LIMIT_CLOSED, INPUT);
  pinMode(DPIN_LIMIT_OPENED, INPUT);
  pinMode(DPIN_RAIN, INPUT);
}

// Main loop
void loop() {
  // determine current state, condition, and fault
  state = determine_state();  // open/closed
// next line removed because we are going to use analog rain triggering
// condition = determine_cond(); // rain or not
  fault = determine_fault(); // check for motor fault
  // print if there is a state change
  if (state != last_state) {
    if (state==STATE_OPENED) {
      Serial.println("Current state is OPENED");
    }
    if (state==STATE_CLOSED) {
      Serial.println("Current state is CLOSED");
    }
    if (state==STATE_ERROR) {
      Serial.println("Current state is ERROR -- both limits triggered");
    }
    if (state==STATE_UNKNOWN) {
      Serial.println("Current state is UNKNOWN");
    }
  }
  last_state = state;
  // look for user input
  while (Serial.available() > 0) {
    // All commands are single characters
    char commandchar = Serial.read();
    Serial.print("Received command: ");
    Serial.println(commandchar);
    switch (commandchar) {
      case 'o':
      case 'O':
        requeststate = STATE_OPENED;
        Serial.println("User requested cover to open");
        break;
      case 'f':
      case 'F':
        requeststate = STATE_FORCEOPEN;
        Serial.println("User forced cover to open as often as possible");
        break;
      case 'c':
      case 'C':
        requeststate = STATE_CLOSED;
        Serial.println("User requested cover to close");
        break;
      case '?':
        report_state();
        break;
      case 'v':
      case 'V':
        verbose_logging = !verbose_logging;
        if (verbose_logging) {
          Serial.println("User requested verbose logging");
        }
        else {
          Serial.println("Verbose logging is off");          
        }
        break;
      default:
        Serial.println("Command not recognized");
    }
  }
  if (condition == COND_RAIN) {
    if (requeststate == STATE_OPENED) // switch requeststate if there is rain to keep closed after 
      requeststate = STATE_CLOSED;
    if (state != STATE_CLOSED) {
      Serial.println("Rain forced cover to close");
      fault = move_to_limit(DIRECTION_CLOSE, &duration);
      if (fault == FAULT_MOTOR_SHORT) 
        Serial.print ("Cover attempted to close but failed to reach limit in ");
      else
        Serial.print("Cover closed successfully in ");
      Serial.print(duration);
      Serial.println(" seconds");
    }
  }
  else {
    switch(requeststate) {
      case (STATE_CLOSED):
        if (state == STATE_CLOSED)
          break;
        fault = move_to_limit(DIRECTION_CLOSE, &duration);
        if (fault == FAULT_MOTOR_SHORT) 
          Serial.print("Cover attempted to close but failed to reach limit in ");
        else
          Serial.print("Cover closed successfully in ");
        Serial.print(duration);
        Serial.println(" seconds");
        break;
      case (STATE_OPENED):  
      case (STATE_FORCEOPEN):
        if (state == STATE_OPENED)
          break;
        fault = move_to_limit(DIRECTION_OPEN, &duration);    
        if (fault == FAULT_MOTOR_SHORT) 
          Serial.print("Cover attempted to open but failed to reach limit in ");
        else
          Serial.print("Cover opened successfully in ");
        Serial.print(duration);
        Serial.println(" seconds");
      default:
        break;  // do nothing 
    }
  }
  // Poll rain sensor and determine average and standard deviation
  i = 0;
  rain_mv_sum = 0;
  rain_mv_sumsqr = 0;
  while(i<NUMPNTS) {
    // read rain sensor analog input
    rain_mv = MILLIVOLTS_PER_COUNT * float(analogRead(APIN_RAIN));
    // sum the values
    rain_mv_sum += rain_mv;
    rain_mv_sumsqr += rain_mv*rain_mv;
    i+=1;
    delay(1000/NUMPNTS); // Polling the rain sensor every 1000ms / N
  } // repeat loop for 1 second user interaction time
  // calculate avg and standard deviation
  rain_mv_avg = rain_mv_sum / float(NUMPNTS);
  rain_mv_std = sqrt((1/float(NUMPNTS-1)) * (rain_mv_sumsqr - float(NUMPNTS)*rain_mv_avg*rain_mv_avg));
  if (verbose_logging) {
    Serial.print("Rainsensor avg = ");
    Serial.print(rain_mv_avg);
    Serial.print(" std = ");
    Serial.print(rain_mv_std);
    Serial.println(" mV");
  }
  if ((rain_mv_avg+2*rain_mv_std) < RAIN_THRESHOLD_MV) {
    condition = COND_RAIN;
  }
  else {
    condition = COND_CLEAR;
  }
}

void report_state() {
  Serial.print("Status: State = ");
  switch (state) {
    case STATE_CLOSED:
      Serial.print("closed ");
      break;
    case STATE_OPENED:
      Serial.print("opened ");
      break;
    default:
      Serial.print("unknown ");
  }
  switch (fault) {
    case FAULT_MOTOR_SHORT:
      Serial.print("fault = motor ran short ");
      break;
    case FAULT_DRIVER_FAULT:
      Serial.print("fault = driver in fault ");
  }
  if (condition == COND_RAIN){
    Serial.print("condition = RAIN ");    
  }
  Serial.print("Analog Rain Signal = ");
  Serial.print(int(rain_mv_avg));
  Serial.print(" std = ");
  Serial.print(rain_mv_std);
  Serial.println(" mV");
}

int determine_state() {
  int retcode;

  retcode = STATE_UNKNOWN;
  if (digitalRead(DPIN_LIMIT_CLOSED) == LOW)
    retcode = STATE_CLOSED; 
  if (digitalRead(DPIN_LIMIT_OPENED) == LOW) {
      if (retcode == STATE_CLOSED) retcode = STATE_ERROR;
      else retcode = STATE_OPENED;
  }
  return retcode; 
}

int determine_cond() {
  int retcode;

  if (digitalRead(DPIN_RAIN) == LOW)
    retcode = COND_RAIN; 
  else 
    retcode = COND_CLEAR;
  return retcode; 
}

int determine_fault() {
  int retcode;

  if (digitalRead(DPIN_DRIVER_FAULT) == LOW)
    retcode = FAULT_DRIVER_FAULT; 
  else 
    retcode = FAULT_NONE;
  return retcode; 
}

int move_to_limit(int dir, float* elapsed_sec) {
  int continu = 1;
  int retcode, dpin_limit;
  unsigned long startmillis, timeout;

  startmillis = millis();
  // Setup digital pin for sought limit switch
  if (dir == DIRECTION_CLOSE) {
      dpin_limit = DPIN_LIMIT_CLOSED;
  } else {
      dpin_limit = DPIN_LIMIT_OPENED;    
  }
  // Enable the driver (turn current on)
  stepper.enableOutputs();
  // start motion towards sought limit
  stepper.move(dir * STEPS_FULLMOVE * USTEP_DIVISOR);
  // set timeout TIMEOUT_MILLIS milliseconds after current time
  timeout = startmillis + TIMEOUT_MILLIS;
  // run the motor
  retcode = FAULT_MOTOR_SHORT;  // default return code is that the motor didn't achieve limit
  while ( (stepper.distanceToGo() != 0) && (millis() < timeout) ) {
    if (digitalRead(dpin_limit) == LOW) {
      retcode = FAULT_NONE;
      stepper.stop();  // tell the stepper to stop via decelleration ramp
    }
    stepper.run();  // keep running the motor
  }
  // Disable the driver (turn current off)
  stepper.disableOutputs();
  // calculate the elapsed time
  *elapsed_sec = (float)(millis() - startmillis)/1000;
  // return the code
  return retcode;
}

