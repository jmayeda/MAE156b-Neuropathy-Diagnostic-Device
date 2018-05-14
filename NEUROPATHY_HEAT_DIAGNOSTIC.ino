/*NEUROPATHY_HEAT_DIAGNOSTIC.ino
*
* @authors: Naif Alzahri, Thomas Ibbetson, Jason Mayeda, Inri Rodriguez
* @date: Feb-June 2018
* @about: Program to control a heat diagnostic test for WinSanTor neuropathy trials.
**/

#include <math.h>
#include <LiquidCrystal.h>

// ================================== INPUTS 11 ================================ //
#define inputPin_TS_Ambient_1     A1
#define inputPin_TS_Ambient_2     A2
#define inputPin_TS_Ambient_3     A3
#define inputPin_TS_Ambient_4     A4

#define inputPin_TS_Focal         A0

#define inputPin_StartButton      2   //interrupt enabled


#define inputPin_MS_Echo          5
#define inputPin_MS_Reflect       6

#define GREEN_LED_PIN             7
#define RED_LED_PIN               8

// ===================== OUTPUTS  2
#define outputPin_MS_Trigger       4
#define outputPin_Heat_Ambient    13
#define outputPin_Heat_Focal      12


// ===================== Config for Sampling and Controllers  ====================== //
#define BETA                      3380  // *linear*ish  coefficient of thermistor resistance to temperature

// Ambient Controller
#define KP_AMBIENT                25     // 0.035 proportional gain for error
#define KI_AMBIENT                0.1   //integral gain for error
#define AMBIENT_INT_MAX           100   // TODO: prevent integrator wind-up
#define ROOM_TEMP_K               298.15 // room temp in Kelvin
#define REF_TEMP_AMBIENT          30.0    // ambient temperature setpoint

// Focal Heating Controller
#define KP_FOCAL                  15    // TODO: tune PID gains
#define KI_FOCAL                  0.5
#define KD_FOCAL                  0.7
#define FOCAL_INT_MAX             100   // TODO: prevent integrator wind-up
#define REF_TEMP_CHANGE_FOCAL     1.0   // we want a 1 degree/s change for focal heating
#define TEMP_CUTOFF               50.0  // we don't want to increase beyond 50 deg C

// FROM ULTRASONIC SENSOR CODE
#define SETUP_TIME_S              5  // amount of time to calibrate the ultrasonic sensor
#define DIST_THRESHOLD            1.1 // trigger the end of the test

// Safety
#define SAFETY_TEMP               49 // maximum temperature of system (celsius)

// Ready up --> indicates that ambient temp is between min and max and therefore is ready to test
#define Min_Test_Temp             28.0
#define Max_Test_Temp             32.0

// ============================== TYPE DEFINITION ========================== //
typedef struct state_t { // important system state parameters
    long time_Milli;     // time at which state was measured
    float temp_Focal;    // glass temperature at the focal source
    float rate_Focal ;   // degrees per second heating rate of focal point
    float temp_Ambient;  // ambient temperature in the chamber
    float dist_Ultra;    // distance measured by ultrasonic sensor
    int duty_Ambient;    // duty that will be applied to ambient heaters
    int duty_Focal;      // duty that will be applied to focal heater
    int motion_Detected; // output of the motion sensor(detected: 1, nothing: 0)
    int running;         // running state - 1 or 0,
} state_t;


// ==================================== Function Declaration ==================================== //
float   Calibrate_Ultra(int TrigPin, int EchoPin, int setupTimeS);
int     Controller_Ambient(float Ref_Temp_Ambient);
int     Controller_Focal(float Ref_Temp_Change_Focal);
state_t MEASURE();
int     read_MotionSensors();
int     read_MS_Reflect(int pin);
float   read_MS_Ultra(int TrigPin,int EchoPin);
float   read_Thermistor(int pin);
void    reset_ISR();
void    safety();
void    shutdown();
void    write_LCD_message(char message);
void    write_LCD_temp(float temp);


// ==================================== GLOBAL VARIABLE ==================================== //
state_t state_Now   = {0,   0.0,   0.0,   0.0,   0.0,   0,   0,   0,  0}; //current state
state_t state_Last  = {0,   0.0,   0.0,   0.0,   0.0,   0,   0,   0,  0}; // previous state
state_t state_Start = {0,   0.0,   0.0,   0.0,   0.0,   0,   0,   0,  0}; // state at beginning of test
state_t state_End   = {0,   0.0,   0.0,   0.0,   0.0,   0,   0,   0,  0}; // state at end of test

//volatile int buttonState = 0; // Flag for reseting the test
int testType = 0;             // Flag for type of test (contact: 0 (default), non-contact: 1)
int test_complete = FALSE;        // signal


// ================================ SETUP =================================== //


void setup() {
  // Initialize serial monitor
  Serial.begin(9600);

  // setup pins
  pinMode(inputPin_TS_Focal, INPUT);
  pinMode(inputPin_TS_Ambient_1, INPUT);
  pinMode(inputPin_TS_Ambient_2, INPUT);
  pinMode(inputPin_TS_Ambient_3, INPUT);
  pinMode(inputPin_TS_Ambient_4, INPUT);

  pinMode(inputPin_StartButton, INPUT) ;

  pinMode(inputPin_MS_Reflect, INPUT);
  pinMode(inputPin_MS_Trigger, OUTPUT);
  pinMode(inputPin_MS_Echo, INPUT);

  pinMode(outputPin_Heat_Focal, OUTPUT);
  pinMode(outputPin_Heat_Ambient, OUTPUT);

  // interrupt pin for the reset button
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), reset_ISR, CHANGE);

  //Chillax bruh
  delay(1000);

}




// ============================== MAIN LOOP ================================= //
void loop() {
  state_Last = state_Now; 
  state_Now = MEASURE(); //update current state with temperatures, time,
  // Ambient Heating controller


  int unsafe = TRUE; // assume conditions are bad
  while (unsafe){ // loop until temperatures are safe
    if ((state_Now.temp_Focal > SAFETY_TEMP) || (state_Now.temp_Ambient > SAFETY_TEMP)) {
      // we're getting too hot up in here
      analogWrite(outputPin_Heat_Ambient, 0); //turn off ambient
      shutdown(); // turn off everything
      write_LCD_message('Overheat');
      state_Now.running = FALSE; // test running flag set to FALSE
      delay(5000); // 5 seconds to cool down before checking again
    }
    else {
      unsafe = FALSE; // conditions are not actually bad, continue
      analogWrite(outputPin_Heat_Ambient, state.duty_Focal);
    }
  }



  if (state_Now.running == FALSE){
    // If test is not running but temps are safe
    shutdown();
    if (state_Now.temp_Ambient > Min_Test_Temp && state_Now.temp_Ambient < Max_Test_Temp){
      write_LCD_message('Ready for test')
    }
    else{
      write_LCD_message('Resetting...');
    }

  }
  else {
    // write to focal heater
    state_Now.duty_Focal = Controller_Focal(REF_TEMP_CHANGE_FOCAL);
    analogWrite(outputPin_Heat_Focal, state_Now.duty_Focal);
  }



  if (state_Now.motion_Detected) {
    state_Now.running = FALSE;
    test_complete = TRUE;
    state_End = state_Now;

    while (test_complete) {
      write_LCD_temp(state_End.temp_Focal);
    }

  }

  write_LCD_temp(state_Now.temp_Focal);


}


// =========================== DEFINED FUNCTIONS ============================ //


/*
 * int Controller_Ambient()
 *
 * Implement controller algorithim for the ambient heater to maintain
 * the housing temperature at ref_Temp_Ambient.
 */
int Controller_Ambient(float Ref_Temp_Ambient){
  // static variables initialize once, the first time the function is called
  static float cumalitiveError = 0;
  static float I_term = 0;
  float P_term = 0;

  // update current error
  float currError = Ref_Temp_Ambient - state_Now.temp_Ambient;

  // sum up cumalitive error
  cumalitiveError += currError;

  // prevent integrator wind-up
  if (cumalitiveError > AMBIENT_INT_MAX) {
      cumalitiveError = AMBIENT_INT_MAX;
  } else if (cumalitiveError < -AMBIENT_INT_MAX) {
      cumalitiveError = -AMBIENT_INT_MAX;
  }

  // PI control
  I_term = KI_AMBIENT * cumalitiveError;
  P_term = KP_AMBIENT * currError;

  // keep control outputs between analogWrite bounds
  int control_output = (int) (I_term + P_term);
  if (control_output > 255) {
      control_output = 255;
  } else if (control_output < 0) {
      control_output = 0;
  }
  state_Now.duty_Ambient = control_output;

  return state_Now.duty_Ambient;
}

/*
 * int controllerFocal()
 *
 * runs control algorithm to maintain the heating rate at the focal
 * source to be at Ref_Temp_Change_Focal degrees/sec
 */
 int Controller_Focal(float Ref_Temp_Change_Focal){
   // local variables
   static float cumalitiveError = 0; // static variables persist b.t function calls
   static float lastError = 0;       // static variables persist b.t function calls
   float currError;
   float P_term;
   float I_term;
   float D_term;
   int control_output;

   float dTemp = state_Now.temp_Focal - state_Last.temp_Focal;
   long dTime = (state_Now.time_Milli - state_Last.time_Milli);
   state_Now.rate_Focal = dTemp/(dTime/1000.0);

   // current error found by comparing reference temperature change with actual temp change
   currError = Ref_Temp_Change_Focal - state_Now.rate_Focal;

   // update cumalitive error
   cumalitiveError += currError;

   // prevent integrator wind-up
   if (cumalitiveError > FOCAL_INT_MAX) {
       cumalitiveError = FOCAL_INT_MAX;
   } else if (cumalitiveError < -FOCAL_INT_MAX) {
       cumalitiveError = -FOCAL_INT_MAX;
   }

   // PID
   P_term = KP_FOCAL * currError;
   I_term = KI_FOCAL * cumalitiveError;
   D_term = KD_FOCAL * (currError - lastError);

   // calculate control signal
   control_output = (int) (P_term + I_term + D_term);

   if (control_output > 255) {
       control_output = 255;
   }
   else if (control_output < 0) {
       control_output = 0;
   }
   state_Now.duty_Focal = control_output;

   // update the previous error
   lastError = currError;

   return state_Now.duty_Focal;
 }








state_t MEASURE() {
  state_t state; //time, F temp ,F rate, A temp, dist, A duty, F duty , motion? running?
  state.time_Milli = millis() - start_millis;
  state.temp_Focal = readThermistor(inputPin_TempSensor_Focal);
  state.rate_Focal = ( state.temp_Focal - state_Last.temp_Focal ) / (state.time_Milli - state_Last.time_Milli) ;
  state.temp_Ambient = readThermistor(inputPin_TempSensor_Ambient); //TODO AVERAGE ALL AMBIENT THERMISTORS
  state.dist_Ultra = read_MS_Ultra(inputPin_MS_Echo,outputPin_MS_Trigger);
  state.duty_Ambient = Controller_Ambient(Ref_Temp_Ambient);
  state.duty_Focal = Controller_Focal(Ref_Temp_Change_Focal); //TODO check rate numbers
  state.motion_Detected = read_MotionSensors(inputPin_MS_Echo,inputPin_MS_Trigger,inputPin_MS_Reflect); //TODO combine motion sensors

  //
  // long time_Milli;     // time at which state was measured
  // float temp_Focal;    // glass temperature at the focal source
  // float rate_Focal ;   // degrees per second heating rate of focal point
  // float temp_Ambient;  // ambient temperature in the chamber
  // float dist_Ultra;    // distance measured by ultrasonic sensor
  // int duty_Ambient;    // duty that will be applied to ambient heaters
  // int duty_Focal;      // duty that will be applied to focal heater
  // int motion_Detected; // outputs if we think there is motion : filters output from both ultrasonic and reflective sensors
  // int running;         // running state - 1 or 0,
  return state;
}





/**
 * Calibrate the ultrasonic sensor with the distance to the stationary hand.
 * @param  trigPin    ultrasonic sensor
 * @param  echoPin    ultrasonic sensor
 * @param  setupTimeS calibration time
 * @return            average distance to stationary hand
 */
float Calibrate_Ultra(int TrigPin, int EchoPin, int setupTimeS) {
  int count = 0;
  long startTime;
  float dist_mm_calibrated = 0;

  setupTimeS *= 1000; // convert from seconds to milliseconds

  startTime = millis(); // get the start time
  digitalWrite(RED_LED_PIN, HIGH);

  while (millis() - startTime < setupTimeS) {
    dist_mm_calibrated += read_MS_Ultra(trigPin, echoPin);
    count++;
  }
  // average the calibrated value
  dist_mm_calibrated /= count;
  digitalWrite(RED_LED_PIN, LOW);

  return dist_mm_calibrated;
}




int read_MotionSensors(){
  //TODO COMBINE READINGS FROM REFLECTIVE AND ULTRASONIC SENSORS TO RETURN A TRUE OR FALSE VALUE FOR THE HAND BEING PRESENT
}

int read_MS_Reflect(int pin){
      return digitalRead(inputPin_MS_Reflect);
}

/**
 * Poll the ultrasonic sensor.
 * @param  trigPin ultrasonic sensor
 * @param  echoPin ultrasonic sensor
 * @return         distance from sensor to object in mm
 */
float read_MS_Ultra(int TrigPin,int EchoPin){
  float dist_mm;    // distance in mm
  int duration_mcs; // duration in microseconds
  // Clears the TRIG_PIN
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);

  // Sets the TRIG_PIN on HIGH state for 10 micro seconds
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);

  // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
  duration_mcs = pulseIn(EchoPin, HIGH);

  // Calculating the distance in mm
  dist_mm = duration_mcs * 0.034/2.0;

  return dist_mm;
}


float read_Thermistor(int pin){
  float ROOM_TEMP_K = 298.15;
  float sensor_voltage = 5.0/1024.0 * (float) analogRead(pin);
  float Thermistor_R;

  if (abs(sensor_voltage - 2.5) < 0.0001) {
      Thermistor_R = 10000.0;
  }
  else {
      //Thermistor_R = R2_THERM * (sensor_voltage / 5.0) * (1.0 - (sensor_voltage / 5.0));
      Thermistor_R = R_at_25DEG / ((1023.0/analogRead(pin)) - 1.0);
  }
  float tempK = 1.0 / ( (-1.0/BETA) * (log(Thermistor_R / 10000)) + (1.0 / ROOM_TEMP_K) );

  float tempC = tempK - 273.0;

  return tempC;
  }
}


void shutdown(){
  analogWrite(outputPin_Heat_Focal,0);
  state_Now.running = FALSE;
}

void write_LCD_message(char message){
 //TODO write to top line :message:
 //https://www.arduino.cc/en/Tutorial/LiquidCrystalDisplay
 // liquid crystal libarry
}
void write_LCD_temp(float temp){
  //TODO write to bottom line of LCD "Temp C: ##.##"
}




/*
 * void reset_ISR()
 *
 * @about: interrupt service routine for start button
 * @param: none
 * @return: none
 */
void reset_ISR() {
  // reset the test
  //  buttonState = digitalRead(BUTTON_PIN); is this ever used?

  // if test has already completed, reset on interupt
  if (test_complete == TRUE){
    state_Now.running = FALSE;
    test_complete = FALSE;
  }
  // if test is running, stop on interrupt
  else if (state_Now.running == TRUE) {
    state_Now.running = FALSE;
  } // if test is not running, start on interrupt
  else if (state_Now.running == TRUE) {
    state_Now.running = FALSE;
  }
}
