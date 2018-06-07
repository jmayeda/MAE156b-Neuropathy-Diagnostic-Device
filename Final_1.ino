#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CONSTANTS

/* HEATERS */
  // Ambient //
const int KI_AMBIENT       = 0; //PI controller gains for ambient temp
const int KP_AMBIENT       = 30;
const float GLASS_TEMP_SETPOINT = 30.0; // what temp are we aiming for in ambient
const int THERMISTOR_AMBIENT_NUM = 2;
const int AMBIENT_INDEX = 3; // Ambient  thermistor is the 2nd index of therm array

  // Focal //
const float FOCAL_PWM_VAL  = 0.25 * 255; // set to constant output
const int FOCAL_INDEX = 2; // FOCAL thermistor is the third index of thermistor array

// LCD Constants
const int TEMP_START_COL   = 11; //[2,6] start coordinate
const int TEMP_START_ROW   = 2;
const int TIME_START_COL   = 11; //[3,6] start coordinate currently
const int TIME_START_ROW   = 3;
const int STATUS_START_COL = 8; //1,8 start coordinate for word STATUS
const int STATUS_START_ROW = 1;
const int LCD_MAX_COL      = 20; // max dimension of screen currently 3x20
const int LCD_MAX_ROW      = 3;
const int WAIT_TIME        = 5000; //time to reset

// Safety //
const float SAFETY_CUTOFF_TEMP  = 49.0; //what is too hot?

/* Thermistor Constants */
const int BETA             = 3380;
const float ROOM_TEMP_K    = 298.0;
const int NUM_THERM        = 4;

/* Ultrasonic Constants*/
const int SETUP_TIME_S = 5;  // amount of time to calibrate the ultrasonic sensor
const float DIST_THRESHOLD = 1.3; // trigger the end of the test

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Pin definitions

//BUTTONS
const int START_BUTTON_PIN = 18;
const int RESET_BUTTON_PIN = 3;
const int STOP_BUTTON_PIN  = 2;

//LED
const int GREEN_LED_PIN    = 38;
const int RED_LED_PIN      = 40;
const int YELLOW_LED_PIN   = 42;

/* Thermistor Pins */
#define THERM0_PIN        A0
#define THERM1_PIN        A1
#define THERM2_PIN        A2
#define THERM3_PIN        A3

// Power measurement
const int SHUNT_PIN        = A7;

/* PWM pins */
const int FOCAL_PWM_PIN    = 4;
const int AMBIENT_PWM_PIN  = 5;

// UltraSonic
const int TRIG_PIN         = 12;  // Ultrasonic sensor
const int ECHO_PIN         = 13;  // Ultrasonic sensor



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// State Type and other global variables

// LCD Object
LiquidCrystal_I2C lcd(0x27,20,4);

// state type
typedef struct state_t {
    //  Status of device
    int Stop;
    int Reset;
    int Ready;
    int Running;
    int Done;

    // Heater Data
    int PWM_Ambient;
    int PWM_Focal;
    float Bulb_Power;

    //  Temperature Data
    float Temp[NUM_THERM];
    float Temp_Ambient;
    float Temp_Focal;

    //  Ultrasonic Data
    float avgDistMM;
    float distance;

    //Results
    unsigned long Time;
    unsigned long Results_Time;
    float         Results_Temp;


}state_t;
state_t STATUS;

volatile float resetTimer    = 0; // initialize timer //TODO: replace with ambient temp condition instead of timer.

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Function Declarations
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Debugging
void STATUS2Serial();


// sensors
void MEASURE();
float pollUltraSensor(int trigPin, int echoPin);
float calibrateUltraSensor(int trigPin, int echoPin, int setupTimeS);
float readThermistor(int pin);

// heaters
void writeToFocalHeater(int PWM);
void writeToAmbientHeater(int PWM);
int controllerAmbient(float ref_Temp_Ambient);

// lcd
void setupLCD();
void updateLCD();
void updateStatusLCD(char *message);
void clearLCDLine(int startColIdx, int endColIdx, int row);

// Interrupt routines
void RESET_ISR();
void START_ISR();
void STOP_ISR();

//Safety
void SAFETY();



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SETUP
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
    // initialize state of device
    STATUS.Stop     = true; // device is not stopped
    STATUS.Reset    = false; // device has been reset
    STATUS.Ready    = false; // device is not ready
    STATUS.Running  = false; // device is not running
    STATUS.Done     = false;
    // setup the pin modes
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(YELLOW_LED_PIN, OUTPUT);
    pinMode(START_BUTTON_PIN, INPUT);
    pinMode(RESET_BUTTON_PIN, INPUT);
    pinMode(STOP_BUTTON_PIN, INPUT);

    pinMode(THERM0_PIN, INPUT);
    pinMode(THERM1_PIN, INPUT);
    pinMode(THERM2_PIN, INPUT);
    pinMode(THERM3_PIN, INPUT);

    pinMode(FOCAL_PWM_PIN,OUTPUT);
    pinMode(AMBIENT_PWM_PIN,OUTPUT);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    pinMode(SHUNT_PIN,INPUT);



    setupLCD();
    updateStatusLCD("Initializing..");
    delay(1000);
    updateLCD(99, 59); // NOTE: for debugging REMOVE later

    attachInterrupt(digitalPinToInterrupt(START_BUTTON_PIN), START_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RESET_BUTTON_PIN), RESET_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(STOP_BUTTON_PIN), STOP_ISR, RISING);

    Serial.begin(9600);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//LOOP
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
    MEASURE();
    SAFETY();
    Serial.print(" MAIN ");
    STATUS2Serial();



    if (STATUS.Stop){
        Serial.print(" STOP ");
        STOPPED();
        // stays in limbo until hit RESET button
        // turn off focal heater and ambient heater
    }
    else if (STATUS.Reset && !STATUS.Stop){
        Serial.print(" RESET ");
        RESET();
        // ambient heating controller
    }
    else if (STATUS.Ready && !STATUS.Stop){
        Serial.print(" READY ");
        READY();
        // ambient temp is REACHED
        // ready for motion sensors calibration
        // move to START when START button is pressed
    }
//////// RUN THE TEST ////////////
    else if (STATUS.Running && !STATUS.Stop){   // run the test
        START(); // begin test
    }
    //////// END THE TEST ////////////



}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//STATE FUNCTIONS IN LOOP
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////
void STOPPED(){
    // indicators
    digitalWrite(YELLOW_LED_PIN,LOW);
    digitalWrite(RED_LED_PIN,HIGH);
    digitalWrite(GREEN_LED_PIN,LOW);
    writeToFocalHeater(0);
    writeToAmbientHeater(0);

    if (STATUS.Done){ // if we just finished a test
        updateStatusLCD("Results");
        updateLCD(STATUS.Results_Temp,STATUS.Results_Time);
    }
    else{ //otherwise
        updateStatusLCD("Stopped");
        updateLCD(STATUS.Temp_Focal,0);
    }
    return;
}

/////////////////////////////
void RESET(){

    SAFETY();
    // indicators
    digitalWrite(GREEN_LED_PIN,LOW);
    digitalWrite(RED_LED_PIN,LOW);
    digitalWrite(YELLOW_LED_PIN,HIGH);

    // Run Ambient controller but make sure focal is off
    STATUS.PWM_Ambient = controllerAmbient(GLASS_TEMP_SETPOINT);// &STATUS);
    writeToAmbientHeater(STATUS.PWM_Ambient);
    writeToFocalHeater(0);

    updateStatusLCD("Resetting");
    updateLCD(STATUS.Temp_Focal,0);

    if ((millis()-resetTimer) > WAIT_TIME) {
        STATUS.Reset = false;
        STATUS.Ready = true ;
    }

    return;
}

/////////////////////////////
void READY(){

    SAFETY();
    // indicators
    digitalWrite(YELLOW_LED_PIN,LOW);
    digitalWrite(RED_LED_PIN,LOW);
    digitalWrite(GREEN_LED_PIN,HIGH);

    // Still run Ambient controller, focal still off
    STATUS.PWM_Ambient = controllerAmbient(GLASS_TEMP_SETPOINT);// &STATUS);
    writeToAmbientHeater(STATUS.PWM_Ambient);
    writeToFocalHeater(0);
    updateStatusLCD("Ready");
    updateLCD(STATUS.Temp_Focal,0);

    return;
}

/////////////////////////////
void START(){
    digitalWrite(RED_LED_PIN,LOW);
    digitalWrite(GREEN_LED_PIN,HIGH);
    digitalWrite(YELLOW_LED_PIN,HIGH);
    updateStatusLCD("Running");
    // CALIBRATE
    unsigned long timeStart = millis()/1000;

    while(!STATUS.Stop) {
        MEASURE();
        SAFETY();
        Serial.print(" RUNNING ");
        STATUS2Serial();

        /* 1. ambient controller */
        STATUS.PWM_Ambient = controllerAmbient(GLASS_TEMP_SETPOINT);// &STATUS);
        writeToAmbientHeater(STATUS.PWM_Ambient);
        /* 2. measure temperatures and current measurement */
        STATUS.PWM_Focal = FOCAL_PWM_VAL;
        writeToFocalHeater(STATUS.PWM_Focal);
        /* 3. run focal controller
        *
        */

        STATUS.Time = millis()/1000 - timeStart;
        updateLCD(STATUS.Temp_Focal,STATUS.Time);
        delay(100);

    }
    unsigned long timeEnd = millis()/1000;
    STATUS.Results_Time = timeEnd-timeStart;
    STATUS.Results_Temp = STATUS.Temp_Focal;
    updateLCD(STATUS.Results_Temp,STATUS.Results_Time);
    return;
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CONTROLLERS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int controllerAmbient(float ref_Temp_Ambient){ //state_t* state) {
  // static variables initialize once, the first time the function is called
  static float cumalitiveError = 0;
  static float I_term = 0;
  float P_term = 0;

  // update current error
  float currError = ref_Temp_Ambient - STATUS.Temp[THERMISTOR_AMBIENT_NUM];

  // sum up cumalitive error
  cumalitiveError += currError;

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
  int ambientPWM = control_output;

  return ambientPWM;
}

//TODO FOCAL CONTROLLER TO REFERENCE POWER LEVEL



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ISR FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////
void RESET_ISR() {
    // start reset if system is stopped
    if (STATUS.Stop == true){
        STATUS.Reset = true;
        STATUS.Ready = false;
        STATUS.Stop   = false;
        STATUS.Running = false;
        STATUS.Done = false;
        //*STATUS = "Resetting...";

    }
    resetTimer = millis();
    return;
    // if system is ready or already running, change nothing
}

/////////////////////////////
void STOP_ISR() {
    // stop no matter what
    STATUS.Stop  = true;
    STATUS.Reset = false;
    STATUS.Ready = false;
    STATUS.Running = false;
    STATUS.Done = false;
    //    *STATUS = "Stopped";
    return;
}

/////////////////////////////
void START_ISR() {
    // if ready, start the test
    if (STATUS.Running == true){
        STATUS.Running  = false ;
        STATUS.Stop     = true ;
        STATUS.Done     = true ;

    }
    else if (STATUS.Ready == true){
        STATUS.Running  = true;
        STATUS.Ready    = false ;
        STATUS.Stop     = false ;
        STATUS.Reset    = false ;
        STATUS.Done     = false ;
    }

    return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//LCD FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setupLCD() {
    lcd.init();
    lcd.backlight();
    lcd.setCursor(5,0);
    lcd.print("WinSanTor");
    lcd.setCursor(0,1);
    lcd.print("Status: ");
    lcd.setCursor(2,2);
    lcd.print("Temp(C): ");
    lcd.print("100");
    lcd.setCursor(2,3);
    lcd.print("Time(s): ");
    lcd.print("60");
}

void updateLCD(int temp, unsigned long times) {
    // set cursor to print the temperature
    clearLCDLine(TEMP_START_COL, LCD_MAX_COL, TEMP_START_ROW);
    lcd.print(String(temp));

    clearLCDLine(TIME_START_COL, LCD_MAX_COL, TIME_START_ROW);
    lcd.print(String(times));
}

void updateStatusLCD(char *message) {
    clearLCDLine(STATUS_START_COL, LCD_MAX_COL, STATUS_START_ROW);
    lcd.print(message);
}

void clearLCDLine(int startColIdx, int endColIdx, int row) {
    lcd.setCursor(startColIdx, row);
    for (int i=startColIdx; i<endColIdx; i++) {
        lcd.print(" ");
    }
    lcd.setCursor(startColIdx, row);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SENSOR MEASUREMENTS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MEASURE(){
    STATUS.Temp[0] = readThermistor(THERM0_PIN);
    STATUS.Temp[1] = readThermistor(THERM1_PIN);
    STATUS.Temp[2] = readThermistor(THERM2_PIN);
    STATUS.Temp[3] = readThermistor(THERM3_PIN);

    STATUS.Temp_Focal = STATUS.Temp[FOCAL_INDEX];
    STATUS.Temp_Ambient = STATUS.Temp[AMBIENT_INDEX];

}




/**
 * Calibrate the ultrasonic sensor with the distance to the stationary hand.
 * @param  trigPin    ultrasonic sensor
 * @param  echoPin    ultrasonic sensor
 * @param  setupTimeS calibration time
 * @return            average distance to stationary hand
 */
float calibrateUltraSensor(int trigPin, int echoPin, int setupTimeS) {
  int count = 0;
  long startTime;
  float dist_mm_calibrated = 0;

  setupTimeS *= 1000; // convert from seconds to milliseconds

  startTime = millis(); // get the start time
  digitalWrite(RED_LED_PIN, HIGH);

  while (millis() - startTime < setupTimeS)
  {
    dist_mm_calibrated += pollUltraSensor(trigPin, echoPin);
    count++;
  }
  // average the calibrated value
  dist_mm_calibrated /= count;
  digitalWrite(RED_LED_PIN, LOW);

  return dist_mm_calibrated;
}

/**
 * Poll the ultrasonic sensor.
 * @param  trigPin ultrasonic sensor
 * @param  echoPin ultrasonic sensor
 * @return         distance from sensor to object in mm
 */
float pollUltraSensor(int trigPin, int echoPin) {
  float dist_mm;    // distance in mm
  int duration_mcs; // duration in microseconds
  // Clears the TRIG_PIN
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the TRIG_PIN on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
  duration_mcs = pulseIn(echoPin, HIGH);  // TODO: don't use pulseIn

  // Calculating the distance in mm
  dist_mm = duration_mcs * 0.034/2.0;

  return dist_mm;
}

/**
 * Read the thermistor for the voltage divider.
 * @param  pin analog pin to read from
 * @return     temperature in celsius
 */
float readThermistor(int pin) {
    // declare local variables
    float sensor_voltage, Thermistor_R;
    float tempK, tempC;
    float volts_raw = analogRead(pin);
    // read sensor voltage from the voltage divider
    sensor_voltage = 5.0/1024.0 * (float) volts_raw;

    // avoid a division by zero
    if (abs(sensor_voltage - 2.5) < 0.0001) Thermistor_R = 10000.0;
    else Thermistor_R = 10000.0 / ((1023.0/volts_raw) - 1.0);

    // thermistor resistance -> temperature in Kelvin (from datasheet)
    tempK = 1.0 / ((-1.0/BETA) * (log(Thermistor_R/10000.0)) + (1.0/ROOM_TEMP_K) );

    // temp, K -> temp, C
    tempC = tempK - 273.0;

    return tempC;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//HEATERS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void writeToFocalHeater(int PWM) {
  analogWrite(FOCAL_PWM_PIN, PWM);
}

void writeToAmbientHeater(int PWM) {
  analogWrite(AMBIENT_PWM_PIN, PWM);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SAFETY
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SAFETY(){

    // if (STATUS.Temp_Ambient > SAFETY_CUTOFF_TEMP || STATUS.Temp_Focal > SAFETY_CUTOFF_TEMP) {
    //     writeToFocalHeater(0);
    //     writeToAmbientHeater(0);
    //     updateStatusLCD("Overheat");
    //     float cooling = millis();
    //     while (millis() - cooling < 10.0 ){
    //         updateLCD(STATUS.Temp_Focal, millis()-cooling );
    //     }
    // }
    // DEBUG: changed from if() to while() because it wasnt stopping the test
    while (STATUS.Temp_Ambient > SAFETY_CUTOFF_TEMP || STATUS.Temp_Focal > SAFETY_CUTOFF_TEMP) {
        writeToFocalHeater(0);
        writeToAmbientHeater(0);
        updateStatusLCD("Overheated.");
        digitalWrite(RED_LED_PIN, HIGH);
        delay(250);
        digitalWrite(RED_LED_PIN, LOW);
    }

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Debugging
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void STATUS2Serial(){
    Serial.print(",     Temps : ");
    for (int i=0; i<NUM_THERM; i++){
      Serial.print( (int)STATUS.Temp[i]);
      Serial.print(", ");
    }
    Serial.print("      PWM_A: ");
    Serial.print(STATUS.PWM_Ambient);
    Serial.print(",     PWM_Focal: ");
    Serial.println(STATUS.PWM_Focal);

}
