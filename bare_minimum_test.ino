/*******************************************************************************
 * bare_minimum_test.ino
 *
 * @date: 05/02/18
 *
 ******************************************************************************/

/* Thermistor Pins */
#define THERM1_PIN A1
#define THERM2_PIN A2
#define THERM3_PIN A3
#define THERM4_PIN A4
const int NUM_THERM = 4;

/* Thermistor Constants */
const int BETA = 3380;
const float ROOM_TEMP_K = 298.0;

/* Ultrasonic Sensor */
const int TRIG_PIN = 5;  // Ultrasonic sensor
const int ECHO_PIN = 3;  // Ultrasonic sensor
const int GREEN_LED_PIN = 7;
const int RED_LED_PIN = 6;
const int BUTTON_PIN = 2;
const int SETUP_TIME_S = 5;  // amount of time to calibrate the ultrasonic sensor
const float DIST_THRESHOLD = 1.3; // trigger the end of the test

/* Focal PWM Value */
const int FOCAL_PWM_PIN = 9;
const float FOCAL_PWM_VAL = 0.25 * 255;

/* Ambient PWM Value */
const int AMBIENT_PWM_PIN = 10;
const int KI_AMBIENT = 1;
const int KP_AMBIENT = 15;

/* Other constants */
const int SAFETY_CUTOFF_TEMP  = 49;  // safety temperature max
const int GLASS_TEMP_SETPOINT = 30; // setpoint temperature at glass

/**************************** Global Variables ********************************/
float thermistorPin[4] = {THERM1_PIN, THERM2_PIN, THERM3_PIN, THERM4_PIN};
float thermistorTemp[4] = {0.0, 0.0, 0.0, 0.0};
float distance;
float avgDistMM;
volatile int buttonState = 0;
int resetFlag = 0;
long timeEnd;
float tempEnd;
int ambientPWM;

/*************************** Function Declaration *****************************/
// sensors
float pollUltraSensor(int trigPin, int echoPin);
float calibrateUltraSensor(int trigPin, int echoPin, int setupTimeS);
float readThermistor(int pin);

// heaters
void writeToFocalHeater(int PWM);
void writeToAmbientHeater(int PWM);
int Controller_Ambient(float ref_Temp_Ambient);

// interrupt routines
void reset_ISR();

/********************************* Setup **************************************/
void setup() {
  // thermistors
  pinMode(THERM1_PIN, INPUT);
  pinMode(THERM2_PIN, INPUT);
  pinMode(THERM3_PIN, INPUT);
  pinMode(THERM4_PIN, INPUT);

  // focal heating
  pinMode(FOCAL_PWM_PIN, OUTPUT);

  // ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  // attach Interrupt on the button pin to wait for changing state
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), reset_ISR, CHANGE);

  // calibrate the motion sensor once
  avgDistMM = calibrateUltraSensor(TRIG_PIN, ECHO_PIN, SETUP_TIME_S);

  // keep the focal heater off until the test
  writeToFocalHeater(0);
  writeToAmbientHeater(0);

  // start the serial monitor for debugging
  Serial.begin(9600);

  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, LOW);

  Serial.println("Running ambient heating controller"); // for debugging
  /*****************************************************************************
   * PRE-TEST: RUN AMBIENT CONTROLLER TO WARM GLASS
   *
   * Before any tests can be run, the glass temperature must be at >= 30 deg C.
   * (1): read the current temperature from the thermistor.
   * (2): run the ambient controller to calculate a PWM.
   * (3): write the pwm value to the ambient heater MOSFET.
   * (4): delay for 250ms (4Hz)
   ****************************************************************************/
  while(thermistorTemp[2] < GLASS_TEMP_SETPOINT)
  {
    thermistorTemp[2] = readThermistor(thermistorPin[2]);
    ambientPWM = Controller_Ambient(GLASS_TEMP_SETPOINT);
    writeToAmbientHeater(ambientPWM);
    Serial.print("Glass temp: ");
    Serial.println(thermistorTemp[2]);
    delay(250);
  }
  Serial.println("Ideal glass temperature reached.."); // for debugging

  digitalWrite(RED_LED_PIN, LOW); // turn the RED LED off
}

/********************************** loop **************************************/
void loop() {
  /*****************************************************************************
   * A. INTERRUPT RESET CASE
   *
   * When the reset button is pressed, the following routines run:
   * (1): calibrateUltraSensor(): distance to new patient hand is calibrated.
   * (2): pollUltraSensor(): Read distance once before entering main running
   *                         case.
   ****************************************************************************/
  if (buttonState == HIGH)
  {
    //reset the test if the reset button is pressed
    avgDistMM = calibrateUltraSensor(TRIG_PIN, ECHO_PIN, SETUP_TIME_S);
    // update distance before testing condition
    distance = pollUltraSensor(TRIG_PIN, ECHO_PIN);

    // if resetFlag == 1, the test has been successfull reset
    resetFlag = 1;
  }
  /*****************************************************************************
   * B. CHECK FOR AMBIENT HEATING
   *
   * No tests can be run until we reach the 30 deg C glass temperature. Just
   * run the ambient heating controller in a loop until the 30 deg C is reached.
   * (1): run ambient controller
   * (2): write to ambient heating controller
   ****************************************************************************/
  if(thermistorTemp[2] < GLASS_TEMP_SETPOINT) {
    thermistorTemp[2] = readThermistor(thermistorPin[2]);
    ambientPWM = Controller_Ambient(GLASS_TEMP_SETPOINT);
    writeToAmbientHeater(ambientPWM);
  }
  /*****************************************************************************
   * C. MAIN RUNNING CASE
   *
   * The main running case checks for the following conditions:
   * (1): Motion sensor: distance read by ultrasonic is less than some threshold
   *                     value.
   * (2): resetFlag == 1: This indicates that the test has been successfully
   *                      initialized/reset.
   * (3): focalTemp < safety_temp: This indicates that the focal temperature at
   *                               the glass is at a safe temperature (<50degC)
   ****************************************************************************/
  if ((distance < DIST_THRESHOLD * avgDistMM  || resetFlag == 1) \
    && thermistorTemp[0] < SAFETY_CUTOFF_TEMP)
  {
    // 1. Reset the reset flag
    resetFlag = 0;

    // 2. Indicate the test is running
    digitalWrite(GREEN_LED_PIN, HIGH);
    Serial.print(millis());
    Serial.print("  ");

    // 3. Read the ultrasonic sensor for movement
    distance = pollUltraSensor(TRIG_PIN, ECHO_PIN);

    // 4. Read the thermistors one at a time
    for (int i = 0; i < NUM_THERM; i++)
    {
      thermistorTemp[i] = readThermistor(thermistorPin[i]);
      Serial.print(thermistorTemp[i]);
      Serial.print(", ");
    }
    Serial.println("; ");

    // 5. run the focal heater
    writeToFocalHeater(FOCAL_PWM_VAL);
  }
  /*****************************************************************************
   * D. STOP CONDITIONS REACHED or WAITING FOR RESET
   *
   * This is the intermediate state between the reset routine and the main
   * running case. In this state the focal heater will be turned off, and the
   * test will be waiting for a reset.
   ****************************************************************************/
  else
  { // test stop condition has been triggered
    writeToFocalHeater(0); // turn the focal heater off
    Serial.print("Time end: ");
    Serial.print(timeEnd);
    Serial.print("  ");
    Serial.print("Temperature end: ");
    Serial.println(tempEnd);

    while (!buttonState) // while button state is low
    {
      // will run in this loop until the button is pressed again
      digitalWrite(GREEN_LED_PIN, LOW);
      digitalWrite(RED_LED_PIN, HIGH);
    }
  }
  /****************************************************************************/
}

/**
 * void reset_ISR()
 *
 * @param: none
 * @return: none
 */
void reset_ISR() {
  // reset the test
  buttonState = digitalRead(BUTTON_PIN);
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, LOW);

  tempEnd = thermistorTemp[0];
  timeEnd = millis();
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
  duration_mcs = pulseIn(echoPin, HIGH);

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

void writeToFocalHeater(int PWM) {
  analogWrite(FOCAL_PWM_PIN, PWM);
}

void writeToAmbientHeater(int PWM) {
  analogWrite(AMBIENT_PWM_PIN, PWM);
}

/*
 * int Controller_Ambient(float ref_Temp_Ambient)
 *
 * Implement controller algorithim for the ambient heater to maintain
 * the housing temperature at ref_Temp_Ambient.
 */
int Controller_Ambient(float ref_Temp_Ambient) {
  // static variables initialize once, the first time the function is called
  static float cumalitiveError = 0;
  static float I_term = 0;
  float P_term = 0;

  // update current error
  float currError = ref_Temp_Ambient - thermistorTemp[2];

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
  ambientPWM = control_output;

  return ambientPWM;
}
