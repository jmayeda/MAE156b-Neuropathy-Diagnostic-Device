#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int TEMP_START_COL = 6;
const int TEMP_START_ROW = 2;
const int TIME_START_COL = 6;
const int TIME_START_ROW = 3;
const int STATUS_START_COL = 8;
const int STATUS_START_ROW = 1;
const int LCD_MAX_COL = 20;
const int LCD_MAX_ROW = 3;

const int GREEN_LED_PIN = 8;
const int RED_LED_PIN = 7;
const int RESET_BUTTON_PIN = 2;
const int STOP_BUTTON_PIN = 3;

volatile int resetState = 0;
volatile int stopState = 0;

LiquidCrystal_I2C lcd(0x27,20,4);

void setup()
{
    // setup the pin modes
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RESET_BUTTON_PIN, INPUT);
    pinMode(STOP_BUTTON_PIN, INPUT);

    digitalWrite(RED_LED_PIN, HIGH);
    setupLCD();
    char tempMessage[ ] = "duck";
    updateStatusLCD(tempMessage);
    delay(1000);
    updateLCD(99, 59);

    attachInterrupt(digitalPinToInterrupt(RESET_BUTTON_PIN), RESET_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(STOP_BUTTON_PIN), STOP_ISR, RISING);

    digitalWrite(RED_LED_PIN, LOW);
}

void loop()
{
    // stop the test
    while (stopState == HIGH)
    {
        if (resetState == HIGH)
        {
            break;  // if we hit the reset button, break the stop conditions
        }
        digitalWrite(RED_LED_PIN, HIGH);
        delay(250);
        digitalWrite(RED_LED_PIN, LOW);
        delay(250);

    }
}

void setupLCD()
{
    lcd.init();
    lcd.backlight();
    lcd.setCursor(5,0);
    lcd.print("WinSanTor");
    lcd.setCursor(0,1);
    lcd.print("Status: ");
    lcd.setCursor(0,2);
    lcd.print("Temp: ");
    lcd.print("100");
    lcd.setCursor(0,3);
    lcd.print("Time: ");
    lcd.print("60");
}

void updateLCD(int temp, int times) {
    // set cursor to print the temperature
    lcd.setCursor(TEMP_START_COL, TEMP_START_ROW);

    // clear the previous stuff
    for (int i=TEMP_START_COL; i<LCD_MAX_COL; i++) {
        lcd.print(" ");
    }
    // print the latest temperature
    lcd.setCursor(TEMP_START_COL, TEMP_START_ROW);
    lcd.print(String(temp));

    // set cursor to print the reaction time
    lcd.setCursor(TIME_START_COL, TIME_START_ROW);
    for (int i=TIME_START_COL; i<LCD_MAX_COL; i++) {
        lcd.print(" ");
    }
    // print the latest temperature
    lcd.setCursor(TIME_START_COL, TIME_START_ROW);
    lcd.print(String(times));
}

void updateStatusLCD(char message) {
    lcd.setCursor(STATUS_START_COL, STATUS_START_ROW);
    // clear the output
    for (int i=STATUS_START_COL; i<LCD_MAX_COL; i++) {
        lcd.print(" ");
    }
    lcd.setCursor(STATUS_START_COL, STATUS_START_ROW);
    lcd.print(message);
}

void RESET_ISR()
{
    // reset the test
    resetState = digitalRead(RESET_BUTTON_PIN);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
}

void STOP_ISR()
{
    // stop the test
    stopState = digitalRead(STOP_BUTTON_PIN);
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);
}
