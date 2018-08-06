
/*------------------------------------------------------------------
  Besturing voor een klimaatkast met 2 compartimenten en PID regelaars
  voor de compartimenten en de koelkast waarin de compartimenten zich
  bevinden.

  Sensor1 meet de temperatuur en rh van het eerste compartiment
  Sensor2 meet de temperatuur en rh van het tweede compartiment
  Sensor3 meet de temperatuur en rh van de koelkast

  Todo;

  1 errorState toevoegen

  -------------------------------------------------------------------
*/

#include <DHT.h>
#include <Wire.h> // Include Standard Wire Library
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <EEPROM.h>

volatile int PinAEncoder = 3; // Used for generating interrupts using CLK signal
volatile int PinBEncoder = 4;  // Used for reading DT signal
bool buttonPin = A3;  // Used for the push button switch
double lastCount = 0; // Keep track of last rotary value
volatile long encoderPosition; // Updated by the ISR (Interrupt Service Routine)
double oldEncoderPosition = 0; //status encoderPosition voordat de functie begon
bool buttonState = 0; //status van de drukknop
unsigned long lastTimeButtonPressed = 0; //tijdstip op het moment dat de knop is ingedrukt

#define DHTPIN 8    // what digital pin we're connected to
// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
DHT dht(DHTPIN, DHTTYPE);

unsigned long tempReadDelay = 2000; //Interval waarmee de dhtsensor wordt gelezen.
double temperatureSensor1, humiditySensor1, temperatureSensor2, humiditySensor2, temperatureSensor3, humiditySensor3;
String shortTemperatureSensor1, shortHumiditySensor1, shortTemperatureSensor2, shortHumiditySensor2, shortTemperatureSensor3, shortHumiditySensor3;
unsigned long lastUpdateSensor = 0; //tijdstip waarop de sensor is gelezen

LiquidCrystal_I2C lcd(0x3F, 16, 2); // Set the LCD address to 0x3F for a 16 chars and 2 line display
String line1 = "ArcheoPro        "; //regel 1 van het display
String line2 = "PID regeling"; //regel 2 van het display

int menuState = 0; //gekozen menu

double setpointT1, setpointT2, setpointT3;
double oldSetpointT1, oldSetpointT2, oldSetpointT3;
String shortSetpointT1, shortSetpointT2, shortSetpointT3;

#define relayPin1 12//uitgang van myPID1
double output1; //berekende uitgang van myPID1

PID myPID1(&temperatureSensor1, &output1, &setpointT1, 2, 5, 1, DIRECT);
int windowSize1 = 5000;
unsigned long windowStartTime1;

// ------------------------------------------------------------------
// INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT
// ------------------------------------------------------------------
void isr ()  {
  static unsigned long lastInterruptTime = 0;
  //unsigned long interruptTime = millis();
  //if (interruptTime - lastInterruptTime > 10) { // If interrupts come faster than 5ms, assume it's a bounce and ignore
  if (digitalRead(PinBEncoder) == LOW) {
    encoderPosition-- ; // Could be -5 or -10
  }
  else {
    encoderPosition++ ; // Could be +5 or +10
  }
  //lastInterruptTime = interruptTime; // Keep track of when we were here last (no more than every 5ms)
  //}
}// einde interrupt

//------------------------------------------------------------------
// SETUP        SETUP        SETUP        SETUP        SETUP
//------------------------------------------------------------------
void setup() {// put your setup code here, to run once:
  Serial.begin(9600);
  lcd.init(); // initialize the lcd
  dht.begin(); // initialize dht sensor
  pinMode(PinAEncoder, INPUT);// Rotary pulses are INPUTs
  pinMode(PinBEncoder, INPUT);// Rotary pulses are INPUTs
  pinMode(buttonPin, INPUT_PULLUP);// Switch is floating so use the in-built PULLUP so we don't need a resistor
  //attachInterrupt(digitalPinToInterrupt(PinAEncoder), isr, LOW); // Attach the routine to service the interrupts
  EEPROM.get(0, setpointT1); //lees het laatst opgeslagen setpointT1 in
  EEPROM.get(1, setpointT2); //lees het laatst opgeslagen setpointT2 in
  EEPROM.get(2, setpointT2); //lees het laatst opgeslagen setpointT3 in
  setpointT1 = 25;
  // Ready to go!
  Serial.println("Start ");
  Serial.println (setpointT1);
  Serial.println(setpointT2);
  Serial.println(setpointT3);

  pinMode(relayPin1, OUTPUT);
  windowStartTime1 = millis();
  myPID1.SetOutputLimits(0, windowSize1); //limiteer de uitgang van myPID1 tussen 0 en WindowSize1
  myPID1.SetMode(AUTOMATIC); //zet myPID1 aan


} //einde setup

//------------------------------------------------------------------
// LOOP       LOOP       LOOP       LOOP       LOOP       LOOP
//------------------------------------------------------------------
void loop() { // put your main code here, to run repeatedly:
  // Ready to go!
  //Serial.println("Start loop");
  button();
  sensor1();
  sensor2();
  sensor3();
  menu();
  printLCD();

  myPID1.Compute();

  /************************************************
     turn the output pin on/off based on pid output
   ************************************************/
  unsigned long now = millis();
  if (now - windowStartTime1 > windowSize1)
  { //time to shift the Relay Window
    windowStartTime1 += windowSize1;
  }
  if (output1 > now - windowStartTime1) digitalWrite(relayPin1, HIGH);
  else digitalWrite(relayPin1, LOW);



} //einde loop

//void printLCD drukt 2 regels af op het scherm
void printLCD() {
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
} // einde printLCD

void menu() {
  if ((millis() - lastTimeButtonPressed) > 10000) {
    menuState = 1;
    if (oldSetpointT1 != setpointT1) { //schrijf setpointT1 naar EEPROM
      EEPROM.put(0, setpointT1);
      oldSetpointT1 = setpointT1;
      //Serial.println ("write eeprom 1");
    } //einde if
    if (oldSetpointT2 != setpointT2) { //schrijf setpointT2 naar EEPROM
      EEPROM.put(1, setpointT2);
      oldSetpointT2 = setpointT2;
      //Serial.println ("write eeprom 2");
    } //einde if
    if (oldSetpointT3 != setpointT3) { //schrijf setpointT3 naar EEPROM
      EEPROM.put(2, setpointT1);
      oldSetpointT3 = setpointT3;
      //Serial.println ("write eeprom 3");
    } //einde if
  } //einde if

  if (buttonState) { //actie als er op de button is gedrukt
    menuState++;
    buttonState == 0;
    oldSetpointT1 = setpointT1;
    oldSetpointT2 = setpointT2;
    oldSetpointT3 = setpointT3;
    oldEncoderPosition = encoderPosition;
  } //einde if

  switch (menuState) {
    case 0: //start scherm 2
      {
        line1 = "ArcheoPro 2.0   ";
        line2 = "call 030-2743123";
      } //einde case 0
      break;
    case 1: //toon sensor 1&2 waarden
      {
        shortTemperatureSensor1 = String(temperatureSensor1, 1);
        shortHumiditySensor1 = String(humiditySensor1, 1);
        shortTemperatureSensor2 = String(temperatureSensor2, 1);
        shortHumiditySensor2 = String(humiditySensor2, 1);
        line1 = "T1 " + shortTemperatureSensor1 + ((char) 223) + " " + shortHumiditySensor1 + "%          ";
        line2 = "T2 " + shortTemperatureSensor2 + ((char) 223) + " " + shortHumiditySensor2 + "%          ";
      } //einde case 1
      break;
    case 2: //toon sensor3 waarden
      {
        shortTemperatureSensor3 = String(temperatureSensor3, 1);
        shortHumiditySensor3 = String(humiditySensor3, 1);
        line1 = "T3 " + shortTemperatureSensor3 + ((char) 223) + " " + shortHumiditySensor3 + "%          ";
        line2 = "                ";
      } //einde case 2
      break;

    case 3: { //toon setpoint sensor1
        line1 = "Setpoint T1     ";
        setpointT1 = oldSetpointT1 + (encoderPosition - oldEncoderPosition) / 10;
        shortSetpointT1 = String(setpointT1, 1);
        line2 = shortSetpointT1 + ((char) 223) + "      ";
      } //einde case 3
      break;

    case 4: { //toon setpoint sensor2
        line1 = "Setpoint T2     ";
        setpointT2 = oldSetpointT2 + (encoderPosition - oldEncoderPosition) / 10;
        shortSetpointT2 = String(setpointT2, 1);
        line2 = shortSetpointT2 + ((char) 223) + "      ";
      } //einde case 4
      break;

    case 5: { //toon setpoint sensor3
        line1 = "Setpoint T3     ";
        setpointT3 = oldSetpointT3 + (encoderPosition - oldEncoderPosition) / 10;
        shortSetpointT3 = String(setpointT3, 1);
        line2 = shortSetpointT3 + ((char) 223) + "      ";
      } //einde case 5
      break;

    case 6: { //toon autotune myPID1
        line1 = "Tune PID1 RTFM";
        line2 = "Draai = start";
        if ((encoderPosition - oldEncoderPosition) > 5) {
          line1 = "Autotune PID1";
          line2 = "is gestart";
          //start autotune
        }
      } //einde case 6


  } //einde switch

} //einde menu

void button() { // Is someone pressing the rotary switch?
  if ((!digitalRead(buttonPin))) {
    while (!digitalRead(buttonPin)) {
      delay(100);
      lastTimeButtonPressed = millis();
    }//einde while
    buttonState = 1;
  }//einde if
  else {
    buttonState = 0;
  }//einde else
} //einde button

void sensor1() {
  if ((millis() - lastUpdateSensor) > tempReadDelay) {
    temperatureSensor1 = dht.readTemperature();
    humiditySensor1 = dht.readHumidity();
    lastUpdateSensor = millis();
  }

} //einde sensor1

void sensor2() {

} //einde sensor2

void sensor3() {

}//einde sensor3

/*--------------------------------------------------------------------
   Bronnen        Bronnen        Bronnen        Bronnen        Bronnen
   -------------------------------------------------------------------

   https://raw.githubusercontent.com/RalphBacon/RotaryEncoderUpdate/master/RotaryEncoderInterrrupts.ino
   http://ryandowning.net/AutoPID/#pwm-relay-control
   https://playground.arduino.cc/Code/PIDLibraryRelayOutputExample
   http://brettbeauregard.com/blog/
   https://github.com/adafruit/DHT-sensor-library/blob/master/examples/DHTtester/DHTtester.ino

*/

