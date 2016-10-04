#include <Wire.h>
//I2C LCD OPTION
#include <LiquidCrystal_I2C.h>
#define I2C_ADDR 0x27 // Define I2C Address for LCD backpack (Connect SDA and SCL to D4/D5)
LiquidCrystal_I2C lcd(I2C_ADDR, 20, 4); //(I2C address, Columns, Rows)
//--
//  4-BIT WIRED LCD OPTION --
// * LCD RS pin to digital pin 12
// * LCD Enable pin to digital pin 11
// * LCD D4 pin to digital pin 5
// * LCD D5 pin to digital pin 4
// * LCD D6 pin to digital pin 3
// * LCD D7 pin to digital pin 2
// * LCD R/W pin to ground
// * LCD VSS pin to ground
// * LCD VCC pin to 5V
// * 10K resistor:
// * ends to +5V and ground
// * wiper to LCD VO pin (pin 3)
//include <LiquidCrystal.h>
//
//LiquidCrystal lcd(12, 11, 6, 5, 4, 3);// initialize the library with the numbers of the interface pins
//(NOTE: original pin setup was (12,11,5,4,3,2) assignments were switched to free D2 for inturrupt services to use with flow calcs)

//This can be remedied and better enhanced by reducing the connections to 3 pins on the nano by using a 595 shift register and associated custom LCD library
//See:(http://www.instructables.com/id/Hookup-a-16-pin-HD44780-LCD-to-an-Arduino-in-6-sec)
//---

//-=-=-=-=-=-=-=-=-= PIN DEFINES -=--=-=-=-=-=-=
// -------------> PINS MUST BE SELECTED TO COMPLIMENT THE LCD CONNECT TYPE CHOSEN <---------------------
// -----> I2C Compatible Pinouts <-------
#define WORKLID 6 //Work area door switch. (Connect one side of switch to this digital pin and the other side to gnd (high = door open - low = door closed))
#define ELECLID 3 //Electronics area door switch (Connect one side of switch to this digital pin and the other side to gnd (high = door open - low = door closed))
#define TUBELID 4 //Laser bay door switch (Connect one side of switch to this digital pin and the other side to gnd (high = door open - low = door closed))
#define INTERLOCK 5 //pin that enables or disables laser (used to make or break enable pins on LPSU job was originally done by the laser enable switch)
#define FLOWSENSORPIN 2 //Connect flow sensor signal output to this digital Pin D2 is also INT0 needed for flow calcs
#define ALARMPIN 9 //Connect + side of Alarm buzzer to this Digital pin ( - side to gnd )
//
// ----------> 4 bit LCD Compatible Pinouts <-----------------
//#define WORKLID 10 //Work area door switch. (Connect one side of switch to this digital pin and the other side to gnd (high = door open - low = door closed))
//#define ELECLID 7 //Electronics area door switch (Connect one side of switch to this digital pin and the other side to gnd (high = door open - low = door closed))
//#define TUBELID 8 //Laser bay door switch (Connect one side of switch to this digital pin and the other side to gnd (high = door open - low = door closed))
//#define INTERLOCK 9 //pin that enables or disables laser (used to make or break enable pins on LPSU job was originally done by the laser enable switch)
//#define FLOWSENSORPIN 2 //Connect flow sensor signal output to this digital Pin (NOTE: this pin will need to be switched to 2 if you wish to use inturrupts for flow calcs)
//#define ALARMPIN 13 //Connect + side of Alarm buzzer to this Digital pin ( - side to gnd )

//-=-=-=-=-=-=-=-=-=-=-=-=-==-=--=-=-=-=-
int TempSensorPin = A0;
//the analog pin the TMP36's Vout (sense) pin is connected to
//the resolution is 10 mV / degree centigrade with a
//500 mV offset to allow for negative temperatures
//-=-=-=-=-=-=-=-=-=-=-=-=-==-=--=-=-=-=-
long previousMillis = 0; // Just a standard Millis clock for comparing against
long interval = 1000; //defined millis interval (1 sec) generic millis unit for comparison

//=-=-=-=- Setup alarm flags -=-=-=-=-=-=
int workLidState = 0; //Work area door 0 = closed, 1 = open
int elecLidState = 0; //LPSU area door 0 = closed, 1 = open
int tubeLidState = 0; //Laser tube area door 0 = closed, 1 = open
int minFlow = 0; //Water flow under minimum threshold 0 = within threshold, 1 = alarm!
int maxTemp = 0; //Coolant temp exceeds max threshold 0 = within threshold, 1 = alarm!
int maxTempVal = 70; //Maximum temperature in degrees F allowable.
int minFlowRateL = 10; //Minimum flow rate/hr.
//-=-=-=-=-=-=-=-=-=-= Flow sensor setup -=-=-=-=-=-=-
volatile int FlowPulseDet; //measuring the rising edges of the signal from the flow meter
int CalcFlow;
//-=-=-=-=-=-=-=-=- SETUP =-=-=-=-==-=--=-=-=-=-
void setup()
{
  attachInterrupt(0, rpm, RISING); //each time the flowmeter pulses D2 (INT0) rpm() will be called
  lcd.begin (20, 4);

  pinMode(INTERLOCK, OUTPUT);
  digitalWrite(INTERLOCK, LOW);
  pinMode(FLOWSENSORPIN, INPUT);
  digitalWrite(FLOWSENSORPIN, HIGH);
  pinMode(ALARMPIN, OUTPUT);
  digitalWrite(ALARMPIN, LOW);
  pinMode(WORKLID, INPUT);
  digitalWrite(WORKLID, HIGH);
  pinMode(ELECLID, INPUT);
  digitalWrite(ELECLID, HIGH);
  pinMode(TUBELID, INPUT);
  digitalWrite(TUBELID, HIGH);
  lcd.clear();
}
//-=-=-=-=-=-=-=-=-=-=-=-=-==-=--=-=-=-=-

void loop() {
  doorUpdate();
  handleCheckInterlocks();
}
void handleCheckInterlocks() //Run through the interlock flags and check for alarms
{
  if (workLidState == 1)
  {
    digitalWrite(INTERLOCK, LOW); //System shuts laser down on alarm (note pin state based on application)
    //Display shows work area door open error
  }
  if (elecLidState == 1)
  {
    digitalWrite(INTERLOCK, LOW); //System shuts laser down on alarm (note pin state based on application)
    //Display shows work area door open error
  }
  if (tubeLidState == 1)
  {
    digitalWrite(INTERLOCK, LOW); //System shuts laser down on alarm (note pin state based on application)
    //Display shows work area door open error
  }
  if (minFlow == 1)
  {
    digitalWrite(INTERLOCK, LOW); //System shuts laser down on alarm (note pin state based on application)
    //Display shows isufficiant coolant flow
  }
  if (maxTemp == 1)
  {
    digitalWrite(INTERLOCK, LOW); //System shuts laser down on alarm (note pin state based on application)
    //Display shows excessive coolant temperature
    lcd.setCursor(0, 0);
    lcd.print(" TEMP WARNING!!!!");
    lcd.setCursor(2, 1);
    lcd.print("Temp > 70");
    lcd.print(" degrees F");
    lcd.setCursor(2, 2);
    lcd.print("IMMEDIATE ACTION");
    lcd.setCursor(6, 3);
    lcd.print("REQUIRED");
  }  else  {
    digitalWrite(INTERLOCK, HIGH);//No alarms, Laser enabled (note pin state based on application)
    //include active functions below to
  }
}
void watchFlow()//Flow Watchdog we will have to move the flow sensor pin from 6 to either D2 or D3 to beable to use inturrupts pending display reconfig.
{
  unsigned long currentMillis = millis();
  FlowPulseDet = 0;      //Set FlowPulseDet to 0 ready for calculations
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;// save the last time we collected counts
    sei();            //Enables interrupts and begins collecting pulses
  } else {
    cli();            //Disable interrupts shuts down interrupt disabling pulse collection
  }
  CalcFlow = (FlowPulseDet * 60 / 7.5); //(Pulse frequency x 60) / 7.5Q, = flow rate in L/hour
  if (CalcFlow < minFlowRateL)
  {
    minFlow = 1;
  } else {
    minFlow = 0;
  }
}

void rpm ()     //This is the function that the interupt calls
{
  FlowPulseDet++;  //This function measures the rising and falling edge of the hall effect sensors signal
}
//-=-=-=-=-=-=- TEMP SENSOR -=-=-=-=-=
void TempSensor()  // Temp sensor loop
{
  int reading = analogRead(TempSensorPin);
  float voltage = reading * 5.0; // converting that reading to voltage, for any 3.3v arduino MCU's use 3.3
  voltage /= 1024.0;
  float temperatureC = (voltage - 0.5) * 100 ;
  //converting from 10 mv per degree wit 500 mV offset
  //to degrees ((voltage - 500mV) times 100)
  lcd.setCursor(2, 2);
  float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
  if (temperatureF > maxTempVal) //Set this to the upper temperature limit . for the K40 it's been suggested to never exceed 70F or 21C
  {
    maxTemp = 1; //set maxTemp alarm flag
  }  else  {
    maxTemp = 0 ; //if under maxTempVal keep the alarm flag off.
    {
      lcd.setCursor(2, 3);
      lcd.print(temperatureF);
      lcd.print(" degrees F");
      digitalWrite(ALARMPIN, LOW);
    }
    delay(50);
  }
}
//-=-=-=-=-=-=-=-=-=-=-=-=-==-=--=-=-=-=-
void doorUpdate() {

  if (digitalRead(WORKLID) == HIGH) {
    workLidState = 1;
  } else {
    workLidState = 0;
  }
  if (digitalRead(ELECLID) == HIGH) {
    elecLidState = 1;
  } else {
    elecLidState = 0;
  }
  if (digitalRead(TUBELID) == HIGH) {
    tubeLidState = 1;
  } else {
    tubeLidState = 0;
  }
}
