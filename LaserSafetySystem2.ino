#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define I2C_ADDR 0x27 // Define I2C Address for LCD backpack
LiquidCrystal_I2C lcd(I2C_ADDR, 20, 4); //(I2C address, Columns, Rows)

enum {
  STATE_WORK_AREA_DOOR_OPEN,
  STATE_LPSU_AREA_DOOR_OPEN,
  STATE_LASER_BAY_DOOR_OPEN,
  STATE_NO_COOLANT_FLOW,
  STATE_OVER_TEMP,
  STATE_CHECK_INTERLOCKS,
  // STATE_BURN_BABY_BURN,
} InterlockState;



//-=-=-=-=-=-=-=-=-= PIN DEFINES -=--=-=-=-=-=-=
// =-=-=-= V2.0 pins =-=-=-=-
#define WORKLID 3 //Work area door switch. (Connect one side of switch to this digital pin and the other side to gnd (high = door open - low = door closed))
#define ELECLID 1 //Electronics area door switch (Connect one side of switch to this digital pin and the other side to gnd (high = door open - low = door closed))
#define TUBELID 1 //Laser bay door switch (Connect one side of switch to this digital pin and the other side to gnd (high = door open - low = door closed))
#define INTERLOCK 5 //pin that enables or disables laser (used to make or break enable pins on LPSU job was originally done by the laser enable switch)
#define FLOWSENSORPIN 2 //Connect flow sensor signal output to this digital Pin
#define ALARMPIN 4 //Connect + side of Alarm buzzer to this Digital pin ( - side to gnd )

//=-=-=-=-=-=-=--=-=-=-=--=-=--=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-==-=--=-=-=-=-
int TempSensorPin = A0;
//the analog pin the TMP36's Vout (sense) pin is connected to
//the resolution is 10 mV / degree centigrade with a
//500 mV offset to allow for negative temperatures
//-=-=-=-=-=-=-=-=-=-=-=-=-==-=--=-=-=-=-


int state;
long previousMillis = 0;
long interval = 1000; //defined millis interval (1 sec)

//=-=-=-=- Setup alarm flags -=-=-=-=-=-=
//=-=-=-=- V2.0 flags =-=-=-
int alarm = 1;
int workLidState = 0; //Work area door 0 = closed, 1 = open
int elecLidState = 0; //LPSU area door 0 = closed, 1 = open
int tubeLidState = 0; //Laser tube area door 0 = closed, 1 = open
int minFlow = 0; //Water flow under minimum threshold 0 = within threshold, 1 = alarm!
int maxTemp = 0; //Coolant temp exceeds max threshold 0 = within threshold, 1 = alarm!
int maxTempVal = 70; //Maximum temperature in degrees F allowable.




//-=-=-=-=-=-=-=-=-=-= Flow sensor setup -=-=-=-=-=-=-
volatile int FlowPulseDet; //measuring the rising edges of the signal from the flow meter
int CalcFlow;

void rpm ()     //This is the function that the interupt calls
{
  FlowPulseDet++;  //This function measures the rising and falling edge of the hall effect sensors signal
}


//-=-=-=-=-=-=-=-=- SETUP =-=-=-=-==-=--=-=-=-=-
void setup()
{
  state = STATE_CHECK_INTERLOCKS;
  lcd.begin (20, 4);
  // Setup the pins
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

  //-=-=-=-=-= millis timer -=-=-=-=
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    // save the last time we came around
    previousMillis = currentMillis;
  }
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-


  switch (state) {
    case STATE_CHECK_INTERLOCKS:        state = handleCheckInterlocks();       break;
    case STATE_WORK_AREA_DOOR_OPEN:     state = handleWorkAreaDoorOpen();      break;
    case STATE_LPSU_AREA_DOOR_OPEN:     state = handleLPSUAreaDoorOpen();      break;
    case STATE_LASER_BAY_DOOR_OPEN:     state = handleLaserBayDoorOpen();      break;
    case STATE_NO_COOLANT_FLOW:         state = handleNoCoolantFlow();         break;
    case STATE_OVER_TEMP:               state = handleOverTemp();              break;
      // case STATE_BURN_BABY_BURN:          state = handleBurnBabyBurn();          break;

  }
}

int  handleCheckInterlocks() //Run through the interlock flags and check for alarms
{
  doorUpdate();

  if (workLidState == alarm)
  {
    return STATE_WORK_AREA_DOOR_OPEN;
  }
  if (elecLidState == alarm)
  {
    return STATE_LPSU_AREA_DOOR_OPEN;
  }
  if (tubeLidState == alarm)
  {
    return STATE_LASER_BAY_DOOR_OPEN;
  }
  if (minFlow == alarm)
  {
    return STATE_NO_COOLANT_FLOW;
  }
  if (maxTemp == alarm)
  {
    return STATE_OVER_TEMP;
  }
  else
  {

    TempSensor();
    watchFlow();
    digitalWrite(INTERLOCK, HIGH);

    //return STATE_BURN_BABY_BURN;
  }
}

int  handleWorkAreaDoorOpen()
{
  digitalWrite(INTERLOCK, LOW);
  doorUpdate();
  return STATE_CHECK_INTERLOCKS;
}

int handleLPSUAreaDoorOpen()
{
  digitalWrite(INTERLOCK, LOW);
  doorUpdate();
   return STATE_CHECK_INTERLOCKS;
}

int handleLaserBayDoorOpen()
{
  digitalWrite(INTERLOCK, LOW);
  doorUpdate();
   return STATE_CHECK_INTERLOCKS;
}

int handleNoCoolantFlow()
{
  digitalWrite(INTERLOCK, LOW);
   return STATE_CHECK_INTERLOCKS;
}

int handleOverTemp()
{
  digitalWrite(INTERLOCK, LOW);
  //display over temp warning on LCD.
  lcd.setCursor(0, 0);
  lcd.print(" TEMP WARNING!!!!");
  lcd.setCursor(2, 1);
  lcd.print("Temp > 70");
  lcd.print(" degrees F");
  lcd.setCursor(2, 2);
  lcd.print("IMMEDIATE ACTION");
  lcd.setCursor(6, 3);
  lcd.print("REQUIRED");
  delay(700);
  lcd.clear();
   return STATE_CHECK_INTERLOCKS;
}

//int handleBurnBabyBurn()
//{
//
//  doorUpdate()
//  TempSensor();
//  watchFlow();
//
//}

void watchFlow()//Flow Watchdog
{
  FlowPulseDet = 0;      //Set FlowPulseDet to 0 ready for calculations
  sei();            //Enables interrupts
  delay (1000);      //Wait 1 second
  cli();            //Disable interrupts
  CalcFlow = (FlowPulseDet * 60 / 7.5); //(Pulse frequency x 60) / 7.5Q, = flow rate in L/hour
}
//-=-=-=-=-=-=-=-=-=-=-=-=-==-=--=-=-=-=-
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
     
  }
  else
  {
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
