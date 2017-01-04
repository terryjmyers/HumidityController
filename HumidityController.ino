//Low pass butterworth filter order=1 alpha1=0.01 
//95% of value in ~50 steps, cross over final value at ~120steps
class  FilterBuLp01
{
  public:
    FilterBuLp01()
    {
      v[0]=0.0;
    }
  private:
    float v[2];
  public:
    float step(float x) //class II 
    {
      v[0] = v[1];
      v[1] = (3.046874709125380054e-2 * x)
         + (0.93906250581749239892 * v[0]);
      return 
         (v[0] + v[1]);
    }
};



#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <EEPROM.h>

#define DOHumOnPIN 4     // SSR Humidity controller Discrete Output
#define DHTPIN 5     // what digital pin we're connected to
#define DIPBPIN 6     // Pushbutton from Rotary encoder

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3
volatile bool PinAONS;
volatile bool PinBONS;
volatile bool PinAONSCMD;
volatile bool PinBONSCMD;
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27,16,4);  // set the LCD address to 0x20 for a 16 chars and 2 line display
  volatile float HumiditySP;
  float HumiditySPREM; //a remember register to write to EEPROM whenever it changes
  
  unsigned int HumidityOnOff; //0= humdity control off, 1 = humidity control on
  
  unsigned long DHTReadTimerACC; //Timer accumulator.
  unsigned long DHTReadTimerPRE = 5000; //Timer preset in ms, how often to read DHT

  int State=5; //5=off, 20=main screen, 30=config screen (future), 40=help screen(future)
  int StateCMD; //CMD to go to a different state
  #define StateMainScreen 20
  #define StateConfigScreen 30
  #define StateHelpScreen 40
  unsigned long DisplayTurnOnMillis; //millis capture of when the display turned on to disable inputs for a time while the screen is turned on.
  unsigned long DisplayTurnOnMillisPRE = 500; //Timer preset of how long to disable inputs while turning the screen on
  
  unsigned long DisplayTurnOffMillis; //millis capture of while the screen is turned on, the last input.  Used to turn the screen on
  unsigned long DisplayTurnOffMillisPRE=20000; //Timer preset of how long to wait to turn the screen off after the last input is received

  int ConfigScreenPosition;
  int ConfigScreenPositionCMD;
  
  float Humidity;
  float HumidityDB = 1.0; //Humidity deadband value
  FilterBuLp01 HumidityLPF;
  bool TemperatureEngUnits; //0=F, 1=C
  float Temperature;
  FilterBuLp01 TemperatureLPF;
  float HeatIndex;
  unsigned long HeatIndexToggleTimer;
  int HeatIndexToggle;

  bool UserFeedbackONS; //A one shot bit that aggregates any user feedback
  bool DOHumOn;// Humidity SSR output
  bool DIPB;
  bool DIPBOnONS;
  bool DIPBOffONS;
  bool DIPBLongPressONS;
  bool DIPBLongPressONSREM;
  bool DIPBLongPress;
  bool DIPBQuickPressONS;
  int DIPBStep; //0=false, 10=waiting for long press, 20
  unsigned long DIPBOnMillis;
  unsigned long DIPBOffMillis;
  unsigned long DIPBOnLongPressMillisPRE=1000;

  int FailureCode=0;
  bool debug=1; //Set to 1 to enable Serial debugging
  
//Setup some tags to monitor program performance including the number of loops (ScanCounter), and the total time through each loop (scan time) in microseconds
unsigned long ScanCounter; //used for average
unsigned long MinScanTimeuS = 4294967295; //smallest recorded ScanTime in us.  Set to the max number so that when a smaller number is recorded during the program its updated
float AvgScanTimeuS; //ScanTime in us averaged over one sec
unsigned long MaxScanTimeuS; //largest ScanTime in us
unsigned long microsREM; //last micros() to remember


//Setup some precise pulse timers
//Set up boolean pulses that go true for one scan of the code.
bool TimerOneSecondPulse = 0;
bool TimerHalfSecondPulse = 0;
bool TimerTenthSecondPulse = 0;
bool TimerTenMSPulse = 0;
bool TimerOneMSPulse = 0;

//Create an array to store pulse timer data
const int PulseTimerLen = 5;
unsigned long PulseTimerPRE[PulseTimerLen]; //Create an array of timer preset values
unsigned long PulseTimerREM[PulseTimerLen]; //Create an array to store millis()

//Set array index values to reference the above timer elements
const int TimerOneSecondPulseIndex = 4;
const int TimerHalfSecondPulseIndex = 3;
const int TimerTenthSecondPulseIndex = 2;
const int TimerTenMSPulseIndex = 1;
const int TimerOneMSPulseIndex = 0;

void setup() {
  Serial.begin(115200);

  //Set up the PRE for the pulse timers.  Note that the closest prime number was choosen to prevent overlapping with other timers
  PulseTimerPRE[TimerOneSecondPulseIndex] = 997;
  PulseTimerPRE[TimerHalfSecondPulseIndex] = 499; //Set up the PRE for the timer.  Note that the closest prime number was choosen to prevent overlapping with other timers
  PulseTimerPRE[TimerTenthSecondPulseIndex] = 101; //Set up the PRE for the timer.  Note that the closest prime number was choosen to prevent overlapping with other timers
  PulseTimerPRE[TimerTenMSPulseIndex] = 11;  //Set up the PRE for the timer.
  PulseTimerPRE[TimerOneMSPulseIndex] = 1;  //Set up the PRE for the timer.

  
  dht.begin();
  
  lcd.init();   
  pinMode(DOHumOnPIN,OUTPUT);
  pinMode(DIPBPIN,INPUT_PULLUP);
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0,PinA,RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1,PinB,RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)

//Read the third bit.  If its a zero, this is the first time the program has run
  if (((EEPROM.read(0)>>2) & 1)==0) {
      Serial.println(F("First Run of Program"));
      //Set initial values
        HumidityOnOff = false; //byte0,bit0
        TemperatureEngUnits = false; //byte0,bit1
        HumiditySP=45.0; //byte 1-4
        HumidityDB=1.0; //bytes 5-9
        EEPROMWrite(); //Write them to EEPROM
  }
  EEPROMRead();
  
 //Saturate the LPF with initial readings
  Humidity = dht.readHumidity();
  Temperature = dht.readTemperature(true);
  for (int i=0;i<120;i++) {
    HumidityLPF.step(Humidity);
    TemperatureLPF.step(Temperature);
  }
   
}



//MAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOP
//MAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOP
void loop() {
 UpdateAllTimers();

  System(); //Stuff that doesn't really fit anywhere
  ReadDHT();
  ReadPB();
  RotaryEncoder();
  HumidityControl();
  StateEngine();
  
  UpdateScanCounter(); //Keep this at the bottom of the loop
}
//MAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOP
//MAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOP
//=============================================================================================
void System() {

  if (FailureCode>0) { HumidityOnOff=0; } //Turn off Humidity Control if there is a failure
  
      
}
//==============================================================================================
void ReadDHT() {

 if  (DHTReadTimerACC+DHTReadTimerPRE > millis()) { return;}
    DHTReadTimerACC = millis();
    // Reading temperature or humidity takes about 250 milliseconds!
   
  if (FailureCode ==1) { //Try to reinitialize on DHT failure
    FailureCode = 0;
    if (debug) {Serial.println(F("Trying to Re-initialize DHT module"));}
    dht.begin();
  } 

  
  Humidity = HumidityLPF.step(dht.readHumidity());
  Temperature = TemperatureLPF.step(dht.readTemperature(true));
    if (TemperatureEngUnits==1) { //Celcius
      Temperature = (Temperature-32)/1.8; //convert to C if configured
      HeatIndex = dht.computeHeatIndex(Temperature, Humidity,false); //calculate Heat Index in Celcius
    } else {
      HeatIndex = dht.computeHeatIndex(Temperature, Humidity); //calculate Heat Index in F
    }
  
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(Humidity) || isnan(Temperature)) {
    FailureCode = 1;//   
  }

  if (debug) {
    Serial.print("Humidity=");
    Serial.print(Humidity,1);
    Serial.print("%RH   ");
    Serial.print("Temp=");
    Serial.print(Temperature,1);
    if (TemperatureEngUnits==0) {
      Serial.print("F");
    } else {
      Serial.print("C");
    }
    Serial.println();
  }
  
}
//==============================================================================================
void ReadPB () {
  
    DIPB = not digitalRead(DIPBPIN); //Read the push button on the rotary encoder and turn humidity on/off
 //turn off one shots created below from the previous scan
    DIPBOnONS=0;
    DIPBOffONS=0; 
    DIPBLongPressONS=0;
    DIPBQuickPressONS=0;

// Create state engine for push button


  if (DIPBStep==0){
    //Push button is not pressed
      if (DIPB==true) { 
        DIPBOnMillis = millis(); //start a timer used in the next step to determine if its a long press
        DIPBOnONS=1; //Set a one shot used elsewhere in the PLC code
        DIPBStep=10;
       if (debug) { Serial.println(F("PB on"));}
      }
        DIPBLongPressONSREM=false;
        DIPBLongPress=false;
  }

  
  //-------------------
  if (DIPBStep==10){    
      if (DIPBOnMillis + DIPBOnLongPressMillisPRE < millis() ) {//long press detected
        DIPBLongPress=true; //Set true the entire time while the PB is down in a long press
        if (DIPBLongPressONSREM==false) { //Create a one shot
          DIPBLongPressONS=1;
          if (debug) {Serial.println(F("Long Press one shot"));}
          DIPBLongPressONSREM=true;
        }
      }
      
      if (DIPB == false) {
       if (debug) { Serial.println(F("PB off"));}
        DIPBOffONS=1;
        DIPBStep=0;
        if (DIPBLongPress==false) {
          DIPBQuickPressONS=true; //Set the quick press ONS
         if (debug) { Serial.println(F("Quick Press Detected"));}
        }
        
      } 
  }
  
}
//==============================================================================================
void RotaryEncoder() {
//The Rotary Encoder is mainly in the AttahcedInput routines PinA and PinB, but this routine handles the interface to the rest of the program for flow

  PinAONS=0; //Reset the ONS
  if (PinAONSCMD==1) {
    PinAONS=1; //Set a ONS
    PinAONSCMD=0; //Reset the CMD interface bit to the Interupt routines
  }

  PinBONS=0; //Reset the ONS
  if (PinBONSCMD==1) {
    PinBONS=1; //Set a ONS
    PinBONSCMD=0; //Reset the CMD interface bit to the Interupt routines
  }
}
//==============================================================================================
void HumidityControl () {
  
  if ((Humidity < HumiditySP - HumidityDB) && HumidityOnOff==true && FailureCode==0) {
    DOHumOn = true;
  }
  if ((Humidity > HumiditySP) && HumidityOnOff==true && FailureCode==0) {
    DOHumOn = false;
  }
  if (TimerOneSecondPulse==1) {Serial.println(DOHumOn);}
  if (DOHumOn == true) {
    digitalWrite(DOHumOnPIN,true);
  } else {
    digitalWrite(DOHumOnPIN,false);
  }
  
}
//==============================================================================================
void StateEngine() {

//Create a button push ONS that will turn on the display
  UserFeedbackONS==0;
  UserFeedbackONS = PinAONS | PinBONS | DIPBOnONS | DIPBOffONS | DIPBLongPressONS;

//Process state transitions in one place
  if (StateCMD!=0 && State!=StateCMD) {
    if (debug) {
      Serial.print(F("State Transition from "));
      Serial.print(State);
      Serial.print(" to ");
      Serial.print(StateCMD);
      Serial.println();
    }
      State=StateCMD;
      StateCMD=0;
   
  }

  
//Process states
  if (State==5) { //Idle State, screen off slow update rate, wait for user input
    if (TimerOneSecondPulse==1) { DiplayUpdateMainScreen();} //update the main screen once a second while idle
    
      if (UserFeedbackONS==1) {
        StateCMD=7;
      }
  }     
  //----------------------  
  if (State==7) {
     DisplayTurnOnMillis = millis(); //Record when the display turned on
      if (FailureCode==0) {
        DiplayUpdateMainScreen();
        StateCMD=10;
      } else { 
        StateCMD=50;
      }
  }
  //----------------------  
  if (State==10) { //Turning on step
      if (DisplayTurnOnMillis + DisplayTurnOnMillisPRE < millis()) {
        DisplayTurnOffMillis = millis();
        StateCMD=20;
         if (debug) {Serial.println(F("Main Screen"));}
      } 
  }   
  //----------------------    
  if (State==20) { // 20 Main screen
    if (TimerHalfSecondPulse==1 | PinAONS==1 | PinBONS==1) { DiplayUpdateMainScreen();}


    
      if (DIPBQuickPressONS ==true) { //on a ONS of the pushbutton true toggle humidity on/off
        if (HumidityOnOff==true) {
          HumidityOnOff = false;
        } else{
          HumidityOnOff = true;
        }
        DisplaySetHumidityOnOff(); //immdeiatly update the status of the humidity for reponsiveness
      }

      if (DIPBLongPressONS==true) { //long press
        ConfigScreenPositionCMD=5;
        lcd.clear();
        DiplayConfigScreen();
        StateCMD=30;
      }
   
  }  
   //----------------------   
  if (State==30) { // 30 Configure Screen (future)
    
      
     if (TimerHalfSecondPulse==1 | PinAONS==1 | PinBONS==1) { DiplayConfigScreen();}
      
           
      if (DIPBQuickPressONS) {
         if (ConfigScreenPosition==5) {// █1. DeadBand: 1.2%RH
            ConfigScreenPositionCMD=20; //PB
         }  
         
          if (ConfigScreenPosition==20) {//  1. DeadBand: 1.2█%RH
            ConfigScreenPositionCMD=5; //PB
          }  
          
          if (ConfigScreenPosition==30) {// █2.Units °F
            ConfigScreenPositionCMD=40; //PB
          }
          
          if (ConfigScreenPosition==40) {//  2.Units °F█
            ReadDHT(); //Update once to change the units
            ConfigScreenPositionCMD=30; //PB
          } 
          if (ConfigScreenPosition==50) {// █3.Help
            StateCMD=40; //PB
          } 
          DiplayConfigScreen(); //immediatly update screen
      }

       //if (TimerHalfSecondPulse) {DiplayConfigScreen();}
      if (DIPBLongPressONS==true) { //long press
        lcd.clear();
        DiplayUpdateMainScreen();
        StateCMD=7;
      }
  }
  //----------------------  
  if (State==40) {   // 40 Help screen (future)
    lcd.clear();
    lcd.home();
    lcd.print("Help Screen:");
    lcd.setCursor(0,1);
    lcd.print("Humidity will turn");
    lcd.setCursor(0,2);
    lcd.print("on under ");
    lcd.print(HumiditySP - HumidityDB,1);
    lcd.print("%RH and");
    lcd.setCursor(0,3);
    lcd.print("off above ");
    lcd.print(HumiditySP,1);
    lcd.print("%RH");
    delay(10000);
    lcd.clear();
    StateCMD=30;
  }
  //----------------------  
  if (State==50) {  // 50 failure screen
    if (TimerOneSecondPulse==1) { DiplayUpdateMainScreen();}
    
  }
   //---------------------- 

  
//----------------------




if (UserFeedbackONS) {
  if (debug) {Serial.println(F("User Feedback Detected, turning on screen"));}
  lcd.backlight();
  DisplayTurnOffMillis = millis();
}

if (State!=5 && DisplayTurnOffMillis + DisplayTurnOffMillisPRE < millis()) {
  if (debug) { 
    Serial.println(F("Turning Display off"));
    Serial.println(F("Saving settings to EEPROM"));
  }
  lcd.noBacklight();
  EEPROMWrite();
  lcd.clear();
  DiplayUpdateMainScreen();
  StateCMD=5; //go back to idle state
}
 
}
//==============================================================================================
void  DiplayUpdateMainScreen()  {
  lcd.noBlink();
  
if (FailureCode > 0) { 
  DisplayFailureScreen();
  return; 
}

  //line 1
  lcd.setCursor(0,0);
  lcd.print("Humidity Control ");
  DisplaySetHumidityOnOff();
      
  //line 2
  DisplayUpdateHumidity();
  lcd.setCursor(5,1);
  lcd.print("%RH Current    ");

  //line 3
  DisplayUpdateHumiditySP();
  lcd.setCursor(5,2);
  lcd.print("%RH Setpoint   ");

  //line 4
  if (HeatIndexToggleTimer +5000 < millis()) {
    if (HeatIndexToggle==0) {
      HeatIndexToggle=1;
    } else {
      HeatIndexToggle=0;
    }
    HeatIndexToggleTimer = millis();
  }
  if (HeatIndexToggle==0) {//show temperature
    lcd.setCursor(1,3);
    lcd.print("      Temp = ");
    lcd.print(Temperature,1);
    lcd.setCursor(18,3);
    lcd.print((char)223);
    if (TemperatureEngUnits==0) {
      lcd.print("F");
    } else {
      lcd.print("C");
    }
  } else {//show Heat Index
    lcd.setCursor(1,3);
    lcd.print("Heat Index = ");
    lcd.print(HeatIndex,1);
    lcd.print((char)223);
    if (TemperatureEngUnits==0) {
      lcd.print("F");
    } else {
      lcd.print("C");
    }
  }
}
//==============================================================================================
void  DiplayConfigScreen()  {
  lcd.blink();
  lcd.setCursor(0,0);
  lcd.print("Configuration       ");
  lcd.setCursor(1,1);
  lcd.print("1.Deadband: ");
    lcd.print(HumidityDB,1);
  lcd.setCursor(17,1);
    lcd.print("%RH");
  lcd.setCursor(1,2);
  lcd.print("2.Units: ");
    lcd.print((char)223);
    if (TemperatureEngUnits==0) {
      lcd.print("F");
    } else {
      lcd.print("C");
    }
  lcd.setCursor(1,3);
  lcd.print("3.Help            ");
    

//Process state transitions in one place
  if (ConfigScreenPositionCMD!=0 && ConfigScreenPosition!=ConfigScreenPositionCMD) {
    ConfigScreenPosition=ConfigScreenPositionCMD;
    ConfigScreenPositionCMD=0;
  }

    
   if (ConfigScreenPosition==5) {// █1. DeadBand: 1.2 %RH      
      lcd.setCursor(0,1);
   }  
   
   if (ConfigScreenPosition==20) {//  1. DeadBand: 1.2█%RH
      lcd.setCursor(16,1);
    }  
    
    if (ConfigScreenPosition==30) {// █2.Units °F
      lcd.setCursor(0,2);
    }
    
    if (ConfigScreenPosition==40) {//  2.Units °F█
      lcd.setCursor(12,2);
    } 
        
    if (ConfigScreenPosition==50) {// █3.Help
      lcd.setCursor(0,3);
    } 


}

//==============================================================================================
void DisplayUpdateHumidity() {
  lcd.setCursor(1,1);
  lcd.print(Humidity,1);
}
//==============================================================================================
void DisplayUpdateHumiditySP() {
  lcd.setCursor(1,2);
  lcd.print(HumiditySP,1);
}
//==============================================================================================
void DisplaySetHumidityOnOff() {
  lcd.setCursor(17,0);
  if (HumidityOnOff==true) {
    lcd.print("ON ");
  } else {
    lcd.print("OFF");
  }
}


//==============================================================================================
void DisplayFailureScreen() {
  

  switch (FailureCode) {
    case 1: //Failure to Read from DHT Module
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Humidity Control Off");
      lcd.setCursor(0,1);
      lcd.print("ERROR: Humidity");
      lcd.setCursor(0,2);
      lcd.print("Sensor Failure");
      lcd.setCursor(0,3);
      lcd.print("                    ");
  }
}

//==============================================================================================
void EEPROMRead() {
//EEPROM Map:
//Byte0: 00000111<LSB
//  Bit0 = HumidityOnOff
//  Bit1 = TemperatureEngUnits
//  Bit2 = FirstPRogramRun
//byte1-4: float HumiditySP
//byte5-9: float HumidityDB

   byte ConfigByte = EEPROM.read(0);
   HumidityOnOff = 1 & ConfigByte;
   TemperatureEngUnits = 1 & (ConfigByte>>1);
   EEPROM.get(1,HumiditySP);
   EEPROM.get(5,HumidityDB);
   if (debug) { 
      Serial.println(F("----------------------------------"));
      Serial.println(F("EEPROMRead():"));
   }
    SerialPrintEEPROMValues();

}
//==============================================================================================
void EEPROMWrite() {
//EEPROM Map:
//Byte0: 00000111<LSB
//  Bit0 = HumidityOnOff
//  Bit1 = TemperatureEngUnits
//  Bit2 = FirstPRogramRun
//byte1-4: float HumiditySP
//byte5-9: float HumidityDB
    byte ConfigByte= (1<<2) | (TemperatureEngUnits<<1) | HumidityOnOff;
    EEPROM.write(0,ConfigByte);
    EEPROM.put(1,HumiditySP);
    EEPROM.put(5,HumidityDB);
    if (debug) {
      Serial.println(F("----------------------------------"));
      Serial.println(F("EEPROMWrite():"));
    }
    SerialPrintEEPROMValues();
}
//==============================================================================================
void SerialPrintEEPROMValues() {
  if (debug) {
   Serial.print(F("HumidityOnOff = "));
   Serial.print(HumidityOnOff);
   Serial.println("");
   Serial.print(F("TemperatureEngUnits = "));
   Serial.print(TemperatureEngUnits);
   Serial.println("");
   Serial.print(F("HumiditySP = "));
   Serial.print(HumiditySP,1);
   Serial.println("");
   Serial.print(F("HumidityDB = "));
   Serial.print(HumidityDB,1);
   Serial.println("");
   Serial.println(F("----------------------------------"));
  }
}
//==============================================================================================
void ConfigScreenHandler(int cmd) {

}
//==============================================================================================
void PinA(){
   
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    
    PinAONSCMD=1;
    
     if (FailureCode==0) {
        
        
        if (State==20) {
          HumiditySP = HumiditySP - 0.1; //decrement the encoder's position count
          HumiditySP=constrain(HumiditySP,20.0,80.0);
        }
        
        if (State==30) {
           if (ConfigScreenPosition==5) {// █1. DeadBand: 1.2 %RH
              ConfigScreenPositionCMD=30;
           }  
           
                   
            if (ConfigScreenPosition==20) {//  1. DeadBand: 1.2█%RH
              HumidityDB= HumidityDB - 0.1;
               HumidityDB=constrain(HumidityDB,0.0, 9.9);
            }  
            
            if (ConfigScreenPosition==30) {// █2.Units °F
              ConfigScreenPositionCMD=50;
            }
            
            if (ConfigScreenPosition==40) {//  2.Units °F█
              //left OR right: toggle units
              if (TemperatureEngUnits==true) {
                TemperatureEngUnits=false;
              } else {
                TemperatureEngUnits=true;            
              }
            }
            if (ConfigScreenPosition==50) {// █3.Help
              ConfigScreenPositionCMD=5;
            }          
          }
     }
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
  
}
//==============================================================================================

void PinB(){
   
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
       
     PinBONSCMD=1;
        
    if (FailureCode==0) {
      
  
      
      if (State==20) {
        HumiditySP = HumiditySP + 0.1; //increment the encoder's position count
        HumiditySP=constrain(HumiditySP,20.0,80.0);
      }
      
      if (State==30) {
         if (ConfigScreenPosition==5) {// █1. DeadBand: 1.2 %RH
            ConfigScreenPositionCMD=50;
         }  
                    
          if (ConfigScreenPosition==20) {//  1. DeadBand: 1.2█%RH
            HumidityDB= HumidityDB + 0.1;
             HumidityDB=constrain(HumidityDB,0.0, 9.9);
          }  
          
          if (ConfigScreenPosition==30) {// █2.Units °F
            ConfigScreenPositionCMD=5;
          }
          
          if (ConfigScreenPosition==40) {//  2.Units °F█
           //left OR right: toggle units
              if (TemperatureEngUnits==true) {
                TemperatureEngUnits=false;
              } else {
                TemperatureEngUnits=true;            
              }
          } 
                       
          if (ConfigScreenPosition==50) {// █3.Help
            ConfigScreenPositionCMD=30;
          }
      }
    }
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}
//==============================================================================================
void   UpdateAllTimers() {
  //Update all pulse timers

  //Set the pulse to 0 to clear it out if it was set from the last scan
  TimerOneSecondPulse = 0;
  TimerHalfSecondPulse = 0;
  TimerTenthSecondPulse = 0;
  TimerTenMSPulse = 0;
  TimerOneMSPulse = 0;

  //For each pulse timer, process the pulse
  for (int i = 0; i < PulseTimerLen; i++) {
    PulseTimerUpdate(i);
  }
  
}

//==============================================================================================

void PulseTimerUpdate(int index) {

  //when timer is enabled on a one shot, record the timestamp
  if (PulseTimerREM[index] == 0) {
    PulseTimerREM[index] = millis();
  }


  if (millis() >=  PulseTimerREM[index] + PulseTimerPRE[index]  ) { //Evaluate if the timer is finished
    PulseTimerREM[index] = PulseTimerREM[index] + PulseTimerPRE[index]; //Set the REM to the REM plus the preset to trigger exactly off of the next second

    if (index == 0) {
      TimerOneMSPulse = 1;
    } else if (index == 1) {
      TimerTenMSPulse = 1;
    } else if (index == 2) {
      TimerTenthSecondPulse = 1;
    } else if (index == 3) {
      TimerHalfSecondPulse = 1;
    } else if (index == 4) {
      TimerOneSecondPulse = 1;
    } else {
      Serial.print(F("ERROR: PulseTimerUpdate Switch did not find a case for value:"));
      Serial.println(index);
    }

  }
}
//==============================================================================================
void UpdateScanCounter() {
  //Update scan time statistics.  This routine should run every scan of the program
  ScanCounter++;
  unsigned long microsCurrent = micros(); // Record the current micros
  unsigned long LastScanTime = microsCurrent - microsREM; //calculate the last scan time
  microsREM = microsCurrent; //Remember for next time

  if (LastScanTime > MaxScanTimeuS) {
  MaxScanTimeuS = LastScanTime;
}
if (LastScanTime < MinScanTimeuS) {
  MinScanTimeuS = LastScanTime;
}

if (TimerOneSecondPulse == 1) {
  AvgScanTimeuS = float(PulseTimerPRE[TimerOneSecondPulseIndex]) * 1000.0 / float(ScanCounter);
    
      if (debug) {
          //Debug code for the scan time
          Serial.print(F("perf "));
          
          Serial.print(F("Avg:"));
          Serial.print(AvgScanTimeuS/1000,3);
          Serial.print(F("ms "));
          
          Serial.print(F("min="));
          Serial.print(float(MinScanTimeuS)/1000);
          Serial.print(F("ms "));
          
          Serial.print(F("max="));
          Serial.print(float(MaxScanTimeuS)/1000);
          Serial.print(F("ms "));
          
          Serial.print(F("scans per sec:"));
          Serial.print(ScanCounter);
          
          Serial.println("");
      }
    ScanCounter = 0;
    MaxScanTimeuS=0;
    MinScanTimeuS=4294967295;
  }
}
