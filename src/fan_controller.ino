#include <LiquidCrystal.h>
#include <Wire.h>
#include <EEPROMex.h>    //https://github.com/thijse/Arduino-Libraries/tree/master/EEPROMEx
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>


#define VERSION              1.1     
#define UP                   1
#define DOWN                 2
#define TIME_BUTTON_SHORT    500
#define TIME_BUTTON_LONG     1000

#define THERMISTORNOMINAL    100000                // resistance at 25 degrees C     
#define TEMPERATURENOMINAL   25                    // temp. for nominal resistance (almost always 25 C)
#define BCOEFFICIENT         3950                  // The beta coefficient of the thermistor (usually 3000-4000)
#define SERIESRESISTOR       98700                 // the value of the 'other' resistor 
#define NUMSAMPLES           20                    // how many samples to take and average, more takes longer
int samples[NUMSAMPLES];


//pin out
#define LCD_DATA_7      2
#define LCD_DATA_6      3
#define LCD_DATA_5      4
#define LCD_DATA_4      5
#define LCD_DATA_E      6
#define LCD_DATA_RS     7
#define BUTTON_UP       8
#define BUTTON_DOWN     9
#define PIN_THM         0
#define PIN_PWM         11

//EEPROM
#define ADDR_FLOAT_VERSION    0
#define ADDR_DOUBLE_KP        5
#define ADDR_DOUBLE_KI        10
#define ADDR_DOUBLE_KD        15
#define ADDR_DOUBLE_SETPOINT  20


//byte version;
const double default_kp=2, 
             default_ki=0.5, 
             default_kd=2,                                //default PID parameters
             default_setpoint=30;
double  input, 
        output, 
        setpoint,
        kp,
        ki,
        kd;  
boolean tuning = false;

unsigned long serialTime, 
              lcdTime,
              buttonUpTime,
              buttonDownTime,
              startTime;
//unsigned int tempRecord[NUM_RECORD], fanRecord[NUM_RECORD], indexRecord;


/*
byte ATuneModeRemember=2;
double outputStart=5;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;
unsigned long  modelTime;
*/

LiquidCrystal  lcd      (LCD_DATA_RS, LCD_DATA_E, LCD_DATA_4, LCD_DATA_5, LCD_DATA_6, LCD_DATA_7);
PID            myPID    (&input, &output, &setpoint, default_kp, default_ki, default_kd, REVERSE);//DIRECT);
PID_ATune      aTune    (&input, &output);




void setup() {
  Serial.begin(9600);
  Serial.println("init...");
  
  lcd.begin(2,16);  
  lcd.home();
  lcd.print("init...");
     
  pinMode(BUTTON_UP,   INPUT_PULLUP);   
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
     
  serialTime      = 0;
  lcdTime         = 0;
  startTime       = 0;
  buttonUpTime    = 0;
  buttonDownTime  = 0;
  
  if (EEPROM.readFloat(ADDR_FLOAT_VERSION) == VERSION){
    Serial.println("restoring EEPROM values");
    setpoint = EEPROM.readDouble(ADDR_DOUBLE_SETPOINT);
    kp       = EEPROM.readDouble(ADDR_DOUBLE_KP);
    ki       = EEPROM.readDouble(ADDR_DOUBLE_KI);
    kd       = EEPROM.readDouble(ADDR_DOUBLE_KD);
    
  } else {
    Serial.println("setting default PID values");
    setpoint = default_setpoint;
    kp       = default_kp;
    ki       = default_ki;
    kd       = default_kd;  
  }
  
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" Kp: "); Serial.print(kp); Serial.print(" ki: "); Serial.print(ki); Serial.print(" Kd: "); Serial.print(kd);
  myPID.SetTunings(kp,ki,kd);  
  myPID.SetMode(AUTOMATIC);    //set mode automatic (on) manual (off)
  
  
  //restore default
  if ( digitalRead(BUTTON_UP)==LOW && digitalRead(BUTTON_DOWN)==LOW){
    Serial.println("restoring default PID values");
    setpoint = default_setpoint;
    kp       = default_kp;
    ki       = default_ki;
    kd       = default_kd;    
    EEPROM.writeDouble(ADDR_DOUBLE_KP, kp);
    EEPROM.writeDouble(ADDR_DOUBLE_KI, ki);
    EEPROM.writeDouble(ADDR_DOUBLE_KD, kd);
    EEPROM.writeDouble(ADDR_DOUBLE_SETPOINT, setpoint);
    EEPROM.writeFloat(ADDR_FLOAT_VERSION, VERSION);
    
  } else {
    //autotunning
    if ( digitalRead(BUTTON_UP)==LOW){
      aTune.SetOutputStep(10);
      aTune.SetControlType(1);
      aTune.SetNoiseBand(0.8);
      aTune.SetLookbackSec(20);
      
      tuning = true;
      
      lcd.clear();
      lcd.home();
      lcd.print("tuning...");
      Serial.println("tuning...");
    }  
  }
}






void loop() { 
  //Serial.println(millis()-now);
  //now   = millis();                  //overflows in ~ 50 days!!  
  input = adc2temp();                //read current temperature in ºC 
  
  if(tuning){
    if ( aTune.Runtime() != 0 ){
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      EEPROM.writeDouble(ADDR_DOUBLE_KP, kp);
      EEPROM.writeDouble(ADDR_DOUBLE_KI, ki);
      EEPROM.writeDouble(ADDR_DOUBLE_KD, kd);
      EEPROM.writeFloat(ADDR_FLOAT_VERSION,VERSION);
      myPID.SetTunings(kp,ki,kd);  
      
      Serial.print(" Kp: "); Serial.print(kp); Serial.print(" ki: "); Serial.print(ki); Serial.print(" Kd: "); Serial.print(kd);
      
      tuning = false;
    }  
    
  } else {
    myPID.Compute();                   //calculate output
  }
  
  analogWrite(PIN_PWM,output);       //fan control (default PID output range is between 0-255)
  
  switch(checkButtons()){
    case UP:
      setpoint++;
      EEPROM.writeDouble(ADDR_DOUBLE_SETPOINT, setpoint);
      break;
    
    case DOWN:
      setpoint--;
      EEPROM.writeDouble(ADDR_DOUBLE_SETPOINT, setpoint);
      break;    
  }
  
  
  //update LCD if needed
  
  if(millis() > lcdTime) {
    if(!tuning){
      //lcd.clear();
      lcd.home();
      lcd.print("Temp: "); lcd.print ( input ); lcd.print ( " (" ); lcd.print((int)setpoint); lcd.print ( ") " );    
      lcd.setCursor ( 0, 1 );
      lcd.print("Vel: "); lcd.print ( map(output,0,255,0,100) ); lcd.print ( "%    " );
      
    } else {
      lcd.setCursor ( 0, 1 );
      lcd.print("S"); lcd.print((int)setpoint); lcd.print(" T"); lcd.print ( (int)input ); lcd.print(" V"); lcd.print ( output );//map(output,0,255,0,100) );
    }
    
    //serialTime+=500; 
    lcdTime = millis() + 200; 
  }
  
  //send with processing if it's time
  if(millis() > serialTime) {
    Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
    Serial.print("input: ");Serial.print(input); Serial.print(" ");
    Serial.print("output: ");Serial.println(output);
    serialTime = millis() + 1000;
  }
  
  
  //helps firt start
  if ( output<15 && millis()>startTime ){
    Serial.println("Help start...");
    analogWrite(PIN_PWM,0x7F);
    delay(1000);
    analogWrite(PIN_PWM,output);    
    
    startTime = millis() + 300000;  // ~5min
  }
}






byte checkButtons(){    
  if (digitalRead(BUTTON_UP) == LOW) {    
    Serial.println(buttonUpTime);
    if (buttonUpTime == 0) {
      Serial.println("Button up");
      delay(15);
      buttonUpTime = millis() + TIME_BUTTON_LONG;
      return UP;
      
    } else if (millis() > buttonUpTime){
      Serial.println("Button up");
      buttonUpTime += TIME_BUTTON_SHORT;
      return UP;    
    }
    
  } else {
    buttonUpTime = 0;    
  }
  
  if (digitalRead(BUTTON_DOWN) == LOW) {
    if (buttonDownTime == 0) {
      Serial.println("Button down");
      delay(15);
      buttonDownTime = millis() + TIME_BUTTON_LONG;
      return DOWN;
      
    } else if (millis() > buttonDownTime){
      Serial.println("Button down");
      buttonDownTime += TIME_BUTTON_SHORT;
      return DOWN;    
    }
    
  } else {
    buttonDownTime = 0;    
  }
  
  return -1;
}


//TODO:Usar librería "https://github.com/jeroendoggen/Arduino-signal-filtering-library" para el filtrado
double adc2temp(){
  uint8_t i,j, kk;
  boolean flag;
  float average;
 
  // take N samples in a row, with a slight delay
  for (i=0; i<NUMSAMPLES; i++) {
   samples[i] = analogRead(PIN_THM);
   delay(10);
  }
    
  // average all the samples out  
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;
  
  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
 
  double steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
 
  return steinhart;
}


