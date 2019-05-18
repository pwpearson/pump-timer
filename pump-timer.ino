/*
 * Pump Timer Proto
 * 
 */

#include <ThreadController.h>
#include <StaticThreadController.h>
#include <Thread.h>
#include <TimerOne.h>
#include "Arduino.h"

// millis
const int pumpOnTime = 5000; //5s
const int pumpOffTime = 10000; //5s
//const int pumpOnTime = 45000; //45s
//const int pumpOffTime = 5400000; //1.5hr

const int interruptPumpTogglePin = 2;
const int interruptTimerTogglePin = 3;

const int pumpTimerLED = 4;

const int lowFloatPin = 11;
const int pumpPin = 12;
                                                                                                      
// micros
const int threadTimerTime = 20000;

volatile boolean requestTimerOn = false;
volatile boolean requestPumpOn = false;
volatile boolean pumpIsOn = false;
volatile boolean pumpTimerIsOn = false;

ThreadController control1 = ThreadController();

/*
 * PumpTimerThread inherits Thred
 */
class PumpTimerThread: public Thread{
  public:
          void reset(){
            runned();
          }
};

class LowFloatSensorThread: public Thread{
  public:
          byte value;
          int pin;

          void run(){

            value = digitalRead(lowFloatPin);
            runned();
          }
};

class AutoRunTimeThread: public Thread{
  public:
          void reset(){
            runned();
          }
};


PumpTimerThread pumpTimerThread = PumpTimerThread();
LowFloatSensorThread lowFloatSensorThread = LowFloatSensorThread();
Thread runPumpThread = Thread();
Thread startStopPumpTimerThread = Thread();
AutoRunTimeThread autoRunTimeThread = AutoRunTimeThread();

/*
 * setup
 */
void setup() {

  Serial.begin(9600);
  Serial.println("Initializing");
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(lowFloatPin, INPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(pumpTimerLED, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(interruptTimerTogglePin), timerToggle, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPumpTogglePin), pumpToggle, RISING);
  
  delay(1000);

  // config pump timer thread
  pumpTimerThread.onRun(pumpTimerCallback);
  pumpTimerThread.setInterval(pumpOffTime);
  pumpTimerThread.enabled = false;
  digitalWrite(pumpTimerLED, LOW);

  lowFloatSensorThread.pin = lowFloatPin;
  lowFloatSensorThread.setInterval(5000);

  // config pump run thread (button)
  runPumpThread.onRun(pumpRunCallback);
  runPumpThread.setInterval(1000);
  digitalWrite(pumpPin, LOW);

  // config pump timer stop/start thread (button)
  startStopPumpTimerThread.onRun(pumpTimerStartStopCallback);
  startStopPumpTimerThread.setInterval(1000);

  autoRunTimeThread.onRun(autoRunTimeCallback);
  autoRunTimeThread.setInterval(pumpOnTime);
  autoRunTimeThread.enabled = false;

  // register threads with controller
  control1.add(&pumpTimerThread);
  control1.add(&runPumpThread);
  control1.add(&startStopPumpTimerThread);
  control1.add(&lowFloatSensorThread);
  control1.add(&autoRunTimeThread);

  // config and start timer to run threads
  Timer1.initialize(threadTimerTime);
  Timer1.attachInterrupt(threadTimerCallback);
  Timer1.start();

  Serial.println("Starting Pump Timer");
}

/*
 * loop
 */
void loop() {
  
}

/*
 * pumpTimerCallback
 * 
 * called when the pumpTimerThread wakes up
 * 
 */
void pumpTimerCallback(){
  
  digitalWrite(LED_BUILTIN, HIGH);

  if(lowFloatSensorThread.value == HIGH){
    Serial.println("Auto Time Pump Start");
    pumpOn(pumpPin);
    autoRunTimeThread.enabled = true;
    autoRunTimeThread.reset();
  }
  else{
    Serial.println("Nothing to pump.");
  }
}

void autoRunTimeCallback(){

  pumpOff(pumpPin);
  autoRunTimeThread.enabled = false;
  digitalWrite(LED_BUILTIN, LOW);
}

/*
 * pumpRunCallback
 */
void pumpRunCallback(){

  if(!pumpTimerIsOn){
    if(requestPumpOn){
      pumpOn(pumpPin);
    }
    else{
      pumpOff(pumpPin);
    }
  }
}

/*
 * pumpTimerStartStopCallback
 */
void pumpTimerStartStopCallback(){

  if(requestTimerOn)
    pumpTimerOn();
  else
    pumpTimerOff();
}

/*
 * threadTimerCallback
 * 
 * called when the internal timer goes off;
 * calls the thread controllers run to check threads
 * 
 */
void threadTimerCallback(){
  control1.run();  
}

/*
 * timerToggle
 * 
 * turns the pump timer on/off
 */
void timerToggle(){
  Serial.println("Auto Time Button Pressed");
  requestTimerOn = !requestTimerOn;
}

/*
 * pumpToggle
 * 
 * manually turn pump on/off
 */
void pumpToggle(){
  Serial.println("Pump Button Pressed");
  requestPumpOn = !requestPumpOn;
}

/*
 * pumpTimerOn
 */
void pumpTimerOn(){
  if(!pumpTimerIsOn){
    pumpTimerIsOn = true;
    Serial.println("Pump Timer is On");
    pumpTimerThread.reset();
    pumpTimerThread.enabled = true;
    digitalWrite(pumpTimerLED, HIGH);  
  }
}

/*
 * pumpTimerOff
 */
void pumpTimerOff(){
  if(pumpTimerIsOn){
    pumpTimerIsOn = false;
    Serial.println("Pump Timer is Off");
    pumpTimerThread.enabled = false;
    digitalWrite(pumpTimerLED, LOW);  
  }
}


/*
 * pumpOn
 * 
 * turn pump on
 * 
 */
void pumpOn(int pumpPin){
  if(!pumpIsOn){
    pumpIsOn = true;
    Serial.println("Pump On");
    digitalWrite(pumpPin, HIGH);
  }  
}

/*
 * pumpOff
 * 
 * turn pump off
 * 
 */
void pumpOff(int pumpPin){
  if(pumpIsOn){
    pumpIsOn = false;
    digitalWrite(pumpPin, LOW);
    Serial.println("Pump Off");    
  }
}
