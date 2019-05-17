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
const int pumpOffTime = 5000; //15s
//const int pumpOnTime = 45000; //45s
//const int pumpOffTime = 5400000; //1.5hr

const int interruptPumpTogglePin = 2;
const int interruptTimerTogglePin = 3;

const int lowFloatPin = 10;
const int pumpPin = 9;

// micros
const int threadTimerTime = 20000;

volatile boolean timerIsOn = false;
volatile boolean pumpIsOn = false;


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

PumpTimerThread pumpTimerThread = PumpTimerThread();

/*
 * setup
 */
void setup() {

  Serial.begin(9600);
  Serial.println("Initializing");
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(lowFloatPin, INPUT);
  pinMode(pumpPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(interruptTimerTogglePin), timerToggle, LOW);
  attachInterrupt(digitalPinToInterrupt(interruptPumpTogglePin), pumpToggle, LOW);
  
  delay(1000);

  pumpTimerThread.onRun(pumpTimerCallback);
  pumpTimerThread.setInterval(pumpOffTime);

  control1.add(&pumpTimerThread);

  Timer1.initialize(threadTimerTime);
  Timer1.attachInterrupt(threadTimerCallback);
  Timer1.start();

  digitalWrite(pumpPin, LOW);

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
  autoPump(pumpPin, pumpOnTime);
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

  timerIsOn = !timerIsOn;

  if(timerIsOn){
    pumpTimerThread.reset();
    pumpTimerThread.enabled = true;
  }
  else{
    pumpTimerThread.enabled = false;
  }
}

/*
 * pumpToggle
 * 
 * manually turn pump on/off
 */
void pumpToggle(){

  pumpIsOn = !pumpIsOn;

  if(pumpIsOn)
    pumpOn(pumpPin);
  else
    pumpOff(pumpPin);
}

/*
 * autoPump
 * 
 * runs pump based on low float state and time interval
 * onTime in millis
 * 
 */
void autoPump(int pumpPin, int onTime){

  Serial.print("OnPump(s): ");
  Serial.println(onTime/1000);

  byte lowFloatState = digitalRead(lowFloatPin);
  
  digitalWrite(LED_BUILTIN, HIGH);

  if(lowFloatState == HIGH)
    runPump(pumpPin, onTime);
  else{
    Serial.println("Nothing to pump.");
  }
  
  digitalWrite(LED_BUILTIN, LOW);  
}

/*
 * runPump
 * 
 * run pump for specified millis
 * runTime in millis
 * 
 */
void runPump(int pumpPin, int runTime){

  pumpOn(pumpPin);
  long t = millis();
  while(millis() - t <= runTime) { }
  pumpOff(pumpPin);    
}

/*
 * pumpOn
 * 
 * turn pump on
 * 
 */
void pumpOn(int pumpPin){
  Serial.println("Pump On");
  digitalWrite(pumpPin, HIGH);  
}

/*
 * pumpOff
 * 
 * turn pump off
 * 
 */
void pumpOff(int pumpPin){
  digitalWrite(pumpPin, LOW);
  Serial.println("Pump Off");    
}
