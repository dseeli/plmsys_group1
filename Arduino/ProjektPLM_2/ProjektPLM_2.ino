#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include <Servo.h>

#include "BluefruitConfig.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "TimerOne.h"
    
#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
    
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

enum jobs {
  JOB_IDLE,
  JOB_PICKUP,
  JOB_DROP_FORWARD,
  JOB_DROP_FORWARD_PART_DETECTION,
  JOB_DROP_BACKWARD,
  JOB_DROP_BACKWARD_PART_DETECTION
};

const int InPin_PartDetector = A0;
int moduleJob;
int timerCounter;

void setup() {
  Timer1.initialize(10000000); 
  AFMS.begin();
  myMotor->setSpeed(200);

  Serial.begin(9600);
  initBLE();
  ble.verbose(false);  // debug info is a little annoying after this point!
  /* Wait for connection*/
  while (! ble.isConnected()) {
      delay(500);
  }
  moduleJob = JOB_IDLE;
}


void loop() {
  listenBLE();
  
  switch(moduleJob){
    case JOB_IDLE:{
      break;
    }

    case JOB_PICKUP: {
      myMotor->run(FORWARD);
      if(TCNT1 > 10000)
      {
        timerCounter++;
        if(timerCounter > 1200)
        {
          sendBLE("pickupSuccess(0)");
          Serial.println("pickupSuccess(0)");
          myMotor->run(RELEASE);
          moduleJob = JOB_IDLE;
        }
      }    
      if (partIsDetected() == 1) {
        sendBLE("pickupSuccess(1)");
        Serial.println("pickupSuccess(1)");
        myMotor->run(RELEASE);
        moduleJob = JOB_IDLE;
      }
      break;  
    }
      
    case JOB_DROP_BACKWARD:{
      myMotor->run(BACKWARD);
      if(TCNT1 > 10000)
      {
        timerCounter++;
        if(timerCounter > 1200)
        {
          moduleJob = JOB_DROP_BACKWARD_PART_DETECTION;
        }
      }
      break;     
    }

    case JOB_DROP_BACKWARD_PART_DETECTION:{
      myMotor->run(FORWARD); 
      if(TCNT1 > 10000)
      {
        timerCounter++;
        if(timerCounter > 2500)
        {
          myMotor->run(RELEASE);
          sendBLE("dropSuccess(1)");
          Serial.println("dropSuccess(1)");
          Timer1.stop();
          moduleJob = JOB_IDLE;
        }
        if (partIsDetected() == 1) {
          sendBLE("dropSuccess(0)");
          Serial.println("dropSuccess(0)");
          myMotor->run(RELEASE);
          moduleJob = JOB_IDLE;
        }
      }
      break;     
    }
     
    case JOB_DROP_FORWARD:{
      myMotor->run(FORWARD);
      if(TCNT1 > 10000)
      {
        timerCounter++;
        if(timerCounter > 1200)
        {
          moduleJob = JOB_DROP_FORWARD_PART_DETECTION;
        }
      }
      break;     
    }

    case JOB_DROP_FORWARD_PART_DETECTION:{
      myMotor->run(BACKWARD); 
      if(TCNT1 > 10000)
      {
        timerCounter++;
        if(timerCounter > 2500)
        {
          myMotor->run(RELEASE);
          sendBLE("dropSuccess(1)");
          Serial.println("dropSuccess(1)");
          Timer1.stop();
          moduleJob = JOB_IDLE;
        }
        if (partIsDetected() == 1) {
          sendBLE("dropSuccess(0)");
          Serial.println("dropSuccess(0)");
          myMotor->run(RELEASE);
          moduleJob = JOB_IDLE;
        }
      }
      break;     
    }
  }
}

int partIsDetected() {
  int partFound = 0;
  if(analogRead(InPin_PartDetector) < 100)
  {
    partFound = 1;
  }
  return partFound;
}

// Handle API commands
void handleApiCommands(String command) {
  Serial.println(command);
  if (command == "pickup();") {
    Timer1.restart();
    timerCounter = 0;
    moduleJob = JOB_PICKUP;    
  }
  if (command == "drop(1);") {
    Timer1.restart();
    timerCounter = 0;
    moduleJob = JOB_DROP_FORWARD;
  }
  if (command == "drop(0);") {
    Timer1.restart();
    timerCounter = 0;
    moduleJob = JOB_DROP_BACKWARD;
  }
}

// Listen to incomminc commands from Bluetooth
void listenBLE() {
  ble.println("AT+BLEUARTRX");
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }
  handleApiCommands(ble.buffer);
}

// Send message over Bluetooth
void sendBLE(String msg) {
  ble.print("AT+BLEUARTTX=");
  ble.println(msg);

  // check response stastus
  if (! ble.waitForOK() ) {
    Serial.println(F("Failed to send?"));
  }
}

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void initBLE() {
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
     Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }
  

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();
  ble.println("AT+GAPDEVNAME=Domenick");

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();
}
