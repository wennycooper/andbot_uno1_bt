#include <Streaming.h>
#include <Metro.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#define DEFAULTPOWER 100
#define DEFAULTTURN 100

SoftwareSerial I2CBT(9, 8); // PIN9 RX, PIN8 TX
/******************************************************************************************************** 
** Gavriel Krauss + Kevin Kuei
** 
** Robot Project - Original Prototype - Basic Driving
** 
** Version 0.0.0.5 Return to check every .1 seconds
**
** Get controls from serial connection.
** Drive with two servo-signal outputs.
** 
** Dont' forget a "commandRcv = 'R'" after any non-repeating function
**
*********************************************************************************************************/

const byte sen10SR = A5, sen10BR = A4, sen10SL = A0, sen10BL = A1;
const byte trigPin = A3, echoPin = A2, s1 = 11, s2 = 10;
const byte uSwitchFR = 3, uSwitchFL = 4, uSwitchBR = 5, uSwitchBC = 6, uSwitchBL = 7;

Servo wheelR, wheelL;

int dist10SR, dist10BR, dist10SL, dist10BL;
int drivePower, turnPower;
boolean isDriving, isTurning, lastForward, lastRight;

Metro updateCommand = Metro(100);

char commandArray[8];
volatile char commandRcv;
volatile int commandVal;

Metro readSonar = Metro(1000);
int distSonar, sonarSamples[5], sonarIndex;

void setup()
{
  I2CBT.begin(115200);
  
  pinMode(sen10SR, INPUT);
  pinMode(sen10BR, INPUT);
  pinMode(sen10SL, INPUT);
  pinMode(sen10BL, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(uSwitchFR, INPUT_PULLUP);
  pinMode(uSwitchFL, INPUT_PULLUP);
  pinMode(uSwitchBR, INPUT_PULLUP);
  pinMode(uSwitchBC, INPUT_PULLUP);
  pinMode(uSwitchBL, INPUT_PULLUP);
  
  wheelR.attach(s2);
  wheelL.attach(s1);
  
  Serial.begin(115200); // Higher if necc. (230400/250000/500000/1000000)
  
  for(int i = 0; i < 8; ++i)
    commandArray[i] = 0;
  
  isDriving = false;
  isTurning = false;
  lastForward = false;
  lastRight = false;
  drivePower = 0;
  turnPower = 0;
  distSonar = 0;
  sonarIndex = 0;
  
  
  commandRcv = 'R';
  commandVal = 0;
  delay(2000);
}

void loop()
{
  byte cmmd[20];
  int insize;
 
  noCrash();
  
  if (digitalRead(uSwitchFR) == LOW)
      Serial.println("uSwitchFR == LOW");
  if (digitalRead(uSwitchFL) == LOW)
      Serial.println("uSwitchFL == LOW");
  if (digitalRead(uSwitchBL) == LOW)
      Serial.println("uSwitchBL == LOW");
  if (digitalRead(uSwitchBC) == LOW)
      Serial.println("uSwitchBC == LOW");
  if (digitalRead(uSwitchBR) == LOW)
      Serial.println("uSwitchBR == LOW");

  //checkFront();
  //Serial.print("dist10BR = ");
  //Serial.println(dist10BR);
  //checkBehind();
  //Serial.print("dist10BL = ");
  //Serial.println(dist10BL);
  //checkSides();
  //Serial.print("dist10SR = ");
  //Serial.println(dist10SR);
  //checkSides();
  //Serial.print("dist10SL = ");
  //Serial.println(dist10SL);
  
  if(Serial.available() > 0)
  {
    Serial.readBytes(commandArray, 8); // Ex: Forward @ 100 speed: 2, 100
    while(Serial.available()) // Empty out garbage, only one command allowed at a time
    {
      Serial.read();
    }
    commandArray[7] = '\0';
    commandRcv = commandArray[0];// Alternatively for byte: = (byte) Serial.parseInt();
    commandVal = atoi(&commandArray[2]);
    Serial << "Amount: " << commandVal << endl;
    
    I2CBT.println("Hello!!");
    
    Serial << "The whole array: " << commandArray[0] << commandArray[1] << commandArray[2] << commandArray[3] << commandArray[4] << commandArray[5] << commandArray[6] << commandArray[7] << endl;
    for(int i = 0; i < 7; ++i)
    {
      commandArray[i] = '\0';
    }
  }
  
  if(readSonar.check() == 1)
  {
    sonarRead();
  }

  if(updateCommand.check() == 1)
  {
    useCommand();
  }
  
  if ((insize=(I2CBT.available()))>0){  //read BT cmd
      //Serial.print("input size = ");
      //Serial.println(insize);
      
      Serial.print("BT command size = ");
      Serial.print(insize);
      Serial.println(" ");
      
      for (int i=0; i<insize; i++){
        Serial.print(cmmd[i]=char(I2CBT.read()));    
        Serial.println(" ");
        
        if (cmmd[i] == 48) {  // stop
          commandRcv = '0';
          commandVal = 200;
          useCommand();
        }
        
        if (cmmd[i] == 50) {  // forward
          commandRcv = '2';
          commandVal = 400;
          useCommand();
        }
        
        if (cmmd[i] == 53) {  // turn left
          commandRcv = '5';
          commandVal = 400;
          useCommand();
        }
        
        if (cmmd[i] == 52) {  // turn right
          commandRcv = '4';
          commandVal = 400;
          useCommand();
        }
        
        if (cmmd[i] == 51) {  // reverse
          commandRcv = '3';
          commandVal = 400;
          useCommand();
        }
      }//此段請參考上一篇解釋
   }
}


void useCommand()
{
  switch (commandRcv)
  {
    case '0':
      //Serial << "~~Stop Driving~~" << endl;
      Serial.println("stopDriving");
      stopDriving();
      break;
    case '2':
      //Serial << "~~Forward March~~" << endl;
      Serial.println("Forward");
      if(commandVal == 0)
        drivePower = DEFAULTPOWER;
      else
        drivePower = commandVal;
      driveNow();
      break;
    case '3':
      //Serial << "~~REVERSE~~" << endl;
      Serial.println("Reverse");
      if(commandVal == 0)
        drivePower = -DEFAULTPOWER;
      else
        drivePower = -commandVal;
      driveNow();
      break;
    case '4':
      //Serial << "~~Turn Right~~" << endl;
      Serial.println("TurnRight");
      if(commandVal == 0)
        turnPower = DEFAULTTURN;
      else
        turnPower = commandVal;
      turnNow();
      break;
    case '5':
      //Serial << "~~Turn Left~~" << endl;
      Serial.println("TurnLeft");
      if(commandVal == 0)
        turnPower = -DEFAULTTURN;
      else
        turnPower = -commandVal;
      turnNow();
      break;
    case 'R':
      break;
    default:
      break;
  }
}

void driveNow()
{
  //Serial << "Start Driving Code" << endl;
  Serial.println("driveNow()");
  
  if(isDriving == true && drivePower > 0)
  {
    checkFront();
    //check the forward proximity sensors
    if(digitalRead(uSwitchFR) == LOW || digitalRead(uSwitchFL) == LOW || dist10BR > 500)
        //digitalRead(sen5R) == LOW || digitalRead(sen5L) == LOW ||
        //digitalRead(senC1) == LOW || digitalRead(senC2) == LOW)
    {
      Serial.println("front crashed!!");
      stopDriving();
      return;
    }
  }
  else if(isDriving == true && drivePower < 0)
  {
    checkBehind();
    if(digitalRead(uSwitchBR) == LOW || digitalRead(uSwitchBC) == LOW ||
        digitalRead(uSwitchBL) == LOW 
        || (dist10BL > 500))
    {
      Serial.println("behind crashed");
      stopDriving();
      return;
    }
  }  
  
  if(drivePower > 0)
  {
        //Serial << "Try to start Forward at " << drivePower << endl;
        wheelR.writeMicroseconds(1500 + drivePower);
        wheelL.writeMicroseconds(1500 + drivePower);
  }
  else if(drivePower < 0)
  {
      //Serial << "Try to start Reverse at " << drivePower << endl;
      wheelR.writeMicroseconds(1500 + drivePower);
      wheelL.writeMicroseconds(1500 + drivePower);
  }
  commandRcv = 'R';
}

void turnNow()
{
  //Serial << "Try to turn at " << turnPower << endl;
  Serial.println("turnNow");
  if(turnPower > 0)
  {
    wheelR.writeMicroseconds(1500 - turnPower);
    wheelL.writeMicroseconds(1500 + turnPower);
  }
  else if(turnPower < 0)
  { 
    wheelR.writeMicroseconds(1500 - turnPower);
    wheelL.writeMicroseconds(1500 + turnPower);
  }
  commandRcv = 'R';
}

void stopDriving()
{
    //Serial << "Try to stop driving" << endl;
    Serial.println("StopDriving()");
    wheelR.writeMicroseconds(1500);
    wheelL.writeMicroseconds(1500);
    drivePower = 0;
    turnPower = 0;
    commandRcv = 'R';
}

void noCrash()
{
  if(isDriving == true && drivePower > 0)
  {
    //check the forward proximity sensors
    if(digitalRead(uSwitchFR) == LOW || digitalRead(uSwitchFL) == LOW
        || dist10BR > 500)
        //digitalRead(sen5R) == LOW || digitalRead(sen5L) == LOW ||
        //digitalRead(senC1) == LOW || digitalRead(senC2) == LOW)
    {
      stopDriving();
    }
  }
  else if(isDriving == true && drivePower < 0)
  {
    checkBehind();
    if(digitalRead(uSwitchBR) == LOW || digitalRead(uSwitchBC) == LOW ||
        digitalRead(uSwitchBL) == LOW 
        || (dist10BL > 500))
    {
      stopDriving();
    }
  }
  else if(isTurning == true)
  {
    checkBehind();
    checkSides();
    if(digitalRead(uSwitchFR) == LOW || digitalRead(uSwitchFL) == LOW ||
        digitalRead(uSwitchBR) == LOW || digitalRead(uSwitchBC) == LOW ||
          digitalRead(uSwitchBL) == LOW || 
              (dist10BR > 500) || (dist10BL > 500) ||
                (dist10SR > 500) || (dist10SL > 500))
    {
      stopDriving();
    }
  }
}

void checkFront()
{
  int samplesBR[9];
//  int samplesBL[9];
  for(int i = 0; i<9; i++)
  {
    samplesBR[i] = analogRead(sen10BR);
//    samplesBL[i] = analogRead(sen10BL);
  }
  insertSort(samplesBR, 9);
//  insertSort(samplesBL, 9);
  dist10BR = samplesBR[4];
//  dist10BL = samplesBL[4];
  //Serial << "Behind numbers: " << dist10BR << " " << dist10BL << endl;
}

void checkBehind()
{
//  int samplesBR[9];
  int samplesBL[9];
  for(int i = 0; i<9; i++)
  {
//    samplesBR[i] = analogRead(sen10BR);
    samplesBL[i] = analogRead(sen10BL);
  }
//  insertSort(samplesBR, 9);
  insertSort(samplesBL, 9);
//  dist10BR = samplesBR[4];
  dist10BL = samplesBL[4];
  //Serial << "Behind numbers: " << dist10BR << " " << dist10BL << endl;
}

void checkSides()
{
  int samplesSR[9];
  int samplesSL[9];
  for(int i = 0; i<9; i++)
  {
    samplesSR[i] = analogRead(sen10SR);
    samplesSL[i] = analogRead(sen10SL);
  }
  insertSort(samplesSR, 9);
  insertSort(samplesSL, 9);
  dist10SR = samplesSR[4];
  dist10SL = samplesSL[4];
  //Serial << "Side numbers: " << dist10SR << " " << dist10SL << endl;
}

void insertSort(int *a, int n) //name of array, size of it
{
  for(int i = 1; i < n; ++i)
  {
    int j = a[i];
    int k;
    for (k = i - 1; (k >= 0) && (j < a[k]); k--)
    {
      a[k + 1] = a[k];
    }
    a[k + 1] = j;
  }
}

void sonarRead()
{
  unsigned long duration;
  int distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (int) duration / 58;
  sonarSamples[sonarIndex] = distance;
  //Serial << " Index " << sonarIndex << " Raw Value " << sonarSamples[sonarIndex] << endl;
  ++sonarIndex;
  if(sonarIndex == 5)
  {
    insertSort(sonarSamples, 5);
    distSonar = sonarSamples[2];
    Serial << "Sonar: " << distSonar << " cm" << endl;
    sonarIndex = 0;
  }
}
