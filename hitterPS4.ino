 #include <PS4BT.h>
#include <usbhub.h>
#include<Servo.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
PS4BT PS4(&Btd, PAIR);


bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;

Servo ms;

int pwm1 = 3 , dir1 = 22 ;
int pwm5 = 4 , dir5 = 5 ;
int pwm3 = 6, dir3 = 7 ;
int pwm4 = 8, dir4 = 24;
int pwm2 = 9 , dir2= 26 ;
int pwm6 = 11, dir6= 12 ;
int srv = 2 ;

int trig1 = 49, echo1 = 48 ;
int trig2 = 44, echo2 = 46;

int in1 = 25 , in2 = 33;
int htt = 180;

int rpm = 180,rot = 130,vpwm= 80;
int uppr = 134, lowr = 116 ;
int upprd = 128, lowrd = 120; 
int vout1 = 20, vout2 = 21;
int pos = 0 ,cur ;
int flagW = 0 , flagF =  0 , flagS =  0;
int Mspd = 0;

void setup() {
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));

  
  pinMode(pwm1, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(dir3, OUTPUT);
  pinMode(pwm4, OUTPUT);
  pinMode(dir4, OUTPUT);
  pinMode(pwm5, OUTPUT);  
  pinMode(dir5, OUTPUT);
  pinMode(pwm6, OUTPUT);  
  pinMode(dir6, OUTPUT);
  pinMode(in2,OUTPUT);
    attachInterrupt(digitalPinToInterrupt(vout1),readEncoder,RISING);
    
  digitalWrite(in2,LOW);

  ms.attach(srv);
}
void readEncoder(){
int b = digitalRead(vout2);
  if(b>0){
  pos++;
}else{
  pos--;
}}
  

void battery(){
   int bat = map(PS4.getBatteryLevel(),0,15,0,100);
   if(bat >= 75 ){
    PS4.setLed(Green);
   }else if(bat < 75 && bat >=50){
    PS4.setLed(Blue);
   }else if(bat < 50 && bat >=20){
    PS4.setLed(Yellow);
   }else{
    PS4.setLed(Red);
   }
}
void loop() {
  Usb.Task();

  if (PS4.connected()) {
    battery();
     int valueLY = PS4.getAnalogHat(LeftHatY);
    int valueLX = PS4.getAnalogHat(LeftHatX);
    int valueRY = PS4.getAnalogHat(RightHatY);
    int valueRX = PS4.getAnalogHat(RightHatX);
     int valueL2 = PS4.getAnalogButton(L2);
    int valueR2 = PS4.getAnalogButton(R2);

    if(valueLY < lowr  && valueLX < uppr+10 && valueLX > lowr-10){
    int rpmm = map(valueLY,lowr,0,0,rpm);
    forward(rpmm);
  }
    else if(valueLY > uppr  && valueLX < uppr+10 && valueLX > lowr-10){
      int rpmm = map(valueLY,uppr,255,0,rpm);
    reverse(rpmm);
  }
   else if(valueLX < lowr  && valueLY < uppr+10 && valueLY > lowr-10){
    int rpmm = map(valueLX,lowr,0,0,rpm);
    left(rpmm);
  }
    else if(valueLX > uppr  && valueLY < uppr+10 && valueLY > lowr-10){
      int rpmm = map(valueLX,uppr,255,0,rpm);
    right(rpmm);
  }else if(valueL2 > 10){
    int rpmm = map(valueL2,11,255,0,rot);
    antiClockwise(rpmm);
    }
     else if(valueR2 > 10){
      int rpmm = map(valueR2,11,255,0,rot);
    clockwise(rpmm);
  }
  else if(valueLY > lowr && valueLY < uppr  && valueLX < uppr && valueLX > lowr){
    stopp();
  }

  if(PS4.getButtonPress(L1)){
    digonal(rpm);
  }
  hitterMechanism(); 
//  Serial.print("Pos = ");
//  Serial.println(pos);

  }}
    void digonal(int rpmm){
      PS4.setLed(160,160,160);
int valueLY = PS4.getAnalogHat(LeftHatY);
    int valueLX = PS4.getAnalogHat(LeftHatX);
 if(valueLY < lowr  && valueLX < uppr+10 && valueLX > lowr-10){
    int rpmm = map(valueLY,lowr,0,0,rpm);
    leftUp(rpmm);
  }
    else if(valueLY > uppr  && valueLX < uppr+10 && valueLX > lowr-10){
      int rpmm = map(valueLY,uppr,255,0,rpm);
    rightDown(rpmm);
  }
   else if(valueLX < lowr  && valueLY < uppr+10 && valueLY > lowr-10){
    int rpmm = map(valueLX,lowr,0,0,rpm);
    leftDown(rpmm);
  }
    else if(valueLX > uppr  && valueLY < uppr+10 && valueLY > lowr-10){
      int rpmm = map(valueLX,uppr,255,0,rpm);
    rightUp(rpmm);}
    else if(valueLY > lowr && valueLY < uppr  && valueLX < uppr && valueLX > lowr){
    stopp();
}}


    void hitterMechanism(){
  if(PS4.getButtonClick(CIRCLE)){
    if(flagW == 0 ){
      digitalWrite(dir5,HIGH);
      analogWrite(pwm5,htt);
      flagW = 1;
      Serial.println("Start Hitter");
    }else if(flagW == 1){
      digitalWrite(dir5,LOW);
      analogWrite(pwm5,0);
      flagW = 0;
    }
  }
  if(PS4.getButtonPress(CIRCLE)){
  PS4.setLed(51,51,0);
}

  if(PS4.getButtonPress(R1)){
      Serial.println("FIRE");
      digitalWrite(in2, HIGH);
       PS4.setLed(153,0,0);
  }else{
      digitalWrite(in2,LOW);
  }
    if(PS4.getButtonClick(TRIANGLE)){
     if(flagS == 0){
      Serial.println("Close");
   ms.write(60);
      flagS = 1;
    }else if(flagS == 1){
      Serial.println("Open");
      ms.write(128);
      flagS = 0;
    }
  }
 if(PS4.getButtonPress(TRIANGLE)){
    PS4.setLed(204,0,204);
  }

  
//  if(ps2x.ButtonPressed(PSB_R1)){
//    cur = 640;
//  }
//  if(ps2x.ButtonPressed(PSB_R2)){
//    cur = 0;
//  }

vertical();
}

    void rightUp(int value){
        digitalWrite(dir1,LOW);
        analogWrite(pwm1,0);
        
        digitalWrite(dir2,HIGH);
        analogWrite(pwm2,value);
        
        digitalWrite(dir3,LOW);
        analogWrite(pwm3,0);
        
        digitalWrite(dir4,HIGH);
        analogWrite(pwm4,value);
        Serial.println("Right Up");
}
void rightDown(int value){

        digitalWrite(dir1,LOW);
        analogWrite(pwm1,value);

        digitalWrite(dir2,LOW);
        analogWrite(pwm2,0);
        
        digitalWrite(dir3,LOW);
        analogWrite(pwm3,value);

        digitalWrite(dir4,LOW);
        analogWrite(pwm4,0);
        
        Serial.println("Right Down");
}
void leftUp(int value){

        digitalWrite(dir1,HIGH);
        analogWrite(pwm1,value);

        digitalWrite(dir2,LOW);
        analogWrite(pwm2,0);
        
        digitalWrite(dir3,HIGH);
        analogWrite(pwm3,value);

        digitalWrite(dir4,LOW);
        analogWrite(pwm4,0);
        Serial.println("Left Up");
}
void leftDown(int value){
        digitalWrite(dir1,LOW);
        analogWrite(pwm1,0);
        
        digitalWrite(dir2,LOW);
        analogWrite(pwm2,value);

        digitalWrite(dir3,LOW);
        analogWrite(pwm3,0);
        
        digitalWrite(dir4,LOW);
        analogWrite(pwm4,value);
        Serial.println("Left Down");
}
void forward(int value){
         digitalWrite(dir1,HIGH);
        analogWrite(pwm1,value+5);
        
        digitalWrite(dir2,HIGH);
        analogWrite(pwm2,value+5);
        
        digitalWrite(dir3,HIGH);
        analogWrite(pwm3,value);
        
        digitalWrite(dir4,HIGH);
        analogWrite(pwm4,value);

        Serial.println("Forward");
        Serial.print("PWM = ");
        Serial.println(value);
        PS4.setLed(51,255,51);
}

void reverse(int value){
         digitalWrite(dir1,LOW);
        analogWrite(pwm1,value+5);
        
       digitalWrite(dir2,LOW);
        analogWrite(pwm2,value+5);
        
       digitalWrite(dir3,LOW);
        analogWrite(pwm3,value);
        
        digitalWrite(dir4,LOW);
        analogWrite(pwm4,value);
        
        Serial.println("Reverse");
        Serial.print("PWM = ");
        Serial.println(value);
        PS4.setLed(153,51,51);
}

void left(int value){
        digitalWrite(dir1,HIGH);
        analogWrite(pwm1,value+5);
        
        digitalWrite(dir2,LOW);
        analogWrite(pwm2,value+5);
        
        digitalWrite(dir3,HIGH);
        analogWrite(pwm3,value);
        
        digitalWrite(dir4,LOW);
        analogWrite(pwm4,value);
        
        Serial.println("left");
        Serial.print("PWM = ");
        Serial.println(value);
        PS4.setLed(255,255,51);
}
void right(int value){
        digitalWrite(dir1,LOW);
        analogWrite(pwm1,value+5);
        
        digitalWrite(dir2,HIGH);
        analogWrite(pwm2,value+5);
        
        digitalWrite(dir3,LOW);
        analogWrite(pwm3,value);
        
        digitalWrite(dir4,HIGH);
        analogWrite(pwm4,value);
        
        Serial.println("Right");
        Serial.print("PWM = ");
        Serial.println(value);
         PS4.setLed(255,255,51);
}
void clockwise(int value){
        digitalWrite(dir1,LOW);
        analogWrite(pwm1,value);
        
        digitalWrite(dir2,HIGH);
        analogWrite(pwm2,value);
        
        digitalWrite(dir3,HIGH);
        analogWrite(pwm3,value);
        
        digitalWrite(dir4,LOW);
        analogWrite(pwm4,value);
        
        Serial.println("ClockWise");
        Serial.print("PWM = ");
        Serial.println(value);
        PS4.setLed(204,102,0);
}
void antiClockwise(int value){
        digitalWrite(dir1,HIGH);
        analogWrite(pwm1,value);
        
        digitalWrite(dir2,LOW);
        analogWrite(pwm2,value);
        
        digitalWrite(dir3,LOW);
        analogWrite(pwm3,value);
        
        digitalWrite(dir4,HIGH);
        analogWrite(pwm4,value);
        
        Serial.println("AntiClockwise");
        Serial.print("PWM = ");
        Serial.println(value);
         PS4.setLed(204,0,204);
}

void stopp(){
     
     analogWrite(pwm1,0);
     analogWrite(pwm2,0);
     analogWrite(pwm3,0);
     analogWrite(pwm4,0);
     
     Serial.println("Stop");
     
}

void vertical(){
//Serial.println("Vertical");

int valueRY = PS4.getAnalogHat(RightHatY);

 if(valueRY <= lowr-5){
       Mspd = map(valueRY,115 , 0 , 10 , vpwm);
      Serial.println("UP");
      digitalWrite(dir6,HIGH);
       analogWrite(pwm6,Mspd);
       PS4.setLed(51,0,51);    
      }else if(valueRY >= uppr+5){
        Mspd = map(valueRY,uppr , 255 , 10 , vpwm);
    Serial.println("Down");
    digitalWrite(dir6,LOW);
    analogWrite(pwm6,Mspd);
     PS4.setLed(32,32,32);
      }else{
//       Serial.println("VertiStop");
       digitalWrite(dir6,LOW);
       analogWrite(pwm6,0);  
      }
}
