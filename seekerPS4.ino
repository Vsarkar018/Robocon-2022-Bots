#include <PS4BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the PS4BT class in two ways */
// This will start an inquiry and then pair with the PS4 controller - you only have to do this once
// You will need to hold down the PS and Share button at the same time, the PS4 controller will then start to blink rapidly indicating that it is in pairing mode
PS4BT PS4(&Btd, PAIR);

// After that you can simply create the instance like so and then press the PS button on the device
//PS4BT PS4(&Btd);

bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;

int pwm1 = 3 , dir1 = 22 ;
int pwm2 = 4 , dir2 = 5 ;
int pwm5= 6, dir5 = 7 ;
int pwm4 = 8, dir4 = 24;
int pwm3 = 9 , dir3 = 26 ;
int pwm6 = 11, dir6= 12 ;
int in1 = 25 , in2 = 27, in3 = 29, in4 = 31;

int trig1 = 48, echo1 = 49 ;
int trig2 = 46, echo2 = 44;

int vout1 = 20, vout2 = 21;
int vpwm = 140, rpm = 200, Mspd = 0,rot = 200 , ang = 40;
int flagG = 0 , flagT = 0 , flagBG = 0 , flagF = 0 ;
int uppr = 134, lowr = 116 ;

int pos = 0 , cur ;

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
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(vout1,INPUT);
  pinMode(vout2,INPUT);

  attachInterrupt(digitalPinToInterrupt(vout1),readEncoder,RISING);

  digitalWrite(in1,HIGH);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,HIGH);

  digitalWrite(13,HIGH);
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

    if(valueLY < lowr  && valueLX < uppr+20 && valueLX > lowr-20){
    int rpmm = map(valueLY,lowr,0,0,rpm);
    forward(rpmm);
  }
    else if(valueLY > uppr  && valueLX < uppr+20 && valueLX > lowr-20){
      int rpmm = map(valueLY,uppr,255,0,rpm);
    reverse(rpmm);
  }
   else if(valueLX < lowr  && valueLY < uppr+20 && valueLY > lowr-20){
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
   
   seekerMechanism();
  }
}

    void digonal(int rpmm){
      PS4.setLed(160,160,160);
      Serial.println("Diagonal");
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
}
}

  void seekerMechanism(){
//  Serial.println("SeekerFunction");
  if(PS4.getButtonPress(R1)){
    rpm = 15;
    rot = 15;
  PS4.setLed(253,106,2);
    
  }
  else{
    rpm = 50;
    rot = 30;
  }
  if(PS4.getButtonClick(CIRCLE)){
    
    if(flagG == 0){
      Serial.println("Grabbing");
      digitalWrite(in1,HIGH);
      flagG = 1;
    }else if(flagG == 1){
      Serial.println("Drop");
      digitalWrite(in1,LOW);
      flagG = 0;
    }
  }
if(PS4.getButtonPress(CIRCLE)){
  PS4.setLed(51,51,0);
}

  
  if(PS4.getButtonClick(CROSS)){  
   if(flagT == 0){
      Serial.println("THROW");
      digitalWrite(in2,HIGH);
      flagT = 1;
 
    }else if(flagT == 1){ 
      Serial.println("RELEASE");
      digitalWrite(in2,LOW);
      flagT = 0;

    }
  }   
  if(PS4.getButtonPress(CROSS)){
     PS4.setLed(51,0,51);
  }


  
  if(PS4.getButtonClick(SQUARE)){
    
     if(flagBG == 0){
      Serial.println("Close");
      digitalWrite(in3,HIGH);
      flagBG = 1;
    }else if(flagBG == 1){
      Serial.println("Open");
      digitalWrite(in3,LOW);
      flagBG = 0;
    }
  }
  if(PS4.getButtonPress(SQUARE)){
    PS4.setLed(204,0,204);
  }
     
//
//    if(ps2x.ButtonPressed(PSB_R1)){
//     cur = 2000;
//     
//  }
//  else if(ps2x.ButtonPressed(PSB_R2)){
//    cur = 0;
//    
//  }
//  else{
//    digitalWrite(dir5,LOW);
//    analogWrite(pwm5,0);
//  }
//
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
        analogWrite(pwm4,value+8);
        Serial.println("Right Up");
}
void rightDown(int value){

        digitalWrite(dir1,LOW);
        analogWrite(pwm1,value+10);

        digitalWrite(dir2,LOW);
        analogWrite(pwm2,0);
        
        digitalWrite(dir2,LOW);
        analogWrite(pwm2,value);

        digitalWrite(dir4,LOW);
        analogWrite(pwm4,0);
        
        Serial.println("Right Down");
}
void leftUp(int value){

        digitalWrite(dir1,HIGH);
        analogWrite(pwm1,value+10);

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
        analogWrite(pwm4,value+8);
        Serial.println("Left Down");
}
void forward(int value){
         digitalWrite(dir1,HIGH);
        analogWrite(pwm1,value);
        
        digitalWrite(dir2,HIGH);
        analogWrite(pwm2,value);
        
        digitalWrite(dir3,HIGH);
        analogWrite(pwm3,value+20);
        
        digitalWrite(dir4,HIGH);
        analogWrite(pwm4,value);

        Serial.println("Forward");
        Serial.print("PWM = ");
        Serial.println(value);

       PS4.setLed(51,255,51);
        }

void reverse(int value){
         digitalWrite(dir1,LOW);
        analogWrite(pwm1,value);
        
       digitalWrite(dir2,LOW);
        analogWrite(pwm2,value);
        
       digitalWrite(dir3,LOW);
        analogWrite(pwm3,value+20);
        
        digitalWrite(dir4,LOW);
        analogWrite(pwm4,value);
        
        Serial.println("Reverse");
        Serial.print("PWM = ");
        Serial.println(value); 
        PS4.setLed(153,51,51);
}

void left(int value){
        digitalWrite(dir1,HIGH);
        analogWrite(pwm1,value);
        
        digitalWrite(dir2,LOW);
        analogWrite(pwm2,value);
        
        digitalWrite(dir3,HIGH);
        analogWrite(pwm3,value+20);
        
        digitalWrite(dir4,LOW);
        analogWrite(pwm4,value);
        
        Serial.println("left");
        Serial.print("PWM = ");
        Serial.println(value);
        PS4.setLed(255,255,51);
}
void right(int value){
        digitalWrite(dir1,LOW);
        analogWrite(pwm1,value);
        
        digitalWrite(dir2,HIGH);
        analogWrite(pwm2,value);
        
        digitalWrite(dir3,LOW);
        analogWrite(pwm3,value+20);
        
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
//     PS4.setLed(255,0,0);     
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
