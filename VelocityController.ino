//60mm WHEEL DIAMETER
#include <avr/io.h>
#include <avr/interrupt.h>


volatile unsigned long leftTick = 0;
volatile unsigned long rightTick = 0;

//velocity of each wheel in meters per millisecond
const double rightVelocity = 0;
const double leftVelocity = 0;

//current speed values
double currentrightSpeed = 0;
double currentleftSpeed = 0;

//derivative motor control variables
unsigned long prev_encRight = 0;
unsigned long prev_encLeft = 0;
unsigned long diff_encRight = 0;
unsigned long diff_encLeft = 0;
double velocity_encRight = 0;
double velocity_encLeft = 0;
double velocity_errorRight = 0;
double velocity_errorLeft = 0;
double set_velocity = 60;
double Kd_speed = .125;

////////////////////////////////////////////////// *****KProportional ONLY***********//////////////////////
//PWM values translated from velocity
double Kp_speed = .6;
int rightSpeed = 40;
int leftSpeed = 40;



const int RencoderA = 14;
const int LencoderA = 16;
const int rightPWMA = 5;
const int rightPWMB = 6;
const int leftPWMA = 3;
const int leftPWMB = 4;
int timer = 0;

void stopboth_motors(void) {
   analogWrite(leftPWMA,0);
   analogWrite(leftPWMB,0);
   analogWrite(rightPWMA,0);
   analogWrite(rightPWMB,0);
}


void move_forward(int rightSpeed,int leftSpeed){
   analogWrite(leftPWMA,rightSpeed);
   analogWrite(leftPWMB,0);
   analogWrite(rightPWMA,leftSpeed);
   analogWrite(rightPWMB,0);
}


void rightENCODER(void){
  rightTick++;
}


void leftENCODER(void){
  leftTick++;
}


void resetValues(void){
  stopboth_motors();
  rightTick = 0;
  leftTick = 0;
  prev_encRight = 0;
  prev_encLeft = 0;
}


void checkVelocity(void){
/*  if(rightTick > 520 && leftTick > 520){
    rightSpeed -= Kp_speed;
    leftSpeed -= Kp_speed;
  }
  else if(rightTick < 520 && leftTick > 520){
    rightSpeed += Kp_speed;
    leftSpeed -= Kp_speed;
  }
  else if(rightTick < 520 && leftTick < 520){
    rightSpeed += Kp_speed;
    leftSpeed +=Kp_speed;
  }
  else if(rightTick > 520 && leftTick < 520){
    rightSpeed -= Kp_speed;
    leftSpeed += Kp_speed;
  }
  else if(rightTick == 520 && leftTick > 520){
    leftSpeed -= Kp_speed;
  }
  else if(rightTick == 520 && leftTick < 520){
    leftSpeed += Kp_speed;
  }
  else if(rightTick > 520 && leftTick == 520){
    rightSpeed -= Kp_speed;
  }
  else if(rightTick < 520 && leftTick == 520){
    rightSpeed += Kp_speed;
  }
  else{
  }
*/
  diff_encRight = rightTick - prev_encRight;
  diff_encLeft = leftTick - prev_encLeft;

  Serial.print(prev_encLeft);
  Serial.print("\r\n");

  prev_encRight = rightTick;
  prev_encLeft = leftTick;

  //Serial.print(diff_encRight);
  //Serial.print("           TEST             ");
  //Serial.print(diff_encLeft);
  //Serial.print("\r\n");




  velocity_encRight = (diff_encRight/.001);
  velocity_encLeft = (diff_encLeft/.001);

  velocity_errorRight = set_velocity - velocity_encRight;
  velocity_errorLeft = set_velocity - velocity_encLeft;

  //rightSpeed = (Kd_speed*velocity_errorRight);
  //leftSpeed = (Kd_speed*velocity_errorLeft);



}



void setup(){
  Serial.begin(115200);
  pinMode(rightPWMA,OUTPUT);
  pinMode(rightPWMB,OUTPUT);
  pinMode(leftPWMA,OUTPUT);
  pinMode(leftPWMB,OUTPUT);
  pinMode(RencoderA,INPUT);
  pinMode(LencoderA,INPUT);
  attachInterrupt(14,rightENCODER,RISING);
  attachInterrupt(16,leftENCODER,RISING);

}



unsigned long count = 0;

void loop(){
  move_forward(rightSpeed,leftSpeed);
  timer = micros();


//  if((timer%1000) == 0)
//  {
//    checkVelocity();
//    resetValues();
//  }
    checkVelocity();
    if(count%1000 == 0){
    resetValues();
    }

    while((micros()-timer)<10000) {count++; delayMicroseconds(1);}



}
