//test code for kit mouse
#include <avr/io.h>
#include <avr/interrupt.h>
//These are the assigned pins for the teensy 3.1
#define NW_EMIT 		      20
#define N_EMIT 		      19
#define NE_EMIT 		      20
#define NW_RECV 		      23
#define N_RECV 		      21
#define NE_RECV 		      22

//#define W_EMIT                19
//#define E_EMIT                19

//#define W_RECV  10
//#define E_RECV  11

//check if left is less than 125
//check if right is less than 100

//these values need to be calibrated
#define FRONT_WALL_DETECT     750
#define LEFT_WALL_STRGHT      600
#define RIGHT_WALL_STRGHT     550 //was 600

#define NW_SET              360
#define NE_SET              135
#define N_SET               285

#define DEBUG_PRINT     1

enum {
	IR_LEFT = 0,
        IR_RIGHT,
        IR_MID,
	NUM_IRLED
} number_of_IRled;

//led connected to pins 3 and 4
//const int Led1 = 20;   //this is for left and right leds
//const int Led2 = 21;	//this is for middle led
//Proportinoal control for mouse (l = left, r = right etc...)


int kp_left = 0;
int ki_left = 0;
int kd_left = 0;
int error_left = 0;
int prev_err_left = 0;
int err_sum_left = 0;
int Output_left = 0;
int Offset_left = 100;

int kp_right = 0;
int ki_right = 0;
int kd_right = 0;
int error_right = 0;
int prev_err_right = 0;
int err_sum_right = 0;
int Output_right = 0;
int Offset_right = 100;


char x_max = 5;
char y_max = 5;
char cell_array [5][5];
int no_l_wall = 100;//was 50
int no_r_wall = 70;

int l_speed = 40 *.9;//was 50
int r_speed = 37 *.9;//was 45

int l_error = 0;
int r_error = 0;
int m_error = 0;

//motor driver logic pins
const int leftPWMA = 3;
const int leftPWMB = 4;
const int rightPWMA = 5;
const int rightPWMB = 6;

//global variables
int ir_raw[3]={0, 0, 0}; //this initializes each arr elem to 0
int sides[2] = {0,0};
int led_state = 0;
int turn_flag = 0;
unsigned long timer = 0;

/*
psuedo code for speed control

unsigned long ENC = 0
unsigned long prev_ENC = 0

long dif_ENC = ENC - prev_ENC

speed_one = dif_ENC / 0.001

int set_val = 10

error_val = set_val - speed_one

speed_control = kp*error_val





*/




void switch_sensors() {
	switch(led_state) {
		case 0:
                        //ACTIONS
			ir_raw[0] = analogRead(NW_RECV);
			ir_raw[2] = analogRead(NE_RECV);

			digitalWrite(N_EMIT,HIGH);
			digitalWrite(NW_EMIT,LOW);
			digitalWrite(NE_EMIT,LOW);
                     //   digitalWrite(W_EMIT,HIGH);
                     //   digitalWrite(E_EMIT,HIGH);
			if (DEBUG_PRINT) {

				Serial.print("\nCase 0:  ");
				Serial.print("\n         ---->left recv: ");
                                Serial.print(ir_raw[0]);
                                Serial.print("<---->mid recv: ");
				Serial.print(ir_raw[1]);
                                Serial.print("---->right recv: ");
				Serial.print(ir_raw[2]);
                                Serial.print("\nWest recv: ");
                                Serial.print(sides[0]);
                                Serial.print("<--------->East recv: ");
                                Serial.print(sides[1]);
                                //delay(500);
			}
                        //TRANSIOTIONS
                        led_state = 1;
			break;
		case 1:
                        //ACTIONS
			ir_raw[1] = analogRead(N_RECV);
              //          sides[0] = analogRead(W_RECV);
              //          sides[1] = analogRead(E_RECV);
		        digitalWrite(N_EMIT,LOW);
			digitalWrite(NW_EMIT,HIGH);
			digitalWrite(NE_EMIT,HIGH);
                //        digitalWrite(W_EMIT, LOW);
                //        digitalWrite(E_EMIT,LOW);
			if (DEBUG_PRINT) {
				Serial.print("\nCase 1:  ");
                                Serial.print("\n       ---->left recv: ");
                                Serial.print(ir_raw[0]);
                                Serial.print("<---->mid recv: ");
				Serial.print(ir_raw[1]);
                                Serial.print("---->right recv: ");
				Serial.print(ir_raw[2]);
                                Serial.print("\nWest recv: ");
                                Serial.print(sides[0]);
                                Serial.print("<--------->East recv: ");
                                Serial.print(sides[1]);
                                //delay(500);
			}
                        //TRANSITIONS
                        led_state = 0;
			break;
		default: break;
	}
}
void maze(){

 char curr_distance = 1;
   for(char x = 1; x <= x_max; x++){

   }
   for(char y = 1; y <= y_max; y++){

   }

   int addr;
   if (addr == 512){
      addr = 0;
   }

}
void stopboth_motors(void) {
   analogWrite(leftPWMA,0);
   analogWrite(leftPWMB,0);
   analogWrite(rightPWMA,0);
   analogWrite(rightPWMB,0);
}
void move_forward(unsigned char left_speed, unsigned char right_speed) {
        analogWrite(leftPWMA,left_speed);
	analogWrite(leftPWMB,0);
	analogWrite(rightPWMA,right_speed);
	analogWrite(rightPWMB,0);
	//digitalWrite(Led1,LOW);
	//digitalWrite(Led2,HIGH);
}
void move_backward(unsigned char left_speed, unsigned char right_speed){
        analogWrite(leftPWMA,0);
	analogWrite(leftPWMB,left_speed);
	analogWrite(rightPWMA,0);
	analogWrite(rightPWMB,right_speed);
	//digitalWrite(Led1,LOW);
	//digitalWrite(Led2,HIGH);
}
void turn_right(unsigned char left_speed, unsigned char right_speed) {
	analogWrite(leftPWMA,left_speed);
	analogWrite(leftPWMB,0);
	analogWrite(rightPWMA,0);
	analogWrite(rightPWMB,right_speed);
        //digitalWrite(Led1,HIGH);
	//digitalWrite(Led2,LOW);
}
void turn_left(unsigned char left_speed, unsigned char right_speed) {
	analogWrite(leftPWMA,0);
	analogWrite(leftPWMB,left_speed);
	analogWrite(rightPWMA,right_speed);
	analogWrite(rightPWMB,0);
        //digitalWrite(Led1,LOW);
	//digitalWrite(Led2,HIGH);
}
void move_robot() {
        //added to get the error for each detector
        //l_error = ir_raw[0] - L_SET;
        //r_error = ir_raw[2] - R_SET;

        error_left = ir_raw[0] - NW_SET;
        err_sum_left += error_left;
        kp_left = 2 * error_left;
        ki_left = 2 * err_sum_left;
        kd_left = 2 * (error_left - prev_err_left);

        Output_left = (kp_left + ki_left + kd_left)/3;

        error_right = ir_raw[2] - NE_SET;
        err_sum_right += error_right;
        kp_right = 2 * error_right;
        ki_right = 2 * err_sum_right;
        kd_right = 2 * (error_right - prev_err_right);

        Output_right = (kp_right + ki_right +kd_right)/3;

        l_error = 16;
        r_error = 10;
  	if(ir_raw[1] < FRONT_WALL_DETECT){
            //go forward if we dont have a wall in front
            move_forward(l_speed, r_speed);
            //proportional control
            if(error_left > 30){
            //if((ir_raw[0] + Offset - ir_raw[2]) > 20){
            //if(ir_raw[0] > LEFT_WALL_STRGHT){    //too close to left
              move_forward(l_speed + Output_left, r_speed);
            }
            //else if((ir_raw[0] + Offset - ir_raw[2]) < 0){

            if(error_right > 30){
            //else if(((ir_raw[0] + Offset - ir_raw[2]) < -20)){    //too close to right
              move_forward(l_speed, r_speed + Output_right);
            }

            else{
                  //move_forward(l_speed,r_speed);      //CHANGED SPEED (LEFT, RIGHT) WAS (128,128)

                  //if(ir_raw[0] < no_l_wall && ir_raw[2] < no_r_wall  && ir_raw[1] < 150){
                  //   move_forward(l_speed, r_speed);
                  //}
                  if (ir_raw[0] < no_l_wall && ir_raw[2] > no_r_wall ){
                    Serial.print("                  GOING LEFT");
                     delay(150);
                     stopboth_motors();
                     delay(100);
                     move_forward(l_speed, r_speed);
                     delay(250);
                     //turn_left(l_speed, r_speed);
                     delay(400);
                     move_forward(l_speed, r_speed);
                     delay(350);
                  }
                  else if (ir_raw[0] > no_l_wall && ir_raw[2] < no_r_wall ){
                     Serial.print("                  GOING RIGHT");
                     delay(150);
                     stopboth_motors();
                     delay(100);
                     move_forward(l_speed, r_speed);
                     delay(250);
                     turn_right(l_speed, r_speed);
                     delay(400);
                     move_forward(l_speed, r_speed);
                     delay(350);
                  }
                //else{
                //    move_forward(l_speed, r_speed);
                //}
            }

           prev_err_left = error_left;
           prev_err_right = error_right;
	}
        else if(ir_raw[1] >= FRONT_WALL_DETECT){
                    Serial.print("                  DETECT FRONT WALL");
                    stopboth_motors();
                    delay(5000);
                    move_backward(l_speed, r_speed);
                    delay(500);
                    //delay(3000);
                    //turn_right(l_speed * 0.8, r_speed * 0.8);
                    //delay(3000);
                    //move_forward(l_speed, r_speed);
	}
}
void setup() {
  Serial1.begin(9600);
  pinMode(NW_EMIT,OUTPUT);
  pinMode(N_EMIT,OUTPUT);
  pinMode(NE_EMIT,OUTPUT);
  pinMode(NW_RECV,INPUT);
  pinMode(N_RECV,INPUT);
  pinMode(NE_RECV,INPUT);
 // pinMode(W_EMIT, OUTPUT);
 // pinMode(E_EMIT, OUTPUT);
 // pinMode(W_RECV, INPUT);
 // pinMode(E_RECV, INPUT);
  //init sensors
  digitalWrite(NW_EMIT,HIGH);
  digitalWrite(N_EMIT,LOW);
  digitalWrite(NE_EMIT,HIGH);
 // digitalWrite(W_EMIT,LOW);
 // digitalWrite(E_EMIT,LOW);
  stopboth_motors();
}
void loop() {
	timer = micros();
	Serial.print("\nswitch begin");
        //delay(500);
	switch_sensors();
	//delay(500);
        Serial.print("\nswitch ended");

	//k_pl = 5 * l_error;
      //  k_pm = 5 * m_error;
      //  k_pr = 5 * r_error;

	Serial.print("\nmove begin");
        //delay(500);
	move_robot();
	//delay(500);
        Serial.print("\nmove ended");
	while((micros()-timer)<1000) {delayMicroseconds(250);}	//was i microsecond
}
