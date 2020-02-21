/*
 * Team Id: 336
 * Author List:Manish Dsilva, Amogh Zare, Kimaya Desai,Pritam Mane
 * Filename: task5.c
 * Theme: Biped Patrol
 * Functions: 
 * Global Variables: 
 */ 

//#include <Arduino.h>


#include <avr/io.h>
#include <avr/interrupt.h>
#include "I2Cdev.h"
//#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#ifdef abs
#undef abs
#endif

#define abs(x) ((x)>0?(x):-(x))
#define PI 3.1415926535897932384626433832795
const int MPU = 0x68; // MPU6050 I2C address

volatile int tot_overflow;
volatile int timer1Flag = 0;

float ax, ay, az;
float gx, gy, gz;
int n = 1;

float roll,roll_prev,pitch,pitch_prev,yaw,omega;
float prev_lqr_torque,lqr_torque,torque;


//STATE VARIABLES
float x,x_dot,prev_x;
float theta,theta_dot,prev_theta;
//STATE VARIABLES

float radius = 0.065/2.0, oneRevTicks = 270.0;
#define OUTPUT_READABLE_ACCELGYRO

//float k1=40.2370,  k2= 50.6292,k3=  -2000.5476,k4=   -50.7686;
//float k1=0.2370,  k2= -0.6292,  k3=  -900.00,  k4=   -0.0;
 //1.00000     0.51360  -105.28663    -7.34284
//float k1=-1.00000, k2=5.58,k3 = -15,k4= - 80.42088;
//float k1 = 31.623,   k2= 36.828, k3= -271.756,k4 =  -33.752;
//float k1 = 100.00, k2 =   109.09, k3= -1160.62,k4 =   -317.17;
//float k1 = 31.6228,k2=    12.8456,k3=  -134.2097,k4=    -8.2903;
//float k1 = 0.67316,k2=   -3.71111,k3=  -15.26510,k4=   -1.27307;
//float k1 = 0.086452,k2 = -1.228377,k3 =  -5.584156,k4=  -0.419026;
//float k1 = 0.25660,k2 = 1.66943,k3 = -6.05484,k4 = -0.55911;
//float k1 = 0.18126,k2 = -1.69480,k3 = -6.15988,k4 = -0.56176;
//float k1 =0.18126,k2 =-1.6948,k3 =-6.1599,k4 =-0.56176;
//float k1 =0.18287,k2 =-1.6138,k3 =-4.7406,k4 =-0.52045;
//float k1 =0.25887,k2 =-1.5836,k3 =-4.6478,k4 =-0.51804;
//float k1 =0.27322,k2 =0.1746,k3 =-4.2593,k4 =-0.00151;
//float k1 =0.27322,k2 =1.1746,k3 =-4.2593,k4 =-0.40151;
//float k1 =0.26943,k2 =1.2607,k3 =-5.3147,k4 =-0.43766;
//float k1 =1,k2 =0.1129,k3 =-65,k4 =-1.5;
//float k1 =1.5478,k2 =5.5149,k3 =-50.4391,k4 =-1.628;
//float k1 =1.9288,k2 =9.1311,k3 =-62.5589,k4 =-1.8164;
//float k1 =2.1986,k2 =7.9812,k3 =-51.3101,k4 =-3.5164;
//float k1 =2.4516,k2 =6.9707,k3 =-57.7848,k4 =-4.0715;
//float k1 =1.1575,k2 =1.2863,k3 =-61.005,k4 =-3.3925;
//float k1 =0.40436,k2 =-8.3847,k3 =-25.2536,k4 =-9.148;
//float k1 =1.1414,k2 =2.3377,k3 =-83.6186,k4 =-3.6214;
//float k1 =1.1881,k2 =-0.81173,k3 =-21.6576,k4 =-2.9583;
//float k1 =0.86509,k2 =-4.1194,k3 =-19.6959,k4 =-6.2396;
//float k1 =0.31623,k2 =0.079211,k3 =-5.8701,k4 =-0.48543;
//float k1 =1,k2 =0.57943,k3 =-6.8103,k4 =-1.0958;
//float k1 =1.934,k2 =2.6039,k3 =-20.4057,k4 =-6.2427;
//float k1 =3.967,k2 =0.80976,k3 =-20.5865,k4 =-4.7516;
//float k1 =3.967,k2 =0.80976,k3 =-20.5865,k4 =-4.7516;//best
//float k1 =10.2345,k2 =6.5131,k3 =-22.4668,k4 =-4.7636;
//float k1 =67.3303,k2 =19.5773,k3 =-11.4695,k4 =-0.9475;
//float k1 =21.3849,k2 =6.5149,k3 =-9.5372,k4 =-0.86486;
//float k1 =6.7749,k2 =1.784,k3 =-8.7792,k4 =-0.83016;
//float k1 =2.1056,k2 =0.87106,k3 =-16.9699,k4 =-1.1559;
float k1 =2.0954,k2 =1.0935,k3 =-19.7222,k4 =-1.2465;

//#include <SoftwareSerial.h>
//#define rxPin 8
//#define txPin 9
//
//SoftwareSerial xbee =  SoftwareSerial(rxPin, txPin);


/*Team ID: 336
 * Team Members: Manish Dsilva, Amogh Zare, Pritam Mane, Kimaya Desai
 * 
 */
 //Macros
#define MagF            23                     // electromagnet pin
#define MagB            25                     // electromagnet pin
#define buzz_pin        22                     // Buzzer pin
#define RED_pin         51                   //LED Pin
#define Common_pin      47                     //LED Pin
#define GREEN_pin       53                     //LED Pin
#define BLUE_pin        49                     //LED Pin
#define InL1            11                      // motor pin     IN1
#define PWML            10                      // PWM motor pin ENA 
#define InL2            9                       // motor pin     IN2
#define InR1            6                       // motor pin     IN3
#define PWMR            5                       // PWM motor pin ENB
#define InR2            4                       // motor pin     IN4

//Global Variables
int received;
int analogX;
int analogY;

#define encodPinR1      2                       // encoder A pin
#define encodPinR2      3                       // encoder B pin
#define encodPinL1      18                       // encoder A pin
#define encodPinL2      19                       // encoder B pin


#define FORWARD         1                       // direction of rotation
#define BACKWARD        2                       // direction of rotation
#define RIGHTWARD       3                       // direction of rotation
#define LEFTWARD        4                       // direction of rotation

volatile int count_r = 0;                                 // right rotation counter
volatile int countInit_r;                                 // right rotation counter Initial
volatile int count_l = 0;                                 // left rotation counter
volatile int countInit_l;                                 // left rotation counter Initial
volatile int tickNumber = 0;                              // ticks(interrupts) counter
volatile boolean run_r = false;                           // right motor moves(true or false)
volatile boolean run_l = false;                           // left motor moves(true or false)

/*
 * Function Name: setup
 * Input: NONE
 * Output: initialization usin function calls
 * Logic: /*
 * motor_init(): To initialize motor pins
 * MAG_init(): To initialize Electromagnet Pins
 * LED_init(): To initialize the LED pins
 * BUZZ_init(): Buzzer initialization
 * accel_init() Accel-Gyto initialization
 * timer1_init() Timer Initialization
 * Example Call: NONE(DEFAULT)
 *
 */ 
void setup() {
 Serial.begin(9600);
 motor_init();
 MAG_init(); 
 LED_init();
 BUZZ_init();
 accel_init();
 timer1_init();
 Serial3.begin(9600);
 //pinMode(rxPin, INPUT);
 //pinMode(txPin, OUTPUT);
 //xbee.begin(19200);
 
}


/*
 * Function Name: timer1_init()
 * Input: NONE
 * Output: initialization of timer
 * Logic: /*
 * timer1_init() Timer Initialization 
 * write soecific values in register
 * Example Call:  timer1_init()
 *
 */ 
void timer1_init()
{
    // set up timer with prescaler = 8
    TCCR1B |= (1 << CS11);
    //TCNT1 = 45536; //10ms
    //TCNT1 = 51536; //7ms
    TCNT1 = 57536; //4ms
    // TCNT1H = 0xC9;
    // TCNT1L = 0x50;
    // enable overflow interrupt
    TIMSK1 |= (1 << TOIE1);
    sei();
    tot_overflow = 0;
}

/*
 * Function Name: accel_init()
 * Input: NONE
 * Output: initialization of accelerometer-gyro
 * Logic: /*
 * accel_init() Accel-Gyro Initialization 
 * read specific values from accelereometer
 * Example Call:  accel_init()
 *
 */ 
void accel_init()
{


  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  //Wire.write(0x1F);                  //register for gy;
  //Wire.write(0x05); 
  Wire.endTransmission(true);        //end the transmission
    
}


/*
 * Function Name: LED_init()
 * Input: NONE
 * Output: LEDs initialization
 * Logic: /*
 * Initializing the LED and making them high to turn th LED off
 * Example Call:  LED_init()
 *
 */ 
void LED_init(){
    pinMode(RED_pin, OUTPUT);
    pinMode(Common_pin, OUTPUT);
    pinMode(GREEN_pin, OUTPUT);
    pinMode(BLUE_pin, OUTPUT);

    digitalWrite(RED_pin, HIGH);
    digitalWrite(Common_pin, HIGH);
    digitalWrite(GREEN_pin, HIGH);
    digitalWrite(BLUE_pin, HIGH);
}

/*
 * Function Name: BUZZ_init()
 * Input: NONE
 * Output: Buzzer Initialize
 * Logic: /*
 * Initializing the Buzzer and making it high to turn th Buzzer off
 * Example Call:  BUZZ_init()
 *
 */ 
void BUZZ_init(void){
    pinMode(buzz_pin, OUTPUT);
    digitalWrite(buzz_pin, HIGH);
}

/*
 * Function Name: motor_init()
 * Input: NONE
 * Output: Motor Initialize
 * Logic: /*
 * Initializing the Motor Pins and encoder pins
 * Example Call:  motor_init()
 *
 */ 
void motor_init(void)
{
 pinMode(InR1, OUTPUT);
 pinMode(InR2, OUTPUT);
 pinMode(PWMR, OUTPUT);
 pinMode(InL1, OUTPUT);
 pinMode(InL2, OUTPUT);
 pinMode(PWML, OUTPUT);
 
 pinMode(encodPinR1, INPUT);
 pinMode(encodPinR2, INPUT);
 pinMode(encodPinL1, INPUT);
 pinMode(encodPinL2, INPUT);
 
 digitalWrite(encodPinR1, HIGH);                      // turn on pullup resistor
 digitalWrite(encodPinR2, HIGH);
 digitalWrite(encodPinL1, HIGH);                      // turn on pullup resistor
 digitalWrite(encodPinL2, HIGH);
 
 attachInterrupt(1, rencoder, RISING);               // arduino pin 3
 attachInterrupt(4, lencoder, RISING);               // arduino pin 19
}

/*
 * Function Name: moveMotor(int direction, int PWM_val, long tick)  
 * Input: (int direction, int PWM_val, long tick)  
 * Output: Motor Movement for 'tick' ticks
 * Logic: /*
 * call various motor functions based on direction
 * Example Call:  moveMotor(FORWARD,200,3*2)
 *
 */ 
void moveMotor(int direction, int PWM_val, long tick)  
{
 count_r = 0;
 count_l = 0;
 countInit_r = count_r;    // abs(count)
 countInit_l = count_l;
 tickNumber = tick;
 if(direction==FORWARD)          
 {  
  motorForward_R(PWM_val);
  motorForward_L(PWM_val);
 }
 else if(direction==BACKWARD)    
 { 
  //Serial.println("Backward");
  motorBackward_R(PWM_val);
  motorBackward_L(PWM_val);
 }
 else if(direction==RIGHTWARD)          
 {  
  motorForward_R(PWM_val);
  motorBackward_L(PWM_val);
 }
 else if(direction==LEFTWARD)          
 {  
  motorBackward_R(PWM_val);
  motorForward_L(PWM_val);
 } 
}

/*
 * Function Name: rencoder()
 * Input: NONE 
 * Output: counting right motor ticks
 * Logic: 
 * Checking if interrupt pin is high then increasing counter
  *//*
 * 
 * Example Call: default
 *
 */ 
void rencoder()  
{                                    // pulse and direction, direct port reading to save cycles              
 if(digitalRead(encodPinR1)==HIGH)   count_r ++;  //motors forward ticks++              
 else if (digitalRead(encodPinR2)==HIGH)   count_r --; //motors backward ticks--   
 if(run_r) 
   if((abs(abs(count_r)-abs(countInit_r))) >= tickNumber)      motorBrake_R();
}

/*
 * Function Name: ;lencoder()
 * Input: NONE 
 * Output: counting left motor ticks
 * Logic: 
 * Checking if interrupt pin is high then increasing counter
  *//*
 * 
 * Example Call: default
 *
 */ 
void lencoder()  
{                                    // pulse and direction, direct port reading to save cycles              
 if(digitalRead(encodPinL1)==HIGH)   count_l ++;  //motors forward ticks++              
 else if (digitalRead(encodPinL2)==HIGH)   count_l --; //motors backward ticks--   
 if(run_l) 
   if((abs(abs(count_l)-abs(countInit_l))) >= tickNumber)      motorBrake_L();
}

/*
 * Function Name: ;motorForward_L(int PWM_val)  
 * Input:(int PWM_val)  
 * Output: left motor moving forward
 * Logic: 
 * write the pins L1,L2 high accordingly with the PWM
  *//*
 * 
 * Example Call: motorForward_L(100)  
 *
 */ 
void motorForward_L(int PWM_val)  
{
    analogWrite(PWML, PWM_val);
    digitalWrite(InL1, LOW);
    digitalWrite(InL2, HIGH);
}

/*
 * Function Name: ;motorForward_R(int PWM_val)  
 * Input:(int PWM_val)  
 * Output: right motor moving forward
 * Logic: 
 * write the pins R1,R2 high accordingly with the PWM
  *//*
 * 
 * Example Call: motorForward_R(100)  
 *
 */ 
void motorForward_R(int PWM_val)  
{
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, HIGH);
}


/*
 * Function Name: ;motorBackward_L(int PWM_val)    
 * Input:(int PWM_val)  
 * Output: left motor moving backward
 * Logic: 
 * write the pins L1,L2 high accordingly with the PWM
  *//*
 * 
 * Example Call: motorBackward_L(100)  
 *
 */ 
void motorBackward_L(int PWM_val)  
{
    analogWrite(PWML, PWM_val);
    digitalWrite(InL1, HIGH);
    digitalWrite(InL2, LOW);
}


/*
 * Function Name: ;motorBackward_R(int PWM_val)    
 * Input:(int PWM_val)  
 * Output: right motor moving backward
 * Logic: 
 * write the pins R1,R2 high accordingly with the PWM
  *//*
 * 
 * Example Call: motorBackward_R(100)  
 *
 */ 
void motorBackward_R(int PWM_val)  
{
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, HIGH);
    digitalWrite(InR2, LOW);
}

/*
 * Function Name: ;motorBrake()  
 * Input:NONE
 * Output: stopping the motors
 * Logic: 
 * Writing 0 PWM to all registers
  *//*
 * 
 * Example Call: motorBrake()
 *
 */ 
void motorBrake()  {
 analogWrite(PWMR, 0);
 analogWrite(PWML, 0);
 digitalWrite(InR1, HIGH);
 digitalWrite(InR2, HIGH);
 digitalWrite(InL1, HIGH);
 digitalWrite(InL2, HIGH);
 run_r = false;
 run_l = false;
}


/*
 * Function Name: ;motorBrake_R()  
 * Input:NONE
 * Output: stopping the right motor
 * Logic: 
 * Writing 0 PWM to all Right registers
  *//*
 * 
 * Example Call: motorBrake_R()
 *
 */ 
void motorBrake_R()  
{
 analogWrite(PWMR, 0);
 digitalWrite(InR1, HIGH);
 digitalWrite(InR2, HIGH);
 run_r = false;
}

/*
 * Function Name: ;motorBrake_L()  
 * Input:NONE
 * Output: stopping the left motor
 * Logic: 
 * Writing 0 PWM to all Left registers
  *//*
 * 
 * Example Call: motorBrake_L()
 *
 */ 
void motorBrake_L()  
{
 analogWrite(PWML, 0);
 digitalWrite(InL1, HIGH);
 digitalWrite(InL2, HIGH);
 run_l = false;
}


/*
 * Function Name: ; MAG_init(void)    
 * Input:NONE  
 * Output: Magnet Initialize
 * Logic: 
 * Write the pins of electrmagnet as output and low
  *//*
 * 
 * Example Call: MAG_init() 
 *
 */ 
void MAG_init(void)
{
    pinMode(MagF, OUTPUT);
    digitalWrite(MagF, LOW);
    pinMode(MagB, OUTPUT);
    digitalWrite(MagB, LOW);
 
}

/*
 * Function Name: ; MagPick(void)     
 * Input:NONE  
 * Output: Magnet High for pickup
 * Logic: 
 * Write the pins of electrmagnet as high
  *//*
 * 
 * Example Call: MagPick() 
 *
 */ 
void MagPick(void)  
{
    digitalWrite(MagF, HIGH);
    digitalWrite(MagB, HIGH);
}

/*
 * Function Name: ; MagDrop(void)      
 * Input:NONE  
 * Output: Magnet Low for drop
 * Logic: 
 * Write the pins of electrmagnet as low
  *//*
 * 
 * Example Call: MagDrop() 
 *
 */ 
void MagDrop(void)  
{
    digitalWrite(MagF, LOW);
    digitalWrite(MagB, LOW);
}


/*
 * Function Name: ; comp_filter_roll     
 * Input:(float ax,float ay,float az,float gx,float gy,float gz)
 * Output: Calculating the Roll using complementary filte
 * Logic: 
 * Apply the formula and get roll
  *//*
 * 
 * Example Call: comp_filter_roll(ax,ay,az,gx,gy,gz)
 *
 */ 
void comp_filter_roll(float ax,float ay,float az,float gx,float gy,float gz)
{
  float alpha = 0.004;
  float dt = 0.004;

  if (n==1)
  {
    roll = (1-alpha)*((-1)*gy*dt) + alpha*(atan2(ax,abs(az))*180/PI);
  }
  else
  {
    roll = (1-alpha)*(roll_prev - (gy*dt)) + alpha*(atan2(ax,abs(az))*180/PI);
  }
  roll_prev=roll;  
}

/*
 * Function Name: ; motorControl   
 * Input:         (int torque)
 * Output:        write the specific pwm value and moves  
 * Logic:         
 * based on the pwm values motor is controlled
  *//*
 * 
 * Example Call: motorControl(lqr_torque)
 *
 */ 
void motorControl(int torque)
{
 //torque between 0-255
 if (torque >= 0) 
{ // drive motors forward

   torque = abs(torque);
   if(torque<60)
      torque = 60;
   motorForward_R(torque); 
   motorForward_L(torque); 
}
 else
{ 
   // drive motors backward
  
   torque = abs(torque);
   if(torque<60)
      torque = 60;
   motorBackward_R(torque); 
   motorBackward_L(torque);
 }
}

/*
 * Function Name: ; ISR  
 * Input:         TIMER1_OVF_vect
 * Output:        makes timer1Flag=1
 * Logic:         
 * based on the the times overflow after designated time
  *//*
 * 
 * Example Call: Default
 *
 */ 
ISR(TIMER1_OVF_vect)
{
  //TCNT1 = 45536;  //10ms
  //TCNT1 = 51536; //7ms
  TCNT1 = 57536; //4ms
  //TCNT1H = 0xC9;
  //TCNT1L = 0x50;
  timer1Flag=1;
  //tot_overflow++;
}

/*
 * Function Name: ; readTiltAngle  
 * Input:         NONE
 * Output:        roll after complementary filter
 * Logic:         Calls   accelGyro() and comp_filter_roll(ax,ay,az,gx,gy,gz);
 * 
 * Example Call: readTiltAngle()
 *
 */ 

void  readTiltAngle()
{ 
  //MPU
  accelGyro();
  comp_filter_roll(ax,ay,az,gx,gy,gz);
  //MPU ENDS
}

/*
 * Function Name: ; lqrControl 
 * Input:         NONE
 * Output:        PWM value for motors for balancing
 * Logic:         LQR implemented in Arduino 
 * 
 * Example Call: lqrControl()
 *
 */ 
void lqrControl()
{
  //STATE VARIABLES
  theta = (roll); 
  //Serial.println(theta);
  theta_dot = (theta-prev_theta);
  
  if(n%10 == 0)
  {
      x = ((count_r)*2*PI*radius)/270.0; //displacement
      x_dot = (x - prev_x);
      countInit_r = count_r;
  }
  //STATE VARIABLES ENDS
  
  lqr_torque =  ((x*k1)+(x_dot*k2)+(theta*k3/57.0)+(theta_dot*k4/57.0))*(255.0/12.0);
  lqr_torque = 14.400921*lqr_torque; 
  lqr_torque = constrain(lqr_torque, -200, 200); 
  
  //Serial.print(count_r);Serial.print("\t");Serial.print(x_dot*k2);Serial.print("\t");Serial.print(theta*k3/57.0);Serial.print("\t");Serial.println(-theta_dot*k4/57.0);
  
  //PREVIOUS STATE VARIABLES
  prev_theta = theta;
  prev_x = x;
  //PREVIOUS STATE VARIABLES ENDS
  

  motorControl(lqr_torque);
}



/*
 * Function Name: ; zigbeeControl 
 * Input:         NONE
 * Output:        Control by Remote
 * Logic:         Bits are checked which is received
 * 
 * Example Call: zigbeeControl()
 *
 */ 
void zigbeeControl()
{
  
  //Serial.println();
  
  //Accept only if characters are 18 or more
  
  if(Serial3.available()>=18)                                
  {
    //0x7E is the first byte of Xbee frame
    //Accept data starting with 0X7E
    if(Serial3.read()==0x7E)
    {
      
      //Ignore the first 10 bytes of data
      for(int i=1 ; i < 11 ; i++)
      {
        byte discardByte = Serial3.read();    
      }

      //Accept the data from the 11th byte to 16th byte
      byte digitalMSB = Serial3.read();
      byte digitalLSB = Serial3.read();
      byte analogMSB1 = Serial3.read();
      byte analogLSB1 = Serial3.read();
      byte analogMSB2 = Serial3.read();
      byte analogLSB2 = Serial3.read();

      //Combining LSB and MSB by left shifting the MSB and adding it to the LSB
      int analogX = analogLSB1+ analogMSB1*256;
      int analogY = analogLSB2+ analogMSB2*256;
      

      int else_flag = 0;

      //Turn On the LED if the digitalMSB value is 0x01
      if(digitalMSB == 0x01 && digitalLSB == 0x10) //led switch
      {
        digitalWrite(RED_pin, LOW);
        digitalWrite(GREEN_pin, HIGH); 
        digitalWrite(BLUE_pin, HIGH); 
        digitalWrite(MagF, LOW);
        digitalWrite(MagB, LOW);
        digitalWrite(buzz_pin, LOW);
        else_flag = 1;
      }
  
      //Turn On the MagF if the digitalLSB value is 0x04
      else if(digitalMSB == 0x00 && digitalLSB == 0x14) //magnet switch
      {
        digitalWrite(MagF, HIGH);
        digitalWrite(MagB, LOW);
        digitalWrite(RED_pin, HIGH);
        digitalWrite(GREEN_pin, HIGH);
        digitalWrite(BLUE_pin, HIGH);
        else_flag = 1;
      }
  
      //Turn On the MagB if the digitalLSB value is 0x08
      else if(digitalMSB == 0x00 && digitalLSB == 0x18) //buzzer switch
      {
        digitalWrite(buzz_pin, HIGH);
        digitalWrite(MagF, LOW);
        digitalWrite(MagB, HIGH);
        digitalWrite(RED_pin, HIGH);
        digitalWrite(GREEN_pin, HIGH);
        digitalWrite(BLUE_pin, HIGH);
        else_flag = 1;
      }
  
      //Turn On the MagnetF and MagnetB if the digitalLSB value is 0x0C
      else if(digitalMSB == 0x00 && digitalLSB == 0x1C) //magnetF and magnetB both
      {
        digitalWrite(MagF, HIGH);
        digitalWrite(MagB, HIGH);
        digitalWrite(buzz_pin, HIGH);
        digitalWrite(RED_pin, HIGH);
        digitalWrite(GREEN_pin, HIGH);
        digitalWrite(BLUE_pin, HIGH);
        else_flag = 1;
      }

      else if(digitalMSB == 0x00 && digitalLSB == 0x00)
      {        
        digitalWrite(RED_pin, HIGH);
        digitalWrite(GREEN_pin, LOW); 
        digitalWrite(BLUE_pin, HIGH); 
        digitalWrite(MagF, LOW);
        digitalWrite(MagB, LOW);
        digitalWrite(buzz_pin, LOW);
        else_flag = 1;
      }
      
      //Turn off everything
      else if(else_flag == 0)
      {
        digitalWrite(RED_pin, HIGH);
        digitalWrite(GREEN_pin, HIGH);
        digitalWrite(BLUE_pin, HIGH);    
        digitalWrite(MagF, LOW);
        digitalWrite(MagB, LOW);
        digitalWrite(buzz_pin, HIGH);
      }
      
      //No Motion
      //Serial.print(analogX);Serial.print("\t");Serial.println(analogY);
      if((analogX > 1023) || (analogY > 1023 ))
        int a;
  
      //Both Wheels Forward
      else if((analogX > 900)  && (400 < analogY) && (analogY <800 ))
      {
        //Serial.println("Both Wheels Forward");
        moveMotor(FORWARD,  100, 3*2);
      }
  
      //Both Wheels Backward
      else if((analogX > 900) && (analogY < 300) )
      {
        //Serial.print("LSB ");
        //Serial.println(digitalLSB);
        //Serial.println(digitalMSB);
        moveMotor(BACKWARD,  100, 3*2);
      }
  
      //Left Wheel Backward, Right Wheel Forward
      else if((analogX < 300) && (analogY > 900))
      {
        //Serial.println("left wheel Backward, right wheel Forward");
        moveMotor(RIGHTWARD,  100, 3*2);
      }
  
      //Left Wheel Forward, Right Wheel Backward
      
      else if((400< analogX)&&(analogX <800 )&& (analogY > 900) )
      {
        //Serial.println("left wheel Forward, right wheel Backward");
        moveMotor(LEFTWARD,  100, 3*2);
      }

      else
      {
        motorBrake();
      }
    
    } 
  } 
}

/*
 * Function Name: ; accelGyro
 * Input:         NONE
 * Output:        Read registers from Accel-Gyro
 * Logic:         Using wire library to read registers
 * 
 * Example Call:  accelGyro()
 *
 */ 
void accelGyro()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  ax = (Wire.read() << 8 | Wire.read())/ 16384.0; // X-axis value
  ay = (Wire.read() << 8 | Wire.read())/ 16384.0; // Y-axis value
  az = (Wire.read() << 8 | Wire.read())/ 16384.0; // Z-axis value

  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); 

  gx = (Wire.read() << 8 | Wire.read()) ; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  gy = (Wire.read() << 8 | Wire.read())/131.0 ;
  gz = (Wire.read() << 8 | Wire.read()) ;
  
  gy-=5.35;
  ax=ax-0.03;
  ay=ay+0.005+0.02;
  az=az-0.08+0.02;
  
}

/*
 * Function Name: ;loop
 * Input:         NONE
 * Output:        Function Calls
 * Logic:         calls the required functions when timer has overflown
 * 
 * Example Call:  default(NONE)
 *
 */ 
int prev_time = millis();
void loop()
{

  if(timer1Flag==1)
  {
    readTiltAngle(); 
    lqrControl();
    zigbeeControl();
    timer1Flag=0;
  }
 // zigbeeControl();
//  if((millis()-prev_time)>100)
//  {
//    prev_time = millis();
//    zigbeeControl();
//  }
  //delay(1);
  n++;
  
}
