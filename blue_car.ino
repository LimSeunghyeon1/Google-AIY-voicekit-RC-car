
#define MOTOR_A_a 3     //모터A의 +출력핀은 3번핀입니다
#define MOTOR_A_b 11    //모터A의 -출력핀은 11번핀입니다
#define MOTOR_B_b 5     //모터B의 +출력핀은 5번핀입니다
#define MOTOR_B_a 6     //모터B의 -출력핀은 6번핀입니다
#define MOTOR_SPEED 255 //모터의 기준속력입니다(0~255)
#define TRIG 9 //TRIG 핀 설정(초음파 보내는 핀)
#define ECHO 8//ECHO 핀 설정 (초음파 받는 핀)
#include <Servo.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
//#define SWITCH 12 //스위치의 출력핀입니다.
const int SERVO=9;
const int POT=A1;
int cnt=0;
unsigned char m_a_spd = 0, m_b_spd = 0; //모터의 속력을 저장하는 전역변수
boolean m_a_dir = 0, m_b_dir = 0;       //모터의 방향을 결정하는 전역변수
Servo myServo;
int val=0;
int phase=0;

MPU9250 accelgyro;
I2Cdev   I2C_M;

uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;

float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

#define sample_num_mdate  5000

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

float temperature;
float pressure;
float atm;
float altitude;

unsigned long time=0;

void setup()
{
  //모터 제어 핀들을 출력으로 설정합니다.
  pinMode(MOTOR_A_a, OUTPUT);
  pinMode(MOTOR_A_b, OUTPUT);
  pinMode(MOTOR_B_a, OUTPUT);
  pinMode(MOTOR_B_b, OUTPUT);
  //pinMode(SWITCH, INPUT_PULLUP);
  pinMode(TRIG,OUTPUT);
  pinMode(ECHO,INPUT);
  myServo.attach(SERVO);
  Wire.begin();
  
  Serial.begin(9600);       //시리얼 통신 초기화
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
  delay(1000);
  Serial.println(" ");
// Mxyz_init_calibrated ();
  Serial.println("Hello!"); //터미널 작동 확인용 문자열
  Serial.println("Delivery start");
  delay(5000);
  time=millis();
}

void loop()
{
  Serial.println(millis());
  servo_control();
  //object detection code
  int dist=get_dist();
/*
  while (dist<8)
  {
    cnt=0;
    digitalWrite(MOTOR_A_a,LOW);
    digitalWrite(MOTOR_B_a,LOW);
    digitalWrite(MOTOR_A_b,LOW);
    digitalWrite(MOTOR_B_b,LOW);
    Serial.println("get dist function");
    delay(3000);
    
  }
  */
  //get_imu_data();
//    m_a_dir = 0;  //모터A 정방향
//    m_b_dir = 0;  //모터B 정방향
//    m_a_spd = MOTOR_SPEED;  //모터A의 속력값 조정
//    m_b_spd = MOTOR_SPEED;  //모터B의 속력값 조정
   
  //unsigned char bt_cmd = 0;   //명령어 저장용 문자형 변수
  
  route_A();
  /*
  if (Serial.available())   //데이터가 입력되었을 때
  {
    int bt_cmd=Serial.read(); //변수에 입력된 데이터 저장
    rc_ctrl_val(bt_cmd); 
    //입력된 데이터에 따라 모터에 입력될 변수를 조정하는 함수
  
  }*/ 
  
  motor_drive();             //모터를 구동하는 함수
}

void rc_ctrl_val(unsigned char cmd) //입력된 데이터에 따라 모터에 입력될 변수를 조정하는 함수
{
  if(cmd == 'w')  //'w'가 입력되었을 때, 전진
  {
    m_a_dir = 0;  //모터A 정방향
    m_b_dir = 0;  //모터B 정방향
    m_a_spd =200;
   //left is a
      //모터A의 속력값 조정
    m_b_spd = 235;//모터B의 속력값 조정
  }
  else if (cmd=='z')
  {
    m_a_dir = 0;  //모터A 정방향
    m_b_dir = 0;  //모터B 정방향
    m_a_spd =200;
   //left is a
      //모터A의 속력값 조정        
    m_b_spd = 235;//모터B의 속력값 조정

  }
  
  else if(cmd == 'a')  //'a'가 입력되었을 때, 제자리 좌회전
  {
    m_a_dir = 1;  //모터A 역방향
    m_b_dir = 0;  //모터B 정방향
    m_a_spd = 180;  //모터A의 속력값 조정
    m_b_spd = 180;  //모터B의 속력값 조정
  }
  else if(cmd == 'd')  //'d'가 입력되었을 때, 제자리 우회전
  {
    m_a_dir = 0;  //모터A 정방향
    m_b_dir = 1;  //모터B 역방향
    m_a_spd = MOTOR_SPEED;  //모터A의 속력값 조정
    m_b_spd = MOTOR_SPEED;  //모터B의 속력값 조정
  }
  else if(cmd == 's')  //'s'가 입력되었을 때, 후진
  {
    m_a_dir = 1;  //모터A 역방향
    m_b_dir = 1;  //모터B 역방향
    m_a_spd = MOTOR_SPEED;  //모터A의 속력값 조정
    m_b_spd = MOTOR_SPEED;  //모터B의 속력값 조정
  }
  else if(cmd == 'x')
  {
    m_a_dir = 0;  //모터A 정방향
    m_b_dir = 0;  //모터B 정방향
    m_a_spd = 0;  //모터A의 정지                  
    m_b_spd = 0;  //모터B의 정지
  }
}

void motor_drive()  //모터를 구동하는 함수
{
  if(m_a_dir == 0)
  {
    digitalWrite(MOTOR_A_a, LOW);     //모터A+ LOW
    analogWrite(MOTOR_A_b, m_a_spd);  //모터A-의 속력을 PWM 출력
  }
  else
  {
    analogWrite(MOTOR_A_a, m_a_spd);  //모터A+의 속력을 PWM 출력
    digitalWrite(MOTOR_A_b, LOW);     //모터A- LOW
  }
  if(m_b_dir == 1)
  {
    digitalWrite(MOTOR_B_a, LOW);     //모터B+ LOW
    analogWrite(MOTOR_B_b, m_b_spd);  //모터B-의 속력을 PWM 출력
  }
  else
  {
    analogWrite(MOTOR_B_a, m_b_spd);  //모터B+의 속력을 PWM 출력
    digitalWrite(MOTOR_B_b, LOW);     //모터B- LOW
  }
}
int get_dist()
{
  int distance=0;
  int volt=map(analogRead(A0),0,1023,0,5000);
  distance=(27.61/(volt-0.1696))*1000;
  Serial.print(distance);
  Serial.print("cm");
  //Serial.println("");
  delay(1000);
  return distance;
}
void servo_control()
{
  val=analogRead(POT);
  val=map(val,0,1023,0,179);
  myServo.write(val);
  delay(15);
}
void get_imu_data()
{
  getAccel_Data();
  getGyro_Data();
  getCompassDate_calibrated(); // compass data has been calibrated here
  getHeading(); //before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .
  getTiltHeading();
  Serial.println("calibration parameter: ");
  Serial.print(mx_centre);
  Serial.print(" ");
  Serial.print(my_centre);
  Serial.print(" ");
  Serial.println(mz_centre);
  Serial.println(" ");
  Serial.println("Acceleration(g) of X,Y,Z:");
  Serial.print(Axyz[0]);
  Serial.print(",");
  Serial.print(Axyz[1]);
  Serial.print(",");
  Serial.println(Axyz[2]);
  Serial.println("Gyro(degress/s) of X,Y,Z:");
  Serial.print(Gxyz[0]);
  Serial.print(",");
  Serial.print(Gxyz[1]);
  Serial.print(",");
  Serial.println(Gxyz[2]);
  Serial.println("Compass Value of X,Y,Z:");
  Serial.print(Mxyz[0]);
  Serial.print(",");
  Serial.print(Mxyz[1]);
  Serial.print(",");
  Serial.println(Mxyz[2]);
  Serial.println("The clockwise angle between the magnetic north and X-Axis:");
  Serial.print(heading);
  Serial.println(" ");
  Serial.println("The clockwise angle between the magnetic north and the projection of the positive X-Axis in the horizontal plane:");
  Serial.println(tiltheading);
  Serial.println(" ");
  Serial.println();
  delay(1000);
}
void getHeading(void)
{
heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
if (heading < 0) heading += 360;
}
void getTiltHeading(void)
{
float pitch = asin(-Axyz[0]);
float roll = asin(Axyz[1] / cos(pitch));
float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
tiltheading = 180 * atan2(yh, xh) / PI;
if (yh < 0) tiltheading += 360;
}
void Mxyz_init_calibrated ()
{
Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
Serial.print(" ");
Serial.println(F("During calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
Serial.print(" ");
Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
while (!Serial.find("ready"));
Serial.println(" ");
Serial.println("ready");
Serial.println("Sample starting......");
Serial.println("waiting ......");
get_calibration_Data ();
Serial.println(" ");
Serial.println("compass calibration parameter ");
Serial.print(mx_centre);
Serial.print(" ");
Serial.print(my_centre);
Serial.print(" ");
Serial.println(mz_centre);
Serial.println(" ");
}
void get_calibration_Data ()
{
for (int i = 0; i < sample_num_mdate; i++)
{
get_one_sample_date_mxyz();
/*
 Serial.print(mx_sample[2]);
 Serial.print(" ");
 Serial.print(my_sample[2]); //you can see the sample data here .
 Serial.print(" ");
 Serial.println(mz_sample[2]);
 */
if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];
if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];
}
mx_max = mx_sample[1];
my_max = my_sample[1];
mz_max = mz_sample[1];
mx_min = mx_sample[0];
my_min = my_sample[0];
mz_min = mz_sample[0];
mx_centre = (mx_max + mx_min) / 2;
my_centre = (my_max + my_min) / 2;
mz_centre = (mz_max + mz_min) / 2;
}
void get_one_sample_date_mxyz()
{
getCompass_Data();
mx_sample[2] = Mxyz[0];
my_sample[2] = Mxyz[1];
mz_sample[2] = Mxyz[2];
}
void getAccel_Data(void)
{
accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
Axyz[0] = (double) ax / 16384;
Axyz[1] = (double) ay / 16384;
Axyz[2] = (double) az / 16384;
}
void getGyro_Data(void)
{
accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
Gxyz[0] = (double) gx * 250 / 32768;
Gxyz[1] = (double) gy * 250 / 32768;
Gxyz[2] = (double) gz * 250 / 32768;
}
void getCompass_Data(void)
{
I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
delay(10);
I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);
mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;
Mxyz[0] = (double) mx * 1200 / 4096;
Mxyz[1] = (double) my * 1200 / 4096;
Mxyz[2] = (double) mz * 1200 / 4096;
}
void getCompassDate_calibrated ()
{
getCompass_Data();
Mxyz[0] = Mxyz[0] - mx_centre;
Mxyz[1] = Mxyz[1] - my_centre;
Mxyz[2] = Mxyz[2] - mz_centre;
}
void route_A()
{
   Serial.println("route_A start");
   unsigned char bt_cmd = 0;   //명령어 저장용 문자형 변수
  
   
   
  
   if(millis()-time<5000)
     rc_ctrl_val('w'); 
    //입력된 데이터에 따라 모터에 입력될 변수를 조정하는 함수
   else if(phase==0)
   {
     rc_ctrl_val('x');
     phase+=1;
   }
   else if (phase==1)
   {
     rc_ctrl_val('a');
     phase+=1;
   }
   else if(millis()-time<7000)
     rc_ctrl_val('w');
   else if(phase==2)
   {
     rc_ctrl_val('x');
     phase+=1;
   }
   else if (millis()-time<12000)
     rc_ctrl_val('z');
   else if(phase==3)
   {
     rc_ctrl_val('x');
     phase+=1;
   }
   else if(phase==4)
   {
     rc_ctrl_val('a');
     phase+=1;
   }
   else if(phase==5||millis()-time<16000)
   {
     rc_ctrl_val('z');
   }
   Serial.println("route_A done");
   
  
    
  
}
