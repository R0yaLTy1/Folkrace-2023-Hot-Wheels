#include <VL53L0X.h>
#include <Wire.h>

#define serial_baud 9200

#define AP 5 
#define AG 6 
#define BP 9 
#define BG 3 

#define drv_spd 140
#define max_spd 255
#define min_spd 80
#define avoid_spd 130

VL53L0X sens1;
VL53L0X sens2;
VL53L0X sens3;

int VLX_1; //int sukuria sveika skaiciu
int VLX_2; 
int VLX_3; 

bool VLX_1_en = true; 
bool VLX_2_en = true;
bool VLX_3_en = true; 
bool avoiding = true; 

#define xshut1 10 
#define xshut2 12 
#define xshut3 11

#define vlx_refresh_rate 18000 //18ms, defaultas 20ms
#define max_vlx_range 1200 
#define min_vlx_range 30
#define avoid_range 40 

int distance[3];

#define kP 0.38
#define kI 0.006
#define kD 0.01
float PID;
float P;
int I = 0;
int lastP = 0;
int speed;
 

int emulate_front() // emulates front sensosr if it's not found
{
  return 0;
}

void sensors();
void drive(int speed_A, int speed_B);
void avoid();
void PID_drive();

void setup() {
  Wire.begin();
  Serial.begin(serial_baud);

  pinMode(AP, OUTPUT);
  pinMode(AG, OUTPUT);
  pinMode(BP, OUTPUT);
  pinMode(BG, OUTPUT);

  pinMode(xshut1, OUTPUT);
  pinMode(xshut2, OUTPUT);
  pinMode(xshut3, OUTPUT);

  // vlx init
  digitalWrite(xshut1, LOW);
  digitalWrite(xshut2, LOW);
  digitalWrite(xshut3, LOW);
  delay(10);

  //------------------------- vlx 1 -----------------------------
  digitalWrite(xshut1, HIGH);
  delay(10);
  sens1.setAddress(0x30); // chahnges the default i2c adress

  sens1.setMeasurementTimingBudget(vlx_refresh_rate); // sets refresh rate
  delay(10); // maybe not needed
  if(sens1.init()) // checks and initialises the vlx
    VLX_1_en = true;
  else
    VLX_1_en = false;
  sens1.startContinuous(); //starts continuous data reading

  //------------------------- vlx 2 ------------------------------
  digitalWrite(xshut2, HIGH);
  delay(10); 
  sens2.setAddress(0x31);

  sens2.setMeasurementTimingBudget(vlx_refresh_rate);
  delay(10);
  if(sens2.init())
    VLX_2_en = true;
  else
    VLX_2_en = false;
  sens2.startContinuous();


  //-------------------- vlx 3 ----------------------------
  digitalWrite(xshut3, HIGH);
  delay(10); 
  sens3.setAddress(0x32);
  
  sens3.setMeasurementTimingBudget(vlx_refresh_rate);
  delay(10);
  if(sens3.init())
    VLX_3_en = true;
  else
    VLX_3_en = false;
  sens3.startContinuous();
}

void sensors()
{
  if(VLX_1_en)
    distance[0]=sens1.readRangeContinuousMillimeters();//vlx1 left
  else
    distance[0]=660;
    
  if(distance[0] <=min_vlx_range)
    distance[0]=1;    
  
  if(distance[0]>= max_vlx_range)
    distance[0]=max_vlx_range; 

  if(VLX_3_en)
    distance[2]=sens3.readRangeContinuousMillimeters();// vlx3 right
  else
   distance[2]=660;
   
  if(distance[2] <=min_vlx_range)
    distance[2]=1;    
  
  if(distance[2] >= max_vlx_range)
    distance[2] = max_vlx_range;

  if(VLX_2_en)
    distance[1]=sens2.readRangeContinuousMillimeters();//vlx2 mid
  else 
    distance[1]=660;

  if(distance[1] <=min_vlx_range)
    distance[1]=1; 

  if(distance[1] >= max_vlx_range)
    distance[1]=max_vlx_range;
}

void avoid()
{
  if(distance[1]<=avoid_range){
    while(distance[0]<distance[2])
      {
        sensors();
        drive(-avoid_spd, -0.8*avoid_spd);
        delay(50);
      }
    while(distance[0]>distance[2])
      {
        sensors();
        drive(-0.8*avoid_spd, -avoid_spd);
        delay(50);
      }
  }
  else
  return 0;
}

void drive(int speedA, int speedB){
  //constrainai
  speedA = constrain(speedA, -max_spd, max_spd);
  speedB = constrain(speedB, -max_spd, max_spd);
  //real drive
  if(speedA==0 && speedB==0)
  {
    digitalWrite(AP, HIGH);
    digitalWrite(AG, HIGH);
    digitalWrite(BP, HIGH);
    digitalWrite(BG, HIGH);
  }
  else 
  {
    if(speedA>0){
      analogWrite(AP, speedA);
      digitalWrite(AG, LOW);
    }
    else{
      digitalWrite(AP, LOW);
      analogWrite(AG, -speedA);
    }
    if(speedB > 0){
      analogWrite(BP, speedB);
      digitalWrite(BG, LOW);
    }
    else 
    {
      digitalWrite(BP, LOW);
      analogWrite(BG, -speedB);
    }
  }  
}
void loop() {
  // put your main code here, to run repeatedly:
  sensors();
  P = distance[0] - distance[2];
  I+=P;
  PID = (kP*P)+(I*kI)+((P-lastP)*kD);
  Serial.print(distance[0]);
  Serial.print("   ");
  Serial.print(distance[1]);
  Serial.print("   ");
  Serial.print(distance[2]);  
  Serial.print("   ");
  Serial.println(PID); 
  lastP = P;
  //drive(drv_spd-PID, drv_spd+PID);
  if(distance[1]<= avoid_range)
    avoid();
  else
    drive(drv_spd-PID, drv_spd+PID);
  

}
