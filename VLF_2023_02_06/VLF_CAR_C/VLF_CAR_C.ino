#define AOpin  A0     // Analog output - yellow
#define SIpin  22     // Start Integration - orange
#define CLKpin 23     // Clock - red
// Vcc - brown
// GND - black
#include<NewPing.h>
#define NPIXELS 128  // No. of pixels in array

byte Pixel[NPIXELS]; // Field for measured values <0-255>

int LineSensor_Data[NPIXELS];           // line sensor data(original)
int LineSensor_Data_Adaption[NPIXELS];  // line sensor data(modified)
int MAX_LineSensor_Data[NPIXELS];       // Max value of sensor
int MIN_LineSensor_Data[NPIXELS];       // Min value of sensor
int flag_line_adapation;          // flag to check line sensor adpation

#define FASTADC 1
// defines for setting and clearing register bits
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

/////////////////////  Ultrasonic Sensor ///////////////////

#define MOTOR_DIR 4
#define MOTOR_PWM 5

/////////////////////  Steering Servo Control ///////////////////
#include <Servo.h>
#define RC_SERVO_PIN 8
#define NETURAL_ANGLE 115
#define LEFT_STEER_ANGLE -38
#define RIGHT_STEER_ANGLE 38
#define L_Sonar_TRIG 48 
#define L_Sonar_ECHO 49   
#define F_Sonar_TRIG 50 
#define F_Sonar_ECHO 51
#define R_Sonar_TRIG 52 
#define R_Sonar_ECHO 53 
#define MaxDistance  350

int flag = 0;
void steering_control(int steer_angle);

 NewPing R_sensor(R_Sonar_TRIG,R_Sonar_ECHO,MaxDistance);
  float R_Sonar_Error = 0.0;
  float R_Sonar_distance = 0.0;
  float R_Sonar_distance_old = 0.0;

  NewPing L_sensor(L_Sonar_TRIG,L_Sonar_ECHO,MaxDistance);
  float L_Sonar_Error = 0.0;
  float L_Sonar_distance = 0.0;
  float L_Sonar_distance_old = 0.0;

  NewPing F_sensor(F_Sonar_TRIG,F_Sonar_ECHO,MaxDistance);
  float F_Sonar_Error = 0.0;
  float F_Sonar_distance = 0.0;
  float F_Sonar_distance_old = 0.0;

  void read_sonar_sensor(void)   //초음파센서 측정
  {
    R_Sonar_distance = R_sensor.ping_cm()*10.0;
    L_Sonar_distance = L_sensor.ping_cm()*10.0;
    F_Sonar_distance = F_sensor.ping_cm()*10.0;
    if(R_Sonar_distance == 0){R_Sonar_distance = MaxDistance * 10.0;}
    if(L_Sonar_distance == 0){L_Sonar_distance = MaxDistance * 10.0;}
    if(F_Sonar_distance == 0){F_Sonar_distance = MaxDistance * 10.0;}
  }

    void update_sonar_old(void)    //초음파 센서의 옛날값 저장
  {
    R_Sonar_distance_old = R_Sonar_distance;
    L_Sonar_distance_old = L_Sonar_distance;
    F_Sonar_distance_old = F_Sonar_distance;
  }
   void update_sonar_error(void)   //초음파 센서의 옛날값과 현재값의 차이 저장
  {
    R_Sonar_Error = R_Sonar_distance - R_Sonar_distance_old;
    L_Sonar_Error = L_Sonar_distance - L_Sonar_distance_old;
    F_Sonar_Error = F_Sonar_distance - F_Sonar_distance_old;
  }




Servo Steeringservo;

int Steering_Angle = NETURAL_ANGLE;

void setup() {
  // put your setup code here, to run once:
  int i;
 
  for (i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data[i] = 0;
    LineSensor_Data_Adaption[i] = 0;
    MAX_LineSensor_Data[i] = 1023; //0;
    MIN_LineSensor_Data[i] = 0; //1023;
  }

  pinMode(SIpin, OUTPUT);
  pinMode(CLKpin, OUTPUT);

  digitalWrite(SIpin, LOW);   // IDLE state
  digitalWrite(CLKpin, LOW);  // IDLE state

#if FASTADC
  // set prescale to 16
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif

  flag_line_adapation = 0;
  Steeringservo.attach(RC_SERVO_PIN);
  Steeringservo.write(NETURAL_ANGLE);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT); 

  Serial.begin(115200);

}
void line_adaptation(void)
{
  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    if (LineSensor_Data[i] >= MAX_LineSensor_Data[i])  MAX_LineSensor_Data[i] = LineSensor_Data[i];
    if (LineSensor_Data[i] <= MIN_LineSensor_Data[i])  MIN_LineSensor_Data[i] = LineSensor_Data[i];
  }

  /*for (i = 0; i < NPIXELS; i++)
    {
    Serial.print("[");
    Serial.print(i);
    Serial.print("]");
    Serial.print("   : ");
    Serial.print(MAX_LineSensor_Data[i]);
    Serial.print(" | ");
    Serial.print(MIN_LineSensor_Data[i]);
    Serial.println(" ");
    }*/
}
void read_line_sensor(void)
{
  int i;

  delayMicroseconds (1);  /* Integration time in microseconds */
  delay(10);              /* Integration time in miliseconds  */

  digitalWrite (CLKpin, LOW);
  digitalWrite (SIpin, HIGH);
  digitalWrite (CLKpin, HIGH);
  digitalWrite (SIpin, LOW);
  
  delayMicroseconds (1);

  for (i = 0; i < NPIXELS; i++) {
    Pixel[i] = analogRead (AOpin) / 4 ; // 8-bit is enough
    digitalWrite (CLKpin, LOW);
    delayMicroseconds (1);
    digitalWrite (CLKpin, HIGH);
  }

  for (i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data_Adaption[i] = map(Pixel[i], MIN_LineSensor_Data[i], MAX_LineSensor_Data[i], 0, 256);
  }

}

#define threshold_value 60

void threshold(void)
{
  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    if((byte)Pixel[i] >= threshold_value)  LineSensor_Data_Adaption[i] = 255;
    else LineSensor_Data_Adaption[i] = 0;
  }
}
int steer_data =0;
#define camera_pixel_offset 5
void steering_by_camera(void)
{
  int i;
  long sum= 0;
  long x_sum= 0;
  //int steer_data =0;
  for (i = 0; i < NPIXELS; i++)
  {
    sum +=LineSensor_Data_Adaption[i];
    x_sum += LineSensor_Data_Adaption[i] * i;
  }
  steer_data = x_sum/sum - NPIXELS/2 + camera_pixel_offset;
  steering_control(steer_data*1.25);        //마음대로 부호가 반대면 음수 적용

  Serial.println(steer_data);
}

/////////////////////////   DC_Motor_Control    //////////////////////////
#define MOTOR_DIR 4
#define MOTOR_PWM 5

int Motor_Speed =0;
#define NORMAL_SPEED 100
#define SLOW_SPEED    70


void motor_control(int direction, int speed)
{
  digitalWrite(MOTOR_DIR, 1-direction);
  analogWrite(MOTOR_PWM, speed);

}
/////////////////////////   DC_Motor_Control     //////////////////////////


///////////////////////   Steering Servo Control   ///////////////////////


//Servo Steeringservo; 
//int Steering_Angle = NEUTRAL_ANGLE;

void steering_control(int steer_angle)
{
  //Steeringservo.write(NEUTRAL_ANGLE + steer_angle);

  if(steer_angle >= RIGHT_STEER_ANGLE) steer_angle = RIGHT_STEER_ANGLE;
  if(steer_angle <= LEFT_STEER_ANGLE) steer_angle = LEFT_STEER_ANGLE;
  Steeringservo.write(NETURAL_ANGLE + steer_angle);
}
///////////////////////   Steering Servo Control   ///////////////////////


void wall_following2() // 벽 두개 
  {
 
    int A=1170, B=100;
  read_sonar_sensor();
  
  if(R_Sonar_distance + L_Sonar_distance <= A && R_Sonar_distance+B >= L_Sonar_distance && R_Sonar_distance-B <= L_Sonar_distance)
  {
    motor_control(-1,110);
    steering_control(14);
    }

  else if(L_Sonar_distance > R_Sonar_distance)  
  {
    motor_control(-1,110);
    steering_control(20);
           // delay(5);
  }
 else if(L_Sonar_distance < R_Sonar_distance)
  {
    motor_control(-1,110);
    steering_control(8);
           // delay(5);
  }
  else
  {
    motor_control(-1,110);
           // delay(5);
  }
  }
  void wall_following1()
  {
       read_sonar_sensor();
      // update_sonar_old();
       update_sonar_error();
   // L_Sonar_distance = L_Sonar[1].ping_cm() *10;
    
       motor_control(-1,100);
       delay(5);
       update_sonar_old();
       
       if(L_Sonar_Error > 0)
       {
         steering_control(20);
         delay(5);
         update_sonar_old();
         
       }
       else if(L_Sonar_Error < 0)
       {
         steering_control(8);
         delay(5);
         update_sonar_old();
       }

    }
    void wall_following()
    {
      if(R_Sonar_distance == 3500)
      {
       wall_following1();
      }
      else
      {
        wall_following2();
      }
    }
  int turn_right()
  {
    if(F_Sonar_distance<1000 && F_Sonar_distance != 3500)
    {
      Steeringservo.write(180);
      motor_control(-1,130);
      delay(700);
    }
     return 1;
  }    
 int count = 0;
 void turn_corner()
{
  int d = 16;
    
  read_sonar_sensor();
  if(R_Sonar_distance+L_Sonar_distance <= 1170)
  {
    wall_following2();
    delay(100);
  }

  else if(R_Sonar_distance+L_Sonar_distance >= 1170)
  {
    steering_control(d);
    if(F_Sonar_distance<1000)
  {
    Steeringservo.write(180);
    motor_control(-1,130);
    delay(100);
  }
  delay(100);
  count++;
  }
  else motor_control(-1,100);
  delay(100);   
 
}

int what=0;
int the=0;
int timer=0;
void loop() {
  // put your main code here, to run repeatedly:

  int i;
 
  line_adaptation();
  read_line_sensor();
  read_sonar_sensor(); 
  //read_sonar_sensor(); 
  //motor_control(-1,100);
  //steering_control(110);
  
  //delay(100);
  //turn_corner();
  //wall_following2();
 //delay(10);
 // if (digitalRead(CLKpin) == HIGH)
  //{
    //line_adaptation
   // flag_line_adapation = 1;
 // }
  //threshold();
  //steering_by_camera();
  //delay(100);
   //motor_control(-1,0); 
  //steering_control(steer_data*1.3); 
  /*for (i = 0; i < NPIXELS; i++)
  {
  //  if (digitalRead(CLKpin) == LOW)      Serial.print(LineSensor_Data_Adaption[i]); // Serial.print(LineSensor_Data[i] );
   // else                                 Serial.print ((byte)Pixel[i] + 1);
  //Serial.print ((byte)Pixel[i] + 1);
    Serial.print(LineSensor_Data_Adaption[i]);
    Serial.print(" ");
  }
  Serial.println("  ");
  delay(100);*/
  //wall_following1();
  /*Serial.print(L_Sonar_distance);
  Serial.print(" ");
  Serial.print(R_Sonar_distance);
  Serial.println(" "); */ 
  //delay(1000);
   
  
  if(flag == 0)
  {
    motor_control(-1,80);
    threshold();
    steering_by_camera();
    delay(100);
    if(L_Sonar_distance+R_Sonar_distance <= 1300){flag=1;}
  }
  else if(flag == 1){
   motor_control(-1,130);
   steering_control(14);
   if(F_Sonar_distance<=1000){
     what++;
   }
   else{what=0;}
  if(what>=6)
  {
    turn_right();
    delay(400);
    flag=2;
    the=millis();
    what=0;
  }
    }
 else if(flag == 2)
 {
   wall_following2();
   if(F_Sonar_distance<=900 && millis()-the>=5000)
  {
    what++;
  }
  else{what=0;}

  if(what>=6)    
  {  turn_right();
    delay(400);
    flag=3;   
  }
 }
 else if(flag == 3)
 {
    motor_control(-1,80);
    threshold();
    steering_by_camera();
    delay(100);
    if(F_Sonar_distance<=400){flag=4;}  
   }
       
 else if(flag == 4) 
 {
    motor_control(0,0);
 }
}