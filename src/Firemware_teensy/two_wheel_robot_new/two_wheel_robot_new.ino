#include "header_def.h"

unsigned long pmil = 0, g_prev_command_time = 0, vac_wait = 0;
float rate = 20;

uint8_t pwml = 21;

const int inr1 = 8;
const int pwmr = 5;

const int STATE;
bool vac_on;

const int inl1 = 20;
const int L_G = 25; // Left motor
const int L_B = 26;
const int L_Y = 27;

const int R_G = 28; // Right motor
const int R_B = 29;
const int R_Y = 30;

const int left_light_pinmod = 31;
const int right_light_pinmod = 32;

const int brush = 33;
const int vacuum = 34;
const int solenoid = 35;
const int horn = 36;

int key_value = 0;

char buffer[50];
int   val_dacl = 0 , val_dacr = 0;
int xl = 0, xr = 0;
int y;
int dir_value = 2;
int cmd;
unsigned long cmil;
geometry_msgs::Twist mov;
std_msgs::Int8 dir1;


void messageCb1(const geometry_msgs::Twist& msg)
{
  dir_value = msg.linear.x;
  xl = msg.linear.y;
  xr = msg.linear.z;
  val_dacl = msg.angular.x;
  val_dacr = msg.angular.y;
  //  dacl.setVoltage(val_dacl, false);
  //  dacr.setVoltage(val_dacr, false);
  g_prev_command_time = millis();
}

void Cbkeyvalue(const std_msgs::Int32& msg)
{
  key_value = msg.data;
}

ros::Subscriber <geometry_msgs::Twist> sub_base_motor("updated_mov_cmd_topic", messageCb1 );
ros::Subscriber <std_msgs::Int32> sub_key_value("key_value_topic", Cbkeyvalue );


void forward()
{
  digitalWrite(inl1, HIGH);
  analogWrite(pwml, xl);

  digitalWrite(inr1, HIGH);
  analogWrite(pwmr, xr);

  digitalWrite(L_G, HIGH); // gets on
  digitalWrite(L_B, HIGH);
  digitalWrite(L_Y, HIGH); // gets on

  digitalWrite(R_G, HIGH);
  digitalWrite(R_B, HIGH); // gets on
  digitalWrite(R_Y, HIGH);

  digitalWrite(brush, LOW); // gets on
  digitalWrite(vacuum, LOW);
  digitalWrite(solenoid, LOW);//solenoid

  sprintf (buffer, "Forward  :%d, %d", int(xl), int(xr));
  //nh.loginfo(buffer);
}

void backward()
{
  // backward

  digitalWrite(inl1, LOW);
  analogWrite(pwml, xl);

  digitalWrite(inr1, LOW);
  analogWrite(pwmr, xr);

  digitalWrite(L_G, LOW); // gets on
  digitalWrite(L_B, LOW);
  digitalWrite(L_Y, LOW); // gets on

  digitalWrite(R_G, LOW);
  digitalWrite(R_B, LOW); // gets on
  digitalWrite(R_Y, LOW);

  digitalWrite(brush, HIGH); // gets off
  digitalWrite(vacuum, HIGH);
  digitalWrite(solenoid, HIGH);//solenoid
  sprintf (buffer, "Backward  :%d, %d", xl, xr);
  // nh.loginfo(buffer);
}

void stopbase()
{
  // stop
  dir_value = 0;
  val_dacl = 0;
  val_dacr = 0;
  digitalWrite(inl1, LOW);
  analogWrite(pwml, 0);

  digitalWrite(inr1, LOW);
  analogWrite(pwmr, 0);

  digitalWrite(L_G, HIGH); // gets on
  digitalWrite(L_B, HIGH);
  digitalWrite(L_Y, HIGH); // gets on

  digitalWrite(R_G, HIGH);
  digitalWrite(R_B, HIGH); // gets on
  digitalWrite(R_Y, HIGH);


  digitalWrite(brush, HIGH); // gets off
  digitalWrite(vacuum, HIGH);
  digitalWrite(solenoid, HIGH);//solenoid

  dacl.setVoltage(0, false);
  dacr.setVoltage(0, false);

  digitalWrite(13, HIGH - digitalRead(13)); // blink Light when robot is stop
  delay(100);
  sprintf (buffer, "Backward  :%d, %d", xl, xr);
  // nh.loginfo(buffer);
}
void left()
{
  //Left

  digitalWrite(inl1, HIGH);
  analogWrite(pwml, xl);

  digitalWrite(inr1, HIGH);
  analogWrite(pwmr, xr);

  digitalWrite(L_G, HIGH); // gets on
  digitalWrite(L_B, HIGH);
  digitalWrite(L_Y, HIGH); // gets on

  digitalWrite(R_G, HIGH);
  digitalWrite(R_B, HIGH); // gets on
  digitalWrite(R_Y, HIGH);

  digitalWrite(brush, LOW);
  digitalWrite(vacuum, LOW);
  digitalWrite(solenoid, LOW);//solenoid

  sprintf (buffer, "Left  :%d, %d", xl, xr);
  // nh.loginfo(buffer);


}

void right()
{
  //Right

  digitalWrite(inl1, HIGH);
  analogWrite(pwml, xl);

  digitalWrite(inr1, HIGH);
  analogWrite(pwmr, xr);

  digitalWrite(L_G, HIGH); // gets on
  digitalWrite(L_B, HIGH);
  digitalWrite(L_Y, HIGH); // gets on

  digitalWrite(R_G, HIGH);
  digitalWrite(R_B, HIGH); // gets on
  digitalWrite(R_Y, HIGH);

  digitalWrite(brush, LOW);
  digitalWrite(vacuum, LOW);
  digitalWrite(solenoid, LOW);//solenoid

  sprintf (buffer, "Right  :%d, %d", xl, xr);
  // nh.loginfo(buffer);

}
void setup()
{
  cmil = millis();
  nh.initNode();
  dacl.begin(0x60); //I2C address for mcp 4725
  dacr.begin(0x61); //I2C address for mcp 4725

  nh.subscribe(sub_base_motor);
  pinMode(inl1, OUTPUT);
  pinMode(pwml, OUTPUT);

  pinMode(inr1, OUTPUT);
  pinMode(pwmr, OUTPUT);

  pinMode(brush, OUTPUT);
  pinMode(vacuum, OUTPUT);
  pinMode(solenoid, OUTPUT);
  pinMode(horn, OUTPUT);

  pinMode(13, OUTPUT);

  pinMode(R_G, OUTPUT);
  pinMode(R_B, OUTPUT);
  pinMode(R_Y, OUTPUT);

  pinMode(L_G, OUTPUT);
  pinMode(L_B, OUTPUT);
  pinMode(L_Y, OUTPUT);

  digitalWrite(brush, HIGH);
  digitalWrite(vacuum, HIGH);
  digitalWrite(solenoid, HIGH);//solenoid
  digitalWrite(horn, HIGH);

  digitalWrite(L_G, HIGH); // gets on
  digitalWrite(L_B, HIGH);
  digitalWrite(L_Y, HIGH); // gets on

  digitalWrite(R_G, HIGH);
  digitalWrite(R_B, HIGH); // gets on
  digitalWrite(R_Y, HIGH);

}
void give_direction()
{
  if ((millis() - g_prev_command_time) >= 5000)
  {
    stopbase();
  }
  //  if ((cmil - pmil) >= (1000 / rate))
  if ((millis() - g_prev_command_time) >= (1000 / COMMAND_RATE))
  {
    if (dir_value == 10)
    {
      forward();
    }

    if (dir_value == -10)
    {
      backward();
    }

    if (dir_value == 20)
    {
      left();
    }

    if (dir_value == 30)
    {
      right();
    }

    if (dir_value == 0)
    {
      stopbase();
    }
  }
}

void horn_fun()
{
  if (key_value == 103)
  {
    digitalWrite(horn, HIGH);
  }

  if (key_value == 104)
  {
    digitalWrite(horn, LOW);
  }
}
void loop()
{
  give_direction();
  dacl.setVoltage(val_dacl, false);
  dacr.setVoltage(val_dacr, false);
  horn_fun();
  nh.spinOnce();
}
