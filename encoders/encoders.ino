
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Vector3.h>
#include <rosserial_arduino/Adc.h>

#include <avr/io.h>
#include <avr/sfr_defs.h>
#define sbi(port,bit) (port) |= (1 << (bit)) 
#define cbi(port,bit) (port) &= ~(1 << (bit)) 
// ros
ros::NodeHandle nh;
geometry_msgs::Vector3 en;

ros::Publisher encoder("encoder", &en); 


// Encoder pin 
#define en1A 2
#define en1B 4
#define en2A 3
#define en2B 5

volatile long en1Pos=0, en2Pos=0;
volatile long pen1Pos=0, pen2Pos=0;

void Int_En()
{
  pinMode(en1A, INPUT_PULLUP);
  pinMode(en1B, INPUT_PULLUP);
  pinMode(en2A, INPUT_PULLUP);
  pinMode(en2B, INPUT_PULLUP);
  attachInterrupt(0, encoder1, FALLING);               // update encoder position
  attachInterrupt(1, encoder2, FALLING); 
}
// speaker 
#define Spin 39
void bip(int n,int timeDelay)
{
  while(n-->=0){
    digitalWrite(Spin, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(timeDelay);                       // wait for a second
    digitalWrite(Spin, LOW);    // turn the LED off by making the voltage LOW
    delay(timeDelay);
  }   
}

void setup() {
  pinMode(Spin, OUTPUT); // speaker
  Int_En();
  bip(3,100);                         
  nh.initNode();
  nh.advertise(encoder);
}

void loop() {
  en.x=en1Pos;
  en.y=en2Pos;
  encoder.publish(&en);
  nh.spinOnce();
  delay(10); // delay for 10 ms
}

void encoder1()  {                                    
  if (PIND & 0b00010000){
    en2Pos--;
    pen2Pos--;              
  }
  else{
    en2Pos++;
    pen2Pos++;              
  }
}
void encoder2()  {                                     
  if (PIND & 0b00100000)    {
    en1Pos--; 
    pen1Pos--;           
  }
  else{
    en1Pos++;
    pen1Pos++;            
  }
}
