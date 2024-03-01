#include <IRremote.h>
int RECV_PIN = 12;
IRrecv irrecv(RECV_PIN);
decode_results results;
#define IR_Go      0x00ff629d
#define IR_Back    0x00ffa857
#define IR_Left    0x00ff22dd
#define IR_Right   0x00ffc23d
#define IR_Stop    0x00ff02fd
#define IR_ESC     0x00ff52ad
#define Lpwm_pin  5     //adjusting speed 
#define Rpwm_pin  6    //adjusting speed //
int pinLB=2;           // defining pin2 left rear
int pinLF=4;           // defining pin4 left front
int pinRB=7;           // defining pin7 right rear
int pinRF=8;           // defining pin8 right front

void go_forward(unsigned char speed_val)    // speed_val：0~255
    {digitalWrite(pinRB,HIGH); 
     digitalWrite(pinRF,LOW);
     digitalWrite(pinLB,HIGH);
     digitalWrite(pinLF,LOW);
     analogWrite(Lpwm_pin,speed_val);
     analogWrite(Rpwm_pin,speed_val);
     
      
    }

void go_backward(unsigned char speed_val)    // speed_val：0~255
    {
     digitalWrite(pinRB,LOW);  
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,LOW);  
     digitalWrite(pinLF,HIGH);
     analogWrite(Lpwm_pin,speed_val);
     analogWrite(Rpwm_pin,speed_val);
    }
    
void rotate_left(unsigned char speed_val)        // speed_val：0~255
    {digitalWrite(pinRB,HIGH);
     digitalWrite(pinRF,LOW );  
     digitalWrite(pinLB,LOW); 
     digitalWrite(pinLF,HIGH);
     analogWrite(Lpwm_pin,speed_val);
     analogWrite(Rpwm_pin,speed_val);
      
     
    }
void rotate_right(unsigned char speed_val)    // speed_val：0~255
    {
      digitalWrite(pinRB,LOW);  
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,HIGH);
     digitalWrite(pinLF,LOW);  
     analogWrite(Lpwm_pin,speed_val);
     analogWrite(Rpwm_pin,speed_val);
     
    }    
void stopp()        //stop
    {
     digitalWrite(pinRB,HIGH);
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,HIGH);
     digitalWrite(pinLF,HIGH);
    }
         
void IR_Control(void)
{
   unsigned long Key;
   if(irrecv.decode(&results)) //judging if serial port receives data   
 {
     Key = results.value;
    switch(Key)
     {
       case IR_Go:go_forward(150);   //UP
       break;
       case IR_Back:go_backward(150);   //back
       break;
       case IR_Left:rotate_left(100);   //Left    
       break;
       case IR_Right:rotate_right(100); //Righ
       break;
       case IR_Stop:stopp();   //stop
       break;
       default: 
       break;      
     } 
     irrecv.resume(); // Receive the next value
    } 
  
}
void setup() 
{ 
   pinMode(pinLB,OUTPUT); // pin2
   pinMode(pinLF,OUTPUT); // pin4
   pinMode(pinRB,OUTPUT); // pin7 
   pinMode(pinRF,OUTPUT); // pin8
   pinMode(Lpwm_pin,OUTPUT); // pin5 (PWM) 
   pinMode(Rpwm_pin,OUTPUT); // pin6 (PWM)   
   irrecv.enableIRIn(); // Start the receiver
   Serial.begin(9600);   //initializing serial port, Bluetooth used as serial port, setting baud ratio at 9600 
   stopp();  
}
void loop() 
{  
   
   IR_Control();
    
}
