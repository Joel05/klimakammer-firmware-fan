#include "Arduino.h"
#include "Wire.h"
#include <RPi_Pico_TimerInterrupt.h>

const char I2C_ADDR = 0x70; //Set to desired i2c-adress
#undef DEBUG    //Define for various debug outputs (#undef to disable) - !!!ENABLING SLOWS DOWN CODE SIGNIFICANTLY!!!

#define Fan1 0x12
#define Fan2 0x13
#define Fans 0x14
char module = 0x00;  //Variable to store the module that is being called


#define ReglerTimeMS 1000
#define DebounceTimeUS 5000
#define Fan1TachoPin 18
#define Fan2TachoPin 19
#define Fan1PWMPin 21
#define Fan2PWMPin 26

RPI_PICO_Timer ITimer(1); //Instantiate Timer 1

volatile unsigned long fan1rpm = 0;  //Variable to store the rpm of the first fan
volatile unsigned long fan2rpm = 0;  //Variable to store the rpm of the second fan
volatile unsigned long fan1targetrpm = 0;  //Variable to store the target rpm of the first fan
volatile unsigned long fan2targetrpm = 0;  //Variable to store the target rpm of the second fan
volatile unsigned int fan1pwm = 0;  //Variable to store the pwm of the first fan
volatile unsigned int fan2pwm = 0;  //Variable to store the pwm of the second fan
volatile unsigned long fan1lastcall = 0;  //Variable to store the last time the first fan was called
volatile unsigned long fan2lastcall = 0;  //Variable to store the last time the second fan was called

#define BUILTIN_LED 25 //GPIO of BUILTIN_LED for pico
#ifdef esp32dev
  #undef BUILTIN_LED
  #define BUILTIN_LED 2 //GPIO of BUILTIN_LED for esp32dev
#endif


void sendData(float data1, float data2);  //Function to send data back to the master
void sendData(int data1, int data2);  //Overload to accept int as argument
void sendData(char data1, char data2);  //Overload to accept char as argument
void onRequest(); //Code to execute when master requests data from the slave
void onReceive(int len);  //Code to execute when master sends data to the slave
void fanTick1(); //Function to be called by the tachometer interrupt
void fanTick2(); //Function to be called by the tachometer interrupt
void Regler(); //Function to be called by the timer interrupt

#pragma region sendData

void sendData(float data1 = 0, float data2 = 0){  //Function to send data back to the master
  //Pointer to the float
  uint8_t *bytePointer1 = reinterpret_cast<uint8_t*>(&data1);

  //Iterate throught the adresses of the pointer, to read the bytes of the float, and send them via i2c
  for (uint8_t i = 0; i < sizeof(float); ++i) {
      Wire1.write((*bytePointer1));
      bytePointer1++;
  }

  //Pointer to the second float
  uint8_t *bytePointer2 = reinterpret_cast<uint8_t*>(&data2);

  //Iterate throught the adresses of the pointer, to read the bytes of the float, and send them via i2c
  for (uint8_t i = 0; i < sizeof(float); ++i) {
      Wire1.write((*bytePointer2));
      bytePointer2++;
  }
}

void sendData(int data1 = 0, int data2 = 0){ //Overload to accept int as argument
  sendData((float)data1, (float)data2);
}

void sendData(char data1 = 0, char data2 = 0){  //Overload to accept char as argument
  sendData((float)data1, (float)data2);
}

#pragma endregion

#ifdef DEBUG
void blink(){
  for (char i = 0; i<10; i++){
    digitalWrite(BUILTIN_LED, HIGH);
    delay(50);
    digitalWrite(BUILTIN_LED, LOW);
    delay(50);
  }
}
#endif

void onRequest(){ //Code to execute when master requests data from the slave
  #ifdef DEBUG
  Serial.println("OnRequest");
  Serial.println(Wire1.peek());
  blink();
  #endif
  //Data is already saved in the module variable
  switch(module){
    case Fan1:
      #ifdef DEBUG
        Serial.println("Module 1 called");
      #endif
      //Code to execute when Module1 is being called
      sendData((int)fan1rpm);
      break;
    case Fan2:
      #ifdef DEBUG
        Serial.println("Module 2 called");
      #endif
      //Code to execute when Module2 is being called
      sendData((int)fan2rpm);
      break;
    case Fans:
      #ifdef DEBUG
        Serial.println("Module 3 called");
      #endif
      //Code to execute when Module3 is being called
      sendData((int)fan1rpm, (int)fan2rpm);
      break;
    default:
      //Code to execute when unkown module is being called
      #ifdef DEBUG
        Serial.println("Unknown module called");
      #endif
      Wire1.write(0);  //Send 0 back to the master
      break;
  }
}

void onReceive(int len){
  #ifdef DEBUG
  Serial.println("OnReceive");
  blink();
  #endif
  //Code to execute when master sends data to the slave
  module = Wire1.read();  //Read from which sensor/module the master wants to change
  if (!Wire1.available()){  //Check if there is no more data to read, which means that the master wants to read data from the slave
    return;
  }
  char data = Wire1.read();  //Read the data the master wants to send
  switch(module){
    case Fan1:
      #ifdef DEBUG
        Serial.println("Module 1 called");
      #endif
      //Code to execute when Module1 is being called
      fan1targetrpm = data*8; //Set the target rpm of the first fan
      fan1pwm = data; //Set the pwm of the first fan
      fan2targetrpm = data*8; //Set the target rpm of the second fan
      fan2pwm = data; //Set the pwm of the second fan
      break;
    case Fan2:
      #ifdef DEBUG
        Serial.println("Module 2 called");
      #endif
      //Code to execute when Module2 is being called
      fan2targetrpm = data*8; //Set the target rpm of the second fan
      fan2pwm = data; //Set the pwm of the second fan
      fan1targetrpm = data*8; //Set the target rpm of the first fan
      fan1pwm = data; //Set the pwm of the first fan
      break;
    case Fans:
      #ifdef DEBUG
        Serial.println("Module 3 called");
      #endif
      //Code to execute when Module3 is being called
      fan1targetrpm = data*8; //Set the target rpm of the first fan
      fan1pwm = data; //Set the pwm of the first fan
      fan2targetrpm = data*8; //Set the target rpm of the second fan
      fan2pwm = data; //Set the pwm of the second fan
      break;
    default:
      //Code to execute when unkown module is being called
      #ifdef DEBUG
        Serial.println("Unknown module called");
      #endif
      break;
  }
}

void fanTick1(){
  //Code to execute when the tachometer interrupt is being called
  unsigned long now = micros();
  unsigned long diff = now - fan1lastcall;
  fan1lastcall = now;
  if (diff > DebounceTimeUS){
    fan1rpm =  1/(diff*0.000001)*60/2;
  }
}

void fanTick2(){
  //Code to execute when the tachometer interrupt is being called
  unsigned long now = micros();
  unsigned long diff = now - fan2lastcall;
  fan2lastcall = now;
  if (diff > DebounceTimeUS){
    fan1rpm =  1/(diff*0.000001)*60/2;
  }
}

void Regler(){
  //Code to execute when the timer interrupt is being called
  if(fan1rpm < fan1targetrpm){
    fan1pwm++;
  }
  else if(fan1rpm > fan1targetrpm){
    fan1pwm--;
  }
  if(fan2rpm < fan2targetrpm){
    fan2pwm++;
  }
  else if(fan2rpm > fan2targetrpm){
    fan2pwm--;
  }
  analogWrite(Fan1PWMPin, fan1pwm);
  analogWrite(Fan2PWMPin, fan2pwm);
}

void setup() {
  // put your setup code here, to run once:
  #ifdef DEBUG
  pinMode(BUILTIN_LED, OUTPUT);
  #endif
  Serial.begin(115200);
  Wire1.setSDA(10);
  Wire1.setSCL(11);
  Wire1.onReceive(onReceive);  //Function to be called when a master sends data to the slave
  Wire1.onRequest(onRequest);  //Function to be called when a master requests data from the slave
  Wire1.begin((uint8_t)I2C_ADDR);  //Register this device as a slave on the i2c-bus (on bus 0)

  pinMode(Fan1TachoPin, INPUT_PULLUP);
  pinMode(Fan2TachoPin, INPUT_PULLUP);
  attachInterrupt(Fan1TachoPin, fanTick1, RISING);
  attachInterrupt(Fan2TachoPin, fanTick2, RISING);
  analogWriteFreq(25000);
  ITimer.setInterval(ReglerTimeMS*1000, pico_timer_callback(Regler));
}

void loop() {
  // put your main code here, to run repeatedly:
  // if((fan1lastcall < micros()-1000*1000) && fan1targetrpm > 200){ //If the last call of the first fan is more than 1 second ago
  //   fan1pwm = 125;  //Set the pwm of the first fan to 100%
  // }
  // if((fan2lastcall < micros()-1000*1000) && fan2targetrpm > 200){ //If the last call of the second fan is more than 1 second ago
  //   fan2pwm = 125;  //Set the pwm of the second fan to 100%
  // }
}