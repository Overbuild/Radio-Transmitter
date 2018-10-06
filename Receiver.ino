/*
Radio    Arduino
CE    -> 9
CSN   -> 10 (Hardware SPI SS)
MOSI  -> 11 (Hardware SPI MOSI)
MISO  -> 12 (Hardware SPI MISO)
SCK   -> 13 (Hardware SPI SCK)
IRQ   -> No connection
VCC   -> No more than 3.6 volts
GND   -> GND
*/

#include <SPI.h>
#include <NRFLite.h>
#include <Servo.h>

Servo Servo_1;
Servo Servo_2;

//LED pins
#define LED_Left_Pin 4
#define LED_Receive_Pin 3
#define LED_Right_Pin 2

//LED blink delay (ms)
#define LED_Blink_Delay 100

//Other variables, not to be changed
int16_t Stick_Left_X = 0;
int16_t Stick_Left_Y = 0;
int16_t Stick_Right_X = 0;
int16_t Stick_Right_Y = 0;
bool Switch_Left = false;
bool Switch_Right = false;
int16_t Potentiometer_Left = 0;
int16_t Potentiometer_Right = 0;
bool Stick_Left_Button = false;
bool Stick_Right_Button = false;

unsigned long Led_Blink_Timer = 0;
bool Led_Blink_State = false;

const static uint8_t RADIO_ID = 0;       // Our radio's id.  The transmitter will send to this id.
const static uint8_t PIN_RADIO_CE = 9;
const static uint8_t PIN_RADIO_CSN = 10;

struct RadioPacket{
  
    uint32_t FailedTxCount;
    //Left stick
    int16_t S_L_X;
    int16_t S_L_Y;
    bool S_L_B;
    //Right stick
    int16_t S_R_X;
    int16_t S_R_Y;
    bool S_R_B;
    //Switches
    bool SW_L;
    bool SW_R;
    //Potentiometers
    int16_t P_L;
    int16_t P_R;
    //Programables/Aux
    int16_t AUX_A0;
    int16_t AUX_A1;
    int16_t AUX_A2;
    int16_t AUX_A3;
};

NRFLite _radio;
RadioPacket _radioData;

void setup () {

  pinMode(LED_Left_Pin, OUTPUT);
  pinMode(LED_Receive_Pin, OUTPUT);
  pinMode(LED_Right_Pin, OUTPUT);
  Servo_1.attach(5);
  Servo_2.attach(6);

  if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN)){
      //Serial.println("Cannot communicate with radio");
      while (1); // Wait here forever.
  }
  
}

void loop(){
  
  while (_radio.hasData()) {

    digitalWrite(LED_Receive_Pin, HIGH);
      
    _radio.readData(&_radioData); // Note how '&' must be placed in front of the variable name.

    Stick_Left_X = _radioData.S_L_X;
    Stick_Left_Y = _radioData.S_L_Y;
    Stick_Right_X = _radioData.S_R_X;
    Stick_Right_Y = _radioData.S_R_Y;
    Switch_Left = _radioData.SW_L;
    Switch_Right = _radioData.SW_R;
    Potentiometer_Left = _radioData.P_L;
    Potentiometer_Right = _radioData.P_R;
    Stick_Left_Button = _radioData.S_L_B;
    Stick_Right_Button = _radioData.S_R_B;

  }

  if(Stick_Left_Button == true){
    if((millis() - Led_Blink_Timer) > LED_Blink_Delay){
      Led_Blink_State = !Led_Blink_State;
      Led_Blink_Timer = millis();
    }
    if(Led_Blink_State == true){
      digitalWrite(LED_Left_Pin, HIGH);
    }else{
      digitalWrite(LED_Left_Pin, LOW);
    }
  }else if(Switch_Left == true){
    digitalWrite(LED_Left_Pin, HIGH);
  }else{
    digitalWrite(LED_Left_Pin, LOW);
  }

  if(Stick_Right_Button == true){
    if((millis() - Led_Blink_Timer) > LED_Blink_Delay){
      Led_Blink_State = !Led_Blink_State;
      Led_Blink_Timer = millis();
    }
    if(Led_Blink_State == true){
      digitalWrite(LED_Right_Pin, HIGH);
    }else{
      digitalWrite(LED_Right_Pin, LOW);
    }
  }else if(Switch_Right == true){
    digitalWrite(LED_Right_Pin, HIGH);
  }else{
    digitalWrite(LED_Right_Pin, LOW);
  }

  if(Switch_Left == true && Switch_Right == false){
    Servo_1.write(map(Stick_Left_X, 0, 1023, 0, 180));
    Servo_2.write(map(Stick_Left_Y, 0, 1023, 0, 180));
  }else if(Switch_Left == false && Switch_Right == true){
    Servo_1.write(map(Stick_Right_X, 0, 1023, 0, 180));
    Servo_2.write(map(Stick_Right_Y, 0, 1023, 0, 180));
  }else if(Switch_Left == true && Switch_Right == true){
    Servo_1.write(map(Potentiometer_Left, 0, 1023, 0, 180));
    Servo_2.write(map(Potentiometer_Right, 0, 1023, 0, 180));
  }

  digitalWrite(LED_Receive_Pin, LOW);
    
}
