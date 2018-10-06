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

LED   -> A1
Buzz  -> 4 


Mux: (this is my multiplexer layout, yours can be diffrent, all pins can be reassigned in the code)
0 - menu switch
1 - trim R
2 - prog R
3 - menu buttons
4 - prog L
5 - trim L
6 - top swich L
7 - L3 (inv)
8 - L X
9 - L Y
10 - L pot
11 - R Y
12 - R X
13 - R3 (inv)
14 - top swich R
15 - R pot

A few usefull functions to check out:

AnalogToButtonID()
DrawInt()
IntLengthCalculator()
PlaySound()
PlayDelayedSound()
GetMuxValue()
GetMuxValue();
BoolToByte();

*/

#include <SPI.h>
#include <NRFLite.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

//The screen
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

//Multiplexer to Arduino pins
#define Mux_S0 5
#define Mux_S1 6
#define Mux_S2 7
#define Mux_S3 8
#define Mux_Analog_Pin A0

//Other
#define LED_Pin A1
#define Buzzer_Pin 4

//Multiplexer pins for each of the inputs
#define MuxPin_Left_Stick_X 8
#define MuxPin_Left_Stick_Y 9
#define MuxPin_Left_Stick_Button 7
#define MuxPin_Right_Stick_X 12
#define MuxPin_Right_Stick_Y 11
#define MuxPin_Right_Stick_Button 13
#define MuxPin_Left_Switch 6
#define MuxPin_Right_Switch 14
#define MuxPin_Left_Pot 10
#define MuxPin_Right_Pot 15
#define MuxPin_Left_Trim 5
#define MuxPin_Right_Trim 1
#define MuxPin_Menu_Buttons 3
#define MuxPin_Menu_Switch 0
#define MuxPin_Programables_L 4
#define MuxPin_Programables_R 2

//Analog stick center offsets- unlike trim, these do not (or almost do not) effect min and max values (0 and 1023). With these parameters set correctly all analog stick center values should be approximately 512
#define Stick_Left_Y_Center_Offset -3
#define Stick_Left_X_Center_Offset -7
#define Stick_Right_Y_Center_Offset -12
#define Stick_Right_X_Center_Offset -14

#define Max_Analog_Input_Reference 1023 /*880*/ //Maximum analog value for reference. Lower this value if transmitter 5V line is less than 5V (usualy when powered by USB only). Currently not all functins use this reference, some analog readings are not effected

#define DebounceDelay 50 //Debounce delay for the buttons and switches
#define GUI_Refresh_Rate 15 //Sreen FPS
#define Value_Change_Delay 50 //Time (in milliseconds) it takes for a changing value to increase/decrease during a long press of a button

bool TransmitData = true;//Send data to the reciever
bool Mute = true;//Disable sounds

const static uint8_t RADIO_ID = 1;             // Our radio's id.
const static uint8_t DESTINATION_RADIO_ID = 0; // Id of the radio we will transmit to.
const static uint8_t PIN_RADIO_CE = 9;
const static uint8_t PIN_RADIO_CSN = 10;

//===== Nothing to change from this point. Just global variables =====

//All 16 combiantions of signals for the multiplexer
byte controlPins[] = {B00000000,
                      B10000000,
                      B01000000,
                      B11000000,
                      B00100000,
                      B10100000,
                      B01100000,
                      B11100000,
                      B00010000,
                      B10010000,
                      B01010000,
                      B11010000,
                      B00110000,
                      B10110000,
                      B01110000,
                      B11110000 };

bool Sound_Playing = false;
uint16_t Current_Note_Playing = 0;
uint16_t Current_Sound_Duration = 0;
bool Delayed_Sound_Ready = false;
uint16_t Delayed_Note = 0;
uint16_t Delayed_Sound_Duration = 0;
uint16_t Delayed_Sound_Time_Delay = 0;

//Timers
unsigned long Last_Bounce_Menu_Buttons = 0;
unsigned long Last_Bounce_Menu_Switch = 0;
unsigned long Last_Bounce_Trim_L = 0;
unsigned long Last_Bounce_Trim_R = 0;
unsigned long Last_Bounce_Programables_L = 0;
unsigned long Last_Bounce_Programables_R = 0;
unsigned long Last_Bounce_L_Stick_Button = 0;
unsigned long Last_Bounce_R_Stick_Button = 0;
unsigned long Last_Bounce_Top_Switch_L = 0;
unsigned long Last_Bounce_Top_Switch_R = 0;
unsigned long Trim_L_Long_Press_Timer = 0;
unsigned long Trim_R_Long_Press_Timer = 0;
unsigned long Value_Change_Timer = 0;
unsigned long Sound_Duration_Timer = 0;
unsigned long Note_Timer = 0;
unsigned long Delayed_Sound_Timer = 0;

//Last raw values
int8_t Last_Menu_Buttons_Value = 0;
int8_t Last_Menu_Switch_Value = 0;
int8_t Last_Trim_L_Value = 0;
int8_t Last_Trim_R_Value = 0;
int8_t Last_Programables_L_Value = 0;
int8_t Last_Programables_R_Value = 0;
bool Last_L_Stick_Button_State = false;
bool Last_R_Stick_Button_State = false;
bool Last_Top_Switch_L_State = false;
bool Last_Top_Switch_R_State = false;

//Last debounced values (for detecting a change of state)
int8_t Trim_L_Old = 0;
int8_t Trim_R_Old = 0;
bool Started_Trimming_L = false;
bool Started_Trimming_R= false;
int8_t Programable_Buttons_L_Old = 0;
int8_t Programable_Buttons_R_Old = 0;
bool Top_Switch_L_Old = false;
bool Top_Switch_R_Old = false;
int8_t Menu_Button_Old = 0;
int8_t Menu_Switch_Old = 0;
bool Stick_Left_Button_Old = false;
bool Stick_Right_Button_Old = false;

//Left stick
int16_t Stick_Left_Y = 0;
int16_t S_L_Y_Trim = 0;
int16_t Stick_Left_X = 0;
int16_t S_L_X_Trim = 0;
bool Stick_Left_Button = false;

//Right stick
int16_t Stick_Right_Y = 0;
int16_t S_R_Y_Trim = 0;
int16_t Stick_Right_X = 0;
int16_t S_R_X_Trim = 0;
bool Stick_Right_Button = false;

//Switches
bool Switch_L = false;
bool Switch_R = false;

//Potentiometers
int16_t Pot_L = 0;
int16_t Pot_R = 0;

//Trim buttons
int8_t Trim_L = 0;
int8_t Trim_R = 0;

//Menu input
int8_t Menu_Button = 0;
int8_t Menu_Switch = 0;

//Programables
int8_t Programable_Buttons_L = 0;
int8_t Programable_Buttons_R = 0;

//Auxiliary channels (not used yet: 2018-10-04)
int16_t Aux0 = 0;
int16_t Aux1 = 0;
int16_t Aux2 = 0;
int16_t Aux3 = 0;

//GUI stuff
unsigned long Last_GUI_Refresh = 0;
bool Constant_Refresh = false;
bool Refresh_GUI = true;
int8_t Menu_Item = 0;
int8_t Menu_Level = 0;
bool CharDrawArray[64];
//Custom chars
uint8_t Square_Rounded_Hollow [8] = {0, 60, 66, 66, 66, 66, 60, 0};
uint8_t Square_Rounded_Filled[8] = {0, 60, 126, 126, 126, 126, 60, 0};
uint8_t Square_Rounded_Hollow_L_Half [8] = {0, 0, 0, 0, 0, 60, 66, 66};
uint8_t Square_Rounded_Hollow_R_Half [8] = {66, 66, 60, 0, 0, 0, 0, 0};
uint8_t Square_Rounded_Filled_L_Half [8] = {0, 0, 0, 0, 0, 60, 126, 126};
uint8_t Square_Rounded_Filled_R_Half [8] = {126, 126, 60, 0, 0, 0, 0, 0};

// Values up to 32 bytes can be sent.
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

void setup(){

  u8x8.begin();
  Serial.begin(115200);

  pinMode(Mux_S0, OUTPUT);
  pinMode(Mux_S1, OUTPUT);
  pinMode(Mux_S2, OUTPUT);
  pinMode(Mux_S3, OUTPUT);
  pinMode(LED_Pin, OUTPUT);
  pinMode(Buzzer_Pin, OUTPUT);
  
  if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN)){
      Serial.println("Cannot communicate with radio");
      while (1); // Wait forever.
  }

  digitalWrite(LED_Pin, HIGH);
  Refresh_GUI = true;
    
  //CreateNewChar();//A non run time function for creating custom characters on the sreen. Outputs data to the serial monitor, check the end of this code.
  
}

void loop(){
  
    GetControlInputs();
    GetMenuInputs();
    ManageSounds();
    
    if(TransmitData == true){
      SendData();
    }

    GUI();

}

void ProgramablePressed (bool leftSide){
  PlaySound(400, 50);
}

void LeftSwitchEnabled () {
  PlaySound(300, 50);
  PlayDelayedSound(70, 500, 50);
}

void LeftSwitchDisabled () {
  PlaySound(500, 50);
  PlayDelayedSound(70, 300, 50);
}

void RightSwitchEnabled () {
  PlaySound(300, 50);
  PlayDelayedSound(70, 500, 50);
}

void RightSwitchDisabled () {
  PlaySound(500, 50);
  PlayDelayedSound(70, 300, 50);
}

void MenuForward () {

  if(Menu_Level == 0){
    if(Menu_Item == 2){
      TransmitData = !TransmitData;
    }else if(Menu_Item == 3){
      Mute = !Mute;
    }else{
      Menu_Level++;
      u8x8.clear();
    }
  }
  
  Refresh_GUI = true;
  PlaySound(500, 20);
}

void MenuBack () {

  if(Menu_Level == 0){
    if(Menu_Item == 2){
      TransmitData = !TransmitData;
    }else if(Menu_Item == 3){
      Mute = !Mute;
    }
  }else if(Menu_Level > 0){
    Menu_Level--;
    u8x8.clear();
  }
  
  Refresh_GUI = true;
  PlaySound(300, 20);
}

void MenuUp () {

  if(Menu_Item > 0){
    Menu_Item--;
  }else{
    Menu_Item = 3;
  }
  
  Refresh_GUI = true;
  PlaySound(400, 20);
}

void MenuDown () {

  if(Menu_Item < 3){
    Menu_Item++;
  }else{
    Menu_Item = 0;
  }
  
  Refresh_GUI = true;
  PlaySound(400, 20);
}

void StickLeftButtonPressed () {
  PlaySound(600, 50);
}

void StickRightButtonPressed () {
  PlaySound(600, 50);
}

void ManageSounds () {
  if(Sound_Playing == true){
    PlaySound(0, 0);
  }
  if(Delayed_Sound_Ready == true){
    PlayDelayedSound(0, 0, 0);
  }
}

void GUI () {

  if(Constant_Refresh == true && (millis() - Last_GUI_Refresh) > (1000 / GUI_Refresh_Rate)){
    Refresh_GUI = true;
  }

  if(Refresh_GUI == true){

    Last_GUI_Refresh = millis();

    if(Menu_Level == 0){
      GUI_Main_Menu();
    }else if(Menu_Item == 0){
      GUI_Inputs_Visualization();
    }else if(Menu_Item == 1){
      GUI_Inputs_Visualization_Bad();
    }

    Refresh_GUI = false;

  }
  
}

void GUI_Main_Menu () {

  Constant_Refresh = false;
  u8x8.setFont(u8x8_font_chroma48medium8_r);

  if(Menu_Item == 0){
    u8x8.setInverseFont(1);
  }else{
    u8x8.setInverseFont(0);
  }

  u8x8.setCursor(1,0);
  u8x8.print("Inputs");

  if(Menu_Item == 1){
    u8x8.setInverseFont(1);
  }else{
    u8x8.setInverseFont(0);
  }

  u8x8.setCursor(1,1);
  u8x8.print("Inputs bad");

  if(Menu_Item == 2){
    u8x8.setInverseFont(1);
  }else{
    u8x8.setInverseFont(0);
  }

  u8x8.setCursor(1,2);
  u8x8.print("Transmit");

  if(TransmitData == true){
    u8x8.drawTile(14, 2, 1, Square_Rounded_Filled);
  }else{
    u8x8.drawTile(14, 2, 1, Square_Rounded_Hollow);
  }

  if(Menu_Item == 3){
    u8x8.setInverseFont(1);
  }else{
    u8x8.setInverseFont(0);
  }

  u8x8.setCursor(1,3);
  u8x8.print("Sound");

  if(Mute == false){
    u8x8.drawTile(14, 3, 1, Square_Rounded_Filled);
  }else{
    u8x8.drawTile(14, 3, 1, Square_Rounded_Hollow);
  }
  
}

void GUI_Inputs_Visualization () {
  
  Constant_Refresh = true;
  u8x8.setFont(u8x8_font_chroma48medium8_r);
    
  DrawInt(Pot_L, 0, 1, 4);
  DrawInt(Pot_R, 12, 1, 4);

  if(Stick_Left_Button == true){
    u8x8.setInverseFont(1);
  }else{
    u8x8.setInverseFont(0);
  }
  DrawInt(Stick_Left_Y, 1, 3, 5);
  DrawInt(Stick_Left_X, 1, 4, 5);

  if(Stick_Right_Button == true){
    u8x8.setInverseFont(1);
  }else{
    u8x8.setInverseFont(0);
  }
  DrawInt(Stick_Right_Y, 10, 3, 5);
  DrawInt(Stick_Right_X, 10, 4, 5);
    
  u8x8.setInverseFont(0);

  if(Switch_L == true){
    u8x8.drawTile(1, 0, 1, Square_Rounded_Filled);
  }else{
    u8x8.drawTile(1, 0, 1, Square_Rounded_Hollow);
  }

  if(Switch_R == true){
    u8x8.drawTile(11, 0, 1, Square_Rounded_Filled);
  }else{
    u8x8.drawTile(11, 0, 1, Square_Rounded_Hollow);
  }

  if(Programable_Buttons_L == 3){
    u8x8.drawTile(3, 5, 1, Square_Rounded_Filled);
  }else{
    u8x8.drawTile(3, 5, 1, Square_Rounded_Hollow);
  }

  if(Programable_Buttons_L == 2){
    u8x8.drawTile(3, 6, 1, Square_Rounded_Filled_L_Half);
    u8x8.drawTile(4, 6, 1, Square_Rounded_Filled_R_Half);
  }else{
    u8x8.drawTile(3, 6, 1, Square_Rounded_Hollow_L_Half);
    u8x8.drawTile(4, 6, 1, Square_Rounded_Hollow_R_Half);
  }

  if(Programable_Buttons_L == 1){
    u8x8.drawTile(4, 7, 1, Square_Rounded_Filled);
  }else{
    u8x8.drawTile(4, 7, 1, Square_Rounded_Hollow);
  }

  if(Programable_Buttons_R == 3){
    u8x8.drawTile(10, 5, 1, Square_Rounded_Filled);
  }else{
    u8x8.drawTile(10, 5, 1, Square_Rounded_Hollow);
  }

  if(Programable_Buttons_R == 2){
    u8x8.drawTile(9, 6, 1, Square_Rounded_Filled_L_Half);
    u8x8.drawTile(10, 6, 1, Square_Rounded_Filled_R_Half);
  }else{
    u8x8.drawTile(9, 6, 1, Square_Rounded_Hollow_L_Half);
    u8x8.drawTile(10, 6, 1, Square_Rounded_Hollow_R_Half);
  }

  if(Programable_Buttons_R == 1){
    u8x8.drawTile(9, 7, 1, Square_Rounded_Filled);
  }else{
    u8x8.drawTile(9, 7, 1, Square_Rounded_Hollow);
  }
}

void SendData () {

    if (_radio.send(DESTINATION_RADIO_ID, &_radioData, sizeof(_radioData))){ // Note how '&' must be placed in front of the variable name.
        //Serial.println("...Success");
    }else{
        //Serial.println("...Failed");
        _radioData.FailedTxCount++;
    }

    _radioData.S_L_X = Stick_Left_X;
    _radioData.S_L_Y = Stick_Left_Y;
    _radioData.S_L_B = Stick_Left_Button;
    _radioData.S_R_X = Stick_Right_X;
    _radioData.S_R_Y = Stick_Right_Y;
    _radioData.S_R_B = Stick_Right_Button;
    _radioData.SW_L = Switch_L;
    _radioData.SW_R = Switch_R;
    _radioData.P_L = Pot_L;
    _radioData.P_R = Pot_R;
    _radioData.AUX_A0 = Aux0;
    _radioData.AUX_A1 = Aux1;
    _radioData.AUX_A2 = Aux2;
    _radioData.AUX_A3 = Aux3;

}

int16_t AnalogToButtonID (int16_t analogSignal, int8_t buttonAmmount){//Outputs a number of a button (from a group of buttons) that is pressed. Parameters: analogSignal- analog value generated by a group of buttons (0 - 1023); buttonAmmount- the ammount of buttons in the group

  int16_t output = -1;

  for(int i = 0; i < (buttonAmmount + 1); i++){
    int compValue = Max_Analog_Input_Reference / buttonAmmount;
    if(analogSignal < ((compValue * i) + (compValue / 2))){
      output = i;
      break;
    }
  }

  return output;
  
}

void ManageTrim () {

  if(Trim_L != Trim_L_Old){
    ApplyLeftTrim(5);
    Trim_L_Old = Trim_L;
  }

  if(Trim_R != Trim_R_Old){
    ApplyRightTrim(5);
    Trim_R_Old = Trim_R;
  }

  if(Trim_L != 0){
    if(Started_Trimming_L == false){
      Trim_L_Long_Press_Timer = millis();
      Started_Trimming_L = true;
    }
  }else{
    Started_Trimming_L = false;
  }

  if(Trim_R != 0){
    if(Started_Trimming_R == false){
      Trim_R_Long_Press_Timer = millis();
      Started_Trimming_R = true;
    }
  }else{
    Started_Trimming_R = false;
  }

  if(Trim_L != 0 || Trim_R != 0){
    if((millis() - Value_Change_Timer) > Value_Change_Delay){
      if((millis() - Trim_L_Long_Press_Timer) > 500){
        ApplyLeftTrim(10);
      }
      if((millis() - Trim_R_Long_Press_Timer) > 500){
        ApplyRightTrim(10);
      }
      Value_Change_Timer = millis();
    }
  }
  
}

void ApplyLeftTrim (int trimAmmount) {
  if(Trim_L == 1){
    S_L_X_Trim -= trimAmmount;
    if(S_L_X_Trim > -510){
      PlaySound(200, 10);
    }
  }
  if(Trim_L == 2){
    S_L_X_Trim += trimAmmount;
    if(S_L_X_Trim < 510){
      PlaySound(350, 10);
    }
  }
  if(Trim_L == 3){
    S_L_Y_Trim -= trimAmmount;
    if(S_L_Y_Trim > -510){
      PlaySound(200, 10);
    }
  }
  if(Trim_L == 4){
    S_L_Y_Trim += trimAmmount;
    if(S_L_Y_Trim < 510){
      PlaySound(350, 10);
    }
  }
  S_L_X_Trim = constrain(S_L_X_Trim, -510, 510);
  S_L_Y_Trim = constrain(S_L_Y_Trim, -510, 510);
}

void ApplyRightTrim (int trimAmmount) {
  if(Trim_R == 1){
    S_R_X_Trim += trimAmmount;
    if(S_R_X_Trim < 510){
      PlaySound(350, 10);
    }
  }
  if(Trim_R == 2){
    S_R_X_Trim -= trimAmmount;
    if(S_R_X_Trim > -510){
      PlaySound(200, 10);
    }
  }
  if(Trim_R == 3){
    S_R_Y_Trim -= trimAmmount;
    if(S_R_Y_Trim > -510){
      PlaySound(200, 10);
    }
  }
  if(Trim_R == 4){
    S_R_Y_Trim += trimAmmount;
    if(S_R_Y_Trim < 510){
      PlaySound(350, 10);
    }
  }
  S_R_X_Trim = constrain(S_R_X_Trim, -510, 510);
  S_R_Y_Trim = constrain(S_R_Y_Trim, -510, 510);
}

void GetMenuInputs () {

  int8_t muxToInt = 0;
  //
  //    Trim
  //
  muxToInt = AnalogToButtonID(GetMuxValue(MuxPin_Left_Trim), 4);
  if(muxToInt != Last_Trim_L_Value){
    Last_Bounce_Trim_L = millis();
  }
  if((millis() - Last_Bounce_Trim_L) >= DebounceDelay){
    Trim_L = muxToInt;
  }
  Last_Trim_L_Value = muxToInt;
  ////
  muxToInt = AnalogToButtonID(GetMuxValue(MuxPin_Right_Trim), 4);
  if(muxToInt != Last_Trim_R_Value){
    Last_Bounce_Trim_R = millis();
  }
  if((millis() - Last_Bounce_Trim_R) >= DebounceDelay){
    Trim_R = muxToInt;
  }
  Last_Trim_R_Value = muxToInt;
  ManageTrim();
  //
  //    Menu
  //
  muxToInt = AnalogToButtonID(GetMuxValue(MuxPin_Menu_Buttons), 2);
  if(muxToInt != Last_Menu_Buttons_Value){
    Last_Bounce_Menu_Buttons = millis();
  }
  if((millis() - Last_Bounce_Menu_Buttons) >= DebounceDelay){
    Menu_Button = muxToInt;
  }
  Last_Menu_Buttons_Value = muxToInt;
  ////
  muxToInt = AnalogToButtonID(GetMuxValue(MuxPin_Menu_Switch), 2);
  if(muxToInt != Last_Menu_Switch_Value){
    Last_Bounce_Menu_Switch = millis();
  }
  if((millis() - Last_Bounce_Menu_Switch) >= DebounceDelay){
    Menu_Switch = muxToInt;
  }
  Last_Menu_Switch_Value = muxToInt;

  if(Menu_Button != Menu_Button_Old){
    if(Menu_Button == 2){
      MenuForward();
    }else if(Menu_Button == 1){
      MenuBack();
    }
    Menu_Button_Old = Menu_Button;
  }

  if(Menu_Switch != Menu_Switch_Old){
    if(Menu_Switch == 2){
      MenuUp();
    }else if(Menu_Switch == 1){
      MenuDown();
    }
    Menu_Switch_Old = Menu_Switch;
  }
  //
  //    Programable Buttons
  //
  muxToInt = AnalogToButtonID(GetMuxValue(MuxPin_Programables_L), 3);
  if(muxToInt != Last_Programables_L_Value){
    Last_Bounce_Programables_L = millis();
  }
  if((millis() - Last_Bounce_Programables_L) >= DebounceDelay){
    Programable_Buttons_L = muxToInt;
  }
  Last_Programables_L_Value = muxToInt;
  ////
  muxToInt = AnalogToButtonID(GetMuxValue(MuxPin_Programables_R), 3);
  if(muxToInt != Last_Programables_R_Value){
    Last_Bounce_Programables_R = millis();
  }
  if((millis() - Last_Bounce_Programables_R) >= DebounceDelay){
    Programable_Buttons_R = muxToInt;
  }
  Last_Programables_R_Value = muxToInt;

  if(Programable_Buttons_L_Old != Programable_Buttons_L){
    if(Programable_Buttons_L != 0){
      ProgramablePressed(true);
    }
    Programable_Buttons_L_Old = Programable_Buttons_L;
  }

  if(Programable_Buttons_R_Old != Programable_Buttons_R){
    if(Programable_Buttons_R != 0){
      ProgramablePressed(false);
    }
    Programable_Buttons_R_Old = Programable_Buttons_R;
  }
}

void GetControlInputs () {
    
  Stick_Left_Y = GetMuxValue(MuxPin_Left_Stick_Y) + Stick_Left_Y_Center_Offset;
  Stick_Left_Y -= abs((float)Stick_Left_Y - 512) / (512 + Stick_Left_Y_Center_Offset) * Stick_Left_Y_Center_Offset * 1.01;
  Stick_Left_Y += S_L_Y_Trim;
  Stick_Left_Y = constrain(((float)Stick_Left_Y * 1023 / Max_Analog_Input_Reference), 0, 1023);

  Stick_Left_X = GetMuxValue(MuxPin_Left_Stick_X) + Stick_Left_X_Center_Offset;
  Stick_Left_X -= abs((float)Stick_Left_X - 512) / (512 + Stick_Left_X_Center_Offset) * Stick_Left_X_Center_Offset * 1.01;
  Stick_Left_X += S_L_X_Trim;
  Stick_Left_X = constrain(((float)Stick_Left_X * 1023 / Max_Analog_Input_Reference), 0, 1023);

  Stick_Right_Y = GetMuxValue(MuxPin_Right_Stick_Y) + Stick_Right_Y_Center_Offset;
  Stick_Right_Y -= abs((float)Stick_Right_Y - 512) / (512 + Stick_Right_Y_Center_Offset) * Stick_Right_Y_Center_Offset * 1.01;
  Stick_Right_Y += S_R_Y_Trim;
  Stick_Right_Y = constrain(((float)Stick_Right_Y * 1023 / Max_Analog_Input_Reference), 0, 1023);

  Stick_Right_X = GetMuxValue(MuxPin_Right_Stick_X) + Stick_Right_X_Center_Offset;
  Stick_Right_X -= abs((float)Stick_Right_X - 512) / (512 + Stick_Right_X_Center_Offset) * Stick_Right_X_Center_Offset * 1.01;
  Stick_Right_X += S_R_X_Trim;
  Stick_Right_X = constrain(((float)Stick_Right_X * 1023 / Max_Analog_Input_Reference), 0, 1023);

  Pot_L = GetMuxValue(MuxPin_Left_Pot);
  Pot_L = constrain(((float)Pot_L * 1023 / Max_Analog_Input_Reference), 0, 1023);
  
  Pot_R = GetMuxValue(MuxPin_Right_Pot);
  Pot_R = constrain(((float)Pot_R * 1023 / Max_Analog_Input_Reference), 0, 1023);
    
  bool muxToBool = false;
  //
  //    Buttons
  //
  if(GetMuxValue(MuxPin_Left_Stick_Button) < (Max_Analog_Input_Reference / 2)){
    muxToBool = true;
  }else{
    muxToBool = false;
  }
  if(muxToBool != Last_L_Stick_Button_State){
    Last_Bounce_L_Stick_Button = millis();
  }
  if((millis() - Last_Bounce_L_Stick_Button) >= DebounceDelay){
    Stick_Left_Button = muxToBool;
  }
  Last_L_Stick_Button_State = muxToBool;
  ////
  if(GetMuxValue(MuxPin_Right_Stick_Button) < (Max_Analog_Input_Reference / 2)){
    muxToBool = true;
  }else{
    muxToBool = false;
  }
  if(muxToBool != Last_R_Stick_Button_State){
    Last_Bounce_R_Stick_Button = millis();
  }
  if((millis() - Last_Bounce_R_Stick_Button) >= DebounceDelay){
    Stick_Right_Button = muxToBool;
  }
  Last_R_Stick_Button_State = muxToBool;

  if(Stick_Left_Button != Stick_Left_Button_Old){
    if(Stick_Left_Button == true){
      StickLeftButtonPressed();
    }
    Stick_Left_Button_Old = Stick_Left_Button;
  }

  if(Stick_Right_Button != Stick_Right_Button_Old){
    if(Stick_Right_Button == true){
      StickRightButtonPressed();
    }
    Stick_Right_Button_Old = Stick_Right_Button;
  }
  //
  //    Switches
  //
  if(GetMuxValue(MuxPin_Left_Switch) > (Max_Analog_Input_Reference / 2)){
    muxToBool = true;
  }else{
    muxToBool = false;
  }
  if(muxToBool != Last_Top_Switch_L_State){
    Last_Bounce_Top_Switch_L = millis();
  }
  if((millis() - Last_Bounce_Top_Switch_L) >= DebounceDelay){
    Switch_L = muxToBool;
  }
  Last_Top_Switch_L_State = muxToBool;
  ////
  if(GetMuxValue(MuxPin_Right_Switch) > (Max_Analog_Input_Reference / 2)){
    muxToBool = true;
  }else{
    muxToBool = false;
  }
  if(muxToBool != Last_Top_Switch_R_State){
    Last_Bounce_Top_Switch_R = millis();
  }
  if((millis() - Last_Bounce_Top_Switch_R) >= DebounceDelay){
    Switch_R = muxToBool;
  }
  Last_Top_Switch_R_State = muxToBool;

  if(Top_Switch_L_Old != Switch_L){
    if(Switch_L == true){
      LeftSwitchEnabled();
    }else{
      LeftSwitchDisabled();
    }
    Top_Switch_L_Old = Switch_L;
  }

  if(Top_Switch_R_Old != Switch_R){
    if(Switch_R == true){
      RightSwitchEnabled();
    }else{
      RightSwitchDisabled();
    }
    Top_Switch_R_Old = Switch_R;
  }
  
}

void DrawInt (int var, int xPos, int yPos, int fieldSize){//Displays an integer on sreen. Parameters: var- the integer that has to be drawn; xPos- (position) row on screen; yPos- (position) colum on screen; fieldSize- amount of characters allocated to display the value (do not forget to leave room for the "-" sign as well if the integer can become negative)
  u8x8.setCursor(xPos,yPos);
  u8x8.print(var);
  int curVarLength = IntLengthCalculator(var);

  if(curVarLength < fieldSize){

    uint8_t emptyChar[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    
    for(int i = (xPos + curVarLength); i < (xPos + fieldSize); i++){
      u8x8.drawTile(i, yPos, 1, emptyChar);
    }
  }
}

uint8_t IntLengthCalculator (int input) {//Input- an integer, output- the ammount of characters it takes up on the screen. IntLengthCalculator(-5252) = 5; IntLengthCalculator(98989898) = 8;
  uint8_t output = 1;

  if(input < 0){
    output ++;
  }

  if(abs(input) >= 10){
    output ++;
  }

  if(abs(input) >= 100){
    output ++;
  }

  if(abs(input) >= 1000){
    output ++;
  }

  return output;
  
}

void PlaySound (uint16_t note, uint16_t duration) {//Requires ManageSounds() function in the main loop. Plays a sound on the buzzer. This function is called just once and plays the note for set ammount of time automaicly. Parameters: note- note (pitch value) that has to be played; duration- the ammount of time the note has to be held

  if(Mute == false){
    if(Sound_Playing == false){
      Sound_Duration_Timer = millis();
      Current_Note_Playing = note;
      Current_Sound_Duration = duration;
      Sound_Playing = true;
    }
    if((millis() - Sound_Duration_Timer) < Current_Sound_Duration){
      tone(Buzzer_Pin, Current_Note_Playing);
    }else{
      noTone(Buzzer_Pin);
      Sound_Playing = false;
    }
  }
  
}

void PlayDelayedSound (uint16_t timeDelay, uint16_t note, uint16_t duration) {//Requires ManageSounds() function in the main loop. Does the same as PlaySound function, has an extra parameter: timeDelay- a delay (in ms) before the set note will be played. This function is used together with PlaySound() for playing two note melodies

  if(Mute == false){
    if(Delayed_Sound_Ready == false){
      Delayed_Sound_Timer = millis();
      Delayed_Note = note;
      Delayed_Sound_Duration = duration;
      Delayed_Sound_Time_Delay = timeDelay;
      Delayed_Sound_Ready = true;
    }else if((millis() - Delayed_Sound_Timer) >= Delayed_Sound_Time_Delay){
      PlaySound(Delayed_Note, Delayed_Sound_Duration);
      Delayed_Sound_Ready = false;
    }
  }
  
}

int16_t GetMuxValue (int id){//Input- multiplexer pin (0 - 15), output- analog value red by the multiplexer at that pin. GetMuxValue(12) = (0 - 1023)
  int16_t result = -1;

  if(bitRead(controlPins[id], 4) == 1){
    digitalWrite(Mux_S3, HIGH);
  }else{
    digitalWrite(Mux_S3, LOW);
  }

  if(bitRead(controlPins[id], 5) == 1){
    digitalWrite(Mux_S2, HIGH);
  }else{
    digitalWrite(Mux_S2, LOW);
  }

  if(bitRead(controlPins[id], 6) == 1){
    digitalWrite(Mux_S1, HIGH);
  }else{
    digitalWrite(Mux_S1, LOW);
  }

  if(bitRead(controlPins[id], 7) == 1){
    digitalWrite(Mux_S0, HIGH);
  }else{
    digitalWrite(Mux_S0, LOW);
  }

  result = analogRead(Mux_Analog_Pin);

  return result;
}

//=====================================================================================================================================
//Non run-time functions- comment them out when uploading the final code, saves RAM and memory

void GUI_Inputs_Visualization_Bad () {//An example of what not to do
    
    Constant_Refresh = true;
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.clear();
    
    u8x8.setCursor(0,1);
    u8x8.print(Pot_L);
    u8x8.setCursor(12,1);
    u8x8.print(Pot_R);

    if(Stick_Left_Button == true){
      u8x8.setInverseFont(1);
    }else{
      u8x8.setInverseFont(0);
    }

    u8x8.setCursor(1,3);
    u8x8.print(Stick_Left_Y);
    u8x8.setCursor(1,4);
    u8x8.print(Stick_Left_X);

    if(Stick_Right_Button == true){
      u8x8.setInverseFont(1);
    }else{
      u8x8.setInverseFont(0);
    }

    u8x8.setCursor(10,3);
    u8x8.print(Stick_Right_Y);
    u8x8.setCursor(10,4);
    u8x8.print(Stick_Right_X);
    
    u8x8.setInverseFont(0);

    if(Switch_L == true){
      u8x8.drawTile(1, 0, 1, Square_Rounded_Filled);
    }else{
      u8x8.drawTile(1, 0, 1, Square_Rounded_Hollow);
    }

    if(Switch_R == true){
      u8x8.drawTile(11, 0, 1, Square_Rounded_Filled);
    }else{
      u8x8.drawTile(11, 0, 1, Square_Rounded_Hollow);
    }

    if(Programable_Buttons_L == 3){
      u8x8.drawTile(3, 5, 1, Square_Rounded_Filled);
    }else{
      u8x8.drawTile(3, 5, 1, Square_Rounded_Hollow);
    }

    if(Programable_Buttons_L == 2){
      u8x8.drawTile(3, 6, 1, Square_Rounded_Filled_L_Half);
      u8x8.drawTile(4, 6, 1, Square_Rounded_Filled_R_Half);
    }else{
      u8x8.drawTile(3, 6, 1, Square_Rounded_Hollow_L_Half);
      u8x8.drawTile(4, 6, 1, Square_Rounded_Hollow_R_Half);
    }

    if(Programable_Buttons_L == 1){
      u8x8.drawTile(4, 7, 1, Square_Rounded_Filled);
    }else{
      u8x8.drawTile(4, 7, 1, Square_Rounded_Hollow);
    }

    if(Programable_Buttons_R == 3){
      u8x8.drawTile(10, 5, 1, Square_Rounded_Filled);
    }else{
      u8x8.drawTile(10, 5, 1, Square_Rounded_Hollow);
    }

    if(Programable_Buttons_R == 2){
      u8x8.drawTile(9, 6, 1, Square_Rounded_Filled_L_Half);
      u8x8.drawTile(10, 6, 1, Square_Rounded_Filled_R_Half);
    }else{
      u8x8.drawTile(9, 6, 1, Square_Rounded_Hollow_L_Half);
      u8x8.drawTile(10, 6, 1, Square_Rounded_Hollow_R_Half);
    }

    if(Programable_Buttons_R == 1){
      u8x8.drawTile(9, 7, 1, Square_Rounded_Filled);
    }else{
      u8x8.drawTile(9, 7, 1, Square_Rounded_Hollow);
    }
}

/*
void CreateNewChar (){//Requires CreateNewChar() function in void setup. Used to create new characters, returns 8 bytes in serial monitor. Not a run-time function

  //Preview: (screen must not auto update, this shape is displayed just once, could be moved to void loop)
  uint8_t New_Shape[8] = {0, 0, 0, 0, 0, 60, 126, 126};
  u8x8.drawTile(0, 0, 1, New_Shape);
  
//         Fresh copy
//    8X8 area of pixels for a new character   |
//                                             V
//                                               
//    CharArrayConstructor(0,      0, 0, 0, 0, 0, 0, 0, 0);
//    CharArrayConstructor(1,      0, 0, 0, 0, 0, 0, 0, 0);
//    CharArrayConstructor(2,      0, 0, 0, 0, 0, 0, 0, 0);
//    CharArrayConstructor(3,      0, 0, 0, 0, 0, 0, 0, 0);
//    CharArrayConstructor(4,      0, 0, 0, 0, 0, 0, 0, 0);
//    CharArrayConstructor(5,      0, 0, 0, 0, 0, 0, 0, 0);
//    CharArrayConstructor(6,      0, 0, 0, 0, 0, 0, 0, 0);
//    CharArrayConstructor(7,      0, 0, 0, 0, 0, 0, 0, 0);
  
  CharArrayConstructor(0,      0, 0, 0, 0, 0, 0, 0, 0);
  CharArrayConstructor(1,      1, 1, 0, 0, 0, 0, 0, 0);
  CharArrayConstructor(2,      1, 1, 1, 0, 0, 0, 0, 0);
  CharArrayConstructor(3,      1, 1, 1, 0, 0, 0, 0, 0);
  CharArrayConstructor(4,      1, 1, 1, 0, 0, 0, 0, 0);
  CharArrayConstructor(5,      1, 1, 1, 0, 0, 0, 0, 0);
  CharArrayConstructor(6,      1, 1, 0, 0, 0, 0, 0, 0);
  CharArrayConstructor(7,      0, 0, 0, 0, 0, 0, 0, 0);

  Serial.print("uint8_t New_Shape[8] = {");

  byte charByte = 0;

  for(int i = 0; i < 64; i += 8){

    charByte = BoolToByte(CharDrawArray[i], CharDrawArray[i + 1], CharDrawArray[i + 2], CharDrawArray[i + 3], CharDrawArray[i + 4], CharDrawArray[i + 5], CharDrawArray[i + 6], CharDrawArray[i + 7]);
    Serial.print(charByte);
    if(i < 56){
      Serial.print(", ");
    }
    
  }

  Serial.println("};");
  
}

void CharArrayConstructor (int8_t line, bool b0, bool b1, bool b2, bool b3, bool b4, bool b5, bool b6, bool b7) {//Assigns bool values to "CharDrawArray" array, also turns the character 90 deg CCW (needed for it to be displayed correctly). Not a run-time function

  int8_t currBoolID = 0;
  
  for(int8_t i = line; i < 64; i += 8){
    if(currBoolID == 0){
      CharDrawArray[i] = b0;
    }else if(currBoolID == 1){
      CharDrawArray[i] = b1;
    }else if(currBoolID == 2){
      CharDrawArray[i] = b2;
    }else if(currBoolID == 3){
      CharDrawArray[i] = b3;
    }else if(currBoolID == 4){
      CharDrawArray[i] = b4;
    }else if(currBoolID == 5){
      CharDrawArray[i] = b5;
    }else if(currBoolID == 6){
      CharDrawArray[i] = b6;
    }else if(currBoolID == 7){
      CharDrawArray[i] = b7;
    }
    currBoolID++;
    
  }
}

byte BoolToByte (bool b0, bool b1, bool b2, bool b3, bool b4, bool b5, bool b6, bool b7){//Converts 8 bool s (8 bits) in to 1 byte. BoolToByte(1, 1, 1, 1, 1, 1, 1, 1) = 255. Not a run-time function
  byte output = 0;

  if(b0 == true){
    output += 1;
  }

  if(b1 == true){
    output += 2;
  }

  if(b2 == true){
    output += 4;
  }

  if(b3 == true){
    output += 8;
  }

  if(b4 == true){
    output += 16;
  }

  if(b5 == true){
    output += 32;
  }

  if(b6 == true){
    output += 64;
  }

  if(b7 == true){
    output += 128;
  }

  return output;
  
}*/
