#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <SPI.h>
#include <RF24.h>

//CONSTANTS
const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
const byte LCD_COLS = 20;
const byte LCD_ROWS = 4;

//address through which two modules communicate.
const byte master_address[6] = "00001";
const byte slave_address[6] = "00002";

char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'.','0','#','D'}
};

struct SetpointData {
  float fltSPTemp;
  float fltSPHum;
};

struct SensorData {
  float fltSnTemp;
  float fltSnHum;
  bool fanStatus;
  bool humidifierStatus;
};

//VARIABLES
SetpointData spData;
SensorData snData;
 
int state = -1;
String strSPTemp;
String strSPHum;
int timer = 20000;

byte rowPins[ROWS] = {9, 8, 7, 6}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {5, 4, 3, 2}; //connect to the column pinouts of the keypad

RF24 radio(11, 12);  // CE, CSN
LiquidCrystal_I2C lcd(0x27, LCD_COLS, LCD_ROWS);
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

void scrollMessage() {
  String message = "REMOTE TEMPERATURE AND HUMIDITY CONTROLLER.";
  
  for (int i=0; i < 20; i++) {
    message = " " + message;  
  } 
  
  message = message + " ";
   
  for (int position = 0 ; position < message.length(); position++) {
    lcd.setCursor(0, 2);
    lcd.print(message.substring(position, position + 20));
    delay(250);
  }
}

bool getRadioStatus(){ //TODO: Blink leds, send a message, etc. to indicate failure
    if(radio.failureDetected){
    radio.begin();                        // Attempt to re-configure the radio with defaults
    radio.openWritingPipe(master_address);  // Re-configure pipe addresses
    radio.openReadingPipe(0, slave_address);  // Re-configure pipe addresses
    return false;
  } else {
    return true;
  }
  radio.failureDetected = 0;            // Reset the detection value
}

void receiveSensorData(){
  radio.startListening(); //This sets the module as receiver
  if ( radio.available() ) {
    radio.read(&snData, sizeof(SensorData));
//     getRadioStatus();
    if (state == 0) {
      displayData();
    }

    //TODO: Remove
//    Serial.print("RX: Sensor Temperature: ");
//    Serial.print(snData.fltSnTemp); 
//    Serial.print(" Sensor Humidity: ");
//    Serial.println(snData.fltSnHum);
  }

 
}

void transmitSetpointData(){
  
  if (--timer == 0) {
    radio.stopListening(); //This sets the module as transmitter
    radio.write(&spData, sizeof(SetpointData)); //Sending the data
    timer = 20000;
    getRadioStatus();
  }
}

void printTemperatureSP() { //print temperature setpoint on lcd
  lcd.clear();                  
  lcd.setCursor(0, 0);
  lcd.print("Enter Temperature");
  lcd.setCursor(0, 1);
  lcd.print("Setpoint");
  lcd.setCursor(0, 3);
  lcd.print("STP = ");
  lcd.setCursor(6, 3);
  lcd.print(strSPTemp);
}

void printHumiditySP() { //print humidity setpoint on lcd
  lcd.clear();                  
  lcd.setCursor(0, 0);
  lcd.print("Enter Humidity");
  lcd.setCursor(0, 1);
  lcd.print("Setpoint");
  lcd.setCursor(0, 3);
  lcd.print("STP = ");
  lcd.setCursor(6, 3);
  lcd.print(strSPHum);
}

void displayData() { //Home Screen
  lcd.clear();                  
  lcd.setCursor(0, 0);
  lcd.print("Temperature");
  lcd.setCursor(0, 1);
  lcd.print("STP:");
  lcd.setCursor(4, 1);
  lcd.print(spData.fltSPTemp);
  lcd.setCursor(11, 1);
  lcd.print("ACT:");
  lcd.setCursor(15, 1);
  lcd.print(snData.fltSnTemp);

  lcd.setCursor(0, 2);
  lcd.print("Humidity");
  lcd.setCursor(0, 3);
  lcd.print("STP:");
  lcd.setCursor(4, 3);
  lcd.print(spData.fltSPHum);
  lcd.setCursor(11, 3);
  lcd.print("ACT:");
  lcd.setCursor(15, 3);
  lcd.print(snData.fltSnHum);
}

void printErrorMessage() {
  lcd.clear();
  lcd.setCursor(5, 0);                        
  lcd.print("Input error!");
  delay(1000);
}

void setup() {
  Serial.begin(9600);

  //Start communication
  radio.begin();
  radio.openWritingPipe(master_address); //set the address
  radio.openReadingPipe(0, slave_address); //set the address
  
  //LED Pin Configuration
  pinMode(38, OUTPUT);
  pinMode(42, OUTPUT);

  lcd.init();                    
  lcd.backlight();

  lcd.setCursor(6,1);
  lcd.print("GROUP 2");
  delay(500);
  scrollMessage();

  lcd.clear();
  lcd.setCursor(0,0);   //Set cursor to character 2 on line 0
  lcd.print("Enter Setpoints");

  lcd.setCursor(0,2);   //Set cursor to character 2 on line 0
  lcd.print("A: Set Temperature");
  
  lcd.setCursor(0,3);   //Set cursor to character 2 on line 0
  lcd.print("B: Set Humidity");
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:

  // receive sensor readings
  receiveSensorData();
  
  char key = keypad.getKey();

  if (key){  //TODO: Remove
    Serial.println(key);
  }

  if (key == 'A'){
    state = 1;
    printTemperatureSP();
  }

  if (key == 'B'){
    state = 2;
    printHumiditySP();
  }
  
  // when setpoint has been entered, data will then get transmitted 
  if (state == 0) {
    transmitSetpointData();

//    Serial.println(snData.fanStatus);
//    Serial.println(snData.humidifierStatus);
    if(snData.fanStatus){
      digitalWrite(38,HIGH);
    } else {
      digitalWrite(38,LOW);
    }
    
    if(snData.humidifierStatus){
      digitalWrite(42,HIGH);
    } else {
      digitalWrite(42,LOW);
    }
  }

  if (state == 1) {
    if ((key >= '0' && key <= '9') || key == '.' ) {
      strSPTemp += key;     // add entered key value from keypad
      printTemperatureSP(); // print screen to enter temperature setpoint
      
    } else if (key == '#') { // when "#" is clicked
      if (strSPTemp.toFloat() == 0 || strSPTemp.toFloat() >= 50.00) { //checks if entered value is a number
        strSPTemp = ""; // clear invalid temperature setpoint
        printErrorMessage();
        printTemperatureSP(); // print screen to enter temperature setpoint
      } else {
        spData.fltSPTemp = strSPTemp.toFloat(); // convert string to float
        displayData();
        strSPTemp = ""; // clear previously entered data
        state = 0; // set program to state 0
      }
    } else if (key == 'C'){ // when "C" is clicked
      strSPTemp = ""; // // clear previously entered data
      printTemperatureSP(); // print screen to enter temperature setpoint
      
    } else if (key == 'D'){ // when "D" is clicked
      if (spData.fltSPTemp != 0) { // checks if setpoint data was entered previously
        displayData();
        strSPTemp = "";
        state = 0;
      }
    }
  }

  if (state == 2) {
    if ((key >= '0' && key <= '9') || key == '.' ) {
      strSPHum += key;
      printHumiditySP();
      
    } else if (key == '#') {
      if (strSPHum.toFloat() == 0 || strSPHum.toFloat() >= 80.00) { //checks if entered value is a number
        strSPHum = "";
        printErrorMessage();
        printHumiditySP();
      } else {
        spData.fltSPHum = strSPHum.toFloat();
        displayData();
        strSPHum = "";
        state = 0;
      }
    } else if (key == 'C'){
      strSPHum = "";
      printHumiditySP();
    } else if (key == 'D'){
      if (spData.fltSPHum != 0) {
        displayData();
        strSPHum = "";
        state = 0;
      }
    }
  }
}
