//LIBRARIES
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <SPI.h>
#include <RF24.h>
#include <PID_v1.h>

#define DHTPIN 12             // DHT11 Sensor Pin
#define DHTTYPE DHT22         // DHT Sensor Type 
#define fan 10                // Fan pin
#define humidifier 9          // Humidifier pin
#define humidityLED 42        // LED pin to indicate low humidity level
#define temperatureLED 36     // LED pin to indicate high temperature level
#define pushButton 11         // Button pin for changing display screen

//CONSTANTS
//Address through which two modules communicate.
const byte master_address[6] = "00001";
const byte slave_address[6] = "00002";

//Temperature Gain Parameters
const double Kp_t = 6.00; //proportional gain
const double Ki_t = 2.00; //integral gain
const double Kd_t = 5.00; //derivative gain

//Humidity Gain Parameters
const double Kp_h = 6.00; //proportional gain
const double Ki_h = 2.00; //integral gain
const double Kd_h = 5.00; //derivative gain

//STRUCTURES
//Store temperature and humidity setpoints received from the control centre
struct SetpointData {
  double fltSPTemp;
  double fltSPHum;
};

//Store temperature and humidity readings from the DHT22 sensor
struct SensorData {
  double fltSnTemp;;
  double fltSnHum;
  bool fanStatus;
  bool humidifierStatus;
};

//VARIABLES
SetpointData spData;
SensorData snData;

double fanOutput, fanPercentage;
double humidifierOutput, humidifierPercentage;
float oldTemperature, oldHumidity;
long timePast;
long timeNow;
int buttonState, state = 0;
bool changeScreen = true;
bool buttonPressed = false;

//INSTANCES
PID tempPID(&snData.fltSnTemp, &fanOutput, &spData.fltSPTemp, Kp_t, Ki_t, Kd_t, REVERSE); //Temperature PID Controller
PID humPID(&snData.fltSnHum, &humidifierOutput, &spData.fltSPHum, Kp_h, Ki_h, Kd_h, DIRECT); //Humidity PID Controller
DHT dht(DHTPIN, DHTTYPE); //DHT Sensor Object 
RF24 radio(7, 8);  //CE, CSN - RF24 object
LiquidCrystal_I2C lcd(0x27, 16, 2); //LCD Object


void setup() {
  while (!Serial);
  Serial.begin(9600);

  //Set fan and humidifier to a LOW state
  analogWrite(fan, LOW);
  analogWrite(humidifier, LOW);
  pinMode(humidityLED, OUTPUT);
  pinMode(temperatureLED, OUTPUT);

  //Initialize setpoints
  spData.fltSPTemp = 26.00;
  spData.fltSPHum = 0.00;

  //Set pin for button
  pinMode(pushButton,INPUT_PULLUP);
  
  //Start DHT sensor
  dht.begin(); 

  //Turn temperature Cascade PID Controller on
  tempPID.SetMode(AUTOMATIC);
  humPID.SetMode(AUTOMATIC);
  
  //Adjust PID values
  tempPID.SetTunings(Kp_t, Ki_t, Kd_t);
  humPID.SetTunings(Kp_h, Ki_h, Kd_h);
    
  //Start communication
  radio.begin();
  radio.openWritingPipe(slave_address);
  radio.openReadingPipe(0, master_address);
  
  lcd.init();
  lcd.backlight();
  
  lcd.clear();
  lcd.print("Temperature");
  lcd.setCursor(0, 1);
  lcd.print("Cooling System ");
  delay(2000);
  
  lcd.clear();
  lcd.print("Using DHT Sensor");
  lcd.setCursor(0, 1);
  lcd.print("Slave Arduino");
  delay(2000);
}

void loop() {
  //receive setpoint data
  receiveSetpointData();

  //read button state
  readButtonState();
  
  //time past in miliseconds
  timeNow = millis();
  
  //Wait a 2 second between sensor measurements
    if (timeNow - timePast >= 2000) {
      
      //read new temperature and humidity
      snData.fltSnHum = dht.readHumidity();
      snData.fltSnTemp = dht.readTemperature();

      //Display new temperature and humidity readings to LCD
      displayScreen();
      
      //check if temperature and humidity readings changed
      if (snData.fltSnTemp != oldTemperature || snData.fltSnHum != oldHumidity ) {
        
        if (fanOutput > 0){
          snData.fanStatus = true;
        } else {
          snData.fanStatus = false;
        }
    
        if (humidifierOutput > 0){
          snData.humidifierStatus = true;
        } else {
          snData.humidifierStatus = false;
        }
        
        //transmit sensor data only when temperature and humidity readings changed
        transmitSensorData();
        
        //update with new readings of temperature and humidity
        oldTemperature = snData.fltSnTemp;
        oldHumidity = snData.fltSnHum;
      }

      //reset time past to current time
      timePast = timeNow;
  }

  //check if readings from sensor are valid
  if (!isnan(snData.fltSnHum) || !isnan(snData.fltSnTemp)) {
    //calculate new PWM output to control temperature and humidity
    tempPID.Compute();
    humPID.Compute();

    //output PWM signals to fan and humidifier to control temperature and humidity
    analogWrite(fan, fanOutput);
    analogWrite(humidifier, humidifierOutput);
  }
}

// FUNCTION DECLARATIONS

//BUTTON - Read button state for changing screen displays
void readButtonState() {
    buttonState = digitalRead(pushButton);

  if (buttonState == 1) {
      buttonPressed = true;
  }

  if (buttonState == 0) {
    if (buttonPressed) {
      switch (state) {
        case 0:
          state = 1;
          break;
        case 1:
          state = 0;
          break;
      }
    }
    buttonPressed = false;
  }
}

//DISPLAY - used to display temperature and humidity screens
void displayScreen() {
  switch (state){
  case 0:
    if (changeScreen){
      temperatureDisplay();
      changeScreen = false;
    } else {
      humidityDisplay();
      changeScreen = true;
    }
  case 1:
    if (changeScreen){
      humidityDisplay(); 
    } else {
      temperatureDisplay();
    }
  }
}

//HUMIDITY DISPLAY
void temperatureDisplay() {
    fanPercentage = (fanOutput/255.00)*100;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("S:");
    lcd.print(spData.fltSPTemp);
    lcd.setCursor(9, 0);
    lcd.print("A:");
    lcd.print(snData.fltSnTemp);
    lcd.setCursor(0, 1);
    lcd.print("Fan Speed: ");
    lcd.print(int(fanPercentage));
    lcd.print("%");
}


//TEMPERATURE DISPLAY
void humidityDisplay() {
    humidifierPercentage = (humidifierOutput/255.00)*100;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("S:");
    lcd.print(spData.fltSPHum);
    lcd.setCursor(9, 0);
    lcd.print("A:");
    lcd.print(snData.fltSnHum);
    lcd.setCursor(0, 1);
    lcd.print("Humidifier: ");
    lcd.print(int(humidifierPercentage));
    lcd.print("%");
}

//TRANSMITTER - Transmit sensor readings, fan and humidifier status
void transmitSensorData() {
    radio.stopListening(); //This sets the module as transmitter
    radio.write(&snData, sizeof(SensorData)); //Sending the data
    getRadioStatus();
}

//RECEIVER - Receive temperature and humidity setpoints
void receiveSetpointData(){
  radio.startListening(); //This sets the module as receiver
  if (radio.available()) {
    radio.read(&spData, sizeof(SetpointData));
    getRadioStatus();
  }
}

//FAILURE DETECTION - Detect if there is a wireless communication malfunction
bool getRadioStatus(){
  if(radio.failureDetected){
    radio.begin();                        // Attempt to re-configure the radio with defaults
    radio.openWritingPipe(slave_address);  // Re-configure pipe addresses
    radio.openReadingPipe(0, master_address);  // Re-configure pipe addresses
    //TODO: Blink leds, send a message, etc. to indicate failure
    return false;
  } else {
    return true;
  }
  radio.failureDetected = 0;            // Reset the detection value
}
