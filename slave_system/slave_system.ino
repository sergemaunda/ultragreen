//LIBRARIES
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <SPI.h>
#include <RF24.h>
#include <PID_v1.h>

#define DHTPIN 12     // DHT11 Sensor Pin
#define led1 2
#define fan 10
#define humidifier 9
#define DHTTYPE DHT22   // DHT 22 

//CONSTANTS
//address through which two modules communicate.
const byte master_address[6] = "00001";
const byte slave_address[6] = "00002";

// Temperature Gains
double Kp_t = 6.00; //proportional gain
double Ki_t = 2.00; //integral gain
double Kd_t = 5.00; //derivative gain

// Humidity Gains
double Kp_h = 90.00; //proportional gain
double Ki_h = 1.00; //integral gain
double Kd_h = 1.00; //derivative gain

struct SetpointData {
  double fltSPTemp;
  double fltSPHum;
};

struct SensorData {
  double fltSnTemp;;
  double fltSnHum;
};

//VARIABLES
SetpointData spData;
SensorData snData;

double fanOutput, fanPercentage;
double humidifierOutput, humidifierPercentage;
float oldTemperature, oldHumidity;
long timePast;
long timeNow;
bool changeScreen = true;

//INSTANCES
//Temperature Control - Cascade PID Controller
PID tempPID(&snData.fltSnTemp, &fanOutput, &spData.fltSPTemp, Kp_t, Ki_t, Kd_t, REVERSE);
PID humPID(&snData.fltSnHum, &humidifierOutput, &spData.fltSPHum, Kp_h, Ki_h, Kd_h, DIRECT);
DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor for normal 16mhz Arduino
RF24 radio(7, 8);  // CE, CSN - create an RF24 object
LiquidCrystal_I2C lcd(0x27, 16, 2);


bool getRadioStatus(){ //TODO: Blink leds, send a message, etc. to indicate failure
  if(radio.failureDetected){
    radio.begin();                        // Attempt to re-configure the radio with defaults
    radio.openWritingPipe(slave_address);  // Re-configure pipe addresses
    radio.openReadingPipe(0, master_address);  // Re-configure pipe addresses
    Serial.println("Failure detected!"); //TODO: Remove
    return false;
  } else {
    return true;
  }
  radio.failureDetected = 0;            // Reset the detection value
}

void transmitSensorData() {

    radio.stopListening(); //This sets the module as transmitter
    radio.write(&snData, sizeof(SensorData)); //Sending the data

    if (getRadioStatus()) {
      // SERIAL MONITOR
//      Serial.print("Sensor Temperature: ");
//      Serial.print(snData.fltSnTemp); 
//      Serial.print(" | Sensor Humidity: ");
//      Serial.println(snData.fltSnHum);
//      Serial.println(); 
    }
}

void receiveSetpointData(){
  //RECEIVER
  radio.startListening(); //This sets the module as receiver
  if (radio.available()) {
    radio.read(&spData, sizeof(SetpointData));

    if (getRadioStatus()) {
      //SETPOINT
//      Serial.print("Setpoint Temperature: ");
//      Serial.println(spData.fltSPTemp); 
//      Serial.print("Setpoint Humidity: ");
//      Serial.println(spData.fltSPHum);
      // SENSOR READING
//      Serial.print("Sensor Temperature: ");
//      Serial.println(snData.fltSnTemp); 
//      Serial.print("Sensor Humidity: ");
//      Serial.println(snData.fltSnHum);
      // OUTPUT
//      Serial.print("Fan Output: ");
//      Serial.print(fanPercentage);
//      Serial.println("%");
//      Serial.print("Humidifier Output: ");
//      Serial.print(humidifierPercentage);
//      Serial.println("%");
//      Serial.println();

      //SERIAL PLOTTER
      // TEMPERATURE
      Serial.print(spData.fltSPTemp); //orange
      Serial.print(" ");
      Serial.println(snData.fltSnTemp); // blue
      Serial.println("Setpoint_Temperature Actual_Temperature");
      // HUMIDITY
//      Serial.print(spData.fltSPHum); //orange
//      Serial.print(" ");
//      Serial.println(snData.fltSnHum); // blue
//      Serial.println("Setpoint_Humidity Actual_Humidity");
      
    }
  }
  
}

void setup() {
  while (!Serial);
  Serial.begin(9600);

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
  
  spData.fltSPTemp = 80.00;
  spData.fltSPHum = 0.00;
//  snData.fltSnHum = dht.readHumidity();
//  snData.fltSnTemp = dht.readTemperature();
  
  
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
  
  dht.begin(); 
}

void displayTemperature() {
    fanPercentage = (fanOutput/255.00)*100;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("S:");
    lcd.print(spData.fltSPTemp);
    lcd.setCursor(9, 0);
    lcd.print("A:");
    lcd.print(snData.fltSnTemp);
//    lcd.print((char)223);
//    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Fan Speed: ");
    lcd.print(int(fanPercentage));
    lcd.print("%");
}

void displayHumidity() {
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

void loop() {
  // receive setpoint data
  receiveSetpointData();
 
  timeNow = millis();
  // Wait a 2 second between measurements.
    if (timeNow - timePast >= 2000) {
      
      // read new temperature and humidity
      snData.fltSnHum = dht.readHumidity();
      snData.fltSnTemp = dht.readTemperature();

      // check if temperature and humidity readings changed
      if (snData.fltSnTemp != oldTemperature || snData.fltSnHum != oldHumidity ) {
        // transmit sensor data only when temperature and humidity readings changed
        transmitSensorData();
        
        // update with new readings of temperature and humidity
        oldTemperature = snData.fltSnTemp;
        oldHumidity = snData.fltSnHum;
      }
   
      displayTemperature();
//      displayHumidity();
      //TODO: Replace with push button to change screens
//      if (changeScreen){
//        displayTemperature();
//        changeScreen = false;
//      } else {
//        displayHumidity();
//        changeScreen = true;
//      }
      
      timePast = timeNow;
  }

  if (!isnan(snData.fltSnHum) || !isnan(snData.fltSnTemp)) {
    // calculate new PWM output 
    tempPID.Compute();
    humPID.Compute();
  
    analogWrite(fan, fanOutput);
    analogWrite(humidifier, humidifierOutput);
  }
}
