//LIBRARIES
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <SPI.h>
#include <RF24.h>

#define PWM 9
#define FAN 3
#define DHTTYPE DHT22   // DHT 22
#define DHTPIN 12     // DHT11 Sensor Pin

DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor for normal 16mhz Arduino
RF24 radio(7, 8);  // CE, CSN - create an RF24 object
LiquidCrystal_I2C lcd(0x27, 16, 2);

//CONSTANTS
const byte master_address[6] = "00001";
const byte slave_address[6] = "00002";

struct SetpointData {
  float fltSPTemp = 80.00;
  float fltSPHum = 100.00;
};

struct SensorData {
  float fltSnTemp;
  float fltSnHum;
};

//VARIABLES
SetpointData spData;
SensorData snData;

int timer = 20000;

//FUNCTIONS
bool getRadioStatus(){ //TODO: Blink leds, send a message, etc. to indicate failure
    if(radio.failureDetected){
    radio.begin();                        // Attempt to re-configure the radio with defaults
    radio.failureDetected = 0;            // Reset the detection value
    radio.openWritingPipe(slave_address);  // Re-configure pipe addresses
    radio.openReadingPipe(0, master_address);  // Re-configure pipe addresses
    Serial.println("Failure detected!"); //TODO: Remove
    return false;
  } else {
    return true;
  }
}

void transmitSensorData() {
    radio.stopListening(); //This sets the module as transmitter
    radio.write(&snData, sizeof(SensorData)); //Sending the data
    getRadioStatus();

    //TODO: Remove
    Serial.print("TX: T: ");
    Serial.print(snData.fltSnTemp); 
    Serial.print(" H: ");
    Serial.println(snData.fltSnHum);
}

void receiveSetpointData(){
  //RECEIVER
  radio.startListening(); //This sets the module as receiver
  while ( radio.available() ) {
    radio.read(&spData, sizeof(SetpointData));
    getRadioStatus();

    //TODO: Remove
    Serial.print("RX: ");
    Serial.print(spData.fltSPTemp); 
    Serial.print(' ');
    Serial.println(spData.fltSPHum);
  }
}

void setup() {
  while (!Serial);
  Serial.begin(9600);
    
  //Start communication
  radio.begin();

  //set the address
  radio.openWritingPipe(slave_address);
  
  //set the address
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
  lcd.clear();
  Serial.begin(9600);
  dht.begin(); 
}

void controlTemperature() {
  if (snData.fltSnTemp > spData.fltSPTemp) {
     analogWrite(PWM, 255);
     lcd.print("Fan Speed: 100%   ");
   
  } else {
    analogWrite(9, 0);
    lcd.print("Fan OFF"); 
  }
}

void loop() {
  // receive setpoint data
  receiveSetpointData(); 
  
  // Introduce delay between measurements
  if (--timer == 0) {
    // Read humidity
    snData.fltSnHum = dht.readHumidity();
    
    // Read temperature
    snData.fltSnTemp = dht.readTemperature();

    // Check if any reads failed.
    if (isnan(snData.fltSnHum) || isnan(snData.fltSnTemp)) {
      //TODO: Display error message
      Serial.println("Failed to read from DHT sensor!"); 
    }
    
    // Transmit sensor readings
    transmitSensorData();

    // Control temperature according to temperature setpoint
    controlTemperature();

    // Reset delay
    timer = 20000;

    // Display sensor readings to LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(snData.fltSnTemp);
    lcd.print((char)223);
    lcd.print("C");
    lcd.setCursor(0, 1);      
  }
}
