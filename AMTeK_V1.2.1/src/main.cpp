#include <Arduino.h>

// Select your modem:
#define TINY_GSM_MODEM_SIM800 // Modem is SIM800L

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands
#define SerialAT Serial1

// // Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[] = "airtelgprs.com"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = "";
const char gprsPass[] = "";

// SIM card PIN (leave empty, if not defined)
const char simPIN[]   = "";

// MQTT details
const char* broker = "18.60.124.113";       // MQTT IP address 
// const char *broker = "3.17.163.180     ";
const char* mqttUsername = "moambulance";  // MQTT username
const char* mqttPassword = "P@$sw0rd2001"; // MQTT password
bool mqtt_status = false;

const char* mqtt_topic_gps = "amb";                        // gps data pushing topic
const char* mqtt_topic_oxy = "amb/osy/pre";                // oxygen data pushing topic
const char* mqtt_topic_alc = "amb/alc";                    // alcohol data pushing topic
String mqtt_topic_aqi = "amb/alt";                 // aqi data pushing topic


// Unique ID and MAC ID
const char *id = "";                                  /*.................Change ID Here ...............*/
const char *OwnerId = "chipl/00001";
String mac = "test2";


// MQTT retry settings
unsigned long lastRetryTime = 0;                      // Last time a message retry was attempted
const unsigned long retryInterval = 2000;             // Retry interval in milliseconds

// Hard Reset counter
int mqttAttempts = 0;


// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS

#include <TinyGsmClient.h>
#include <TinyGPS++.h>
// #include <DFRobot_ENS160.h>
#include <SoftwareSerial.h>     
#include <Adafruit_AHTX0.h>      
#include <Adafruit_NeoPixel.h>

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

/*Header Files*/
#include <ArduinoJson.h>
#include <PubSubClient.h>

TinyGsmClient client(modem);
PubSubClient mqtt(client);

// GPRS Pins
#define MODEM_TX             26
#define MODEM_RX             25
#define MODEM_RST             4

uint8_t localTimezoneOffsetHours = 5;
uint8_t localTimezoneOffsetMin = 30;

TinyGPSPlus gps;

//macro for gps
#define RXD2 16
#define TXD2 17

// Macros used for ambiance 
#define OXYGENPIN       12         // Data pin for Oxygen NeoPixel LED
#define AQIPIN          14         // Data pin for aqi NeoPixel LED 
#define NUM_OXYGEN_LED  1
#define NUM_AQI_LED     1

static const uint32_t GPSBaud = 9600;
SoftwareSerial gpsData(RXD2, TXD2);

uint32_t lastReconnectAttempt = 0;
long lastMsg = 0;

#define oxygenData1      36         //O2 Sensor Data
#define oxygenData2      39         //O2 Sensor Data

//pin for I2C communication
#define SCL             22           
#define SDA             21

//alc pin 
#define alc_pin         34

// camera control
#define ctrl            13

//Sos Switch
#define SOS             27

// Wifi led
#define wl              5

// NeoPixel declarations
Adafruit_NeoPixel oxygenLed(NUM_OXYGEN_LED, OXYGENPIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel aqiLed(NUM_AQI_LED, AQIPIN, NEO_GRB + NEO_KHZ800);

// Gps Sensor Credentials
unsigned long preTime;
unsigned long pushpreTime;

         
//Variable declaration for gps
double longitude = 0 ,latitude = 0;
float speed = 0, altitude =0;
uint16_t year = 0; 
uint8_t month = 0, day = 0;
uint8_t hour = 0, minute = 0, second = 0;
// Array to store formatted time and date
char formattedTime[10];
char formattedDate[12];

// Emergency Responses
uint8_t sos_msg = 0,tamper_msg = 0;

// messages for alcohol and air quality respectively
char msgAlert[10];
char alertMsg[10]; 
    
//oxygen sensor variables
float filteredVal1 = 450.0;  // midway starting point
float filteredVal2 = 450.0;  // midway starting point
float calsensorVal1 = 0.0;
float calsensorVal2 = 0.0;
float percalsensorVal1 = 0, percalsensorVal2 = 0;

uint8_t aind = 0; //variable for storing the alcohol value

// ENS 160 and Temp sensor variables
uint8_t AQI = 0;
uint16_t TVOC = 0,  ECO2 = 0;
float temp = 0, hum = 0;

//ENS 160
// #define I2C_COMMUNICATION  //I2C communication. Comment out this line of code if you want to use SPI communication.

// #ifdef  I2C_COMMUNICATION
//   /**
//    *   For Fermion version, the default I2C address is 0x53, connect SDO pin to GND and I2C address will be 0x52
//    */
//   DFRobot_ENS160_I2C ENS160(&Wire, /*I2CAddr*/ 0x53);
// #else
// #endif

// // temperature sensor 
Adafruit_AHTX0 aht;

/*.....Function Declarations.....*/
boolean mqttConnect();

/*Setup functions*/
void setColor(Adafruit_NeoPixel &strip, uint8_t red, uint8_t green, uint8_t blue);

/*Sensor Reading functions*/
void readOxygenSensor();
void readAlc();
// void readEns160Sensor();
// void readTempSensor();

/*Led Indication functions*/
void updateOxygenLED(float percentage);
// void updateAlcLed(uint8_t alcIndex, float temp, char* msgAlert);
// void updateAqiLed(uint8_t airIndex, char* alertMsg);


/*mqtt publishing functions*/
void publishDataGps(char* date, char* time, double latitude, double longitude, double speed, double altitude, uint8_t sos);
void publishDataOxy(float sensorVal,float sensorVal2, float filteredVal, float filteredVal2, unsigned long tm);
void publishDataAlc(uint8_t index, String msgc);
// void publishDataAqi(uint8_t voc_index, uint16_t volComp, uint16_t co2, float temp, float hum, String amsg);

/*Fail & Safe functions*/
void reconnect();

void setup() {
  // Serial.begin(115200);
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);
  pinMode(SOS, INPUT);
  pinMode(ctrl, OUTPUT);  // CAMERA control
  pinMode(wl, OUTPUT);
  
  SerialMon.println("Wait...");

  pinMode(MODEM_RST, OUTPUT);
  digitalWrite(MODEM_RST, HIGH);
  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();
  // mac = WiFi.macAddress();
  // mac.replace(":", ""); // Remove colons from MAC address
  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if ( GSM_PIN && modem.getSimStatus() != 3 ) {
    modem.simUnlock(GSM_PIN);
  }

  SerialMon.print("Connecting to APN: ");
  // modem.gprsConnect(apn, gprsUser, gprsPass);
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    ESP.restart();
  }
  else {
    SerialMon.println(" OK");
  }
  
  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
    digitalWrite(wl, HIGH);
  }

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqttConnect();
  // mqtt.setCallback(mqttCallback);


  /******GPS Begin******/
  gpsData.begin(GPSBaud);
  delay(2000);

  /******ENS 160 Initialize******/
  // if( NO_ERR != ENS160.begin() ){
  //   Serial.println("Communication with device failed, please check connection");
  //   // ens_state = false;
  //   delay(2000);
  // }else{
  //   Serial.println("Begin ok!");
  // }

  // ENS160.setPWRMode(ENS160_STANDARD_MODE);

  // /******Temperature sensor setup******/
  // if (! aht.begin()) 
  // {
  //   Serial.println("Could not find AHT? Check wiring");
  // }
  // else{
  //   Serial.println("AHT10 or AHT20 found");
  // }

  // Neopixel Led 
  oxygenLed.begin();
  oxygenLed.show();
  aqiLed.begin();
  aqiLed.show();

  // Retry MQTT connection after successful Wi-Fi reconnection
  if (mqtt_status == false)
  {
    Serial.println("Reconnecting to mqtt...");
    reconnect();
  }

}

void loop() 
{
  while (gpsData.available() > 0)
  // while (1)
  {
    // mqtt.publish("SOS", "HEllo");
    unsigned long timestamp = millis() / 1000;
    if (gps.encode(gpsData.read())) 
    {
      if (gps.location.isValid()) 
      {
        latitude =(gps.location.lat());
        latitude = round(latitude * 1000000.0) / 1000000.0;
        longitude = (gps.location.lng());
        longitude = round(longitude * 1000000.0) / 1000000.0;
      }

      if (gps.speed.isValid()) 
      {
        speed = gps.speed.kmph();
      }

      if (gps.altitude.isValid()) 
      {
        altitude = round(gps.altitude.meters());
      }

      // Date
      if (gps.date.isValid()) 
      {
        year = gps.date.year();
        month = gps.date.month();
        day = gps.date.day();

        // Format the date as a string
        sprintf(formattedDate, "%02d-%02d-%04d", day, month, year);
      }
      // Time
      if (gps.time.isValid()) 
      {
        hour = gps.time.hour() + localTimezoneOffsetHours;
        minute = gps.time.minute() + localTimezoneOffsetMin;
        second = gps.time.second();

        // Adjust hour and minute for rollover
        if (minute < 0) 
        {
          minute += 60;
          hour--;
        }
        if (minute >= 60) 
        {
          minute -= 60;
          hour++;
        }
        if (hour < 0) 
        {
          hour += 24;
        }
        if (hour >= 24) 
        {
          hour -= 24;
        }

        // Format the time as a string
        sprintf(formattedTime, "%02d:%02d:%02d", hour, minute, second);
      }
    }
    if(timestamp % 2 == 0 && (timestamp != preTime))
    {
      preTime = timestamp;
      sos_msg = digitalRead(SOS);
      // tamper_msg = digitalRead(tamper);
      Serial.print("SOS : ");
      Serial.println(sos_msg);

      // Function to read oxygen sensor
      readOxygenSensor();
      // Update the color and LCD based on the sensor value
      updateOxygenLED(percalsensorVal1);

      // Function to read alcohol sensor
      readAlc();
      // Ambiance and temp sensor readings
      // readEns160Sensor();
      // readTempSensor();              
      // Update the color and LCD based on the sensor value
      // updateAqiLed(AQI, alertMsg);

      // Update the message on the sensor value 
      // updateAlcLed(aind, temp, msgAlert);

      // MQTT Settings with Fail Safe 
      if (!client.connected())
      {
        reconnect();
      }
      else
      {
        mqttAttempts = 0;
        // publishing values 
        publishDataGps(formattedDate, formattedTime, latitude, longitude, speed, altitude, sos_msg);
      }
    }
    if(timestamp - pushpreTime > 4 )
    {
      pushpreTime = timestamp;
      // MQTT Settings with Fail Safe 
      if (!client.connected())
      {
        reconnect();
      }
      else
      {
        mqttAttempts = 0;
        // publishing values 
        publishDataOxy(calsensorVal1,calsensorVal2, percalsensorVal1, percalsensorVal2, timestamp); 
        publishDataAlc(aind, msgAlert);
        // publishDataAqi(AQI, TVOC, ECO2, temp, hum, alertMsg);
      }
    }
  }
  mqtt.loop();
}


boolean mqttConnect() 
{
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker without username and password
  //boolean status = mqtt.connect("GsmClientN");

  // Or, if you want to authenticate MQTT:
  mqtt_status = mqtt.connect("GsmClientN", mqttUsername, mqttPassword);
  if(mqtt_status == true)
  {
    Serial.print(mqtt_status);
    setColor(aqiLed, 0, 0, 255);
  }

  if (mqtt_status == false) {
    SerialMon.println(" fail");
    ESP.restart();
    // return false;connect
  }
  // SerialMon.println(" success");
  // mqtt.subscribe(topicOutput1);
  // mqtt.subscribe(topicOutput2);

  return mqtt.connected();
}

void setColor(Adafruit_NeoPixel &strip, uint8_t red, uint8_t green, uint8_t blue)
{
  // Check which strip to update based on the provided strip reference
  if (&strip == &oxygenLed)
  {
    // Oxygen LED indication
    oxygenLed.setPixelColor(0, oxygenLed.Color(red, green, blue));
    oxygenLed.show();
  }
  else if (&strip == &aqiLed)
  {
    // AQI LED indication
    aqiLed.setPixelColor(0, aqiLed.Color(red, green, blue));
    aqiLed.show();
  }
}

// Function to read Oxygen Cylinder
void readOxygenSensor() {
  float sensorVal1 = 0.0, sensorVal2 = 0.0, sum1 = 0.0, sum2 = 0.0;
  int offsetVal = 430;
  const float alpha = 0.17;   // Low Pass Filter alpha (0 - 1)
  // Oxygen sensor Readings
  for (int k = 0; k < 10; k++) {
    sensorVal1 = (float)analogRead(oxygenData1);  // Read pressure sensor val
    sensorVal2 = (float)analogRead(oxygenData2);  // Read pressure sensor val
    sum1 += sensorVal1;
    sum2 += sensorVal2;
    delay(50);
  }
  sensorVal1 = sum1 / 10; 
  sensorVal2 = sum2 / 10;
  calsensorVal1 = sensorVal1 - offsetVal;                                               // calibrated sensor value
  calsensorVal2 = sensorVal2 - offsetVal;  
  filteredVal1 = ((alpha * filteredVal1) + ((1.0 - alpha) * (calsensorVal1)));               // Low Pass Filter
  filteredVal2 = ((alpha * filteredVal2) + ((1.0 - alpha) * (calsensorVal2)));               // Low Pass Filter
  // calsensorVal = filteredVal - offsetVal;
  // percalsensorVal = map(filteredVal * 1000, 0, 20500, 0, 100) / 100.0;                
  percalsensorVal1 = (float)((filteredVal1/2100) * 100);                                 // Convert calsensorVal to percentage
  percalsensorVal2 = (float)((filteredVal2/2100) * 100);                                 // Convert calsensorVal to percentage

  // percalsensorVal1 = 0.00;
  // percalsensorVal2 = 0.00;

  Serial.print("raw value :  ");
  Serial.println(calsensorVal1);
  Serial.print("filtered value :  ");
  Serial.println(filteredVal1);
  Serial.print("Value in Percentage");
  Serial.print(percalsensorVal1);
  Serial.println("%");
  Serial.print("raw value :  ");
  Serial.println(calsensorVal2);
  Serial.print("filtered value :  ");
  Serial.println(filteredVal2);
  Serial.print("Value in Percentage");
  Serial.print(percalsensorVal2);
  Serial.println("%");
}

// Oxygen Indication 
void updateOxygenLED(float percentage)
{
  if (percentage >= 60 && percentage <= 120)
  {
    setColor(oxygenLed, 0, 255, 0); //Green
  }
  else if (percentage >= 30)
  {
    setColor(oxygenLed, 255, 255, 0); // Change the color to yellow (red + green)
  }
  else if (percentage >= 0)
  {
    setColor(oxygenLed, 255, 0, 0); // Red
  }
  else
  {
    setColor(oxygenLed, 0, 0, 255); // Blue
  }

}

//Alcohol data read 
void readAlc()
{
  aind = analogRead(alc_pin);
  // *index = random(700,720);
  Serial.print("Alc Val: "); // alcohol 
  Serial.println(aind);
}

// void updateAlcLed(uint8_t alcIndex, float temp, char* msgAlert)
// {
//   if(temp >= 38.00)
//   {
//     if(alcIndex > 2700){
//     strcpy(msgAlert, "Bad");
//     }else if(alcIndex > 2200 ){
//       strcpy(msgAlert, "Avg");
//     }else if(alcIndex >= 0){
//       strcpy(msgAlert, "Good");
//     }
//     else{
//       strcpy(msgAlert, "Failed");
//     }
//   }else
//   {
//     if(alcIndex > 1500){
//     strcpy(msgAlert, "Bad");
//     }else if(alcIndex > 1200 ){
//       strcpy(msgAlert, "Avg");
//     }else if(alcIndex >= 0){
//       strcpy(msgAlert, "Good");
//     }
//     else{
//       strcpy(msgAlert, "Failed");
//     }
//   }
// }

//Function for Read the ENS160 
// void readEns160Sensor()
// {
//   uint8_t Status = ENS160.getENS160Status();
//   Serial.print("Sensor operating status : ");
//   Serial.println(Status);

//   /**
//    * Get the air quality index
//    * Return value: 1-Excellent, 2-Good, 3-Moderate, 4-Poor, 5-Unhealthy
//    */
//   AQI = ENS160.getAQI();
//   Serial.print("Air quality index : ");
//   Serial.println(AQI);
//   // *AQI = random(1,2);

//   TVOC = ENS160.getTVOC();
//   Serial.print("Concentration of total volatile organic compounds : ");
//   Serial.print(TVOC);
//   Serial.println(" ppb");
//   // *TVOC = random(100,150);

//   /**
//    * Get CO2 equivalent concentration calculated according to the detected data of VOCs and hydrogen (eCO2 – Equivalent CO2)
//    * Return value range: 400–65000, unit: ppm
//    * Five levels: Excellent(400 - 600), Good(600 - 800), Moderate(800 - 1000), 
//    *               Poor(1000 - 1500), Unhealthy(> 1500
//    */
//   ECO2 = ENS160.getECO2();
//   Serial.print("Carbon dioxide equivalent concentration : ");
//   Serial.print(ECO2);
//   Serial.println(" ppm");
//   // *ECO2 = random(500,550);

//   Serial.println();
//   // delay(1000);
// }

// Function to read temp sensor
void readTempSensor()
{
  sensors_event_t humidity, tempm;
  aht.getEvent(&humidity, &tempm); // populate temp and humidity objects with fresh data

  temp = (tempm.temperature) - 3;
  hum = humidity.relative_humidity;
  // *temp = 27;
  // *hum = 60;
  Serial.println(temp);
  Serial.println(hum);

}

// Air Index Indication
// void updateAqiLed(uint8_t airIndex, char* alertMsg)
// {
//   if (airIndex == 5)
//   {
//     strcpy(alertMsg, "Bad");
//     setColor(aqiLed, 255, 0, 0); //Red
//   }
//   else if (airIndex > 2 && airIndex <=4 )
//   {
//     strcpy(alertMsg, "Avg");
//     setColor(aqiLed, 255, 255, 0); //Yellow
//   }
//   else if (airIndex >= 0 && airIndex <=2 )
//   {
//     strcpy(alertMsg, "Good");
//     setColor(aqiLed, 0, 255, 0); //Green
//   }
//   else
//   {
    
//     setColor(aqiLed, 0, 0, 255); // Blue
//     strcpy(alertMsg, "Failed");
//   }
// }


//gps data publish to mqtt
void publishDataGps(char* date, char* time, double latitude, double longitude, double speed, double altitude, uint8_t sos)
{
  // Build the JSON object
  JsonDocument jsonDocumentGps;
  // jsonDocumentGps["OwnerId"] = OwnerId;
  jsonDocumentGps["ID"] = mac;
  // jsonDocumentGps["MAC"] = mac;
  jsonDocumentGps["date"] = date;
  jsonDocumentGps["time"] = time;
  jsonDocumentGps["latitude"] = latitude;
  jsonDocumentGps["longitude"] = longitude;
  jsonDocumentGps["speed"] = speed;
  jsonDocumentGps["altitude"] = altitude;
  jsonDocumentGps["sos"] = sos;
  // jsonDocumentGps["tamper"] = tamp;
  // Serialize the JSON object to a string
  String jsonStringGps;
  serializeJson(jsonDocumentGps, jsonStringGps);

  String tempTopic = mqtt_topic_gps;
  if (mqtt.publish(tempTopic.c_str(), jsonStringGps.c_str()))
  {
    Serial.println(F("Message sent to MQTT topic"));
    lastRetryTime = 0; // Reset retry timer on successful message send
  }
  else
  {
    // Failed to send message, check for retry interval
    unsigned long currentMillis = millis();
    if (currentMillis - lastRetryTime >= retryInterval)
    {
      Serial.println(F("Failed to send message to MQTT topic. Retrying..."));
      lastRetryTime = currentMillis;
    }
  }
}

// Oxygen Data Publish to mqtt
void publishDataOxy(float sensorVal, float sensorVal2, float filteredVal, float filteredVal2, unsigned long tm)
{
  // Get current timestamp in milliseconds
  // Build the JSON object
  JsonDocument jsonDocumentOxy;
  jsonDocumentOxy["ID"] = mac;
  // jsonDocumentOxy["MAC"] = mac;
  jsonDocumentOxy["sensorVal"] = sensorVal;
  jsonDocumentOxy["filteredVal"] = filteredVal;
  jsonDocumentOxy["sensorVal2"] = sensorVal2;
  jsonDocumentOxy["filteredVal2"] = filteredVal2;
  jsonDocumentOxy["time"] = tm;
  // Serialize the JSON object to a string
  String jsonStringOxy;
  serializeJson(jsonDocumentOxy, jsonStringOxy);

  String tempTopic = mqtt_topic_oxy;
  if (mqtt.publish(tempTopic.c_str(), jsonStringOxy.c_str()))
  {
    Serial.println(F("Message sent to MQTT topic"));
    lastRetryTime = 0; // Reset retry timer on successful message send
  }
  else
  {
    // Failed to send message, check for retry interval
    unsigned long currentMillis = millis();
    if (currentMillis - lastRetryTime >= retryInterval)
    {
      Serial.println(F("Failed to send message to MQTT topic. Retrying..."));
      lastRetryTime = currentMillis;
    }
  }
}

//Alcohol data publish to mqtt
void publishDataAlc(uint8_t index, String msgc)
{
  // Get current timestamp in milliseconds
  // Build the JSON object
  JsonDocument jsonDocumentAlc;
  jsonDocumentAlc["ID"] = mac;
  // jsonDocumentAlc["MAC"] = mac;
  jsonDocumentAlc["Alert"] = msgc;
  jsonDocumentAlc["Index"] = index;
  // Serialize the JSON object to a string
  String jsonStringAlc;
  serializeJson(jsonDocumentAlc, jsonStringAlc);

  String tempTopic = mqtt_topic_alc;
  if (mqtt.publish(tempTopic.c_str(), jsonStringAlc.c_str()))
  {
    Serial.println(F("Message sent to MQTT topic"));
    lastRetryTime = 0; // Reset retry timer on successful message send
  }
  else
  {
    // Failed to send message, check for retry interval
    unsigned long currentMillis = millis();
    if (currentMillis - lastRetryTime >= retryInterval)
    {
      Serial.println(F("Failed to send message to MQTT topic. Retrying..."));
      lastRetryTime = currentMillis;
    }
  }
}

// Ambiance Data Publish to mqtt 
// void publishDataAqi(uint8_t voc_index, uint16_t volComp, uint16_t co2, float temp, float hum, String amsg)
// {
//   // Clear the JSON document before adding new data
//   // jsonDocument.clear();
//   JsonDocument jsonDocumentAqi;
//   jsonDocumentAqi["OwnerId"] = OwnerId;
//   jsonDocumentAqi["ID"] = mac;
//   // jsonDocumentAqi["MAC"] = mac;
//   jsonDocumentAqi["Alert"] = amsg;
//   jsonDocumentAqi["Index"] = voc_index;
//   jsonDocumentAqi["Voc"] = volComp;
//   jsonDocumentAqi["Eco2"] = co2;
//   jsonDocumentAqi["Temperature"] = temp;
//   jsonDocumentAqi["Humidity"] = hum;

//   String jsonStringAqi;
//   serializeJson(jsonDocumentAqi, jsonStringAqi);

//   String tempTopic = mqtt_topic_aqi;
//   if (mqtt.publish(tempTopic.c_str(), jsonStringAqi.c_str()))
//   {
//     Serial.println(F("Message sent to MQTT topic"));
//     lastRetryTime = 0; // Reset retry timer on successful message send
//   }
//   else
//   {
//     // Failed to send message, check for retry interval
//     unsigned long currentMillis = millis();
//     if (currentMillis - lastRetryTime >= retryInterval)
//     {
//       Serial.println(F("Failed to send message to MQTT topic. Retrying..."));
//       lastRetryTime = currentMillis;
//     }
//   }
// }

// Reconnect function
void reconnect()
{
  digitalWrite(wl, LOW);
  String clientId = "GsmClientN" + String(random(0xffff), HEX);
  Serial.print("Attempting MQTT connection...");
  mqtt_status = mqtt.connect("GsmClientN", mqttUsername, mqttPassword);
  if (mqtt_status == true)
  {
    digitalWrite(wl, HIGH);
    Serial.println(F("connected"));
  }
  else
  {
    digitalWrite(wl, LOW);
    Serial.print(F("failed, rc="));
    Serial.print(mqtt.state());
    Serial.println(F(" try again in 2 seconds"));
    mqttAttempts++;
    Serial.print(mqttAttempts);
    Serial.println(F(" time attempted"));
    if (mqttAttempts == 1200)
    {
      Serial.println(F("Restarting the ESP.........."));
      ESP.restart();
    }
    delay(1000);
  }
}

