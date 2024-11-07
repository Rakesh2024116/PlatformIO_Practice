#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <HTTPClient.h>
#include <ESP32httpUpdate.h>
#include <LittleFS.h> 
#include <PNGdec.h>
#include "Main_screen.h"
#include <Adafruit_NeoPixel.h>
#include <TinyGPS++.h>
#include <PubSubClient.h> 
#include <Adafruit_AHTX0.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Kalman.h"
#include <DFRobot_ENS160.h>
#include <ArduinoJson.h>

/*LCD Settings*/
PNG png; // PNG decoder instance

#define MAX_IMAGE_WIDTH 240 // Adjust for your images

uint8_t xpos = 0;
uint8_t ypos = 0;

// #include "SPI.h"
#include <TFT_eSPI.h>              // Hardware-specific library
TFT_eSPI tft = TFT_eSPI();         // Invoke custom library

// temperature sensor 
Adafruit_AHTX0 aht;

//macro for gps
#define RXD2 16
#define TXD2 17

// Macros used for ambiance 
#define OXYGENPIN       12         // Data pin for Oxygen NeoPixel LED
// #define ALCPIN          18         // Data pin for alcohol NeoPixel LED
#define AQIPIN          14         // Data pin for aqi NeoPixel LED 
#define NUM_OXYGEN_LED  1
#define NUM_AQI_LED     1

#define oxygenData1      36         //O2 Sensor Data
#define oxygenData2      39         //O2 Sensor Data

//pin for I2C communication
#define SCL             22           
#define SDA             21 

//alc pin 
#define alc_pin         34

//Sos Switch
#define SOS             27
// #define tamper          26

// camera control
#define ctrl            13  

// Wifi led
#define wl              5

// Firmware version
#define CURRENT_FIRMWARE_VERSION "1.0.0"                                        /*............Version..........*/

#define RAD_TO_DEG      57.2958


/* ...................WiFi settings...................*/

const char *ssid = "MO AMBULANCE";
const char *password = "CHIPL_2024";


// Unique ID and MAC ID
const char *id = "";                                                         
const char *OwnerId = "Chipl";               // Change Owner ID here
String mac;


/*..................mqtt settings........................*/


// const char *mqtt_server = "18.60.124.113";                                      //MQTT IP
const char *mqtt_server = "3.17.163.180";                                       //MQTT IP
// const char *mqtt_server = "98.130.28.156";                                      //MQTT IP
const char *mqttUsername = "moambulance";                                       // MQTT username
const char *mqttPassword = "P@$sw0rd2001";                                      // MQTT password
const char* mqtt_topic_gps = "amb";                                             // gps data pushing topic
const char* mqtt_topic_oxy = "amb/osy/pre";                                     // oxygen data pushing topic
const char* mqtt_topic_alc = "amb/alc";                                         // alcohol data pushing topic
const char* mqtt_topic_aqi = "amb/alt";                                         // aqi data pushing topic
const char* mqtt_topic_acc = "amb/acctest";                                  // gyrometer data pushing topic/*.................Change topic Here ...............*/
const char* mqtt_topic_odata = "amb/offdata";                                   // Offline data pushing topic
const char* mqtt_topic_sensor1 = "amb/sensor1";                                 // sensor State data pushing topic
const char* sub_topic = "mac/control";                                         // Unique topic for camera control ( to be updated )
const char* offlineData = "/Data.json";

WiFiClient espClient;
PubSubClient client(espClient);


//ENS 160 initialization
#define I2C_COMMUNICATION  //I2C communication. Comment out this line of code if you want to use SPI communication.

#ifdef  I2C_COMMUNICATION
  /**
   *   For Fermion version, the default I2C address is 0x53, connect SDO pin to GND and I2C address will be 0x52
   */
  DFRobot_ENS160_I2C ENS160(&Wire, /*I2CAddr*/ 0x53);
#else
#endif

// NeoPixel declarations
Adafruit_NeoPixel oxygenLed(NUM_OXYGEN_LED, OXYGENPIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel aqiLed(NUM_AQI_LED, AQIPIN, NEO_GRB + NEO_KHZ800);

// Gps Sensor Credentials
HardwareSerial neogps(1);
unsigned long preTime;
unsigned long pushpreTime;

TinyGPSPlus gps;

//Variable declaration for gps
double longitude = 0 ,latitude = 0;
float speed = 0, altitude =0;
uint8_t localTimezoneOffsetHours = 5;
uint8_t localTimezoneOffsetMin = 30;
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
// Oxygen Variables
float calsensorVal1 = 0.0;
float calsensorVal2 = 0.0;
float percalsensorVal1 = 0, percalsensorVal2 = 0;

uint16_t aind = 0; //variable for storing the alcohol value

// ENS 160 and Temp sensor variables
uint8_t AQI = 0;
uint16_t TVOC = 0,  ECO2 = 0;
float temp = 0, hum = 0;

// MPU6050 variables and Initializations
Adafruit_MPU6050 mpu; //Mpu6050
Kalman kalmanX, kalmanY, kalmanZ; // Kalman filters for each axis

// Variables for Kalman filter output
double kalAngleX, kalAngleY, kalAngleZ; 
double lastKalAngleX, lastKalAngleY, lastKalAngleZ;
double jerkX, jerkY, jerkZ;

// Timing
unsigned long lastUpdateTime = 0;

float jerkMagnitude = 0.00; // Jerking from Acceleration

// Counters
uint32_t sumAlc = 0; 
uint8_t alcCount = 0;

// MQTT retry settings
unsigned long lastRetryTime = 0;          // Last time a message retry was attempted
const unsigned long retryInterval = 2000; // Retry interval in milliseconds

// Hard Reset counterpublishDataState
uint8_t mqttAttempts = 0;

unsigned long previousMillis = 0;       // Store the last time data was written
const long timeInterval = 300;           // time interval for publishing offline data in seconds(5min)



/*.....Function Declarations.....*/


/*Setup functions*/
void setup_wifi();
void setColor(Adafruit_NeoPixel &strip, uint8_t red, uint8_t green, uint8_t blue);
void checkForOtaUpdate();

/*Sensor Reading functions*/
void readOxygenSensor();
void readAlc();
void readEns160Sensor();
void readTempSensor();
void readMPU6050();

/*Led Indication functions*/
void updateOxygenLED(float percentage);
void updateAlcLed(uint8_t alcIndex, float temp, char* msgAlert);
void updateAqiLed(uint8_t airIndex, char* alertMsg);

/******LCD Function******/
// void displayMainScreen();
void updateLcdContent(int x, int y, String content);

/*mqtt publishing functions*/
void publishDataGps(char* date, char* time, double latitude, double longitude, double speed, double altitude, uint8_t sos);
void publishDataOxy(float sensorVal,float sensorVal2, float filteredVal, float filteredVal2, unsigned long tm);
void publishDataAlc(uint8_t index, String msgc, float jerka);
void publishDataAqi(uint8_t voc_index, uint16_t volComp, uint16_t co2, float temp, float hum, String amsg);
void publishDataMpu(double jerkX, double jerkY, double jerkZ, float jerkAcc);
// void publishDataState(char* wifi, char* gps, char* alc, char* ens, char* sht, char* mpu);

/*Storage functions with resend*/
String offlineStoreData(String mac, char* date, char* time, double latitude, double longitude, float filteredVal, float filteredVal2, uint8_t index, uint8_t AQI, float temp);
void storeData(const char* path, const String &data);
void readAndPrintStoredData(const char* path);
void deleteFile(fs::FS &fs, const char *path);
bool checkForFile(fs::FS &fs, const char *path);
bool republishStoredData(const char* path, const char* reptopic);
void fileSystem();

/*Fail & Safe functions*/
void pngDraw(PNGDRAW *pDraw);
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void reconnectWifi();


void setup()
{
    Serial.begin(115200);

    // Initialize the LittleFS file system
    if(!LittleFS.begin()){
        Serial.println("An error has occurred while mounting LittleFS. Formatting...");
        if (LittleFS.format()) {
        Serial.println("Filesystem formatted successfully. Re-mounting...");
        if (LittleFS.begin()) {
            Serial.println("LittleFS mounted successfully after formatting.");
        } else {
            Serial.println("Failed to mount LittleFS after formatting.");
            return;
        }
        } else {
        Serial.println("Failed to format LittleFS.");
        return;
        }
    }

    Wire.begin(SDA,SCL);//Initialize Wire for SGP40
    pinMode(SOS, INPUT);
    // pinMode(tamper, INPUT);
    pinMode(ctrl, OUTPUT); // CAMERA control
    pinMode(wl, OUTPUT);
    Serial.println("Booting");
    // dht.begin(); // Initialize the dht.


    /******GPS Begin******/
    neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
    delay(2000);


    /******WiFi and MQTT******/
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.subscribe(sub_topic);
    client.setCallback(callback);
    mac = WiFi.macAddress();
    mac.replace(":", ""); // Remove colons from MAC address
    // mac = "test";                                                 /*Comment this while final code upload*/


    // Check for OTA updates
    checkForOtaUpdate();


    /******ENS 160 Initialize******/
    if( NO_ERR != ENS160.begin() ){
        Serial.println("Communication with device failed, please check connection");
        delay(2000);
    }else{
        Serial.println("Begin ok!");
    }

    ENS160.setPWRMode(ENS160_STANDARD_MODE);


    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens


    /******Temperature sensor setup******/
    if (! aht.begin()) 
    {
        Serial.println("Could not find AHT? Check wiring");
        // while (1) delay(10);
    }
    else{
        Serial.println("AHT10 or AHT20 found");
    }
    // Serial.println("Adafruit MPU6050 test!");

    /******MPU6050 setup******/
    // // Try to initialize!
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        // while (1) {
        //   delay(10);
        // }
    }else{
        Serial.println("MPU6050 Found!");
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

    mpu.setGyroRange(MPU6050_RANGE_500_DEG);

    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

    // Initialize Kalman filters
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Compute the initial angles from accelerometer readings
    double initialAngleX = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
    double initialAngleY = atan2(a.acceleration.x, a.acceleration.z) * RAD_TO_DEG;
    double initialAngleZ = atan2(a.acceleration.y, a.acceleration.x) * RAD_TO_DEG;

    // Initialize Kalman filters with initial angles
    kalmanX.setAngle(initialAngleX);
    kalmanY.setAngle(initialAngleY);
    kalmanZ.setAngle(initialAngleZ);

    lastKalAngleX = initialAngleX;
    lastKalAngleY = initialAngleY;
    lastKalAngleZ = initialAngleZ;

    lastUpdateTime = millis();

    Serial.println("");
    delay(100);


    /******LCD begin and setup******/
    tft.begin();
    tft.fillScreen(TFT_BLACK);
    /*LCD Dispaly*/

    int16_t rc5 = png.openFLASH((uint8_t *)Main_screen, sizeof(Main_screen), pngDraw);
    if (rc5 == PNG_SUCCESS) {
    Serial.println("Successfully opened png file");
    Serial.printf("image specs: (%d x %d), %d bpp, pixel type: %d\n", png.getWidth(), png.getHeight(), png.getBpp(), png.getPixelType());
    tft.startWrite();
    uint32_t dt5 = millis();
    rc5 = png.decode(NULL, 0);
    Serial.print(millis() - dt5); Serial.println("ms");
    tft.endWrite();
    // png.close(); // not needed for memory->memory decode
    }
    delay(2000);

    // Retry MQTT connection after successful Wi-Fi reconnection
    if (WiFi.status() == WL_CONNECTED && !client.connected())
    {
    for(int j= 0 ; j<10;j++){
        reconnect();
    }
    }


    oxygenLed.begin();
    oxygenLed.show();
    aqiLed.begin();
    aqiLed.show();

    // Check file system  
    fileSystem();

    delay(2000);
}

void loop()
{
  //gps program
  // TimeStamp
  boolean newData = false;
  //ps_val = digitalRead(postSwitch);

  while (neogps.available())
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      digitalWrite(wl, LOW);
    }
    else
    {
      digitalWrite(wl, HIGH);
    }

    unsigned long timestamp = millis() / 1000; // Time in second 
    if (gps.encode(neogps.read())) 
    {
      latitude =(gps.location.lat());
      latitude = round(latitude * 1000000.0) / 1000000.0;
      longitude = (gps.location.lng());
      longitude = round(longitude * 1000000.0) / 1000000.0;
      speed = gps.speed.kmph();
      altitude = round(gps.altitude.meters());
      // Date
      if (gps.date.isValid()) {
        year = gps.date.year();
        month = gps.date.month();
        day = gps.date.day();

        // Format the date as a string
        sprintf(formattedDate, "%04d-%02d-%02d", year, month, day);
      }
      // Time
      if (gps.time.isValid()) {
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
      Serial.print("TimeStamp : ");
      Serial.println(timestamp);
      sos_msg = digitalRead(SOS);
      // tamper_msg = digitalRead(tamper);
      Serial.print("SOS : ");
      Serial.println(sos_msg);
      // Serial.print("Tamperd Signal : ");
      // Serial.println(tamper_msg);
      delay(50);  // Debounce delay

      // Function to read oxygen sensor
      readOxygenSensor();
      // Update the color and LCD based on the sensor value
      updateOxygenLED(percalsensorVal1);
      if(percalsensorVal1<=0.00)
      {
        tft.fillRect(105, 180, 70, 15, ILI9341_WHITE);
        updateLcdContent(105, 180, String(0.00) + "%");
      }else if(percalsensorVal1>=100.00)
      {
        tft.fillRect(105, 180, 70, 15, ILI9341_WHITE);
        updateLcdContent(105, 180, String(100.00) + "%");
      }else
      {
        tft.fillRect(105, 180, 70, 15, ILI9341_WHITE);
        updateLcdContent(105, 180, String(percalsensorVal1) + "%");
      }

      // 2nd Oxygen 
      // updateOxygenLED(percalsensorVal2);
      if(percalsensorVal2<=0.00)
      {
        tft.fillRect(105, 240, 70, 15, ILI9341_WHITE);
        updateLcdContent(105, 240, String(0.00) + "%");
      }else if(percalsensorVal2>=100.00)
      {
        tft.fillRect(105, 240, 70, 15, ILI9341_WHITE);
        updateLcdContent(105, 240, String(100.00) + "%");
      }else
      {
        tft.fillRect(105, 240, 70, 15, ILI9341_WHITE);
        updateLcdContent(105, 240, String(percalsensorVal2) + "%");
      }
      // Function to read alcohol sensor
      readAlc();
      // Ambiance and temp sensor readings
      readEns160Sensor();
      readTempSensor();              
      // Update the message on the sensor value 
      updateAlcLed(aind, temp, msgAlert);
      // Update the color and LCD based on the sensor value
      updateAqiLed(AQI, alertMsg);
      tft.fillRect (0, 50, 240, 20, ILI9341_WHITE);
      updateLcdContent(8, 55, String(alertMsg));
      updateLcdContent(100, 50, String(ECO2));
      updateLcdContent(175, 50, String(TVOC));
      tft.fillRect (40, 115, 72, 30, ILI9341_WHITE);
      updateLcdContent(40, 115, String(temp) + "C");
      tft.fillRect (160, 115, 72, 30, ILI9341_WHITE);
      updateLcdContent(160, 115, String(hum) + "%");
      /* MPU6050 readings */
      readMPU6050(); 

      // if (ps_val == 1)
      // {
      // }

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

      if ((WiFi.status() != WL_CONNECTED))
      {
        Serial.println("..................................WiFi Connection Lost..................................................");
        reconnectWifi();
        if (!client.connected()) 
        {
          Serial.println("..................................MQTT Connection Lost..................................................");
          unsigned long currentMillis = timestamp;
          Serial.print("Current Time : ");
          Serial.println(currentMillis);
          if (currentMillis - previousMillis >= timeInterval)
          {
            previousMillis = currentMillis;
            Serial.print("Previous Time : ");
            Serial.println(previousMillis);
            Serial.println(".........................................................................");
            String buildStoreData = offlineStoreData(mac, formattedDate, formattedTime, latitude, longitude, percalsensorVal1, percalsensorVal2, aind, AQI, temp);
            storeData(offlineData, buildStoreData + "\n");
            if (Serial.available() > 0)
            {
              char cmd = Serial.read();
              if (cmd == 'r')
              {
                readAndPrintStoredData(offlineData);
                // Terminate the code
                Serial.println("Data read. Code terminated.");
                while (true) {} // Infinite loop to keep the code halted
              } 
              else 
              {
                Serial.println("Please enter 'r' to read stored data and terminate.");
              }
            }
          }
        }
      }   
      else
      {
        // reconnect();
        if (checkForFile(LittleFS, offlineData))
        {
          if(republishStoredData(offlineData, mqtt_topic_odata))
          {
            Serial.println("....Data republish success....");
            // deleteFile(LittleFS ,offlineData);
          }
          else
          {
            Serial.println("....Data republish Failed....");
          }
        }
      }
    }
    if(timestamp % 30 == 0 && (timestamp != pushpreTime))
    {
      pushpreTime = timestamp;

      // Alcohol value averaging 
      sumAlc += aind;
      aind = sumAlc/alcCount;
      alcCount = 0;

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
        publishDataAlc(aind, msgAlert, jerkMagnitude);
        publishDataAqi(AQI, TVOC, ECO2, temp, hum, alertMsg);
        publishDataMpu(jerkX, jerkY, jerkZ, jerkMagnitude);  
      }
    }
  }
  client.loop();
}

void callback(char* topic, byte* payload, unsigned int length) 
{
    Serial.println("MQTT message received");
    if (strcmp(topic, sub_topic) == 0) 
    {
      int trig = payload[0] - '0'; // Convert from ASCII to integer
      Serial.println(trig);
      if(trig == 1)
      {
        digitalWrite(ctrl, HIGH);
        Serial.println(ctrl);
        // delay(3000);
      }else if(trig == 0)
      {
        digitalWrite(ctrl, LOW);
        Serial.println(ctrl);
      }
    }
}

// Function Definitions

void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  int attempts = 0;
  bool con = true;
  while (WiFi.status() != WL_CONNECTED && attempts < 80)
  {
    con = false;
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (!con)
  {
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("WiFi connection failed");
  }
}

// Callback function to draw pixels to the display
void pngDraw(PNGDRAW *pDraw) 
{
  uint16_t lineBuffer[MAX_IMAGE_WIDTH];
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  tft.pushImage(xpos, ypos + pDraw->y, pDraw->iWidth, 1, lineBuffer);
}

void updateLcdContent(int x, int y, String content) 
{
  // Set cursor to the specified position
  tft.setCursor(x, y);
  
  tft.setTextColor(ILI9341_BLACK); // Set text color
  tft.setTextSize(2); // Set text size
  tft.print(content);
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
  uint16_t alcVal;
  alcVal = analogRead(alc_pin);
  Serial.print("Alc Val: "); // alcohol 
  Serial.println(alcVal);
  aind = alcVal;
  alcCount++;
}

void updateAlcLed(uint8_t alcIndex, float temp, char* msgAlert)
{
  if(temp >= 38.00)
  {
    if(alcIndex > 2700){
    strcpy(msgAlert, "Bad");
    }else if(alcIndex > 2200 ){
      strcpy(msgAlert, "Avg");
    }else if(alcIndex >= 0){
      strcpy(msgAlert, "Good");
    }
    else{
      strcpy(msgAlert, "Failed");
    }
  }else
  {
    if(alcIndex > 1500){
    strcpy(msgAlert, "Bad");
    }else if(alcIndex > 1200 ){
      strcpy(msgAlert, "Avg");
    }else if(alcIndex >= 0){
      strcpy(msgAlert, "Good");
    }
    else{
      strcpy(msgAlert, "Failed");
    }
  }
}


//Function for Read the ENS160
void readEns160Sensor()
{
  uint8_t Status = ENS160.getENS160Status();
  Serial.print("Sensor operating status : ");
  Serial.println(Status);

  /**
   * Get the air quality index
   * Return value: 1-Excellent, 2-Good, 3-Moderate, 4-Poor, 5-Unhealthy
   */
  AQI = ENS160.getAQI();
  Serial.print("Air quality index : ");
  Serial.println(AQI);
  // *AQI = random(1,2);

  TVOC = ENS160.getTVOC();
  Serial.print("Concentration of total volatile organic compounds : ");
  Serial.print(TVOC);
  Serial.println(" ppb");
  // *TVOC = random(100,150);

  /**
   * Get CO2 equivalent concentration calculated according to the detected data of VOCs and hydrogen (eCO2 – Equivalent CO2)
   * Return value range: 400–65000, unit: ppm
   * Five levels: Excellent(400 - 600), Good(600 - 800), Moderate(800 - 1000), 
   *               Poor(1000 - 1500), Unhealthy(> 1500
   */
  ECO2 = ENS160.getECO2();
  Serial.print("Carbon dioxide equivalent concentration : ");
  Serial.print(ECO2);
  Serial.println(" ppm");
  // *ECO2 = random(500,550);

  Serial.println();
  // delay(1000);
}

// Function to read temp sensor
void readTempSensor()
{
  sensors_event_t humidity, tempm;
  aht.getEvent(&humidity, &tempm); // populate temp and humidity objects with fresh data

  temp = (tempm.temperature) - 3;
  hum = humidity.relative_humidity;
  Serial.println(temp);
  Serial.println(hum);

}

// Air Index Indication
void updateAqiLed(uint8_t airIndex, char* alertMsg)
{
  if (airIndex == 5)
  {
    strcpy(alertMsg, "Bad");
    setColor(aqiLed, 255, 0, 0); //Red
  }
  else if (airIndex > 2 && airIndex <=4 )
  {
    strcpy(alertMsg, "Avg");
    setColor(aqiLed, 255, 255, 0); //Yellow
  }
  else if (airIndex >= 0 && airIndex <=2 )
  {
    strcpy(alertMsg, "Good");
    setColor(aqiLed, 0, 255, 0); //Green
  }
  else
  {
    
    setColor(aqiLed, 0, 0, 255); // Blue
    strcpy(alertMsg, "Failed");
  }
}

// Function to read MPU 6050
void readMPU6050() 
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long currentTime = millis();
    double dt = (currentTime - lastUpdateTime) / 1000.0; // Delta time in seconds
    lastUpdateTime = currentTime;

    // Compute angles from accelerometer
    double roll = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
    double pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;
    double yaw = atan2(a.acceleration.y, a.acceleration.x) * RAD_TO_DEG;

    // Get filtered angles using Kalman filters (prediction)
    kalAngleX = kalmanX.getAngle(roll, g.gyro.x, dt);
    kalAngleY = kalmanY.getAngle(pitch, g.gyro.y, dt);
    kalAngleZ = kalmanZ.getAngle(yaw, g.gyro.z, dt);

    // Calculate jerk (rate of change of acceleration)(correction)
    jerkX = (kalAngleX - lastKalAngleX) / dt;
    jerkY = (kalAngleY - lastKalAngleY) / dt;
    jerkZ = (kalAngleZ - lastKalAngleZ) / dt;

    // Update last angle values 
    lastKalAngleX = kalAngleX;
    lastKalAngleY = kalAngleY;
    lastKalAngleZ = kalAngleZ;

    jerkMagnitude = sqrt(jerkX * jerkX + jerkY * jerkY + jerkZ * jerkZ);

    // Threshold for detecting a jerk
    double jerkThreshold = 5.0; // Adjust this threshold as needed

    // // Check for significant jerk
    // if (abs(jerkX) > jerkThreshold || abs(jerkY) > jerkThreshold || abs(jerkZ) > jerkThreshold) {
    //     Serial.println("Jerk detected!");
    //     Serial.print("Jerk X: ");
    //     Serial.println(jerkX);
    //     Serial.print("Jerk Y: ");
    //     Serial.println(jerkY);
    //     Serial.print("Jerk Z: ");
    //     Serial.println(jerkZ);
    //     Serial.println();
    // }

    // Print filtered angles and jerk
    // Serial.print("Kalman Angle X: ");
    // Serial.println(kalAngleX);
    // Serial.print("Kalman Angle Y: ");
    // Serial.println(kalAngleY);
    // Serial.print("Kalman Angle Z: ");
    // Serial.println(kalAngleZ);

    Serial.print("Jerk X: ");
    Serial.println(jerkX);
    Serial.print("Jerk Y: ");
    Serial.println(jerkY);
    Serial.print("Jerk Z: ");
    Serial.println(jerkZ);

    Serial.print("Jerk Magnitude : ");
    Serial.println(jerkMagnitude);

    Serial.println();

}

//gps data publish 
void publishDataGps(char* date, char* time, double latitude, double longitude, double speed, double altitude, uint8_t sos)
{
  // Build the JSON object
  StaticJsonDocument<400> jsonDocumentGps;
  jsonDocumentGps["OwnerId"] = OwnerId;
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
  if (client.publish(tempTopic.c_str(), jsonStringGps.c_str()))
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

// Oxygen Data Publish to the mqtt
void publishDataOxy( float sensorVal, float sensorVal2, float filteredVal, float filteredVal2, unsigned long tm)
{
  // Get current timestamp in milliseconds
  // Build the JSON object
  StaticJsonDocument<400> jsonDocumentOxy;
  jsonDocumentOxy["OwnerId"] = OwnerId;
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
  if (client.publish(tempTopic.c_str(), jsonStringOxy.c_str()))
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
void publishDataAlc(uint8_t index, String msgc, float jerka)
{
  // Get current timestamp in milliseconds
  // Build the JSON object
  StaticJsonDocument<200> jsonDocumentAlc;
  jsonDocumentAlc["OwnerId"] = OwnerId;
  jsonDocumentAlc["ID"] = mac;
  // jsonDocumentAlc["MAC"] = mac;
  jsonDocumentAlc["Alert"] = msgc;
  jsonDocumentAlc["Index"] = index;
  jsonDocumentAlc["jerkAcc"] = jerka;
  // Serialize the JSON object to a string
  String jsonStringAlc;
  serializeJson(jsonDocumentAlc, jsonStringAlc);

  String tempTopic = mqtt_topic_alc;
  if (client.publish(tempTopic.c_str(), jsonStringAlc.c_str()))
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
void publishDataAqi(uint8_t voc_index, uint16_t volComp, uint16_t co2, float temp, float hum, String amsg)
{

  StaticJsonDocument<400> jsonDocumentAqi;
  jsonDocumentAqi["OwnerId"] = OwnerId;
  jsonDocumentAqi["ID"] = mac;
  // jsonDocumentAqi["MAC"] = mac;
  jsonDocumentAqi["Alert"] = amsg;
  jsonDocumentAqi["Index"] = voc_index;
  jsonDocumentAqi["Voc"] = volComp;
  jsonDocumentAqi["Eco2"] = co2;
  jsonDocumentAqi["Temperature"] = temp;
  jsonDocumentAqi["Humidity"] = hum;

  String jsonStringAqi;
  serializeJson(jsonDocumentAqi, jsonStringAqi);

  String tempTopic = mqtt_topic_aqi;
  if (client.publish(tempTopic.c_str(), jsonStringAqi.c_str()))
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


void publishDataMpu(double jerkX, double jerkY, double jerkZ, float jerkAcc)
{
  // Get current timestamp in milliseconds
  // Build the JSON object
  StaticJsonDocument<400> jsonDocumentAcc;
  jsonDocumentAcc["OwnerId"] = OwnerId;
  jsonDocumentAcc["ID"] = mac;
  // jsonDocumentAcc["MAC"] = mac;
  jsonDocumentAcc["X-axis"] = jerkX;
  jsonDocumentAcc["Y-axis"] = jerkY;
  jsonDocumentAcc["Z-axis"] = jerkZ;
  jsonDocumentAcc["jerkAcc"] = jerkAcc;  

  // Serialize the JSON object to a string
  String jsonStringAcc;
  serializeJson(jsonDocumentAcc, jsonStringAcc);

  String tempTopic = mqtt_topic_acc;
  if (client.publish(tempTopic.c_str(), jsonStringAcc.c_str()))
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

void reconnect()
{
  String clientId = "ESP8266Client-" + String(random(0xffff), HEX);
  Serial.print("Attempting MQTT connection...");
  if (client.connect(clientId.c_str(), mqttUsername, mqttPassword))
  {
    Serial.println(F("connected"));
    client.subscribe(sub_topic);
  }
  else
  {
    Serial.print(F("failed, rc="));
    Serial.print(client.state());
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

void reconnectWifi()
{
  Serial.println(F("Attempting to reconnect Wi-Fi..."));
  delay(10);
  Serial.println();
  Serial.print(F("Connecting to "));
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  int attempts = 0;
  // bool con = true;
  while (WiFi.status() != WL_CONNECTED && attempts < 5)
  {
    // con = false;
    delay(300);
    Serial.print(F("."));
    attempts++;
  }
}

void checkForOtaUpdate() 
{
  if (WiFi.status() == WL_CONNECTED) {
    String updateUrl = "https://smartambulance.in/otaFile/download?version=" + String(CURRENT_FIRMWARE_VERSION)+"&deviceId=" +mac;
    t_httpUpdate_return ret = ESPhttpUpdate.update(updateUrl);

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;
      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;
      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK");
        break;
    }
  }
}


//Offline store data
String offlineStoreData(String mac, char* date, char* time, double latitude, double longitude, float filteredVal, float filteredVal2, uint8_t index, uint8_t AQI, float temp)
{
  // Build the JSON object
  StaticJsonDocument<400> jsonDocumentStore;
  jsonDocumentStore["Device_ID"] = mac;
  jsonDocumentStore["Date"] = date;
  jsonDocumentStore["Time"] = time;
  jsonDocumentStore["latitude"] = latitude;
  jsonDocumentStore["longitude"] = longitude;
  jsonDocumentStore["Oxygen1_Per"] = filteredVal;
  jsonDocumentStore["Oxygen2_Per"] = filteredVal2;
  jsonDocumentStore["Alcohol_Index"] = index;
  jsonDocumentStore["AQI_Index"] = AQI;
  jsonDocumentStore["Temperature"] = temp;
 
  String jsonStringStore; // Serialize the JSON object to a string
  serializeJson(jsonDocumentStore, jsonStringStore);
  return jsonStringStore;
}

void storeData(const char *path, const String &data)
{
    File file = LittleFS.open(path, "a");
    if (!file) {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (file.print(data)) {
        Serial.println("Data stored successfully");
    } else {
        Serial.println("Failed to store data");
    }
    file.close();
}

void readAndPrintStoredData(const char* path)
{
    File file = LittleFS.open(path, "r");
    if (!file) {
        Serial.println("Failed to open file for reading");
        return;
    }

    while (file.available()) {
        String line = file.readStringUntil('\n');
        Serial.println(line);
    }
    file.close();
}

void deleteFile(fs::FS &fs, const char *path) 
{
  Serial.printf("Deleting file: %s\r\n", path);
  if (fs.remove(path)) {
    Serial.println("- file deleted");
  } else {
    Serial.println("- delete failed");
  }
}

bool checkForFile(fs::FS &fs, const char *path) 
{
  // Check if the specific file exists
  File file = fs.open(path);
  if (file) {
    // File exists
    Serial.println("File available");
    file.close();
    return true;
  } else {
    // File does not exist
    Serial.println("File not available");
    return false;
  }
}

bool republishStoredData(const char *path, const char *reptopic) 
{
  if (!client.connected()) {
    reconnect();
  }

  File file = LittleFS.open(path, "r");
  if (!file) {
    Serial.println("Failed to open file for reading");
    return false;
  }
  bool success = true;

  while (file.available()) 
  {
    String data = file.readStringUntil('\n');
    StaticJsonDocument<400> doc;
    DeserializationError error = deserializeJson(doc, data);

    if (error) {
      Serial.print(F("Failed to read file, using default configuration: "));
      Serial.println(error.f_str());
      file.close();
      success = false;
      return false;
    } else {
      Serial.println("Json data read successfully");
    }

    // Ensure the MQTT client is connected
    if (!client.connected()) {
      Serial.println("MQTT client not connected");
      file.close();
      success = false;
      return false;
    }

    // Extract data from JSON document
    String payload;
    serializeJson(doc, payload);

    // Debugging topic
    Serial.print("Topic: ");
    Serial.println(reptopic);

    bool result = client.publish(reptopic, payload.c_str());

    if (!result) 
    {
      Serial.println("Data publishing failed");
      Serial.print("MQTT client state: ");
      Serial.println(client.state());
      file.close();
      success = false;
      return false;
    } else 
    {
      Serial.println("Data published successfully: " + payload);
    }
    delay(50);
  }

  file.close();
  if (success)
  {
    deleteFile(LittleFS, offlineData);
    Serial.println("Delete success");
  }
  else
  {
    Serial.println("Delete failed");
  }
  return true;
}

void fileSystem()
{
    // Get filesystem info
  size_t totalBytes = LittleFS.totalBytes();
  size_t usedBytes = LittleFS.usedBytes();
  float usedPercentage = (float)usedBytes / totalBytes * 100;

  Serial.println("Filesystem Info:");
  Serial.print("Total space: ");
  Serial.print(totalBytes);
  Serial.println(" bytes");

  Serial.print("Used space: ");
  Serial.print(usedBytes);
  Serial.println(" bytes");

  Serial.print("Free space: ");
  Serial.print(totalBytes - usedBytes);
  Serial.println(" bytes");

  Serial.print("Used percentage: ");
  Serial.print(usedPercentage);
  Serial.println("%");

  // Check if the used space is more than 90%
  if (usedPercentage > 90.0) {
    Serial.println("Storage is more than 80% full. Deleting all files...");
    deleteFile(LittleFS, offlineData);
    Serial.println("All files deleted.");
  } else {
    Serial.println("Storage usage is within acceptable limits.");
  }

  Serial.print("Total space: ");
  Serial.print(totalBytes / 1024.0);
  Serial.println(" KB");

  Serial.print("Used space: ");
  Serial.print(usedBytes / 1024.0);
  Serial.println(" KB");

  Serial.print("Free space: ");
  Serial.print((totalBytes - usedBytes) / 1024.0);
  Serial.println(" KB");
}