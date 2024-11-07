#include <Arduino.h>

#include <SoftwareSerial.h>
#include <TinyGPS++.h>

uint8_t localTimezoneOffsetHours = 5;
uint8_t localTimezoneOffsetMin = 30;

//macro for gps
#define RXD2 16
#define TXD2 17


TinyGPSPlus gps;
static const uint32_t GPSBaud = 9600;
SoftwareSerial gpsData(RXD2, TXD2);

//Variable declaration for gps
double longitude = 0 ,latitude = 0;
float speed = 0, altitude =0;
uint16_t year = 0; 
uint8_t month = 0, day = 0;
uint8_t hour = 0, minute = 0, second = 0;
// Array to store formatted time and date
char formattedTime[10];
char formattedDate[12];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  /******GPS Begin******/
  gpsData.begin(GPSBaud);
  delay(2000);

}

void loop() {
  // put your main code here, to run repeatedly:
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
  }
}

