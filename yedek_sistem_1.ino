#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
//lora gps
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;
#include "LoRa_E32.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
static const int RXPin = 12, TXPin = 13; //GPS
TinyGPSPlus gps;
SoftwareSerial portgps(RXPin, TXPin);
//
int aci;
long SimdikiZaman = 0;
long OncekiZaman = 0;
int syc;
long aralik = 96000;
//
typedef struct{
double lati;
double longi;
double alt ;
double aciX;
double aciY;
double aciZ;
} Signal ;
Signal data;

//lora sx
SoftwareSerial portlora(10, 11); // Arduino RX <-- e32 TX, Arduino TX --> e32 RX
LoRa_E32 e32ttl(&portlora);


void setup() {
  syc = 0;
  pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);
  Serial.begin(9600);
  Wire.begin();

    if (!bmp.begin()) 
  {
  Serial.println("BMP180 sensor not found");
  while (1) {}
  }
  
  //Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  //GPS
  portgps.begin(9600);
  delay(500);

  // LORA

  e32ttl.begin();
  delay(500);
  
  delay(1000);
  bno.setExtCrystalUse(true);
  //Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

void sensor() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print(bmp.readAltitude(101500));
  Serial.print("*");
  //Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print("*");
  //Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print("*");
  //Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.println("\t\t");
  

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  delay(BNO055_SAMPLERATE_DELAY_MS);


}
void loop() {
  SimdikiZaman = millis();

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print(bmp.readAltitude(101500));
  Serial.print("*");
  //Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print("*");
  //Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print("*");
  //Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.println("\t\t");
  
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  delay(BNO055_SAMPLERATE_DELAY_MS);
  
  aci = euler.y();
  //gps 
//   portgps.listen();
//  while (portgps.available()) //While there are characters to come from the GPS
//  {
//    gps.encode(portgps.read());//This feeds the serial NMEA data into the library one char at a time
//  }
//  if (gps.location.isUpdated()) //This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
//  {
//    Serial.println("Latitude:");
//    Serial.println(gps.location.lat(), 6);
//    Serial.println("Longitude:");
//    Serial.println(gps.location.lng(), 6);
 //lora sx 
  {
      if (bmp.readAltitude(101500))
      { // Serial.print("alt:");
        //Serial.println(bme.readAltitude(SEALEVELPRESSURE_HPA));
        data.alt = bmp.readAltitude(101500);
      }
      if (euler.x()) {
        // Serial.print("aciX:");
        data.aciX = euler.x();
      }
      if (euler.y()) {
        // Serial.print("aciY:");
        data.aciY = euler.y();
      }
      if (euler.z()) {
        // Serial.print("aciZ:");
        data.aciZ = euler.z();
      }
      if (gps.location.lat()) {

        data.lati = gps.location.lat();

      }
      if (gps.location.lng()) {

        data.longi = gps.location.lng();

      }

      else
        Serial.println( "error");

      delay(300); // smartDelay(1000);
     portlora.listen();

      ResponseStatus rs = e32ttl.sendFixedMessage(0, 6, 7, &data, sizeof(Signal));
      Serial.println(rs.getResponseDescription());
      //lora sx
  
  if (syc == 0) { 
  Serial.print(bmp.readAltitude(101500));
  Serial.print("*");
  //Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print("*");
  //Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print("*");
  //Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.println("\t\t");
 
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  delay(BNO055_SAMPLERATE_DELAY_MS);
  //Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);
  syc = 1;
  }

  if (syc == 1)
  {
    if (aci > 45) {
      digitalWrite(2, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);                       // wait for a second
      digitalWrite(2, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);
      syc = 2;
    }
  }

  sensor();

  if (syc == 2)
  {
    if (SimdikiZaman - OncekiZaman >= aralik) {
      OncekiZaman = SimdikiZaman;
      digitalWrite(2, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);                       // wait for a second
      digitalWrite(2, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);

      syc = 3;
    }
  }
  if (syc == 3)
  {
      //gps 
   portgps.listen();
  while (portgps.available()) //While there are characters to come from the GPS
  {
    gps.encode(portgps.read());//This feeds the serial NMEA data into the library one char at a time
  }
  if (gps.location.isUpdated()) //This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
  {
    Serial.println("Latitude:");
    Serial.println(gps.location.lat(), 6);
    Serial.println("Longitude:");
    Serial.println(gps.location.lng(), 6);
  }
}
}
}
