#include "LoRa_E32.h"
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

#include <TinyGPS++.h>
static const int RXPin = 12, TXPin = 13; //GPS
TinyGPSPlus gps;
SoftwareSerial portgps(RXPin, TXPin);

int aci;                                      //integer tipinde aci değişkeni oluşturuldu
int count;                                    //integer tipinde sayac için count değişkeni oluşturuldu,
#include <MPU6050_tockn.h>
int myPressure;
float rakim;
float maxYukseklik;
float minYukseklik;
float birinciYukseklik = 1500;
float ikinciYukseklik = 1000;
MPU6050 mpu6050(Wire);

//lora sx
SoftwareSerial portlora(10, 11); // Arduino RX <-- e32 TX, Arduino TX --> e32 RX
LoRa_E32 e32ttl(&portlora);

//lora sx
typedef  struct {

  double lati;
  double longi;
  double alt;
  double aciX;
  double aciY;
  double aciZ;

} Signal;

Signal data;

void setup() {
  count = 0;//sayaç sıfırlandı
  Serial.begin(9600);
  // MPU
  Wire.begin();
  if (! bme.begin(0x77, &Wire)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  //GPS
  portgps.begin(9600);
  delay(300);

  // LORA

  e32ttl.begin();
  delay(300);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  delay(100);


}


void loop()
{

  mpu6050.update();
  Serial.print("Alt: ");// ekrana rakım yazdırıldı
  Serial.println(bme.readAltitude(SEALEVELPRESSURE_HPA));
  //Serial.print("angleX : ");
  Serial.print("aciX:");
  Serial.println(mpu6050.getAngleX());
  //Serial.print("\tangleY : ");
  Serial.print("aciY:");
  Serial.println(mpu6050.getAngleY());
  //Serial.print("\tangleZ : ");
  Serial.print("aciZ:");
  Serial.println(mpu6050.getAngleZ());
  //Serial.print("Altitude = ");// ekrana rakım yazdırıldı
  //Serial.print("*");
  //Serial.print(bmp.readAltitude());           // rakım değeri okunup ekrana yazdırıldı

  //Serial.println(" metre");                   // ekrana metre yazdırıldı
  //Serial.print("Real altitude = ");
  //Serial.print(bmp.readAltitude(101500));
  //Serial.println(" metre");
  //Serial.println();
  delay(250);
  rakim = bme.readAltitude(SEALEVELPRESSURE_HPA);//bmp den okuduğumuz değeri rakim diye bir değişkene atadık.

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
  

    // lora sx
    {
      if (bme.readAltitude(SEALEVELPRESSURE_HPA))
      { // Serial.print("alt:");
        //Serial.println(bme.readAltitude(SEALEVELPRESSURE_HPA));
        data.alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
      }
      if (mpu6050.getAngleX()) {
        // Serial.print("aciX:");
        // Serial.println(mpu6050.getAngleX());
        data.aciX = mpu6050.getAngleX();
      }
      if (mpu6050.getAngleX()) {
        // Serial.print("aciY:");
        // Serial.println(mpu6050.getAngleY());
        data.aciY = mpu6050.getAngleY();
      }
      if (mpu6050.getAngleX()) {
        // Serial.print("aciZ:");
        // Serial.println(mpu6050.getAngleZ());
        data.aciZ = mpu6050.getAngleZ();
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

      ResponseStatus rs = e32ttl.sendFixedMessage(0, 4, 7, &data, sizeof(Signal));
      Serial.println(rs.getResponseDescription());
      //lora sx

      if (count == 0)                             //sayaç sıfırsa burayı yap
      {
        maxYukseklik = rakim;                     //max yüksekliği rakima eşitle
        minYukseklik = rakim;                     //min yüksekliği rakima eşitle
        count = 1;                                //sayacı 1 yap ve tekrar buraya dönme
      }

      if (count == 1)                             //sayaç birse burayı yap
      {
        if (rakim > maxYukseklik)                 //eğer rakim büyükse maxYukseklikten - ki sürekli yukarı çıktığı için büyük olacaktır
        {
          maxYukseklik = rakim;                   //yükseldiği sürece maxYukseklik de artar. - Düşmeye başladığında da elimizde çıktığı en yüksek yerin değer olacaktır
        }
        if (maxYukseklik > (rakim + 50))          //buraya girdiyse artık düşüyor demektir - max noktadan 50 m aşağı geldiğinde anlamalıyız ki düşüyor.
        {
          count = 2;                              //artık bu döngüye girmesin diye sayacı değiştirdik
        }
      }

      if (count == 2)                             //sayaç ikiysde burayı yap
      {
        if (rakim < (minYukseklik + birinciYukseklik)) // rakım, başlangıç rakımı ve verdiğimiz yükseliğin toplamından küçükse ilk paraşüt açılma zamanı gelmiştir
        {
          digitalWrite(7, HIGH);   // buradaki 7 nolu pini değiştirecez turn the LED on (HIGH is the voltage level)
          delay(1000);             // wait for a second
          digitalWrite(7, LOW);    // 7 nolu pini değiştirecez turn the LED off by making the voltage LOW

          delay(3000);                                // servonun çalışması için biraz programı burda durdurduk,beklettik
          count = 3;                                  // tekrar buraya girmemesi için sayacı ilerlettik
        }
      }

      if (count == 3)                                 //sayaç üçse burayı yap
      {
        if (rakim < (minYukseklik + ikinciYukseklik)) //rakim, başlangıç rakımı ve verdiğimiz ikinci yüksekliğind toplamından küçükse ikinci paraşüt zamanı gelmiştir.
        {
          digitalWrite(7, HIGH);   //7 nolu pini üsttekinden farklı yapmalıyız  turn the LED on (HIGH is the voltage level)
          delay(1000);             // wait for a second
          digitalWrite(7, LOW);    //7 nolu pini üsttekinden farklı yapmalıyız turn the LED off by making the voltage LOW
          delay(1000);
          count = 4;               // burayı tekrarlamaması için sayacı artır gitsin
        }
      }

    }
  }
}
