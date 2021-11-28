#include "LoRa_E32.h"
#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // Arduino RX <-- e32 TX, Arduino TX --> e32 RX
LoRa_E32 e32ttl(&mySerial);


typedef  struct {
  double longi;
  double lati;
  double alt;
  double aciX;
  double aciY;
  double aciZ;
} Signal;

Signal data;

void setup() {
  Serial.begin(9600);
  e32ttl.begin();
  delay(500);
}

void loop() {
  if (e32ttl.available()  > 1) {
    ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(Signal));
    data = *(Signal*) rsc.data;
    rsc.close();
    // Serial.print(F("gelen yukseklik bilgisi: "));
    
    Serial.print(data.alt);
    Serial.print("*");
    Serial.print(data.aciX);
    Serial.print("*");
    Serial.print(data.aciY);
    Serial.print("*");
    Serial.print(data.aciZ);
    Serial.print("*");
    Serial.print(data.longi , 3);
    Serial.print("*");
    Serial.println(data.lati , 3);
   
    

  }

}
