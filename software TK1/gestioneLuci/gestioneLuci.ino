#include <Adafruit_NeoPixel.h>

int analogInPin = A0;
int sensorValue = 0;
int sensorValue2 = 0;
boolean change = true;
int k;
int intensita = 0;
boolean stato = true;

#define PIN 6

// Parameter 1 = number of pixels in strip (60 per metro, noi se usiamo 5 metri sono 300)
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(300, PIN, NEO_GRB + NEO_KHZ800); //300 numero led usati
void setup() {
  Serial.begin(9600);
  pinMode( analogInPin, INPUT);
 
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  
}
void loop() {

  sensorValue = analogRead(analogInPin);            
 
  Serial.print("sensor = " );
  Serial.println(sensorValue);  

  delay(100); //delay per rilevare intensità da sensore

  int wait = 10;
  uint32_t c = strip.Color(255, 255, 200);
  uint32_t off = strip.Color(0, 0, 0);

  if(sensorValue < 860 or sensorValue >870){
              stato = true;
     }
  
  while(stato){   
     for(uint16_t i=17; i<277; i++) {
          strip.setPixelColor(i, c);
          strip.setBrightness(intensita); //regolare luminosità da 0 a 255
     }
     strip.show(); 
     delay(wait); 
    
     sensorValue2 = analogRead(analogInPin);
     Serial.print("sensor_2 = " );
     Serial.println(sensorValue2); 
     delay(100);
      

     if(sensorValue2 < 855 and intensita <= 254){ //abbasso di poco il valore per evitare che il sensore aumenti a dismisura poichè faccia continuamente i valori limite
              intensita += 1;
     }
     if(sensorValue2 > 875 and intensita >= 1){
               intensita -= 1;
     }
     if((sensorValue2 >= 860 and sensorValue2 <= 870) or intensita == 0){
              stato = false;
     }
     
     Serial.print("intensita = " );
     Serial.println(intensita); 
   
  }
   
}







