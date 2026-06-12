#include <Adafruit_NeoPixel.h>

#define PIN 2
#define NUMPIXELS 8   // pon aquí el número real de LEDs

Adafruit_NeoPixel tira(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  tira.begin();
  tira.setBrightness(255); // máximo brillo
  tira.clear();

  for (int i = 0; i < NUMPIXELS; i++) {
    tira.setPixelColor(i, tira.Color(255, 255, 255)); // aquí cambiamos los colores en BGR (255 todos, blanco)
  }

  tira.show();      //Para que se enciendan todos  
}

void loop() {
}
