#include "esp_camera.h"
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define CAMERA_MODEL_AI_THINKER 
#include "camera_pins.h"

// Hardware Pines
#define I2C_SDA 14
#define I2C_SCL 15
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 32 
#define OLED_RESET    -1 
#define PIN 13        
#define NUMPIXELS 8

Adafruit_NeoPixel tira(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// VARIABLES GLOBALES (Las compartiremos con app_httpd.cpp)
int red = 0, green = 0, blue = 0;
int cuadrante_silueta = 1;
bool actualizarHardware = true;

const uint8_t silueta_persona[] PROGMEM = {
  0x00, 0x00, 0x03, 0xc0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x03, 0xc0, 0x00, 0x00, 0x0f, 0xf0,
  0x1f, 0xf8, 0x3f, 0xfc, 0x3f, 0xfc, 0x3f, 0xfc, 0x3f, 0xfc, 0x3f, 0xfc, 0x3f, 0xfc, 0x3f, 0xfc,
  0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1b, 0xd8,
  0x1b, 0xd8, 0x1b, 0xd8, 0x1b, 0xd8, 0x1b, 0xd8, 0x1b, 0xd8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const char* ssid = "IZZI-EE90";
const char* password = "CDAECHitron";

void startCameraServer();
void setupLedFlash(int pin);

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { Serial.println("OLED error"); }
  display.clearDisplay();
  display.display();

  tira.begin();
  tira.setBrightness(100);
  tira.clear();
  tira.show();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 15;
  config.fb_count = 1;
  
  if(config.pixel_format == PIXFORMAT_JPEG && psramFound()){
      config.jpeg_quality = 12;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) { return; }

  #if defined(LED_GPIO_NUM)
    setupLedFlash(LED_GPIO_NUM);
  #endif

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }

  // Arranca el servidor (esto llamará al código interno de app_httpd.cpp)
  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  if (actualizarHardware) {
    for (int i = 0; i < NUMPIXELS; i++) {
      tira.setPixelColor(i, tira.Color(red, green, blue));
    }
    tira.show();

    int ancho_cuadrante = SCREEN_WIDTH / 5; 
    int posX = ((cuadrante_silueta - 1) * ancho_cuadrante) + ((ancho_cuadrante - 16) / 2);

    display.clearDisplay();
    display.drawBitmap(posX, 0, silueta_persona, 16, 32, SSD1306_WHITE); 
    display.display();

    actualizarHardware = false; 
  }
  delay(10); 
}
