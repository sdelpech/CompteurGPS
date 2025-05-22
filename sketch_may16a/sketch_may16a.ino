#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>     
#include <Adafruit_ST7735.h> 
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include "images.h"

#include <TinyGPSPlus.h>

// Définir les broches RX et TX de l'UART que vous utiliserez
// Ces broches sont des EXEMPLES et doivent être remplacées par les broches que vous trouverez sur le schéma de votre carte.
// Par exemple, si vous trouvez que GPIO8 et GPIO9 sont libres et conviennent pour l'UART1 :
#define GPS_RX_PIN 1 // Broche RX sur l'ESP32-C6 connectée au TXD du GPS
#define GPS_TX_PIN 3 // Broche TX sur l'ESP32-C6 connectée au RXD du GPS

// Utilise l'UART1 (ou 2 si vous choisissez cet UART et des broches différentes)
HardwareSerial SerialGPS(1); // Mettez 1 pour UART1, 2 pour UART2.

TinyGPSPlus gps;

#define TFT_CS 14
#define TFT_RST 21
#define TFT_DC 15
#define TFT_MOSI 6  
#define TFT_SCLK 7 
#define TFT_LIGHT 22

#define RGB_LED_PIN 80

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
Adafruit_NeoPixel pixels(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

float p = 3.1415926;

void setup(void) {
  Serial.begin(9600);

  pinMode(TFT_LIGHT, OUTPUT);
  digitalWrite(TFT_LIGHT, LOW);

  pixels.begin();

  tft.init(172, 320); 
  tft.setRotation(3);      

  tft.setSPISpeed(40000000);

  Serial.println(F("Initialized"));

  // large block of text
  tft.fillScreen(ST77XX_BLACK);

  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("Serial GPS démarré sur les broches RX:" + String(GPS_RX_PIN) + " TX:" + String(GPS_TX_PIN));
  Serial.println("Démarrage du module GPS...");
  
  tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK);
  setBacklightBrightness(5);

  drawImageFromBytes( 0,  0, 320, 172, gImage_compteur);
  delay(3000);
  masqueLogo();
  tft.setTextSize(14);
  tft.setCursor(0, 32);
  tft.println("188");
  tft.setCursor(240,100);
  tft.setTextSize(3);
  tft.println("km/h");
  masqueVitesse();
  delay(1000);
  masqueUnite();
  drawImageFromBytes( 0,  0, 320, 172, gImage_compteur);
  delay(10000);
}

void loop() {
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      if (gps.location.isValid()) {  
        Serial.println(gps.speed.kmph());
        //Ajustement de la luminosité en fonction de l'heure 
        if((gps.time.hour() > 18) && (gps.time.hour()<10)){
          setBacklightBrightness(10);
        }
        //Conversion de la vitesse en chaine
        String vitesse_str = String(gps.speed.kmph(),0);
        //Position du curseur en fonction du nombre
        tft.setTextSize(14);
        if(gps.speed.kmph()>10){
          tft.setCursor(74, 32);
          if(gps.speed.kmph()>100){
            tft.setCursor(0, 32);
          }
          tft.println(vitesse_str);
        }else{
          masqueVitesse();
          masqueUnite();
          drawImageFromBytes( 0,  0, 320, 172, gImage_compteur);
          delay(10000);
          masqueLogo();
          tft.setTextWrap(true);
          tft.setCursor(240,100);
          tft.setTextSize(3);
          tft.println("km/h");
        }
      } else {
        tft.fillScreen(ST77XX_BLACK);
        drawImageFromBytes( 0,  0, 320, 172, gImage_compteur);
        Serial.println("Pas de données GPS valides.");
        delay(10000);
        masqueLogo();
      }
    }
  }

  // Si le module GPS n'envoie plus de données pendant un certain temps
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    tft.fillScreen(ST77XX_BLACK);
    drawImageFromBytes( 0,  0, 320, 172, gImage_compteur);
    delay(10000);
    masqueLogo();
    Serial.println("Pas de données GPS reçues, vérifiez les connexions et l'antenne.");
    // while(true); // Décommentez pour stopper le programme si aucune donnée n'est reçue
  }

}

void drawImageFromBytes(int16_t x, int16_t y, int16_t w, int16_t h, const unsigned char *imageData) {
  if ((x >= tft.width()) || (y >= tft.height())) return;
  if ((x + w - 1 < 0) || (y + h - 1 < 0)) return; // Totalement hors écran

  int16_t x2 = x + w - 1;
  int16_t y2 = y + h - 1;

  if (x2 >= tft.width())  x2 = tft.width()  - 1;
  if (y2 >= tft.height()) y2 = tft.height() - 1;

  tft.setAddrWindow(x, y, w, h); // Prépare la zone d'affichage

  for (int16_t j = 0; j < h; j++) {
    for (int16_t i = 0; i < w; i++) {
      // Calcul de l'index dans le tableau 1D
      // Chaque pixel prend 2 octets
      unsigned long pixelIndex = (unsigned long)(j * w + i) * 2;

      // Lire les deux octets pour la couleur depuis PROGMEM
      uint8_t highByte = pgm_read_byte(&imageData[pixelIndex]);
      uint8_t lowByte  = pgm_read_byte(&imageData[pixelIndex + 1]);

      // Combiner les deux octets en une couleur RGB565 (16 bits)
      // Ordre: Poids Fort (MSB) en premier, Poids Faible (LSB) ensuite
      uint16_t color = (highByte << 8) | lowByte;

      // Si l'ordre était Poids Faible en premier, ce serait :
      // uint16_t color = (lowByte << 8) | highByte;

      tft.pushColor(color); // Envoyer la couleur à l'écran
    }
  }
}

void masqueLogo(){
  tft.fillRect(28,27,265,120,ST77XX_BLACK);
}

void masqueUnite(){
  tft.fillRect(240,100,85,25,ST77XX_BLACK);
}

void masqueVitesse(){
  tft.fillRect(14,32,224,98,ST77XX_BLACK);
}

void setBacklightBrightness(int brightness) {
  // 'brightness' doit être une valeur entre 0 et (2^resolution - 1)
  // Par exemple, 0-255 pour une résolution de 8 bits.
  analogWrite(TFT_LIGHT, brightness);
}

void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}
