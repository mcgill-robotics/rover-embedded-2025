#include <Arduino.h>
#include <INA230.cpp>
#include <INA230.hpp>


#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>

#define TFT_CS   10
#define TFT_DC    9
#define TFT_RST   8

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  tft.begin();
  tft.setRotation(1);

  tft.fillScreen(ILI9341_BLACK);

  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("Yay,it works!! :)");

  tft.drawRect(10, 90, 100, 50, ILI9341_RED);
  tft.fillRect(120, 90, 100, 50, ILI9341_BLUE);
  tft.drawCircle(60, 180, 30, ILI9341_GREEN);
  tft.fillCircle(180, 180, 30, ILI9341_CYAN);
  tft.drawTriangle(10, 230, 60, 150, 110, 230, ILI9341_MAGENTA);
  tft.drawLine(0, 0, 319, 239, tft.color565(255, 165, 0));
  tft.drawPixel(160, 120, ILI9341_WHITE);
}

void loop() {
  static int x = 10;
  static int dir = 1;

  tft.fillCircle(x, 120, 10, ILI9341_BLACK);
  x += dir * 3;
  if (x > 310 || x < 10) dir *= -1;
  tft.fillCircle(x, 120, 10, ILI9341_RED);

  delay(16);
}