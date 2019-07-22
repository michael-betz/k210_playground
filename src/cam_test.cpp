#include <Sipeed_OV2640.h>
#include <Sipeed_ST7789.h>

SPIClass spi_(SPI0); // MUST be SPI0 for Maix series on board LCD
Sipeed_ST7789 lcd(320, 240, spi_);
Sipeed_OV2640 cam(FRAMESIZE_QVGA, PIXFORMAT_RGB565);

void setup()
{
    Serial.begin(1000000);
    lcd.begin(15000000, COLOR_RED);
    cam.begin();
    // cam.ov2640_set_colorbar(true);
    // cam.ov2640_set_gainceiling(GAINCEILING_4X);
    // cam.ov2640_set_manual_exposure(12000, 2);
    // cam.ov2640_set_auto_exposure(1, 0);
    cam.ov2640_set_hmirror(0);
    cam.ov2640_set_fdiv(1);  // For better low light performance but lower framerate
    cam.ov2640_set_auto_gain(1, 0, 6.0);
    cam.setPixFormat(PIXFORMAT_JPEG);
    cam.run(true);
}

void loop()
{
  float g;
  int exp = 123;
  uint8_t*img = cam.snapshot();
  cam.ov2640_get_gain_db(&g);
  cam.ov2640_get_exposure_us(&exp);
  // lcd.drawImage(0, 0, cam.width(), cam.height(), (uint16_t*)img);
  Serial.printf("gain: %6.3f dB,  exposure: %d\n", g, exp);
  Serial.write("imagedata\n");
  // Serial.write(img, 320 * 240 * 2);
  int zerosCount = 0;
  uint8_t *p = img;
  while(zerosCount < 32) {
    if (*p == 0)
      zerosCount++;
    else
      zerosCount = 0;
    Serial.write(*p++);
  }
  Serial.write("finito!\n");
}

