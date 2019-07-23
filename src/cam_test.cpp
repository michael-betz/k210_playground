#include <Sipeed_OV2640.h>
#include <Sipeed_ST7789.h>
#include <WiFiEsp.h>
#include <WiFiEspUdp.h>

SPIClass spi_(SPI0); // MUST be SPI0 for Maix series on board LCD
Sipeed_ST7789 lcd(320, 240, spi_);
Sipeed_OV2640 cam(FRAMESIZE_QVGA, PIXFORMAT_RGB565);

// // Tasrget IP / port
IPAddress tIp = IPAddress(192, 168, 42, 130);
unsigned tPort = 1331;
const unsigned int localPort = 10002;  // local port to listen on
WiFiEspUDP Udp;

unsigned getJpegSize(uint8_t *buf, unsigned maxSize)
{
  unsigned n = 0, zerosCount = 0;
  while(zerosCount < 32 && n < maxSize) {
    n++;
    if (*buf++ == 0)
      zerosCount++;
    else
      zerosCount = 0;
  }
  return n;
}

#define ESP_BAUD 100000
#define SER_BAUD 1000000

void setup()
{
    Serial.begin(SER_BAUD);
    Serial1.begin(115200);
    delay(100);
    Serial.printf("Setting ESP baudrate to %d\n", ESP_BAUD);
    Serial1.printf("\r\n\r\n\r\nAT+UART_CUR=%d,8,1,0,0\r\n", ESP_BAUD);
    delay(100);
    Serial1.flush();
    Serial1.begin(ESP_BAUD);

    char ssid[] = "Narrenzunft-FV";  // your network SSID (name)
    char pass[] = ;   // your network password
    int status = WL_IDLE_STATUS;     // the Wifi radio's status
    WiFi.init(&Serial1);

    // attempt to connect to WiFi network
    while (status != WL_CONNECTED) {
      Serial.print("Attempting to connect to WPA SSID: ");
      Serial.println(ssid);
      // Connect to WPA/WPA2 network
      status = WiFi.begin(ssid, pass);
    }
    Serial.println("Connected to wifi");
    Udp.begin(localPort);

    Serial.print("Listening on port ");
    Serial.println(localPort);

    lcd.begin(15000000, COLOR_RED);
    cam.begin();
    // cam.ov2640_set_colorbar(true);
    // cam.ov2640_set_gainceiling(GAINCEILING_4X);
    // cam.ov2640_set_manual_exposure(12000, 2);
    // cam.ov2640_set_auto_exposure(1, 0);
    cam.ov2640_set_hmirror(0);
    cam.ov2640_set_fdiv(1);  // For better low light performance but lower framerate
    // cam.ov2640_set_auto_gain(1, 0, 6.0);
    cam.setPixFormat(PIXFORMAT_JPEG);
    cam.run(true);
    lcd.begin(15000000, COLOR_GREEN);

    while (1) {
      // // test wifi wire feed-through
      // char c;
      // if (Serial.available()) {
      //   Serial1.write((char)(Serial.read() & 0xFF));
      // }
      // if (Serial1.available()) {
      //   Serial.write((char)(Serial1.read() & 0xFF));
      // }

      uint8_t *img = cam.snapshot();

      // float g;
      // int exp = 123;
      // cam.ov2640_get_gain_db(&g);
      // cam.ov2640_get_exposure_us(&exp);
      // lcd.drawImage(0, 0, cam.width(), cam.height(), (uint16_t *)img);
      // Serial.printf("gain: %6.3f dB,  exposure: %d\n", g, exp);
      // Serial.write("imagedata\n");
      // Serial.write(img, 320 * 240 * 2);
      // Serial.write(img, getJpegSize(img, 16000));
      // Serial.write("finito!\n");

      int pLen, imgLen = getJpegSize(img, 32000);
      // imgLen = 320 * 240 * 2;
      uint8_t *p = img, pCnt = 0, pBuff[1201];
      Serial.printf("%d: ", imgLen);
      while (imgLen > 0) {
        pLen = min(imgLen, 1200);
        pBuff[0] = pCnt;
        memcpy(&pBuff[1], p, pLen);
        Udp.beginPacket(tIp, tPort);
        Udp.write(pBuff, pLen + 1);
        Udp.endPacket();
        Serial.write('*');
        p += pLen;
        imgLen -= pLen;
        pCnt++;
      }
      Serial.write('\n');
      // nc -lu 192.168.42.183 -p 1331
    }
}

void loop () {}
