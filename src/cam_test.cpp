#include <Sipeed_OV2640.h>
#include <Sipeed_ST7789.h>
#include <WiFiEsp.h>
#include <WiFiEspUdp.h>
#include "wifi_credentials.h"

SPIClass spi_(SPI0); // MUST be SPI0 for Maix series on board LCD
Sipeed_ST7789 lcd(320, 240, spi_);
Sipeed_OV2640 cam(FRAMESIZE_VGA, PIXFORMAT_RGB565);

// Tasrget IP / port
// Will be overwritten when UDP received
IPAddress tIp = IPAddress(192, 168, 42, 147);
unsigned tPort = 1331;
const unsigned int localPort = 10002;  // local port to listen on
WiFiEspUDP Udp;

unsigned getJpegSize(uint8_t *buf, unsigned maxSize)
{
  unsigned n = 0, nMatches = 0;
  const uint8_t eofData[] = {0xFF, 0xD9, 0x00, 0x00};
  while(n < maxSize) {
    n++;
    if (*buf++ == eofData[nMatches]) {
      nMatches++;
      if (nMatches >= sizeof(eofData)) {
        return n - 2;
      }
    } else {
      nMatches = 0;
    }
  }
  return n;
}

#define ESP_BAUD 115200 * 30
#define SER_BAUD 1000000

void initFastWifi(void) {
    Serial1.begin(ESP_BAUD);
    Serial1.printf("\r\nAT+RST\r\n");
    Serial1.flush();
    delay(500);
    Serial1.end();
    Serial1.begin(115200);
    WiFi.init(&Serial1);
    Serial.printf("Setting ESP to %d baud/s\n", ESP_BAUD);
    Serial1.printf("\r\nAT+UART_CUR=%d,8,1,0,0\r\n", ESP_BAUD);
    Serial1.flush();
    delay(100);
    Serial1.end();
    Serial1.begin(ESP_BAUD);
}

void setup()
{
    lcd.begin(15000000, COLOR_BLACK);
    lcd.setTextWrap(true);
    lcd.setTextSize(4);
    lcd.setTextColor(COLOR_GREEN);
    lcd.println("cam_test.cpp");
    lcd.setTextSize(2);

    Serial.begin(SER_BAUD);
    initFastWifi();
    int status = WL_IDLE_STATUS;     // the Wifi radio's status

    // attempt to connect to WiFi network
    lcd.setTextColor(COLOR_YELLOW);
    while (status != WL_CONNECTED) {
      lcd.println(WIFI_SSID);
      status = WiFi.begin(WIFI_SSID, WIFI_PW);
    }
    lcd.setTextColor(COLOR_WHITE);

    Udp.begin(localPort);
    Serial.printf("Listening on port %d\n", localPort);
    IPAddress myIP = WiFi.localIP();
    lcd.print("<- ");
    lcd.print(myIP);
    lcd.printf(":%d\n", localPort);
    lcd.print("-> ");
    lcd.print(tIp);
    lcd.printf(":%d\n", tPort);

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

    uint8_t qual = 24;
    cam.ov2640_set_quality(qual);

    while (1) {

    // test ESP with serial port feed-through
      if (Serial.available()) {
        char c = (char)Serial.read();
        switch (c) {
    //       case '<':
    //         Serial1.end();
    //         Serial1.begin(115200);
    //         break;
    //       case '>':
    //         Serial1.end();
    //         Serial1.begin(ESP_BAUD);
    //         break;
    //       case '?':
    //         WiFi.init(&Serial1);
    //         break;
          case '=':
            qual++;
            Serial.printf("\nqual: %d\n", qual);
            cam.ov2640_set_quality(qual);
            break;
          case '-':
            qual--;
            Serial.printf("\nqual: %d\n", qual);
            cam.ov2640_set_quality(qual);
            break;
          default:
            Serial.printf("what? ");
    //         Serial1.write(c);
        }
      }
    //   if (Serial1.available()) {
    //     Serial.write((char)(Serial1.read() & 0xFF));
    //   }

      // set new target IP address on UDP received packet
      if (Udp.parsePacket()) {
        IPAddress rIp = Udp.remoteIP();
        Serial.print("RX: ");
        Serial.println(rIp);
        if (rIp != tIp) {
          tIp = rIp;
          lcd.print("-> ");
          lcd.print(tIp);
          lcd.printf(":%d\n", tPort);
        }
        Udp.flush();
      }

      uint8_t *img = cam.snapshot();
      unsigned jSize = getJpegSize(img, 320 * 240 * 2);
      if (jSize >= 320 * 240 * 2) // Could not find end of jpeg :(
        continue;

      // const unsigned bufSize = 320 * 240 * 2;
      // // Erase memory after jpeg
      // // for (unsigned i = jSize; i < bufSize; i++) {
      // //   img[i] = 0;
      // // }
      // float g;
      // int exp = 123;
      // cam.ov2640_get_gain_db(&g);
      // cam.ov2640_get_exposure_us(&exp);
      // // lcd.drawImage(0, 0, 320, 240, (uint16_t*)img);
      // Serial.printf("gain: %6.3f dB,  exposure: %d\n", g, exp);
      // Serial.write("imagedata\n");
      // // Serial.write(img, 320 * 240 * 2);
      // Serial.write(img, jSize);
      // Serial.write("finito!\n");

      uint8_t *p = img, pCnt = 0, pBuff[1201];
      Serial.printf("%d: ", jSize);
      while (jSize > 0) {
        unsigned pLen = min(jSize, 1200);
        pBuff[0] = pCnt;
        memcpy(&pBuff[1], p, pLen);
        Udp.beginPacket(tIp, tPort);
        Udp.write(pBuff, pLen + 1);
        Udp.endPacket();
        Serial.write('*');
        p += pLen;
        jSize -= pLen;
        pCnt++;
      }
      Serial.write('\n');
      // nc -lu 192.168.42.183 -p 1331
    }
}

void loop () {}
