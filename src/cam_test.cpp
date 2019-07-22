#include <Sipeed_OV2640.h>
#include <Sipeed_ST7789.h>
#include <WiFiEsp.h>
#include <WiFiEspUdp.h>

SPIClass spi_(SPI0); // MUST be SPI0 for Maix series on board LCD
Sipeed_ST7789 lcd(320, 240, spi_);
Sipeed_OV2640 cam(FRAMESIZE_QVGA, PIXFORMAT_RGB565);

// Tasrget IP / port
IPAddress tIp = IPAddress(192, 168, 42, 201);
unsigned tPort = 1331;
const unsigned int localPort = 10002;  // local port to listen on
WiFiEspUDP Udp;

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void setup()
{
    Serial.begin(1000000);
    Serial1.begin(115200);

    char ssid[] = "Narrenzunft-FV";  // your network SSID (name)
    char pass[] = XXX;   // your network password
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
    printWifiStatus();

    Serial.println("\nStarting connection to server...");
    // if you get a connection, report back via serial:
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
    cam.ov2640_set_auto_gain(1, 0, 6.0);
    cam.setPixFormat(PIXFORMAT_JPEG);
    cam.run(true);
    lcd.begin(15000000, COLOR_GREEN);
}

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

void loop()
{
  // test wifi wire feed-through
  // char c;
  // if (Serial.available()) {
  //   Serial1.write((char)(Serial.read() & 0xFF));
  // }
  // if (Serial1.available()) {
  //   Serial.write((char)(Serial1.read() & 0xFF));
  // }


  // float g;
  // int exp = 123;
  uint8_t *img = cam.snapshot();
  // cam.ov2640_get_gain_db(&g);
  // cam.ov2640_get_exposure_us(&exp);
  lcd.drawImage(0, 0, cam.width(), cam.height(), (uint16_t *)img);
  // Serial.printf("gain: %6.3f dB,  exposure: %d\n", g, exp);
  // Serial.write("imagedata\n");
  // // Serial.write(img, 320 * 240 * 2);
  // Serial.write(img, getJpegSize(img, 16000));
  // Serial.write("finito!\n");
  Udp.beginPacket(tIp, tPort);
  Udp.write(img, getJpegSize(img, 1500));
  Udp.endPacket();

  // nc -lu 192.168.42.183 -p 1331
}

