
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "YOUR WIFI NAME";
const char* password = "self-explanatory";

#define API_URL "http://pi2.local:8400/point/"

#include "DHT.h"

#define DHTPIN 19     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11

DHT dht(DHTPIN, DHTTYPE);


//MH-Z19
#include <Arduino.h>
#include "MHZ19.h"                                        
#include <SoftwareSerial.h>                                // Remove if using HardwareSerial

#define TX_PIN 26                                          // pin which the MHZ19 Tx pin is attached to
#define RX_PIN 25                                          // pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)

MHZ19 myMHZ19;                                             // Constructor for library
SoftwareSerial mySerial(TX_PIN, RX_PIN);                   // (Uno example) create device to MH-Z19 serial

void SendDHReadings() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  // float f = dht.readTemperature(true);

  if (isnan(h) || isnan(t)) {
    return;
  }

  Serial.print("sending DH readings: ");
  Serial.print(h);
  Serial.println(t);
  sendMeasurement("office", "humidity", h);
  sendMeasurement("office", "temperature", t);
}

void SendMHZReadings() {
    int ppm = myMHZ19.getCO2();
    Serial.print("sending MHZ reading: ");
    Serial.println(ppm);
    sendMeasurement("office", "co2", ppm);
}


WebServer server(80);

bool ota_flag = false;
bool http_processing = false;
uint16_t time_elapsed = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");

  WiFi.mode(WIFI_STA);  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }

  ArduinoOTA.setHostname("sensors");
  if (MDNS.begin("sensors")) {
    Serial.println("MDNS responder started");
  }

  dht.begin();
  mySerial.begin(BAUDRATE);                               // (Uno example) device to MH-Z19 serial start   
  myMHZ19.begin(mySerial);                                // *Serial(Stream) refence must be passed to library begin(). 

  myMHZ19.autoCalibration();                              // Turn auto calibration ON (OFF autoCalibration(false))

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/restart",[](){
    server.send(200,"text/plain", "Restarting...");
    delay(1000);
    ESP.restart();
  });

  server.on("/read", []() {
    int val = analogRead(A0);
    server.send(200, "application/json", String(val));
  });

  server.on("/flash",[](){
    server.send(200,"text/plain", "Prepared to flash for the next 15s...");
    ota_flag = true;
    time_elapsed = 0;
  });

  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });
  server.begin();
}

unsigned long previousTime = 0;
unsigned long currentTime = 0;

bool asyncWait(int timeout) {
  currentTime = millis();
  if (previousTime == 0) {
    previousTime = currentTime;
  }
  if ((currentTime - previousTime) >= timeout ) {
    previousTime = currentTime;
    return true;
  }
  return false;
}

void sendMeasurement(String location, String field, float value) {
    http_processing = true;
    WiFiClient client;
    HTTPClient http;
    http.begin(client, API_URL);
    http.addHeader("Content-Type", "application/json");  

    // start connection and send HTTP header and body
    String body = "{\"location\":\"" + location + "\", \"field\":\"" + field + "\", \"value\":" + String(value) + "}";
    int httpCode = http.POST(body);

    // httpCode will be negative on error
    if (httpCode > 0) {
      // HTTP header has been send and Server response header has been handled
      Serial.printf("[HTTP] POST... code: %d\n", httpCode);
    }

    http.end();
    http_processing = false;
}


void loop() {
  if(ota_flag)
  {
    Serial.println("Updating???");
    uint16_t time_start = millis();
    while(time_elapsed < 15000)
    {
      ArduinoOTA.handle();
      time_elapsed = millis() - time_start;
      delay(10);
    }
    ota_flag = false;
  }
  else {
    if (asyncWait(5000)) {
      Serial.println("getting readings");
      SendDHReadings();
      SendMHZReadings();
    }
    else {
      server.handleClient();
    }
  }
}
