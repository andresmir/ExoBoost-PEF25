#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "driver/gpio.h"

struct __attribute__((packed)) ControlPacket {
  //bool ledState;
  int motorID;
  char mode;
  int intensity;
  int minVal;
  int maxVal;
};

struct __attribute__((packed)) QuatPacket {
  uint8_t start;
  int16_t qw,qx,qy,qz;
  int16_t pos[3];            // posiciones motores
  int16_t vel[3];            // velocidades motores
  int16_t cur[3];            // corrientes motores
};


uint8_t esp32_leg_address[] = {0xE0, 0x5A, 0x1B, 0x77, 0x8F, 0x4C};

void parseMessage(String msg, ControlPacket* Control) {

  // Ejemplo mensaje: M2A100-050+250
  Control->motorID   = msg.substring(1, 2).toInt();     // Char 1
  Control->mode      = msg.charAt(2);                   // Char 2
  Control->intensity = msg.substring(3, 6).toInt();     // Char 3–5
  Control->minVal    = msg.substring(6, 10).toInt();    // Char 6–9
  Control->maxVal    = msg.substring(10, 14).toInt();   // Char 10–13

  

}

void onReceive(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  if (len == sizeof(QuatPacket)) {
    QuatPacket packet;
    memcpy(&packet, incomingData, sizeof(packet));
    // Serial.println("Received QuatPacket via ESP-NOW:");

    if (packet.start == 0xAA) {

      // Forward to computer as CSV
      Serial.printf("%d,%d,%d,%d,", packet.qw, packet.qx, packet.qy, packet.qz);
      for (int i = 0; i < 3; i++) {
        Serial.printf("%d,%d,%d", packet.pos[i], packet.vel[i], packet.cur[i]);
        if (i < 2) Serial.print(",");
      }
      Serial.println();
    }
  }
}


void setup(){
    Serial.begin(500000);
    Serial.setTimeout(10);

    while(!Serial){ delay(10); }
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        while (1);
    }

    Serial.println("Receiver ready!");
    Serial.print("Receiver MAC Address: ");
    Serial.println(WiFi.macAddress());

    esp_now_register_recv_cb(onReceive);
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, esp32_leg_address, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

void loop(){
    if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim(); 

    if (msg.length() == 14 && msg.startsWith("M")) {
      ControlPacket control;
      parseMessage(msg, &control);
    } else if (msg.length() > 0) {
      Serial.println("Mensaje invalido.");
    }
  }
}   

