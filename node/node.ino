/*
 * UIPEthernet EchoServer example.
 *
 * UIPEthernet is a TCP/IP stack that can be used with a enc28j60 based
 * Ethernet-shield.
 *
 * UIPEthernet uses the fine uIP stack by Adam Dunkels <adam@sics.se>
 *
 *      -----------------
 *
 * This Hello World example sets up a server at 192.168.1.6 on port 1000.
 * Telnet here to access the service.  The uIP stack will also respond to
 * pings to test if you have successfully established a TCP connection to
 * the Arduino.
 *
 * This example was based upon uIP hello-world by Adam Dunkels <adam@sics.se>
 * Ported to the Arduino IDE by Adam Nielsen <malvineous@shikadi.net>
 * Adaption to Enc28J60 by Norbert Truchsess <norbert.truchsess@t-online.de>
 */

#include <string.h>
#include <UIPEthernet.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <dht.h>

#include "node.h"

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 12
// Will try to detect all DHT sensors inside this pin range
#define DHT_MIN_PIN 3
#define DHT_MAX_PIN 9

uint8_t debug = 0;

dht DHT;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
int numberOfDevices; // Number of temperature devices found
DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

int ledPin = 13;

EthernetServer server = EthernetServer(1000);

uint8_t term = '\n';

void setup() {
  Serial.begin(115200);
  Serial.print("Node #");
  Serial.println(NODE_ID);
  uint8_t mac[6] = {0x02,0x00,0x00,0x00,0x00,NODE_ID};
  IPAddress myIP(192, 168, 137, 20 + NODE_ID);
  Ethernet.begin(mac,myIP);
  server.begin();
}

void loop() {
  size_t size;
  size_t start;
  size_t finish;

  if (EthernetClient client = server.available()) {
    while((size = client.available()) > 0) {
        uint8_t* msg = (uint8_t*)malloc(size);
        size = client.read(msg,size);
        start = 0;
        finish = 0;

        while (start < size) {
          finish = start;
          while(finish < size && msg[finish] != term) finish++;
          process_command(&msg[start], finish-start);
          start = finish+1;
        }
        free(msg);
      }
      client.println("DATA from Server!");
      client.stop();
    }
}

void process_command(uint8_t* msg, size_t size) {
  if (debug) {
    Serial.print("New command (");
    Serial.print(size);
    Serial.print("): |");
    Serial.write(msg,size);
    Serial.println("|");
  }
  switch (msg[0]) {
    case 'D':
      debug = !debug;
      Serial.print("Debug set to ");
      Serial.println(debug ? "ON" : "OFF");
      break;
    case 'I':
      if (debug) Serial.println("Init!");
      break;
    case 'W':
      if (debug) Serial.println("Wait");
      break;
    case 'L':
      if (debug) Serial.println("LED");
      break;
    default:
      Serial.println("Unknown command!");
      break;
  }
}

