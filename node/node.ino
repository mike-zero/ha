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

//#include <string.h>
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

uint8_t debug = 1;

uint8_t buf_in[CH_COUNT][BUF_LEN];
uint8_t buf_out[CH_COUNT][BUF_LEN];
uint8_t buf_pos[CH_COUNT];
size_t answer_size = 0;

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
  memset(buf_pos, 0, sizeof(buf_pos));
  Serial.begin(115200);
  Serial.print(F("Node #"));
  Serial.println(NODE_ID);
  uint8_t mac[6] = {0x02,0x00,0x00,0x00,0x00,NODE_ID};
  IPAddress myIP(192, 168, 137, 20 + NODE_ID);
  Ethernet.begin(mac,myIP);
  server.begin();
}

void loop() {
  // Serial
  while (Serial.available() > 0) {
    char thisChar = Serial.read();
    answer_size = process_byte(thisChar, CH_SERIAL);
    if (answer_size > 0) Serial.write(buf_out[CH_SERIAL], answer_size);
  }

  // Ethernet
  if (EthernetClient client = server.available()) {
    buf_pos[CH_ETH] = 0;
    while (client.available() > 0) {
      char thisChar = client.read();
      answer_size = process_byte(thisChar, CH_ETH);
      if (answer_size > 0) client.write(buf_out[CH_ETH], answer_size);
    }
    if (buf_pos[CH_ETH] > 0) {
      answer_size = process_byte(term, CH_ETH);
      if (answer_size > 0) client.write(buf_out[CH_ETH], answer_size);
    }
    client.println("Bye!");
    client.stop();
    buf_pos[CH_ETH] = 0;
  }
}

size_t process_byte(uint8_t inbyte, uint8_t channel_id) {
  size_t res = 0;
  if (debug) {
    Serial.print(F("New byte: "));
    if (inbyte < 16) Serial.print("0");
    Serial.println(inbyte, HEX);
  }
  if (inbyte == term) {
    if (buf_pos[channel_id] > 0) {
      process_command(buf_in[channel_id], buf_pos[channel_id], buf_out[channel_id], &answer_size);
      res = answer_size;
    }
    buf_pos[channel_id] = 0;
  } else {
    if (buf_pos[channel_id] < BUF_LEN) {
      buf_in[channel_id][buf_pos[channel_id]++] = inbyte;
    }
  }
  return res;
}

void process_command(uint8_t* msg, size_t size, uint8_t* answer, size_t* answer_size) {
  *answer_size = 0;
  if (debug) {
    Serial.print(F("New command ("));
    Serial.print(size);
    Serial.print("): |");
    Serial.write(msg,size);
    Serial.print("|");
    for (uint8_t i = 0; i < size; i++) {
      if (i > 0) Serial.print(" ");
      if (msg[i] < 16) Serial.print("0");
      Serial.print(msg[i], HEX);
    }
    Serial.println("|");
  }
  switch (msg[0]) {
    case 'D':
      if (size > 1) {
        debug = (msg[1] == 0 || msg[1] == '0') ? 0 : 1;
      } else {
        debug = !debug;
      }
      *answer_size = 2;
      answer[0] = 'D';
      answer[1] = '0' + (debug ? 1 : 0);
      Serial.print(F("Debug set to "));
      Serial.println(debug ? "ON" : "OFF");
      break;
    case 'I':
      if (debug) Serial.println("Init!");
      break;
    case 'W':
      if (debug) Serial.println("Wait");
      break;
    default:
      Serial.println(F("Unknown command!"));
      break;
  }
  if (debug) {
    if (answer_size > 0) {
      Serial.print(F("Got answer! answer_size="));
      Serial.print(*answer_size);
      Serial.print(F(", answer="));
      for (uint8_t i = 0; i < *answer_size; i++) {
        if (i > 0) Serial.print(" ");
        if (answer[i] < 16) Serial.print("0");
        Serial.print(answer[i], HEX);
      }
      Serial.println();
    }
  }
}

