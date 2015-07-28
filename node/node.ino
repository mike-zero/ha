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

uint8_t debug = 0;

uint8_t buf_in[CH_COUNT][BUF_LEN];
uint8_t buf_pos[CH_COUNT];

dht DHT;
uint16_t DHT_pins = 0;
uint16_t OW_pins = 0;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
int numberOfDevices; // Number of temperature devices found
DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

byte ow_addr[8];

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
    process_byte(thisChar, CH_SERIAL, &Serial);
  }

  // Ethernet
  if (EthernetClient client = server.available()) {
    buf_pos[CH_ETH] = 0;
    while (client.available() > 0) {
      char thisChar = client.read();
      process_byte(thisChar, CH_ETH, &client);
    }
    if (buf_pos[CH_ETH] > 0) {
      process_byte(term, CH_ETH, &client);
    }
    client.println("Bye!");
    client.stop();
    buf_pos[CH_ETH] = 0;
  }
}

size_t process_byte(uint8_t inbyte, uint8_t channel_id, Stream* strm) {
  size_t res = 0;
  if (debug) {
    Serial.print(F("New byte: "));
    if (inbyte < 16) Serial.print("0");
    Serial.println(inbyte, HEX);
  }
  if (inbyte == term) {
    if (buf_pos[channel_id] > 0) {
      process_command(channel_id, strm);
    }
    buf_pos[channel_id] = 0;
  } else {
    if (buf_pos[channel_id] < BUF_LEN) {
      buf_in[channel_id][buf_pos[channel_id]++] = inbyte;
    }
  }
  return res;
}

void process_command(uint8_t channel_id, Stream* strm) {
  uint8_t* msg = buf_in[channel_id];
  size_t size = buf_pos[channel_id];
  if (debug) {
    Serial.print(F("Got command ("));
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
    case 'D': // Debug
      if (size > 1) {
        debug = (msg[1] == 0 || msg[1] == '0') ? 0 : 1;
      } else {
        debug = !debug;
      }
      strm->write('D');
      strm->write('0' + (debug ? 1 : 0));
      Serial.print(F("Debug set to "));
      Serial.println(debug ? "ON" : "OFF");
      break;
    case 'R': // Read
      if (size == 4) {
        uint8_t mask_d = msg[2] & 0xFF;
        uint8_t mask_b = msg[3] & 0xFF;
        uint8_t mask_a = msg[1] & 0xFF;

        if (mask_d) { 
          strm->write(DDRD & mask_d);
          strm->write(PORTD & mask_d);
          strm->write(PIND & mask_d);
        }
        if (mask_b) { 
          strm->write(DDRB & mask_b);
          strm->write(PORTB & mask_b);
          strm->write(PINB & mask_b);
        }

        uint8_t i = 0;
        int val;
        strm->write(mask_a);
        while (mask_a > 0) {
          if ((mask_a & 1) == 1) {
            val = analogRead(i);
            strm->write(highByte(val));
            strm->write(lowByte(val));
            if (debug) {
              Serial.print("analogRead(");
              Serial.print(i);
              Serial.print(")=");
              Serial.println(val);
            }
          }
          i++;
          mask_a >>= 1;
        }
      } else {
        strm->write("R-err");
      }
      break;
    case 'I': // Init
      if (debug) Serial.println("Init!");
      strm->write("I");
      break;
    case 'W': // Wait
      if (debug) Serial.println("Wait");
      strm->write("W");
      break;
    case 'S': // Scan
      if (debug) Serial.println("Scan...");
      strm->write("S");
      DHT_pins = 0;
      OW_pins = 0;
      for (uint8_t pin = DHT_MIN_PIN; pin <= DHT_MAX_PIN; i++) {
        if (debug) Serial.println(pin);
        int chk = DHT.read22(pin);
        if (chk == DHTLIB_OK) {
          DHT_pins |= 1 << pin;
          if (debug) {
            Serial.print("DHT22 found on pin ");
            Serial.println(pin);
          }
        } else {
          // 1-wire?
          OneWire ds(pin);
          if (ds.search(ow_addr)) {
            OW_pins |= 1 << pin;
            if (debug) {
              Serial.print("ow[");
              Serial.print(pin);
              Serial.print("] R=");
              for (i = 0; i < 8; i++) {
                Serial.print(ow_addr[i], HEX);
                Serial.print(" ");
              }
              Serial.println();
            }
          }
        }
      }
      strm->write(highByte(DHT_pins));
      strm->write(lowByte(DHT_pins));
      strm->write(highByte(OW_pins));
      strm->write(lowByte(OW_pins));
      break;
    default:
      Serial.println(F("Unknown command!"));
      break;
  }
}

// if (((signed long)(millis()-next))>0)
