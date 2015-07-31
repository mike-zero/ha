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

//#define ETHERNET_ENABLE

//#include <string.h>

#ifdef ETHERNET_ENABLE
#include <UIPEthernet.h>
#endif

#include <OneWire.h>
#include <DallasTemperature.h>
#include <dht.h>

#include "node.h"

#define TEMPERATURE_PRECISION 12
#define GPIO_MASK 0b0000000111111000  // TODO: set all usefull bits, then clear used by Eth, LED, Serial, interrupts, etc.

uint8_t debug = 0;
uint8_t textmode = 0;

#ifdef ETHERNET_ENABLE
//#define ETH_RESET_PIN 9 // uncomment to be able to reset it manually
uint8_t mac[6] = {0x02,0x00,0x00,0x00,0x00,NODE_ID};
IPAddress myIP(192, 168, 137, 20 + NODE_ID);
#endif

#ifndef ETHERNET_ENABLE
#define LED 13  // uncomment to show command processing
#endif

uint8_t buf_in[CH_COUNT][BUF_LEN];
uint8_t buf_pos[CH_COUNT];

dht DHT;
uint16_t DHT_pins = 0;
uint16_t OW_pins = 0;

byte ow_addr[8];

#ifdef ETHERNET_ENABLE
EthernetServer server = EthernetServer(1000);
#endif

uint8_t term = '\n';

void setup() {
  memset(buf_pos, 0, sizeof(buf_pos));
  Serial.begin(115200);
  Serial.print(F("Node #"));
  Serial.println(NODE_ID);
#ifdef ETHERNET_ENABLE
  eth_init();
#endif
#ifdef LED
pinMode(LED, OUTPUT);
digitalWrite(LED, LOW);
#endif
  Serial.print(F("Free RAM = "));
  Serial.print(freeRam());
  Serial.println();
}

void loop() {
  // Serial
  while (Serial.available() > 0) {
    char thisChar = Serial.read();
    process_byte(thisChar, CH_SERIAL, &Serial);
  }

  // Ethernet
#ifdef ETHERNET_ENABLE
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
#endif
}

size_t process_byte(uint8_t inbyte, uint8_t channel_id, Stream* strm) {
  size_t res = 0;
/*
  if (debug) {
    Serial.print(F("New byte: "));
    if (inbyte < 16) Serial.print("0");
    Serial.println(inbyte, HEX);
  }
*/
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
#ifdef LED
  digitalWrite(LED, HIGH);
#endif
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
    case 'b': // textmode
      if (size > 1) {
        textmode = (msg[1] == 0 || msg[1] == '0') ? 0 : 1;
      } else {
        textmode = !textmode;
      }
      strm->write('b');
      strm->write('0' + (textmode ? 1 : 0));
      Serial.print(F("textmode set to "));
      Serial.println(textmode ? "ON" : "OFF");
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

        uint8_t pin = 0;
        int val;
        strm->write(mask_a);
        while (mask_a > 0) {
          if ((mask_a & 1) == 1) {
            val = analogRead(pin);
            strm->write(highByte(val));
            strm->write(lowByte(val));
            if (debug) {
              Serial.print("analogRead(");
              Serial.print(pin);
              Serial.print(")=");
              Serial.println(val);
            }
          }
          pin++;
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
    case 'T': // Test
      if (debug) Serial.println("Test:");
      if (size == 3) {
        uint16_t mask = 0b0000000111111000; // (msg[1] << 8) & msg[2];
        uint8_t pin = 0;
        while (mask > 0) {
          if ((mask & 1) == 1) {
                        Serial.print("ow[");
                        Serial.print(pin);
                        OneWire ds(pin);
                        while (ds.search(ow_addr)) {
                          strm->write(ow_addr, 8);
                          if (debug) {
                            for (uint8_t i = 0; i < 8; i++) {
                              Serial.print("] R=");
                              Serial.print(ow_addr[i], HEX);
                              Serial.print(" ");
                            }
                            Serial.println();
                          }
                        }
                        Serial.println();

          }
          pin++;
          mask >>= 1;
        }

      } else {
        strm->write("T-err");
      }
      // strm->write("T");
      break;
    case 'S': // Scan
      if (debug) Serial.println("Scan...");
      strm->write("S");
      DHT_pins = 0;
      uint16_t mask;
      mask = GPIO_MASK;
      uint8_t pin;
      pin = 0;
      while (mask > 0) {
        if ((mask & 1) == 1) {
          int chk = DHT.read22(pin);
          if (chk == DHTLIB_OK) {
            DHT_pins |= 1 << pin;
            if (debug) {
              Serial.print("DHT22 found on pin ");
              Serial.println(pin);
            }
          }
        }
        pin++;
        mask >>= 1;
      }
      strm->write(highByte(DHT_pins));
      strm->write(lowByte(DHT_pins));

      uint8_t numberOfDevices; // Number of temperature devices found
      DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address
      OW_pins = 0;
      mask = GPIO_MASK & ~ DHT_pins;
      pin = 0;
      while (mask > 0) {
        if ((mask & 1) == 1) {
          OneWire ds(pin);
          DallasTemperature sensors(&ds);
          sensors.begin();
          numberOfDevices = sensors.getDeviceCount();
          if (numberOfDevices > 0) {
            OW_pins |= 1 << pin;
            strm->write(pin);
            strm->write(numberOfDevices);
            if (debug) {
              Serial.print("ow[");
              Serial.print(pin);
              Serial.print("] ");
              Serial.print(numberOfDevices);
              Serial.println(" devices:");
            }
            if (sensors.isParasitePowerMode()) Serial.println("Parasite power is ON");
            for (int i=0; i<numberOfDevices; i++) {
              if (sensors.getAddress(tempDeviceAddress, i)) {
                strm->write(tempDeviceAddress, sizeof(tempDeviceAddress));
                if (debug) {
                  Serial.print("Found device ");
                  Serial.print(i, DEC);
                  Serial.print(" with address: ");
                  printAddress(tempDeviceAddress);
                  Serial.println();
                }
              }
            }
          }
        }
        pin++;
        mask >>= 1;
      }
      // strm->write(highByte(OW_pins));
      // strm->write(lowByte(OW_pins));
      break;
    case 'Z': // Reset (zero) some parts or all
#ifdef ETH_RESET_PIN
      if (msg[1] == 'e') {
        pinMode(ETH_RESET_PIN, OUTPUT);
        digitalWrite(ETH_RESET_PIN, LOW);
        delay(100);
        digitalWrite(ETH_RESET_PIN, HIGH);
        pinMode(ETH_RESET_PIN, INPUT);
        delay(1000);
        eth_init();
      }
#endif
      if (debug) Serial.println("Init!");
      break;
    case 'l': // LED: 0, 1, Auto
      break;
    default:
      Serial.println(F("Unknown command!"));
      break;
  }
  if (debug) {
    Serial.print(F("Free RAM = "));
    Serial.print(freeRam());
    Serial.println();
  }
#ifdef LED
  digitalWrite(LED, LOW);
#endif
}

// if (((signed long)(millis()-next))>0)

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

#ifdef ETHERNET_ENABLE
void eth_init() {
  Ethernet.begin(mac,myIP);
  server.begin();
}
#endif

