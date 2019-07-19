/*
 * A multi-purpose sensor node, controlled by unified commands via Serial
 * and/or UIPEthernet (ENC28J60 module).
 * 
 * Common abilities: A/D GPIO read/write, impulse counter,
 * 1-wire thermometers (DS18B20), DHT22
 */

#include <OneWire.h>
#include <DallasTemperature.h>
#include <dht.h>
#include "node_config.h"
#include "node.h"

#ifdef ETHERNET_ENABLE
#include <UIPEthernet.h>
#endif

// uint8_t configured = 0;
uint8_t debug = 0;

uint8_t buf_in[BUF_LEN];
uint16_t buf_in_pos = 0;
uint8_t buf_out[BUF_LEN];
uint16_t buf_out_pos = 0;

uint16_t DHT_pins = 0;
uint16_t OW_pins = 0;

DeviceAddress ow_addr;

dht DHT;

#ifdef ETHERNET_ENABLE
IPAddress myIP(IP);
EthernetServer server = EthernetServer(1000);
#endif

uint8_t serial_state = SERIAL_STATE_READY;
uint8_t addr = 0;
uint8_t addr_from = 0;

void setup() {
  memset(buf_in, 0, sizeof(buf_in));
  memset(buf_out, 0, sizeof(buf_out));
#ifdef ETHERNET_ENABLE
  eth_init();
#else
  #ifdef RS485_TX_EN
    digitalWrite(RS485_TX_EN, LOW);
    pinMode(RS485_TX_EN, OUTPUT);
  #endif

  Serial.begin(115200);
#endif

#ifdef LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
#endif

  // Say "Hi! I'm here!" to everybody
  buf_out_add_byte(0x00);
  send_to(BROADCAST_ID);
}

void reset_serial_state() {
  buf_in_pos = 0;
  serial_state = SERIAL_STATE_READY;
}

void loop() {
  // Ethernet
#ifdef ETHERNET_ENABLE
  if (EthernetClient client = server.available()) {
    buf_in_pos = 0;
    while (client.available() > 0) {
      char thisChar = client.read();
      process_byte(thisChar);
    }
    if (buf_in_pos > 0) {
      process_byte(ETX);    // !!! is this an error if there is no ETX at the end of packet? May be better to discard this?
    }
    client.stop();
    buf_in_pos = 0;
  }
#else
  // Serial
  while (Serial.available() > 0) {
    uint8_t thisChar = Serial.read();
/*
Serial.write(0xEE);
Serial.write(thisChar);
Serial.write(serial_state);
Serial.write(buf_in_pos);
Serial.write(buf_in_pos > 0 ? buf_in[buf_in_pos-1] : 0xCC);
// Serial.write('\n');
*/
    if (thisChar == STX) { // start packet resets any state and discard all received data
      buf_in_pos = 0;
      serial_state = SERIAL_STATE_ADDR_H;
    } else if (thisChar == ETX) {
      if (serial_state == SERIAL_STATE_MSG_H && buf_in_pos > 0) {
        process_command();
        if (buf_out_pos > 0 && addr_from > 0) {
          send_to(addr_from);
          addr_from = 0;
        }
      }
      reset_serial_state();
    } else if ((thisChar & 0x0F) == ((~thisChar >> 4) & 0x0F)) {
      if (serial_state == SERIAL_STATE_ADDR_H) {
        addr = thisChar & 0xF0;
        serial_state = SERIAL_STATE_ADDR_L;
      } else if (serial_state == SERIAL_STATE_ADDR_L) {
        addr |= thisChar & 0x0F;
        if (addr == NODE_ID || addr == BROADCAST_ID) {
          serial_state = SERIAL_STATE_FROM_H;
        } else {
          reset_serial_state();
        }
      } else if (serial_state == SERIAL_STATE_FROM_H) {
        addr_from = thisChar & 0xF0;
        serial_state = SERIAL_STATE_FROM_L;
      } else if (serial_state == SERIAL_STATE_FROM_L) {
        addr_from |= thisChar & 0x0F;
        serial_state = SERIAL_STATE_MSG_H;
      } else if (serial_state == SERIAL_STATE_MSG_H) {
        if (buf_in_pos < BUF_LEN) {
          buf_in[buf_in_pos] = thisChar & 0xF0;
          serial_state = SERIAL_STATE_MSG_L;
        } else { // discard too long packet
          reset_serial_state();
        }
      } else if (serial_state == SERIAL_STATE_MSG_L) {
        buf_in[buf_in_pos++] |= thisChar & 0x0F;
        serial_state = SERIAL_STATE_MSG_H;
      }
    } else { // error
      reset_serial_state();
//Serial.write(0xDD);
    }
//Serial.write('\n');
  }
#endif
}

void buf_out_add_byte(uint8_t value) {
  if (buf_out_pos < BUF_LEN) {
    buf_out[buf_out_pos++] = value;
  } else if (buf_out_pos == BUF_LEN) {
    buf_out_pos = BUF_LEN+1;  // just a buffer overflow mark
  }
}

void buf_out_add_array(uint8_t *temp_buf, size_t buf_size) {
  for (uint8_t i = 0; i < buf_size; i++) {
    buf_out_add_byte(temp_buf[i]);
  }
}

void buf_out_add_string(const char *str) {
  for (uint8_t i = 0; i < strlen(str); i++) {
    buf_out_add_byte(str[i]);
  }
}

void serial_send_byte_as_two(uint8_t val) {
  Serial.write((val & 0xF0) | ((~val >> 4) & 0x0F));
  Serial.write((val & 0x0F) | ((~val << 4) & 0xF0));
}

int16_t send_to(uint8_t addr_to) {
  if (buf_out_pos > BUF_LEN) { // buffer overflow
    buf_out_pos = 0;
    return 0;
  }
#ifdef RS485_TX_EN
  digitalWrite(RS485_TX_EN, HIGH);
  pinMode(RS485_TX_EN, OUTPUT);
#endif
  Serial.write(STX);
  serial_send_byte_as_two(addr_to);
  serial_send_byte_as_two(NODE_ID);
  for (uint8_t i = 0; i < buf_out_pos; i++) {
    serial_send_byte_as_two(buf_out[i]);
  }
  Serial.write(ETX);
  Serial.flush();
#ifdef RS485_TX_EN
  digitalWrite(RS485_TX_EN, LOW);
#endif
  buf_out_pos = 0;
  return 1;
}

size_t process_byte(uint8_t inbyte) {
  size_t res = 0;
  if (inbyte == ETX) {
    if (buf_in_pos > 0) {
      process_command();
      if (buf_out_pos > 0) {
        send_to(addr_from);
      }
    }
    buf_in_pos = 0;
  } else {
    if (buf_in_pos < BUF_LEN) {
      buf_in[buf_in_pos++] = inbyte;
    }
  }
  return res;
}

void process_command() {
  uint8_t* msg = buf_in;
  size_t size = buf_in_pos;
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
    case 'R': // Read
      if (size == 2) {
        buf_out_add_string("R");
        uint8_t mask_a = msg[1] & 0xFF;
/*
        uint8_t mask_d = msg[2] & 0xFF;
        uint8_t mask_b = msg[3] & 0xFF;

        if (mask_d) { 
          buf_out_add_byte(DDRD & mask_d);
          buf_out_add_byte(PORTD & mask_d);
          buf_out_add_byte(PIND & mask_d);
        }
        if (mask_b) { 
          buf_out_add_byte(DDRB & mask_b);
          buf_out_add_byte(PORTB & mask_b);
          buf_out_add_byte(PINB & mask_b);
        }
*/
        buf_out_add_byte(DDRB);
        buf_out_add_byte(PORTB);
        buf_out_add_byte(PINB);
        buf_out_add_byte(DDRD);
        buf_out_add_byte(PORTD);
        buf_out_add_byte(PIND);

        uint8_t pin = 0;
        int val;
        buf_out_add_byte(mask_a);
        while (mask_a > 0) {
          if ((mask_a & 1) == 1) {
            val = analogRead(pin);
            buf_out_add_byte(highByte(val));
            buf_out_add_byte(lowByte(val));
/*
            if (debug) {
              Serial.print("analogRead(");
              Serial.print(pin);
              Serial.print(")=");
              Serial.println(val);
            }
*/
          }
          pin++;
          mask_a >>= 1;
        }
      } else {
        buf_out_add_string("R-err");
      }
      break;
    case 'T': // Test
      buf_out_add_string("T");
      {
        uint16_t mask = GPIO_MASK; // (msg[1] << 8) & msg[2];
        uint16_t mask_ow = 0;
        uint16_t mask_dht = 0;
        uint8_t pin = 0;

        while (mask > 0) {
          if ((mask & 1) == 1) {
            int chk = DHT.read22(pin);
            if (chk == DHTLIB_OK) {
              mask_dht |= 1 << pin;
/*
              if (debug) {
                Serial.print("#0 DHT22 found on pin ");
                Serial.print(pin);
                Serial.print(": ");
                Serial.print(DHT.humidity, 1);
                Serial.print(",\t");
                Serial.print(DHT.temperature, 1);
                Serial.println();
              }
*/
            }
          }
          pin++;
          mask >>= 1;
        }
        unsigned long dht_conv_ready_time = millis() + 450; // minimum tested is 410 at -15*C and 370 at +24*C, see libraries/DHTlib/examples/dht_tuning/dht_tuning.ino
        buf_out_add_byte(highByte(mask_dht));
        buf_out_add_byte(lowByte(mask_dht));

        mask = GPIO_MASK & ~ mask_dht;
        pin = 0;
        uint8_t numberOfDevices = 0; // Number of temperature devices found
        while (mask > 0) {
          if ((mask & 1) == 1) {
            OneWire ds(pin);
            DallasTemperature sensors(&ds);
            sensors.begin();
            sensors.setWaitForConversion(0);
            numberOfDevices += sensors.getDeviceCount();
            if (sensors.getDeviceCount() > 0) {
              sensors.requestTemperatures();
              mask_ow |= 1 << pin;
            }
          }
          pin++;
          mask >>= 1;
        }
        unsigned long ow_conv_ready_time = millis() + millisToWaitForConversion(TEMPERATURE_PRECISION);
        //buf_out_add(highByte(mask_ow));
        //buf_out_add(lowByte(mask_ow));

        buf_out_add_byte(numberOfDevices);

// ...

        if (mask_dht > 0) {
          while (((signed long)(millis() - dht_conv_ready_time))<0) delay(5);
          pin = 0;
          mask = mask_dht;
          int16_t tmp = 0;
          while (mask > 0) {
            if ((mask & 1) == 1) {
              int chk = DHT.read22(pin);
              if (chk == DHTLIB_OK) {
                tmp = DHT.temperature * 10;
                buf_out_add_byte(highByte(tmp));
                buf_out_add_byte(lowByte(tmp));
                tmp = DHT.humidity * 10;
                buf_out_add_byte(highByte(tmp));
                buf_out_add_byte(lowByte(tmp));
              } else {
                for (uint8_t i = 0; i < 4; i++) {
                  buf_out_add_byte(0xFF); // NAN?
                }
              }
            }
            pin++;
            mask >>= 1;
          }
        }
// ...

        if (mask_ow > 0) {
          while (((signed long)(millis() - ow_conv_ready_time))<0) delay(5);

          mask = mask_ow; // GPIO_MASK; // (msg[1] << 8) & msg[2];
          pin = 0;
          while (mask > 0 && numberOfDevices > 0) {
            if ((mask & 1) == 1) {
/*
              if (debug) {
                Serial.print("ow[");
                Serial.print(pin);
                Serial.print("]");
              }
*/
              OneWire ds(pin);

              DallasTemperature sensors(&ds);
//              sensors.begin();
//              numberOfDevices = sensors.getDeviceCount();

                ds.reset_search();
                while (ds.search(ow_addr) && numberOfDevices-- > 0) {
/*
                  if (debug) {
                    for (uint8_t i = 0; i < 8; i++) {
                      Serial.print(" ");
                      if (ow_addr[i] < 0x10) {
                        Serial.print("0");
                      }
                      Serial.print(ow_addr[i], HEX);
                    }
                    Serial.print(" | ");
                  }
*/
                  if (ow_addr[0] == 0x28) {
                    int16_t temp = sensors.getTemp(ow_addr);
/*
                    if (debug) {
                      Serial.print(": ");
                      Serial.print(sensors.rawToCelsius(temp));
                    }
*/
                    if (temp != DEVICE_DISCONNECTED_RAW) {
                      buf_out_add_array(ow_addr, sizeof(ow_addr));
                      buf_out_add_byte(highByte(temp));
                      buf_out_add_byte(lowByte(temp));
                    }
                    sensors.setResolution(ow_addr, TEMPERATURE_PRECISION);  // for next time
                  }
                }
/*
                if (debug) {
                  Serial.println();
                }
*/
            }
            pin++;
            mask >>= 1;
          }
          while (numberOfDevices-- > 0) {
            for (int i=0; i<10; i++) {  // placeholder for 8 bytes address and 2 bytes of payload
              buf_out_add_byte(0xFF);
            }
          }
        }
      }
      // buf_out_add("T");
      break;
    case 'S': // Scan
//      if (debug) Serial.println("Scan...");
      buf_out_add_string("S");
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
/*
            if (debug) {
              Serial.print("DHT22 found on pin ");
              Serial.println(pin);
            }
*/
          }
        }
        pin++;
        mask >>= 1;
      }
      buf_out_add_byte(highByte(DHT_pins));
      buf_out_add_byte(lowByte(DHT_pins));

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
            buf_out_add_byte(pin);
            buf_out_add_byte(numberOfDevices);
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
                buf_out_add_array(tempDeviceAddress, sizeof(tempDeviceAddress));
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
      // buf_out_add(highByte(OW_pins));
      // buf_out_add(lowByte(OW_pins));
      break;
    case 'Z': // Reset (zero) some parts or all
#ifdef ETH_RESET_PIN
      if (msg[1] == 'e') {
        eth_init();
      }
#endif
//      if (debug) Serial.println("Init!");
      break;
    case 'l': // LED: 0, 1, Auto
      break;
    default:
      Serial.println(F("Unknown command!"));	// !!!!!
      break;
  }
#ifdef LED
  digitalWrite(LED, LOW);
#endif
}

int freeRam() {
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
  pinMode(ETH_RESET_PIN, OUTPUT);
  digitalWrite(ETH_RESET_PIN, LOW);
  delay(100);
  digitalWrite(ETH_RESET_PIN, HIGH);
  pinMode(ETH_RESET_PIN, INPUT);
  delay(1000);
  Ethernet.begin(mac,myIP);
  server.begin();
}
#endif

// returns number of milliseconds to wait till conversion is complete (based on IC datasheet)
int16_t millisToWaitForConversion(uint8_t bitResolution){

    switch (bitResolution){
    case 9:
        return 94;
    case 10:
        return 188;
    case 11:
        return 375;
    default:
        return 750;
    }
}
