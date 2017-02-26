/******************************************************************************
* Telemetry
*
* DS18B20
*
* Bluetooth Low Energy (BLE) CC41-A (fake HM-10)
* Before using configure once using this commands:
* AT+DEFAULT
* AT+NAME<yourdevicename>
* AT+NOTI1
* AT+UUID0x1809
* AT+CHAR0x2A1C
* AT+POWE0 or AT+POWE1 or AT+POWE2 (0 - less power, 2 - more power)
* at+reset
*
* RFM69CW
*
******************************************************************************/
#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "tm_config.h"

#ifdef YIELD
#undef YIELD
#endif

#define YIELD yield()

#define RH_FLAGS_HAS_POWER_INFO 0x01

const unsigned long main_period = 1000L * 15;    // ms
unsigned long next_action_time = 0;
bool onthego = false;

unsigned long next_indication_time = 0;
unsigned long indication_period = 2000L; // ms
uint16_t indication_step_duration = 25; // ms
uint8_t indication_step = 0;

RH_RF69 driver;
RHReliableDatagram manager(driver, MY_ADDRESS);
uint8_t last_RSSI = 0;
int8_t tx_power = -18;
bool have_radio = false;
bool radio_link_ok = false;

const long InternalReferenceVoltage = 1100;  // Adjust this value to your board's specific internal BG voltage

OneWire ds(PIN_ONEWIRE);
DallasTemperature sensors(&ds);
DeviceAddress ow_addr;

int32_t saved_temp = DEVICE_DISCONNECTED_RAW;
float real_temp = 0;

TinyGPSPlus gps;

//File file;
//SdFat sd;
//SdFile file;
//bool have_sd = false;

String BTinputString = "";
boolean BT_connected = false;

uint8_t data[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data_size = 0;
// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

SoftwareSerial GPSSerial(PIN_GPS_RX, PIN_GPS_TX); // RX, TX

void setup() {
  digitalWrite(SS, HIGH);
  pinMode(SS, OUTPUT);

  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  BTinputString.reserve(64);


  Serial.begin(9600);  // BLE module
  
  GPSSerial.begin(9600);
  GPSSerial.println(F("$PSRF100,1,57600,8,1,0*36"));
  GPSSerial.begin(57600);
  GPSSerial.println(F("$PSRF103,02,00,00,01*26")); // disable GSA
  GPSSerial.println(F("$PSRF103,03,00,00,01*27")); // disable GSV
  GPSSerial.println(F("$PSRF103,05,00,00,01*21")); // disable VTG
  GPSSerial.println(F("$PSRF103,00,00,01,01*25")); // GGA 1 pps
  GPSSerial.println(F("$PSRF103,04,00,01,01*21")); // RMC 1 pps

  analogReference(INTERNAL);
  analogRead(A0);  // force voltage reference to be turned on
  battery_voltage();    // workaround needed due to wrong value returned by first call

/*
  if (SD.begin(PIN_SS_SD)) {
    while (SD.exists(fileName)) {
      if (fileName[BASE_NAME_SIZE + 1] != '9') {
        fileName[BASE_NAME_SIZE + 1]++;
      } else if (fileName[BASE_NAME_SIZE] != '9') {
        fileName[BASE_NAME_SIZE + 1] = '0';
        fileName[BASE_NAME_SIZE]++;
      // } else {
      //  error("Can't create file name");
      }
    }
    file = SD.open(fileName, FILE_WRITE);
    if (file) {
      have_sd = true;
    }
  }
*/
  if (manager.init()) {
    have_radio = true;
    driver.setTxPower(tx_power); // default is +13
    driver.setFrequency(FREQUENCY);
    driver.setModemConfig(RH_RF69::GFSK_Rb2_4Fd4_8);
    driver.setSyncWords(sync_words, sizeof(sync_words));
    driver.setEncryptionKey(aes_key);
    manager.setTimeout(500);
    manager.setRetries(0);
  }

/*
//  if (sd.begin(PIN_SS_SD, SPI_HALF_SPEED)) {
  if (sd.begin(PIN_SS_SD)) {
    while (sd.exists(fileName)) {
      if (fileName[BASE_NAME_SIZE + 1] != '9') {
        fileName[BASE_NAME_SIZE + 1]++;
      } else if (fileName[BASE_NAME_SIZE] != '9') {
        fileName[BASE_NAME_SIZE + 1] = '0';
        fileName[BASE_NAME_SIZE]++;
      // } else {
      //  error("Can't create file name");
      }
    }
    if (file.open(fileName, O_CREAT | O_WRITE | O_EXCL)) {
      have_sd = true;
    }
  }
*/

  sensors.setWaitForConversion(0);
  sensors.begin();
  next_action_time = millis();
}

void yield() {
  while (GPSSerial.available()) gps.encode(GPSSerial.read());
  if (((signed long)(millis() - next_indication_time)) >= 0) {
    uint16_t time_in_period = millis() % indication_period;
    uint8_t step_number = time_in_period / indication_step_duration;
    switch (step_number) {
      case 0:
        // BR 25 B 27 BG 29 G 31 GR 33 R
        if (real_temp < 29) {
          digitalWrite(LED_BLUE, HIGH);
        } else if (real_temp < 33) {
          digitalWrite(LED_GREEN, HIGH);
        } else {
          digitalWrite(LED_RED, HIGH);
        }
        break;
      case 4:
        if (real_temp < 25 || real_temp >= 31 && real_temp < 33) {
          digitalWrite(LED_BLUE, LOW);
          digitalWrite(LED_GREEN, LOW);
          digitalWrite(LED_RED, HIGH);
        } else if (real_temp >= 27 && real_temp < 29) {
          digitalWrite(LED_BLUE, LOW);
          digitalWrite(LED_GREEN, HIGH);
        }
        break;
      case 20:
        digitalWrite(radio_link_ok ? LED_GREEN : LED_RED, HIGH);
        break;
      case 8:
      case 21:
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_BLUE, LOW);
        break;
    }
    next_indication_time = int(millis() / indication_step_duration + 1) * indication_step_duration;
  }
}

void loop() {
  if (((signed long)(millis() - next_action_time)) >= 0) {
    if (onthego) {  // measure should be done, let's read the results
      next_action_time += main_period - 750;  // hardcoded for 12 bit resolution

      uint8_t flags = (have_radio             ?  1 : 0) 
//                    | (have_sd                ?  2 : 0) 
                    | (gps.location.isValid() ?  4 : 0)
                    | (BT_connected           ?  8 : 0)
                    ;
      data_size = 0;
      int volts = battery_voltage();
      uint8_t tx_power_position = 0;
      uint8_t ds_counter_position = 0;

      int8_t radio_temp = -128;
      uint16_t gps_age = 0xFFFF;

      if (have_radio) {
        tx_power_position = data_size; // save this position for future update
        buf_out_add_byte(tx_power);
        buf_out_add_byte(flags);
        buf_out_add_byte(highByte(volts));
        buf_out_add_byte(lowByte(volts));
        driver.setModeIdle(); // for temperatureRead
        radio_temp = driver.temperatureRead();
        buf_out_add_byte(radio_temp);

        buf_out_add_byte(last_RSSI);
      
        uint32_t age = gps.location.age() / 1000;
        if (age < 0xFFFF) {
          gps_age = age;
        } else {
          gps_age = 0xFFFF;
        }
        buf_out_add_byte(highByte(gps_age));
        buf_out_add_byte(lowByte(gps_age));

        int32_t lat = gps.location.lat() * 10000000;
        buf_out_add_i32(lat);
        int32_t lng = gps.location.lng() * 10000000;
        buf_out_add_i32(lng);

        uint16_t hdopx10 = gps.hdop.isValid() ? gps.hdop.value() / 10 : 0xFF;
        buf_out_add_byte(hdopx10 <= 0xFF ? (uint8_t)hdopx10 : 0xFF);

        uint16_t gps_course = gps.course.isValid() ? gps.course.value() : 0xFFFF;
        buf_out_add_byte(highByte(gps_course));
        buf_out_add_byte(lowByte(gps_course));

        uint16_t gps_speed = gps.speed.isValid() ? gps.speed.value() : 0xFFFF;
        buf_out_add_byte(highByte(gps_speed));
        buf_out_add_byte(lowByte(gps_speed));

        ds_counter_position = data_size; // save this position for future update
        buf_out_add_byte(0); // reserve one byte in buffer for counter
      }

      uint8_t i = 0;
      ds.reset_search();
      saved_temp = DEVICE_DISCONNECTED_RAW;
      while (ds.search(ow_addr)) {
        if (ow_addr[0] == 0x28) {
          int32_t temp = sensors.getTemp(ow_addr);
          if (temp != DEVICE_DISCONNECTED_RAW) {
            i++;
            if (have_radio) {
              buf_out_add_byte(highByte(temp));
              buf_out_add_byte(lowByte(temp));
            }
            if (i == 1) { // we can send to BT only one temperature, so let it be the first one
              saved_temp = temp;
              real_temp = temp * 0.0078125;
              BT_send_temp(temp);
            }
          }
        }
      }
      if (i && have_radio) {
        data[ds_counter_position] = i; // the real count of thermometers
      }
  
      if (have_radio) {
        radio_link_ok = false;
        for (uint8_t i = 0; i < 4; i++) {
          manager.setHeaderFlags(RH_FLAGS_HAS_POWER_INFO, RH_FLAGS_NONE);
          if (manager.sendtoWait(data, data_size, SERVER_ADDRESS)) {
            radio_link_ok = true;
            last_RSSI = driver.lastRssi();
            if (tx_power > RFM_MIN_POWER) tx_power--;
            data[tx_power_position] = tx_power;
            driver.setTxPower(tx_power);
            break;
          } else {
            if (tx_power < RFM_MAX_POWER) {
              tx_power += 3;
              if (tx_power > RFM_MAX_POWER) tx_power = RFM_MAX_POWER;
              data[tx_power_position] = tx_power;
              driver.setTxPower(tx_power);
            }
          }
        }
      }
    } else {    // just start the measure
      sensors.requestTemperatures();
      next_action_time += 750;  // hardcoded for 12 bit resolution
    }
    onthego = not onthego;
  }
  YIELD;
}

uint8_t BT_send_temp(int32_t temp) {
  if (BT_connected) {
    temp *= 0.78125;        // raw_temp / 128 * 100
    // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.temperature_measurement.xml
    uint8_t buf[6] = {0x04, (uint8_t)(temp & 0xFF), (uint8_t)((temp >> 8) & 0xFF), (uint8_t)((temp >> 16) & 0xFF), 0xFE, 0x02};
    Serial.write(buf, sizeof(buf));
    return 1;
  } else {
    return 0;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    BTinputString += inChar;
    if (inChar == '\n') {
      if (BTinputString.endsWith(F("K+CONN\r\n"))) {
        BT_connected = true;
        if (saved_temp != DEVICE_DISCONNECTED_RAW) {
          BT_send_temp(saved_temp);
        }
      } else if (BTinputString.endsWith(F("K+LOST\r\n"))) {
        BT_connected = false;
      }
      BTinputString = "";
    }
  }
}

int battery_voltage() {
   ADMUX = bit (REFS0) | bit (MUX3) | bit (MUX2) | bit (MUX1);
   ADCSRA |= bit( ADSC );  // start conversion
   while (ADCSRA & bit (ADSC)) {}  // wait for conversion to complete
   int results = (((InternalReferenceVoltage * 1024) / ADC) + 5) / 10; 
   return results;
}

void buf_out_add_byte(uint8_t value) {
  if (data_size < RH_RF69_MAX_MESSAGE_LEN) {
    data[data_size++] = value;
  } else if (data_size == RH_RF69_MAX_MESSAGE_LEN) {
    data_size = RH_RF69_MAX_MESSAGE_LEN+1;  // just a buffer overflow mark
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

void buf_out_add_i32(int32_t value) {
  for (int8_t i = 3; i >= 0; i--) {
    buf_out_add_byte((uint8_t)((value >> (i << 3)) & 0xFF));
  }
}
