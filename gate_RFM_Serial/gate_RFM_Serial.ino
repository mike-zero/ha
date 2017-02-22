/******************************************************************************
* Gate between radio RFM69 and Serial (RS232/RS485/FTDI)
*
* RFM69CW
*
******************************************************************************/
#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>
#include "gate_config.h"

#define BUF_OUT_MAX_MESSAGE_LEN (RH_RF69_MAX_MESSAGE_LEN + 10)

RH_RF69 driver;
RHReliableDatagram manager(driver, MY_ADDRESS);

String inputString = "";
boolean stringComplete = false;

uint8_t buf_out[BUF_OUT_MAX_MESSAGE_LEN];
uint8_t buf_out_pos = 0;
uint8_t buf_in[RH_RF69_MAX_MESSAGE_LEN];

uint8_t last_RSSI = 0;
int8_t tx_power = 0;

void setup() {
  inputString.reserve(200);
  Serial.begin(115200);
  while (!Serial) ;

#ifdef RS485_TX_EN
  digitalWrite(RS485_TX_EN, LOW);
  pinMode(RS485_TX_EN, OUTPUT);
#endif

  if (!manager.init()) {
    while (1) ;
  }
  driver.setTxPower(tx_power); // default is +13
  driver.setFrequency(FREQUENCY);
  driver.setModemConfig(RH_RF69::GFSK_Rb2_4Fd4_8);
  driver.setSyncWords(sync_words, sizeof(sync_words));
  driver.setEncryptionKey(aes_key);

  manager.setTimeout(1000);
  manager.setRetries(0);
}

void loop() {
  uint8_t len = sizeof(buf_in);
  uint8_t from;
  uint8_t to;
  uint8_t id;
  uint8_t flags;
  if (manager.recvfromAck(buf_in, &len, &from, &to, &id, &flags)) {
    driver.setModeIdle(); // for temperatureRead
    int8_t temp = driver.temperatureRead();
    uint8_t last_RSSI = driver.lastRssi();
    buf_out_add_byte(temp);

    buf_out_add_byte(last_RSSI);
    buf_out_add_byte(tx_power);

    buf_out_add_byte(len);
    buf_out_add_byte(from);
    buf_out_add_byte(to);
    buf_out_add_byte(id);
    buf_out_add_byte(flags);

    buf_out_add_array(buf_in, len);
    send_to(SERVER_ADDRESS);
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      if (inputString.endsWith("K+CONN\r\n")) {
//        BT_connected = true;
      } else if (inputString.endsWith("K+LOST\r\n")) {
//        BT_connected = false;
      }
      inputString = "";
    }
  }
}

void buf_out_add_byte(uint8_t value) {
  if (buf_out_pos < BUF_OUT_MAX_MESSAGE_LEN) {
    buf_out[buf_out_pos++] = value;
  } else if (buf_out_pos == BUF_OUT_MAX_MESSAGE_LEN) {
    buf_out_pos = BUF_OUT_MAX_MESSAGE_LEN+1;  // just a buffer overflow mark
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
  if (buf_out_pos > BUF_OUT_MAX_MESSAGE_LEN) { // buffer overflow
    buf_out_pos = 0;
    return 0;
  }
#ifdef RS485_TX_EN
  digitalWrite(RS485_TX_EN, HIGH);
#endif
  Serial.write(STX);
  serial_send_byte_as_two(addr_to);
  serial_send_byte_as_two(MY_ADDRESS);
  for (uint8_t i = 0; i < buf_out_pos; i++) {
    serial_send_byte_as_two(buf_out[i]);
  }
  Serial.write(ETX);
#ifdef RS485_TX_EN
  Serial.flush();
  digitalWrite(RS485_TX_EN, LOW);
#endif
  buf_out_pos = 0;
  return 1;
}
