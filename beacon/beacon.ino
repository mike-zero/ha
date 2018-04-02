/******************************************************************************
* Just a beacon
*
* RFM69CW
*
******************************************************************************/
#include "LowPower.h"
#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include "beacon_config.h"

#ifdef YIELD
#undef YIELD
#endif

#define YIELD yield()

#define RH_FLAGS_HAS_POWER_INFO 0x01

const unsigned long main_period = 1000L * 60;    // ms
unsigned long next_action_time = 0;
unsigned long last_good_transmit_time = 0;
// bool config_changed = false;
// bool data_changed = false;

// unsigned long next_indication_time = 0;
// unsigned long indication_period = 2000L; // ms
// uint16_t indication_step_duration = 25; // ms
// uint8_t indication_step = 0;

RH_RF69 driver;
RHReliableDatagram manager(driver, MY_ADDRESS);
uint8_t last_RSSI = 0;
int8_t tx_power = -18;

const long InternalReferenceVoltage = 1100;  // Adjust this value to your board's specific internal BG voltage

uint8_t data[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data_size = 0;
// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

void setup() {
  digitalWrite(SS, HIGH);
  pinMode(SS, OUTPUT);

  analogReference(INTERNAL);
  analogRead(A0);  // force voltage reference to be turned on
  battery_voltage();    // workaround needed due to wrong value returned by first call

  if (manager.init()) {
    driver.setTxPower(tx_power); // default is +13
    driver.setFrequency(FREQUENCY);
    driver.setModemConfig(RH_RF69::GFSK_Rb2_4Fd4_8);
    driver.setSyncWords(sync_words, sizeof(sync_words));
    driver.setEncryptionKey(aes_key);
    manager.setTimeout(500);
    manager.setRetries(0);
  }

  next_action_time = millis();
}

bool send_telemetry() {
  uint8_t flags = 0;
  driver.setModeIdle(); // for temperatureRead
  int8_t radio_temp = driver.temperatureRead();
  int volts = battery_voltage();

  radio_buf_put_header();
  buf_out_add_byte(0x00); // Hello, World!

  buf_out_add_byte(flags);
  buf_out_add_byte(highByte(volts));
  buf_out_add_byte(lowByte(volts));
  buf_out_add_byte(radio_temp);

  return radio_send();
}

void radio_buf_put_header() {
  data_size = 0;
  buf_out_add_byte(tx_power);
  buf_out_add_byte(last_RSSI);
}

bool radio_send() {
  manager.setHeaderFlags(RH_FLAGS_HAS_POWER_INFO, RH_FLAGS_NONE);
  driver.setTxPower(tx_power);
  if (manager.sendtoWait(data, data_size, SERVER_ADDRESS)) {
    last_good_transmit_time = millis();
    last_RSSI = driver.lastRssi();
    if (tx_power > RFM_MIN_POWER) tx_power--;
    return true;
  } else {
    if (tx_power < RFM_MAX_POWER) {
      tx_power += 3;
      if (tx_power > RFM_MAX_POWER) tx_power = RFM_MAX_POWER;
    }
    return false;
  }
}

void yield() {
}

void loop() {
    if (next_action_time && ((signed long)(millis() - next_action_time)) >= 0) {
        ow_read_results();
        data_changed = true;
        next_action_time = 0;
    }

    if (step >= 2 && step <= 5) {
        if (config_changed) {
            if (send_config()) {
                config_changed = false;
            }
        }
        if (!config_changed && data_changed) {
            if (send_telemetry()) {
                data_changed = false;
            }
        }
    }

  LowPower.idle(SLEEP_1S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
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
